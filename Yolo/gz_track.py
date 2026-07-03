"""
Gazebo (ArduPilot SITL) - YOLO + SORT (çoklu nesne takibi) + ResNet ReID (hedef kilitleme)
+ DroneKit/MAVLink ile GÖRÜNTÜ TABANLI TAKİP (visual servoing)

Neden SORT?
-----------
Tek başına elle yazılmış bir Kalman filtresi, YOLO tespiti kaçırdığında (oklüzyon, motion
blur, conf eşiği altına düşme vb.) kör tahmin yapmaya devam eder ve gerçek tespitle tekrar
karşılaştığında IOU eşleşmesi zayıfsa savrulur/kayar. SORT bunu üç şeyle çözer:
  1) Her track için kendi içinde bir Kalman filtresi tutar (pozisyon+hız modeli),
  2) Yeni tespitleri Hungarian algoritmasıyla mevcut track'lere en iyi IOU eşleşmesine göre atar,
  3) Eşleşme bulunamayan track'leri belirli bir süre (max_age) daha tahmin ederek yaşatır,
     bu süre sonunda hâlâ eşleşme yoksa track'i düşürür.
Bu, bizim senaryomuzda "hedefi bulduktan sonra bbox'ın sağlam kalması" için çok daha uygun.

ÖN KOŞULLAR
-----------
- Drone terminalden ARM edilip TAKEOFF verilmiş ve havada olmalı.
- pip install dronekit pymavlink filterpy (SORT genelde filterpy'ye bağımlı)
- ./sort dizininde SORT reposu klonlanmış olmalı (sort.py içeren).
"""

import time
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# gz/ROS binding'leri icin sistem Python path'i lazim, ama NumPy/Torch/TensorFlow
# paketleri venv'den gelmeli. Bu ayar, herhangi bir ucuncu parti C eklentisi
# yuklenmeden once yapilmali; aksi halde eski sistem NumPy'si bellekte kalabilir.
_VENV_SITE = "/opt/venv/lib/python3.10/site-packages"
if os.path.isdir(_VENV_SITE):
    if _VENV_SITE in sys.path:
        sys.path.remove(_VENV_SITE)
    sys.path.insert(1 if sys.path else 0, _VENV_SITE)

import cv2
import numpy as np

# --- dronekit'in eski collections API beklentisi için uyumluluk yaması (Python 3.10+) ---
import collections
import collections.abc
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

# --- protobuf: gz.msgs pb2 dosyaları eski protoc ile üretildi, TF ise yeni protobuf istiyor ---
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

from ultralytics import YOLO
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from sklearn.metrics.pairwise import cosine_similarity

# --- SORT ---
SORT_DIR = os.path.join(SCRIPT_DIR, "sort")
if SORT_DIR not in sys.path:
    sys.path.append(SORT_DIR)
from sort import Sort

# --- GAZEBO ---
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GZImage
from gz.msgs10.double_pb2 import Double as GZDouble

# --- OTOPİLOT ---
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --- GPU BELLEK ---
import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    for gpu in gpus:
        tf.config.experimental.set_memory_growth(gpu, True)


# -------------------------------------------------
# RESNET REID
# -------------------------------------------------
class AracTakipSistemi:
    def __init__(self, hedef_gorsel_yolu):
        print("ResNet hazırlanıyor...")
        base = ResNet50(weights='imagenet', include_top=False, pooling='avg')
        self.extractor = Model(inputs=base.input, outputs=base.output)
        self.hedef_vektor = self.vektor_cikar(hedef_gorsel_yolu)

    def vektor_cikar(self, img):
        if isinstance(img, str):
            img = cv2.imread(img)
            if img is None:
                raise FileNotFoundError(f"Görsel okunamadı: {img}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (224, 224))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        return self.extractor.predict(x, verbose=0)

    def dogrula(self, frame, bboxlar):
        best_sim = -1
        best_bbox = None
        for bbox in bboxlar:
            x1, y1, x2, y2 = map(int, bbox)
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            try:
                vec = self.vektor_cikar(crop)
                sim = cosine_similarity(self.hedef_vektor, vec)[0][0]
                if sim > best_sim:
                    best_sim = sim
                    best_bbox = bbox
            except Exception:
                continue
        return best_bbox, best_sim


# -------------------------------------------------
# GAZEBO CAMERA
# -------------------------------------------------
class CameraSubscriber:
    def __init__(self, topic):
        self.latest_frame = None
        self.node = Node()
        self.node.subscribe(GZImage, topic, self.cb)

    def cb(self, msg):
        try:
            width, height = msg.width, msg.height
            img = np.frombuffer(msg.data, dtype=np.uint8).copy()  # yazılabilir kopya
            frame = img.reshape((height, width, 3))
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.latest_frame = frame
        except Exception:
            pass


# -------------------------------------------------
# MOUSE / BUTON
# -------------------------------------------------
def mouse_callback(event, x, y, flags, param):
    durum = param
    if event == cv2.EVENT_LBUTTONDOWN:
        if 50 < x < 250 and 100 < y < 160:
            durum['baslatildi'] = True
            print("\n>> TAKİP BAŞLATILDI!\n")


# ============================================================
#                DRONE / MAVLINK KONTROLCÜSÜ
# ============================================================
class DroneController:
    def __init__(self, connection_string, baud=None, send_interval=0.2):
        print(f"[DRONE] Otopilota bağlanılıyor: {connection_string}")
        connect_kwargs = dict(
            wait_ready=['mode', 'armed', 'system_status', 'gps_0'],
            timeout=60,
        )
        if baud:
            connect_kwargs['baud'] = baud
        self.vehicle = connect(connection_string, **connect_kwargs)
        print(f"[DRONE] Bağlantı OK. Mod: {self.vehicle.mode.name}, armed: {self.vehicle.armed}")
        self.send_interval = send_interval
        self._last_send = 0.0
        self._last_yaw_send = 0.0
        self.yaw_send_interval = 0.5

    def guided_moda_gec(self):
        if self.vehicle.mode.name != "GUIDED":
            print("[DRONE] GUIDED moduna geçiliyor...")
            self.vehicle.mode = VehicleMode("GUIDED")
            t0 = time.time()
            while self.vehicle.mode.name != "GUIDED" and time.time() - t0 < 3:
                time.sleep(0.1)

    def mevcut_irtifa(self):
        try:
            return self.vehicle.location.global_relative_frame.alt
        except Exception:
            return None

    def yaw_ile_don(self, aci_derece, hiz_derece_sn=30, yon=1, relative=True):
        """
        aci_derece: relative=True ise dönülecek açı miktarı (+ sağa/saat yönü, - sola)
                    relative=False ise mutlak hedef heading
        """
        now = time.time()
        if now - self._last_yaw_send < self.yaw_send_interval:
            return
        self._last_yaw_send = now

        self.vehicle.message_factory
        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            abs(aci_derece),
            hiz_derece_sn,
            1 if aci_derece >= 0 else -1,
            1 if relative else 0,
            0, 0, 0
        )


    def send_body_velocity(self, vx, vy, vz=0.0, force=False):
        now = time.time()
        if not force and (now - self._last_send) < self.send_interval:
            return
        self._last_send = now

        X_IGNORE=1; Y_IGNORE=2; Z_IGNORE=4
        AX_IGNORE=64; AY_IGNORE=128; AZ_IGNORE=256
        YAW_IGNORE=1024; YAW_RATE_IGNORE=2048

        type_mask = (X_IGNORE|Y_IGNORE|Z_IGNORE|
                    AX_IGNORE|AY_IGNORE|AZ_IGNORE|
                    YAW_IGNORE|YAW_RATE_IGNORE)

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            float(vx), float(vy), float(vz),
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def dur(self):
        self.send_body_velocity(0, 0, 0, force=True)

    def kapat(self):
        try:
            self.dur()
        except Exception:
            pass
        self.vehicle.close()


class GimbalController:
    def __init__(self, pitch_baslangic=-0.785, yaw_baslangic=0.0,
                 pitch_min=-1.57, pitch_max=-0.3, yaw_max=0.6,
                 kp_pitch=0.25, kp_yaw=0.35, send_interval=0.05):
        self.node = Node()
        self._pub_pitch = self.node.advertise('/gimbal/cmd_pitch', GZDouble)
        self._pub_yaw = self.node.advertise('/gimbal/cmd_yaw', GZDouble)
        self.pitch_baslangic = pitch_baslangic
        self.yaw_baslangic = yaw_baslangic
        self.pitch = pitch_baslangic
        self.yaw = yaw_baslangic
        self.pitch_min = pitch_min
        self.pitch_max = pitch_max
        self.yaw_max = yaw_max
        self.kp_pitch = kp_pitch
        self.kp_yaw = kp_yaw
        self.send_interval = send_interval
        self._last_send = 0.0
        self._send(force=True)

    def _send(self, force=False):
        now = time.time()
        if not force and (now - self._last_send) < self.send_interval:
            return
        self._last_send = now

        pitch_msg = GZDouble()
        yaw_msg = GZDouble()
        pitch_msg.data = float(self.pitch)
        yaw_msg.data = float(self.yaw)
        self._pub_pitch.publish(pitch_msg)
        self._pub_yaw.publish(yaw_msg)

    def update(self, err_x_norm, err_y_norm, dt):
        self.yaw = float(np.clip(
            self.yaw + self.kp_yaw * err_x_norm * dt,
            -self.yaw_max,
            self.yaw_max,
        ))
        self.pitch = float(np.clip(
            self.pitch - self.kp_pitch * err_y_norm * dt,
            self.pitch_min,
            self.pitch_max,
        ))
        self._send()

    def reset(self):
        self.pitch = self.pitch_baslangic
        self.yaw = self.yaw_baslangic
        self._send(force=True)


class TakipController:
    def __init__(self, frame_w, frame_h, kp_yatay=0.58, ki_yatay=0.12,
                 max_yatay_vel=0.85, deadzone_px=12, takip_irtifasi=None,
                 kp_alt=0.5, max_dikey_vel=1.0, err_alpha=0.36,
                 err_zero_alpha=0.65, vel_alpha=0.55, max_accel=1.10,
                 max_decel=1.80, min_vel=0.010, min_duzeltme_hizi=0.11,
                 komut_deadband_norm=0.006, integral_limit=0.65):
        self.cx = frame_w / 2.0
        self.cy = frame_h / 2.0
        self.kp_yatay = kp_yatay
        self.ki_yatay = ki_yatay
        self.max_yatay_vel = max_yatay_vel
        self.deadzone_px = deadzone_px
        self.takip_irtifasi = takip_irtifasi
        self.kp_alt = kp_alt
        self.max_dikey_vel = max_dikey_vel
        self.err_alpha = err_alpha
        self.err_zero_alpha = err_zero_alpha
        self.vel_alpha = vel_alpha
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.min_vel = min_vel
        self.min_duzeltme_hizi = min_duzeltme_hizi
        self.komut_deadband_norm = komut_deadband_norm
        self.integral_limit = integral_limit
        self.err_x_filt = 0.0
        self.err_y_filt = 0.0
        self.err_x_i = 0.0
        self.err_y_i = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def hesapla_merkezleme(self, bbox, mevcut_irtifa=None, dt=0.1):
        x1, y1, x2, y2 = bbox
        bx, by = (x1 + x2) / 2.0, (y1 + y2) / 2.0

        err_x = bx - self.cx
        err_y = by - self.cy

        err_x_norm = self._deadband_norm(err_x, self.cx)
        err_y_norm = self._deadband_norm(err_y, self.cy)

        dt = max(0.01, min(float(dt), 0.3))
        alpha_x = self.err_zero_alpha if err_x_norm == 0.0 else self.err_alpha
        alpha_y = self.err_zero_alpha if err_y_norm == 0.0 else self.err_alpha
        self.err_x_filt = self._ema(self.err_x_filt, err_x_norm, alpha_x)
        self.err_y_filt = self._ema(self.err_y_filt, err_y_norm, alpha_y)
        self.err_x_i = self._integral_guncelle(self.err_x_i, err_x_norm, dt)
        self.err_y_i = self._integral_guncelle(self.err_y_i, err_y_norm, dt)

        kontrol_x = self.kp_yatay * self.err_x_filt + self.ki_yatay * self.err_x_i
        kontrol_y = self.kp_yatay * self.err_y_filt + self.ki_yatay * self.err_y_i
        vx_hedef = self._komut_hizi(-kontrol_y)
        vy_hedef = self._komut_hizi(kontrol_x)
        vz_hedef = 0.0
        if self.takip_irtifasi is not None and mevcut_irtifa is not None:
            alt_hata = self.takip_irtifasi - mevcut_irtifa
            vz_hedef = self._clamp(-alt_hata * self.kp_alt, self.max_dikey_vel)

        self.vx = self._filtreli_hiz(self.vx, vx_hedef, dt)
        self.vy = self._filtreli_hiz(self.vy, vy_hedef, dt)
        self.vz = self._filtreli_hiz(self.vz, vz_hedef, dt)

        return self.vx, self.vy, self.vz, self.err_x_filt, self.err_y_filt

    def reset(self):
        self.err_x_filt = 0.0
        self.err_y_filt = 0.0
        self.err_x_i = 0.0
        self.err_y_i = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def _deadband_norm(self, err_px, half_axis):
        mag = abs(err_px)
        if mag <= self.deadzone_px:
            return 0.0
        return np.sign(err_px) * min(1.0, mag / half_axis)

    def _integral_guncelle(self, onceki, err_norm, dt):
        if err_norm == 0.0:
            decay = max(0.0, 1.0 - 3.0 * dt)
            return onceki * decay
        yeni = onceki + err_norm * dt
        return self._clamp(yeni, self.integral_limit)

    def _komut_hizi(self, kontrol):
        if abs(kontrol) < self.komut_deadband_norm:
            return 0.0
        hiz = self._clamp(kontrol, self.max_yatay_vel)
        if abs(hiz) < self.min_duzeltme_hizi:
            return float(np.sign(hiz) * self.min_duzeltme_hizi)
        return hiz

    def _filtreli_hiz(self, onceki, hedef, dt):
        hedef = self._ema(onceki, hedef, self.vel_alpha)
        limit = self.max_decel if abs(hedef) < abs(onceki) else self.max_accel
        hiz = self._slew(onceki, hedef, limit * dt)
        if abs(hedef) < self.min_vel and abs(hiz) < self.min_vel:
            return 0.0
        return hiz

    @staticmethod
    def _ema(eski, yeni, alpha):
        return eski + alpha * (yeni - eski)

    @staticmethod
    def _slew(eski, yeni, max_delta):
        delta = yeni - eski
        if abs(delta) <= max_delta:
            return yeni
        return eski + np.sign(delta) * max_delta

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, v))

# ============================================================
#                        ANA PROGRAM
# ============================================================
def main():
    TOPIC_NAME = "/world/sonoma/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
    HEDEF_GORSEL = os.path.join(SCRIPT_DIR, "track_nadir.png")
    MODEL_PATH = os.path.join(SCRIPT_DIR, "best.pt")

    BAGLANTI_STR = "udp:127.0.0.1:14550"
    TAKIP_IRTIFASI = None
    DRONE_SEND_INTERVAL = 0.05
    MAX_YATAY_HIZ = 0.85
    MAX_DIKEY_HIZ = 1.0
    KP_MERKEZLEME = 0.58
    KI_MERKEZLEME = 0.12
    MERKEZ_DEADZONE_PX = 12

    HATA_FILTRE_ALPHA = 0.36
    HIZ_FILTRE_ALPHA = 0.55
    MAX_IVMELENME = 1.10
    MAX_YAVASLAMA = 1.80
    MIN_DUZELTME_HIZI = 0.11
    KOMUT_DEADBAND_NORM = 0.006

    GIMBAL_AKTIF = True
    GIMBAL_TAKIP_AKTIF = False
    GIMBAL_PITCH_BASLANGIC = -0.785
    GIMBAL_YAW_BASLANGIC = 0.0

    KAYIP_TOLERANSI = 15    # hedef track kaç frame boyunca görünmezse kilit bırakılsın

    camera = CameraSubscriber(TOPIC_NAME)
    takip = AracTakipSistemi(HEDEF_GORSEL)
    model = YOLO(MODEL_PATH)

    drone = DroneController(BAGLANTI_STR, send_interval=DRONE_SEND_INTERVAL)
    gimbal = GimbalController(
        pitch_baslangic=GIMBAL_PITCH_BASLANGIC,
        yaw_baslangic=GIMBAL_YAW_BASLANGIC,
    ) if GIMBAL_AKTIF else None

    tracker = Sort(max_age=20, min_hits=1, iou_threshold=0.3)
    checked_tracks = set()

    hedef_kilitlendi = False
    hedef_track_id = None
    kayip_sayaci = 0

    frame_counter = 0
    SKIP_FRAMES = 1
    CONF_THRESHOLD = 0.85
    RESNET_SIMILARITY = 0.3

    sistem_durumu = {'baslatildi': False}
    takip_ctrl = None

    prev_time = time.time()
    last_debug_print = 0.0

    pencere_adi = "Gazebo Takip (SORT + Otopilot)"
    cv2.namedWindow(pencere_adi, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(pencere_adi, mouse_callback, param=sistem_durumu)

    print("Sistem hazır...")

    try:
        while True:
            time.sleep(0.01)
            frame = camera.latest_frame
            if frame is None:
                continue
            frame = frame.copy()

            if takip_ctrl is None:
                h, w = frame.shape[:2]
                takip_ctrl = TakipController(
                    frame_w=w, frame_h=h,
                    kp_yatay=KP_MERKEZLEME,
                    ki_yatay=KI_MERKEZLEME,
                    max_yatay_vel=MAX_YATAY_HIZ,
                    deadzone_px=MERKEZ_DEADZONE_PX,
                    takip_irtifasi=TAKIP_IRTIFASI,
                    max_dikey_vel=MAX_DIKEY_HIZ,
                    err_alpha=HATA_FILTRE_ALPHA,
                    vel_alpha=HIZ_FILTRE_ALPHA,
                    max_accel=MAX_IVMELENME,
                    max_decel=MAX_YAVASLAMA,
                    min_duzeltme_hizi=MIN_DUZELTME_HIZI,
                    komut_deadband_norm=KOMUT_DEADBAND_NORM,
                )

            frame_counter += 1
            detection_step = (frame_counter % SKIP_FRAMES == 0)

            detections = []
            if detection_step:
                #results = model(frame, conf=CONF_THRESHOLD, device=0, half=True, verbose=False)
                results = model(frame, conf=CONF_THRESHOLD, verbose=False)
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        detections.append([x1, y1, x2, y2, conf])

            detections = np.array(detections) if len(detections) else np.empty((0, 5))
            #print(f"[DEBUG] Bu frame'de ham YOLO tespiti: {len(detections)}")   # <-- ekleyin
            tracks = tracker.update(detections.astype(np.float32))

            hedef_bu_frame_var = False

            for track in tracks:
                x1, y1, x2, y2, track_id = track
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                track_id = int(track_id)

                if hedef_kilitlendi and track_id == hedef_track_id:
                    continue

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"ID {track_id}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                # DİKKAT: artık "not hedef_kilitlendi" şartı yok - kilit kaybolduysa
                # yeni track'ler her zaman denenebilsin diye checked_tracks'i de
                # kilit sıfırlandığında temizleyeceğiz (aşağıda).
                if track_id not in checked_tracks:
                    best_bbox, score = takip.dogrula(frame, [[x1, y1, x2, y2]])
                    checked_tracks.add(track_id)
                    print(f"[DEBUG] ID={track_id} ResNet skoru: {score:.3f} (eşik: {RESNET_SIMILARITY})")  # <-- ekleyin
                    if best_bbox is not None and score > RESNET_SIMILARITY and not hedef_kilitlendi:
                        print(f"HEDEF BULUNDU! ID={track_id} score={score:.2f}")
                        hedef_kilitlendi = True
                        hedef_track_id = track_id
                        kayip_sayaci = 0
                        if takip_ctrl is not None:
                            takip_ctrl.reset()

            # kilitli hedefi iç Kalman durumundan çiz
            if hedef_kilitlendi:
                for trk in tracker.trackers:
                    if trk.id + 1 == hedef_track_id:
                        state = trk.get_state()[0]
                        x1, y1, x2, y2 = map(int, state)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                        cv2.putText(frame, "TAKIP EDIYOR", (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        hedef_bu_frame_var = True

                        h, w = frame.shape[:2]
                        merkez = (w // 2, h // 2)
                        hedef_merkez = ((x1 + x2) // 2, (y1 + y2) // 2)
                        cv2.arrowedLine(frame, merkez, hedef_merkez, (0, 0, 255), 2, tipLength=0.2)
                        cv2.circle(frame, merkez, 5, (255, 0, 255), -1)

                        if sistem_durumu['baslatildi']:
                            drone.guided_moda_gec()
                            mevcut_irtifa = drone.mevcut_irtifa()
                            control_dt = max(time.time() - prev_time, 1e-3)
                            vx, vy, vz, err_x, err_y = takip_ctrl.hesapla_merkezleme(
                                [x1, y1, x2, y2],
                                mevcut_irtifa=mevcut_irtifa,
                                dt=control_dt,
                            )

                            if gimbal is not None and GIMBAL_TAKIP_AKTIF:
                                gimbal.update(err_x, err_y, control_dt)
                            drone.send_body_velocity(vx, vy, vz)

                            now_dbg = time.time()
                            if now_dbg - last_debug_print > 0.5:
                                last_debug_print = now_dbg
                                print(f"[DEBUG] vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} ex={err_x:.2f} ey={err_y:.2f}")
                        break

                # --- EKSİK OLAN KISIM: kilit kaybını takip et ve sıfırla ---
                if not hedef_bu_frame_var:
                    kayip_sayaci += 1
                    if kayip_sayaci > KAYIP_TOLERANSI:
                        print(">> Hedef gerçekten kayboldu (SORT track öldü). Kilit bırakılıyor, yeniden aranıyor...")
                        hedef_kilitlendi = False
                        hedef_track_id = None
                        kayip_sayaci = 0
                        checked_tracks.clear()   # <-- ÖNEMLİ: eski ID'ler temizlenmeli ki yeni ID kontrol edilebilsin
                        if sistem_durumu['baslatildi']:
                            drone.dur()
                        if takip_ctrl is not None:
                            takip_ctrl.reset()
                        if gimbal is not None:
                            gimbal.reset()
                else:
                    kayip_sayaci = 0

            new_time = time.time()
            fps = 1 / (new_time - prev_time)
            prev_time = new_time
            cv2.putText(frame, f"FPS:{int(fps)}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Mod: {drone.vehicle.mode.name}", (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

            if not sistem_durumu['baslatildi']:
                overlay = frame.copy()
                cv2.rectangle(overlay, (50, 100), (250, 160), (0, 200, 0), -1)
                cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
                cv2.putText(frame, "BASLAT", (85, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            cv2.imshow(pencere_adi, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                sistem_durumu['baslatildi'] = False
                drone.dur()
                if takip_ctrl is not None:
                    takip_ctrl.reset()
                if gimbal is not None:
                    gimbal.reset()
                print(">> ACİL DUR")

    except KeyboardInterrupt:
        print("\nKapatılıyor...")
    finally:
        try:
            drone.kapat()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
