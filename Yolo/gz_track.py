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

import cv2
import numpy as np
import time
import os
import sys

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
sys.path.append("./sort")
from sort import Sort

# --- GAZEBO ---
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GZImage

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

    def send_body_velocity(self, vx, vy, vz, yaw_rate=0.0, force=False):
        now = time.time()
        if not force and (now - self._last_send) < self.send_interval:
            return
        self._last_send = now

        X_IGNORE        = 1     # bit0
        Y_IGNORE        = 2     # bit1
        Z_IGNORE        = 4     # bit2
        VX_IGNORE       = 8     # bit3
        VY_IGNORE       = 16    # bit4
        VZ_IGNORE       = 32    # bit5
        AX_IGNORE       = 64    # bit6
        AY_IGNORE       = 128   # bit7
        AZ_IGNORE       = 256   # bit8
        # bit9 (FORCE_SET) KESİNLİKLE dahil edilmeyecek - ArduCopter'da desteklenmiyor
        YAW_IGNORE      = 1024  # bit10
        YAW_RATE_IGNORE = 2048  # bit11

        # pozisyon yoksay, VY yoksay (yanal kaymayı istemiyoruz), ivme yoksay, yaw yoksay
        # VX, VZ ve YAW_RATE aktif kullanılıyor (maskeye dahil EDİLMİYOR)
        type_mask = (X_IGNORE | Y_IGNORE | Z_IGNORE |
                    VY_IGNORE |
                    AX_IGNORE | AY_IGNORE | AZ_IGNORE |
                    YAW_IGNORE)

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,
            vx, 0.0, vz,
            0, 0, 0,
            0, yaw_rate)
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

import math

class TakipController:
    def __init__(self, frame_w, frame_h, kp_forward=0.004, kp_yaw=0.0025,
                 max_forward_vel=1.0, max_yaw_rate=0.5, deadzone_px=20,
                 takip_irtifasi=None, kp_alt=0.5, max_dikey_vel=1.0,
                 kamera_yaw_ofseti_derece=-90.0):
        self.cx = frame_w / 2.0
        self.cy = frame_h / 2.0
        self.kp_forward = kp_forward
        self.kp_yaw = kp_yaw
        self.max_forward_vel = max_forward_vel
        self.max_yaw_rate = max_yaw_rate
        self.deadzone_px = deadzone_px
        self.takip_irtifasi = takip_irtifasi
        self.kp_alt = kp_alt
        self.max_dikey_vel = max_dikey_vel
        self.kamera_yaw_ofseti = math.radians(kamera_yaw_ofseti_derece)

    def _ofset_uygula(self, a, b):
        c, s = math.cos(self.kamera_yaw_ofseti), math.sin(self.kamera_yaw_ofseti)
        return a * c - b * s, a * s + b * c

    def hesapla_yonelerek(self, bbox, mevcut_irtifa=None):
        x1, y1, x2, y2 = bbox
        bx, by = (x1 + x2) / 2.0, (y1 + y2) / 2.0

        err_x = bx - self.cx   # yatay sapma -> dönüş (yaw)
        err_y = by - self.cy   # dikey sapma -> ileri hız

        if abs(err_x) < self.deadzone_px:
            err_x = 0
        if abs(err_y) < self.deadzone_px:
            err_y = 0

        # daha önce kalibre ettiğiniz kamera montaj ofsetini burada da uyguluyoruz
        err_ileri, err_yanal = self._ofset_uygula(err_y, err_x)

        yaw_rate = self._clamp(err_yanal * self.kp_yaw, self.max_yaw_rate)

        # keskin dönüş sırasında ileri hızı azalt (overshoot'u engellemek için)
        donus_orani = min(1.0, abs(yaw_rate) / self.max_yaw_rate) if self.max_yaw_rate else 0
        forward_carpan = 1.0 - 0.6 * donus_orani   # keskin dönüşte hız ~%40'a düşer

        vx = self._clamp(err_ileri * self.kp_forward, self.max_forward_vel) * forward_carpan
        vx = max(vx, 0.0)   # geri gitmesin, sadece dur veya ileri gitsin

        vz = 0.0
        if self.takip_irtifasi is not None and mevcut_irtifa is not None:
            alt_hata = self.takip_irtifasi - mevcut_irtifa
            vz = self._clamp(-alt_hata * self.kp_alt, self.max_dikey_vel)

        return vx, vz, yaw_rate

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, v))

# ============================================================
#                        ANA PROGRAM
# ============================================================
def main():
    TOPIC_NAME = "/world/sonoma/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
    HEDEF_GORSEL = "./track_nadir.png"
    MODEL_PATH = "./best.pt"

    BAGLANTI_STR = "udp:127.0.0.1:14550"
    TAKIP_IRTIFASI = None
    MAX_YATAY_HIZ = 1.0
    MAX_DIKEY_HIZ = 1.0

    KAYIP_TOLERANSI = 15    # hedef track kaç frame boyunca görünmezse kilit bırakılsın

    camera = CameraSubscriber(TOPIC_NAME)
    takip = AracTakipSistemi(HEDEF_GORSEL)
    model = YOLO(MODEL_PATH)

    drone = DroneController(BAGLANTI_STR)

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
                    max_forward_vel=1.0,
                    max_yaw_rate=0.4,          # rad/s, çok agresif dönmesin diye düşük başlayın
                    takip_irtifasi=TAKIP_IRTIFASI,
                    max_dikey_vel=MAX_DIKEY_HIZ,
                    kamera_yaw_ofseti_derece=-90.0,
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
                            vx, vz, yaw_rate = takip_ctrl.hesapla_yonelerek([x1, y1, x2, y2], mevcut_irtifa=mevcut_irtifa)
                            print(f"[DEBUG] gönderiliyor -> vx={vx:.3f} vz={vz:.3f} yaw_rate={yaw_rate:.3f} "
                                  f"| mod={drone.vehicle.mode.name} armed={drone.vehicle.armed}")
                            drone.send_body_velocity(vx, 0, vz, yaw_rate=yaw_rate)
                            cv2.putText(frame, f"vx:{vx:+.2f} yaw_rate:{yaw_rate:+.2f} vz:{vz:+.2f}",
                                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
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
