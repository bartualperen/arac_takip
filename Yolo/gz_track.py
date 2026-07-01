 
"""
Gazebo (ArduPilot SITL) - YOLO + Kalman + ResNet tespit/takip sistemi
+ DroneKit/MAVLink ile otopilot üzerinden GÖRÜNTÜ TABANLI TAKİP (visual servoing)

ÖN KOŞULLAR
-----------
1) Drone terminalden ARM edilip TAKEOFF verilmiş ve havada olmalı (örn. mavproxy/QGC/CLI ile).
2) SITL'in MAVLink çıkışına DroneKit ile bağlanabiliyor olmanız gerekir
   (docker ortamınızda genelde: udp:127.0.0.1:14550 veya 14551 - kendi setup'ınıza göre BAGLANTI_STR'i güncelleyin).
3) pip install dronekit pymavlink
   NOT: dronekit-python eski bir proje, bazı pymavlink/python sürümleriyle uyumsuzluk çıkabilir.
   Sorun yaşarsanız 'dronekit-python' yerine topluluk forku (örn. dronekit2) kullanmanız gerekebilir.

ÇALIŞMA MANTIĞI
----------------
- Kod, hedefi tespit edip Kalman ile kilitlediği her frame'de, hedefin bbox merkezinin
  görüntü merkezine göre piksel sapmasını (err_x, err_y) hesaplar.
- Bu sapma, oransal (P) bir kontrolcü ile body-frame hız komutuna (vx: ileri/geri, vy: sağ/sol,
  vz: yukarı/aşağı) çevrilir.
- Drone GUIDED moddaysa bu hız komutu MAVLink SET_POSITION_TARGET_LOCAL_NED mesajıyla
  belirli aralıklarla (varsayılan 0.2 sn) gönderilir.
- Hedef kaybedilirse veya "BASLAT" butonuna basılmadıysa drone'a sıfır hız (dur/hover) komutu gönderilir.

ÖNEMLİ - EKSEN EŞLEŞMESİ
--------------------------
Kamera nadir (tam aşağı bakan) ve gövdeyle hizalı varsayılmıştır:
  - Görüntüde hedef SAĞDA (err_x > 0)  -> drone SAĞA kaysın   (body Y ekseni, +)
  - Görüntüde hedef AŞAĞIDA (err_y > 0) -> drone İLERİ gitsin (body X ekseni, +)
Bu eşleşme kameranızın gerçek montaj yönüne (yaw ofseti, ayna simetrisi vb.) göre TERS ya da
90 derece kaymış olabilir. İlk testlerde drone'u düşük hızla (max_vel küçük) çalıştırıp
yön doğruysa devam edin; ters ise TakipController içindeki işaretleri (+/-) veya err_x/err_y
eşlemesini değiştirin.
"""
import logging
logging.basicConfig(level=logging.DEBUG)

import cv2
import numpy as np
import time
import os
from ultralytics import YOLO
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from sklearn.metrics.pairwise import cosine_similarity

# --- GAZEBO KÜTÜPHANELERİ ---
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image as GZImage

import collections
import collections.abc
# dronekit eski Python API'sini bekliyor (Python <3.10), uyumluluk için yama:
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

# --- OTOPİLOT KÜTÜPHANELERİ ---
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --- BELLEK YÖNETİMİ (GPU) ---
import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError:
        pass


# --- IOU HESAPLAMA ---
def calculate_iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter_area = max(0, x2 - x1) * max(0, y2 - y1)
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = box1_area + box2_area - inter_area

    if union_area == 0:
        return 0
    return inter_area / union_area


# --- KALMAN FİLTRESİ ---
class KalmanTracker:
    def __init__(self, initial_bbox):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array(
            [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03

        x1, y1, x2, y2 = initial_bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        self.kalman.statePre = np.array([[cx], [cy], [0], [0]], np.float32)
        self.kalman.statePost = np.array([[cx], [cy], [0], [0]], np.float32)
        self.w = x2 - x1
        self.h = y2 - y1

    def predict(self):
        pred = self.kalman.predict()
        cx, cy = pred[0], pred[1]
        x1 = int(cx - self.w / 2)
        y1 = int(cy - self.h / 2)
        x2 = int(cx + self.w / 2)
        y2 = int(cy + self.h / 2)
        return [x1, y1, x2, y2]

    def correct(self, bbox):
        x1, y1, x2, y2 = bbox
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        self.w = x2 - x1
        self.h = y2 - y1
        self.kalman.correct(np.array([[np.float32(cx)], [np.float32(cy)]]))


# --- RESNET BENZERLİK ---
class AracTakipSistemi:
    def __init__(self, hedef_gorsel_yolu):
        print("ResNet50 Hazırlanıyor...")
        base = ResNet50(weights='imagenet', include_top=False, pooling='avg')
        self.feat_extractor = Model(inputs=base.input, outputs=base.output)
        hedef_img = cv2.imread(hedef_gorsel_yolu)
        if hedef_img is None:
            raise FileNotFoundError(f"Hedef görsel bulunamadı veya okunamadı: {hedef_gorsel_yolu}")

        self.hedef_vektor = self.vektor_cikar(hedef_img)

    def vektor_cikar(self, img_source):
        img = cv2.cvtColor(img_source, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (224, 224))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        return self.feat_extractor.predict(x, verbose=0)

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
                    best_bbox = [x1, y1, x2, y2]
            except Exception:
                continue
        return best_bbox, best_sim


# --- GAZEBO CAMERA SUBSCRIBER ---
class CameraSubscriber:
    def __init__(self, topic_name):
        self.latest_frame = None
        self.node = Node()
        self.node.subscribe(GZImage, topic_name, self.cb)

    def cb(self, msg):
        try:
            width, height = msg.width, msg.height
            img_data = np.frombuffer(msg.data, dtype=np.uint8).copy()
            self.latest_frame = img_data.reshape((height, width, 3))
        except Exception:
            pass


# --- MOUSE CALLBACK ---
def mouse_callback(event, x, y, flags, param):
    durum_sozlugu = param
    if event == cv2.EVENT_LBUTTONDOWN:
        if 50 < x < 250 and 100 < y < 160:
            durum_sozlugu['baslatildi'] = True
            print("\n>> TESPİT VE TAKİP BAŞLATILDI! \n")


# ============================================================
#                DRONE / MAVLINK KONTROLCÜSÜ
# ============================================================
class DroneController:
    """
    Zaten armed + havada olan bir araca (SITL) bağlanır, GUIDED moda geçirir
    ve body-frame hız komutları gönderir.
    """
    def __init__(self, connection_string, baud=None, send_interval=0.2):
        print(f"[DRONE] Otopilota bağlanılıyor: {connection_string}")
        if baud:
            self.vehicle = connect(connection_string, wait_ready=['mode', 'armed', 'system_status', 'gps_0'], baud=baud, timeout=60)
        else:
            self.vehicle = connect(connection_string, wait_ready=['mode', 'armed', 'system_status', 'gps_0'], timeout=60)
        print(f"[DRONE] Bağlantı OK. Mevcut mod: {self.vehicle.mode.name}, "
              f"armed: {self.vehicle.armed}")
        self.send_interval = send_interval
        self._last_send = 0.0

    def guided_moda_gec(self):
        if self.vehicle.mode.name != "GUIDED":
            print("[DRONE] GUIDED moduna geçiliyor...")
            self.vehicle.mode = VehicleMode("GUIDED")
            t0 = time.time()
            while self.vehicle.mode.name != "GUIDED" and time.time() - t0 < 3:
                time.sleep(0.1)
            if self.vehicle.mode.name == "GUIDED":
                print("[DRONE] GUIDED moda geçildi.")
            else:
                print("[DRONE] UYARI: GUIDED moda geçilemedi, mevcut mod:",
                      self.vehicle.mode.name)

    def mevcut_irtifa(self):
        try:
            return self.vehicle.location.global_relative_frame.alt
        except Exception:
            return None

    def send_body_velocity(self, vx, vy, vz, force=False):
        """
        vx : ileri (+) / geri (-)    [body frame, m/s]
        vy : sağa  (+) / sola  (-)   [body frame, m/s]
        vz : aşağı (+) / yukarı (-)  [body frame, m/s]  (ArduPilot NED -> down pozitif)
        """
        now = time.time()
        if not force and (now - self._last_send) < self.send_interval:
            return
        self._last_send = now

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,  # sadece vx, vy, vz bileşenlerini kullan (pos ve accel yok say)
            0, 0, 0,             # x, y, z pozisyon (kullanılmıyor)
            vx, vy, vz,          # hız
            0, 0, 0,             # ivme (kullanılmıyor)
            0, 0)                # yaw, yaw_rate (kullanılmıyor)
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


class TakipController:
    """
    Görüntüdeki hedefin bbox merkezinin görüntü merkezine göre piksel sapmasını
    body-frame hız komutuna çeviren basit P (oransal) kontrolcü.
    """
    def __init__(self, frame_w, frame_h, kp_forward=0.004, kp_right=0.004,
                 max_yatay_vel=2.0, takip_irtifasi=None, kp_alt=0.5, max_dikey_vel=1.0):
        self.cx = frame_w / 2.0
        self.cy = frame_h / 2.0
        self.kp_forward = kp_forward
        self.kp_right = kp_right
        self.max_yatay_vel = max_yatay_vel
        self.takip_irtifasi = takip_irtifasi  # metre; None ise irtifaya dokunulmaz (vz=0)
        self.kp_alt = kp_alt
        self.max_dikey_vel = max_dikey_vel

    def hesapla(self, bbox, mevcut_irtifa=None):
        x1, y1, x2, y2 = bbox
        bx, by = (x1 + x2) / 2.0, (y1 + y2) / 2.0

        err_x = bx - self.cx   # + ise hedef görüntüde sağda
        err_y = by - self.cy   # + ise hedef görüntüde aşağıda

        # Eksen eşlemesi (dosya başındaki NOT'a bakın, gerekirse işaretleri değiştirin)
        vy = self._clamp(err_x * self.kp_right, self.max_yatay_vel)     # sağ/sol
        vx = self._clamp(err_y * self.kp_forward, self.max_yatay_vel)   # ileri/geri

        vz = 0.0
        if self.takip_irtifasi is not None and mevcut_irtifa is not None:
            alt_hata = self.takip_irtifasi - mevcut_irtifa  # + ise hedef irtifa daha yüksek -> yukarı çıkmalı
            # NED'de "down" pozitif olduğundan yukarı çıkmak için vz negatif olmalı
            vz = self._clamp(-alt_hata * self.kp_alt, self.max_dikey_vel)

        return vx, vy, vz

    @staticmethod
    def _clamp(v, limit):
        return max(-limit, min(limit, v))


# ============================================================
#                        ANA PROGRAM
# ============================================================
def main():
    TOPIC_NAME = "/world/sonoma/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
    HEDEF_GORSEL = './track_nadir.png'
    MODEL_PATH = './best.pt'

    # --- OTOPİLOT AYARLARI ---
    BAGLANTI_STR = "udp:127.0.0.1:14550"   # SITL/docker ortamınıza göre değiştirin
    TAKIP_IRTIFASI = None                   # örn: 15.0 -> sabit irtifada takip eder, None -> irtifaya dokunma
    MAX_YATAY_HIZ = 2.0                     # m/s, ilk testlerde düşük tutun (örn. 0.5-1.0)
    MAX_DIKEY_HIZ = 1.0                     # m/s

    print("Gimbal yere dik konuma getiriliyor...")
    os.system('gz topic -t "/gimbal/cmd_pitch" -p "data: -1.57"')

    camera_node = CameraSubscriber(TOPIC_NAME)
    takip_sistemi = AracTakipSistemi(HEDEF_GORSEL)
    model = YOLO(MODEL_PATH)

    # --- Otopilota bağlan (drone zaten terminalden armed + havada olmalı) ---
    drone = DroneController(BAGLANTI_STR)

    kalman_tracker = None
    hedef_kilitlendi = False
    frame_counter = 0
    sistem_durumu = {'baslatildi': False}
    SKIP_FRAMES = 5
    CONF_THRESHOLD = 0.40
    RESNET_SIMILARITY = 0.40
    IOU_THRESHOLD = 0.5
    prev_time = time.time()

    takip_ctrl = None  # ilk frame gelince (genişlik/yükseklik bilinince) oluşturulacak

    pencere_adi = 'Gazebo Canli Takip Sistemi (+ Otopilot)'
    cv2.namedWindow(pencere_adi, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(pencere_adi, 1024, 768)
    cv2.setMouseCallback(pencere_adi, mouse_callback, param=sistem_durumu)

    print("\n>>> SİSTEM HAZIR! Gazebo'dan görüntü bekleniyor...")

    try:
        while True:
            time.sleep(0.01)
            frame = camera_node.latest_frame
            if frame is None:
                continue
            frame = frame.copy()

            if takip_ctrl is None:
                h, w = frame.shape[:2]
                takip_ctrl = TakipController(
                    frame_w=w, frame_h=h,
                    max_yatay_vel=MAX_YATAY_HIZ,
                    takip_irtifasi=TAKIP_IRTIFASI,
                    max_dikey_vel=MAX_DIKEY_HIZ,
                )

            frame_counter += 1
            detection_step = (frame_counter % SKIP_FRAMES == 0)
            yolo_bboxlar = []

            if detection_step:
                results = model(frame, verbose=False, conf=CONF_THRESHOLD)
                for r in results:
                    for box in r.boxes:
                        coords = box.xyxy[0].cpu().numpy().astype(int)
                        yolo_bboxlar.append(coords)
                print(f"[DEBUG] YOLO tespit sayısı: {len(yolo_bboxlar)}")

            if hedef_kilitlendi and kalman_tracker is not None:
                tahmini_bbox = kalman_tracker.predict()
                if detection_step and len(yolo_bboxlar) > 0:
                    best_iou = -1
                    match_bbox = None
                    for y_box in yolo_bboxlar:
                        iou = calculate_iou(tahmini_bbox, y_box)
                        if iou > best_iou:
                            best_iou = iou
                            match_bbox = y_box
                    if best_iou > IOU_THRESHOLD:
                        kalman_tracker.correct(match_bbox)
                        tahmini_bbox = match_bbox

                x1, y1, x2, y2 = tahmini_bbox
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(frame, "TAKIP EDIYOR", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # --- OTOPİLOTA HIZ KOMUTU GÖNDER ---
                if sistem_durumu['baslatildi']:
                    drone.guided_moda_gec()
                    mevcut_irtifa = drone.mevcut_irtifa()
                    vx, vy, vz = takip_ctrl.hesapla(tahmini_bbox, mevcut_irtifa=mevcut_irtifa)
                    drone.send_body_velocity(vx, vy, vz)
                    cv2.putText(frame, f"vx:{vx:+.2f} vy:{vy:+.2f} vz:{vz:+.2f}",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            elif detection_step and not hedef_kilitlendi:
                best_bbox, score = takip_sistemi.dogrula(frame, yolo_bboxlar)
                print(f"[DEBUG] En iyi ResNet skoru: {score}")
                if best_bbox is not None and score > RESNET_SIMILARITY:
                    print(f"HEDEF BULUNDU! Skor: {score:.2f}")
                    kalman_tracker = KalmanTracker(best_bbox)
                    hedef_kilitlendi = True
                else:
                    # hedef henüz yok -> güvenlik amacıyla dron dursun/hover
                    if sistem_durumu['baslatildi']:
                        drone.dur()

            new_time = time.time()
            fps = 1 / (new_time - prev_time)
            prev_time = new_time
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Otopilot Mod: {drone.vehicle.mode.name}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

            # Buton çizimi
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
                # acil dur / takibi geçici olarak durdur
                sistem_durumu['baslatildi'] = False
                drone.dur()
                print(">> ACİL DUR: Takip durduruldu, drone hover'a alındı.")

    except KeyboardInterrupt:
        print("\nKapatiliyor...")
    finally:
        try:
            drone.kapat()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
