import cv2
import numpy as np
import tensorflow as tf
from ultralytics import YOLO
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from sklearn.metrics.pairwise import cosine_similarity
import time
import os # Gimbal komutu göndermek için
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# --- BELLEK YÖNETİMİ ---
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError: pass

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
    
    if union_area == 0: return 0
    return inter_area / union_area

# --- GAZEBO GÖRÜNTÜ ABONE SINIFI ---
class CameraSubscriber:
    def __init__(self, topic_name):
        self.node = Node()
        self.latest_frame = None
        self.node.subscribe(Image, topic_name, self.listener_callback)

    def listener_callback(self, data):
        try:
            width = data.width
            height = data.height
            img_data = np.frombuffer(data.data, dtype=np.uint8)
            img_rgb = img_data.reshape((height, width, 3))
            self.latest_frame = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

# --- KALMAN FİLTRESİ ---
class KalmanTracker:
    def __init__(self, initial_bbox):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
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

# --- RESNET (BENZERLİK) ---
class AracTakipSistemi:
    def __init__(self, hedef_gorsel_yolu):
        print("ResNet50 Hazırlanıyor...")
        base = ResNet50(weights='imagenet', include_top=False, pooling='avg')
        self.feat_extractor = Model(inputs=base.input, outputs=base.output)
        self.hedef_vektor = self.vektor_cikar(hedef_gorsel_yolu)

    def vektor_cikar(self, img_source):
        if isinstance(img_source, str):
            img = cv2.imread(img_source)
            if img is None: raise ValueError("Görsel yok!")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
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
            h, w, _ = frame.shape
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0: continue
            try:
                vec = self.vektor_cikar(crop)
                sim = cosine_similarity(self.hedef_vektor, vec)[0][0]
                if sim > best_sim:
                    best_sim = sim
                    best_bbox = [x1, y1, x2, y2]
            except: continue
        return best_bbox, best_sim

# --- FARE TIKLAMA OLAYI (BUTON KONTROLÜ) ---
def mouse_callback(event, x, y, flags, param):
    """
    Kullanıcı butona tıkladığında 'baslatildi' durumunu True yapar.
    """
    durum_sozlugu = param
    if event == cv2.EVENT_LBUTTONDOWN:
        # Buton Koordinatları: x(50-250), y(100-160)
        if 50 < x < 250 and 100 < y < 160:
            durum_sozlugu['baslatildi'] = True
            print("\n>> TESPİT BAŞLATILDI! \n")

# --- ANA PROGRAM ---
def main():
    # 1. Kamerayı Sabitle (Bulduğunuz 1300 PWM değerine karşılık gelen açı)
    print("Gimbal yere dik konuma getiriliyor...")
    os.system('gz topic -t "/gimbal/cmd_pitch" -p "data: -1.57"')
    
    # 2. Kamera Düğümünü (Node) Başlat
    CAMERA_TOPIC = '/world/sonoma/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
    print(f"Kamera topic'e abone olunuyor: {CAMERA_TOPIC}")
    camera_node = CameraSubscriber(CAMERA_TOPIC)
    
    HEDEF_GORSEL = 'track_nadir.png'  # Nadir (kuş bakışı) hedef görseli kullanın 
    MODEL_PATH = 'best.pt'
    
    # AYARLAR (Kuş bakışı için optimize edildi)
    SKIP_FRAMES = 5        
    CONF_THRESHOLD = 0.40
    RESNET_SIMILARITY = 0.80  # Kuş bakışı benzerlik için biraz düşürüldü
    IOU_THRESHOLD = 0.5       # Tepeden bakışta araç çakışması az olduğu için artırıldı

    takip_sistemi = AracTakipSistemi(HEDEF_GORSEL)
    print("YOLO Yükleniyor...")
    model = YOLO(MODEL_PATH)
    
    kalman_tracker = None
    hedef_kilitlendi = False
    frame_counter = 0      
    sistem_durumu = {'baslatildi': False}
    prev_time = time.time()
    
    pencere_adi = 'Gazebo Canlı Takip Sistemi'
    cv2.namedWindow(pencere_adi, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(pencere_adi, 1024, 768)
    cv2.setMouseCallback(pencere_adi, mouse_callback, param=sistem_durumu)

    print("\n>>> SİSTEM HAZIR! Gazebo'dan görüntü bekleniyor...")

    while True:
        frame = camera_node.latest_frame
        
        if frame is None:
            time.sleep(0.01)
            continue # Görüntü henüz gelmediyse bekle
        
        new_time = time.time()
        son_kare = frame.copy()
        
        if not sistem_durumu['baslatildi']:
            # Buton Çizimi
            overlay = frame.copy()
            cv2.rectangle(overlay, (50, 100), (250, 160), (0, 200, 0), -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
            cv2.putText(frame, "BASLAT", (85, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(frame, "Gazebo Bagli - Beklemede", (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            frame_counter += 1
            detection_step = (frame_counter % SKIP_FRAMES == 0)
            yolo_bboxlar = []
            
            if detection_step:
                results = model(frame, verbose=False, conf=CONF_THRESHOLD)
                for r in results:
                    for box in r.boxes:
                        coords = box.xyxy[0].cpu().numpy().astype(int)
                        yolo_bboxlar.append(coords)

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
                cv2.putText(frame, "TAKIP EDIYOR", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            elif detection_step and not hedef_kilitlendi:
                cv2.putText(frame, "HEDEF ARANIYOR...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                best_bbox, score = takip_sistemi.dogrula(frame, yolo_bboxlar)
                if best_bbox is not None and score > RESNET_SIMILARITY:
                    print(f"HEDEF BULUNDU! Skor: {score:.2f}")
                    kalman_tracker = KalmanTracker(best_bbox)
                    hedef_kilitlendi = True

        # FPS Hesaplama
        fps = 1 / (new_time - prev_time)
        prev_time = new_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow(pencere_adi, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
