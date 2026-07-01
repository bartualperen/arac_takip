import cv2
import numpy as np
import tensorflow as tf
from ultralytics import YOLO
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from sklearn.metrics.pairwise import cosine_similarity
import time

# --- 1. BELLEK YÖNETİMİ ---
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError as e:
        print(e)

# --- 2. BENZERLİK ANALİZİ SINIFI ---
class AracTakipSistemi:
    def __init__(self, hedef_gorsel_yolu, threshold=0.85):
        self.threshold = threshold
        print("ResNet50 Özellik Çıkarıcı Hazırlanıyor...")
        # ResNet50 yükleniyor
        base_model = ResNet50(weights='imagenet', include_top=False, pooling='avg')
        self.feature_extractor = Model(inputs=base_model.input, outputs=base_model.output)
        
        self.hedef_vektor = self.gorsel_vektor_cikar(hedef_gorsel_yolu)
        print("Sistem Hazır: Hedef araç imzası oluşturuldu.")

    def gorsel_vektor_cikar(self, img_source):
        if isinstance(img_source, str):
            img = cv2.imread(img_source)
            if img is None: raise ValueError("Görsel bulunamadı!")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            img = cv2.cvtColor(img_source, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, (224, 224))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        return self.feature_extractor.predict(x, verbose=0)

    def tespiti_dogrula(self, frame, bboxlar):
        en_iyi_benzerlik = -1
        en_iyi_bbox = None

        for bbox in bboxlar:
            x1, y1, x2, y2 = map(int, bbox)
            h_img, w_img, _ = frame.shape
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_img, x2), min(h_img, y2)
            
            kirpilmis_arac = frame[y1:y2, x1:x2]
            if kirpilmis_arac.size == 0: continue

            try:
                aday_vektor = self.gorsel_vektor_cikar(kirpilmis_arac)
                benzerlik = cosine_similarity(self.hedef_vektor, aday_vektor)[0][0]

                if benzerlik > en_iyi_benzerlik:
                    en_iyi_benzerlik = benzerlik
                    en_iyi_bbox = [x1, y1, x2, y2]
            
            except Exception as e:
                continue

        return en_iyi_bbox, en_iyi_benzerlik

# --- 3. ANA DÖNGÜ ---
def main():
    # AYARLAR
    VIDEO_PATH = 'D:\\workspace\\Term_7\\Tasarim\\videos\\Visdrone_uav0000305_00000_v.mp4'  # Videonun yolu veya 0 (webcam için)
    HEDEF_GORSEL = 'track.png'   # Takip edilecek aracın resmi
    MODEL_PATH = 'best.pt'            # YOLO modeli
    CONF_THRESHOLD = 0.40             # YOLO Güven Eşiği
    
    # STRATEJİ AYARLARI
    # Bu değerin üzerindeyse YOLO'yu kapatıp Tracker'a geçer
    KILITLENME_ESIGI = 0.90 
    
    # Tracker kaybederse veya benzerlik düşerse tekrar arama yapmak için alt limit (opsiyonel kontrol için)
    ARAMA_MODU_ESIGI = 0.85 

    # --- SİSTEM BAŞLATMA ---
    try:
        takip_sistemi = AracTakipSistemi(HEDEF_GORSEL, threshold=ARAMA_MODU_ESIGI)
    except Exception as e:
        print(f"HATA: {e}")
        return

    print(f"YOLOv8 Modeli Yükleniyor...")
    try:
        model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"Model hatası: {e}")
        return

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("Video açılamadı.")
        return

    # Görsel Ayarlar
    pencere_ismi = 'Akilli Takip Sistemi'
    cv2.namedWindow(pencere_ismi, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(pencere_ismi, 1024, 768)

    # --- DEĞİŞKENLER ---
    tracker = None
    takip_modu_aktif = False # False: Arama Modu (YOLO), True: Takip Modu (Tracker)
    
    prev_frame_time = 0
    new_frame_time = 0
    
    print("\n--- SİSTEM BAŞLADI ---")
    print("Mod 1: ARAMA (YOLO + ResNet) ile hedef aranıyor...")
    
    while True:
        new_frame_time = time.time()
        ret, frame = cap.read()
        if not ret: break

        height, width, _ = frame.shape

        # ============================================================
        # DURUM 1: TAKİP MODU (HIZLI MOD)
        # ============================================================
        if takip_modu_aktif and tracker is not None:
            # Sadece Tracker güncellemesi yap (YOLO ve ResNet YOK)
            basari, box = tracker.update(frame)
            
            if basari:
                # Tracker koordinatları (x, y, w, h) formatındadır
                tx, ty, tw, th = map(int, box)
                
                # Çizim
                cv2.rectangle(frame, (tx, ty), (tx+tw, ty+th), (0, 255, 0), 3)
                cv2.putText(frame, "KILITLENDI (TRACKER)", (tx, ty-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Bilgi mesajı
                cv2.putText(frame, "MOD: TAKIP (HIZLI)", (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            else:
                # Nesne kayboldu! Arama moduna geri dön
                print(">> Nesne Takibi Kayboldu! Arama moduna dönülüyor...")
                takip_modu_aktif = False
                tracker = None

        # ============================================================
        # DURUM 2: ARAMA MODU (YOLO + RESNET)
        # ============================================================
        else:
            # YOLO Tespiti
            results = model(frame, verbose=False, conf=CONF_THRESHOLD)
            
            yolo_bboxlar = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    yolo_bboxlar.append([int(x1), int(y1), int(x2), int(y2)])
                    # Arama yaparken adayları mavi çiz
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 1)
            
            # ResNet ile Doğrulama
            en_iyi_bbox, skor = takip_sistemi.tespiti_dogrula(frame, yolo_bboxlar)
            
            # Eğer hedef bulunduysa
            if en_iyi_bbox is not None and skor > ARAMA_MODU_ESIGI:
                x1, y1, x2, y2 = en_iyi_bbox
                
                # Ekrana çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                cv2.putText(frame, f"BULUNDU: {skor:.2f}", (x1, y1-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # --- KRİTİK NOKTA: KİLİTLENME KARARI ---
                if skor >= KILITLENME_ESIGI:
                    print(f">> Hedef Kilitlendi! Skor: {skor:.2f}. Tracker başlatılıyor...")
                    
                    # Tracker'ı Başlat (CSRT daha hassas, KCF daha hızlıdır)
                    # Hız için: cv2.TrackerKCF_create()
                    # Hassasiyet için: cv2.TrackerCSRT_create()
                    tracker = cv2.TrackerCSRT_create() 
                    
                    # Tracker (x, y, w, h) ister, YOLO (x1, y1, x2, y2) verir. Dönüştür:
                    w = x2 - x1
                    h = y2 - y1
                    tracker.init(frame, (x1, y1, w, h))
                    
                    takip_modu_aktif = True # Modu değiştir
            
            cv2.putText(frame, "MOD: ARAMA (SCAN)", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # FPS Göstergesi
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow(pencere_ismi, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()