import cv2
import torch
import numpy as np
import os

# --- TENSORFLOW BELLEK AYARI (GPU Çökmesini Önlemek İçin En Başta Olmalı) ---
import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            # TensorFlow'a sadece ihtiyacı kadar bellek almasını söylüyoruz
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError as e:
        print(e)

from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from sklearn.metrics.pairwise import cosine_similarity

# --- 1. BÖLÜM: BENZERLİK ANALİZİ SINIFI ---
class AracTakipSistemi:
    def __init__(self, hedef_gorsel_yolu, threshold=0.85):
        self.threshold = threshold
        print("Model hazırlanıyor (ResNet50)...")
        # TensorFlow modelini CPU'da çalışmaya zorlayabiliriz (GPU'yu YOLO'ya bırakmak için)
        # Eğer GPU yetmezse aşağıdaki 'with tf.device...' satırını açabilirsin:
        # with tf.device('/CPU:0'):
        base_model = ResNet50(weights='imagenet', include_top=False, pooling='avg')
        self.feature_extractor = Model(inputs=base_model.input, outputs=base_model.output)
        
        self.hedef_vektor = self.gorsel_vektor_cikar(hedef_gorsel_yolu)
        print("Sistem Hazır: Hedef araç analizi tamamlandı.")

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
            except:
                continue

            benzerlik = cosine_similarity(self.hedef_vektor, aday_vektor)[0][0]

            if benzerlik > self.threshold:
                if benzerlik > en_iyi_benzerlik:
                    en_iyi_benzerlik = benzerlik
                    en_iyi_bbox = [x1, y1, x2, y2]

        return en_iyi_bbox, en_iyi_benzerlik

# --- 2. BÖLÜM: YOLOv7 TESPİT VE ANA DÖNGÜ ---

def main():
    # --- AYARLAR ---
    VIDEO_PATH = 'D:\\workspace\\Term_7\\Tasarim\\videos\\Visdrone_uav0000305_00000_v.mp4'  # Videonun yolu veya 0 (webcam için)
    HEDEF_GORSEL = 'track.png'   # Takip edilecek aracın resmi
    MODEL_PATH = 'best.pt'             # Eğittiğin YOLOv7 ağırlık dosyası (yoksa 'yolov7.pt')
    CONF_THRESHOLD = 0.40              # YOLO tespit güven eşiği
    SIMILARITY_THRESHOLD = 0.85        # Hedef doğrulama eşiği     

    # 1. Takip Sistemini Başlat
    try:
        takip_sistemi = AracTakipSistemi(HEDEF_GORSEL, threshold=SIMILARITY_THRESHOLD)
    except Exception as e:
        print(f"HATA: Hedef görsel yüklenemedi. ({e})")
        return

    # 2. YOLOv7 Modelini Yükle (PyTorch 2.6 Yama Uygulanmış)
    print("YOLOv7 Modeli Yükleniyor...")
    
    # --- PYTORCH 2.6 DÜZELTMESİ (Monkey Patch) ---
    # Torch.load fonksiyonunu geçici olarak değiştiriyoruz ki weights_only hatası vermesin.
    _original_load = torch.load
    def safe_load(*args, **kwargs):
        if 'weights_only' not in kwargs:
            kwargs['weights_only'] = False
        return _original_load(*args, **kwargs)
    torch.load = safe_load
    # ---------------------------------------------

    try:
        # Eğer 'custom' hata verirse ve model standart YOLOv7 ise source='local' deneyebilirsin
        model = torch.hub.load('WongKinYiu/yolov8', 'custom', MODEL_PATH, force_reload=False, trust_repo=True)
    except Exception as e:
        print(f"\nCRITICAL ERROR: Model yüklenemedi.\nDetay: {e}")
        print("İPUCU: Eğer 'best.pt' dosyanız YOLOv8 (Ultralytics) ile eğitildiyse, YOLOv7 kodu bunu açamaz.")
        # Orijinal fonksiyonu geri yükle
        torch.load = _original_load
        return

    # Orijinal fonksiyonu geri yükle
    torch.load = _original_load
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)
    model.eval()

    # 3. Videoyu Başlat
    cap = cv2.VideoCapture(VIDEO_PATH)
    
    if not cap.isOpened():
        print(f"Hata: Video açılamadı: {VIDEO_PATH}")
        return

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Video bitti.")
            break

        # A. YOLOv7 Tespiti Yap
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Inference
        with torch.no_grad(): # Bellek tasarrufu için gradient hesaplamayı kapat
            results = model(img_rgb)
        
        df = results.pandas().xyxy[0] 
        detections = df[df['confidence'] > CONF_THRESHOLD]

        yolo_bboxlar = []
        for index, row in detections.iterrows():
            xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
            yolo_bboxlar.append([xmin, ymin, xmax, ymax])
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 1)

        # B. Hedef Doğrulama
        hedef_bbox, skor = takip_sistemi.tespiti_dogrula(frame, yolo_bboxlar)

        # C. Sonucu Çiz
        if hedef_bbox is not None:
            x1, y1, x2, y2 = hedef_bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            label = f"HEDEF (%{skor*100:.1f})"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            print(f"Hedef Tespit Edildi! Benzerlik: {skor}")
        
        cv2.imshow('Otonom Arac Takip', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()