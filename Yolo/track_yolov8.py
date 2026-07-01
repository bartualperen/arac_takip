import cv2
import numpy as np
import tensorflow as tf
from ultralytics import YOLO  # YOLOv8 kütüphanesi
from tensorflow.keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from sklearn.metrics.pairwise import cosine_similarity
import time 

# --- 1. BELLEK YÖNETİMİ (TensorFlow & PyTorch Çakışmasını Önle) ---
# GPU belleğini dinamik olarak yönetmesini sağlıyoruz
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError as e:
        print(e)

# --- 2. BENZERLİK ANALİZİ SINIFI (ResNet50) ---
class AracTakipSistemi:
    def __init__(self, hedef_gorsel_yolu, threshold=0.85):
        self.threshold = threshold
        print("ResNet50 Özellik Çıkarıcı Hazırlanıyor...")
        
        # ResNet50'yi özellik çıkarıcı olarak yükle
        base_model = ResNet50(weights='imagenet', include_top=False, pooling='avg')
        self.feature_extractor = Model(inputs=base_model.input, outputs=base_model.output)
        
        # Hedef aracı analiz et
        self.hedef_vektor = self.gorsel_vektor_cikar(hedef_gorsel_yolu)
        print("Sistem Hazır: Hedef araç imzası oluşturuldu.")

    def gorsel_vektor_cikar(self, img_source):
        # Girdi bir dosya yolu mu yoksa görüntü matrisi mi?
        if isinstance(img_source, str):
            img = cv2.imread(img_source)
            if img is None: raise ValueError("Görsel bulunamadı!")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            img = cv2.cvtColor(img_source, cv2.COLOR_BGR2RGB)

        # Modelin beklediği boyuta getir (224x224)
        img = cv2.resize(img, (224, 224))
        x = image.img_to_array(img)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        
        # Vektörü çıkar (TensorFlow verbose=0 sessiz mod)
        return self.feature_extractor.predict(x, verbose=0)

    def tespiti_dogrula(self, frame, bboxlar):
        en_iyi_benzerlik = -1
        en_iyi_bbox = None

        for bbox in bboxlar:
            # YOLOv8 bbox formatı: [x1, y1, x2, y2]
            x1, y1, x2, y2 = map(int, bbox)
            
            # Görüntü sınırlarını kontrol et
            h_img, w_img, _ = frame.shape
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w_img, x2), min(h_img, y2)
            
            # Aracı kırp
            kirpilmis_arac = frame[y1:y2, x1:x2]
            if kirpilmis_arac.size == 0: continue

            # Vektör çıkar ve karşılaştır
            try:
                aday_vektor = self.gorsel_vektor_cikar(kirpilmis_arac)
                benzerlik = cosine_similarity(self.hedef_vektor, aday_vektor)[0][0]

                if benzerlik > self.threshold:
                    if benzerlik > en_iyi_benzerlik:
                        en_iyi_benzerlik = benzerlik
                        en_iyi_bbox = [x1, y1, x2, y2]
            except Exception as e:
                print(f"Hata: {e}")
                continue

        return en_iyi_bbox, en_iyi_benzerlik

# --- 3. ANA DÖNGÜ (YOLOv8 Entegrasyonu) ---

def main():
    # AYARLAR
    VIDEO_PATH = 'D:\\workspace\\Term_7\\Tasarim\\videos\\Visdrone_uav0000305_00000_v.mp4'  # Videonun yolu veya 0 (webcam için)
    HEDEF_GORSEL = 'track.png'   # Takip edilecek aracın resmi
    MODEL_PATH = 'best.pt'             # Eğittiğin YOLOv7 ağırlık dosyası (yoksa 'yolov7.pt')
    CONF_THRESHOLD = 0.40              # YOLO tespit güven eşiği
    SIMILARITY_THRESHOLD = 0.85        # Hedef doğrulama eşiği     

    # 1. Takip Sistemini (ResNet) Başlat
    try:
        takip_sistemi = AracTakipSistemi(HEDEF_GORSEL, threshold=SIMILARITY_THRESHOLD)
    except Exception as e:
        print(f"HATA: Hedef görsel yüklenemedi. ({e})")
        return

    # 2. YOLOv8 Modelini Yükle
    print(f"YOLOv8 Modeli Yükleniyor ({MODEL_PATH})...")
    try:
        model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"Model yükleme hatası: {e}")
        return

    # 3. Videoyu Aç
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print("Video açılamadı.")
        return

    # --- FPS Değişkenleri ---
    prev_frame_time = 0
    new_frame_time = 0
    toplam_kare_sayisi = 0
    
    print("Takip başlatılıyor... Çıkmak için 'q' tuşuna basın.")

    pencere_ismi = 'Otonom Takip Sistemi'
    cv2.namedWindow(pencere_ismi, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(pencere_ismi, 1024, 768)
    
    baslangic_zamani = time.time()
    while True:
        # FPS Hesaplaması Başlangıcı (Okuma süresi dahil edilmezse buraya, edilirse yukarıya)
        new_frame_time = time.time()

        ret, frame = cap.read()
        if not ret: break

        toplam_kare_sayisi += 1
        
        # --- A. YOLOv8 İle Tespit ---
        results = model(frame, verbose=False, conf=CONF_THRESHOLD)
        
        yolo_bboxlar = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                yolo_bboxlar.append([x1, y1, x2, y2])
                # Adayları mavi çiz
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 1)

        # --- B. Hedef Doğrulama ---
        hedef_bbox, skor = takip_sistemi.tespiti_dogrula(frame, yolo_bboxlar)

        # --- C. Sonuç Gösterimi ---
        if hedef_bbox is not None:
            tx1, ty1, tx2, ty2 = hedef_bbox
            cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), (0, 255, 0), 3)
            label = f"HEDEF: {skor:.2f}"
            cv2.putText(frame, label, (tx1, ty1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # --- FPS HESAPLAMA VE YAZDIRMA ---
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        cv2.putText(frame, f"FPS: {int(fps)}", (7, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow(pencere_ismi, frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    bitis_zamani = time.time()
    toplam_sure = bitis_zamani - baslangic_zamani
    
    cap.release()
    cv2.destroyAllWindows()
    
    if toplam_sure > 0:
        ortalama_fps = toplam_kare_sayisi / toplam_sure
        print("-" * 40)
        print(f"SONUÇ RAPORU:")
        print(f"Toplam Süre       : {toplam_sure:.2f} saniye")
        print(f"İşlenen Kare Sayısı : {toplam_kare_sayisi}")
        print(f"ORTALAMA FPS      : {ortalama_fps:.2f}")
        print("-" * 40)
    else:
        print("Video işlenemedi veya çok kısa sürdü.")

if __name__ == "__main__":
    main()