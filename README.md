# Arac Takip - Drone Simulasyon Projesi

Gazebo Harmonic simülatörü üzerinde ArduPilot yazılımını çalıştırarak drone simülasyonları gerçekleştiren kapsamlı bir proje.

## 🎯 Proje Amacı

Bu proje, Gazebo simülatörü içinde drone (İris) modelini kontrol etmek ve gimbal kamerasından görüntü almak için kullanılır. ArduPilot ve Gazebo entegrasyonu sayesinde gerçekçi drone simülasyonları yapılabilir.

## 🛠️ Teknolojiler

- **Gazebo Harmonic** - 3D simülatör
- **ArduPilot** - Drone kontrol yazılımı
- **Python 3** - Kamera görüntü işleme
- **OpenCV** - Bilgisayarlı görü uygulamaları
- **Docker** - Kontainerize ortam

## 📁 Proje Yapısı

```
arac_takip/
├── docker/
│   └── Dockerfile                          # Docker imajı tanımı
├── ardupilot_gazebo/
│   ├── models/                             # Gazebo drone modelleri
│   │   ├── iris_with_gimbal/               # Gimbal kameralı İris dronu
│   │   ├── iris_with_standoffs/            # Standoff'lı İris dronu
│   │   └── gimbal_small_3d/                # 3 DOF gimbal kamerası
│   ├── worlds/
│   │   ├── iris_runway.sdf                 # Runway ortamı
│   │   └── prius_on_sonoma_raceway/
│   │       └── sonoma.sdf                  # Sonoma Raceway ortamı (yeni)
│   ├── config/
│   │   ├── gazebo-iris-gimbal.parm         # İris gimbal parametreleri
│   │   └── sonoma-gps.parm                 # Sonoma dünyası parametreleri
│   └── include/, src/, hooks/              # Plugin ve sistem dosyaları
├── docker-compose.yml                      # Docker servis yapılandırması
├── kamera_izle.py                         # Kamera görüntü alıcı script
├── komutlar.txt                           # Kullanışlı komutlar referansı
└── README.md                              # Bu dosya
```

## 🚀 Başlangıç

### Gereksinimler

- Docker ve Docker Compose
- Linux ortamı (X11 display desteğine ihtiyaç vardır)
- GPU desteği (opsiyonel, yazılı konfigürasyonda NVIDIA GPU desteği mevcut)

### Kurulum

1. **Docker imajını oluşturun:**
```bash
# Çalıştıracağı yer: /home/bartu/development/arac_takip (proje kök dizini)
docker-compose build
```

2. **Container'ı başlatın:**
```bash
# Çalıştıracağı yer: /home/bartu/development/arac_takip (proje kök dizini)
docker-compose up -d
```

3. **Container'ın başladığını kontrol edin:**
```bash
# Çalıştıracağı yer: Herhangi bir dizin (host makinesi)
docker ps
```

## 💻 Kullanım

### Gazebo Simülasyonu Başlatma

1. **Container'a giriş yapın:**
```bash
# Çalıştıracağı yer: Host makinesi (herhangi bir dizin)
# Bu komut container içinde bash shell açar
docker exec -it tubitak_drone_sim bash

# Artık container içindesiniz, prompt'unuz değişecektir
# İçeride komutlar çalıştıracaksınız
```

2. **Gazebo'yu başlatın:**
```bash
# Çalıştıracağı yer: Container içinde
cd /gz_ws && source install/setup.bash

# İris Runway dünyasını başlat
gz sim -v4 -r iris_runway.sdf

# VEYA Sonoma Raceway dünyasını başlat
gz sim -v4 -r ardupilot_gazebo/worlds/prius_on_sonoma_raceway/sonoma.sdf
```

3. **Başka bir terminal açın ve ArduPilot SITL'i başlatın:**
```bash
# Çalıştıracağı yer: Container içinde
# İris Runway için:
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

# VEYA Sonoma Raceway için (gerçek Sonoma koordinatları):
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -l 38.2959,-122.4580,584,90
```


### Kamera Görüntüsü Görüntüleme

```bash
# Çalıştıracağı yer: Host makinesi (herhangi bir dizin)
# Container içindeki Python scriptini çalıştırır
docker exec -it tubitak_drone_sim python3 /home/droneuser/proje/kamera_izle.py
```

Bu script, gimbal kamerasından gelen görüntüleri OpenCV kullanarak ekranda gösterir.

---

**Komut Yürütme Özeti:**
- `docker-compose` komutları → Host makinede proje kök dizininde (`.../arac_takip/`)
- `docker exec` komutları → Host makinede herhangi bir dizinden
- Container içi komutlar → `docker exec -it tubitak_drone_sim bash` ile container'a girdikten sonra
  - Gazebo komutları → `/gz_ws` dizininde
  - Python scriptleri → `/home/droneuser/proje` dizininde

## 📂 Volume Mounts (Dosya Senkronizasyonu)

Docker container aşağıdaki dizinlerle senkronize çalışır:

| Host Makinesi | Docker Container | Amaç |
|---|---|---|
| `./models` | `/gz_ws/src/ardupilot_gazebo/models` | Drone modelleri |
| `./worlds` | `/gz_ws/src/ardupilot_gazebo/worlds` | Gazebo dünya dosyaları |
| `.` | `/home/droneuser/proje` | Tüm proje dosyaları |

**Avantajları:**
- Host makinede yapılan değişiklikler otomatik olarak container'da yansıtılır
- Container'da yapılan değişiklikler otomatik olarak host makinede görünür
- Geliştirme sırasında editör ile çalışmak mümkün

## � Simülasyon Dünyaları

### İris Runway
- **Konum:** Canberra, Avustralya (GPS: -35.363262, 149.165237)
- **Başlangıç Yüksekliği:** 0.195 metre
- **Ortam:** Düz runway ve çevresinde ağaçlar
- **Başlat:** `gz sim -v4 -r iris_runway.sdf`

### Sonoma Raceway (YENİ)
- **Konum:** Sonoma, Kaliforniya, USA (GPS: 38.2959, -122.4580)
- **Başlangıç Yüksekliği:** 30 metre (landing platform üzerinde)
- **Ortam:** Yarış pisti ve Prius hybrid aracı
- **Özel Özellikler:**
  - Landing platform: İris dronesinin güvenli şekilde inişi için
  - Prius Hybrid modeli: 4 kamera sensörü ile (ön, arka, sol, sağ)
  - Gimbal kamerası: 60 FPS çözünürlükte video yayını
- **Başlat:** `gz sim -v4 -r ardupilot_gazebo/worlds/prius_on_sonoma_raceway/sonoma.sdf`
- **SITL Komutu:** `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -l 38.2959,-122.4580,584,90`

## 📸 Kamera Konuları (Topics)

Simülasyondaki kameralar aşağıdaki Gazebo topic'leri üzerinden görüntü yayını yaparlar:

### İris Runway
```
/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

### Sonoma Raceway - Gimbal Kamerası
```
/world/sonoma/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

### Sonoma Raceway - Prius Kameraları
```
# Ön kamera
/world/sonoma/model/prius_hybrid/link/chassis/sensor/front_camera_sensor/image

# Arka kamera
/world/sonoma/model/prius_hybrid/link/chassis/sensor/back_camera_sensor/image

# Sol kamera
/world/sonoma/model/prius_hybrid/link/chassis/sensor/left_camera_sensor/image

# Sağ kamera
/world/sonoma/model/prius_hybrid/link/chassis/sensor/right_camera_sensor/image
```

**Not:** Dünya adını veya model adını değiştirirseniz topic yollarını buna göre güncelleyin.

## 🐛 Sorun Giderme

### X11 Display Hatası
```
Error: DISPLAY is not set
```
**Çözüm:** Host makinede `DISPLAY` environment variable'ı ayarlandığından emin olun:
```bash
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
echo $DISPLAY
# Çıktı: :0 veya :1 benzeri
```

### Gazebo Başlamıyor
1. Container'ın doğru başladığını kontrol edin: 
```bash
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
docker ps
```
2. Container loglarını kontrol edin: 
```bash
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
docker logs tubitak_drone_sim
```
3. GPU sorunları yaşıyorsanız docker-compose.yml'de GPU ayarlarını kontrol edin

### Kamera Görüntüsü Alınamıyor
1. Gazebo'nun çalışıyor olduğundan emin olun
2. Doğru topic adını kullandığınızdan emin olun:
```bash
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
docker exec -it tubitak_drone_sim gz topic -l
```
3. Sonoma Raceway kullanıyorsanız topic adında `sonoma` bulunması gerektiğini kontrol edin:
```bash
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
docker exec -it tubitak_drone_sim gz topic -l | grep -i camera
```

### Sonoma Raceway'de Iris Düşüyor
- Gazebo'yu başlattıktan sonra fizik simülasyonun başlaması için biraz bekleyin
- Landing platform'un iris'in altında bulunduğundan emin olun
- Iris'in başlangıç yüksekliği 30 metre olmalı (Z=30)

## 🎮 Drone Kontrolü

ArduPilot MAVProxy veya Mission Planner ile kontrol edilebilir:

```bash
# Container içinde (docker exec -it tubitak_drone_sim bash ile girdikten sonra)

# İris Runway ortamı (varsayılan konum)
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

# Sonoma Raceway ortamı (Sonoma, Kaliforniya koordinatları)
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -l 38.2959,-122.4580,584,90
```

### MAVProxy Komutları (SITL konsolunda)

```
arm throttle          # Motorları kımıldatmaya hazırla
takeoff 30            # 30 metreye kalkış yap
land                  # İniş yap
wp add 38.2959 -122.4580 30   # Waypoint ekle
mode guided           # Guided moda geç
```

### DroneKit Python ile Programlı Kontrol

```bash
# Çalıştıracağı yer: Host makinesi (herhangi bir dizin)
docker exec -it tubitak_drone_sim python3 /home/droneuser/proje/script_adı.py
```

## 🔧 Yapılandırma

### docker-compose.yml

Önemli ortam değişkenleri:
- `DISPLAY` - X11 display (Linux GUI desteği için)
- `NVIDIA_VISIBLE_DEVICES` - GPU ayarı

GPU desteği devre dışı bırakmak için:
```yaml
environment:
  - LIBGL_ALWAYS_SOFTWARE=1
```

## 📝 Komutlar Referansı

Sık kullanılan komutlar için `komutlar.txt` dosyasını kontrol edin.

## 🔄 Güncellemeler

Modelleri veya dünya dosyalarını güncellemek için:
1. `./ardupilot_gazebo/models/` veya `./ardupilot_gazebo/worlds/` dizinindeki dosyaları düzenleyin
2. Değişiklikler otomatik olarak container'da yansıtılır (volume mount sayesinde)
3. Gazebo'yu yeniden başlatın

## 📊 Proje Özellikleri

### Gimbal Kamerası
- **Çözünürlük:** 1280x720 piksel
- **Format:** R8G8B8 (RGB)
- **FPS:** 60 frame/saniye
- **DOF:** 3 eksen (roll, pitch, yaw)
- **Kontrol:** ArduPilot channels 8-10

### Prius Hybrid Modeli (Sonoma Raceway)
- **Kameralar:** 4 adet (ön, arka, sol, sağ)
- **Çözünürlük:** 800x800 piksel
- **FPS:** 60 frame/saniye her kamera
- **Aç:** Ackermann steering sistemi ile kontrol
- **Harita Komutu:** `/cmd_vel` topic'i üzerinden

### Landing Platform (Sonoma Raceway)
- **Boyutlar:** 3m x 3m x 0.2m kalınlık
- **Konum:** Z=29.8 metre (iris collision box'ının altında)
- **Fizik:** Statik, sabit yerleşim
- **Amaç:** İris dronesinin güvenli inişi

## 📚 Kaynaklar

- [Gazebo Harmonic Dokümantasyonu](https://gazebosim.org/docs/harmonic)
- [ArduPilot Dokümantasyonu](https://ardupilot.org/dev/)
- [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)

## ⚙️ Container Yönetimi

```bash
# Container'ı durdur
# Çalıştıracağı yer: Host makinesi, proje kök dizininde
docker-compose down

# Container'ı yeniden başlat
# Çalıştıracağı yer: Host makinesi, proje kök dizininde
docker-compose restart

# Container loglarını göster
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
docker-compose logs -f

# Container'a giriş yap
# Çalıştıracağı yer: Host makinesi, herhangi bir dizin
docker exec -it tubitak_drone_sim bash
```

## 📝 Lisans

Bu proje Gazebo, ArduPilot ve ilgili açık kaynaklı yazılımları kullanır.

---

**Not:** Docker container'ı ilk kez oluştururken `Dockerfile` içindeki paketlerin indirilmesi birkaç dakika sürebilir. Sabırlı olun ve işlemın tamamlanmasını bekleyin.
