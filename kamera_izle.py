import sys
import time
import cv2
import numpy as np

# Gazebo kütüphanelerini import et
# Harmonic sürümü için genelde bu versiyon numaraları kullanılır
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# Sizin bulduğunuz topic adresi
TOPIC_NAME = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

def cb(msg):
    try:
        width = msg.width
        height = msg.height
        img_data = np.frombuffer(msg.data, dtype=np.uint8)
        img = img_data.reshape((height, width, 3))
        
        # Dönüşüm ve gösterme işlemi
        # OpenCV işlemleri bazen CPU yorar, gerekirse buraya frame atlama (skip) eklenebilir
        cv2.imshow("Drone Kamerasi", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        
        # waitKey(1) önemlidir, pencerenin tepki vermesini sağlar
        cv2.waitKey(1)
        
    except Exception as e:
        pass # Hata olursa akışı kesme

def main():
    # Gazebo Node oluştur
    node = Node()

    # Abone ol (Subscribe)
    print(f"Abone olunuyor: {TOPIC_NAME}")
    # Image sınıfı Protobuf mesaj tipidir
    node.subscribe(Image, TOPIC_NAME, cb)

    # Programın kapanmaması için döngü
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKapatiliyor...")

if __name__ == "__main__":
    main()