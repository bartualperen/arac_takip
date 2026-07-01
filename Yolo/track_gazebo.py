#!/usr/bin/env python3
"""
Gazebo + ArduPilot Drone Takip Sistemi
- YOLOv8 (best.pt) ile 'tasit' (class=1) tespiti
- Kalman filtresi ile smooth takip
- gz transport ile gimbal pitch/yaw PID kontrolu
- MAVLink ile drone velocity kontrolu (GUIDED mod)

Calistirma:
    PYTHONPATH=/usr/lib/python3/dist-packages python3 track_gazebo.py

Kisa yollar:
    t → Arm + Kalkis (30m)
    l → Takibi baslat/durdur
    r → Tracker sifirla (yeniden ara)
    h → Hover (yerinde dur)
    q → Cikis
"""

import sys
import os

# gz.msgs10 protobuf dosyalari eski formatta; venv'in yeni protobuf'u (4.x) ile
# uyumlu olmasi icin saf-Python uygulamasi gerekiyor.
# (performans kaybi ihmal edilebilir: sadece mesaj (de)serializasyonu etkilenir)
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import cv2
import numpy as np
import time
import threading

# gz Python paketleri venv'de symlink ile kuruluysa direkt import edilir.
# Yoksa sistem Python dist-packages'tan yuklenir (PYTHONPATH ile).
# Her iki durumda da venv paketleri (ultralytics/torch/sympy) oncelikli olmali;
# bu nedenle venv site-packages'i sys.path'in basina tasiyoruz.
_VENV_SITE = '/opt/venv/lib/python3.10/site-packages'
if _VENV_SITE in sys.path and sys.path.index(_VENV_SITE) > 0:
    sys.path.remove(_VENV_SITE)
    sys.path.insert(0, _VENV_SITE)

try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.double_pb2 import Double
except ImportError:
    sys.exit("HATA: gz modulu bulunamadi.\n"
             "Cozum: sudo ln -sf /usr/lib/python3/dist-packages/gz "
             "/opt/venv/lib/python3.10/site-packages/gz")

from ultralytics import YOLO
from pymavlink import mavutil

# ── Konfigürasyon ─────────────────────────────────────────────────────────────

CAM_TOPIC    = '/world/sonoma/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image'
MODEL_PATH   = os.path.join(os.path.dirname(__file__), 'best.pt')
MAVLINK_URI  = 'udpin:0.0.0.0:14550'

FRAME_W, FRAME_H = 1280, 720
TASIT_CLASS  = 1       # best.pt'deki 'tasit' sinifi
CONF         = 0.40    # YOLO guven esigi
SKIP         = 3       # Her kac karede bir YOLO calistir

# Gimbal sinirlar (radyan)
PITCH_NADIR  = -0.785  # 45 derece one-asagi (baslangic)
PITCH_MIN    = -1.57   # En fazla yere dik
PITCH_MAX    = -0.3    # En fazla ufka yakin
YAW_MAX      =  1.0    # ±1.0 rad gimbal yaw

# PID kazanimlari
KP_GIMBAL_YAW   = 0.8   # Gimbal yaw oransal kazanim
KP_GIMBAL_PITCH = 0.4   # Gimbal pitch oransal kazanim
KP_VEL          = 0.8   # Drone velocity kazanim (dusuk = yumusak hareket)
VEL_MAX         = 1.5   # Maks hiz (m/s)

DEAD_ZONE    = 0.08    # Hata bu esik altindaysa komut gonderme (frame'in %8'i)
VEL_RATE     = 0.10    # Drone velocity komutlari arasi minimum sure (saniye) = 10Hz
IOU_THRESH   = 0.30    # Kalman guncelleme icin minimum IOU
TAKEOFF_ALT  = 30.0    # Kalkis yuksekligi (m)

# ── Kamera ────────────────────────────────────────────────────────────────────

class CameraNode:
    def __init__(self):
        self._node  = Node()
        self._frame = None
        self._lock  = threading.Lock()
        self._node.subscribe(Image, CAM_TOPIC, self._cb)

    def _cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            with self._lock:
                self._frame = bgr
        except Exception:
            pass

    def get(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

# ── Gimbal ────────────────────────────────────────────────────────────────────

class GimbalController:
    """gz transport araciligiyla gimbal pitch/yaw kontrolu."""

    def __init__(self):
        self._node      = Node()
        self._pub_pitch = self._node.advertise('/gimbal/cmd_pitch', Double)
        self._pub_yaw   = self._node.advertise('/gimbal/cmd_yaw',   Double)
        self.pitch      = PITCH_NADIR
        self.yaw        = 0.0
        # Baslangic konumu
        self._send()

    def _send(self):
        mp, my = Double(), Double()
        mp.data = self.pitch
        my.data = self.yaw
        self._pub_pitch.publish(mp)
        self._pub_yaw.publish(my)

    def update(self, err_x: float, err_y: float, dt: float):
        """
        err_x: hedef yatay sapma [-1, +1]; pozitif → hedef sag → yaw artar
        err_y: hedef dikey sapma [-1, +1]; pozitif → hedef goruntu altinda
               → kamera daha asagi bakmali → pitch AZALIR (daha negatif)
        """
        self.yaw   = float(np.clip(self.yaw   + KP_GIMBAL_YAW   * err_x * dt, -YAW_MAX, YAW_MAX))
        self.pitch = float(np.clip(self.pitch - KP_GIMBAL_PITCH * err_y * dt,  PITCH_MIN, PITCH_MAX))
        self._send()

    def reset(self):
        self.pitch = PITCH_NADIR
        self.yaw   = 0.0
        self._send()

# ── Kalman Tracker ────────────────────────────────────────────────────────────

class KalmanTracker:
    def __init__(self, bbox):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.transitionMatrix  = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.processNoiseCov   = np.eye(4, dtype=np.float32) * 0.03
        x1, y1, x2, y2 = bbox
        cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
        self.kf.statePre  = np.array([[cx],[cy],[0],[0]], np.float32)
        self.kf.statePost = np.array([[cx],[cy],[0],[0]], np.float32)
        self.w = float(x2 - x1)
        self.h = float(y2 - y1)

    def predict(self):
        p = self.kf.predict()
        cx, cy = float(p[0]), float(p[1])
        return [int(cx - self.w/2), int(cy - self.h/2),
                int(cx + self.w/2), int(cy + self.h/2)]

    def correct(self, bbox):
        x1, y1, x2, y2 = bbox
        cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
        self.w, self.h = float(x2-x1), float(y2-y1)
        self.kf.correct(np.array([[np.float32(cx)],[np.float32(cy)]]))


def _iou(a, b):
    ix1, iy1 = max(a[0],b[0]), max(a[1],b[1])
    ix2, iy2 = min(a[2],b[2]), min(a[3],b[3])
    inter = max(0, ix2-ix1) * max(0, iy2-iy1)
    ua    = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0

# ── Drone (MAVLink) ───────────────────────────────────────────────────────────

class DroneController:
    def __init__(self, uri):
        self.conn = mavutil.mavlink_connection(uri)
        print("MAVLink heartbeat bekleniyor...")
        self.conn.wait_heartbeat(timeout=10)
        self.sys  = self.conn.target_system
        self.comp = self.conn.target_component
        print(f"Baglandi → system={self.sys}")

    # --- Komutlar ---

    def set_mode(self, mode: str):
        self.conn.set_mode(mode)

    def arm(self):
        self.conn.arducopter_arm()
        self.conn.motors_armed_wait()
        print("ARMED!")

    def takeoff(self, alt: float):
        self.conn.mav.command_long_send(
            self.sys, self.comp,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt
        )
        print(f"Kalkis → {alt}m")

    def arm_and_takeoff(self, alt: float):
        self.set_mode('GUIDED')
        time.sleep(0.5)
        self.arm()
        time.sleep(1.0)
        self.takeoff(alt)

    def send_body_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """Body-NED cati altinda hiz komutu gonder (m/s)."""
        vx = float(np.clip(vx, -VEL_MAX, VEL_MAX))
        vy = float(np.clip(vy, -VEL_MAX, VEL_MAX))
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.sys, self.comp,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,   # sadece hiz kullan
            0, 0, 0,              # pozisyon (yok sayilir)
            vx, vy, vz,
            0, 0, 0,              # ivme (yok sayilir)
            0, 0                  # yaw (yok sayilir)
        )

    def hover(self):
        """Oldugu yerde dur."""
        self.send_body_velocity(0, 0, 0)

    def get_relative_alt(self):
        msg = self.conn.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        return msg.relative_alt / 1000.0 if msg else None

# ── Ana Program ───────────────────────────────────────────────────────────────

def main():
    print("=== Drone Takip Sistemi Baslatiliyor ===")

    # Kamera
    print(f"Kamera topic: {CAM_TOPIC}")
    cam = CameraNode()

    # Gimbal → nadir
    print("Gimbal nadir konumuna getiriliyor...")
    gimbal = GimbalController()

    # YOLO
    print(f"YOLOv8 yukleniyor: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    # MAVLink
    drone = DroneController(MAVLINK_URI)

    # Durum
    tracker         = None
    locked          = False
    tracking_active = False
    frame_count     = 0
    prev_time       = time.time()
    last_vel_send   = 0.0   # Son velocity komutu zamani (rate limiting icin)

    WIN = 'Drone Takip Sistemi'
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)

    print("\n=== HAZIR ===")
    print("  t → Arm + Kalkis 30m")
    print("  l → Takibi Baslat/Durdur")
    print("  r → Tracker Sifirla")
    print("  h → Hover")
    print("  q → Cikis")

    while True:
        frame = cam.get()
        if frame is None:
            time.sleep(0.01)
            continue

        now = time.time()
        dt  = max(now - prev_time, 1e-4)
        prev_time = now
        disp = frame.copy()

        frame_count += 1
        run_yolo  = (frame_count % SKIP == 0)
        yolo_boxes = []

        # A. YOLO tespiti
        if run_yolo:
            results = model(frame, verbose=False, conf=CONF, classes=[TASIT_CLASS])
            for r in results:
                for box in r.boxes:
                    b = box.xyxy[0].cpu().numpy().astype(int).tolist()
                    yolo_boxes.append(b)
                    cv2.rectangle(disp, (b[0],b[1]), (b[2],b[3]), (255, 100, 0), 1)
                    cv2.putText(disp, f"{float(box.conf):.2f}",
                                (b[0], b[1]-4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,100,0), 1)

        # B. Kalman güncelleme
        if locked and tracker:
            pred = tracker.predict()

            if run_yolo and yolo_boxes:
                best_box, best_iou = None, -1.0
                for yb in yolo_boxes:
                    v = _iou(pred, yb)
                    if v > best_iou:
                        best_iou, best_box = v, yb
                if best_iou >= IOU_THRESH and best_box is not None:
                    tracker.correct(best_box)
                    pred = list(best_box)

            x1, y1, x2, y2 = pred
            x1 = max(0, x1); y1 = max(0, y1)
            x2 = min(FRAME_W, x2); y2 = min(FRAME_H, y2)
            cx, cy = (x1+x2)/2.0, (y1+y2)/2.0

            # Normallestirilmis hata [-1, +1]
            err_x = (cx - FRAME_W/2) / (FRAME_W/2)
            err_y = (cy - FRAME_H/2) / (FRAME_H/2)

            # C. Kontrol (takip aktifse)
            if tracking_active:
                # Dead zone: cok kucuk hatalar icin komut gonderme
                ex = err_x if abs(err_x) > DEAD_ZONE else 0.0
                ey = err_y if abs(err_y) > DEAD_ZONE else 0.0

                # Gimbal her frame guncellenir (hafif ve hizli)
                gimbal.update(ex, ey, dt)

                # Drone velocity → rate limit (10 Hz)
                if now - last_vel_send >= VEL_RATE:
                    last_vel_send = now
                    # err_y < 0 (hedef ustte/ufukta) → drone ileri (vx > 0)
                    # err_x > 0 (hedef sagda)         → drone saga (vy > 0)
                    vx = KP_VEL * (-ey)
                    vy = KP_VEL *   ex
                    drone.send_body_velocity(vx, vy)

            # Cizim
            cv2.rectangle(disp, (x1,y1), (x2,y2), (0, 255, 0), 3)
            cv2.circle(disp, (int(cx), int(cy)), 6, (0,255,0), -1)
            label = f"tasit | ex={err_x:+.2f} ey={err_y:+.2f}"
            cv2.putText(disp, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        elif not locked and run_yolo and yolo_boxes:
            # İlk tespit → kilitle
            best = max(yolo_boxes,
                       key=lambda b: (b[2]-b[0]) * (b[3]-b[1]))  # En buyuk bbox
            tracker = KalmanTracker(best)
            locked  = True
            print(f"HEDEF KILITLENDI: bbox={best}")

        # Crosshair
        cx0, cy0 = FRAME_W//2, FRAME_H//2
        cv2.line(disp, (cx0-25, cy0), (cx0+25, cy0), (0,255,255), 1)
        cv2.line(disp, (cx0, cy0-25), (cx0, cy0+25), (0,255,255), 1)

        # Durum bilgisi
        fps    = 1.0 / dt
        status = "TAKİP" if (locked and tracking_active) else \
                 "KİLİTLİ" if locked else "ARANIYOR"
        color  = (0,255,0) if tracking_active else (0,200,255)
        cv2.putText(disp,
                    f"FPS:{fps:.0f} | {status} | P:{gimbal.pitch:.2f} Y:{gimbal.yaw:.2f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        cv2.imshow(WIN, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('t'):
            print("Kalkis komutu gonderiliyor...")
            drone.arm_and_takeoff(TAKEOFF_ALT)
        elif key == ord('l'):
            tracking_active = not tracking_active
            if tracking_active and not locked:
                print("Henuz hedef yok, once tespit bekleniyor...")
                tracking_active = False
            else:
                print(f"Takip: {'AKTIF' if tracking_active else 'PASIF'}")
        elif key == ord('r'):
            tracker  = None
            locked   = False
            tracking_active = False
            gimbal.reset()
            drone.hover()
            print("Tracker sifirlanadi, gimbal nadir konumuna dondu.")
        elif key == ord('h'):
            drone.hover()
            tracking_active = False
            print("HOVER")

    cv2.destroyAllWindows()
    drone.hover()
    print("Cikis.")


if __name__ == '__main__':
    main()
