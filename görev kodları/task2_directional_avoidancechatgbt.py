import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode

# —————————————————— Kullanıcı ayarları ——————————————————
CONNECTION = 'COM18'    # Telemetri bağlantı adresin
BAUD       = 57600
CAM_INDEX  = 0
SHOW_WIN   = True

# PWM değerleri
PWM_STOP   = 1500
PWM_SLOW   = 1550
PWM_FWD    = 1600
PWM_LEFT   = 1400
PWM_RIGHT  = 1600

# Renk karar eşiği
MIN_RATIO  = 0.05
ROI_SCALE  = 0.75

# —————————————————— Pixhawk Bağlantı ——————————————————
print("▶ Pixhawk'a bağlanılıyor …")
vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False)
time.sleep(2)
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.armed:
    vehicle.armed = True
    time.sleep(0.5)
print("ARMED ✔")

def send_rc(throttle_pwm: int, steer_pwm: int):
    vehicle.channels.overrides = {'3': int(throttle_pwm), '1': int(steer_pwm)}

def stop_all():
    send_rc(PWM_STOP, PWM_STOP)

stop_all()

# —————————————————— Kamera ——————————————————
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Kamera açılamadı.")
print("▶ Kamera aktif – q / Ctrl-C ile çıkış\n")

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            stop_all()
            continue

        h, w = frame.shape[:2]
        rw, rh = int(w * ROI_SCALE), int(h * ROI_SCALE)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        roi = frame[y0:y0+rh, x0:x0+rw].copy()

        roi_blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2LAB)
        L = lab[:,:,0]
        h_, s_, v_ = cv2.split(hsv)
        v_eq = clahe.apply(v_)
        hsv = cv2.merge([h_, s_, v_eq])

        mask_yel = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))
        k = np.ones((5,5), np.uint8)
        mask_yel = cv2.morphologyEx(mask_yel, cv2.MORPH_OPEN, k, iterations=1)

        ratio = float(np.count_nonzero(mask_yel)) / (roi.shape[0] * roi.shape[1])

        if ratio > MIN_RATIO:
            contours, _ = cv2.findContours(mask_yel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                (x, y, w_box, h_box) = cv2.boundingRect(largest)
                center_x = x + w_box // 2

                frame_width = roi.shape[1]
                if center_x > frame_width // 2:
                    thr, steer = PWM_SLOW, PWM_LEFT
                    print("⚠️ Engel SAĞDA → Sola dönülüyor")
                else:
                    thr, steer = PWM_SLOW, PWM_RIGHT
                    print("⚠️ Engel SOLDa → Sağa dönülüyor")
            else:
                thr, steer = PWM_STOP, PWM_STOP
        else:
            thr, steer = PWM_FWD, PWM_STOP
            print("✅ Engel yok → İleri")

        send_rc(thr, steer)

        if SHOW_WIN:
            cv2.putText(frame, f"YELLOW: {ratio*100:.1f}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.rectangle(frame, (x0, y0), (x0+rw, y0+rh), (0,255,0), 2)
            cv2.imshow("Yönlü Engel Kaçınma", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        time.sleep(0.03)

except KeyboardInterrupt:
    pass
finally:
    print("\n▶ Override temizleniyor ve disarm …")
    vehicle.channels.overrides = {}
    try:
        vehicle.armed = False
    except Exception:
        pass
    vehicle.close()
    cap.release()
    if SHOW_WIN:
        cv2.destroyAllWindows()
    print("✅ Görev sonlandı.")
