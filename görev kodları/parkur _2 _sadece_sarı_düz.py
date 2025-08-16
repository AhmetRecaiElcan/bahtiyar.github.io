# scripts/task2_color_control.py
"""
Renk → Motor Haritası (Tek Motor / Çift Motor)
----------------------------------------------
• Kırmızı  → sadece SOL motor ileri
• Sarı     → sadece SAĞ motor ileri
• Siyah    → iki motor tam gaz ileri
• Beyaz    → dur
• Diğer    → dur

Çıkış: pencere odaklı iken 'q' veya terminalde Ctrl-C.
Not: Pixhawk MANUAL modda ve arm edilmiş olmalı.
"""

import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode

# —————————————————— Kullanıcı ayarları ——————————————————
CONNECTION = 'COM18'    # telemetri kullanacaksan buraya 'COM18' ya da 'udp:127.0.0.1:14550' yaz
BAUD       = 57600
CAM_INDEX  = 0          # kamera index
SHOW_WIN   = True

# PWM değerleri
PWM_STOP   = 1500
PWM_FWD    = 1600       # tek motor ileri kullanacağımız temel hız
PWM_MAX    = 2000       # siyah için tam gaz

# Renk karar eşiği
MIN_RATIO  = 0.05       # ROI’nin en az %10’u aynı renkten olmalı
MARGIN     = 0.05       # kazanan - 2.ci arasındaki fark en az %5 olmalı
SMOOTH_N   = 5          # son N karara çoğunluk oyu
DEADMAN_T  = 0.6        # (sn) bu sürede frame/karar yoksa dur

# ROI (çerçevenin ortası)
ROI_SCALE  = 0.75        # genişlik ve yükseklik için %60’lık orta bölge

# —————————————————— Bağlantı ——————————————————
print("▶ Pixhawk'a telemetri ile bağlanılıyor (wait_ready=False)...")
vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False)

time.sleep(2)  # bağlantı oturması için bekleme

vehicle.mode = VehicleMode("MANUAL")
while not vehicle.armed:
    vehicle.armed = True
    time.sleep(0.2)
print("ARMED ✔")


# ArduRover miks parametresi (tek motoru durdurma hesabı için gerekli)
try:
    mix = float(vehicle.parameters.get('MOT_STR_THR_MIX', 0.5))
    if not (0.1 <= mix <= 1.0):
        mix = 0.5
except Exception:
    mix = 0.5
print(f"▶ MOT_STR_THR_MIX = {mix:.2f}")

# RC override helper
def send_rc(throttle_pwm: int, steer_pwm: int):
    vehicle.channels.overrides = {'3': int(throttle_pwm), '1': int(steer_pwm)}

def stop_all():
    send_rc(PWM_STOP, PWM_STOP)

# Başlangıç dur
stop_all()

# —————————————————— Kamera ——————————————————
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Kamera açılamadı.")
print("▶ Kamera aktif – q / Ctrl-C ile çıkış\n")

# CLAHE: aydınlık dengesi (V kanalında)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

# Temporal smoothing hafızası
last_choices = []
last_cmd_time = time.time()

def percent(mask, roi_pixels):
    return float(np.count_nonzero(mask)) / float(roi_pixels + 1e-6)

def majority_vote(history, default='none'):
    if not history:
        return default
    vals, counts = np.unique(history, return_counts=True)
    return vals[np.argmax(counts)]

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            # kamera kaçar ise dur
            if time.time() - last_cmd_time > DEADMAN_T:
                stop_all()
            continue

        # ROI kırp
        h, w = frame.shape[:2]
        rw, rh = int(w * ROI_SCALE), int(h * ROI_SCALE)
        x0 = (w - rw) // 2
        y0 = (h - rh) // 2
        roi = frame[y0:y0+rh, x0:x0+rw].copy()

        # hafif blur + HSV
        roi_blur = cv2.GaussianBlur(roi, (5,5), 0)
        hsv = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HSV)

        # V kanalına CLAHE (ışık dengesizliğini toparlar)
        h_, s_, v_ = cv2.split(hsv)
        v_eq = clahe.apply(v_)
        hsv = cv2.merge([h_, s_, v_eq])

        # LAB ile parlaklık bilgisi (beyaz/siyah ayrımı için)
        lab = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2LAB)
        L = lab[:,:,0]

        # ——— Maske tanımları ———
        # Kırmızı (iki aralık), doygunluk ve parlaklık şartlı
        red1 = cv2.inRange(hsv, (0, 120, 60),  (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 120, 60), (180,255, 255))
        mask_red = cv2.bitwise_or(red1, red2)

        # Sarı (yüksek S ve orta-yüksek V)
        mask_yel = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))

        # Beyaz: düşük S + yüksek V + yüksek L
        mask_wht = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))
        mask_wht &= cv2.inRange(L, 200, 255)

        # Siyah: düşük V VE düşük L (parlamaları dışlar)
        mask_blk_v = cv2.inRange(hsv[:,:,2], 0, 60)
        mask_blk_L = cv2.inRange(L, 0, 50)
        mask_blk = cv2.bitwise_and(mask_blk_v, mask_blk_L)

        # morfolojik temizlik
        k = np.ones((5,5), np.uint8)
        masks = {
            'red':   cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, k, iterations=1),
            'yellow':cv2.morphologyEx(mask_yel, cv2.MORPH_OPEN, k, iterations=1),
            'white': cv2.morphologyEx(mask_wht, cv2.MORPH_OPEN, k, iterations=1),
            'black': cv2.morphologyEx(mask_blk, cv2.MORPH_OPEN, k, iterations=1)
        }

        # oranlar
        total_px = roi.shape[0] * roi.shape[1]
        ratios = {c: percent(m, total_px) for c, m in masks.items()}

        # en baskın rengi seç
        sorted_colors = sorted(ratios.items(), key=lambda x: x[1], reverse=True)
        top_color, top_ratio = sorted_colors[0]
        second_ratio = sorted_colors[1][1] if len(sorted_colors) > 1 else 0.0

        chosen = 'none'
        if top_ratio >= MIN_RATIO and (top_ratio - second_ratio) >= MARGIN:
            chosen = top_color

        # temporal smoothing
        last_choices.append(chosen)
        if len(last_choices) > SMOOTH_N:
            last_choices.pop(0)
        stable = majority_vote(last_choices, default='none')


        # --- Sarı oranı MIN_RATIO üstündeyse iki motor tam gaz ileri, değilse dur ---
        if ratios['yellow'] >= MIN_RATIO:
            thr, steer = PWM_MAX, PWM_STOP  # iki motor tam gaz ileri
        else:
            thr, steer = PWM_STOP, PWM_STOP

        send_rc(thr, steer)
        last_cmd_time = time.time()

        # ——— Görsel debug ———
        if SHOW_WIN:
            dbg = roi.copy()

            # metin: sadece sarı renk yüzdesi ve komut bilgisi
            txt1 = f"YELLOW: {ratios['yellow']*100:.1f}%"
            txt2 = f"stable:{stable.upper()}  THR:{thr}  STR:{steer}"
            cv2.rectangle(frame, (x0, y0), (x0+rw, y0+rh), (0,255,0), 2)
            cv2.putText(frame, txt1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)
            cv2.putText(frame, txt2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)

            # sadece sarı mask önizlemesi
            small = 160
            vis = np.zeros((small, small, 3), np.uint8)
            m3 = cv2.cvtColor(masks['yellow'], cv2.COLOR_GRAY2BGR)
            m3 = cv2.resize(m3, (small, small))
            vis[:,:] = m3
            cv2.putText(vis, f"YEL {ratios['yellow']*100:.1f}%", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255),1)

            # yerleştir
            H = max(frame.shape[0], vis.shape[0]+10)
            W = frame.shape[1] + vis.shape[1] + 10
            canvas = np.zeros((H, W, 3), dtype=np.uint8)
            canvas[:frame.shape[0], :frame.shape[1]] = frame
            canvas[:vis.shape[0], frame.shape[1]+10:frame.shape[1]+10+vis.shape[1]] = vis
            cv2.imshow('Color Control (YELLOW ONLY)', canvas)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # deadman
        if time.time() - last_cmd_time > DEADMAN_T:
            stop_all()

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
    print("✅ Renk kontrolü sonlandı.")
