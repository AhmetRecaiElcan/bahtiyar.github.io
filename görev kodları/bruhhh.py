# scripts/task2_color_control.py
"""
Renk → Motor Haritası (Sadece Kırmızı ve Sarı)
----------------------------------------------
• Kırmızı  → sadece SAĞ motor ileri
• Sarı     → sadece SOL motor ileri
• Diğer    → du            if stable == 'yellow':
                # SARI → SOL motor çalışsın → steer sağa çevir (sağ motor yavaşlar/durur)
                steer = PWM_STOP + S_rel              
            else:  # red
                # KIRMIZI → SAĞ motor çalışsın → steer sola çevir (sol motor yavaşlar/durur)
                steer = PWM_STOP - S_relkış: pencere odaklı iken 'q' veya terminalde Ctrl-C.
Not: Pixhawk MANUAL modda ve arm edilmiş olmalı.
"""

import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode

# —————————————————— Kullanıcı ayarları ——————————————————
CONNECTION = 'COM18'     # telemetri kullanacaksan buraya 'COM5' ya da 'udp:127.0.0.1:14550' yaz
BAUD       = 57600       # Test sonucuna göre 57600 çalışıyor
CAM_INDEX  = 0          # kamera index
SHOW_WIN   = True
CONNECTION_TIMEOUT = 60  # Bağlantı timeout süresini artır

# PWM değerleri
PWM_STOP   = 1500
PWM_FWD    = 1600       # tek motor ileri kullanacağımız temel hız
PWM_MAX    = 2000       # siyah için tam gaz

# Motor karışım oranı (dönüş keskinliği)
MANUAL_MIX = 0.5        # 0.1=keskin, 0.5=orta, 1.0=yumuşak

# Renk karar eşiği
MIN_RATIO  = 0.10       # ROI’nin en az %10’u aynı renkten olmalı
MARGIN     = 0.05       # kazanan - 2.ci arasındaki fark en az %5 olmalı
SMOOTH_N   = 5          # son N karara çoğunluk oyu
DEADMAN_T  = 0.6        # (sn) bu sürede frame/karar yoksa dur

# ROI (çerçevenin ortası)
ROI_SCALE  = 0.6        # genişlik ve yükseklik için %60’lık orta bölge

# —————————————————— Bağlantı ——————————————————
print("▶ Pixhawk'a bağlanılıyor …")
try:
    # İlk önce wait_ready=False ile bağlan
    vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False, timeout=CONNECTION_TIMEOUT)
    print("▶ Bağlantı kuruldu, araç hazırlanıyor...")
    
    # Manuel olarak hazırlık durumunu kontrol et
    print("▶ Sistem durumu kontrol ediliyor...")
    start_time = time.time()
    
    while time.time() - start_time < CONNECTION_TIMEOUT:
        try:
            if vehicle.system_status and vehicle.mode:
                print(f"▶ Sistem durumu: {vehicle.system_status}")
                print(f"▶ Mevcut mod: {vehicle.mode}")
                break
        except Exception as e:
            print(f"▶ Bekleniyor... ({int(time.time() - start_time)}s)")
            time.sleep(1)
    else:
        print("WARNING: Araç tam olarak hazır değil ama devam ediliyor...")
        
except Exception as e:
    print(f"HATA: Bağlantı kurulamadı - {e}")
    print("Çözüm önerileri:")
    print("1. COM18 portunun doğru olduğundan emin olun")
    print("2. Telemetri radyonun bağlı ve çalıştığından emin olun") 
    print("3. Mission Planner'da telemetri çalışıyor mu kontrol edin")
    print("4. Baud rate'i 57600'e değiştirmeyi deneyin")
    exit(1)

# Mod ayarla ve arm et
print("▶ MANUAL moda geçiliyor...")
vehicle.mode = VehicleMode("MANUAL")

# Mod değişimini bekle
mode_start = time.time()
while vehicle.mode.name != "MANUAL" and time.time() - mode_start < 10:
    print(f"▶ Mod değiştiriliyor... Şu anki mod: {vehicle.mode.name}")
    time.sleep(0.5)

print(f"▶ Mevcut mod: {vehicle.mode.name}")

# Arm etmeyi dene
print("▶ Araç arm ediliyor...")
arm_attempts = 0
max_arm_attempts = 10

while not vehicle.armed and arm_attempts < max_arm_attempts:
    try:
        vehicle.armed = True
        arm_attempts += 1
        time.sleep(0.5)
        if not vehicle.armed:
            print(f"▶ Arm denemesi {arm_attempts}/{max_arm_attempts}...")
    except Exception as e:
        print(f"▶ Arm hatası: {e}")
        time.sleep(1)

if vehicle.armed:
    print("ARMED ✔")
else:
    print("WARNING: Araç arm edilemedi ama devam ediliyor...")
    print("Manuel olarak Mission Planner'dan arm etmeyi deneyin")

# ArduRover miks parametresi (tek motoru durdurma hesabı için gerekli)
try:
    mix = float(vehicle.parameters.get('MOT_STR_THR_MIX', MANUAL_MIX))
    if not (0.1 <= mix <= 1.0):
        mix = MANUAL_MIX
except Exception:
    mix = MANUAL_MIX
print(f"▶ MOT_STR_THR_MIX = {mix:.2f} (Manuel: {MANUAL_MIX})")

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

        # ——— Maske tanımları (sadece kırmızı ve sarı) ———
        # Kırmızı (iki aralık), doygunluk ve parlaklık şartlı
        red1 = cv2.inRange(hsv, (0, 120, 60),  (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 120, 60), (180,255, 255))
        mask_red = cv2.bitwise_or(red1, red2)

        # Sarı (yüksek S ve orta-yüksek V)
        mask_yel = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))

        # morfolojik temizlik
        k = np.ones((5,5), np.uint8)
        masks = {
            'red':   cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, k, iterations=1),
            'yellow':cv2.morphologyEx(mask_yel, cv2.MORPH_OPEN, k, iterations=1),
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

        # ——— Motor komutu ———
        thr = PWM_STOP
        steer = PWM_STOP

        if stable in ('red', 'yellow'):
            # Tek motor çalıştır: miks formülü ile diğer tarafı 1500’e getir
            T = PWM_FWD
            T_rel = T - PWM_STOP           # ör. +100
            if T_rel < 0:
                T_rel = 0
            # sağ motoru durdurmak için steer sağa; sol motoru durdurmak için steer sola
            if mix <= 0:
                mix = 0.5
            S_rel = int(round(T_rel / mix))           # PWM ofseti
            if stable == 'yellow':
                steer = PWM_STOP - S_rel              # sola çevir → sol yavaşlar, sağ durur
            else:  # red
                steer = PWM_STOP + S_rel              # sağa çevir → sağ yavaşlar, sağ durur?  (tersine!)
                # DİKKAT: ArduRover’da steer sağ (+) iken sağ taraf yavaşlar.
                # Eğer ters gözlüyorsan steer işaretini değiştir:
                # steer = PWM_STOP - S_rel

            # Limitler
            steer = max(1100, min(1900, steer))
            thr   = T

        else:
            thr, steer = PWM_STOP, PWM_STOP

        send_rc(thr, steer)
        last_cmd_time = time.time()

        # ——— Görsel debug ———
        if SHOW_WIN:
            dbg = roi.copy()
            # metin
            txt1 = f"top:{top_color} {top_ratio*100:.1f}%  2nd:{second_ratio*100:.1f}%"
            txt2 = f"stable:{stable.upper()}  THR:{thr}  STR:{steer}"
            cv2.rectangle(frame, (x0, y0), (x0+rw, y0+rh), (0,255,0), 2)
            cv2.putText(frame, txt1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)
            cv2.putText(frame, txt2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,255,0),2)

            # küçük mask önizleme (sadece kırmızı ve sarı)
            small = 160
            vis = np.zeros((small, small*2, 3), np.uint8)  # 1x2 grid
            def put_mask(m, pos, label):
                m3 = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                m3 = cv2.resize(m3, (small, small))
                y, x = pos
                vis[y:y+small, x:x+small] = m3
                cv2.putText(vis, label, (x+5, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(255,255,255),1)
            put_mask(masks['red'],    (0,0),     f"RED {ratios['red']*100:.1f}%")
            put_mask(masks['yellow'], (0,small), f"YEL {ratios['yellow']*100:.1f}%")

            # yerleştir
            H = max(frame.shape[0], vis.shape[0]+10)
            W = frame.shape[1] + vis.shape[1] + 10
            canvas = np.zeros((H, W, 3), dtype=np.uint8)
            canvas[:frame.shape[0], :frame.shape[1]] = frame
            canvas[:vis.shape[0], frame.shape[1]+10:frame.shape[1]+10+vis.shape[1]] = vis
            cv2.imshow('Color Control (ROI+Masks)', canvas)

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
