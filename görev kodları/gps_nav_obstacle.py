"""
GPS Hedefe Gitme + Basit Engel Kaçınma (RC Override ile)
--------------------------------------------------------
Bu betik, mevcut RC/PWM motor sürüş yapısını kullanarak verilen bir GPS
hedefine doğru gitmeye çalışır. Ön taraftaki orta-alt bölgede kamera ile
basit bir engel algılama yapılır. Engel algılanırsa kısa bir kaçınma manevrası
uygulanır (sağa kır – ileri – sola kır) ve ardından hedefe gitmeye devam edilir.

Notlar:
- ArduRover kullanıldığı ve RC override ile sürüş yapıldığı varsayılmıştır.
- Mod olarak MANUAL tercih edilmiştir; ArduPilot miksajı kullanılır.
- Kameradan basit Canny kenar/pürüzlülük tabanlı bir engel skoru elde edilir.
- Bu, temel bir örnektir; gerçek sahada eşiklerin ayarı ve manevra süreleri
  test/kalibrasyon gerektirir.
"""

import time
import math
from typing import Tuple

import cv2
import numpy as np
from dronekit import connect, VehicleMode
import argparse


# —————————————————— Kullanıcı ayarları ——————————————————
CONNECTION = 'COM17'        # Örn: 'COM5' ya da 'udp:127.0.0.1:14550'
BAUD       = 115200
CAM_INDEX  = 0
SHOW_WIN   = True
CONNECTION_TIMEOUT = 60

# Hedef koordinat (Derece)
# ÖRNEK: 40.7710894, 29.9784
TARGET_LAT = 40.7710894
TARGET_LON = 29.9784

# PWM değerleri
PWM_STOP   = 1500
PWM_CRUISE = 1580          # normal seyir gazı (düşük hız)
PWM_AVOID  = 1560          # kaçınma sırasında daha düşük hız
PWM_MAX    = 2000

# Direksiyon kazancı ve limitler
STEER_GAIN_DEG_PER_PWM = 1.2  # 1 PWM ~ 1.2 derece karşılık gelecek şekilde kaba ayar
STEER_MIN = 1100
STEER_MAX = 1900

# Varış yarıçapı (metre)
ARRIVAL_RADIUS_M = 3.0

# Engelden kaçınma manevra ayarları (saniye)
AVOID_TURN_RIGHT_S = 1.0
AVOID_STRAIGHT_S   = 0.7
AVOID_TURN_LEFT_S  = 1.0
AVOID_STEER_OFFSET = 180     # PWM_STOP ± bu kadar ofset ile keskin dönüş

# Kamera tabanlı engel algılama (basit)
OBS_ROI_W_RATIO = 0.5        # orta genişlik oranı
OBS_ROI_H_RATIO = 0.5        # alt yükseklik oranı
OBS_CANNY1      = 60
OBS_CANNY2      = 180
OBS_MIN_EDGE_RATIO = 0.06    # ROI'de kenar piksel oranı eşiği

# Deadman
DEADMAN_T = 0.7


# —————————————————— Yardımcı fonksiyonlar ——————————————————
def send_rc(vehicle, throttle_pwm: int, steer_pwm: int) -> None:
    vehicle.channels.overrides = {'3': int(throttle_pwm), '1': int(steer_pwm)}


def stop_all(vehicle) -> None:
    send_rc(vehicle, PWM_STOP, PWM_STOP)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb/2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlmb = math.radians(lon2 - lon1)
    y = math.sin(dlmb) * math.cos(p2)
    x = math.cos(p1)*math.sin(p2) - math.sin(p1)*math.cos(p2)*math.cos(dlmb)
    brng = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return brng


def shortest_angle_error_deg(target: float, current: float) -> float:
    # Sonuç: [-180, 180]
    return ((target - current + 540.0) % 360.0) - 180.0


def get_heading_deg(vehicle) -> float:
    # ArduPilot Rover genelde vehicle.heading (0-360) sağlar.
    try:
        hdg = float(vehicle.heading)
        if 0.0 <= hdg < 360.0:
            return hdg
    except Exception:
        pass
    # Alternatif: yaw (radyan, -pi..pi)
    try:
        yaw = float(vehicle.attitude.yaw)
        hdg = (math.degrees(yaw) + 360.0) % 360.0
        return hdg
    except Exception:
        return 0.0


def detect_obstacle(frame: np.ndarray) -> Tuple[bool, float]:
    if frame is None or frame.size == 0:
        return False, 0.0
    h, w = frame.shape[:2]
    rw = int(w * OBS_ROI_W_RATIO)
    rh = int(h * OBS_ROI_H_RATIO)
    x0 = (w - rw) // 2
    y0 = h - rh
    roi = frame[y0:y0+rh, x0:x0+rw]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, OBS_CANNY1, OBS_CANNY2)
    edge_ratio = float(np.count_nonzero(edges)) / float(edges.size + 1e-6)
    return (edge_ratio >= OBS_MIN_EDGE_RATIO), edge_ratio


def do_avoid_maneuver(vehicle, update_vis_cb=None) -> None:
    # Kısa, deterministik bir kaçınma: sağa kır -> düz -> sola kır
    # Bu fonksiyon, süre boyunca küçük adımlarla RC komutlarını gönderir.
    def drive_for(duration_s: float, thr: int, steer: int):
        t0 = time.time()
        while time.time() - t0 < duration_s:
            send_rc(vehicle, thr, steer)
            if update_vis_cb is not None:
                update_vis_cb()
            time.sleep(0.03)

    # 0) Kısa dur
    drive_for(0.2, PWM_STOP, PWM_STOP)
    # 1) Sağa kırarak ileri
    steer_right = max(STEER_MIN, min(STEER_MAX, PWM_STOP + AVOID_STEER_OFFSET))
    drive_for(AVOID_TURN_RIGHT_S, PWM_AVOID, steer_right)
    # 2) Düz geçiş
    drive_for(AVOID_STRAIGHT_S, PWM_AVOID, PWM_STOP)
    # 3) Sola kırarak geri rotaya dön
    steer_left = max(STEER_MIN, min(STEER_MAX, PWM_STOP - AVOID_STEER_OFFSET))
    drive_for(AVOID_TURN_LEFT_S, PWM_AVOID, steer_left)
    # 4) Kısa düzleştirme
    drive_for(0.2, PWM_AVOID, PWM_STOP)


def steer_from_heading_error_deg(err_deg: float) -> int:
    # Hata pozitifse hedef sağda demektir -> direksiyonu sağa çevir (PWM artar)
    pwm_offset = int(round(err_deg / STEER_GAIN_DEG_PER_PWM))
    steer = PWM_STOP + pwm_offset
    steer = max(STEER_MIN, min(STEER_MAX, steer))
    return steer


# —————————————————— Bağlantı ve hazırlık ——————————————————
# —————————————————— Argümanlar ——————————————————
parser = argparse.ArgumentParser(description='GPS hedefe git + engel kaçınma')
parser.add_argument('--lat', type=float, help='Hedef enlem (derece)')
parser.add_argument('--lon', type=float, help='Hedef boylam (derece)')
parser.add_argument('--show', action='store_true', help='Pencere göster (varsayılan SHOW_WIN ayarıyla birlikte)')
parser.add_argument('--no-show', action='store_true', help='Pencere gösterme')
parser.add_argument('--conn', type=str, default=None, help='Bağlantı dizesi (örn. COM5 veya udp:127.0.0.1:14550)')
parser.add_argument('--baud', type=int, default=None, help='Baud hızı (vars: 57600)')
args, _ = parser.parse_known_args()

if args.lat is not None and args.lon is not None:
    TARGET_LAT = float(args.lat)
    TARGET_LON = float(args.lon)

if args.show:
    SHOW_WIN = True
if args.no_show:
    SHOW_WIN = False
if args.conn:
    CONNECTION = args.conn
if args.baud:
    BAUD = int(args.baud)

print("▶ Pixhawk'a bağlanılıyor …")
try:
    vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False, timeout=CONNECTION_TIMEOUT)
    print("▶ Bağlantı kuruldu, araç hazırlanıyor...")
except Exception as e:
    print(f"HATA: Bağlantı kurulamadı - {e}")
    raise SystemExit(1)

print("▶ MANUAL moda geçiliyor…")
vehicle.mode = VehicleMode("MANUAL")
mode_start = time.time()
while vehicle.mode.name != "MANUAL" and time.time() - mode_start < 10:
    print(f"▶ Mod değiştiriliyor... Şu anki mod: {vehicle.mode.name}")
    time.sleep(0.5)
print(f"▶ Mevcut mod: {vehicle.mode.name}")

print("▶ Araç arm ediliyor…")
arm_attempts = 0
while not vehicle.armed and arm_attempts < 10:
    try:
        vehicle.armed = True
        arm_attempts += 1
        time.sleep(0.5)
    except Exception as e:
        print(f"▶ Arm hatası: {e}")
        time.sleep(0.5)

if vehicle.armed:
    print("ARMED ✔")
else:
    print("WARNING: Araç arm edilemedi, manuel arm deneyin.")


# Kamera
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Kamera açılamadı.")
print("▶ Kamera aktif – q / Ctrl-C ile çıkış")


last_cmd_time = time.time()


def update_window(vis_img=None):
    if SHOW_WIN and vis_img is not None:
        cv2.imshow('GPS NAV + AVOID', vis_img)
        cv2.waitKey(1)


try:
    while True:
        ok, frame = cap.read()
        if not ok:
            if time.time() - last_cmd_time > DEADMAN_T:
                stop_all(vehicle)
            time.sleep(0.03)
            continue

        loc = vehicle.location.global_frame
        gps = vehicle.gps_0
        have_fix = getattr(gps, 'fix_type', 0) >= 3 and loc and loc.lat is not None and loc.lon is not None

        # Görsel engel tespiti
        obstacle, edge_ratio = detect_obstacle(frame)

        # Görselleştirme tuvali
        vis = frame.copy()
        h, w = frame.shape[:2]
        # Engel ROI kutusu
        rw = int(w * OBS_ROI_W_RATIO)
        rh = int(h * OBS_ROI_H_RATIO)
        x0 = (w - rw) // 2
        y0 = h - rh
        color_roi = (0, 0, 255) if obstacle else (0, 255, 0)
        cv2.rectangle(vis, (x0, y0), (x0+rw, y0+rh), color_roi, 2)

        if not have_fix:
            cv2.putText(vis, 'GPS yok/fix yetersiz -> DUR', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            stop_all(vehicle)
            update_window(vis)
            time.sleep(0.03)
            continue

        cur_lat, cur_lon = float(loc.lat), float(loc.lon)
        dist_m = haversine_m(cur_lat, cur_lon, TARGET_LAT, TARGET_LON)

        if dist_m <= ARRIVAL_RADIUS_M:
            cv2.putText(vis, f'Varildi ({dist_m:.1f} m) -> DUR', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            stop_all(vehicle)
            update_window(vis)
            break

        desired_bearing = bearing_deg(cur_lat, cur_lon, TARGET_LAT, TARGET_LON)
        current_heading = get_heading_deg(vehicle)
        err_deg = shortest_angle_error_deg(desired_bearing, current_heading)

        thr = PWM_CRUISE
        steer = steer_from_heading_error_deg(err_deg)

        # Engel varsa kaçınma manevrası
        if obstacle:
            cv2.putText(vis, f'ENGEL! edge_ratio={edge_ratio:.3f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            update_window(vis)
            do_avoid_maneuver(vehicle, update_vis_cb=lambda: update_window(vis))
            last_cmd_time = time.time()
            # Manevra sonrası bir sonraki döngüde yeniden yönelme yapılacak
        else:
            # Normal GPS takip sürüşü
            send_rc(vehicle, thr, steer)
            last_cmd_time = time.time()
            cv2.putText(vis, f'Dist:{dist_m:.1f}m Brg:{desired_bearing:.0f} hdg:{current_heading:.0f} err:{err_deg:.0f}',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(vis, f'THR:{thr} STR:{steer}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        update_window(vis)

        # Deadman
        if time.time() - last_cmd_time > DEADMAN_T:
            stop_all(vehicle)

        # Çıkış
        if SHOW_WIN:
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
    print("✅ GPS navigasyon betiği sonlandı.")


