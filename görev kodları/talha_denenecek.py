# scripts/task2_color_lidar_control.py
# Kamera (renk) + LiDAR (güvenlik) → Pixhawk RC override (Rover)
# Kullanım:
#   python scripts\task2_color_lidar_control.py [PIX_PORT] [LIDAR_PORT]
# Örnek:
#   python scripts\task2_color_lidar_control.py COM4 COM6

from _future_ import annotations
import sys, time, math, threading, collections, collections.abc
from collections import deque, Counter

# --- DroneKit Python 3.10+ uyumluluk yaması (DRONEKIT'TEN ÖNCE!) ---
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping
if not hasattr(collections, "Mapping"):
    collections.Mapping = collections.abc.Mapping
if not hasattr(collections, "MutableSet"):
    collections.MutableSet = collections.abc.MutableSet

import cv2
import numpy as np
from dronekit import connect, VehicleMode

# RPLIDAR (A1M8) için
# pip install rplidar-roboticia   # (veya) pip install rplidar
from rplidar import RPLidar, RPLidarException

# ================== AYARLAR ==================
# Pixhawk bağ.
PIX_PORT   = 'COM18'
PIX_BAUD   = 57600

# LiDAR bağ.
LIDAR_PORT = sys.argv[2] if len(sys.argv) >= 3 else 'COM6'

# RC kanalları (ArduRover varsayılanları)
CH_STEER = '1'
CH_THR   = '3'

# PWM değerleri
THR_STOP  = 1500
THR_FWD   = 1600
THR_MAX   = 2000
STR_CENT  = 1500
STR_LEFT  = 1400
STR_RIGHT = 1600

# Renk → komut haritası (sadece sarı ve kırmızı)
CMD = {
    'red':    (THR_STOP, STR_CENT),   # kırmızıda dur
    'yellow': (THR_FWD, STR_RIGHT),    # sarıda sağa kaç (örnek)
}

# HSV aralıkları (aydınlık/karanlık değişimlerine göre esnek tutuldu)
RED_LO1,  RED_HI1  = (0,   100,  50), (10, 255,255)
RED_LO2,  RED_HI2  = (170, 100,  50), (180,255,255)
YEL_LO,   YEL_HI   = (20,  100,  60), (35, 255,255)
BLK_LO,   BLK_HI   = (0,     0,   0), (180, 60,  60)   # siyah: düşük V & S
WHT_LO,   WHT_HI   = (0,     0, 200), (180, 40, 255)   # beyaz: yüksek V, düşük S

AREA_THRESH = 1200          # piksel sayısı eşiği (gürültü filtresi)
SMOOTH_N    = 7             # renk kararında son N karenin çoğunluğu

# LiDAR güvenlik bölgesi
LIDAR_WEDGE_DEG  = 10       # önde ±10° yatay sektör
LIDAR_FRONT_OFFS = 0        # LiDAR 0°'si tekne ileri doğrultusuna göre kaç ° kayık?
STOP_RADIUS_M    = 0.50     # 0.5 m yarıçap (1 m çap)
MIN_VALID_MM     = 80       # çok yakındaki yansıma/gürültüyü at (mm)
MAX_VALID_MM     = 6000     # güvenli üst sınır (mm)

PRINT_EVERY = 0.5           # bilgi satırı periyodu (s)

# Hedef koordinatı (kullanıcı değiştirebilir)
TARGET_LAT = 41.0123
TARGET_LON = 28.9876

# Haversine ile mesafe (metre)
def distance_to_target(current_lat, current_lon, target_lat, target_lon):
    from math import radians, cos, sin, sqrt, atan2
    R = 6371000
    dlat = radians(target_lat - current_lat)
    dlon = radians(target_lon - current_lon)
    a = sin(dlat/2)**2 + cos(radians(current_lat)) * cos(radians(target_lat)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

def get_gps():
    loc = vehicle.location.global_frame
    return loc.lat, loc.lon

def get_heading():
    return vehicle.heading if hasattr(vehicle, 'heading') else 0

def go_to_gps_target(target_lat, target_lon, current_lat, current_lon, heading):
    # Basit yönlendirme: Hedefe doğru iki motor ileri
    send_rc(THR_FWD, STR_CENT)
    print(f"GPS ile hedefe yöneliniyor: {current_lat:.6f},{current_lon:.6f} -> {target_lat:.6f},{target_lon:.6f}")

# =============================================

# ---------- yardımcılar ----------
def open_vehicle(port: str, baud: int):
    print(f"▶ Pixhawk'a bağlanılıyor → {port} @ {baud} …")
    v = connect(port, baud=baud, wait_ready=True, heartbeat_timeout=30)
    v.mode = VehicleMode("MANUAL")
    while not v.armed:
        v.armed = True
        time.sleep(0.2)
    print("ARMED ✔\n")
    # başlangıç dur
    v.channels.overrides = {CH_THR: THR_STOP, CH_STEER: STR_CENT}
    return v

def choose_color(hsv: np.ndarray) -> str:
    # maske + morfoloji
    mask_red = cv2.inRange(hsv, RED_LO1, RED_HI1) | cv2.inRange(hsv, RED_LO2, RED_HI2)
    mask_yel = cv2.inRange(hsv, YEL_LO,  YEL_HI)

    kernel = np.ones((5,5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yel = cv2.morphologyEx(mask_yel, cv2.MORPH_OPEN, kernel)

    counts = {
        'red':    int(mask_red.sum() // 255),
        'yellow': int(mask_yel.sum() // 255),
    }
    color = max(counts, key=counts.get)
    if counts[color] < AREA_THRESH:
        return 'none'
    return color

def most_common(lst: deque[str]) -> str:
    if not lst:
        return 'none'
    return Counter(lst).most_common(1)[0][0]

def angle_in_wedge(a_deg: float, center: float, half_width: float) -> bool:
    # 0..360 dairesinde "center±half_width" aralığına düşüyor mu?
    # tüm açıları [0,360) normalize et
    a = (a_deg - center + 360.0) % 360.0
    return a <= half_width or a >= (360.0 - half_width)

# ---------- LiDAR okuma iş parçacığı ----------
class LidarGuard(threading.Thread):
    def _init_(self, port: str):
        super()._init_(daemon=True)
        self.port = port
        self._stop = threading.Event()
        self.blocked = False
        self.min_mm = None
        self.err = None

    def run(self):
        try:
            lidar = RPLidar(self.port)
            lidar.start_motor()
            time.sleep(0.2)
            center = (0 + LIDAR_FRONT_OFFS) % 360.0
            half   = LIDAR_WEDGE_DEG / 2.0
            for scan in lidar.iter_scans(max_buf_meas=600):
                if self._stop.is_set():
                    break
                # scan: list of (quality, angle, distance_mm)
                mm_list = []
                for q, ang, dist in scan:
                    if dist == 0:
                        continue
                    if dist < MIN_VALID_MM or dist > MAX_VALID_MM:
                        continue
                    if angle_in_wedge(ang, center, half):
                        mm_list.append(dist)
                if mm_list:
                    m = min(mm_list)
                    self.min_mm = m
                    self.blocked = (m <= int(STOP_RADIUS_M * 1000.0))
                else:
                    self.min_mm = None
                    self.blocked = False
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
        except Exception as e:
            self.err = e
            self.blocked = False
            self.min_mm = None

    def stop(self):
        self._stop.set()

# Lidar ayarları ve başlatma
LIDAR_PORT = 'COM6'  # Gerekirse değiştir
LIDAR_DIST_THRESHOLD = 1200  # mm, bu mesafeden yakınsa engel var kabul et
from rplidar import RPLidar, RPLidarException
lidar = RPLidar(LIDAR_PORT)
lidar_scan = []
lidar_lock = threading.Lock()

def lidar_thread():
    global lidar_scan
    try:
        for scan in lidar.iter_scans():
            with lidar_lock:
                lidar_scan = scan
    except RPLidarException as e:
        print(f"Lidar hatası: {e}")

# Lidar thread başlat
lt = threading.Thread(target=lidar_thread, daemon=True)
lt.start()

def is_lidar_obstacle(threshold=LIDAR_DIST_THRESHOLD, angle_range=60):
    # Ön 60 derece içinde threshold'dan yakın engel var mı?
    with lidar_lock:
        scan = list(lidar_scan)
    for (_, angle, dist) in scan:
        if (angle > 330 or angle < 30) and 0 < dist < threshold:
            return True
    return False

# -------------- Ana --------------
def main():
    vehicle = open_vehicle(PIX_PORT, PIX_BAUD)

    # LiDAR başlat
    print(f"▶ LiDAR açılıyor → {LIDAR_PORT}  (ön sektör ±{LIDAR_WEDGE_DEG/2}°, dur yarıçap {STOP_RADIUS_M} m)")
    guard = LidarGuard(LIDAR_PORT)
    guard.start()
    time.sleep(0.5)
    if guard.err:
        print(f"⚠ LiDAR açılamadı: {guard.err}")

    # Kamera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Kamera açılamadı.")
    print("▶ Kamera aktif – çıkış: pencere odaklıyken 'q' veya Ctrl+C\n")

    history = deque(maxlen=SMOOTH_N)
    last_print = 0.0
    last_cmd_time = 0.0
    DEADMAN_T = 3.0            # 3 saniye içinde komut gelmezse dur

    try:
        gps_active = True  # Koordinata gitme aktif mi?
        while True:
            ok, frame = cap.read()
            if not ok:
                if time.time() - last_cmd_time > DEADMAN_T:
                    stop_all()
                continue

            # GPS verisi oku
            current_lat, current_lon = get_gps()
            heading = get_heading()

            # ROI kırp
            h, w = frame.shape[:2]
            rw, rh = int(w * ROI_SCALE), int(h * ROI_SCALE)
            x0 = (w - rw) // 2
            y0 = (h - rh) // 2
            roi = frame[y0:y0+rh, x0:x0+rw].copy()

            # Otomatik parıltı dengesi (normalize) → renk daha istikrarlı
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, s, v = cv2.split(hsv)
            v = cv2.equalizeHist(v)
            hsv = cv2.merge([h, s, v])

            raw_color = choose_color(hsv)
            history.append(raw_color)
            color = most_common(history)

            # Renk komutunu hazırla
            thr, steer = CMD.get(color, (THR_STOP, STR_CENT))

            # LiDAR güvenliği uygula (engel varsa dur)
            blocked, mind = guard.blocked, guard.min_mm
            if blocked:
                thr = THR_STOP

            # RC override gönder
            vehicle.channels.overrides = {CH_THR: thr, CH_STEER: steer}

            # Görsel debug overlay
            dbg = frame.copy()
            txt = f"{color.upper():>6}  THR:{thr}  STR:{steer}"
            if mind is not None:
                txt += f"   LIDAR:{mind:.0f}mm {'BLOCK' if blocked else 'OK'}"
            cv2.putText(dbg, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if not blocked else (0, 0, 255), 2)

            # LiDAR sektörünü görsel çiz (ekrana)
            h_, w_ = dbg.shape[:2]
            cx, cy = w_//2, h_-10
            r_pix = 120
            ang0 = -90  # ekran için yukarıyı 0 kabul edip aşağıya doğru açı gibi
            half = LIDAR_WEDGE_DEG/2
            # basit yay çizimi
            for k in np.linspace(-half, half, 15):
                a = math.radians(ang0 + k)
                x = int(cx + r_pix*math.cos(a))
                y = int(cy + r_pix*math.sin(a))
                cv2.circle(dbg, (x,y), 1, (0,0,255) if blocked else (0,255,0), -1)

            cv2.imshow("Color + LiDAR", dbg)

            # periyodik terminal logu
            now = time.time()
            if now - last_print >= PRINT_EVERY:
                print(f"{color:>6}  → THR:{thr} STR:{steer}   "
                      f"LiDAR: {'—' if mind is None else str(int(mind))+'mm'} "
                      f"{'BLOCK' if blocked else 'OK'}")
                last_print = now

            # Sadece sarı ve kırmızı renk kontrolü
            if ratios['red'] >= MIN_RATIO:
                print("Kırmızı tespit edildi! Araç tamamen duruyor ve koordinata gitmeyi bırakıyor.")
                stop_all()
                gps_active = False
                time.sleep(1)
            elif ratios['yellow'] >= MIN_RATIO:
                print("Sarı engel tespit edildi! Kaçma manevrası başlatılıyor.")
                M = cv2.moments(masks['yellow'])
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    if cx < rw // 2:
                        print("Engel solda, sağa kaç!")
                        send_rc(THR_FWD, STR_RIGHT)  # sağa kaç
                    else:
                        print("Engel sağda, sola kaç!")
                        send_rc(THR_FWD, STR_LEFT)   # sola kaç
                else:
                    stop_all()
                time.sleep(1)
            elif gps_active:
                dist = distance_to_target(current_lat, current_lon, TARGET_LAT, TARGET_LON)
                print(f"Engel yok, hedefe ilerleniyor. Kalan mesafe: {dist:.1f} m")
                go_to_gps_target(TARGET_LAT, TARGET_LON, current_lat, current_lon, heading)
                time.sleep(0.5)
            else:
                stop_all()

            # Lidar ve kamera ile engel kontrolü
            lidar_engel = is_lidar_obstacle()
            sari_engel = ratios['yellow'] >= MIN_RATIO
            kirmizi = ratios['red'] >= MIN_RATIO

            if kirmizi:
                print("Kırmızı tespit edildi! Araç tamamen duruyor ve koordinata gitmeyi bırakıyor.")
                stop_all()
                gps_active = False
                time.sleep(1)
            elif sari_engel or lidar_engel:
                if sari_engel:
                    print("Sarı engel tespit edildi! Kaçma manevrası başlatılıyor.")
                    M = cv2.moments(masks['yellow'])
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        if cx < rw // 2:
                            print("Engel solda, sağa kaç!")
                            send_rc(THR_FWD, STR_RIGHT)
                        else:
                            print("Engel sağda, sola kaç!")
                            send_rc(THR_FWD, STR_LEFT)
                    else:
                        stop_all()
                elif lidar_engel:
                    print("Lidar engel tespit etti! Kaçma manevrası başlatılıyor.")
                    send_rc(THR_FWD, STR_RIGHT)  # Basitçe sağa kaç (geliştirilebilir)
                time.sleep(1)
            elif gps_active:
                dist = distance_to_target(current_lat, current_lon, TARGET_LAT, TARGET_LON)
                print(f"Engel yok, hedefe ilerleniyor. Kalan mesafe: {dist:.1f} m")
                go_to_gps_target(TARGET_LAT, TARGET_LON, current_lat, current_lon, heading)
                time.sleep(0.5)
            else:
                stop_all()

            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    finally:
        print("\n▶ Override temizleniyor ve disarm …")
        vehicle.channels.overrides = {}
        vehicle.armed = False
        vehicle.close()

        cap.release()
        cv2.destroyAllWindows()

        try:
            guard.stop()
        except Exception:
            pass
        time.sleep(0.2)
        print("✅ Renk + LiDAR kontrolü sonlandırıldı.")

if __name__ == "__main__":
    main()