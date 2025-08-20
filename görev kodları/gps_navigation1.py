# scripts/gps_navigation_with_obstacle_avoidance.py
"""
GPS Navigasyon + Engel Atlama Sistemi
=====================================
• Hedef GPS koordinatına gitme
• Yolda engel algılama (sarı renk = engel)
• Engel atlama manevrası
• Hedefe devam etme

Renk → Davranış Haritası:
• Yeşil/Mavi → Düz git (güvenli yol)
• Sarı → Engel! Kaçınma manevrası
• Kırmızı → Güvenli yol

Motor Kontrol:
• Sol motor → PWM_STOP - steer_offset
•            cv2.putText(frame, f"Mod: {current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(frame, f"GPS: {status_data.get('gps_status', 'Bilinmiyor')}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.putText(frame, f"Hedef: {status_data['distance_to_target']:.1f}m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(frame, f"Engel: {yellow_ratio*100:.1f}% ({status_data['obstacle_position']})", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame, f"THR:{thr} STR:{steer}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2) motor → PWM_STOP + steer_offset
"""

import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
import threading
from queue import Queue
from geopy.distance import geodesic

# —————————————————— Kullanıcı ayarları ——————————————————
CONNECTION = 'COM18'
BAUD       = 57600
CAM_INDEX  = 0
SHOW_WIN   = True
CONNECTION_TIMEOUT = 60

# PWM değerleri
PWM_STOP   = 1500
PWM_SLOW   = 1550       # yavaş ileri
PWM_NORMAL = 1700       # normal ileri (hızlandırıldı)
PWM_FAST   = 1800       # hızlı ileri (hızlandırıldı)

# Navigasyon ayarları
TARGET_LAT = 40.7710894  # Hedef koordinat - Mission Planner'dan alınan
TARGET_LON = 29.4372419  # Hedef koordinat - Mission Planner'dan alınan
ARRIVAL_RADIUS = 2.0     # metre - hedefe bu kadar yaklaşınca "vardı" say
GPS_UPDATE_RATE = 1.0   # saniye - GPS kontrolü

# Engel algılama ayarları
OBSTACLE_THRESHOLD = 0.15   # ROI'nin %15'i sarı ise engel var
SAFE_DISTANCE = 1.0         # metre - engelden bu kadar uzak dur
AVOIDANCE_DURATION = 3.0    # saniye - kaçınma manevrası süresi

# —————————————————— Global değişkenler ——————————————————
current_mode = "MANUAL"     # MANUAL, AUTO_GPS, OBSTACLE_AVOID
obstacle_detected = False
avoidance_start_time = 0
last_gps_check = 0

# Thread-safe haberleşme
command_queue = Queue()
status_data = {
    'distance_to_target': 999,
    'bearing_to_target': 0,
    'current_lat': 0,
    'current_lon': 0,
    'obstacle_ratio': 0,
    'obstacle_position': 'none',
    'gps_status': 'Başlatılıyor...'
}

def calculate_bearing(lat1, lon1, lat2, lon2):
    """İki GPS koordinatı arasındaki bearing (derece) hesapla"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing = math.atan2(y, x)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

def calculate_distance(lat1, lon1, lat2, lon2):
    """İki GPS koordinatı arasındaki mesafe (metre)"""
    return geodesic((lat1, lon1), (lat2, lon2)).meters

def bearing_to_motor_command(target_bearing, current_bearing):
    """Bearing farkına göre motor komutları oluştur"""
    bearing_diff = target_bearing - current_bearing
    
    # -180 ile +180 arası normalize et
    if bearing_diff > 180:
        bearing_diff -= 360
    elif bearing_diff < -180:
        bearing_diff += 360
    
    print(f"🧭 Target: {target_bearing:.0f}°, Current: {current_bearing:.0f}°, Diff: {bearing_diff:.0f}°")
    
    # Motor komutlarına çevir
    base_throttle = PWM_FAST
    
    if abs(bearing_diff) < 15:  # düz git
        thr, steer = base_throttle, PWM_STOP
        print(f"➡️ DÜZ GİT: THR={thr}, STR={steer}")
    elif bearing_diff > 0:  # sağa dön (target bearing daha büyük)
        # Sağa dönmek için: sol motor hızlı, sağ motor yavaş
        steer_offset = min(150, abs(bearing_diff) * 2)  # maksimum 150 PWM fark
        thr = base_throttle
        steer = PWM_STOP + steer_offset  # DÜZELTİLDİ: sağa steer
        print(f"↗️ SAĞA DÖN: THR={thr}, STR={steer} (offset: +{steer_offset})")
    else:  # sola dön (target bearing daha küçük)
        # Sola dönmek için: sağ motor hızlı, sol motor yavaş  
        steer_offset = min(150, abs(bearing_diff) * 2)  # maksimum 150 PWM fark
        thr = base_throttle
        steer = PWM_STOP - steer_offset  # DÜZELTİLDİ: sola steer
        print(f"↖️ SOLA DÖN: THR={thr}, STR={steer} (offset: -{steer_offset})")
    
    return thr, steer

def gps_navigation_thread(vehicle):
    """GPS navigasyon thread'i"""
    global current_mode, last_gps_check, status_data
    
    print("🛰️ GPS thread başlatıldı")
    
    while True:
        try:
            if time.time() - last_gps_check >= GPS_UPDATE_RATE:
                last_gps_check = time.time()
                
                # GPS durumu kontrolü
                gps_fix_type = getattr(vehicle.gps_0, 'fix_type', 0) if hasattr(vehicle, 'gps_0') else 0
                satellites_visible = getattr(vehicle.gps_0, 'satellites_visible', 0) if hasattr(vehicle, 'gps_0') else 0
                
                print(f"🛰️ GPS Fix Type: {gps_fix_type}, Uydu: {satellites_visible}")
                
                # GPS verisi kontrol et
                if not vehicle.location.global_frame:
                    print("⚠️ GPS location verisi yok")
                    status_data['gps_status'] = "GPS verisi yok"
                    time.sleep(2)
                    continue
                    
                current_lat = vehicle.location.global_frame.lat
                current_lon = vehicle.location.global_frame.lon
                
                # Geçerli koordinat kontrolü
                if current_lat == 0 and current_lon == 0:
                    print(f"⚠️ Geçersiz GPS (0,0) - Fix: {gps_fix_type}, Uydu: {satellites_visible}")
                    status_data['gps_status'] = f"Fix:{gps_fix_type} Uydu:{satellites_visible}"
                    time.sleep(2)
                    continue
                
                # GPS fix kalitesi kontrolü
                if gps_fix_type < 3:  # 3D fix gerekli
                    print(f"⚠️ GPS fix yetersiz (Fix Type: {gps_fix_type}) - 3D fix gerekli")
                    status_data['gps_status'] = f"Fix yetersiz:{gps_fix_type}"
                    time.sleep(2)
                    continue
                
                # Uydu sayısı kontrolü
                if satellites_visible < 6:
                    print(f"⚠️ Uydu sayısı yetersiz ({satellites_visible}) - En az 6 gerekli")
                    status_data['gps_status'] = f"Uydu yetersiz:{satellites_visible}"
                    time.sleep(2)
                    continue
                
                print(f"✅ GPS İYİ: Lat:{current_lat:.6f}, Lon:{current_lon:.6f}, Fix:{gps_fix_type}, Uydu:{satellites_visible}")
                
                # Hedefe olan mesafe ve bearing
                distance = calculate_distance(current_lat, current_lon, TARGET_LAT, TARGET_LON)
                bearing = calculate_bearing(current_lat, current_lon, TARGET_LAT, TARGET_LON)
                
                # Status güncelle
                status_data.update({
                    'distance_to_target': distance,
                    'bearing_to_target': bearing,
                    'current_lat': current_lat,
                    'current_lon': current_lon,
                    'gps_status': f"İYİ Fix:{gps_fix_type} Uydu:{satellites_visible}"
                })
                
                # Hedefe vardı mı?
                if distance <= ARRIVAL_RADIUS:
                    command_queue.put(('ARRIVED', None))
                    print(f"🎯 Hedefe varıldı! Mesafe: {distance:.1f}m")
                    break
                
                # Auto mode'da GPS komutu gönder
                if current_mode == "AUTO_GPS" and not obstacle_detected:
                    # Şu anki heading'i al (compass)
                    current_heading = vehicle.heading if vehicle.heading else 0
                    
                    # Motor komutlarını hesapla
                    thr, steer = bearing_to_motor_command(bearing, current_heading)
                    command_queue.put(('GPS_COMMAND', (thr, steer)))
                    
                    print(f"📍 GPS: {distance:.1f}m, bearing: {bearing:.0f}°, heading: {current_heading:.0f}°")
            
            time.sleep(0.5)  # GPS için daha uzun bekleme
            
        except Exception as e:
            print(f"❌ GPS thread hatası: {e}")
            time.sleep(2)

# —————————————————— Bağlantı ——————————————————
print("▶ Pixhawk'a bağlanılıyor …")
try:
    vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False, timeout=CONNECTION_TIMEOUT)
    print("▶ Bağlantı kuruldu, araç hazırlanıyor...")
    
    # Sistem hazırlık
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
    
except Exception as e:
    print(f"HATA: Bağlantı kurulamadı - {e}")
    exit(1)

# Mod ayarla ve arm et
print("▶ MANUAL moda geçiliyor...")
vehicle.mode = VehicleMode("MANUAL")

# Arm et
print("▶ Araç arm ediliyor...")
arm_attempts = 0
while not vehicle.armed and arm_attempts < 10:
    try:
        vehicle.armed = True
        arm_attempts += 1
        time.sleep(0.5)
    except Exception as e:
        print(f"▶ Arm hatası: {e}")

if vehicle.armed:
    print("ARMED ✔")
else:
    print("WARNING: Araç arm edilemedi")

# RC override helper
def send_rc(throttle_pwm: int, steer_pwm: int):
    """Motor komutlarını gönder ve debug yap"""
    # PWM değerlerini güvenli aralıkta tut
    throttle_pwm = max(1100, min(1900, int(throttle_pwm)))
    steer_pwm = max(1100, min(1900, int(steer_pwm)))
    
    # Hem CH1 hem CH2'ye steer komutunu gönder (test için)
    vehicle.channels.overrides = {
        '3': throttle_pwm,   # throttle
        '1': steer_pwm,      # steer CH1
        '2': steer_pwm       # steer CH2 (test)
    }
    print(f"🔧 MOTOR KOMUT: CH3={throttle_pwm}, CH1={steer_pwm}, CH2={steer_pwm}")

def stop_all():
    print("⏹️ TÜM MOTORLAR DURDURULDU")
    send_rc(PWM_STOP, PWM_STOP)

# Başlangıç dur
stop_all()

# GPS thread'ini başlat
gps_thread = threading.Thread(target=gps_navigation_thread, args=(vehicle,), daemon=True)
gps_thread.start()

# —————————————————— Kamera ——————————————————
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    raise RuntimeError("Kamera açılamadı.")

print("▶ Sistem hazır!")
print(f"🎯 Hedef: {TARGET_LAT:.6f}, {TARGET_LON:.6f}")
print("🎮 Kontroller:")
print("  'g' = GPS moduna geç (hedefe git)")
print("  'm' = Manuel moda geç") 
print("  's' = Dur")
print("  'q' = Çıkış")

# CLAHE
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))

try:
    while True:
        # Kamera frame'i oku
        ok, frame = cap.read()
        if not ok:
            continue

        # ROI kırp
        h, w = frame.shape[:2]
        rw, rh = int(w * 0.6), int(h * 0.6)
        x0 = (w - rw) // 2
        y0 = (h - rh) // 2
        roi = frame[y0:y0+rh, x0:x0+rw].copy()

        # HSV dönüştür
        roi_blur = cv2.GaussianBlur(roi, (5,5), 0)
        hsv = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HSV)

        # CLAHE uygula
        h_, s_, v_ = cv2.split(hsv)
        v_eq = clahe.apply(v_)
        hsv = cv2.merge([h_, s_, v_eq])

        # Renk maskeleri
        # Sarı (engel)
        mask_yellow = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))

        # Yeşil (güvenli yol)
        mask_green = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))

        # Kırmızı (güvenli yol)
        red1 = cv2.inRange(hsv, (0, 120, 60), (10, 255, 255))
        red2 = cv2.inRange(hsv, (170, 120, 60), (180, 255, 255))
        mask_red = cv2.bitwise_or(red1, red2)

        # Morfolojik temizlik
        k = np.ones((5,5), np.uint8)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, k, iterations=1)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, k, iterations=1)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, k, iterations=1)

        # Engel konumu analizi (ROI'yi üç parçaya böl)
        roi_width = roi.shape[1]
        left_section = roi[:, 0:roi_width//3]
        center_section = roi[:, roi_width//3:2*roi_width//3]
        right_section = roi[:, 2*roi_width//3:]
        
        # Her bölümdeki sarı piksel oranı
        left_hsv = cv2.cvtColor(left_section, cv2.COLOR_BGR2HSV)
        center_hsv = cv2.cvtColor(center_section, cv2.COLOR_BGR2HSV)
        right_hsv = cv2.cvtColor(right_section, cv2.COLOR_BGR2HSV)
        
        left_yellow = cv2.inRange(left_hsv, (18, 140, 80), (38, 255, 255))
        center_yellow = cv2.inRange(center_hsv, (18, 140, 80), (38, 255, 255))
        right_yellow = cv2.inRange(right_hsv, (18, 140, 80), (38, 255, 255))
        
        left_ratio = np.count_nonzero(left_yellow) / (left_section.shape[0] * left_section.shape[1])
        center_ratio = np.count_nonzero(center_yellow) / (center_section.shape[0] * center_section.shape[1])
        right_ratio = np.count_nonzero(right_yellow) / (right_section.shape[0] * right_section.shape[1])
        
        # Engel konumunu belirle
        obstacle_position = "none"
        max_ratio = max(left_ratio, center_ratio, right_ratio)
        
        if max_ratio > OBSTACLE_THRESHOLD:
            if right_ratio == max_ratio and right_ratio > 0.1:
                obstacle_position = "right"  # Engel sağda → sola kaç
            elif left_ratio == max_ratio and left_ratio > 0.1:
                obstacle_position = "left"   # Engel solda → sağa kaç
            elif center_ratio == max_ratio and center_ratio > 0.1:
                obstacle_position = "center" # Engel ortada → sola kaç (varsayılan)
        
        print(f"🎯 Engel konumu: SOL={left_ratio*100:.1f}% ORTA={center_ratio*100:.1f}% SAĞ={right_ratio*100:.1f}% → {obstacle_position.upper()}")
        
        status_data['obstacle_ratio'] = max_ratio
        status_data['obstacle_position'] = obstacle_position

        # Engel algılama (SARI = ENGEL)
        if max_ratio > OBSTACLE_THRESHOLD:
            if not obstacle_detected:
                print(f"🚨 ENGEL ALGILANDI! Konum: {obstacle_position.upper()}, Oran: {max_ratio*100:.1f}%")
                obstacle_detected = True
                avoidance_start_time = time.time()
                current_mode = "OBSTACLE_AVOID"
        else:
            if obstacle_detected and time.time() - avoidance_start_time > AVOIDANCE_DURATION:
                print("✅ Engel aşıldı, GPS moduna dönülüyor")
                obstacle_detected = False
                current_mode = "AUTO_GPS"

        # Komut işleme
        thr, steer = PWM_STOP, PWM_STOP
        command_executed = False

        # Queue'dan komutları işle
        while not command_queue.empty():
            cmd_type, cmd_data = command_queue.get()
            if cmd_type == 'GPS_COMMAND' and current_mode == "AUTO_GPS":
                thr, steer = cmd_data
                command_executed = True
                print(f"✅ GPS KOMUT İŞLENDİ: THR={thr}, STR={steer}")
            elif cmd_type == 'ARRIVED':
                current_mode = "MANUAL"
                thr, steer = PWM_STOP, PWM_STOP
                command_executed = True
                print("🎯 HEDEFE VARILDI!")

        # Engel kaçınma manevrası
        if current_mode == "OBSTACLE_AVOID":
            elapsed = time.time() - avoidance_start_time
            if elapsed < AVOIDANCE_DURATION:
                # Engel konumuna göre kaçınma yönü belirle
                if status_data.get('obstacle_position') == 'right':
                    # Engel sağda → sola kaç (sağ motor çalıştır, sol motor yavaş)
                    thr = PWM_FAST
                    steer = PWM_STOP - 120  # sola steer = sağ motor hızlı, sol yavaş
                    print(f"🔄 SOLA KAÇINMA: {elapsed:.1f}/{AVOIDANCE_DURATION}s (engel sağda)")
                elif status_data.get('obstacle_position') == 'left':
                    # Engel solda → sağa kaç (sol motor çalıştır, sağ motor yavaş)
                    thr = PWM_FAST
                    steer = PWM_STOP + 120  # sağa steer = sol motor hızlı, sağ yavaş
                    print(f"🔄 SAĞA KAÇINMA: {elapsed:.1f}/{AVOIDANCE_DURATION}s (engel solda)")
                else:
                    # Engel ortada veya belirsiz → varsayılan sola kaç
                    thr = PWM_FAST
                    steer = PWM_STOP - 120  # sola steer = sağ motor hızlı
                    print(f"🔄 VARSAYILAN SOLA KAÇINMA: {elapsed:.1f}/{AVOIDANCE_DURATION}s")
                command_executed = True

        # Motor komutunu gönder (sadece AUTO modlarda)
        if current_mode != "MANUAL" and command_executed:
            send_rc(thr, steer)
        elif current_mode == "MANUAL":
            # Manuel modda motorları durdur
            stop_all()

        # Görsel debug
        if SHOW_WIN:
            # ROI çerçevesi
            cv2.rectangle(frame, (x0, y0), (x0+rw, y0+rh), (0,255,0), 2)
            
            # ROI bölümlerini göster (sol, orta, sağ)
            section_w = rw // 3
            cv2.line(frame, (x0+section_w, y0), (x0+section_w, y0+rh), (255,0,0), 1)  # sol-orta ayıracı
            cv2.line(frame, (x0+2*section_w, y0), (x0+2*section_w, y0+rh), (255,0,0), 1)  # orta-sağ ayıracı
            
            # Bölüm etiketleri
            cv2.putText(frame, "SOL", (x0+10, y0+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            cv2.putText(frame, "ORTA", (x0+section_w+10, y0+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            cv2.putText(frame, "SAG", (x0+2*section_w+10, y0+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            
            # Durum bilgileri
            status_color = (0,255,0) if current_mode == "AUTO_GPS" else (0,255,255) if current_mode == "OBSTACLE_AVOID" else (255,255,255)
            
            cv2.putText(frame, f"Mod: {current_mode}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(frame, f"Hedef: {status_data['distance_to_target']:.1f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(frame, f"Engel: {max_ratio*100:.1f}% ({obstacle_position})", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame, f"THR:{thr} STR:{steer}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)

            # Renk maskeleri
            small = 120
            vis = np.zeros((small, small*3, 3), np.uint8)
            
            def put_mask(m, pos, label):
                m3 = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                m3 = cv2.resize(m3, (small, small))
                y, x = pos
                vis[y:y+small, x:x+small] = m3
                cv2.putText(vis, label, (x+5, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            
            # Renk maskeleri + bölgesel analiz
            small = 120
            vis = np.zeros((small*2, small*3, 3), np.uint8)
            
            def put_mask(m, pos, label):
                m3 = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
                m3 = cv2.resize(m3, (small, small))
                y, x = pos
                vis[y:y+small, x:x+small] = m3
                cv2.putText(vis, label, (x+5, y+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            
            # Üst sıra: Engel bölge analizi
            put_mask(left_yellow, (0, 0), f"SOL {left_ratio*100:.1f}%")
            put_mask(center_yellow, (0, small), f"ORTA {center_ratio*100:.1f}%")
            put_mask(right_yellow, (0, small*2), f"SAĞ {right_ratio*100:.1f}%")
            
            # Alt sıra: Genel renk maskeleri
            put_mask(mask_yellow, (small, 0), f"SARI {max_ratio*100:.1f}%")
            put_mask(mask_green, (small, small), f"YEŞİL")
            put_mask(mask_red, (small, small*2), f"KIRMIZI")

            # Ana görüntü + maskeler
            H = max(frame.shape[0], vis.shape[0])
            W = frame.shape[1] + vis.shape[1] + 10
            canvas = np.zeros((H, W, 3), dtype=np.uint8)
            canvas[:frame.shape[0], :frame.shape[1]] = frame
            canvas[:vis.shape[0], frame.shape[1]+10:frame.shape[1]+10+vis.shape[1]] = vis

            cv2.imshow('GPS Navigation + Obstacle Avoidance', canvas)

        # Klavye kontrolleri
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('g'):
            current_mode = "AUTO_GPS"
            obstacle_detected = False
            print("🎯 GPS moduna geçildi - hedefe gidiliyor")
            print(f"📍 Mevcut GPS: {status_data.get('current_lat', 0):.6f}, {status_data.get('current_lon', 0):.6f}")
            print(f"🎯 Hedef GPS: {TARGET_LAT:.6f}, {TARGET_LON:.6f}")
            print(f"🛰️ GPS Durumu: {status_data.get('gps_status', 'Bilinmiyor')}")
        elif key == ord('t'):
            # GPS olmadan test modu
            current_mode = "MANUAL"
            print("🧪 GPS'SİZ TEST MODU")
            print("Simüle edilen bearing farkı: -90° (sola dönmeli)")
            thr, steer = bearing_to_motor_command(270, 0)  # test: batıya git, kuzeyden
            send_rc(thr, steer)
            print(f"🔧 Test komutu gönderildi: THR={thr}, STR={steer}")
        elif key == ord('m'):
            current_mode = "MANUAL"
            obstacle_detected = False
            stop_all()
            print("🎮 Manuel moda geçildi")
        elif key == ord('s'):
            current_mode = "MANUAL"
            obstacle_detected = False
            stop_all()
            print("⏹️ Durduruldu")
        elif key == ord('1'):
            # Test: düz git
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP)
            print("🧪 TEST: Düz git")
        elif key == ord('2'):
            # Test: sağa dön
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP + 100)
            print("🧪 TEST: Sağa dön")
        elif key == ord('3'):
            # Test: sola dön
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP - 100)
            print("🧪 TEST: Sola dön")

        time.sleep(0.03)

except KeyboardInterrupt:
    pass
finally:
    print("\n▶ Sistem kapatılıyor...")
    stop_all()
    vehicle.channels.overrides = {}
    try:
        vehicle.armed = False
    except:
        pass
    vehicle.close()
    cap.release()
    if SHOW_WIN:
        cv2.destroyAllWindows()
    print("✅ GPS navigasyon sistemi sonlandı.")
