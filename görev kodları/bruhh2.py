# GPS Navigation - Simüle Test Modu
"""
GPS Simülasyon Test Sistemi
==========================
• Sahte GPS koordinatları kullanır
• Motor komutlarını test eder
• Engel algılama çalışır
• Gerçek GPS olmadan test edilebilir
"""

import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode

# Ayarlar
CONNECTION = 'COM17'
BAUD = 115200
CAM_INDEX = 0

# PWM değerleri
PWM_STOP = 1500
PWM_FAST = 1800

# Dönüş keskinliği ayarları (normal GPS navigasyon için)
# Deadzone küçültüldü, kazanç ve limit eklendi
TURN_DEADZONE_DEG = 6          # 15 → 6 derece: daha erken dönmeye başla
STEER_GAIN_DEG2PWM = 6.0       # 2 → 6: açı hatasını daha agresif PWM'e çevir
STEER_OFFSET_MIN = 70          # küçük hatalarda dahi en az bu kadar kır
STEER_OFFSET_MAX = 350         # direksiyon maksimum ofset (1100-1900 limitlerine saygılı)
HARD_TURN_THRESHOLD_DEG = 35   # büyük açı hatası: tam kilide yakın dön
TURN_THROTTLE_REDUCTION = 120  # keskin dönerken gazı biraz azalt

# Hedef koordinat
TARGET_LAT = 40.7712335   # Mission Planner hedef
TARGET_LON = 29.4375378   # Mission Planner hedef

# Simüle GPS (test için)
SIMULATE_GPS = False   # True = sahte GPS kullan, False = gerçek GPS
sim_lat = 40.7712335    # Başlangıç koordinatı (hedefe yakın)
sim_lon = 29.4375378 
sim_heading = 45     # Başlangıç yönü (kuzeydoğu)

# Global değişkenler
current_mode = "MANUAL"
obstacle_detected = False
obstacle_avoidance_active = False
avoidance_start_time = 0
avoidance_stage = 0  # 0: normal, 1: yan hareket, 2: düz git, 3: geri dön

def detect_obstacle_position(roi, mask_yellow):
    """Engelin konumunu belirle (sol, orta, sağ)"""
    h, w = roi.shape[:2]
    
    # ROI'yi 3 parçaya böl
    left_part = mask_yellow[:, :w//3]
    center_part = mask_yellow[:, w//3:2*w//3]
    right_part = mask_yellow[:, 2*w//3:]
    
    # Her bölgedeki sarı piksel yoğunluğu
    left_density = np.count_nonzero(left_part) / (left_part.shape[0] * left_part.shape[1])
    center_density = np.count_nonzero(center_part) / (center_part.shape[0] * center_part.shape[1])
    right_density = np.count_nonzero(right_part) / (right_part.shape[0] * right_part.shape[1])
    
    print(f"🔍 Engel yoğunluğu - Sol: {left_density*100:.1f}%, Orta: {center_density*100:.1f}%, Sağ: {right_density*100:.1f}%")
    
    # En yoğun bölgeyi bul
    max_density = max(left_density, center_density, right_density)
    
    if max_density < 0.05:  # çok az engel
        return "none"
    elif left_density == max_density:
        return "left"
    elif right_density == max_density:
        return "right"
    else:
        return "center"

def obstacle_avoidance_maneuver(obstacle_position):
    """Engel atlama manevrası"""
    global obstacle_avoidance_active, avoidance_start_time, avoidance_stage
    
    current_time = time.time()
    
    if not obstacle_avoidance_active:
        # Manevra başlat
        obstacle_avoidance_active = True
        avoidance_start_time = current_time
        avoidance_stage = 1
        print(f"🚁 ENGEL ATLAMA BAŞLADI - Engel konumu: {obstacle_position}")
    
    elapsed = current_time - avoidance_start_time
    
    if avoidance_stage == 1:  # Yan hareket (2 saniye)
        if obstacle_position == "left":
            # Sol engel -> sağa git
            send_rc(PWM_FAST, PWM_STOP + 120)  # sağa dön
            print("↗️ SOL ENGEL - SAĞA KAÇIYOR")
        elif obstacle_position == "right":
            # Sağ engel -> sola git  
            send_rc(PWM_FAST, PWM_STOP - 120)  # sola dön
            print("↖️ SAĞ ENGEL - SOLA KAÇIYOR")
        else:  # center
            # Orta engel -> rastgele tarafa kaç
            direction = 1 if (current_time % 2) > 1 else -1
            send_rc(PWM_FAST, PWM_STOP + (120 * direction))
            print(f"{'↗️ ORTA ENGEL - SAĞA' if direction > 0 else '↖️ ORTA ENGEL - SOLA'} KAÇIYOR")
        
        if elapsed > 2.0:  # 2 saniye yan hareket
            avoidance_stage = 2
            avoidance_start_time = current_time
            
    elif avoidance_stage == 2:  # Düz git (1.5 saniye)
        send_rc(PWM_FAST, PWM_STOP)
        print("➡️ ENGEL ATLAMA - DÜZ GİDİYOR")
        
        if elapsed > 1.5:  # 1.5 saniye düz git
            avoidance_stage = 3
            avoidance_start_time = current_time
            
    elif avoidance_stage == 3:  # Geri dön (1.5 saniye)
        if obstacle_position == "left":
            # Sola geri dön
            send_rc(PWM_FAST, PWM_STOP - 100)
            print("↖️ ENGEL ATLAMA - SOLA GERİ DÖNÜYOR")
        elif obstacle_position == "right":
            # Sağa geri dön
            send_rc(PWM_FAST, PWM_STOP + 100)
            print("↗️ ENGEL ATLAMA - SAĞA GERİ DÖNÜYOR")
        else:  # center
            # Ters yöne geri dön
            direction = -1 if (current_time % 2) > 1 else 1
            send_rc(PWM_FAST, PWM_STOP + (100 * direction))
            print(f"{'↗️' if direction > 0 else '↖️'} ENGEL ATLAMA - GERİ DÖNÜYOR")
        
        if elapsed > 1.5:  # 1.5 saniye geri dön
            # Manevra tamamlandı
            obstacle_avoidance_active = False
            avoidance_stage = 0
            print("✅ ENGEL ATLAMA TAMAMLANDI")
            return False  # manevra bitti
    
    return True  # manevra devam ediyor

def calculate_bearing(lat1, lon1, lat2, lon2):
    """İki GPS koordinatı arasındaki bearing hesapla"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing = math.atan2(y, x)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

def calculate_distance(lat1, lon1, lat2, lon2):
    """İki koordinat arası mesafe (metre)"""
    # Basit hesaplama (küçük mesafeler için)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Derece başına yaklaşık metre
    lat_to_m = 111000  # 1 derece = ~111km
    lon_to_m = 111000 * math.cos(math.radians(lat1))
    
    distance = math.sqrt((dlat * lat_to_m)**2 + (dlon * lon_to_m)**2)
    return distance

def bearing_to_motor_command(target_bearing, current_heading):
    """Bearing farkına göre motor komutları"""
    bearing_diff = target_bearing - current_heading
    
    # -180 ile +180 normalize
    if bearing_diff > 180:
        bearing_diff -= 360
    elif bearing_diff < -180:
        bearing_diff += 360
    
    print(f"🧭 Hedef: {target_bearing:.0f}°, Mevcut: {current_heading:.0f}°, Fark: {bearing_diff:.0f}°")

    abs_err = abs(bearing_diff)

    if abs_err < TURN_DEADZONE_DEG:
        thr, steer = PWM_FAST, PWM_STOP
        print(f"➡️ DÜZ GİT: THR={thr}, STR={steer} (deadzone {TURN_DEADZONE_DEG}°)")
    else:
        # Agresif direksiyon: min ofset + kazanç, büyük hatalarda hızlı saturasyon
        if abs_err >= HARD_TURN_THRESHOLD_DEG:
            steer_offset = STEER_OFFSET_MAX
        else:
            steer_offset = int(max(
                STEER_OFFSET_MIN,
                min(STEER_OFFSET_MAX, abs_err * STEER_GAIN_DEG2PWM)
            ))

        # Keskin dönerken gazı bir miktar azalt
        thr = max(PWM_STOP, PWM_FAST - TURN_THROTTLE_REDUCTION if abs_err >= 20 else PWM_FAST)

        if bearing_diff > 0:  # sağa dön
            steer = PWM_STOP - steer_offset  # TERS: donanım yönüne uygun
            print(f"↗️ SAĞA DÖN: THR={thr}, STR={steer} (offset: -{steer_offset})")
        else:  # sola dön
            steer = PWM_STOP + steer_offset  # TERS: donanım yönüne uygun
            print(f"↖️ SOLA DÖN: THR={thr}, STR={steer} (offset: +{steer_offset})")
    
    return thr, steer

def send_rc(throttle_pwm: int, steer_pwm: int):
    """Motor komutları gönder"""
    throttle_pwm = max(1100, min(1900, int(throttle_pwm)))
    steer_pwm = max(1100, min(1900, int(steer_pwm)))
    
    vehicle.channels.overrides = {
        '3': throttle_pwm,
        '1': steer_pwm,
        '2': steer_pwm
    }
    print(f"🔧 MOTOR: CH3={throttle_pwm}, CH1={steer_pwm}, CH2={steer_pwm}")

def stop_all():
    send_rc(PWM_STOP, PWM_STOP)
    print("⏹️ MOTORLAR DURDURULDU")

def get_gps_data():
    """GPS verisi al (gerçek veya simüle)"""
    global sim_lat, sim_lon, sim_heading
    
    if SIMULATE_GPS:
        return sim_lat, sim_lon, sim_heading
    else:
        # Gerçek GPS
        if vehicle.location.global_frame:
            lat = vehicle.location.global_frame.lat
            lon = vehicle.location.global_frame.lon
            heading = vehicle.heading if vehicle.heading else 0
            
            if lat != 0 and lon != 0:
                return lat, lon, heading
        
        return None, None, None

def update_simulated_position(thr, steer):
    """Simüle pozisyonu güncelle (motor komutlarına göre)"""
    global sim_lat, sim_lon, sim_heading
    
    if not SIMULATE_GPS:
        return
    
    # Motor komutlarına göre pozisyon değişimi simüle et
    if thr > PWM_STOP:  # ileri gidiyoruz
        # Heading değişimi (steering'e göre)
        if steer > PWM_STOP + 20:  # sağa dönüş
            sim_heading += 2
        elif steer < PWM_STOP - 20:  # sola dönüş
            sim_heading -= 2
        
        sim_heading = (sim_heading + 360) % 360
        
        # Pozisyon değişimi (heading yönünde)
        speed_factor = 0.00001  # simülasyon hızı
        sim_lat += speed_factor * math.cos(math.radians(sim_heading))
        sim_lon += speed_factor * math.sin(math.radians(sim_heading))

# Bağlantı
print("▶ Bağlanılıyor...")
try:
    vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False, timeout=30)
    vehicle.mode = VehicleMode("MANUAL")
    
    # Arm
    for i in range(5):
        vehicle.armed = True
        time.sleep(0.5)
        if vehicle.armed:
            break
    
    print("✅ Bağlantı OK, ARMED" if vehicle.armed else "⚠️ ARM EDİLEMEDİ")
    
except Exception as e:
    print(f"❌ Hata: {e}")
    exit(1)

stop_all()

# Kamera
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    print("❌ Kamera hatası")
    exit(1)

print(f"\n📍 SİMÜLE GPS: {'AÇIK' if SIMULATE_GPS else 'KAPALI'}")
print(f"🎯 Hedef: {TARGET_LAT:.6f}, {TARGET_LON:.6f}")
print(f"📍 Başlangıç: {sim_lat:.6f}, {sim_lon:.6f}")

print("\n🎮 KONTROLLER:")
print("'g' = GPS otomatik modu")
print("'1' = Test: Düz git")
print("'2' = Test: Sağa dön")
print("'3' = Test: Sola dön")
print("'4' = Test: Engel atlama (sol)")
print("'5' = Test: Engel atlama (sağ)")
print("'6' = Test: Engel atlama (orta)")
print("'r' = Simüle pozisyonu sıfırla")
print("'s' = Dur")
print("'q' = Çıkış")

last_nav_update = 0

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        
        # GPS verisi al
        current_lat, current_lon, current_heading = get_gps_data()
        
        if current_lat is not None:
            # Hedefe olan mesafe ve bearing
            distance = calculate_distance(current_lat, current_lon, TARGET_LAT, TARGET_LON)
            target_bearing = calculate_bearing(current_lat, current_lon, TARGET_LAT, TARGET_LON)
            
            gps_status = f"Lat:{current_lat:.6f}, Lon:{current_lon:.6f}"
            distance_status = f"Mesafe: {distance:.1f}m"
            bearing_status = f"Hedef bearing: {target_bearing:.0f}°, Heading: {current_heading:.0f}°"
        else:
            gps_status = "GPS verisi yok"
            distance_status = "Mesafe: ---"
            bearing_status = "Bearing: ---"
            distance = 999
        
        # ROI ve engel algılama
        h, w = frame.shape[:2]
        rw, rh = int(w * 0.6), int(h * 0.6)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        roi = frame[y0:y0+rh, x0:x0+rw]
        
        # Sarı engel algılama
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))
        yellow_ratio = np.count_nonzero(mask_yellow) / (roi.shape[0] * roi.shape[1])
        
        # Engel konumu tespit
        obstacle_position = detect_obstacle_position(roi, mask_yellow)
        
        # Engel kontrolü
        if yellow_ratio > 0.15:
            if not obstacle_detected:
                print(f"🚨 ENGEL ALGILANDI! {yellow_ratio*100:.1f}% - Konum: {obstacle_position}")
                obstacle_detected = True
        else:
            if obstacle_detected:
                print("✅ ENGEL TEMİZLENDİ")
            obstacle_detected = False
            # Engel yoksa manevraya son ver
            if obstacle_avoidance_active:
                obstacle_avoidance_active = False
                avoidance_stage = 0
                print("🔄 ENGEL ATLAMA İPTAL - ENGEL YOK")
        
        # Otomatik GPS navigasyon
        if current_mode == "AUTO_GPS" and current_lat is not None:
            if time.time() - last_nav_update > 0.5:  # 0.5 saniyede bir güncelle
                
                # Önce engel atlama kontrolü
                if obstacle_detected and yellow_ratio > 0.15:
                    # Engel atlama manevrası aktif
                    maneuver_active = obstacle_avoidance_maneuver(obstacle_position)
                    if maneuver_active:
                        # Manevra sırasında simüle pozisyonu güncelle
                        current_rc = vehicle.channels.overrides if hasattr(vehicle, 'channels') else {}
                        thr = current_rc.get('3', PWM_STOP)
                        steer = current_rc.get('1', PWM_STOP)
                        update_simulated_position(thr, steer)
                        last_nav_update = time.time()
                        continue  # Normal navigasyona geçme
                
                # Normal GPS navigasyon (engel yoksa veya manevra bitmişse)
                if not obstacle_detected and not obstacle_avoidance_active:
                    if distance > 2:  # hedefe 2m'den uzaksa
                        thr, steer = bearing_to_motor_command(target_bearing, current_heading)
                        send_rc(thr, steer)
                        update_simulated_position(thr, steer)
                    else:
                        stop_all()
                        print("🎯 HEDEFE VARILDI!")
                        current_mode = "MANUAL"
                
                last_nav_update = time.time()
        
        # Görsel
        cv2.rectangle(frame, (x0, y0), (x0+rw, y0+rh), (0,255,0), 2)
        
        # ROI'yi 3 parçaya böl (görsel gösterim için)
        part_w = rw // 3
        cv2.line(frame, (x0 + part_w, y0), (x0 + part_w, y0 + rh), (255,255,0), 1)
        cv2.line(frame, (x0 + 2*part_w, y0), (x0 + 2*part_w, y0 + rh), (255,255,0), 1)
        
        # Durum bilgileri
        status_color = (0,255,0) if current_mode == "MANUAL" else (0,255,255)
        avoidance_text = " - ENGEL ATLAMA" if obstacle_avoidance_active else ""
        cv2.putText(frame, f"Mod: {current_mode}{avoidance_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(frame, gps_status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        cv2.putText(frame, distance_status, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.putText(frame, bearing_status, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
        
        # Engel bilgisi
        obstacle_color = (0,0,255) if obstacle_detected else (0,255,255)
        obstacle_info = f"Engel: {yellow_ratio*100:.1f}%"
        if obstacle_detected:
            obstacle_info += f" ({obstacle_position})"
        cv2.putText(frame, obstacle_info, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, obstacle_color, 2)
        
        # Engel atlama aşama bilgisi
        if obstacle_avoidance_active:
            stage_names = ["", "Yan Hareket", "Düz Git", "Geri Dön"]
            stage_text = f"Aşama: {stage_names[avoidance_stage]}"
            cv2.putText(frame, stage_text, (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,100,100), 2)
        
        cv2.imshow('GPS Navigation Simulator', frame)
        
        # Kontroller
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('g'):
            current_mode = "AUTO_GPS"
            print("🎯 GPS otomatik modu aktif")
        elif key == ord('1'):
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP)
            update_simulated_position(PWM_FAST, PWM_STOP)
            print("➡️ DÜZ GİT")
        elif key == ord('2'):
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP + 100)
            update_simulated_position(PWM_FAST, PWM_STOP + 100)
            print("↗️ SAĞA DÖN")
        elif key == ord('3'):
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP - 100)
            update_simulated_position(PWM_FAST, PWM_STOP - 100)
            print("↖️ SOLA DÖN")
        elif key == ord('4'):
            # Sol engel atlama testi
            current_mode = "MANUAL"
            obstacle_detected = True
            obstacle_avoidance_maneuver("left")
            print("🧪 TEST: Sol engel atlama")
        elif key == ord('5'):
            # Sağ engel atlama testi
            current_mode = "MANUAL"
            obstacle_detected = True
            obstacle_avoidance_maneuver("right")
            print("🧪 TEST: Sağ engel atlama")
        elif key == ord('6'):
            # Orta engel atlama testi
            current_mode = "MANUAL"
            obstacle_detected = True
            obstacle_avoidance_maneuver("center")
            print("🧪 TEST: Orta engel atlama")
        elif key == ord('r'):
            # Pozisyonu sıfırla
            sim_lat = 41.0080
            sim_lon = 28.9780
            sim_heading = 45
            print("🔄 Simüle pozisyon sıfırlandı")
        elif key == ord('s'):
            current_mode = "MANUAL"
            stop_all()
            print("⏹️ DUR")

except KeyboardInterrupt:
    pass
finally:
    print("\n▶ Kapatılıyor...")
    stop_all()
    vehicle.channels.overrides = {}
    try:
        vehicle.armed = False
    except:
        pass
    vehicle.close()
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Tamamlandı")