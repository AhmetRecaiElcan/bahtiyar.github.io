# GPS Navigation Test (Basitleştirilmiş)
"""
GPS Test Sistemi
===============
• Sadece motor testi
• Manuel GPS komutları
• Debug bilgileri
"""

import time
import cv2
import numpy as np
from dronekit import connect, VehicleMode

# Bağlantı ayarları
CONNECTION = 'COM18'
BAUD = 57600
CAM_INDEX = 0

# PWM değerleri
PWM_STOP = 1500
PWM_NORMAL = 1700
PWM_FAST = 1800

# Global değişkenler
current_mode = "MANUAL"  # MANUAL, GPS_TEST
test_commands = [
    (PWM_FAST, PWM_STOP),      # düz git
    (PWM_FAST, PWM_STOP + 100), # sağa dön
    (PWM_FAST, PWM_STOP - 100), # sola dön
    (PWM_STOP, PWM_STOP)        # dur
]
command_index = 0

def send_rc(throttle_pwm: int, steer_pwm: int):
    """Motor komutlarını gönder"""
    throttle_pwm = max(1100, min(1900, int(throttle_pwm)))
    steer_pwm = max(1100, min(1900, int(steer_pwm)))
    
    vehicle.channels.overrides = {
        '3': throttle_pwm,   # throttle
        '1': steer_pwm,      # steer CH1
        '2': steer_pwm       # steer CH2
    }
    print(f"🔧 MOTOR: CH3={throttle_pwm}, CH1={steer_pwm}, CH2={steer_pwm}")

def stop_all():
    print("⏹️ MOTORLAR DURDURULDU")
    send_rc(PWM_STOP, PWM_STOP)

# Bağlantı kur
print("▶ Pixhawk'a bağlanılıyor...")
try:
    vehicle = connect(CONNECTION, baud=BAUD, wait_ready=False, timeout=30)
    print("✅ Bağlantı başarılı")
    
    # Manuel mod
    vehicle.mode = VehicleMode("MANUAL")
    
    # Arm et
    arm_attempts = 0
    while not vehicle.armed and arm_attempts < 5:
        vehicle.armed = True
        arm_attempts += 1
        time.sleep(0.5)
    
    if vehicle.armed:
        print("✅ ARMED")
    else:
        print("⚠️ ARM EDİLEMEDİ")
        
except Exception as e:
    print(f"❌ Bağlantı hatası: {e}")
    exit(1)

# Başlangıç
stop_all()

# Kamera
cap = cv2.VideoCapture(CAM_INDEX)
if not cap.isOpened():
    print("❌ Kamera açılamadı")
    exit(1)

print("\n🎮 KONTROLLER:")
print("'g' = GPS test modu (otomatik komutlar)")
print("'1' = Düz git")
print("'2' = Sağa dön") 
print("'3' = Sola dön")
print("'s' = Dur")
print("'q' = Çıkış")

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
            
        # Durum bilgisi
        status_text = f"Mod: {current_mode}"
        cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        
        if current_mode == "GPS_TEST":
            # Otomatik test komutları
            if time.time() % 4 < 0.1:  # her 4 saniyede bir değiştir
                thr, steer = test_commands[command_index]
                send_rc(thr, steer)
                command_index = (command_index + 1) % len(test_commands)
                print(f"🧪 Test komutu {command_index}: THR={thr}, STR={steer}")
        
        cv2.imshow('Motor Test', frame)
        
        # Klavye kontrolleri
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('g'):
            current_mode = "GPS_TEST"
            print("🧪 GPS test modu aktif")
        elif key == ord('1'):
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP)
            print("➡️ DÜZ GİT")
        elif key == ord('2'):
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP + 100)
            print("↗️ SAĞA DÖN")
        elif key == ord('3'):
            current_mode = "MANUAL"
            send_rc(PWM_FAST, PWM_STOP - 100)
            print("↖️ SOLA DÖN")
        elif key == ord('s'):
            current_mode = "MANUAL"
            stop_all()
            print("⏹️ DUR")

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
    cv2.destroyAllWindows()
    print("✅ Test tamamlandı")
