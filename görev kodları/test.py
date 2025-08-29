import time
from dronekit import connect, VehicleMode
import pymavlink

# Drone'a bağlan
print("Drone'a bağlanılıyor...")
vehicle = connect('COM5', baud=57600, wait_ready=True)

def motor_kontrol(sol_motor, sag_motor, sure):
    """
    Motorları belirtilen değerlerle kontrol eder
    sol_motor, sag_motor: 0-100 arası değerler
    sure: saniye cinsinden süre
    """
    print(f"Motorlar çalışıyor: Sol={sol_motor}, Sağ={sag_motor}, Süre={sure}s")
    
    # Motor değerlerini ayarla
    vehicle.channels.overrides['1'] = sol_motor  # Sol motor
    vehicle.channels.overrides['2'] = sag_motor  # Sağ motor
    
    time.sleep(sure)
    
    # Motorları durdur
    vehicle.channels.overrides['1'] = 0
    vehicle.channels.overrides['2'] = 0
    print("Motorlar durduruldu")

def ileri_git():
    """Her iki motor da ileri gider"""
    print("İLERİ GİDİYOR...")
    motor_kontrol(80, 80, 3)  # Her iki motor da %80 güçle 3 saniye

def geri_git():
    """Her iki motor da geri gider"""
    print("GERİ GİDİYOR...")
    motor_kontrol(-80, -80, 3)  # Her iki motor da %80 güçle geri 3 saniye

def saga_git():
    """Sağa döner (sağ motor geri, sol motor ileri)"""
    print("SAĞA GİDİYOR...")
    motor_kontrol(80, -80, 3)  # Sol motor ileri, sağ motor geri

def sola_git():
    """Sola döner (sol motor geri, sağ motor ileri)"""
    print("SOLA GİDİYOR...")
    motor_kontrol(-80, 80, 3)  # Sol motor geri, sağ motor ileri

def main():
    try:
        print("Drone motor kontrol programı başlatılıyor...")
        print("Drone durumu:", vehicle.system_status.state)
        
        # Güvenlik için önce motorları durdur
        vehicle.channels.overrides['1'] = 0
        vehicle.channels.overrides['2'] = 0
        
        print("\n=== HAREKET SEQUENCE BAŞLIYOR ===")
        
        # 1. İleri git (3 saniye)
        ileri_git()
        time.sleep(1)  # 1 saniye bekle
        
        # 2. Geri git (3 saniye)
        geri_git()
        time.sleep(1)  # 1 saniye bekle
        
        # 3. Sağa git (3 saniye)
        saga_git()
        time.sleep(1)  # 1 saniye bekle
        
        # 4. Sola git (3 saniye)
        sola_git()
        
        print("\n=== HAREKET SEQUENCE TAMAMLANDI ===")
        
    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Hata oluştu: {e}")
    finally:
        # Güvenlik için motorları durdur
        vehicle.channels.overrides['1'] = 0
        vehicle.channels.overrides['2'] = 0
        print("Motorlar güvenli şekilde durduruldu")
        
        # Bağlantıyı kapat
        vehicle.close()
        print("Drone bağlantısı kapatıldı")

if __name__ == "__main__":
    main()
