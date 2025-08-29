#!/usr/bin/env python3
"""
Telemetri kayıt sistemi test scripti
"""

import os
import csv
import time
from datetime import datetime

def test_telemetry_recording():
    """Telemetri kayıt sistemini test eder"""
    
    # Test dizini oluştur
    test_dir = os.path.join(os.path.dirname(__file__), 'logs', 'test_recording')
    os.makedirs(test_dir, exist_ok=True)
    
    # CSV dosyası oluştur
    csv_path = os.path.join(test_dir, 'test_telemetry.csv')
    
    print(f"Test CSV dosyası oluşturuluyor: {csv_path}")
    
    with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        
        # Header yaz
        writer.writerow([
            'timestamp', 'lat', 'lon', 'groundspeed_mps', 'roll_deg', 'pitch_deg', 'heading_deg',
            'speed_setpoint_mps', 'heading_setpoint_deg', 'vehicle_mode', 'armed_status'
        ])
        
        # Test verileri yaz (5 satır)
        for i in range(5):
            timestamp = datetime.utcnow().isoformat() + 'Z'
            
            # Test verileri
            test_data = [
                timestamp,
                f"{41.0082 + i*0.001:.7f}",  # Test lat
                f"{28.9784 + i*0.001:.7f}",  # Test lon
                f"{1.5 + i*0.1:.3f}",        # Test hız
                f"{2.5 + i*0.5:.2f}",        # Test roll
                f"{1.2 + i*0.3:.2f}",        # Test pitch
                f"{45.0 + i*10:.1f}",        # Test heading
                f"{2.0 + i*0.2:.3f}",        # Test speed setpoint
                f"{50.0 + i*15:.1f}",        # Test heading setpoint
                "MANUAL" if i % 2 == 0 else "AUTO",  # Test mod
                "ARMED" if i % 3 == 0 else "DISARMED"  # Test armed
            ]
            
            writer.writerow(test_data)
            print(f"Test veri satırı {i+1} yazıldı: {test_data}")
            
            # Dosyayı flush et
            csvfile.flush()
            
            # 1 saniye bekle
            time.sleep(1)
    
    print(f"Test tamamlandı! {csv_path} dosyasını kontrol edin.")
    
    # Dosyayı oku ve kontrol et
    with open(csv_path, 'r', encoding='utf-8') as csvfile:
        reader = csv.reader(csvfile)
        rows = list(reader)
        print(f"CSV dosyasında {len(rows)} satır var:")
        for i, row in enumerate(rows):
            print(f"  Satır {i+1}: {row}")

if __name__ == "__main__":
    test_telemetry_recording()
