#!/usr/bin/env python3
"""
Basit YKİ Test Uygulaması
Port listesi ve temel bağlantı testleri için
"""

import sys
import os
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, 
                             QLabel, QComboBox, QTextEdit, QHBoxLayout, QGroupBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import serial.tools.list_ports

class SimpleGCSTest(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.refresh_ports()
        
    def initUI(self):
        self.setWindowTitle('YKİ Basit Test - Port Kontrolü')
        self.setGeometry(100, 100, 600, 400)
        
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Başlık
        title = QLabel("🚢 İDA Yer Kontrol İstasyonu - Port Test")
        title.setFont(QFont('Arial', 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Port seçim grubu
        port_group = QGroupBox("Serial Port Seçimi")
        port_layout = QVBoxLayout(port_group)
        
        # Port listesi
        port_select_layout = QHBoxLayout()
        port_select_layout.addWidget(QLabel("Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(300)
        port_select_layout.addWidget(self.port_combo)
        
        self.refresh_button = QPushButton("🔄 Yenile")
        self.refresh_button.clicked.connect(self.refresh_ports)
        port_select_layout.addWidget(self.refresh_button)
        
        port_layout.addLayout(port_select_layout)
        
        # Baud seçimi
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("57600")
        baud_layout.addWidget(self.baud_combo)
        baud_layout.addStretch()
        port_layout.addLayout(baud_layout)
        
        # Test butonları
        button_layout = QHBoxLayout()
        self.test_port_button = QPushButton("🔌 Port Testi")
        self.test_port_button.clicked.connect(self.test_selected_port)
        button_layout.addWidget(self.test_port_button)
        
        self.simulate_button = QPushButton("🎮 Simülasyon Modu")
        self.simulate_button.clicked.connect(self.start_simulation)
        button_layout.addWidget(self.simulate_button)
        
        port_layout.addLayout(button_layout)
        layout.addWidget(port_group)
        
        # Sonuç alanı
        results_group = QGroupBox("Test Sonuçları")
        results_layout = QVBoxLayout(results_group)
        
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setMaximumHeight(200)
        results_layout.addWidget(self.results_text)
        
        layout.addWidget(results_group)
        
        # Durum bilgisi
        self.status_label = QLabel("Hazır - Port seçin ve test edin")
        self.status_label.setStyleSheet("padding: 10px; background-color: #f0f0f0; border-radius: 5px;")
        layout.addWidget(self.status_label)
        
    def refresh_ports(self):
        """Serial portları yenile"""
        self.port_combo.clear()
        
        try:
            ports = list(serial.tools.list_ports.comports())
            self.log(f"Port tarama tamamlandı. {len(ports)} port bulundu.")
            
            if ports:
                for port in ports:
                    display_text = f"{port.device}"
                    if port.description and port.description != "n/a":
                        display_text += f" ({port.description})"
                    if port.manufacturer:
                        display_text += f" - {port.manufacturer}"
                    
                    self.port_combo.addItem(display_text, port.device)
                    self.log(f"Port eklendi: {display_text}")
                
                self.status_label.setText(f"✅ {len(ports)} port bulundu")
                self.status_label.setStyleSheet("padding: 10px; background-color: #d4edda; border-radius: 5px; color: #155724;")
            else:
                self.port_combo.addItem("Hiç port bulunamadı", None)
                self.status_label.setText("⚠️ Hiç port bulunamadı")
                self.status_label.setStyleSheet("padding: 10px; background-color: #fff3cd; border-radius: 5px; color: #856404;")
                
        except Exception as e:
            self.log(f"Port tarama hatası: {e}")
            self.status_label.setText(f"❌ Port tarama hatası: {e}")
            self.status_label.setStyleSheet("padding: 10px; background-color: #f8d7da; border-radius: 5px; color: #721c24;")
    
    def test_selected_port(self):
        """Seçilen portu test et"""
        if self.port_combo.count() == 0 or self.port_combo.currentData() is None:
            self.log("❌ Test edilecek port yok")
            return
        
        port_device = self.port_combo.currentData()
        baud_rate = int(self.baud_combo.currentText())
        
        self.log(f"\n🔍 Port testi başlatılıyor...")
        self.log(f"Port: {port_device}")
        self.log(f"Baud Rate: {baud_rate}")
        
        try:
            import serial
            import time
            
            # Port açmayı dene
            self.log("📡 Port açılıyor...")
            ser = serial.Serial(port_device, baud_rate, timeout=2)
            
            self.log("✅ Port başarıyla açıldı")
            self.log(f"Port bilgileri: {ser}")
            
            # Kısa bir test
            time.sleep(0.1)
            
            # Gelen veri var mı kontrol et
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                self.log(f"📨 Port'tan veri alındı: {len(data)} byte")
                self.log(f"Veri önizleme: {data[:50]}...")
            else:
                self.log("📭 Port'ta bekleyen veri yok")
            
            # Basit test verisi gönder
            test_msg = b"AT\r\n"
            ser.write(test_msg)
            self.log(f"📤 Test mesajı gönderildi: {test_msg}")
            
            # Yanıt bekle
            time.sleep(0.5)
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting)
                self.log(f"📥 Yanıt alındı: {response}")
            else:
                self.log("📭 Yanıt alınamadı (normal olabilir)")
            
            ser.close()
            self.log("✅ Port testi tamamlandı - Port kapatıldı")
            
            self.status_label.setText(f"✅ Port testi başarılı: {port_device}")
            self.status_label.setStyleSheet("padding: 10px; background-color: #d4edda; border-radius: 5px; color: #155724;")
            
        except Exception as e:
            self.log(f"❌ Port testi başarısız: {e}")
            self.status_label.setText(f"❌ Port testi başarısız")
            self.status_label.setStyleSheet("padding: 10px; background-color: #f8d7da; border-radius: 5px; color: #721c24;")
    
    def start_simulation(self):
        """Simülasyon modunu başlat"""
        self.log("\n🎮 Simülasyon modu başlatılıyor...")
        
        # Test koordinatları
        test_coords = [
            {"lat": 41.025631, "lon": 28.974298, "heading": 0, "name": "Galata Kulesi"},
            {"lat": 41.008238, "lon": 28.978359, "heading": 90, "name": "Eminönü"},
            {"lat": 41.040556, "lon": 29.000000, "heading": 180, "name": "Üsküdar"}
        ]
        
        self.log("📍 Test koordinatları hazırlandı:")
        for coord in test_coords:
            self.log(f"  • {coord['name']}: {coord['lat']:.6f}, {coord['lon']:.6f}, {coord['heading']}°")
        
        # Koordinat debug testi çalıştır
        try:
            from coordinate_debug import CoordinateDebugger
            debugger = CoordinateDebugger()
            
            self.log("\n🔍 Koordinat doğrulama testi...")
            
            test_points = [(coord["lat"], coord["lon"]) for coord in test_coords]
            results = debugger.test_coordinate_accuracy(test_points, tolerance_meters=0.1)
            
            self.log(f"Test sonuçları:")
            self.log(f"  Toplam: {results['test_count']}")
            self.log(f"  Başarılı: {results['passed']} ✅")
            self.log(f"  Başarısız: {results['failed']} ❌")
            self.log(f"  Maksimum hata: {results['max_error']:.6f}m")
            self.log(f"  Ortalama hata: {results['avg_error']:.6f}m")
            
            if results['failed'] == 0:
                self.log("✅ Tüm koordinat testleri başarılı!")
                self.status_label.setText("✅ Simülasyon ve koordinat testleri başarılı")
                self.status_label.setStyleSheet("padding: 10px; background-color: #d4edda; border-radius: 5px; color: #155724;")
            else:
                self.log("⚠️ Bazı koordinat testleri başarısız")
                
        except Exception as e:
            self.log(f"❌ Koordinat test hatası: {e}")
    
    def log(self, message):
        """Log mesajı ekle"""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.results_text.append(f"[{timestamp}] {message}")
        
        # Scroll to bottom
        scrollbar = self.results_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

def main():
    app = QApplication(sys.argv)
    
    # Basit test penceresi
    test_window = SimpleGCSTest()
    test_window.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 