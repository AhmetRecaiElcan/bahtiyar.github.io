import sys
import os
import time
import threading
from PyQt5.QtCore import QUrl, Qt, QTimer, pyqtSignal, QObject, pyqtSlot
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
                             QTextEdit, QHBoxLayout, QMessageBox, QGridLayout,
                             QProgressBar, QGroupBox, QComboBox, QDoubleSpinBox, QDialog, QFormLayout)
from PyQt5.QtGui import QPixmap, QIcon, QTransform, QTextCursor, QTextCursor
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtWebChannel import QWebChannel

# Pixhawk bağlantısı için import edilecek (bu satırları aktif edin)
# from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
# import serial.tools.list_ports

class MapBridge(QObject):
    """Harita ve Python arasında köprü görevi gören sınıf"""
    addWaypointSignal = pyqtSignal(float, float, int)
    
    @pyqtSlot(float, float)
    def addWaypoint(self, lat, lng):
        """JavaScript'ten çağrılarak yeni bir waypoint ekler"""
        # Şu anki waypoint sayısını al ve 1 ekle
        current_count = self.parent().waypoints_count
        self.parent().waypoints_count += 1
        self.addWaypointSignal.emit(lat, lng, current_count + 1)
        
        # Ana GUI'ye bilgi ver
        self.parent().update_log(f"Waypoint {current_count + 1} eklendi: Lat={lat:.6f}, Lng={lng:.6f}")


class USVControlGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.vehicle = None  # Pixhawk bağlantısı
        self.waypoints = []  # Rota noktaları
        self.waypoints_count = 0
        self.current_heading = 0  # Şu anki yön
        self.is_connected = False  # Bağlantı durumu
        
        self.setWindowTitle("İnsansız Deniz Aracı Kontrol Paneli")
        self.setGeometry(100, 100, 1200, 800)
        
        # Ana layout
        main_layout = QVBoxLayout()
        
        # Başlık
        self.label = QLabel("İnsansız Deniz Aracı Kontrol Paneli")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-size: 22px; font-weight: bold; color: #2c3e50; margin: 10px;")
        main_layout.addWidget(self.label)
        
        # Üst kısım (harita ve telemetri)
        top_layout = QHBoxLayout()
        
        # Harita Paneli
        map_panel = QGroupBox("Harita ve Rota Planlama")
        map_panel.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #bdc3c7; border-radius: 5px; margin-top: 1ex; } "
                              "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 5px; }")
        map_layout = QVBoxLayout()
        
        # Harita ayarları ve kontroller
        map_controls = QHBoxLayout()
        self.clear_route_button = QPushButton("Rotayı Temizle")
        self.clear_route_button.clicked.connect(self.clear_route)
        self.send_mission_button = QPushButton("Rotayı Gönder")
        self.send_mission_button.clicked.connect(self.send_mission_to_vehicle)
        self.send_mission_button.setEnabled(False)  # Bağlantı kurulamadığı için devre dışı
        
        map_controls.addWidget(self.clear_route_button)
        map_controls.addWidget(self.send_mission_button)
        map_layout.addLayout(map_controls)
        
        # Harita görünümü
        self.map_view = QWebEngineView()
        self.map_bridge = MapBridge()
        self.map_bridge.setParent(self)
        self.map_bridge.addWaypointSignal.connect(self.add_waypoint_to_list)
        
        self.channel = QWebChannel()
        self.channel.registerObject("mapBridge", self.map_bridge)
        self.map_view.page().setWebChannel(self.channel)
        
        try:
            map_file_path = os.path.abspath("map.html")
            self.map_view.load(QUrl.fromLocalFile(map_file_path))
            self.map_view.loadFinished.connect(self.on_map_loaded)
        except Exception as e:
            self.update_log(f"Harita yüklenirken hata: {str(e)}")
        
        self.map_view.setMinimumSize(600, 450)
        map_layout.addWidget(self.map_view)
        map_panel.setLayout(map_layout)
        top_layout.addWidget(map_panel, 3)  # 3 birim genişlik
        
        # Telemetri Paneli
        telemetry_panel = QGroupBox("Telemetri ve Araç Durumu")
        telemetry_panel.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #bdc3c7; border-radius: 5px; margin-top: 1ex; } "
                              "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 5px; }")
        telemetry_layout = QVBoxLayout()
        
        # Pusula Görünümü
        compass_container = QVBoxLayout()
        compass_label_header = QLabel("Pusula")
        compass_label_header.setAlignment(Qt.AlignCenter)
        compass_container.addWidget(compass_label_header)
        
        self.compass_label = QLabel()
        self.compass_label.setFixedSize(150, 150)
        self.compass_label.setAlignment(Qt.AlignCenter)
        
        compass_path = "compass.png"
        if os.path.exists(compass_path):
            self.compass_pixmap = QPixmap(compass_path)
            if not self.compass_pixmap.isNull():
                self.compass_pixmap = self.compass_pixmap.scaled(130, 130, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.compass_label.setPixmap(self.compass_pixmap)
            else:
                self.update_log("Pusula görseli yüklenemedi.")
        else:
            self.update_log(f"Pusula görseli bulunamadı: {compass_path}")
            # Alternatif basit pusula oluştur
            self.compass_pixmap = QPixmap(130, 130)
            self.compass_pixmap.fill(Qt.white)
            # İleride dinamik olarak çizilecek
            
        compass_container.addWidget(self.compass_label)
        compass_container.setAlignment(Qt.AlignCenter)
        telemetry_layout.addLayout(compass_container)
        
        # Telemetri değerleri için grid oluştur
        telemetry_grid = QGridLayout()
        
        # Telemetri etiketleri ve değerleri
        labels = ["Hız:", "Yükseklik:", "Heading:", "Batarya:", "GPS Fix:", "Mod:"]
        self.telemetry_values = {}
        
        for i, label in enumerate(labels):
            telemetry_grid.addWidget(QLabel(label), i, 0)
            value_label = QLabel("--")
            value_label.setStyleSheet("font-weight: bold;")
            self.telemetry_values[label] = value_label
            telemetry_grid.addWidget(value_label, i, 1)
        
        # Batarya için progress bar
        self.battery_progress = QProgressBar()
        self.battery_progress.setRange(0, 100)
        self.battery_progress.setValue(0)
        telemetry_grid.addWidget(self.battery_progress, 3, 2)
        
        telemetry_layout.addLayout(telemetry_grid)
        telemetry_panel.setLayout(telemetry_layout)
        top_layout.addWidget(telemetry_panel, 1)  # 1 birim genişlik
        
        main_layout.addLayout(top_layout)
        
        # Orta Panel - Mod değiştirme ve bağlantı ayarları
        middle_panel = QHBoxLayout()
        
        # Bağlantı ayarları
        connection_group = QGroupBox("Bağlantı Ayarları")
        connection_group.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #bdc3c7; border-radius: 5px; margin-top: 1ex; } "
                                    "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 5px; }")
        connection_layout = QHBoxLayout()
        
        # Port seçimi
        self.port_combo = QComboBox()
        self.refresh_ports_button = QPushButton("Portları Yenile")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        
        # Baud rate seçimi
        self.baud_combo = QComboBox()
        for baud in ["57600", "115200", "921600"]:
            self.baud_combo.addItem(baud)
        self.baud_combo.setCurrentText("57600")  # Pixhawk için varsayılan
        
        # Bağlantı butonu
        self.connect_button = QPushButton("  BAĞLAN")
        self.connect_button.setStyleSheet("background-color: green; color: white; font-weight: bold")
        self.connect_button.setIcon(QIcon.fromTheme("network-connect"))
        self.connect_button.clicked.connect(self.connect_to_vehicle)
        
        connection_layout.addWidget(QLabel("Port:"))
        connection_layout.addWidget(self.port_combo)
        connection_layout.addWidget(self.refresh_ports_button)
        connection_layout.addWidget(QLabel("Baud:"))
        connection_layout.addWidget(self.baud_combo)
        connection_layout.addWidget(self.connect_button)
        connection_group.setLayout(connection_layout)
        middle_panel.addWidget(connection_group)
        
        # Mod değiştirme butonları
        mode_group = QGroupBox("Çalışma Modu")
        mode_group.setStyleSheet("QGroupBox { font-weight: bold; border: 2px solid #bdc3c7; border-radius: 5px; margin-top: 1ex; } "
                              "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 5px; }")
        mode_layout = QHBoxLayout()
        
        self.manual_button = QPushButton("MANUEL MOD")
        self.manual_button.setEnabled(False)
        self.auto_button = QPushButton("OTONOM MOD")
        self.auto_button.setEnabled(False)
        self.ready_button = QPushButton("HAZIR MOD")
        self.ready_button.setEnabled(False)
        
        for btn in [self.manual_button, self.auto_button, self.ready_button]:
            btn.setFixedHeight(50)
            mode_layout.addWidget(btn)
        
        self.manual_button.clicked.connect(lambda: self.set_vehicle_mode("MANUAL"))
        self.auto_button.clicked.connect(lambda: self.set_vehicle_mode("AUTO"))
        self.ready_button.clicked.connect(lambda: self.set_vehicle_mode("GUIDED"))
        
        mode_group.setLayout(mode_layout)
        middle_panel.addWidget(mode_group)
        
        main_layout.addLayout(middle_panel)
        
        # Alt kısım - Log ve durum paneli
        bottom_panel = QHBoxLayout()
        
        # Durum paneli
        status_group = QGroupBox("Durum")
        status_layout = QVBoxLayout()
        
        self.status_output = QTextEdit()
        self.status_output.setReadOnly(True)
        self.status_output.setMaximumHeight(80)
        self.status_output.setPlaceholderText("Durum bilgisi burada gösterilecek...")
        self.status_output.setStyleSheet("background-color: #f8f9fa;")
        
        status_layout.addWidget(self.status_output)
        status_group.setLayout(status_layout)
        bottom_panel.addWidget(status_group)
        
        # Log paneli
        log_group = QGroupBox("Sistem Logları")
        log_layout = QVBoxLayout()
        
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Sistem logları burada gösterilecek...")
        self.log.setStyleSheet("background-color: #f8f9fa;")
        
        log_layout.addWidget(self.log)
        log_group.setLayout(log_layout)
        bottom_panel.addWidget(log_group)
        
        main_layout.addLayout(bottom_panel)
        
        self.setLayout(main_layout)
        
        # Başlangıç mesajı
        self.update_status("Sistem başlatıldı. Pixhawk ile bağlantı kurulmadı.")
        self.update_log("Kontrol paneli hazır...")
        
        # Portları yenile
        self.refresh_ports()
        
        # Telemetri verileri için timer
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        # Bağlantı kurulduğunda başlayacak
        
        # Pusula güncellemesi için timer
        self.compass_timer = QTimer()
        self.compass_timer.timeout.connect(self.update_compass)
        self.compass_timer.start(100)  # 100ms'de bir güncelle
    
    def on_map_loaded(self):
        """Harita yüklendikten sonra çağrılır, JavaScript-Python bağlantısını başlatır"""
        self.map_view.page().runJavaScript("""
            // İki yönlü Harita-Python iletişimi için köprü
            new QWebChannel(qt.webChannelTransport, function(channel) {
                window.mapBridge = channel.objects.mapBridge;
                
                // Haritaya tıklama olayını dinle
                map.on('click', function(e) {
                    window.mapBridge.addWaypoint(e.latlng.lat, e.latlng.lng);
                });
                
                // Waypoint eklemek için fonksiyon
                window.addWaypointMarker = function(lat, lng, num) {
                    var marker = L.marker([lat, lng], {
                        title: "Waypoint " + num,
                        draggable: true
                    }).addTo(map);
                    
                    marker.bindPopup("Waypoint " + num + "<br>Lat: " + lat.toFixed(6) + "<br>Lng: " + lng.toFixed(6));
                    
                    // Waypointler arasında çizgi çiz
                    if (window.lastPoint) {
                        var line = L.polyline([window.lastPoint, [lat, lng]], {color: 'blue'}).addTo(map);
                    }
                    window.lastPoint = [lat, lng];
                    
                    return marker;
                };
                
                // Rotayı temizleme fonksiyonu
                window.clearRoute = function() {
                    map.eachLayer(function(layer) {
                        if (layer instanceof L.Marker || layer instanceof L.Polyline) {
                            map.removeLayer(layer);
                        }
                    });
                    window.lastPoint = null;
                    
                    // Temel harita katmanını korumak için
                    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                      maxZoom: 19,
                      attribution: '&copy; OpenStreetMap'
                    }).addTo(map);
                };
            });
        """)
        self.update_log("Harita başarıyla yüklendi.")
    
    def add_waypoint_to_list(self, lat, lng, num):
        """Waypoint'i listeye ekler ve haritada gösterir"""
        self.waypoints.append({"lat": lat, "lng": lng, "num": num})
        
        # JavaScript'e waypoint'i eklemesini söyle
        self.map_view.page().runJavaScript(f"window.addWaypointMarker({lat}, {lng}, {num});")
    
    def clear_route(self):
        """Rotayı temizler"""
        self.waypoints = []
        self.waypoints_count = 0
        self.map_view.page().runJavaScript("window.clearRoute();")
        self.update_log("Rota temizlendi.")
    
    def connect_to_vehicle(self):
        """Pixhawk ile bağlantı kurma"""
        if self.is_connected:
            # Bağlantıyı kes
            self.disconnect_from_vehicle()
            return
        
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "Bağlantı Hatası", "Lütfen bir port seçin.")
            return
        
        baud = int(self.baud_combo.currentText())
        
        self.update_status(f"Pixhawk ile bağlantı kuruluyor... ({port}, {baud} baud)")
        
        # Gerçek bağlantı için aşağıdaki kod bloğunu etkinleştirin
        """
        try:
            # Bağlantı için ayrı bir thread başlat
            threading.Thread(target=self._connect_thread, args=(port, baud)).start()
        except Exception as e:
            self.update_status(f"Bağlantı hatası: {str(e)}")
            self.update_log(f"HATA: {str(e)}")
        """
        
        # Şimdilik sadece simülasyon için bağlantı olduğunu varsayalım
        self.is_connected = True
        self.connect_button.setText("  BAĞLANTIYI KES")
        self.connect_button.setStyleSheet("background-color: red; color: white; font-weight: bold")
        
        # Mod butonlarını etkinleştir
        self.manual_button.setEnabled(True)
        self.auto_button.setEnabled(True)
        self.ready_button.setEnabled(True)
        self.send_mission_button.setEnabled(True)
        
        # Telemetri güncellemelerini başlat
        self.telemetry_timer.start(1000)  # 1 saniyede bir güncelle
        
        self.update_status("Pixhawk ile bağlantı kuruldu!")
        self.update_log("Araç bağlantısı başarılı.")
        
        # Simülasyon için bazı değerler ata
        self.telemetry_values["Hız:"].setText("0.0 m/s")
        self.telemetry_values["Yükseklik:"].setText("0.0 m")
        self.telemetry_values["Heading:"].setText("0°")
        self.telemetry_values["Batarya:"].setText("80%")
        self.telemetry_values["GPS Fix:"].setText("3D Fix")
        self.telemetry_values["Mod:"].setText("MANUAL")
        self.battery_progress.setValue(80)
    
    def _connect_thread(self, port, baud):
        """Ayrı bir thread'de Pixhawk ile bağlantı kurma (gerçek implementasyon)"""
        """
        try:
            # DroneKit ile bağlantı kur
            self.vehicle = connect(port, baud=baud, wait_ready=True)
            
            # Bağlantı başarılı olduğunda UI güncellemesi
            self.is_connected = True
            # UI güncellemelerini ana thread'de yap
            QApplication.instance().processEvents()
            self.connect_button.setText("  BAĞLANTIYI KES")
            self.connect_button.setStyleSheet("background-color: red; color: white; font-weight: bold")
            
            # Mod butonlarını etkinleştir
            self.manual_button.setEnabled(True)
            self.auto_button.setEnabled(True)
            self.ready_button.setEnabled(True)
            self.send_mission_button.setEnabled(True)
            
            # Telemetri güncellemelerini başlat
            self.telemetry_timer.start(1000)  # 1 saniyede bir güncelle
            
            self.update_status("Pixhawk ile bağlantı kuruldu!")
            self.update_log("Araç bağlantısı başarılı.")
            
        except Exception as e:
            self.update_status(f"Bağlantı hatası: {str(e)}")
            self.update_log(f"HATA: {str(e)}")
        """
        pass
    
    def disconnect_from_vehicle(self):
        """Pixhawk ile bağlantıyı kesme"""
        if not self.is_connected:
            return
            
        # Gerçek bağlantı için aşağıdaki kod bloğunu etkinleştirin
        """
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
        """
        
        self.is_connected = False
        self.connect_button.setText("  BAĞLAN")
        self.connect_button.setStyleSheet("background-color: green; color: white; font-weight: bold")
        
        # Mod butonlarını devre dışı bırak
        self.manual_button.setEnabled(False)
        self.auto_button.setEnabled(False)
        self.ready_button.setEnabled(False)
        self.send_mission_button.setEnabled(False)
        
        # Telemetri güncellemelerini durdur
        self.telemetry_timer.stop()
        
        # Telemetri değerlerini sıfırla
        for key in self.telemetry_values:
            self.telemetry_values[key].setText("--")
        self.battery_progress.setValue(0)
        
        self.update_status("Pixhawk ile bağlantı kesildi.")
        self.update_log("Araç bağlantısı kapatıldı.")
    
    def refresh_ports(self):
        """Seri portları yenile"""
        self.port_combo.clear()
        
        # Gerçek port listesi için aşağıdaki kod bloğunu etkinleştirin
        """
        ports = [p.device for p in serial.tools.list_ports.comports()]
        """
        
        # Simülasyon için bazı örnek portlar ekleyelim
        ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "COM3", "COM4"]
        
        for port in ports:
            self.port_combo.addItem(port)
        
        self.update_log(f"{len(ports)} port bulundu.")
    
    def update_telemetry(self):
        """Telemetri verilerini günceller"""
        if not self.is_connected:
            return
            
        # Gerçek telemetri için aşağıdaki kod bloğunu etkinleştirin
        """
        if self.vehicle:
            # Hız
            speed = self.vehicle.groundspeed
            self.telemetry_values["Hız:"].setText(f"{speed:.1f} m/s")
            
            # Yükseklik
            alt = self.vehicle.location.global_relative_frame.alt
            self.telemetry_values["Yükseklik:"].setText(f"{alt:.1f} m")
            
            # Yön
            heading = self.vehicle.heading
            self.current_heading = heading
            self.telemetry_values["Heading:"].setText(f"{heading}°")
            
            # Batarya
            if self.vehicle.battery:
                battery = self.vehicle.battery.level
                self.telemetry_values["Batarya:"].setText(f"{battery}%")
                self.battery_progress.setValue(battery)
            
            # GPS durumu
            if self.vehicle.gps_0:
                fix_type = self.vehicle.gps_0.fix_type
                if fix_type == 3:
                    fix_str = "3D Fix"
                elif fix_type == 2:
                    fix_str = "2D Fix"
                else:
                    fix_str = "No Fix"
                self.telemetry_values["GPS Fix:"].setText(fix_str)
            
            # Mod
            mode = self.vehicle.mode.name
            self.telemetry_values["Mod:"].setText(mode)
        """
        
        # Simülasyon için rastgele değerler üretelim
        import random
        
        speed = random.uniform(0.0, 5.0)
        self.telemetry_values["Hız:"].setText(f"{speed:.1f} m/s")
        
        alt = random.uniform(0.0, 0.5)  # Deniz yüzeyine yakın
        self.telemetry_values["Yükseklik:"].setText(f"{alt:.1f} m")
        
        # Yön rastgele değişir (gerçekçi olması için küçük değişimler)
        heading_change = random.uniform(-5, 5)
        self.current_heading = (self.current_heading + heading_change) % 360
        self.telemetry_values["Heading:"].setText(f"{int(self.current_heading)}°")
        
        # Batarya yavaşça azalır
        current_battery = int(self.telemetry_values["Batarya:"].text().replace("%", ""))
        battery = max(0, current_battery - random.uniform(0, 0.5))
        self.telemetry_values["Batarya:"].setText(f"{int(battery)}%")
        self.battery_progress.setValue(int(battery))
    
    def update_compass(self):
        """Pusula görselini günceller"""
        if not hasattr(self, 'compass_pixmap') or self.compass_pixmap.isNull():
            return
        
        # Orijinal görüntüyü al
        original = self.compass_pixmap
        
        # Yön kadar döndür
        transform = QTransform()
        transform.rotate(-self.current_heading)  # Negatif değer çünkü saat yönünde dönüş
        rotated = original.transformed(transform, Qt.SmoothTransformation)
        
        # Görseli güncelle
        self.compass_label.setPixmap(rotated)
    
    def set_vehicle_mode(self, mode):
        """Araç modunu değiştirir"""
        if not self.is_connected:
            return
        
        # Gerçek mod değişimi için aşağıdaki kod bloğunu etkinleştirin
        """
        if self.vehicle:
            try:
                self.vehicle.mode = VehicleMode(mode)
                self.update_status(f"{mode} modu aktifleştirildi.")
                self.update_log(f"Araç modu {mode} olarak değiştirildi.")
            except Exception as e:
                self.update_status(f"Mod değiştirme hatası: {str(e)}")
                self.update_log(f"HATA: {str(e)}")
        """
        
        # Simülasyon
        self.telemetry_values["Mod:"].setText(mode)
        self.update_status(f"{mode} modu aktifleştirildi.")
        self.update_log(f"Araç modu {mode} olarak değiştirildi.")
    
    def send_mission_to_vehicle(self):
        """Rotayı araca gönderir"""
        if not self.is_connected:
            self.update_status("Araç bağlı değil. Rotayı gönderemezsiniz.")
            return
            
        if not self.waypoints:
            self.update_status("Rota boş. Önce waypoint ekleyin.")
            return
            
        self.update_status("Rota araca gönderiliyor...")
        self.update_log(f"Toplam {len(self.waypoints)} waypoint gönderiliyor...")
        
        # Gerçek rota gönderimi için aşağıdaki kod bloğunu etkinleştirin
        """
        if self.vehicle:
            try:
                # Önce mevcut misyonu temizle
                self.vehicle.commands.clear()
                self.vehicle.commands.upload()
                
                # Yeni misyonu oluştur
                from dronekit import Command
                from pymavlink import mavutil
                
                for i, wp in enumerate(self.waypoints):
                    cmd = Command(
                        0, 0, 0, 
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 0, 0, 0, 0, 0,
                        wp["lat"], wp["lng"], 0
                    )
                    self.vehicle.commands.add(cmd)
                
                # Misyonu araca gönder
                self.vehicle.commands.upload()
                self.update_status("Rota başarıyla gönderildi!")
                self.update_log("Misyon araca yüklendi.")
            except Exception as e:
                self.update_status(f"Rota gönderme hatası: {str(e)}")
                self.update_log(f"HATA: {str(e)}")
        """
        
        # Simülasyon
        import time
        time.sleep(1)  # Gönderim simülasyonu için kısa bir bekleme
        self.update_status("Rota başarıyla gönderildi!")
        self.update_log(f"{len(self.waypoints)} waypoint içeren rota araca gönderildi.")
    
    def update_status(self, message):
        """Durum mesajını günceller"""
        self.status_output.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        # Otomatik kaydırma
        self.status_output.moveCursor(QTextCursor.End)
    
    def update_log(self, message):
        """Log mesajını günceller"""
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {message}")
        # Otomatik kaydırma
        self.log.moveCursor(QTextCursor.End)

# Ana uygulamayı başlatmak için kod eklenebilir
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = USVControlGUI()
    window.show()
    sys.exit(app.exec_())