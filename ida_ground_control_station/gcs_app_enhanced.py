#!/usr/bin/env python3
"""
İDA Yer Kontrol İstasyonu - Gelişmiş Versiyon
Gelişmiş koordinat debug, telemetri görselleştirme ve harita özellikleri ile
"""

import sys
import os
import time
import threading
import json
from PyQt5.QtCore import QUrl, Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QDateTime
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
                             QTextEdit, QHBoxLayout, QMessageBox, QGridLayout,
                             QProgressBar, QGroupBox, QComboBox, QDoubleSpinBox, 
                             QDialog, QFormLayout, QFrame, QTabWidget, QCheckBox)
from PyQt5.QtGui import QPixmap, QIcon, QTransform, QTextCursor, QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtWebChannel import QWebChannel

# Pixhawk bağlantısı için import edilecek
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import serial.tools.list_ports

# Local imports
from attitude_indicator import AttitudeIndicator
from telemetry_visualizer import AdvancedTelemetryPanel
from coordinate_debug import CoordinateDebugger

class EnhancedMapBridge(QObject):
    """Gelişmiş harita köprüsü - debug fonksiyonları ile"""
    updateVehiclePosition = pyqtSignal(float, float, float)
    updateVehicleData = pyqtSignal(dict)  # Gelişmiş telemetri verisi
    addWaypoint = pyqtSignal(float, float)
    waypoint_from_user = pyqtSignal(float, float)
    waypoint_removed = pyqtSignal(int)
    clearMap = pyqtSignal()
    updateDebugInfo = pyqtSignal(str, str)  # Debug bilgisi gönderme

    @pyqtSlot(float, float)
    def add_waypoint_to_ui(self, lat, lng):
        """JavaScript tarafından haritaya çift tıklandığında çağrılır."""
        self.waypoint_from_user.emit(lat, lng)
    
    @pyqtSlot(int)
    def remove_waypoint_from_ui(self, index):
        """JavaScript tarafından waypoint silindiğinde çağrılır."""
        self.waypoint_removed.emit(index)
    
    @pyqtSlot(str)
    def log_from_map(self, message):
        """Haritadan gelen log mesajları"""
        print(f"[Map Debug] {message}")

class EnhancedGCSApp(QWidget):
    """Gelişmiş YKİ Uygulaması"""
    
    log_message_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool, str)
    status_message_received = pyqtSignal(str)
    telemetry_data_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.vehicle = None
        self.is_connected = False
        self.waypoints = []
        self.current_heading = 0
        self.coordinate_debugger = CoordinateDebugger()
        
        # Telemetri verileri
        self.telemetry_data = {
            'speed': 0.0,
            'speed_setpoint': 0.0,
            'heading': 0.0,
            'heading_setpoint': 0.0,
            'rpm': 0.0,
            'thruster_forces': [0.0, 0.0, 0.0, 0.0],
            'thruster_status': ['OK', 'OK', 'OK', 'OK'],
            'position': {'lat': 0.0, 'lon': 0.0, 'alt': 0.0},
            'battery': {'voltage': 0.0, 'current': 0.0, 'level': 0},
            'gps': {'fix_type': 0, 'satellites': 0, 'hdop': 99.99}
        }
        
        self.initUI()
        self.setup_signals()

    def initUI(self):
        """Gelişmiş kullanıcı arayüzü"""
        self.setWindowTitle('İDA Yer Kontrol İstasyonu - Gelişmiş v2.0')
        self.setGeometry(100, 100, 1600, 1000)

        # Ana layout
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # Sol taraf - Kontrol panelleri (Tab'lı)
        left_widget = QTabWidget()
        left_widget.setMaximumWidth(450)

        # Tab 1: Bağlantı ve Temel Kontroller
        basic_tab = QWidget()
        self.setup_basic_controls_tab(basic_tab)
        left_widget.addTab(basic_tab, "🔌 Bağlantı")

        # Tab 2: Gelişmiş Telemetri
        telemetry_tab = QWidget()
        self.setup_telemetry_tab(telemetry_tab)
        left_widget.addTab(telemetry_tab, "📊 Telemetri")

        # Tab 3: Debug ve Test
        debug_tab = QWidget()
        self.setup_debug_tab(debug_tab)
        left_widget.addTab(debug_tab, "🐛 Debug")

        # Orta - Harita
        map_layout = QVBoxLayout()
        
        # Harita kontrolleri
        map_controls = QHBoxLayout()
        
        self.use_enhanced_map = QCheckBox("Gelişmiş Harita Kullan")
        self.use_enhanced_map.setChecked(True)
        self.use_enhanced_map.stateChanged.connect(self.toggle_map_version)
        map_controls.addWidget(self.use_enhanced_map)
        
        self.debug_map_button = QPushButton("🔍 Harita Debug")
        self.debug_map_button.clicked.connect(self.toggle_map_debug)
        map_controls.addWidget(self.debug_map_button)
        
        map_controls.addStretch()
        map_layout.addLayout(map_controls)
        
        # Web View (Harita)
        self.web_view = QWebEngineView()
        self.setup_web_channel()
        map_layout.addWidget(self.web_view)

        # Sağ taraf - Attitude ve Sistem Durumu
        right_layout = QVBoxLayout()
        right_frame = QFrame()
        right_frame.setFrameShape(QFrame.StyledPanel)
        right_frame.setLayout(right_layout)
        right_frame.setMaximumWidth(300)

        # Attitude Indicator (büyük)
        attitude_group = QGroupBox("Araç Attitude")
        attitude_layout = QVBoxLayout(attitude_group)
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedSize(200, 200)
        attitude_layout.addWidget(self.attitude_indicator)
        right_layout.addWidget(attitude_group)

        # Sistem durumu
        status_group = QGroupBox("Sistem Durumu")
        status_layout = QVBoxLayout(status_group)
        
        self.system_status_label = QLabel("Sistem hazır")
        self.system_status_label.setWordWrap(True)
        status_layout.addWidget(self.system_status_label)
        
        # Koordinat accuracy göstergesi
        coord_layout = QHBoxLayout()
        coord_layout.addWidget(QLabel("GPS Accuracy:"))
        self.coordinate_accuracy_label = QLabel("N/A")
        coord_layout.addWidget(self.coordinate_accuracy_label)
        status_layout.addLayout(coord_layout)
        
        # Debug bilgileri
        self.debug_info_text = QTextEdit()
        self.debug_info_text.setMaximumHeight(150)
        self.debug_info_text.setPlainText("Debug bilgileri burada görünecek...")
        status_layout.addWidget(self.debug_info_text)
        
        right_layout.addWidget(status_group)

        # Layout'ları ana layout'a ekle
        main_layout.addWidget(left_widget)
        main_layout.addWidget(self.web_view, 1)  # Harita en geniş alan
        main_layout.addWidget(right_frame)

        # Timer'lar
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        self.attitude_timer = QTimer(self)
        self.attitude_timer.timeout.connect(self.update_attitude)
        
        # Debug timer
        self.debug_timer = QTimer(self)
        self.debug_timer.timeout.connect(self.update_debug_info)
        self.debug_timer.start(2000)  # 2 saniyede bir

        # Portları ilk defa yükle
        self.refresh_ports()

    def setup_basic_controls_tab(self, tab):
        """Temel kontrol tab'ını oluştur"""
        layout = QVBoxLayout(tab)
        
        # Bağlantı grubu
        connection_group = QGroupBox("Bağlantı Ayarları")
        connection_layout = QVBoxLayout(connection_group)
        
        # Bağlantı tipi
        self.connection_type = QComboBox()
        self.connection_type.addItems(["Serial (USB)", "UDP (Kablosuz)", "TCP (WiFi)"])
        connection_layout.addWidget(QLabel("Bağlantı Tipi:"))
        connection_layout.addWidget(self.connection_type)
        
        # Port ve baud
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "38400", "19200", "9600"])
        self.baud_combo.setCurrentText("57600")
        
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("Port:"))
        port_layout.addWidget(self.port_combo)
        
        self.refresh_button = QPushButton("Portları Yenile")
        self.refresh_button.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.refresh_button)
        
        connection_layout.addLayout(port_layout)
        
        baud_layout = QHBoxLayout()
        baud_layout.addWidget(QLabel("Baud:"))
        baud_layout.addWidget(self.baud_combo)
        connection_layout.addLayout(baud_layout)
        
        # Bağlan butonu
        self.connect_button = QPushButton("BAĞLAN")
        self.connect_button.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_button)
        
        layout.addWidget(connection_group)
        
        # Görev kontrol grubu
        mission_group = QGroupBox("Görev Kontrolü")
        mission_layout = QVBoxLayout(mission_group)
        
        # Mod butonları
        mode_layout = QHBoxLayout()
        self.stabilize_button = QPushButton("STABILIZE")
        self.auto_button = QPushButton("AUTO")
        self.guided_button = QPushButton("GUIDED")
        
        mode_layout.addWidget(self.stabilize_button)
        mode_layout.addWidget(self.auto_button)
        mode_layout.addWidget(self.guided_button)
        mission_layout.addLayout(mode_layout)
        
        # Görev butonları
        self.upload_mission_button = QPushButton("Rotayı Gönder")
        self.read_mission_button = QPushButton("Rotayı Oku")
        self.clear_mission_button = QPushButton("Rotayı Temizle")
        
        mission_layout.addWidget(self.upload_mission_button)
        mission_layout.addWidget(self.read_mission_button)
        mission_layout.addWidget(self.clear_mission_button)
        
        layout.addWidget(mission_group)
        
        # Log paneli
        log_group = QGroupBox("Sistem Logları")
        log_layout = QVBoxLayout(log_group)
        self.log_display = QTextEdit()
        self.log_display.setMaximumHeight(200)
        log_layout.addWidget(self.log_display)
        layout.addWidget(log_group)
        
        layout.addStretch()
        
        # Event bağlantıları
        self.stabilize_button.clicked.connect(lambda: self.set_vehicle_mode("STABILIZE"))
        self.auto_button.clicked.connect(lambda: self.set_vehicle_mode("AUTO"))
        self.guided_button.clicked.connect(lambda: self.set_vehicle_mode("GUIDED"))
        self.upload_mission_button.clicked.connect(self.send_mission_to_vehicle)
        self.read_mission_button.clicked.connect(self.read_mission_from_vehicle)
        self.clear_mission_button.clicked.connect(self.clear_mission)

    def setup_telemetry_tab(self, tab):
        """Gelişmiş telemetri tab'ını oluştur"""
        layout = QVBoxLayout(tab)
        
        # Gelişmiş telemetri paneli
        self.advanced_telemetry = AdvancedTelemetryPanel()
        layout.addWidget(self.advanced_telemetry)

    def setup_debug_tab(self, tab):
        """Debug tab'ını oluştur"""
        layout = QVBoxLayout(tab)
        
        # Koordinat debug grubu
        coord_debug_group = QGroupBox("Koordinat Debug")
        coord_debug_layout = QVBoxLayout(coord_debug_group)
        
        # Test butonları
        self.test_coords_button = QPushButton("Koordinat Testini Çalıştır")
        self.test_coords_button.clicked.connect(self.run_coordinate_tests)
        coord_debug_layout.addWidget(self.test_coords_button)
        
        self.test_known_location_button = QPushButton("Bilinen Konum Testi")
        self.test_known_location_button.clicked.connect(self.test_known_location)
        coord_debug_layout.addWidget(self.test_known_location_button)
        
        # Test sonuçları
        self.test_results_text = QTextEdit()
        self.test_results_text.setMaximumHeight(200)
        coord_debug_layout.addWidget(self.test_results_text)
        
        layout.addWidget(coord_debug_group)
        
        # Simülasyon grubu
        sim_group = QGroupBox("Veri Simülasyonu")
        sim_layout = QVBoxLayout(sim_group)
        
        self.simulate_data_checkbox = QCheckBox("Test Verisi Simüle Et")
        self.simulate_data_checkbox.stateChanged.connect(self.toggle_simulation)
        sim_layout.addWidget(self.simulate_data_checkbox)
        
        # Simülasyon parametreleri
        sim_params_layout = QGridLayout()
        sim_params_layout.addWidget(QLabel("Merkez Lat:"), 0, 0)
        self.sim_lat_spin = QDoubleSpinBox()
        self.sim_lat_spin.setRange(-90, 90)
        self.sim_lat_spin.setValue(41.015137)
        self.sim_lat_spin.setDecimals(6)
        sim_params_layout.addWidget(self.sim_lat_spin, 0, 1)
        
        sim_params_layout.addWidget(QLabel("Merkez Lon:"), 1, 0)
        self.sim_lon_spin = QDoubleSpinBox()
        self.sim_lon_spin.setRange(-180, 180)
        self.sim_lon_spin.setValue(28.979530)
        self.sim_lon_spin.setDecimals(6)
        sim_params_layout.addWidget(self.sim_lon_spin, 1, 1)
        
        sim_layout.addLayout(sim_params_layout)
        layout.addWidget(sim_group)
        
        layout.addStretch()

    def setup_web_channel(self):
        """Web channel kurul"""
        self.bridge = EnhancedMapBridge(self)
        self.channel = QWebChannel(self.web_view.page())
        self.web_view.page().setWebChannel(self.channel)
        self.channel.registerObject('py_bridge', self.bridge)
        
        # İlk harita dosyasını yükle
        self.load_map()

    def load_map(self):
        """Seçilen harita versiyonunu yükle"""
        if self.use_enhanced_map.isChecked():
            map_file = 'map_enhanced.html'
        else:
            map_file = 'map.html'
        
        map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), map_file))
        if os.path.exists(map_path):
            self.web_view.setUrl(QUrl.fromLocalFile(map_path))
            self.log_message_received.emit(f"Harita yüklendi: {map_file}")
        else:
            self.log_message_received.emit(f"Harita dosyası bulunamadı: {map_file}")

    def setup_signals(self):
        """Sinyal bağlantılarını kur"""
        self.log_message_received.connect(self.update_log_safe)
        self.connection_status_changed.connect(self.on_connection_status_changed)
        self.status_message_received.connect(self.update_status_safe)
        self.telemetry_data_received.connect(self.advanced_telemetry.update_telemetry)
        
        # Bridge sinyalleri
        self.bridge.waypoint_from_user.connect(self.add_waypoint_to_list)
        self.bridge.waypoint_removed.connect(self.remove_waypoint_from_list)

    def toggle_connection(self):
        """Bağlantıyı aç/kapat"""
        if self.is_connected:
            self.disconnect_from_vehicle()
        else:
            self.connect_to_vehicle()

    def connect_to_vehicle(self):
        """Araç bağlantısı kur"""
        # Simülasyon modunda ise simülasyonu başlat
        if hasattr(self, 'simulate_data_checkbox') and self.simulate_data_checkbox.isChecked():
            self.start_simulation()
            return
        
        # Gerçek araç bağlantısı
        connection_type = self.connection_type.currentText()
        
        if connection_type == "UDP (Kablosuz)":
            connection_string = f"udp:127.0.0.1:14550"
            self.status_message_received.emit(f"UDP bağlantısı kuruluyor... ({connection_string})")
        elif connection_type == "TCP (WiFi)":
            connection_string = f"tcp:192.168.1.100:5760"
            self.status_message_received.emit(f"TCP bağlantısı kuruluyor... ({connection_string})")
        else:  # Serial
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if not port or "bulunamadı" in port:
                self.status_message_received.emit("Geçerli bir port seçilmedi.")
                return
            connection_string = port
            self.status_message_received.emit(f"Serial bağlantı kuruluyor... ({port}, {baud})")
        
        threading.Thread(target=self._connect_thread, args=(connection_string, connection_type), daemon=True).start()

    def _connect_thread(self, connection_string, connection_type):
        """Bağlantı thread'i"""
        try:
            if connection_type == "Serial (USB)":
                baud = int(self.baud_combo.currentText())
                # Telemetri modülleri için optimize edilmiş ayarlar
                self.vehicle = connect(connection_string, baud=baud, wait_ready=True, 
                                     heartbeat_timeout=15, timeout=60)
            else:  # UDP veya TCP
                # Kablosuz bağlantılar için daha kısa timeout
                self.vehicle = connect(connection_string, wait_ready=True, 
                                     heartbeat_timeout=10, timeout=30)
            
            self.connection_status_changed.emit(True, f"Pixhawk'a başarıyla bağlanıldı! ({connection_type})")
            
            # Dinleyicileri ekle
            self.vehicle.add_attribute_listener('location.global_relative_frame', self.location_callback)
            self.vehicle.add_attribute_listener('heading', self.heading_callback)

        except Exception as e:
            self.connection_status_changed.emit(False, f"Bağlantı hatası: {e}")
            self.log_message_received.emit(f"Pixhawk bağlantısı başarısız: {e}")
            # Bağlantı başarısız olsa bile uygulama çalışmaya devam etsin
            self.vehicle = None

    def disconnect_from_vehicle(self):
        """Araç bağlantısını kes"""
        if hasattr(self, 'simulation_timer'):
            self.simulation_timer.stop()
        self.is_connected = False
        self.connection_status_changed.emit(False, "Bağlantı kesildi")

    def start_simulation(self):
        """Veri simülasyonunu başlat"""
        import random
        import math
        
        self.simulation_timer = QTimer()
        self.simulation_step = 0
        
        def simulate_telemetry():
            self.simulation_step += 1
            
            # Simüle edilmiş konum (dairesel hareket)
            center_lat = self.sim_lat_spin.value()
            center_lon = self.sim_lon_spin.value()
            radius = 0.001  # Yaklaşık 100 metre
            angle = (self.simulation_step * 2) % 360
            
            lat = center_lat + radius * math.cos(math.radians(angle))
            lon = center_lon + radius * math.sin(math.radians(angle))
            heading = angle
            
            # Telemetri verisini güncelle
            self.telemetry_data.update({
                'speed': random.uniform(2, 8),
                'speed_setpoint': 5.0,
                'heading': heading,
                'heading_setpoint': (heading + random.uniform(-10, 10)) % 360,
                'rpm': random.uniform(1200, 2000),
                'thruster_forces': [
                    random.uniform(-50, 80),
                    random.uniform(-50, 80),
                    random.uniform(-50, 80),
                    random.uniform(-50, 80)
                ],
                'position': {'lat': lat, 'lon': lon, 'alt': 0.5}
            })
            
            # Sinyalleri gönder
            self.bridge.updateVehiclePosition.emit(lat, lon, heading)
            self.telemetry_data_received.emit(self.telemetry_data)
            
            # Debug bilgisini güncelle
            precision = self.coordinate_debugger.validate_coordinate_precision(lat, lon)
            accuracy_text = f"{precision['overall_accuracy_meters']:.3f}m"
            self.coordinate_accuracy_label.setText(accuracy_text)
            
        self.simulation_timer.timeout.connect(simulate_telemetry)
        self.simulation_timer.start(500)  # 500ms = 2 Hz
        
        self.is_connected = True
        self.connection_status_changed.emit(True, "Simülasyon bağlantısı aktif")

    def toggle_simulation(self, state):
        """Simülasyonu aç/kapat"""
        if state == Qt.Checked and not self.is_connected:
            self.start_simulation()
        elif state == Qt.Unchecked and self.is_connected:
            self.disconnect_from_vehicle()

    def toggle_map_version(self, state):
        """Harita versiyonunu değiştir"""
        self.load_map()

    def toggle_map_debug(self):
        """Harita debug panelini aç/kapat"""
        # JavaScript ile debug panelini aç/kapat
        script = "if(typeof toggleDebugPanel === 'function') toggleDebugPanel();"
        self.web_view.page().runJavaScript(script)

    def run_coordinate_tests(self):
        """Koordinat testlerini çalıştır"""
        # Test koordinatları
        test_coords = [
            (41.015137, 28.979530),  # İstanbul merkez
            (41.020000, 28.980000),  # Yakın nokta
            (41.010000, 28.985000),  # Yakın nokta
        ]
        
        # Test çalıştır
        results = self.coordinate_debugger.test_coordinate_accuracy(test_coords, tolerance_meters=0.1)
        
        # Sonuçları göster
        result_text = f"""Koordinat Test Sonuçları:
Toplam test: {results['test_count']}
Başarılı: {results['passed']}
Başarısız: {results['failed']}
Maksimum hata: {results['max_error']:.6f} metre
Ortalama hata: {results['avg_error']:.6f} metre

Detaylar:
"""
        
        for detail in results['details']:
            result_text += f"Nokta {detail['point']}: {detail['error_meters']:.6f}m - {'✅' if detail['passed'] else '❌'}\n"
        
        self.test_results_text.setPlainText(result_text)
        self.log_message_received.emit("Koordinat testleri tamamlandı")

    def test_known_location(self):
        """Bilinen konumu test et"""
        # Galata Kulesi koordinatları
        test_lat, test_lon = 41.025631, 28.974298
        
        # Haritaya gönder
        self.bridge.updateVehiclePosition.emit(test_lat, test_lon, 0)
        
        # Precision analizi
        precision = self.coordinate_debugger.validate_coordinate_precision(test_lat, test_lon)
        
        result_text = f"""Bilinen Konum Testi - Galata Kulesi:
Koordinatlar: {test_lat}, {test_lon}
Precision: {precision['overall_accuracy_meters']}m
Denizcilik için yeterli: {'✅' if precision['sufficient_for_marine'] else '❌'}
"""
        
        self.test_results_text.setPlainText(result_text)
        self.log_message_received.emit("Bilinen konum testi tamamlandı")

    def update_debug_info(self):
        """Debug bilgilerini güncelle"""
        if self.is_connected:
            debug_info = f"""Debug Bilgileri:
Bağlantı: {'Simülasyon' if hasattr(self, 'simulation_timer') else 'Gerçek'}
Telemetri Frekansı: ~2 Hz
Son Konum: {self.telemetry_data['position']['lat']:.6f}, {self.telemetry_data['position']['lon']:.6f}
Heading: {self.telemetry_data['heading']:.1f}°
Hız: {self.telemetry_data['speed']:.1f} m/s
"""
            self.debug_info_text.setPlainText(debug_info)

    # Telemetri güncelleme metodları
    def update_telemetry(self):
        """Telemetri güncelleme"""
        if not self.is_connected or not self.vehicle:
            return

        try:
            # Temel telemetri verileri
            speed = self.vehicle.groundspeed
            alt = self.vehicle.location.global_relative_frame.alt
            heading = self.vehicle.heading
            mode = self.vehicle.mode.name
            
            # Telemetri verisini güncelle
            self.telemetry_data.update({
                'speed': speed,
                'heading': heading,
                'position': {
                    'lat': self.vehicle.location.global_relative_frame.lat,
                    'lon': self.vehicle.location.global_relative_frame.lon,
                    'alt': alt
                }
            })
            
            # Gelişmiş telemetri paneline gönder
            self.telemetry_data_received.emit(self.telemetry_data)

        except Exception as e:
            self.log_message_received.emit(f"Telemetri okuma hatası: {e}")

    def update_attitude(self):
        """Attitude güncelleme"""
        if not self.is_connected or not self.vehicle:
            return
        
        try:
            # Dronekit'ten attitude bilgilerini al
            if hasattr(self.vehicle, 'attitude'):
                pitch = self.vehicle.attitude.pitch  # radyan
                roll = self.vehicle.attitude.roll    # radyan
                self.attitude_indicator.set_attitude(pitch, roll)
        except Exception as e:
            self.log_message_received.emit(f"Attitude güncelleme hatası: {e}")

    @pyqtSlot(bool, str)
    def on_connection_status_changed(self, connected, message):
        """Bağlantı durumu değişti"""
        self.is_connected = connected
        self.system_status_label.setText(message)
        
        if connected:
            self.connect_button.setText("BAĞLANTIYI KES")
            self.connect_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            # Timer'ları başlat
            self.telemetry_timer.start(1000)
            self.attitude_timer.start(100)
        else:
            self.connect_button.setText("BAĞLAN")
            self.connect_button.setStyleSheet("")
            # Timer'ları durdur
            self.telemetry_timer.stop()
            self.attitude_timer.stop()
            self.vehicle = None

    @pyqtSlot(str)
    def update_log_safe(self, message):
        """Log güncelleme"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_display.append(f"[{timestamp}] {message}")

    @pyqtSlot(str)
    def update_status_safe(self, message):
        """Durum güncelleme"""
        self.system_status_label.setText(message)

    def add_waypoint_to_list(self, lat, lng):
        """Waypoint ekleme"""
        waypoint_num = len(self.waypoints) + 1
        self.waypoints.append({"lat": lat, "lng": lng, "num": waypoint_num})
        self.log_message_received.emit(f"Waypoint #{waypoint_num} eklendi: {lat:.6f}, {lng:.6f}")
        self.bridge.addWaypoint.emit(lat, lng)

    def remove_waypoint_from_list(self, index):
        """Waypoint silme"""
        if 0 <= index < len(self.waypoints):
            removed_wp = self.waypoints.pop(index)
            self.log_message_received.emit(f"Waypoint #{removed_wp['num']} silindi")

    def set_vehicle_mode(self, mode):
        """Araç modu değiştirme (placeholder)"""
        self.log_message_received.emit(f"Mod değiştirildi: {mode}")

    def send_mission_to_vehicle(self):
        """Görev gönderme (placeholder)"""
        self.log_message_received.emit("Görev gönderme henüz aktif değil")

    def read_mission_from_vehicle(self):
        """Görev okuma (placeholder)"""
        self.log_message_received.emit("Görev okuma henüz aktif değil")

    def clear_mission(self):
        """Görev temizleme"""
        self.waypoints = []
        self.bridge.clearMap.emit()
        self.log_message_received.emit("Görev ve harita temizlendi")

    def location_callback(self, vehicle, attr_name, value):
        """Konum callback'i"""
        if value and self.vehicle.heading is not None:
            self.bridge.updateVehiclePosition.emit(value.lat, value.lon, self.vehicle.heading)

    def heading_callback(self, vehicle, attr_name, value):
        """Heading callback'i"""
        self.current_heading = value

    def refresh_ports(self):
        """Serial portları yenile"""
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if not ports:
            self.port_combo.addItem("Port bulunamadı")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gcs_app = EnhancedGCSApp()
    gcs_app.show()
    sys.exit(app.exec_()) 