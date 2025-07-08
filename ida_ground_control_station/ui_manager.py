import os
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QComboBox, QLabel, QFrame, QGridLayout, QProgressBar, QTextEdit)
from PyQt5.QtGui import QFont, QPixmap, QTransform
from PyQt5.QtCore import Qt, QUrl, QSize
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from attitude_indicator import AttitudeIndicator

class UIManager(QWidget):
    def __init__(self, main_app, parent=None):
        super().__init__(parent)
        self.main_app = main_app
        self.setWindowTitle("Yer Kontrol İstasyonu")
        self.setGeometry(100, 100, 1600, 900)

        # Ana layout
        main_layout = QHBoxLayout(self)
        
        # Sol ve Sağ panelleri oluştur
        sidebar = self._create_sidebar()
        map_view = self._create_map_view()

        main_layout.addWidget(sidebar)
        main_layout.addWidget(map_view, 1) # Haritanın daha fazla yer kaplaması için

    def _create_panel_title(self, title):
        """Paneller için standart bir başlık etiketi oluşturur."""
        title_label = QLabel(title)
        title_label.setFont(QFont("Arial", 14, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        return title_label
        
    def _create_sidebar(self):
        """Sol taraftaki kontrol panelini (sidebar) oluşturur."""
        sidebar_frame = QFrame()
        sidebar_frame.setFrameShape(QFrame.StyledPanel)
        sidebar_layout = QVBoxLayout(sidebar_frame)
        sidebar_frame.setMaximumWidth(450)

        # Panelleri oluştur ve ekle
        sidebar_layout.addWidget(self._create_status_panel())
        sidebar_layout.addWidget(self._create_mission_control_panel())
        sidebar_layout.addWidget(self._create_connection_panel())
        sidebar_layout.addWidget(self._create_telemetry_panel())
        sidebar_layout.addWidget(self._create_log_panel())
        
        sidebar_layout.addStretch(1) 
        return sidebar_frame

    def _create_status_panel(self):
        """Genel durum ve mod bilgisi için panel oluşturur."""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        layout.addWidget(self._create_panel_title("Durum"))
        self.status_label = QLabel("Bağlantı bekleniyor...")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        return frame

    def _create_mission_control_panel(self):
        """Görev ve mod kontrol butonları için panel oluşturur."""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QGridLayout(frame)
        layout.addWidget(self._create_panel_title("Görev Kontrol"), 0, 0, 1, 2)

        self.arm_button = QPushButton("ARM")
        self.disarm_button = QPushButton("DISARM")
        self.start_mission_button = QPushButton("Görevi Başlat")
        self.clear_mission_button = QPushButton("Görevi Temizle")
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["STABILIZE", "AUTO", "GUIDED", "RTL"])
        self.set_mode_button = QPushButton("Mod Değiştir")
        
        self.set_mode_buttons_enabled(False) # Başlangıçta pasif

        layout.addWidget(self.arm_button, 1, 0)
        layout.addWidget(self.disarm_button, 1, 1)
        layout.addWidget(self.start_mission_button, 2, 0)
        layout.addWidget(self.clear_mission_button, 2, 1)
        layout.addWidget(self.mode_combo, 3, 0)
        layout.addWidget(self.set_mode_button, 3, 1)
        
        return frame

    def _create_connection_panel(self):
        """Bağlantı ayarları için panel oluşturur."""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QGridLayout(frame)
        layout.addWidget(self._create_panel_title("Bağlantı"), 0, 0, 1, 2)
        
        self.port_combo = QComboBox()
        self.port_combo.addItems(["/dev/tty.usbmodem14201", "COM3", "COM4", "/dev/ttyACM0"]) # Örnek portlar
        
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200"])
        
        self.connect_button = QPushButton("Bağlan")
        
        layout.addWidget(QLabel("Port:"), 1, 0)
        layout.addWidget(self.port_combo, 1, 1)
        layout.addWidget(QLabel("Baud:"), 2, 0)
        layout.addWidget(self.baud_combo, 2, 1)
        layout.addWidget(self.connect_button, 3, 0, 1, 2)
        
        return frame

    def _create_telemetry_panel(self):
        """Telemetri verilerini gösteren paneli oluşturur."""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        main_layout = QVBoxLayout(frame)
        main_layout.addWidget(self._create_panel_title("Telemetri Verileri"))

        # Yapay Ufuk Göstergesi
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedSize(150, 150)
        
        # Grid Layout (sağdaki metin verileri)
        grid_layout = QGridLayout()
        self.telemetry_labels = {}
        telemetry_keys = {
            "latitude": "Enlem:", "longitude": "Boylam:",
            "altitude": "Yükseklik:", "groundspeed": "Hız:",
            "heading": "Yön:", "roll": "Yatış (Roll):", "pitch": "Yunuslama (Pitch):",
            "voltage": "Voltaj:", "current": "Akım:"
        }
        
        row = 0
        for key, text in telemetry_keys.items():
            label = QLabel(text)
            value = QLabel("N/A")
            grid_layout.addWidget(label, row, 0)
            grid_layout.addWidget(value, row, 1)
            self.telemetry_labels[key] = value
            row += 1
            
        telemetry_content_layout = QHBoxLayout()
        telemetry_content_layout.addWidget(self.attitude_indicator)
        telemetry_content_layout.addLayout(grid_layout)
        
        main_layout.addLayout(telemetry_content_layout)
        
        self.battery_progress = QProgressBar()
        main_layout.addWidget(QLabel("Batarya Seviyesi"))
        main_layout.addWidget(self.battery_progress)
        
        return frame

    def _create_log_panel(self):
        """Log mesajları için panel oluşturur."""
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(frame)
        layout.addWidget(self._create_panel_title("Loglar"))
        self.log_browser = QTextEdit()
        self.log_browser.setReadOnly(True)
        layout.addWidget(self.log_browser)
        return frame

    def _create_map_view(self):
        """Harita görünümünü oluşturur ve Python ile JS arasında köprü kurar."""
        self.map_view = QWebEngineView()
        self.channel = QWebChannel()
        self.channel.registerObject("py_bridge", self.main_app)
        self.map_view.page().setWebChannel(self.channel)
        
        # Haritanın yerel HTML dosyasını yükle
        map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'map.html'))
        self.map_view.setUrl(QUrl.fromLocalFile(map_path))
        return self.map_view
    
    def set_mode_buttons_enabled(self, enabled):
        """Mod ile ilgili butonların aktif/pasif durumunu ayarlar."""
        self.arm_button.setEnabled(enabled)
        self.disarm_button.setEnabled(enabled)
        self.start_mission_button.setEnabled(enabled)
        self.set_mode_button.setEnabled(enabled)

    def append_log(self, message):
        """Log paneline mesaj ekler."""
        self.log_browser.append(message)
        
    def update_telemetry_display(self, data):
        """Gelen telemetri verisiyle arayüzü günceller."""
        # Metin tabanlı telemetri verileri
        self.telemetry_labels["latitude"].setText(f"{data.get('lat', 0):.6f}")
        self.telemetry_labels["longitude"].setText(f"{data.get('lon', 0):.6f}")
        self.telemetry_labels["altitude"].setText(f"{data.get('alt', 0):.2f} m")
        self.telemetry_labels["groundspeed"].setText(f"{data.get('groundspeed', 0):.2f} m/s")
        self.telemetry_labels["heading"].setText(f"{data.get('heading', 0)}°")
        
        # Batarya verileri
        voltage = data.get('voltage', 0)
        current = data.get('current', 0)
        level = data.get('level', 0)
        self.telemetry_labels["voltage"].setText(f"{voltage:.2f} V")
        self.telemetry_labels["current"].setText(f"{current:.2f} A")
        if self.battery_progress.value() != level:
            self.battery_progress.setValue(level)
            
        # Yapay Ufuk (Attitude Indicator) güncellemesi
        roll_rad = data.get('roll', 0)
        pitch_rad = data.get('pitch', 0)
        self.attitude_indicator.set_attitude(pitch_rad, roll_rad)
            
        # Açısal veriler (Radyandan dereceye çevir)
        roll_deg = (roll_rad * 180 / 3.14159)
        pitch_deg = (pitch_rad * 180 / 3.14159)
        self.telemetry_labels["roll"].setText(f"{roll_deg:.2f}°")
        self.telemetry_labels["pitch"].setText(f"{pitch_deg:.2f}°")
