import sys
import os
import time
import threading
import math
import io
import serial.tools.list_ports
from collections import deque
import csv
from datetime import datetime
from PyQt5.QtCore import QUrl, Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QDateTime
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
                             QTextEdit, QHBoxLayout, QMessageBox, QGridLayout,
                             QProgressBar, QGroupBox, QComboBox, QDoubleSpinBox, QDialog, QFormLayout, QFrame, QScrollArea, QLineEdit)
from PyQt5.QtGui import QPixmap, QIcon, QTransform, QTextCursor, QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtWebChannel import QWebChannel

# Matplotlib ve PyQt5 backend için
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation

# Pixhawk bağlantısı için import edilecek (bu satırları aktif edin)
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import serial.tools.list_ports

# Attitude Indicator import
from attitude_indicator import AttitudeIndicator

# Otonom kayıt sistemi için OpenCV
try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("OpenCV bulunamadı. Kamera kayıt özelliği devre dışı.")

class AutonomousRecordingSystem:
    """Otonom modda kayıt yapma sistemi"""
    
    def __init__(self, gcs_app):
        self.gcs_app = gcs_app
        self.is_recording = False
        self.recording_start_time = None
        self.recording_dir = None
        
        # Video kayıt için
        self.camera_capture = None
        self.camera_writer = None
        self.map_writer = None
        
        # Telemetri kayıt için
        self.telemetry_csv_file = None
        self.telemetry_csv_writer = None
        
        # Kayıt timer'ları
        self.recording_timer = QTimer()
        self.recording_timer.timeout.connect(self._record_frame)
        self.recording_timer.setInterval(1000)  # 1 Hz
        
        # Logs dizini oluştur
        self.logs_dir = os.path.join(os.path.dirname(__file__), 'logs')
        os.makedirs(self.logs_dir, exist_ok=True)
    
    def start_recording(self):
        """Otonom kayıt sistemini başlatır"""
        if self.is_recording:
            return
        
        if not OPENCV_AVAILABLE:
            self.gcs_app.log_message_received.emit("❌ OpenCV bulunamadı! Kamera kayıt özelliği devre dışı.")
            return
        
        try:
            # Kayıt dizini oluştur
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.recording_dir = os.path.join(self.logs_dir, f'autonomous_recording_{timestamp}')
            os.makedirs(self.recording_dir, exist_ok=True)
            
            # Kamera başlat
            self.camera_capture = cv2.VideoCapture(0)
            if not self.camera_capture.isOpened():
                self.gcs_app.log_message_received.emit("❌ Kamera açılamadı!")
                return
            
            # Test frame al
            ret, test_frame = self.camera_capture.read()
            if not ret:
                self.gcs_app.log_message_received.emit("❌ Kamera test frame'i alınamadı!")
                return
            
            height, width = test_frame.shape[:2]
            
            # Video writer'ları oluştur
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            
            # 1. İşlenmiş kamera verisi (tespit ve takip sonuçları ile)
            camera_path = os.path.join(self.recording_dir, 'processed_camera_data.mp4')
            self.camera_writer = cv2.VideoWriter(camera_path, fourcc, 1.0, (width, height))
            
            # 2. Lokal harita/cost map/engel haritası
            map_path = os.path.join(self.recording_dir, 'local_obstacle_map.mp4')
            self.map_writer = cv2.VideoWriter(map_path, fourcc, 1.0, (width, height))
            
            # 3. Telemetri CSV dosyası
            telemetry_path = os.path.join(self.recording_dir, 'vehicle_telemetry.csv')
            self.telemetry_csv_file = open(telemetry_path, 'w', newline='', encoding='utf-8')
            self.telemetry_csv_writer = csv.writer(self.telemetry_csv_file)
            
            # CSV header
            self.telemetry_csv_writer.writerow([
                'timestamp', 'lat', 'lon', 'groundspeed_mps', 'roll_deg', 'pitch_deg', 'heading_deg',
                'speed_setpoint_mps', 'heading_setpoint_deg', 'vehicle_mode', 'armed_status'
            ])
            
            # Kayıt durumunu güncelle
            self.is_recording = True
            self.recording_start_time = time.time()
            
            # Timer'ı başlat
            self.recording_timer.start()
            
            # UI'yi güncelle
            self.gcs_app._update_recording_ui(True)
            
            self.gcs_app.log_message_received.emit(f"🎥 OTONOM KAYIT BAŞLADI: {self.recording_dir}")
            self.gcs_app.log_message_received.emit("📁 Kayıt dosyaları:")
            self.gcs_app.log_message_received.emit(f"   📹 Kamera: {camera_path}")
            self.gcs_app.log_message_received.emit(f"   🗺️  Harita: {map_path}")
            self.gcs_app.log_message_received.emit(f"   📊 Telemetri: {telemetry_path}")
            
        except Exception as e:
            self.gcs_app.log_message_received.emit(f"❌ Kayıt başlatma hatası: {e}")
            self.stop_recording()
    
    def stop_recording(self):
        """Otonom kayıt sistemini durdurur"""
        if not self.is_recording:
            return
        
        try:
            # Timer'ı durdur
            self.recording_timer.stop()
            
            # Video writer'ları kapat
            if self.camera_writer:
                self.camera_writer.release()
                self.camera_writer = None
            
            if self.map_writer:
                self.map_writer.release()
                self.map_writer = None
            
            # Kamera'yı kapat
            if self.camera_capture:
                self.camera_capture.release()
                self.camera_capture = None
            
            # CSV dosyasını kapat
            if self.telemetry_csv_file:
                self.telemetry_csv_file.close()
                self.telemetry_csv_file = None
                self.telemetry_csv_writer = None
            
            # Kayıt süresini hesapla
            recording_duration = time.time() - self.recording_start_time if self.recording_start_time else 0
            
            self.gcs_app.log_message_received.emit(f"⏹️ OTONOM KAYIT DURDURULDU")
            self.gcs_app.log_message_received.emit(f"⏱️ Kayıt süresi: {recording_duration:.1f} saniye")
            self.gcs_app.log_message_received.emit(f"📁 Kayıt dosyaları: {self.recording_dir}")
            
            # UI'yi güncelle
            self.gcs_app._update_recording_ui(False)
            
            # Kayıt durumunu sıfırla
            self.is_recording = False
            self.recording_start_time = None
            self.recording_dir = None
            
        except Exception as e:
            self.gcs_app.log_message_received.emit(f"❌ Kayıt durdurma hatası: {e}")
    
    def _record_frame(self):
        """1 Hz'de frame kaydet"""
        if not self.is_recording or not self.camera_capture:
            return
        
        try:
            # Kamera frame'i al
            ret, frame = self.camera_capture.read()
            if not ret:
                return
            
            # Zaman etiketi
            timestamp = datetime.utcnow().isoformat() + 'Z'
            timestamp_display = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # 1. İşlenmiş kamera verisi (tespit ve takip sonuçları)
            processed_frame = self._process_camera_frame(frame.copy(), timestamp_display)
            if self.camera_writer:
                self.camera_writer.write(processed_frame)
            
            # 2. Lokal harita/cost map/engel haritası
            map_frame = self._create_obstacle_map(frame.copy(), timestamp_display)
            if self.map_writer:
                self.map_writer.write(map_frame)
            
            # 3. Telemetri verisi
            self._record_telemetry_data(timestamp)
            
        except Exception as e:
            self.gcs_app.log_message_received.emit(f"❌ Frame kayıt hatası: {e}")
    
    def _process_camera_frame(self, frame, timestamp):
        """Kamera frame'ini işler (tespit ve takip sonuçları ekler)"""
        try:
            h, w = frame.shape[:2]
            
            # ROI (Region of Interest) çiz
            roi_w, roi_h = int(w * 0.6), int(h * 0.6)
            x0, y0 = (w - roi_w) // 2, (h - roi_h) // 2
            cv2.rectangle(frame, (x0, y0), (x0 + roi_w, y0 + roi_h), (0, 255, 0), 2)
            
            # Engel algılama (sarı renk tespiti)
            roi = frame[y0:y0+roi_h, x0:x0+roi_w]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))
            
            # Engel tespiti
            yellow_ratio = np.count_nonzero(mask_yellow) / (roi.shape[0] * roi.shape[1])
            obstacle_detected = yellow_ratio > 0.15
            
            # Engel konumu tespiti
            obstacle_position = self._detect_obstacle_position(roi, mask_yellow)
            
            # Görsel overlay
            status_color = (0, 255, 0) if not obstacle_detected else (0, 0, 255)
            cv2.putText(frame, f"OTONOM MOD - {timestamp}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Engel bilgisi
            obstacle_info = f"Engel: {yellow_ratio*100:.1f}%"
            if obstacle_detected:
                obstacle_info += f" ({obstacle_position})"
            cv2.putText(frame, obstacle_info, (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Araç bilgileri
            if self.gcs_app.vehicle:
                mode = getattr(self.gcs_app.vehicle.mode, 'name', 'UNKNOWN')
                armed = getattr(self.gcs_app.vehicle, 'armed', False)
                armed_text = "ARMED" if armed else "DISARMED"
                cv2.putText(frame, f"Mod: {mode} | {armed_text}", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Zaman etiketi
            cv2.putText(frame, f"TS: {timestamp}", (10, h - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            return frame
            
        except Exception as e:
            self.gcs_app.log_message_received.emit(f"❌ Kamera işleme hatası: {e}")
            return frame
    
    def _create_obstacle_map(self, frame, timestamp):
        """Lokal harita/cost map/engel haritası oluşturur"""
        try:
            h, w = frame.shape[:2]
            
            # ROI
            roi_w, roi_h = int(w * 0.6), int(h * 0.6)
            x0, y0 = (w - roi_w) // 2, (h - roi_h) // 2
            roi = frame[y0:y0+roi_h, x0:x0+roi_w]
            
            # Engel algılama
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask_yellow = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))
            
            # Harita overlay'i oluştur
            map_overlay = np.zeros_like(frame)
            map_overlay[y0:y0+roi_h, x0:x0+roi_w] = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2BGR)
            
            # Harita görselleştirmesi
            map_vis = cv2.addWeighted(frame, 0.3, map_overlay, 0.7, 0)
            
            # ROI çerçevesi
            cv2.rectangle(map_vis, (x0, y0), (x0 + roi_w, y0 + roi_h), (0, 255, 255), 2)
            
            # Bölge ayırıcıları
            part_w = roi_w // 3
            cv2.line(map_vis, (x0 + part_w, y0), (x0 + part_w, y0 + roi_h), (255, 255, 0), 1)
            cv2.line(map_vis, (x0 + 2*part_w, y0), (x0 + 2*part_w, y0 + roi_h), (255, 255, 0), 1)
            
            # Başlık
            cv2.putText(map_vis, f"LOKAL ENGEL HARİTASI - {timestamp}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Engel yoğunluğu
            yellow_ratio = np.count_nonzero(mask_yellow) / (roi.shape[0] * roi.shape[1])
            cv2.putText(map_vis, f"Engel Yoğunluğu: {yellow_ratio*100:.1f}%", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Zaman etiketi
            cv2.putText(map_vis, f"TS: {timestamp}", (10, h - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            return map_vis
            
        except Exception as e:
            self.gcs_app.log_message_received.emit(f"❌ Harita oluşturma hatası: {e}")
            return frame
    
    def _detect_obstacle_position(self, roi, mask_yellow):
        """Engel konumunu tespit eder"""
        try:
            h, w = roi.shape[:2]
            left_part = mask_yellow[:, :w//3]
            center_part = mask_yellow[:, w//3:2*w//3]
            right_part = mask_yellow[:, 2*w//3:]
            
            left_density = np.count_nonzero(left_part) / (left_part.shape[0] * left_part.shape[1])
            center_density = np.count_nonzero(center_part) / (center_part.shape[0] * center_part.shape[1])
            right_density = np.count_nonzero(right_part) / (right_part.shape[0] * right_part.shape[1])
            
            max_density = max(left_density, center_density, right_density)
            
            if max_density < 0.02:
                return "none"
            elif left_density == max_density:
                return "left"
            elif right_density == max_density:
                return "right"
            else:
                return "center"
                
        except Exception:
            return "unknown"
    
    def manual_start_recording(self):
        """Manuel olarak kayıt başlatır"""
        if not self.autonomous_recording.is_recording:
            self.autonomous_recording.start_recording()
            self._update_recording_ui(True)
        else:
            self.log_message_received.emit("⚠️ Kayıt zaten aktif!")
    
    def manual_stop_recording(self):
        """Manuel olarak kayıt durdurur"""
        if self.autonomous_recording.is_recording:
            self.autonomous_recording.stop_recording()
            self._update_recording_ui(False)
        else:
            self.log_message_received.emit("⚠️ Kayıt zaten kapalı!")
    
    def _update_recording_ui(self, is_recording):
        """Kayıt UI'sını günceller"""
        if is_recording:
            self.recording_status_label.setText("🟢 Kayıt Aktif")
            self.recording_status_label.setStyleSheet("font-weight: bold; color: green; padding: 5px; border: 1px solid green;")
            self.start_recording_btn.setEnabled(False)
            self.stop_recording_btn.setEnabled(True)
            
            # Kayıt süresi timer'ı başlat
            self.recording_duration_timer = QTimer()
            self.recording_duration_timer.timeout.connect(self._update_recording_duration)
            self.recording_duration_timer.start(1000)  # 1 saniye
            self.recording_start_time = time.time()
        else:
            self.recording_status_label.setText("🔴 Kayıt Kapalı")
            self.recording_status_label.setStyleSheet("font-weight: bold; color: red; padding: 5px; border: 1px solid red;")
            self.start_recording_btn.setEnabled(True)
            self.stop_recording_btn.setEnabled(False)
            
            # Kayıt süresi timer'ını durdur
            if hasattr(self, 'recording_duration_timer'):
                self.recording_duration_timer.stop()
            self.recording_duration_label.setText("Süre: 00:00")
    
    def _update_recording_duration(self):
        """Kayıt süresini günceller"""
        if hasattr(self, 'recording_start_time') and self.recording_start_time:
            duration = int(time.time() - self.recording_start_time)
            minutes = duration // 60
            seconds = duration % 60
            self.recording_duration_label.setText(f"Süre: {minutes:02d}:{seconds:02d}")

    def manual_start_recording(self):
        """Manuel olarak kayıt başlatır"""
        if not self.autonomous_recording.is_recording:
            self.autonomous_recording.start_recording()
            self._update_recording_ui(True)
        else:
            self.log_message_received.emit("⚠️ Kayıt zaten aktif!")
    
    def manual_stop_recording(self):
        """Manuel olarak kayıt durdurur"""
        if self.autonomous_recording.is_recording:
            self.autonomous_recording.stop_recording()
            self._update_recording_ui(False)
        else:
            self.log_message_received.emit("⚠️ Kayıt zaten kapalı!")
    
    def _update_recording_ui(self, is_recording):
        """Kayıt UI'sını günceller"""
        if is_recording:
            self.recording_status_label.setText("🟢 Kayıt Aktif")
            self.recording_status_label.setStyleSheet("font-weight: bold; color: green; padding: 5px; border: 1px solid green;")
            self.start_recording_btn.setEnabled(False)
            self.stop_recording_btn.setEnabled(True)
            
            # Kayıt süresi timer'ı başlat
            self.recording_duration_timer = QTimer()
            self.recording_duration_timer.timeout.connect(self._update_recording_duration)
            self.recording_duration_timer.start(1000)  # 1 saniye
            self.recording_start_time = time.time()
        else:
            self.recording_status_label.setText("🔴 Kayıt Kapalı")
            self.recording_status_label.setStyleSheet("font-weight: bold; color: red; padding: 5px; border: 1px solid red;")
            self.start_recording_btn.setEnabled(True)
            self.stop_recording_btn.setEnabled(False)
            
            # Kayıt süresi timer'ını durdur
            if hasattr(self, 'recording_duration_timer'):
                self.recording_duration_timer.stop()
            self.recording_duration_label.setText("Süre: 00:00")

    def _record_telemetry_data(self, timestamp):
        """Telemetri verisini CSV'ye kaydeder"""
        if not self.telemetry_csv_writer or not self.gcs_app.vehicle:
            return
        
        try:
            # Araç verilerini al
            vehicle = self.gcs_app.vehicle
            
            # Konum
            lat = None
            lon = None
            if hasattr(vehicle, 'location') and vehicle.location and hasattr(vehicle.location, 'global_frame'):
                gf = vehicle.location.global_frame
                lat = getattr(gf, 'lat', None)
                lon = getattr(gf, 'lon', None)
            
            # Hız ve yönelim
            groundspeed = getattr(vehicle, 'groundspeed', None)
            heading = getattr(vehicle, 'heading', None)
            
            # Roll, pitch
            roll_deg = None
            pitch_deg = None
            if hasattr(vehicle, 'attitude') and vehicle.attitude is not None:
                try:
                    roll_deg = math.degrees(vehicle.attitude.roll)
                    pitch_deg = math.degrees(vehicle.attitude.pitch)
                except Exception:
                    pass
            
            # Setpoint değerleri
            speed_setpoint = None
            heading_setpoint = None
            mode_name = getattr(getattr(vehicle, 'mode', None), 'name', None)
            
            if mode_name in ["AUTO", "GUIDED", "RTL"]:
                # Hız setpoint
                wpnav_speed = vehicle.parameters.get('WPNAV_SPEED', None)
                wp_speed = vehicle.parameters.get('WP_SPEED', None)
                speed_param = wpnav_speed if wpnav_speed is not None else wp_speed
                if speed_param is not None:
                    speed_setpoint = speed_param / 100.0
                
                # Heading setpoint
                if len(self.gcs_app.waypoints) > 0 and isinstance(heading, (int, float)):
                    heading_setpoint = self.gcs_app.get_target_heading_from_mission(heading)
            
            # Armed durumu
            armed = getattr(vehicle, 'armed', False)
            
            # CSV'ye yaz
            self.telemetry_csv_writer.writerow([
                timestamp,
                f"{lat:.7f}" if isinstance(lat, float) else "",
                f"{lon:.7f}" if isinstance(lon, float) else "",
                f"{groundspeed:.3f}" if isinstance(groundspeed, float) else "",
                f"{roll_deg:.2f}" if isinstance(roll_deg, float) else "",
                f"{pitch_deg:.2f}" if isinstance(pitch_deg, float) else "",
                f"{heading:.1f}" if isinstance(heading, (int, float)) else "",
                f"{speed_setpoint:.3f}" if isinstance(speed_setpoint, float) else "",
                f"{heading_setpoint:.1f}" if isinstance(heading_setpoint, (int, float)) else "",
                mode_name or "",
                "ARMED" if armed else "DISARMED"
            ])
            
            # Dosyayı flush et
            if self.telemetry_csv_file:
                self.telemetry_csv_file.flush()
                
        except Exception as e:
            self.gcs_app.log_message_received.emit(f"❌ Telemetri kayıt hatası: {e}")

class MapBridge(QObject):
    """Harita ve Python arasında köprü görevi gören sınıf"""
    updateVehiclePosition = pyqtSignal(float, float, float)
    addWaypoint = pyqtSignal(float, float)
    waypoint_from_user = pyqtSignal(float, float)
    waypoint_removed = pyqtSignal(int)  # Waypoint silme sinyali
    clearMap = pyqtSignal()
    mapReady = pyqtSignal()

    @pyqtSlot(float, float)
    def add_waypoint_to_ui(self, lat, lng):
        """JavaScript tarafından haritaya çift tıklandığında çağrılır."""
        self.waypoint_from_user.emit(lat, lng)
    
    @pyqtSlot(int)
    def remove_waypoint_from_ui(self, index):
        """JavaScript tarafından waypoint silindiğinde çağrılır."""
        self.waypoint_removed.emit(index)

    @pyqtSlot()
    def notify_ready(self):
        """JavaScript tarafı hazır olduğunda çağrılır."""
        self.mapReady.emit()

class GCSApp(QWidget):
    log_message_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool, str)
    status_message_received = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.vehicle = None
        self.is_connected = False
        self.waypoints = []
        self.current_heading = 0
        self.gorev2_process = None
        self.gorev2_running = False
        self._gorev2_reader = None
        self._gorev2_thread = None
        
        # Otonom kayıt sistemi
        self.autonomous_recording = AutonomousRecordingSystem(self)
        self.last_vehicle_mode = None
        
        # Önceden tanımlanmış görev koordinatları (İstanbul çevresi)
        self.mission_coordinates = [
            [41.0082, 28.9784],   # İstanbul merkez (Fatih)
            [41.0186, 28.9647],   # Eminönü
            [41.0214, 28.9731],   # Karaköy
            [41.0136, 28.9550]    # Beyoğlu
        ]
        
        # Görev 2 hedef koordinatları (erkan_denendi.py ile uyumlu sıralı liste)
        self.gorev2_targets = [
            (40.771275, 29.437543),
            (40.771600, 29.437900),
            (40.771900, 29.437300),
            (40.771400, 29.436900),
        ]
        # İç navigasyon için ilk hedef (geri uyum)
        self.gorev2_target = self.gorev2_targets[0]
        
        # Grafik verileri için deque'lar (son 100 veri noktası)
        self.graph_data_size = 100
        self.speed_data = deque(maxlen=self.graph_data_size)
        self.speed_setpoint_data = deque(maxlen=self.graph_data_size)
        self.heading_data = deque(maxlen=self.graph_data_size)
        self.heading_setpoint_data = deque(maxlen=self.graph_data_size)
        self.thruster_left_data = deque(maxlen=self.graph_data_size)
        self.thruster_right_data = deque(maxlen=self.graph_data_size)
        self.time_data = deque(maxlen=self.graph_data_size)
        
        # SERVO_OUTPUT_RAW cache - DroneKit channels güncellenmiyor
        self.servo_output_cache = {}
        
        # Harita yüklenme durumu ve bekleyen işlemler
        self.map_ready = False
        self._pending_waypoints = []  # [(lat, lon)]
        self._pending_clear = False
        
        # Telemetri CSV kaydı için durum
        self._telemetry_csv_file = None
        self._telemetry_csv_writer = None
        self._telemetry_log_timer = QTimer(self)
        self._telemetry_log_timer.timeout.connect(self._log_telemetry_row)
        self.initUI()
        self.log_message_received.connect(self.update_log_safe)
        self.connection_status_changed.connect(self.on_connection_status_changed)
        self.status_message_received.connect(self.update_status_safe)

    def initUI(self):
        self.setWindowTitle('IDA Yer Kontrol İstasyonu')
        self.setGeometry(100, 100, 1400, 900)

        # Ana layout
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # Sol taraf (Sidebar) - Kaydırmalı (sadece dikey)
        sidebar_scroll = QScrollArea()
        sidebar_widget = QWidget()
        sidebar_layout = QVBoxLayout(sidebar_widget)
        
        sidebar_scroll.setWidget(sidebar_widget)
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setMaximumWidth(400)
        sidebar_scroll.setMinimumWidth(400)  # Sabit genişlik
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Widget'in genişliğini sabitle
        sidebar_widget.setMaximumWidth(380)  # Scroll bar için biraz yer bırak
        sidebar_widget.setMinimumWidth(380)

        # Sağ taraf (Harita)
        map_layout = QVBoxLayout()
        
        # Web View (Harita)
        self.web_view = QWebEngineView()
        self.setup_web_channel()
        map_layout.addWidget(self.web_view)

        # 1. TELEMETRİ PANELİ - EN ÜSTTE
        telemetry_frame = QFrame()
        telemetry_frame.setFrameShape(QFrame.StyledPanel)
        telemetry_layout = QGridLayout(telemetry_frame)
        sidebar_layout.addWidget(telemetry_frame)
        
        telemetry_title = QLabel("Telemetri")
        telemetry_title.setFont(QFont('Arial', 14, QFont.Bold))
        telemetry_layout.addWidget(telemetry_title, 0, 0, 1, 4)

        self.telemetry_values = {
            "Hız:": QLabel("N/A"), "Hız Setpoint:": QLabel("N/A"),
            "Yükseklik:": QLabel("N/A"), "Heading:": QLabel("N/A"), 
            "Heading Setpoint:": QLabel("N/A"), "Pitch:": QLabel("N/A"),
            "Yaw:": QLabel("N/A"), "GPS Fix:": QLabel("N/A"),
            "Mod:": QLabel("N/A"), "Batarya:": QLabel("N/A"),
            "ARM Durumu:": QLabel("N/A")
        }
        
        row = 1
        for label, value_widget in self.telemetry_values.items():
            telemetry_layout.addWidget(QLabel(label), row, 0)
            telemetry_layout.addWidget(value_widget, row, 1)
            row += 1

        self.battery_progress = QProgressBar()
        telemetry_layout.addWidget(QLabel("Batarya Seviyesi:"), row, 0)
        telemetry_layout.addWidget(self.battery_progress, row, 1)
        
        # Thruster durumu için basit gösterim (Deniz aracı - 2 motor)
        row += 1
        thruster_layout = QVBoxLayout()  # Dikey layout - daha iyi görünüm
        self.thruster_labels = []
        for i in range(2):  # Deniz aracı için 2 thruster
            thruster_label = QLabel(f"T{i+1}: 0%")
            thruster_label.setStyleSheet("border: 1px solid gray; padding: 5px; font-size: 10px; font-weight: bold;")
            thruster_label.setMinimumHeight(25)  # Yükseklik arttır
            thruster_label.setWordWrap(False)  # Text wrapping kapalı
            thruster_layout.addWidget(thruster_label)
            self.thruster_labels.append(thruster_label)
        
        telemetry_layout.addWidget(QLabel("Thruster'lar:"), row, 0)
        telemetry_layout.addLayout(thruster_layout, row, 1, 1, 2)

        # Attitude Indicator (Gyro) ekle
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedSize(120, 120)
        telemetry_layout.addWidget(self.attitude_indicator, 1, 2, 4, 2)
        
        # 2. GÖREVLER PANELİ - TELEMETRİ ALTINDA
        missions_frame = QFrame()
        missions_frame.setFrameShape(QFrame.StyledPanel)
        missions_layout = QVBoxLayout(missions_frame)
        sidebar_layout.addWidget(missions_frame)
        
        missions_title = QLabel("Görevler")
        missions_title.setFont(QFont('Arial', 14, QFont.Bold))
        missions_layout.addWidget(missions_title)
        
        # Parkur 1 - 4 koordinat
        parkur1_group = QGroupBox("Parkur 1 (4 Koordinat)")
        parkur1_layout = QGridLayout(parkur1_group)
        
        self.parkur1_coords = []
        for i in range(4):
            lat_label = QLabel(f"P{i+1} Lat:")
            lon_label = QLabel(f"P{i+1} Lon:")
            lat_input = QDoubleSpinBox()
            lon_input = QDoubleSpinBox()
            
            lat_input.setDecimals(6)
            lat_input.setRange(-90.0, 90.0)
            lat_input.setValue(41.0082 + i*0.001)  # Varsayılan değerler
            
            lon_input.setDecimals(6)
            lon_input.setRange(-180.0, 180.0)
            lon_input.setValue(28.9784 + i*0.001)  # Varsayılan değerler
            
            parkur1_layout.addWidget(lat_label, i, 0)
            parkur1_layout.addWidget(lat_input, i, 1)
            parkur1_layout.addWidget(lon_label, i, 2)
            parkur1_layout.addWidget(lon_input, i, 3)
            
            self.parkur1_coords.append({"lat": lat_input, "lon": lon_input})
        
        missions_layout.addWidget(parkur1_group)
        
        # Parkur 2 - 1 koordinat
        parkur2_group = QGroupBox("Parkur 2 (1 Koordinat)")
        parkur2_layout = QGridLayout(parkur2_group)
        
        self.parkur2_lat = QDoubleSpinBox()
        self.parkur2_lon = QDoubleSpinBox()
        
        self.parkur2_lat.setDecimals(6)
        self.parkur2_lat.setRange(-90.0, 90.0)
        self.parkur2_lat.setValue(41.0100)  # Varsayılan değer
        
        self.parkur2_lon.setDecimals(6)
        self.parkur2_lon.setRange(-180.0, 180.0)
        self.parkur2_lon.setValue(28.9800)  # Varsayılan değer
        
        parkur2_layout.addWidget(QLabel("Lat:"), 0, 0)
        parkur2_layout.addWidget(self.parkur2_lat, 0, 1)
        parkur2_layout.addWidget(QLabel("Lon:"), 0, 2)
        parkur2_layout.addWidget(self.parkur2_lon, 0, 3)
        
        missions_layout.addWidget(parkur2_group)
        
        # Parkur 3 - Renk seçimi
        parkur3_group = QGroupBox("Parkur 3 (Renk Seçimi)")
        parkur3_layout = QHBoxLayout(parkur3_group)
        
        self.color_buttons = []
        self.selected_color = None
        
        colors = [("Kırmızı", "red"), ("Yeşil", "green"), ("Siyah", "black")]
        for color_name, color_value in colors:
            btn = QPushButton(color_name)
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, c=color_value, n=color_name: self.select_color(c, n))
            if color_value == "red":
                btn.setStyleSheet("background-color: #ffcccc;")
            elif color_value == "green":
                btn.setStyleSheet("background-color: #ccffcc;")
            else:  # black
                btn.setStyleSheet("background-color: #cccccc;")
            parkur3_layout.addWidget(btn)
            self.color_buttons.append(btn)
        
        missions_layout.addWidget(parkur3_group)
        
        # Gönder butonu
        self.send_mission_btn = QPushButton("GÖREV GÖNDER")
        self.send_mission_btn.setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white; padding: 8px;")
        self.send_mission_btn.clicked.connect(self.send_custom_mission)
        self.send_mission_btn.setEnabled(False)  # Başlangıçta devre dışı
        missions_layout.addWidget(self.send_mission_btn)
        
        # Başlat butonu - sadece görev gönderildikten sonra aktif olur
        self.start_mission_btn = QPushButton("🚀 GÖREVI BAŞLAT")
        self.start_mission_btn.setStyleSheet("font-weight: bold; background-color: #FF9800; color: white; padding: 8px; font-size: 12px;")
        self.start_mission_btn.clicked.connect(self.start_custom_mission)
        self.start_mission_btn.setEnabled(False)  # Başlangıçta devre dışı
        self.start_mission_btn.setVisible(False)  # Başlangıçta gizli
        missions_layout.addWidget(self.start_mission_btn)
        
        # Durdur butonu - sadece görev çalışırken görünür
        self.stop_mission_btn = QPushButton("⏹️ GÖREVI DURDUR")
        self.stop_mission_btn.setStyleSheet("font-weight: bold; background-color: #F44336; color: white; padding: 8px; font-size: 12px;")
        self.stop_mission_btn.clicked.connect(self.stop_custom_mission)
        self.stop_mission_btn.setEnabled(False)  # Başlangıçta devre dışı
        self.stop_mission_btn.setVisible(False)  # Başlangıçta gizli
        missions_layout.addWidget(self.stop_mission_btn)
        
        # 3. GRAFİKLER PANELİ - GÖREVLER ALTINDA
        graphs_frame = QFrame()
        graphs_frame.setFrameShape(QFrame.StyledPanel)
        graphs_layout = QVBoxLayout(graphs_frame)
        sidebar_layout.addWidget(graphs_frame)
        
        graphs_title = QLabel("Grafikler")
        graphs_title.setFont(QFont('Arial', 14, QFont.Bold))
        graphs_layout.addWidget(graphs_title)
        
        # 1. Grafik - Hız karşılaştırması (İki ayrı çizgi)
        self.speed_figure = Figure(figsize=(3.5, 1.2), facecolor='white')
        self.speed_canvas = FigureCanvas(self.speed_figure)
        self.speed_canvas.setMaximumHeight(100)  # Çok küçük yükseklik
        self.speed_canvas.setMinimumHeight(100)
        self.speed_ax = self.speed_figure.add_subplot(111)
        self.speed_ax.set_title('Hız: Mavi=Gerçek, Kırmızı=Setpoint', fontsize=8, pad=2)
        self.speed_ax.set_ylabel('m/s', fontsize=6)
        self.speed_ax.grid(True, alpha=0.2)
        self.speed_ax.tick_params(labelsize=5, pad=1)
        # X ekseni etiketlerini gizle (alan kazanmak için)
        self.speed_ax.set_xticklabels([])
        self.speed_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.1)
        graphs_layout.addWidget(self.speed_canvas)
        
        # 2. Grafik - Heading karşılaştırması (İki ayrı çizgi)
        self.heading_figure = Figure(figsize=(3.5, 1.2), facecolor='white')
        self.heading_canvas = FigureCanvas(self.heading_figure)
        self.heading_canvas.setMaximumHeight(100)  # Çok küçük yükseklik
        self.heading_canvas.setMinimumHeight(100)
        self.heading_ax = self.heading_figure.add_subplot(111)
        self.heading_ax.set_title('Heading: Yeşil=Gerçek, Kırmızı=Setpoint', fontsize=8, pad=2)
        self.heading_ax.set_ylabel('°', fontsize=6)
        self.heading_ax.grid(True, alpha=0.2)
        self.heading_ax.tick_params(labelsize=5, pad=1)
        # X ekseni etiketlerini gizle (alan kazanmak için)
        self.heading_ax.set_xticklabels([])
        self.heading_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.1)
        graphs_layout.addWidget(self.heading_canvas)
        
        # 3. Grafik - Thruster karşılaştırması (İki ayrı çizgi)
        self.thruster_figure = Figure(figsize=(3.5, 1.2), facecolor='white')
        self.thruster_canvas = FigureCanvas(self.thruster_figure)
        self.thruster_canvas.setMaximumHeight(100)  # Çok küçük yükseklik
        self.thruster_canvas.setMinimumHeight(100)
        self.thruster_ax = self.thruster_figure.add_subplot(111)
        self.thruster_ax.set_title('Thruster: Mavi=Sol, Kırmızı=Sağ', fontsize=8, pad=2)
        self.thruster_ax.set_ylabel('%', fontsize=6)
        self.thruster_ax.grid(True, alpha=0.2)
        self.thruster_ax.tick_params(labelsize=5, pad=1)
        # Sadece son grafikte X ekseni etiketleri göster
        self.thruster_ax.set_xlabel('Zaman (s)', fontsize=6)
        self.thruster_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.2)
        graphs_layout.addWidget(self.thruster_canvas)
        
        # 4. OTONOM KAYIT PANELİ - GRAFİKLER ALTINDA
        recording_frame = QFrame()
        recording_frame.setFrameShape(QFrame.StyledPanel)
        recording_layout = QVBoxLayout(recording_frame)
        sidebar_layout.addWidget(recording_frame)
        
        recording_title = QLabel("Otonom Kayıt Sistemi")
        recording_title.setFont(QFont('Arial', 14, QFont.Bold))
        recording_layout.addWidget(recording_title)
        
        # Kayıt durumu göstergesi
        self.recording_status_label = QLabel("🔴 Kayıt Kapalı")
        self.recording_status_label.setStyleSheet("font-weight: bold; color: red; padding: 5px; border: 1px solid red;")
        recording_layout.addWidget(self.recording_status_label)
        
        # Kayıt süresi göstergesi
        self.recording_duration_label = QLabel("Süre: 00:00")
        self.recording_duration_label.setStyleSheet("font-size: 12px; color: gray;")
        recording_layout.addWidget(self.recording_duration_label)
        
        # Manuel kayıt kontrol butonları
        recording_buttons_layout = QHBoxLayout()
        self.start_recording_btn = QPushButton("🎥 Kayıt Başlat")
        self.stop_recording_btn = QPushButton("⏹️ Kayıt Durdur")
        self.start_recording_btn.clicked.connect(self.manual_start_recording)
        self.stop_recording_btn.clicked.connect(self.manual_stop_recording)
        self.stop_recording_btn.setEnabled(False)
        recording_buttons_layout.addWidget(self.start_recording_btn)
        recording_buttons_layout.addWidget(self.stop_recording_btn)
        recording_layout.addLayout(recording_buttons_layout)
        
        # 5. SİSTEM LOGLARI PANELİ - OTONOM KAYIT ALTINDA
        log_frame = QFrame()
        log_frame.setFrameShape(QFrame.StyledPanel)
        log_layout = QVBoxLayout(log_frame)
        sidebar_layout.addWidget(log_frame)
        log_title = QLabel("Sistem Logları")
        log_title.setFont(QFont('Arial', 14, QFont.Bold))
        log_layout.addWidget(log_title)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        log_layout.addWidget(self.log_display)

        # 5. BAĞLANTI PANELİ - LOG ALTINDA
        connection_frame = QFrame()
        connection_frame.setFrameShape(QFrame.StyledPanel)
        connection_layout = QVBoxLayout(connection_frame)
        sidebar_layout.addWidget(connection_frame)

        connection_title = QLabel("Bağlantı")
        connection_title.setFont(QFont('Arial', 14, QFont.Bold))
        connection_layout.addWidget(connection_title)
        
        # Bağlantı tipi seçimi
        self.connection_type = QComboBox()
        self.connection_type.addItems(["Serial (USB)", "UDP (Kablosuz)", "TCP (WiFi)"])
        self.connection_type.currentTextChanged.connect(self.on_connection_type_changed)
        
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "38400", "19200", "9600"])
        self.baud_combo.setCurrentText("57600")  # Telemetri modülleri için varsayılan
        
        # UDP/TCP için IP ve Port alanları
        self.ip_input = QDoubleSpinBox()
        self.ip_input.setDecimals(0)
        self.ip_input.setRange(0, 999)
        self.ip_input.setValue(127)
        self.ip_input.setVisible(False)
        
        self.udp_port_input = QDoubleSpinBox()
        self.udp_port_input.setDecimals(0)
        self.udp_port_input.setRange(1000, 65535)
        self.udp_port_input.setValue(14550)
        self.udp_port_input.setVisible(False)
        
        self.refresh_button = QPushButton("Portları Yenile")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button = QPushButton("  BAĞLAN")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        connection_grid = QGridLayout()
        connection_grid.addWidget(QLabel("Tip:"), 0, 0)
        connection_grid.addWidget(self.connection_type, 0, 1)
        connection_grid.addWidget(QLabel("Port:"), 1, 0)
        connection_grid.addWidget(self.port_combo, 1, 1)
        connection_grid.addWidget(QLabel("Baud:"), 2, 0)
        connection_grid.addWidget(self.baud_combo, 2, 1)
        connection_grid.addWidget(QLabel("IP:"), 3, 0)
        connection_grid.addWidget(self.ip_input, 3, 1)
        connection_grid.addWidget(QLabel("UDP Port:"), 4, 0)
        connection_grid.addWidget(self.udp_port_input, 4, 1)
        connection_grid.addWidget(self.refresh_button, 5, 0)
        connection_grid.addWidget(self.connect_button, 5, 1)
        connection_layout.addLayout(connection_grid)

        # 6. DURUM PANELİ
        status_frame = QFrame()
        status_frame.setFrameShape(QFrame.StyledPanel)
        status_layout = QVBoxLayout(status_frame)
        sidebar_layout.addWidget(status_frame)
        status_title = QLabel("Durum")
        status_title.setFont(QFont('Arial', 14, QFont.Bold))
        status_layout.addWidget(status_title)
        self.status_label = QLabel("Sistem hazır. Bağlantı bekleniyor...")
        self.status_label.setWordWrap(True)
        status_layout.addWidget(self.status_label)

        # 7. GÖREV KONTROL PANELİ
        mission_control_frame = QFrame()
        mission_control_frame.setFrameShape(QFrame.StyledPanel)
        mission_control_layout = QVBoxLayout(mission_control_frame)
        sidebar_layout.addWidget(mission_control_frame)
        mission_title = QLabel("Görev Kontrolü")
        mission_title.setFont(QFont('Arial', 14, QFont.Bold))
        mission_control_layout.addWidget(mission_title)
        
        mode_buttons_layout = QHBoxLayout()
        self.stabilize_button = QPushButton("STABILIZE")
        self.auto_button = QPushButton("AUTO")
        self.guided_button = QPushButton("GUIDED")
        mode_buttons_layout.addWidget(self.stabilize_button)
        mode_buttons_layout.addWidget(self.auto_button)
        mode_buttons_layout.addWidget(self.guided_button)
        mission_control_layout.addLayout(mode_buttons_layout)
        
        self.stabilize_button.clicked.connect(lambda: self.set_vehicle_mode("STABILIZE"))
        self.auto_button.clicked.connect(lambda: self.set_vehicle_mode("AUTO"))
        self.guided_button.clicked.connect(lambda: self.set_vehicle_mode("GUIDED"))

        mission_buttons_layout = QHBoxLayout()
        self.upload_mission_button = QPushButton("Rotayı Gönder")
        self.read_mission_button = QPushButton("Rotayı Oku")
        self.clear_mission_button = QPushButton("Rotayı Temizle")
        self.load_mission_button = QPushButton("Görev")
        self.gorev2_button = QPushButton("Görev 2")
        mission_buttons_layout.addWidget(self.upload_mission_button)
        mission_buttons_layout.addWidget(self.read_mission_button)
        mission_buttons_layout.addWidget(self.clear_mission_button)
        mission_buttons_layout.addWidget(self.load_mission_button)
        mission_buttons_layout.addWidget(self.gorev2_button)
        mission_control_layout.addLayout(mission_buttons_layout)
        
        # ARM/DISARM butonları ekle
        arm_buttons_layout = QHBoxLayout()
        self.arm_button = QPushButton("ARM")
        self.disarm_button = QPushButton("DISARM")
        self.test_motors_button = QPushButton("TEST MOTORLAR")
        self.arm_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
        self.disarm_button.setStyleSheet("background-color: #F44336; color: white; font-weight: bold; padding: 5px;")
        self.test_motors_button.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 5px;")
        self.arm_button.clicked.connect(self.arm_vehicle)
        self.disarm_button.clicked.connect(self.disarm_vehicle)
        self.test_motors_button.clicked.connect(self.test_motors_manual)
        arm_buttons_layout.addWidget(self.arm_button)
        arm_buttons_layout.addWidget(self.disarm_button)
        arm_buttons_layout.addWidget(self.test_motors_button)
        mission_control_layout.addLayout(arm_buttons_layout)
        
        self.upload_mission_button.clicked.connect(self.send_mission_to_vehicle)
        self.read_mission_button.clicked.connect(self.read_custom_mission_from_vehicle)
        self.clear_mission_button.clicked.connect(self.clear_mission)
        self.load_mission_button.clicked.connect(self.load_predefined_mission)
        self.gorev2_button.clicked.connect(self.launch_gorev2)

        self.mode_buttons = [self.stabilize_button, self.auto_button, self.guided_button, 
                           self.upload_mission_button, self.read_mission_button, self.clear_mission_button, self.load_mission_button, self.gorev2_button,
                           self.arm_button, self.disarm_button, self.test_motors_button]
        for btn in self.mode_buttons:
            btn.setEnabled(False)

        # Görev gönder butonu da bağlantı kontrolü altına ekle
        self.mission_control_buttons = [self.send_mission_btn, self.start_mission_btn, self.stop_mission_btn]
        for btn in self.mission_control_buttons:
            btn.setEnabled(False)

        # ... (main_layout'a widget'ların eklenmesi)
        main_layout.addWidget(sidebar_scroll)
        main_layout.addWidget(self.web_view, 1) # Haritayı daha geniş yap

        # Timer'lar
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        self.attitude_timer = QTimer(self)
        self.attitude_timer.timeout.connect(self.update_attitude)
        self.motor_timer = QTimer(self)
        self.motor_timer.timeout.connect(self.update_motor_simulation)
        self.motor_timer.start(1000)  # Motor simülasyonu 1s'de bir - daha az CPU kullanımı
        
        # Grafik güncellemesi için timer
        self.graph_timer = QTimer(self)
        self.graph_timer.timeout.connect(self.update_graphs)

        self.refresh_ports()

    def select_color(self, color_value, color_name):
        """Parkur 3 için renk seçimini işler"""
        # Önce tüm butonları sıfırla
        for btn in self.color_buttons:
            btn.setChecked(False)
        
        # Seçilen butonu işaretle
        sender = self.sender()
        sender.setChecked(True)
        
        # Seçilen rengi kaydet
        self.selected_color = {"value": color_value, "name": color_name}
        self.log_message_received.emit(f"Parkur 3 renk seçimi: {color_name}")

    def send_custom_mission(self):
        """Özel görev koordinatlarını ve renk seçimini araca gönderir"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Görev göndermek için önce araca bağlanın!")
            return
        
        try:
            # Mevcut waypoint'leri temizle
            self.clear_mission()
            
            # Koordinatları topla
            mission_coords = []
            
            # Parkur 1 koordinatları (4 adet)
            for i, coord_pair in enumerate(self.parkur1_coords):
                lat = coord_pair["lat"].value()
                lon = coord_pair["lon"].value()
                mission_coords.append((lat, lon, f"Parkur1_P{i+1}"))
            
            # Parkur 2 koordinatı (1 adet)
            lat = self.parkur2_lat.value()
            lon = self.parkur2_lon.value()
            mission_coords.append((lat, lon, "Parkur2"))
            
            # Koordinatları waypoint listesine ekle ve haritaya gönder
            for lat, lon, name in mission_coords:
                waypoint_num = len(self.waypoints) + 1
                self.waypoints.append({"lat": lat, "lng": lon, "num": waypoint_num, "name": name})
                self._emit_add_waypoint_safe(lat, lon)
            
            # Seçilen rengi logla
            color_info = ""
            if self.selected_color:
                color_info = f" | Seçilen Renk: {self.selected_color['name']}"
            
            # Detaylı log mesajı
            log_message = f"Özel Görev Gönderildi ({len(mission_coords)} waypoint):{color_info}\n"
            log_message += "Parkur 1 Koordinatları:\n"
            for i, (lat, lon, name) in enumerate(mission_coords[:4]):
                log_message += f"  P{i+1}: {lat:.6f}, {lon:.6f}\n"
            log_message += f"Parkur 2 Koordinatı:\n"
            log_message += f"  {mission_coords[4][0]:.6f}, {mission_coords[4][1]:.6f}\n"
            if self.selected_color:
                log_message += f"Parkur 3 Seçilen Renk: {self.selected_color['name']}"
            
            self.log_message_received.emit(log_message)
            
            # ArduPilot'a gönder
            if len(self.waypoints) > 0:
                self.send_mission_to_vehicle()
                
                # Görev başarıyla gönderilirse başlat butonunu aktif et
                self.start_mission_btn.setVisible(True)
                self.start_mission_btn.setEnabled(True)
                self.send_mission_btn.setText("GÖREV GÖNDERİLDİ ✓")
                self.send_mission_btn.setStyleSheet("font-weight: bold; background-color: #2196F3; color: white; padding: 8px;")
                self.log_message_received.emit("Görev gönderildi! Artık 'GÖREVI BAŞLAT' butonuna basabilirsiniz.")
            
        except Exception as e:
            self.log_message_received.emit(f"Özel görev gönderme hatası: {e}")

    def arm_vehicle(self):
        """Aracı ARM eder (motorları aktif hale getirir)"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("ARM için araç bağlantısı gerekli!")
            return
        
        try:
            self.log_message_received.emit("Araç ARM ediliyor...")
            # ARM command
            self.vehicle.armed = True
            
            # ARM durumunu kontrol et
            timeout = 10  # 10 saniye timeout
            start_time = time.time()
            while not self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.vehicle.armed:
                self.log_message_received.emit("Araç başarıyla ARM edildi! Motorlar aktif.")
                self.arm_button.setText("ARMED ✓")
                self.arm_button.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 5px;")
            else:
                self.log_message_received.emit("ARM işlemi başarısız! Araç ARM edilemedi.")
                
        except Exception as e:
            self.log_message_received.emit(f"ARM hatası: {e}")

    def disarm_vehicle(self):
        """Aracı DISARM eder (motorları devre dışı bırakır)"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("DISARM için araç bağlantısı gerekli!")
            return
        
        try:
            self.log_message_received.emit("Araç DISARM ediliyor...")
            # DISARM command
            self.vehicle.armed = False
            
            # DISARM durumunu kontrol et
            timeout = 10  # 10 saniye timeout
            start_time = time.time()
            while self.vehicle.armed and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self.vehicle.armed:
                self.log_message_received.emit("Araç başarıyla DISARM edildi! Motorlar devre dışı.")
                self.arm_button.setText("ARM")
                self.arm_button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 5px;")
            else:
                self.log_message_received.emit("DISARM işlemi başarısız!")
                
        except Exception as e:
            self.log_message_received.emit(f"DISARM hatası: {e}")

    def test_motors_manual(self):
        """Motorları manuel olarak test eder"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Motor test için araç bağlantısı gerekli!")
            return
        
        if not self.vehicle.armed:
            self.log_message_received.emit("Motor test için araç ARM edilmeli!")
            return
        
        try:
            self.log_message_received.emit("Motor test başlatılıyor...")
            
            # RC Override ile motor test
            # Kanal 1: Sol thruster
            # Kanal 3: Sağ thruster (veya throttle)
            test_pwm = 1600  # Orta seviye PWM
            
            # 2 saniye motor test
            self.vehicle.channels.overrides['1'] = test_pwm  # Sol
            self.vehicle.channels.overrides['3'] = test_pwm  # Sağ/Throttle
            self.log_message_received.emit(f"Motor test: CH1={test_pwm}, CH3={test_pwm} PWM gönderildi")
            
            # 2 saniye bekle
            QTimer.singleShot(2000, self.stop_motor_test)
            
        except Exception as e:
            self.log_message_received.emit(f"Motor test hatası: {e}")
    
    def stop_motor_test(self):
        """Motor testini durdurur"""
        try:
            # Motorları durdur
            self.vehicle.channels.overrides['1'] = 1500  # Nötr
            self.vehicle.channels.overrides['3'] = 1500  # Nötr
            self.log_message_received.emit("Motor test tamamlandı - motorlar durduruldu")
            
            # 1 saniye sonra override'ları temizle
            QTimer.singleShot(1000, self.clear_motor_overrides)
            
        except Exception as e:
            self.log_message_received.emit(f"Motor test durdurma hatası: {e}")
    
    def clear_motor_overrides(self):
        """Motor override'larını temizler"""
        try:
            self.vehicle.channels.overrides = {}
            self.log_message_received.emit("Motor override'ları temizlendi")
        except Exception as e:
            self.log_message_received.emit(f"Override temizleme hatası: {e}")

    def start_custom_mission(self):
        """Gönderilen görevi başlatır (AUTO moda geçer)"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Görev başlatmak için araç bağlantısı gerekli!")
            return
            
        try:
            # Önce ARM kontrolü yap
            if not self.vehicle.armed:
                self.log_message_received.emit("Araç ARM edilmemiş. Önce ARM ediliyor...")
                self.arm_vehicle()
                
                # ARM edilmesini bekle
                timeout = 10
                start_time = time.time()
                while not self.vehicle.armed and (time.time() - start_time) < timeout:
                    time.sleep(0.1)
                
                if not self.vehicle.armed:
                    self.log_message_received.emit("ARM işlemi başarısız! Görev başlatılamadı.")
                    return
            
            # AUTO moduna geç
            self.log_message_received.emit("Görev başlatılıyor... AUTO moduna geçiliyor.")
            self.set_vehicle_mode("AUTO")
            
            # Başlat butonunu güncelle
            self.start_mission_btn.setText("GÖREV ÇALIŞIYOR...")
            self.start_mission_btn.setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white; padding: 8px; font-size: 12px;")
            self.start_mission_btn.setEnabled(False)
            
            # Durdur butonunu aktif et
            self.stop_mission_btn.setVisible(True)
            self.stop_mission_btn.setEnabled(True)
            
            self.log_message_received.emit("Görev başlatıldı! Durdurmak için 'GÖREVI DURDUR' butonuna basın.")
            
        except Exception as e:
            self.log_message_received.emit(f"Görev başlatma hatası: {e}")

    def stop_custom_mission(self):
        """Çalışan görevi durdurur (MANUAL moda geçer)"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Görev durdurmak için araç bağlantısı gerekli!")
            return
            
        try:
            # Görev 2 çalışıyorsa onu durdur
            if self.gorev2_running:
                self.terminate_gorev2()
                self.log_message_received.emit("Görev 2 durduruldu!")
            
            # MANUAL moduna geç
            self.log_message_received.emit("Görev durduruluyor... MANUAL moduna geçiliyor.")
            self.set_vehicle_mode("MANUAL")
            
            # Thruster'ları durdur
            self.stop_thrusters()
            
            # Buton durumlarını güncelle
            self.stop_mission_btn.setText("GÖREV DURDURULDU ✓")
            self.stop_mission_btn.setStyleSheet("font-weight: bold; background-color: #795548; color: white; padding: 8px; font-size: 12px;")
            self.stop_mission_btn.setEnabled(False)
            
            self.start_mission_btn.setText("GÖREV DURDURULDU")
            self.start_mission_btn.setStyleSheet("font-weight: bold; background-color: #9E9E9E; color: white; padding: 8px; font-size: 12px;")
            
            self.log_message_received.emit("Görev durduruldu! Araç MANUAL modunda.")
            
            # 3 saniye sonra butonları sıfırla
            QTimer.singleShot(3000, self._reset_mission_buttons)
            
        except Exception as e:
            self.log_message_received.emit(f"Görev durdurma hatası: {e}")

    def reset_mission_ui(self):
        """Görev UI'sını sıfırlar"""
        # 3 saniye sonra butonları sıfırla
        QTimer.singleShot(3000, self._reset_mission_buttons)
    
    def _reset_mission_buttons(self):
        """Görev butonlarını sıfırlar"""
        self.send_mission_btn.setText("GÖREV GÖNDER")
        self.send_mission_btn.setStyleSheet("font-weight: bold; background-color: #4CAF50; color: white; padding: 8px;")
        
        self.start_mission_btn.setText("🚀 GÖREVI BAŞLAT")
        self.start_mission_btn.setStyleSheet("font-weight: bold; background-color: #FF9800; color: white; padding: 8px; font-size: 12px;")
        self.start_mission_btn.setVisible(False)
        self.start_mission_btn.setEnabled(False)
        
        self.stop_mission_btn.setText("⏹️ GÖREVI DURDUR")
        self.stop_mission_btn.setStyleSheet("font-weight: bold; background-color: #F44336; color: white; padding: 8px; font-size: 12px;")
        self.stop_mission_btn.setVisible(False)
        self.stop_mission_btn.setEnabled(False)

    def read_custom_mission_from_vehicle(self):
        """Aractan okunan misyonu özel formatta gösterir"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota okumak için araç bağlantısı gerekli.")
            return
        
        # Önce normal misyon okuma işlemini yap
        self.read_mission_from_vehicle()
        
        # Okunan waypoint'leri analiz et ve özel format ile logla
        if len(self.waypoints) >= 5:
            log_message = f"Aractan Okunan Özel Görev ({len(self.waypoints)} waypoint):\n"
            log_message += "Parkur 1 Koordinatları:\n"
            for i in range(min(4, len(self.waypoints))):
                wp = self.waypoints[i]
                log_message += f"  P{i+1}: {wp['lat']:.6f}, {wp['lng']:.6f}\n"
            
            if len(self.waypoints) >= 5:
                wp = self.waypoints[4]
                log_message += f"Parkur 2 Koordinatı:\n"
                log_message += f"  {wp['lat']:.6f}, {wp['lng']:.6f}\n"
            
            if self.selected_color:
                log_message += f"Seçili Renk: {self.selected_color['name']}"
            
            self.log_message_received.emit(log_message)

    def launch_gorev2(self):
        try:
            if not self.is_connected or not self.vehicle:
                self.log_message_received.emit("Görev 2 için önce araca bağlanın.")
                return
            if self.gorev2_running:
                self.log_message_received.emit("Görev 2 zaten çalışıyor.")
                return
            # Haritada Görev 2 hedefini göster (görev butonundaki gibi)
            try:
                # Mevcut waypoint'leri temizle ve tüm hedefleri sırayla ekle
                self.clear_mission()
                for lat, lon in self.gorev2_targets:
                    self.add_waypoint_to_list(lat, lon)
                self.log_message_received.emit(f"Görev 2 hedefleri haritaya eklendi: {len(self.gorev2_targets)} adet")
            except Exception as e:
                self.log_message_received.emit(f"Görev 2 hedefleri haritaya eklenemedi: {e}")
            
            # Durdur butonunu Görev 2 için de aktif et
            self.stop_mission_btn.setVisible(True)
            self.stop_mission_btn.setEnabled(True)
            self.stop_mission_btn.setText("⏹️ GÖREV 2'Yİ DURDUR")
            
            # Otonom kayıt sistemini başlat (Görev 2 otonom modda çalışır)
            if not self.autonomous_recording.is_recording:
                self.autonomous_recording.start_recording()
            
            self.gorev2_running = True
            self.log_message_received.emit("Görev 2 başlatılıyor (uygulama içi)...")
            self._gorev2_thread = threading.Thread(target=self._gorev2_worker, daemon=True)
            self._gorev2_thread.start()
        except Exception as e:
            self.log_message_received.emit(f"Görev 2 başlatma hatası: {e}")

    def _gorev2_worker(self):
        import cv2
        import numpy as np
        try:
            # Parametreler (erkan_denendi.py ile uyumlu)
            PWM_STOP = 1500
            PWM_FAST = 1800
            TARGET_LAT = self.gorev2_target[0]
            TARGET_LON = self.gorev2_target[1]
            obstacle_detected = False
            obstacle_avoidance_active = False
            avoidance_start_time = 0
            avoidance_stage = 0
            current_mode = "MANUAL"
            last_nav_update = 0

            def calculate_bearing(lat1, lon1, lat2, lon2):
                lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
                dlon = lon2 - lon1
                y = math.sin(dlon) * math.cos(lat2)
                x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
                bearing = math.atan2(y, x)
                bearing = math.degrees(bearing)
                return (bearing + 360) % 360

            def calculate_distance(lat1, lon1, lat2, lon2):
                dlat = lat2 - lat1
                dlon = lon2 - lon1
                lat_to_m = 111000
                lon_to_m = 111000 * math.cos(math.radians(lat1))
                return math.sqrt((dlat * lat_to_m)**2 + (dlon * lon_to_m)**2)

            def bearing_to_motor_command(target_bearing, current_heading):
                bearing_diff = target_bearing - current_heading
                if bearing_diff > 180:
                    bearing_diff -= 360
                elif bearing_diff < -180:
                    bearing_diff += 360
                if abs(bearing_diff) < 25:
                    thr, steer = PWM_FAST, PWM_STOP
                elif bearing_diff > 0:
                    steer_offset = min(180, abs(bearing_diff) * 2.2)
                    thr = PWM_FAST
                    steer = PWM_STOP - steer_offset
                else:
                    steer_offset = min(180, abs(bearing_diff) * 2.2)
                    thr = PWM_FAST
                    steer = PWM_STOP + steer_offset
                return thr, steer

            def send_rc(throttle_pwm: int, steer_pwm: int):
                if not self.vehicle:
                    return
                throttle_pwm = max(1100, min(1900, int(throttle_pwm)))
                steer_pwm = max(1100, min(1900, int(steer_pwm)))
                try:
                    self.vehicle.channels.overrides['3'] = throttle_pwm
                    self.vehicle.channels.overrides['1'] = steer_pwm
                    self.vehicle.channels.overrides['2'] = steer_pwm
                except Exception as e:
                    self.log_message_received.emit(f"[GÖREV 2] RC gönderim hatası: {e}")

            def stop_all():
                send_rc(PWM_STOP, PWM_STOP)

            def detect_obstacle_position(roi, mask_yellow):
                h, w = roi.shape[:2]
                left_part = mask_yellow[:, :w//3]
                center_part = mask_yellow[:, w//3:2*w//3]
                right_part = mask_yellow[:, 2*w//3:]
                left_density = np.count_nonzero(left_part) / (left_part.shape[0] * left_part.shape[1])
                center_density = np.count_nonzero(center_part) / (center_part.shape[0] * center_part.shape[1])
                right_density = np.count_nonzero(right_part) / (right_part.shape[0] * right_part.shape[1])
                max_density = max(left_density, center_density, right_density)
                if max_density < 0.02:
                    return "none"
                elif left_density == max_density:
                    return "left"
                elif right_density == max_density:
                    return "right"
                else:
                    return "center"

            def obstacle_avoidance_maneuver(obstacle_position):
                nonlocal obstacle_avoidance_active, avoidance_start_time, avoidance_stage
                current_time = time.time()
                if not obstacle_avoidance_active:
                    obstacle_avoidance_active = True
                    avoidance_start_time = current_time
                    avoidance_stage = 1
                    self.log_message_received.emit(f"[GÖREV 2] ENGEL ATLAMA BAŞLADI - Engel konumu: {obstacle_position}")
                elapsed = current_time - avoidance_start_time
                if avoidance_stage == 1:
                    if obstacle_position == "left":
                        send_rc(PWM_FAST, PWM_STOP + 200)
                    elif obstacle_position == "right":
                        send_rc(PWM_FAST, PWM_STOP - 200)
                    else:
                        direction = 1 if (current_time % 2) > 1 else -1
                        send_rc(PWM_FAST, PWM_STOP + (200 * direction))
                    if elapsed > 2.0:
                        avoidance_stage = 2
                        avoidance_start_time = current_time
                elif avoidance_stage == 2:
                    send_rc(PWM_FAST, PWM_STOP)
                    if elapsed > 1.5:
                        avoidance_stage = 3
                        avoidance_start_time = current_time
                elif avoidance_stage == 3:
                    if obstacle_position == "left":
                        send_rc(PWM_FAST, PWM_STOP - 180)
                    elif obstacle_position == "right":
                        send_rc(PWM_FAST, PWM_STOP + 180)
                    else:
                        direction = -1 if (current_time % 2) > 1 else 1
                        send_rc(PWM_FAST, PWM_STOP + (180 * direction))
                    if elapsed > 1.5:
                        obstacle_avoidance_active = False
                        avoidance_stage = 0
                        self.log_message_received.emit("[GÖREV 2] ENGEL ATLAMA TAMAMLANDI")
                        return False
                return True

            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                self.log_message_received.emit("[GÖREV 2] Kamera açılamadı")
                self.gorev2_running = False
                return
            self.log_message_received.emit("[GÖREV 2] Kamera hazır. 'q' ile kapatabilirsiniz.")
            # Kayıt hazırlıkları
            recordings_dir = os.path.join(os.path.dirname(__file__), 'logs')
            os.makedirs(recordings_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            cam_path = os.path.join(recordings_dir, f"camera_processed_{ts}.mp4")
            map_path = os.path.join(recordings_dir, f"local_obstacle_map_{ts}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            ok_probe, frame_probe = cap.read()
            if not ok_probe:
                self.log_message_received.emit("[GÖREV 2] İlk kare okunamadı, kayıt açılamadı")
                self.gorev2_running = False
                cap.release()
                return
            h0, w0 = frame_probe.shape[:2]
            fps_out = 10  # >=1 Hz
            cam_writer = cv2.VideoWriter(cam_path, fourcc, fps_out, (w0, h0))
            map_writer = cv2.VideoWriter(map_path, fourcc, fps_out, (w0, h0))
            self.log_message_received.emit(f"[GÖREV 2] Kayıt başlatıldı: {cam_path} ve {map_path}")
            # Okunan ilk kareyi akışa geri koyamayız, devam edelim
            while self.gorev2_running:
                ok, frame = cap.read()
                if not ok:
                    continue
                # GPS/Heading oku
                current_lat = None
                current_lon = None
                current_heading = None
                if self.vehicle and hasattr(self.vehicle, 'location') and self.vehicle.location and hasattr(self.vehicle.location, 'global_frame'):
                    gf = self.vehicle.location.global_frame
                    current_lat = getattr(gf, 'lat', None)
                    current_lon = getattr(gf, 'lon', None)
                if self.vehicle:
                    current_heading = getattr(self.vehicle, 'heading', None)

                if current_lat is not None and current_lon is not None and current_heading is not None:
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
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask_yellow = cv2.inRange(hsv, (18, 140, 80), (38, 255, 255))
                yellow_ratio = np.count_nonzero(mask_yellow) / (roi.shape[0] * roi.shape[1])
                obstacle_position = detect_obstacle_position(roi, mask_yellow)

                # Engel kontrolü
                if yellow_ratio > 0.15:
                    if not obstacle_detected:
                        self.log_message_received.emit(f"[GÖREV 2] ENGEL ALGILANDI! {yellow_ratio*100:.1f}% - Konum: {obstacle_position}")
                        obstacle_detected = True
                else:
                    if obstacle_detected:
                        self.log_message_received.emit("[GÖREV 2] ENGEL TEMİZLENDİ")
                    obstacle_detected = False
                    if obstacle_avoidance_active:
                        obstacle_avoidance_active = False
                        avoidance_stage = 0
                        self.log_message_received.emit("[GÖREV 2] ENGEL ATLAMA İPTAL - ENGEL YOK")

                # Otomatik navigasyon
                now = time.time()
                if current_mode == "AUTO_GPS" and current_lat is not None and (now - last_nav_update > 0.5):
                    if obstacle_detected and yellow_ratio > 0.15 and not obstacle_avoidance_active:
                        obstacle_avoidance_maneuver(obstacle_position)
                        last_nav_update = now
                    elif obstacle_avoidance_active:
                        maneuver_active = obstacle_avoidance_maneuver(obstacle_position)
                        last_nav_update = now
                    else:
                        if distance > 2 and current_heading is not None:
                            thr, steer = bearing_to_motor_command(target_bearing, current_heading)
                            send_rc(thr, steer)
                        else:
                            stop_all()
                            self.log_message_received.emit("[GÖREV 2] HEDEFE VARILDI!")
                            current_mode = "MANUAL"
                        last_nav_update = now

                # Görsel overlay
                cv2.rectangle(frame, (x0, y0), (x0+rw, y0+rh), (0,255,0), 2)
                part_w = rw // 3
                cv2.line(frame, (x0 + part_w, y0), (x0 + part_w, y0 + rh), (255,255,0), 1)
                cv2.line(frame, (x0 + 2*part_w, y0), (x0 + 2*part_w, y0 + rh), (255,255,0), 1)
                status_color = (0,255,0) if current_mode == "MANUAL" else (0,255,255)
                avoidance_text = " - ENGEL ATLAMA" if obstacle_avoidance_active else ""
                cv2.putText(frame, f"Mod: {current_mode}{avoidance_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                cv2.putText(frame, gps_status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
                cv2.putText(frame, distance_status, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                cv2.putText(frame, bearing_status, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
                obstacle_color = (0,0,255) if obstacle_detected else (0,255,255)
                obstacle_info = f"Engel: {yellow_ratio*100:.1f}%"
                if obstacle_detected:
                    obstacle_info += f" ({obstacle_position})"
                cv2.putText(frame, obstacle_info, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, obstacle_color, 2)
                if obstacle_avoidance_active:
                    stage_names = ["", "Yan Hareket", "Düz Git", "Geri Dön"]
                    stage_text = f"Aşama: {stage_names[avoidance_stage]}"
                    cv2.putText(frame, stage_text, (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,100,100), 2)

                cv2.imshow('Görev 2 Kamera', frame)
                # Zaman etiketi ekle ve videolara yaz
                ts_text = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] + 'Z'
                frame_to_write = frame.copy()
                cv2.putText(frame_to_write, f"TS: {ts_text}", (10, h0 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                overlay_map = np.zeros_like(frame)
                overlay_map[mask_yellow > 0] = (0, 255, 255)
                map_vis = cv2.addWeighted(frame, 0.3, overlay_map, 0.7, 0)
                cv2.putText(map_vis, f"TS: {ts_text}", (10, h0 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
                try:
                    cam_writer.write(frame_to_write)
                    map_writer.write(map_vis)
                except Exception:
                    pass
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('g'):
                    current_mode = "AUTO_GPS"
                    self.log_message_received.emit("[GÖREV 2] GPS otomatik modu aktif")
                    try:
                        self.set_vehicle_mode("AUTO")
                    except Exception:
                        pass
                elif key == ord('1'):
                    current_mode = "MANUAL"
                    send_rc(PWM_FAST, PWM_STOP)
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass
                elif key == ord('2'):
                    current_mode = "MANUAL"
                    send_rc(PWM_FAST, PWM_STOP + 100)
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass
                elif key == ord('3'):
                    current_mode = "MANUAL"
                    send_rc(PWM_FAST, PWM_STOP - 100)
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass
                elif key == ord('4'):
                    current_mode = "MANUAL"
                    obstacle_detected = True
                    obstacle_avoidance_maneuver("left")
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass
                elif key == ord('5'):
                    current_mode = "MANUAL"
                    obstacle_detected = True
                    obstacle_avoidance_maneuver("right")
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass
                elif key == ord('6'):
                    current_mode = "MANUAL"
                    obstacle_detected = True
                    obstacle_avoidance_maneuver("center")
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass
                elif key == ord('r'):
                    self.log_message_received.emit("[GÖREV 2] Reset tuşu (r) sadece simülasyonda etkili")
                elif key == ord('s'):
                    current_mode = "MANUAL"
                    stop_all()
                    try:
                        self.set_vehicle_mode("MANUAL")
                    except Exception:
                        pass

            cap.release()
            try:
                cam_writer.release()
                map_writer.release()
            except Exception:
                pass
            cv2.destroyAllWindows()
            self.log_message_received.emit("[GÖREV 2] Kapatıldı.")
        except Exception as e:
            self.log_message_received.emit(f"[GÖREV 2] Hata: {e}")
        finally:
            self.gorev2_running = False

    def terminate_gorev2(self):
        try:
            if self.gorev2_running:
                self.log_message_received.emit("Görev 2 sonlandırılıyor...")
                self.gorev2_running = False
                
                # Otonom kayıt sistemini durdur
                if self.autonomous_recording.is_recording:
                    self.autonomous_recording.stop_recording()
                    
            if self.gorev2_process and self.gorev2_process.poll() is None:
                self.gorev2_process.terminate()
        except Exception as e:
            self.log_message_received.emit(f"Görev 2 sonlandırma hatası: {e}")

    def closeEvent(self, event):
        try:
            # Görev 2'yi sonlandır
            self.terminate_gorev2()
            
            # Otonom kayıt sistemini durdur
            if self.autonomous_recording.is_recording:
                self.autonomous_recording.stop_recording()
                
        finally:
            super().closeEvent(event)

    def keyPressEvent(self, event):
        try:
            if event.key() in (Qt.Key_G,):
                # 'g' → otomatik (AUTO)
                self.set_vehicle_mode("AUTO")
                self.log_message_received.emit("Klavye: 'g' algılandı → AUTO moda geçiliyor")
                event.accept()
                return
            if event.key() in (Qt.Key_S,):
                # 's' → manuel (MANUAL) ve thruster durdur
                self.set_vehicle_mode("MANUAL")
                self.stop_thrusters()
                self.log_message_received.emit("Klavye: 's' algılandı → MANUAL ve motorlar durduruldu")
                event.accept()
                return
        except Exception as e:
            self.log_message_received.emit(f"Klavye kısayol hatası: {e}")
        super().keyPressEvent(event)
    
    def setup_web_channel(self):
        self.bridge = MapBridge(self)
        self.channel = QWebChannel(self.web_view.page())
        self.web_view.page().setWebChannel(self.channel)
        self.channel.registerObject('py_bridge', self.bridge)
        
        # Harita yüklenince bekleyen işlemleri uygula
        try:
            self.web_view.loadFinished.connect(self.on_map_load_finished)
        except Exception:
            pass
        
        # JavaScript'ten gelen waypoint ekleme/silme isteklerini yakala
        self.bridge.waypoint_from_user.connect(self.add_waypoint_to_list)
        self.bridge.waypoint_removed.connect(self.remove_waypoint_from_list)
        self.bridge.mapReady.connect(self.on_js_channel_ready)

        map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'map.html'))
        self.web_view.setUrl(QUrl.fromLocalFile(map_path))

    def on_js_channel_ready(self):
        """JS tarafı QWebChannel bağlantısını kurduğunda çağrılır."""
        self.map_ready = True
        # Küçük bir gecikmeden sonra bekleyen işlemleri uygula
        QTimer.singleShot(100, lambda: self.on_map_load_finished(True))

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_from_vehicle()
        else:
            self.connect_to_vehicle()

    def connect_to_vehicle(self):
        connection_type = self.connection_type.currentText()
        
        if connection_type == "UDP (Kablosuz)":
            ip = int(self.ip_input.value())
            udp_port = int(self.udp_port_input.value())
            connection_string = f"udp:127.0.0.1:{udp_port}"
            self.status_message_received.emit(f"UDP bağlantısı kuruluyor... ({connection_string})")
        elif connection_type == "TCP (WiFi)":
            ip = int(self.ip_input.value())
            connection_string = f"tcp:192.168.1.{ip}:5760"
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

    @pyqtSlot(bool, str)
    def on_connection_status_changed(self, connected, message):
        self.is_connected = connected
        self.status_message_received.emit(message)
        self.log_message_received.emit(message)
        
        if connected:
            self.connect_button.setText("BAĞLANTIYI KES")
            self.connect_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            self.telemetry_timer.start(500)  # 500ms = 2Hz - daha hızlı güncelleme
            self.attitude_timer.start(50)   # 50ms = 20Hz - daha smooth attitude
            self.graph_timer.start(1000)    # 1s = 1Hz - grafik güncellemesi
            
            # SERVO_OUTPUT_RAW mesajını request et
            self.request_servo_output()
            
            # Bağlandıktan hemen sonra telemetri güncelle
            QTimer.singleShot(100, self.update_telemetry)  # 100ms sonra
            # Thruster verisi için hemen motor güncelleme başlat
            QTimer.singleShot(200, self.update_motor_simulation)  # 200ms sonra
            QTimer.singleShot(500, self.update_motor_simulation)  # 500ms sonra da bir kez daha
            for btn in self.mode_buttons:
                btn.setEnabled(True)
            # Sadece send_mission_btn'i aktif et, diğer görev butonları durum bazlı aktif olacak
            self.send_mission_btn.setEnabled(True)
            # Telemetri CSV kaydını başlat
            self.start_telemetry_logging()
        else:
            self.connect_button.setText("BAĞLAN")
            self.connect_button.setStyleSheet("")
            self.telemetry_timer.stop()
            self.attitude_timer.stop()
            self.graph_timer.stop()
            for btn in self.mode_buttons:
                btn.setEnabled(False)
            # Tüm görev butonlarını devre dışı bırak
            self.send_mission_btn.setEnabled(False)
            self.start_mission_btn.setEnabled(False)
            self.stop_mission_btn.setEnabled(False)
            # Bağlantı kesildiğinde görev butonlarını sıfırla
            self._reset_mission_buttons()
            self.vehicle = None
            
            # Cache'i temizle
            self.servo_output_cache.clear()
            
            # Grafik verilerini temizle
            self.clear_graph_data()
            # Telemetri CSV kaydını durdur
            self.stop_telemetry_logging()
            
            # Otonom kayıt sistemini durdur
            if self.autonomous_recording.is_recording:
                self.autonomous_recording.stop_recording()

    def request_servo_output(self):
        """ArduPilot'tan SERVO_OUTPUT_RAW mesajını request et"""
        if self.vehicle:
            try:
                # SERVO_OUTPUT_RAW mesajını aktif et
                msg = self.vehicle.message_factory.request_data_stream_encode(
                    self.vehicle._master.target_system,
                    self.vehicle._master.target_component,
                    2,  # MAV_DATA_STREAM_EXTENDED_STATUS
                    2,  # 2 Hz rate
                    1   # start_stop (1=start)
                )
                self.vehicle.send_mavlink(msg)
                self.log_message_received.emit("SERVO_OUTPUT_RAW stream başlatıldı (2Hz)")
                
                # Ek olarak: SET_MESSAGE_INTERVAL ile mesaj ID bazlı zorla (daha güvenilir)
                try:
                    from pymavlink import mavutil
                    set_interval = self.vehicle.message_factory.command_long_encode(
                        self.vehicle._master.target_system,
                        self.vehicle._master.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0,
                        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
                        200000,  # 5 Hz (µs)
                        0, 0, 0, 0, 0
                    )
                    self.vehicle.send_mavlink(set_interval)
                    self.log_message_received.emit("SET_MESSAGE_INTERVAL: SERVO_OUTPUT_RAW 5Hz ayarlandı")
                except Exception as e_set:
                    self.log_message_received.emit(f"SET_MESSAGE_INTERVAL gönderilemedi: {e_set}")
                
                # Servo function'ları da alalım - daha hızlı başlat
                QTimer.singleShot(500, self.log_servo_functions)  # 500ms sonra (eskiden 2000ms)
                
            except Exception as e:
                self.log_message_received.emit(f"SERVO_OUTPUT request hatası: {e}")

    def log_servo_functions(self):
        """Servo function parametrelerini logla"""
        if self.vehicle and hasattr(self.vehicle, 'parameters'):
            try:
                # Vehicle tipi ve versiyonu
                vehicle_type = getattr(self.vehicle, '_vehicle_type', 'UNKNOWN')
                version = getattr(self.vehicle, 'version', 'UNKNOWN')
                
                self.log_message_received.emit(f"Vehicle: {vehicle_type}, Version: {version}")
                
                # Servo function'ları
                servo1_func = self.vehicle.parameters.get('SERVO1_FUNCTION', None)
                servo2_func = self.vehicle.parameters.get('SERVO2_FUNCTION', None)
                servo3_func = self.vehicle.parameters.get('SERVO3_FUNCTION', None)
                servo4_func = self.vehicle.parameters.get('SERVO4_FUNCTION', None)
                
                # Frame class ve type
                frame_class = self.vehicle.parameters.get('FRAME_CLASS', None)
                frame_type = self.vehicle.parameters.get('FRAME_TYPE', None)
                
                self.log_message_received.emit(f"Frame: CLASS={frame_class}, TYPE={frame_type}")
                self.log_message_received.emit(f"Servo Functions: S1={servo1_func}, S2={servo2_func}, S3={servo3_func}, S4={servo4_func}")
                
                # Motor çıkış kanalları kontrol et
                self.debug_all_servo_channels()
                
            except Exception as e:
                self.log_message_received.emit(f"Vehicle info okuma hatası: {e}")

    def debug_all_servo_channels(self):
        """Tüm servo kanallarını debug et"""
        if self.vehicle:
            try:
                # DroneKit channels attribute'u kontrol et
                self.log_message_received.emit("=== CHANNELS DEBUG ===")
                
                if hasattr(self.vehicle, 'channels'):
                    if self.vehicle.channels is not None:
                        self.log_message_received.emit(f"✓ vehicle.channels type: {type(self.vehicle.channels)}")
                        
                        # Channels içindeki attribute'ları kontrol et
                        channels_attrs = dir(self.vehicle.channels)
                        self.log_message_received.emit(f"Channels attributes: {[attr for attr in channels_attrs if not attr.startswith('_')]}")
                        
                        # Eğer channels bir dict-like object ise, içeriğini göster
                        try:
                            channels_dict = dict(self.vehicle.channels)
                            self.log_message_received.emit(f"Channels dict: {channels_dict}")
                        except:
                            self.log_message_received.emit("Channels dict'e çevrilemedi")
                        
                        # Override durumu
                        if hasattr(self.vehicle.channels, 'overrides'):
                            self.log_message_received.emit(f"Overrides mevcut: {self.vehicle.channels.overrides}")
                        else:
                            self.log_message_received.emit("Overrides attribute yok")
                            
                    else:
                        self.log_message_received.emit("✗ vehicle.channels = None")
                else:
                    self.log_message_received.emit("✗ vehicle.channels attribute yok")
                
                # Servo output raw message listener ekle - CACHE'E KAYDET
                def servo_output_listener(vehicle, name, message):
                    # Servo değerlerini parse et
                    servo_values = [
                        message.servo1_raw, message.servo2_raw, message.servo3_raw, message.servo4_raw,
                        message.servo5_raw, message.servo6_raw, message.servo7_raw, message.servo8_raw
                    ]
                    active_servos = [(i+1, val) for i, val in enumerate(servo_values) if val != 0]
                    self.log_message_received.emit(f"✓ SERVO_OUTPUT_RAW alındı! Aktif: {active_servos}")
                    
                    # Cache'e kaydet - DroneKit channels güncellenmiyor
                    for i, val in enumerate(servo_values):
                        if val != 0:  # Sadece aktif kanalları kaydet
                            self.servo_output_cache[str(i+1)] = val
                                
                self.vehicle.on_message('SERVO_OUTPUT_RAW')(servo_output_listener)
                self.log_message_received.emit("✓ SERVO_OUTPUT_RAW listener eklendi")
                
                # Periyodik channel durumu kontrol et - daha hızlı başlat
                QTimer.singleShot(1000, self.periodic_channel_check)  # 1 saniye sonra (eskiden 5000ms)
                
            except Exception as e:
                self.log_message_received.emit(f"Servo debug hatası: {e}")
                import traceback
                self.log_message_received.emit(f"Stack trace: {traceback.format_exc()}")

    def periodic_channel_check(self):
        """Periyodik olarak channel durumunu kontrol et"""
        if self.vehicle:
            try:
                # Cache durumunu kontrol et
                ch1 = self.servo_output_cache.get('1', 'YOK')
                ch2 = self.servo_output_cache.get('2', 'YOK') 
                ch3 = self.servo_output_cache.get('3', 'YOK')
                ch4 = self.servo_output_cache.get('4', 'YOK')
                
                cache_count = len([x for x in [ch1, ch2, ch3, ch4] if x != 'YOK'])
                self.log_message_received.emit(f"Cache check: CH1={ch1}, CH2={ch2}, CH3={ch3}, CH4={ch4} (Aktif: {cache_count})")
                    
                # 10 saniye sonra tekrar kontrol et
                QTimer.singleShot(10000, self.periodic_channel_check)
                
            except Exception as e:
                self.log_message_received.emit(f"Periyodik check hatası: {e}")

    def change_mode(self, mode_name):
        """Vehicle modunu değiştir"""
        if self.vehicle:
            try:
                self.vehicle.mode = VehicleMode(mode_name)
                self.log_message_received.emit(f"Mod değiştirildi: {mode_name}")
            except Exception as e:
                self.log_message_received.emit(f"Mod değiştirme hatası: {e}")

    def test_thruster(self, side):
        """Manuel thruster test - PWM komutları gönder"""
        if not self.vehicle:
            self.log_message_received.emit("Thruster test: Vehicle bağlı değil")
            return
            
        try:
            # Önce channels.overrides'ı initialize et
            if not hasattr(self.vehicle.channels, 'overrides'):
                self.log_message_received.emit("channels.overrides initialize ediliyor...")
                self.vehicle.channels.overrides = {}
            
            # Mevcut override durumunu logla
            current_overrides = getattr(self.vehicle.channels, 'overrides', {})
            self.log_message_received.emit(f"Mevcut overrides: {current_overrides}")
            
            if side == 'left':
                # Sol thruster test: Kanal 2'ye 1600μs (60% güç)
                self.vehicle.channels.overrides['2'] = 1600
                self.log_message_received.emit("Sol thruster test: CH2=1600μs GÖNDER")
                
                # Doğrulama
                if '2' in self.vehicle.channels.overrides:
                    self.log_message_received.emit(f"Sol override doğrulandı: CH2={self.vehicle.channels.overrides['2']}")
                else:
                    self.log_message_received.emit("SOL OVERRIDE BAŞARISIZ!")
                    
            elif side == 'right':
                # Sağ thruster test: Kanal 1'e 1600μs (60% güç)  
                self.vehicle.channels.overrides['1'] = 1600
                self.log_message_received.emit("Sağ thruster test: CH1=1600μs GÖNDER")
                
                # Doğrulama
                if '1' in self.vehicle.channels.overrides:
                    self.log_message_received.emit(f"Sağ override doğrulandı: CH1={self.vehicle.channels.overrides['1']}")
                else:
                    self.log_message_received.emit("SAĞ OVERRIDE BAŞARISIZ!")
            
            # Override sonrası durumu logla
            final_overrides = getattr(self.vehicle.channels, 'overrides', {})
            self.log_message_received.emit(f"Test sonrası overrides: {final_overrides}")
                
        except Exception as e:
            self.log_message_received.emit(f"Thruster test hatası: {e}")
            import traceback
            self.log_message_received.emit(f"Stack trace: {traceback.format_exc()}")

    def stop_thrusters(self):
        """Tüm thruster'ları durdur"""
        if not self.vehicle:
            return
            
        try:
            # Deniz aracı için yaygın kanal konfigürasyonları:
            # CH1: Sağ thruster, CH3: Sol thruster veya throttle
            self.vehicle.channels.overrides['1'] = 1500  # Sağ thruster nötr
            self.vehicle.channels.overrides['3'] = 1500  # Sol thruster/throttle nötr
            self.log_message_received.emit("Thruster'lar durduruldu: CH1=CH3=1500μs")
            
        except Exception as e:
            self.log_message_received.emit(f"Thruster durdurma hatası: {e}")

    def disconnect_from_vehicle(self):
        if self.vehicle:
            self.vehicle.close()
        self.on_connection_status_changed(False, "Bağlantı sonlandırıldı.")

    def refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if not ports:
            self.port_combo.addItem("Port bulunamadı")

    def update_telemetry(self):
        if not self.is_connected or not self.vehicle:
            return

        try:
            # Zaman damgası ekle
            current_time = time.time()
            self.time_data.append(current_time)
            
            # GERÇEK ARDUPİLOT VERİLERİ - sadece mevcut olanları al
            speed = getattr(self.vehicle, 'groundspeed', None)
            heading = getattr(self.vehicle, 'heading', None)  
            mode = getattr(self.vehicle.mode, 'name', None) if hasattr(self.vehicle, 'mode') else None
            
            # Otonom kayıt sistemi - mod değişikliği kontrolü
            if mode != self.last_vehicle_mode:
                self._handle_mode_change(self.last_vehicle_mode, mode)
                self.last_vehicle_mode = mode
            
            # Location güvenli alım
            alt = None
            if hasattr(self.vehicle, 'location') and self.vehicle.location:
                if hasattr(self.vehicle.location, 'global_relative_frame'):
                    alt = getattr(self.vehicle.location.global_relative_frame, 'alt', None)
            
            # Armed durumu
            armed = getattr(self.vehicle, 'armed', None)
            
            # Grafik verileri için değerleri topla
            if speed is not None:
                self.speed_data.append(speed)
            else:
                self.speed_data.append(0)
                
            if heading is not None:
                self.heading_data.append(heading)
            else:
                self.heading_data.append(0)
            
            # Veri eksikse: gösterme 
            if speed is None:
                self.telemetry_values["Hız:"].setText("Veri yok")
            else:
                self.telemetry_values["Hız:"].setText(f"{speed:.1f} m/s")
                
            if heading is None:
                self.telemetry_values["Heading:"].setText("Veri yok")
            else:
                self.telemetry_values["Heading:"].setText(f"{heading}°")
                
            if alt is None:
                self.telemetry_values["Yükseklik:"].setText("Veri yok") 
            else:
                self.telemetry_values["Yükseklik:"].setText(f"{alt:.1f} m")
                
            if mode is None:
                self.telemetry_values["Mod:"].setText("UNKNOWN")
            else:
                self.telemetry_values["Mod:"].setText(mode)
            
            # GERÇEK SETPOINT VERİLERİ - ArduPilot'tan al
            speed_setpoint = 0
            heading_setpoint = 0
            
            if mode in ["AUTO", "GUIDED", "RTL"]:
                # Heading Setpoint: Aktif waypoint varsa bearing hesapla
                if len(self.waypoints) > 0 and heading is not None:
                    heading_setpoint = self.get_target_heading_from_mission(heading)
                    self.telemetry_values["Heading Setpoint:"].setText(f"{heading_setpoint:.0f}°")
                else:
                    self.telemetry_values["Heading Setpoint:"].setText("Veri yok")

                # Hız Setpoint: WPNAV_SPEED veya WP_SPEED parametresinden al (cm/s → m/s)
                wpnav_speed = self.vehicle.parameters.get('WPNAV_SPEED', None)
                wp_speed = self.vehicle.parameters.get('WP_SPEED', None)
                speed_param = wpnav_speed if wpnav_speed is not None else wp_speed
                if speed_param is not None:
                    speed_setpoint = speed_param / 100.0  # cm/s → m/s
                    self.telemetry_values["Hız Setpoint:"].setText(f"{speed_setpoint:.2f} m/s")
                else:
                    self.telemetry_values["Hız Setpoint:"].setText("Veri yok")
            else:  # MANUAL, STABILIZE, etc.
                # Manuel modda: setpoint YOK
                self.telemetry_values["Hız Setpoint:"].setText("MANUAL")
                self.telemetry_values["Heading Setpoint:"].setText("MANUAL")
            
            # Grafik için setpoint verilerini topla
            self.speed_setpoint_data.append(speed_setpoint)
            self.heading_setpoint_data.append(heading_setpoint)
            
            self.telemetry_values["Yükseklik:"].setText(f"{alt:.1f} m")
            self.telemetry_values["Heading:"].setText(f"{heading}°")
            
            # Attitude - basit
            pitch_deg = math.degrees(self.vehicle.attitude.pitch)
            yaw_deg = math.degrees(self.vehicle.attitude.yaw)
            self.telemetry_values["Pitch:"].setText(f"{pitch_deg:.1f}°")
            self.telemetry_values["Yaw:"].setText(f"{yaw_deg:.1f}°")
            
            self.telemetry_values["Mod:"].setText(mode)
            
            # ARM durumu
            if armed is not None:
                arm_status = "ARMED" if armed else "DISARMED"
                arm_color = "color: green;" if armed else "color: red;"
                self.telemetry_values["ARM Durumu:"].setText(arm_status)
                self.telemetry_values["ARM Durumu:"].setStyleSheet(f"font-weight: bold; {arm_color}")
            else:
                self.telemetry_values["ARM Durumu:"].setText("N/A")

            if self.vehicle.battery and self.vehicle.battery.level is not None:
                battery_level = self.vehicle.battery.level
                self.telemetry_values["Batarya:"].setText(f"{battery_level}%")
                self.battery_progress.setValue(battery_level)

            if self.vehicle.gps_0:
                fix_str = f"{self.vehicle.gps_0.fix_type}D Fix ({self.vehicle.gps_0.satellites_visible} uydu)"
                self.telemetry_values["GPS Fix:"].setText(fix_str)
            
            # Rota label'ını güncelle
            if heading is not None and hasattr(self, 'current_heading_label'):
                if mode == "MANUAL":
                    self.current_heading_label.setText("Mevcut Rota: MANUAL")
                else:
                    self.current_heading_label.setText(f"Mevcut Rota: {heading}°")

        except Exception as e:
            self.log_message_received.emit(f"Telemetri okuma hatası: {e}")

    def update_motor_simulation(self):
        """Thruster güçleri - gerçek bağlantıda Pixhawk'tan PWM değerleri alınır"""
        if not hasattr(self, 'thruster_labels'):
            return
        
        if self.is_connected and self.vehicle:
            # GERÇEK VERİ: Pixhawk'tan servo çıkışları (PWM 1000-2000)
            # self.vehicle.channels['1'] = Sol thruster PWM
            # SERVO_OUTPUT_RAW cache'den PWM değerlerini al
            try:
                # Cache'den PWM değerlerini al
                # SERVO1_FUNCTION=74 (Motor2/Sağ), SERVO2_FUNCTION=73 (Motor1/Sol)
                right_pwm = self.servo_output_cache.get('1')  # Kanal 1: Sağ thruster (SERVO_FUNCTION=74)
                left_pwm = self.servo_output_cache.get('2')   # Kanal 2: Sol thruster (SERVO_FUNCTION=73)
                
                # None kontrolü yap
                if left_pwm is not None and right_pwm is not None:
                    # Debug: Tüm PWM kanallarını kontrol et
                    all_channels_debug = []
                    for ch in range(1, 9):  # Kanal 1-8 arası kontrol et
                        pwm_val = self.servo_output_cache.get(str(ch))
                        if pwm_val is not None:
                            all_channels_debug.append(f"CH{ch}:{pwm_val}")
                    
                    debug_msg = f"Cache PWM: {', '.join(all_channels_debug)} | SERVO1_FUNC=74(Sağ), SERVO2_FUNC=73(Sol)"
                    self.log_message_received.emit(debug_msg)
                    
                    # Marine thruster PWM: 1500=neutral, 1000=tam geri, 2000=tam ileri
                    def calculate_thruster_power(pwm):
                        if pwm is None:
                            return 0, "NEUTRAL"
                        # Neutral noktası 1500μs, ±25μs tolerans
                        diff = pwm - 1500
                        power = abs(diff) / 5.0  # Her 5μs = %1 güç
                        power = max(0, min(100, power))
                        
                        # Geniş neutral bölgesi: 1475-1525μs arası NEUTRAL
                        if abs(diff) <= 25:
                            direction = "NEUTRAL"
                            power = 0  # Neutral'da güç 0 göster
                        elif diff > 25:
                            direction = "GERİ"
                        else:
                            direction = "İLERİ"
                        return power, direction
                    
                    left_power, left_dir = calculate_thruster_power(left_pwm)
                    right_power, right_dir = calculate_thruster_power(right_pwm)
                    
                    # Grafik için thruster verilerini topla
                    self.thruster_left_data.append(left_power)
                    self.thruster_right_data.append(right_power)
                    
                    # Sol motor ilk (UI sırası), Sağ motor ikinci
                    motor_data = [(left_power, left_dir, left_pwm, "Sol", "CH2(73)"),
                                  (right_power, right_dir, right_pwm, "Sağ", "CH1(74)")]
                    
                    for i, (power, direction, pwm_val, side, channel_info) in enumerate(motor_data):
                        # Renk: güç ve yöne göre
                        if direction == "NEUTRAL":
                            color = "gray"
                        elif power < 30:
                            color = "green"
                        elif power < 70:
                            color = "orange"
                        else:
                            color = "red"
                            
                        real_pwm = pwm_val if pwm_val else 0
                        # Kısa format: "Sol: 80% GERİ (1100μs)"
                        self.thruster_labels[i].setText(f"{side}: {power:.0f}% {direction} ({real_pwm}μs)")
                        self.thruster_labels[i].setStyleSheet(f"border: 1px solid {color}; padding: 5px; font-size: 10px; font-weight: bold; color: {color};")
                    return
                else:
                    # PWM değerleri henüz cache'de yok - basit fallback göster
                    for i in range(2):
                        side = "Sol" if i == 0 else "Sağ"
                        self.thruster_labels[i].setText(f"{side}: VERİ BEKLENİYOR")
                        self.thruster_labels[i].setStyleSheet("border: 1px solid orange; padding: 5px; font-size: 10px; font-weight: bold; color: orange;")
                    return
            except Exception as e:
                self.log_message_received.emit(f"Thruster cache okunamadı: {e}")
                # Hata durumunda da fallback göster
                for i in range(2):
                    side = "Sol" if i == 0 else "Sağ"
                    self.thruster_labels[i].setText(f"{side}: HATA")
                    self.thruster_labels[i].setStyleSheet("border: 1px solid red; padding: 5px; font-size: 10px; font-weight: bold; color: red;")
                return
        
        # SADECE GERÇEK VERİ - SİMÜLASYON YOK
        if not self.is_connected or not self.vehicle:
            # Bağlantı yoksa: grafik için 0 değeri ekle
            self.thruster_left_data.append(0)
            self.thruster_right_data.append(0)
            
            for i in range(2):
                side = "Sol" if i == 0 else "Sağ"
                self.thruster_labels[i].setText(f"{side}: BAĞLANTI YOK")
                self.thruster_labels[i].setStyleSheet("border: 1px solid gray; padding: 5px; font-size: 10px; font-weight: bold; color: gray;")
            return
            
        # Cache boşsa: grafik için 0 değeri ekle ama bekle
        if not self.servo_output_cache or '1' not in self.servo_output_cache or '2' not in self.servo_output_cache:
            # Grafik için 0 değeri ekle - grafik hemen başlasın
            self.thruster_left_data.append(0)
            self.thruster_right_data.append(0)
            
            for i in range(2):
                side = "Sol" if i == 0 else "Sağ"
                self.thruster_labels[i].setText(f"{side}: VERİ BEKLENİYOR")
                self.thruster_labels[i].setStyleSheet("border: 1px solid orange; padding: 5px; font-size: 10px; font-weight: bold; color: orange;")
            return


    def location_callback(self, vehicle, attr_name, value):
        # GPS fix_type ve uydu sayısını logla, haritayı her durumda güncelle
        if value and self.vehicle.heading is not None:
            import time
            current_time = time.time()
            gps = getattr(self.vehicle, 'gps_0', None)
            fix_type = getattr(gps, 'fix_type', 'N/A') if gps else 'N/A'
            satellites = getattr(gps, 'satellites_visible', 'N/A') if gps else 'N/A'
            if not hasattr(self, '_last_coord_log') or current_time - self._last_coord_log > 10:
                self._last_coord_log = current_time
                precision_info = (
                    f"Koordinat: {value.lat:.6f}, {value.lon:.6f}, Heading: {self.vehicle.heading}°, "
                    f"GPS Fix: {fix_type}, Uydu: {satellites}"
                )
                self.log_message_received.emit(f"📍 {precision_info}")
            # Fix tipi ne olursa olsun haritayı güncelle
            self.bridge.updateVehiclePosition.emit(value.lat, value.lon, self.vehicle.heading)

    def heading_callback(self, vehicle, attr_name, value):
        self.current_heading = value
    
    def get_target_heading_from_mission(self, current_heading):
        """Mission waypoint'lerinden target heading hesapla"""
        if len(self.waypoints) > 0 and self.vehicle and hasattr(self.vehicle, 'location'):
            # Bir sonraki waypoint'e doğru hedef açıyı hesapla
            current_loc = self.vehicle.location.global_relative_frame
            next_wp = self.waypoints[0]  # İlk waypoint'i hedef al
            
            # Basit bearing hesaplama
            dlat = next_wp["lat"] - current_loc.lat
            dlon = next_wp["lng"] - current_loc.lon
            target_heading = math.degrees(math.atan2(dlon, dlat)) % 360
            return target_heading
        else:
            # Waypoint yoksa hafif sapma simüle et
            return (current_heading + 15) % 360

    def update_attitude(self):
        """Attitude indicator'ı günceller."""
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
    
    def add_waypoint_to_list(self, lat, lng):
        """Haritadan gelen waypoint ekleme isteğini işler."""
        waypoint_num = len(self.waypoints) + 1
        self.waypoints.append({"lat": lat, "lng": lng, "num": waypoint_num})
        self.log_message_received.emit(f"Yeni Waypoint #{waypoint_num} eklendi: {lat:.6f}, {lng:.6f}")
        
        # Haritaya waypoint'i güvenli şekilde ilet
        self._emit_add_waypoint_safe(lat, lng)
    
    def remove_waypoint_from_list(self, index):
        """Haritadan gelen waypoint silme isteğini işler."""
        if 0 <= index < len(self.waypoints):
            removed_wp = self.waypoints.pop(index)
            self.log_message_received.emit(f"Waypoint #{removed_wp['num']} silindi")
            
            # Kalan waypoint'lerin numaralarını güncelle
            for i, wp in enumerate(self.waypoints):
                wp['num'] = i + 1

    @pyqtSlot(str)
    def update_log_safe(self, message):
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_display.append(f"[{timestamp}] {message}")

    @pyqtSlot(str)
    def update_status_safe(self, message):
        self.status_label.setText(f"  Durum: {message}")

    def set_vehicle_mode(self, mode):
        if not self.is_connected or not self.vehicle:
            self.status_message_received.emit("Mod değiştirmek için araç bağlantısı gerekli.")
            return
        
        self.status_message_received.emit(f"{mode} moduna geçiliyor...")
        threading.Thread(target=self._set_mode_thread, args=(mode,), daemon=True).start()

    def _set_mode_thread(self, mode):
        try:
            self.vehicle.mode = VehicleMode(mode)
            self.status_message_received.emit(f"Araç modu başarıyla {mode} olarak ayarlandı.")
            self.log_message_received.emit(f"Mod değiştirildi: {mode}")
        except Exception as e:
            self.status_message_received.emit(f"Mod değiştirme hatası: {e}")
            self.log_message_received.emit(f"HATA: Mod değiştirilemedi - {e}")

    def send_mission_to_vehicle(self):
        if not self.is_connected or not self.vehicle:
            self.status_message_received.emit("Rota göndermek için araç bağlantısı gerekli.")
            return
        if not self.waypoints:
            self.status_message_received.emit("Gönderilecek rota (waypoint) bulunmuyor.")
            return
        
        self.status_message_received.emit("Rota araca gönderiliyor...")
        threading.Thread(target=self._upload_mission_thread, daemon=True).start()

    def _upload_mission_thread(self):
        try:
            from dronekit import Command
            from pymavlink import mavutil
            
            cmds = self.vehicle.commands
            cmds.clear()

            for wp in self.waypoints:
                cmd = Command(
                    0, 0, 0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0,
                    wp["lat"], wp["lng"], 0
                )
                cmds.add(cmd)
            
            self.log_message_received.emit(f"{len(self.waypoints)} komut araca yükleniyor...")
            # Yükleme ve güvenli tekrar denemesi
            try:
                cmds.upload()
                self.status_message_received.emit("Rota başarıyla gönderildi!")
                self.log_message_received.emit("Misyon araca yüklendi.")
                return
            except Exception as e_first:
                self.log_message_received.emit(f"Rota yükleme ilk deneme hatası: {e_first}")
                # Kısa bekleme sonrası bir kez daha dene
                try:
                    time.sleep(1.0)
                    cmds.upload()
                    self.status_message_received.emit("Rota başarıyla gönderildi!")
                    self.log_message_received.emit("Misyon araca yüklendi (2. deneme).")
                    return
                except Exception as e_second:
                    # İkinci deneme de başarısız → durum ve log'a hata yaz
                    self.status_message_received.emit(f"Rota gönderme hatası: {e_second}")
                    self.log_message_received.emit(f"HATA: Rota gönderilemedi - {e_second}")
                    return
        except Exception as e:
            self.status_message_received.emit(f"Rota gönderme hatası: {e}")
            self.log_message_received.emit(f"HATA: Rota gönderilemedi - {e}")

    def read_mission_from_vehicle(self):
        """Aracın bellekindeki misyonu okur (Mission Planner'daki Read butonu işlevi)"""
        if not self.is_connected or not self.vehicle:
            self.status_message_received.emit("Rota okumak için araç bağlantısı gerekli.")
            return
        
        self.status_message_received.emit("Aracın bellekindeki rota okunuyor...")
        threading.Thread(target=self._read_mission_thread, daemon=True).start()

    def _read_mission_thread(self):
        """Aractan misyonu okuma thread'i"""
        try:
            # Önce mevcut misyonu temizle
            self.waypoints = []
            self.bridge.clearMap.emit()
            
            # Aracın komutlarını indir
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()  # İndirmenin tamamlanmasını bekle
            
            waypoint_count = 0
            # Her komutu kontrol et
            for i, cmd in enumerate(cmds):
                # Sadece waypoint komutlarını al (NAV_WAYPOINT)
                if cmd.command == 16:  # MAV_CMD_NAV_WAYPOINT
                    waypoint_count += 1
                    waypoint = {
                        "lat": cmd.x,
                        "lng": cmd.y, 
                        "alt": cmd.z,
                        "num": waypoint_count
                    }
                    self.waypoints.append(waypoint)
                    
                    # Haritaya waypoint ekle
                    self.bridge.addWaypoint.emit(cmd.x, cmd.y)
                    
                    self.log_message_received.emit(
                        f"Waypoint #{waypoint_count} okundu: {cmd.x:.6f}, {cmd.y:.6f}, {cmd.z:.1f}m"
                    )
            
            if waypoint_count > 0:
                self.status_message_received.emit(f"Rota başarıyla okundu! {waypoint_count} waypoint alındı.")
                self.log_message_received.emit(f"Toplam {waypoint_count} waypoint araçtan okundu.")
            else:
                self.status_message_received.emit("Araçta kayıtlı rota bulunamadı.")
                self.log_message_received.emit("Araçta herhangi bir waypoint bulunamadı.")
                
        except Exception as e:
            self.status_message_received.emit(f"Rota okuma hatası: {e}")
            self.log_message_received.emit(f"HATA: Rota okunamadı - {e}")
    
    def clear_mission(self):
        self.waypoints = []
        self._emit_clear_map_safe()
        self.log_message_received.emit("Rota ve harita temizlendi.")

    def on_map_load_finished(self, ok):
        # Harita sayfası yüklendikten sonra bekleyen işlemleri uygula
        self.map_ready = bool(ok)
        if not self.map_ready:
            return
        try:
            if self._pending_clear:
                self.bridge.clearMap.emit()
                self._pending_clear = False
            for lat, lon in self._pending_waypoints:
                self.bridge.addWaypoint.emit(lat, lon)
            if self._pending_waypoints:
                self.log_message_received.emit(f"Harita yüklendi, {len(self._pending_waypoints)} bekleyen waypoint çizildi")
        finally:
            self._pending_waypoints.clear()

    def _emit_add_waypoint_safe(self, lat, lon):
        # Harita hazırsa hemen gönder, değilse kuyruğa al
        if self.map_ready:
            self.bridge.addWaypoint.emit(lat, lon)
        else:
            self._pending_waypoints.append((lat, lon))

    def _emit_clear_map_safe(self):
        # Harita hazırsa hemen temizle, değilse bayrak koy
        if self.map_ready:
            self.bridge.clearMap.emit()
        else:
            self._pending_clear = True

    @pyqtSlot(str)
    def on_connection_type_changed(self, text):
        if text == "UDP (Kablosuz)":
            self.ip_input.setVisible(True)
            self.udp_port_input.setVisible(True)
        elif text == "TCP (WiFi)":
            self.ip_input.setVisible(True)
            self.udp_port_input.setVisible(False)
        else:
            self.ip_input.setVisible(False)
            self.udp_port_input.setVisible(False)

    # Rota Kontrol Fonksiyonları
    def read_current_heading(self):
        """Aracın mevcut rotasını okur"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota okumak için araç bağlantısı gerekli")
            return
        
        try:
            current_heading = getattr(self.vehicle, 'heading', None)
            if current_heading is not None:
                self.current_heading_label.setText(f"Mevcut Rota: {current_heading}°")
                self.target_heading_input.setText(str(int(current_heading)))
                self.log_message_received.emit(f"Mevcut rota okundu: {current_heading}°")
            else:
                self.log_message_received.emit("Rota bilgisi henüz mevcut değil")
        except Exception as e:
            self.log_message_received.emit(f"Rota okuma hatası: {e}")

    def send_heading_command(self):
        """Girilen hedef rotayı araca gönderir"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota göndermek için araç bağlantısı gerekli")
            return
        
        try:
            target_heading_text = self.target_heading_input.text().strip()
            if not target_heading_text:
                self.log_message_received.emit("Hedef rota değeri giriniz (0-359°)")
                return
            
            target_heading = float(target_heading_text)
            if target_heading < 0 or target_heading > 359:
                self.log_message_received.emit("Rota değeri 0-359° arasında olmalıdır")
                return
            
            # GUIDED modda hedef rota gönder
            if self.vehicle.mode.name != "GUIDED":
                self.log_message_received.emit("Rota göndermek için GUIDED moda geçiliyor...")
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(2)  # Mod değişimini bekle
            
            # Rota komutunu gönder - ArduPilot SUB için
            msg = self.vehicle.message_factory.set_position_target_global_int_encode(
                0,  # time_boot_ms
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                8,  # coordinate_frame (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                0b110111111000,  # type_mask (sadece yaw açısını kullan)
                0, 0, 0,  # lat, lon, alt (kullanılmıyor)
                0, 0, 0,  # vx, vy, vz (kullanılmıyor) 
                0, 0, math.radians(target_heading)  # afx, afy, yaw
            )
            self.vehicle.send_mavlink(msg)
            
            self.log_message_received.emit(f"Hedef rota gönderildi: {target_heading}°")
            
        except ValueError:
            self.log_message_received.emit("Geçersiz rota değeri. Sayı giriniz.")
        except Exception as e:
            self.log_message_received.emit(f"Rota gönderme hatası: {e}")

    def clear_heading_command(self):
        """Rota komutlarını temizler"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota temizlemek için araç bağlantısı gerekli")
            return
        
        try:
            # MANUAL moda geçerek otomatik rota kontrolünü durdur
            self.vehicle.mode = VehicleMode("MANUAL")
            self.current_heading_label.setText("Mevcut Rota: MANUAL")
            self.target_heading_input.clear()
            self.log_message_received.emit("Rota komutları temizlendi - MANUAL moda geçildi")
            
        except Exception as e:
            self.log_message_received.emit(f"Rota temizleme hatası: {e}")

    def update_graphs(self):
        """Grafikleri günceller - bağlantı yoksa da çiz"""
        if len(self.time_data) < 2:
            return
        
        try:
            # X ekseni için zaman verisi - son 30 saniye
            time_array = list(self.time_data)
            current_time = time_array[-1] if time_array else time.time()
            relative_times = [(t - current_time) for t in time_array]
            
            # 1. Hız grafiği - İki ayrı çizgi gösterimi
            self.speed_ax.clear()
            # Mavi çizgi: Gerçek hız (x-y uzunluklarını hizala)
            speed_vals = list(self.speed_data)
            n_speed = min(len(relative_times), len(speed_vals))
            if n_speed > 1:
                self.speed_ax.plot(relative_times[-n_speed:], speed_vals[-n_speed:], 'b-', linewidth=2, alpha=0.9, label='Gerçek Hız')
            # Kırmızı kesikli çizgi: Hız setpoint (ayrı hizalama)
            speed_sp_vals = list(self.speed_setpoint_data)
            n_speed_sp = min(len(relative_times), len(speed_sp_vals))
            if n_speed_sp > 1:
                self.speed_ax.plot(relative_times[-n_speed_sp:], speed_sp_vals[-n_speed_sp:], 'r--', linewidth=2, alpha=0.9, label='Setpoint')
            self.speed_ax.set_title('Hız: Mavi=Gerçek, Kırmızı=Setpoint', fontsize=8, pad=2)
            self.speed_ax.set_ylabel('m/s', fontsize=6)
            self.speed_ax.grid(True, alpha=0.2)
            self.speed_ax.tick_params(labelsize=5, pad=1)
            self.speed_ax.set_xticklabels([])  # X ekseni etiketlerini gizle
            self.speed_ax.set_xlim(-30, 0)  # Son 30 saniye
            # Y ekseni otomatik ölçekleme - hız değerlerine göre
            speed_values = list(self.speed_data) + list(self.speed_setpoint_data)
            if speed_values:
                valid_speeds = [v for v in speed_values if v is not None and v >= 0]
                if valid_speeds:
                    max_speed = max(valid_speeds)
                    self.speed_ax.set_ylim(0, max(max_speed + 1, 5))  # En az 5 m/s göster
                else:
                    self.speed_ax.set_ylim(0, 5)
            else:
                self.speed_ax.set_ylim(0, 5)
            self.speed_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.1)
            self.speed_canvas.draw()
            
            # 2. Heading grafiği - İki ayrı çizgi gösterimi
            self.heading_ax.clear()
            # Yeşil çizgi: Gerçek heading
            heading_vals = list(self.heading_data)
            n_head = min(len(relative_times), len(heading_vals))
            if n_head > 1:
                self.heading_ax.plot(relative_times[-n_head:], heading_vals[-n_head:], 'g-', linewidth=2, alpha=0.9, label='Gerçek Heading')
            # Kırmızı kesikli çizgi: Heading setpoint
            heading_sp_vals = list(self.heading_setpoint_data)
            n_head_sp = min(len(relative_times), len(heading_sp_vals))
            if n_head_sp > 1:
                self.heading_ax.plot(relative_times[-n_head_sp:], heading_sp_vals[-n_head_sp:], 'r--', linewidth=2, alpha=0.9, label='Setpoint')
            self.heading_ax.set_title('Heading: Yeşil=Gerçek, Kırmızı=Setpoint', fontsize=8, pad=2)
            self.heading_ax.set_ylabel('°', fontsize=6)
            self.heading_ax.grid(True, alpha=0.2)
            self.heading_ax.tick_params(labelsize=5, pad=1)
            self.heading_ax.set_xticklabels([])  # X ekseni etiketlerini gizle
            self.heading_ax.set_xlim(-30, 0)  # Son 30 saniye
            # Y ekseni akıllı ölçekleme - heading değerlerine göre
            heading_values = list(self.heading_data) + list(self.heading_setpoint_data)
            if heading_values:
                valid_headings = [v for v in heading_values if v is not None and v >= 0]
                if valid_headings:
                    min_heading = min(valid_headings)
                    max_heading = max(valid_headings)
                    # Eğer değerler 0 civarında değilse, o aralığı göster
                    if max_heading - min_heading > 180:  # 180°'den fazla fark varsa tam aralık göster
                        self.heading_ax.set_ylim(0, 360)
                    else:
                        # Dar aralık için optimize edilmiş görünüm
                        margin = max(20, (max_heading - min_heading) * 0.2)  # En az 20° margin
                        self.heading_ax.set_ylim(max(0, min_heading - margin), min(360, max_heading + margin))
                else:
                    self.heading_ax.set_ylim(0, 360)  # Varsayılan
            else:
                self.heading_ax.set_ylim(0, 360)
            self.heading_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.1)
            self.heading_canvas.draw()
            
            # 3. Thruster grafiği - İki ayrı çizgi gösterimi
            self.thruster_ax.clear()
            # Mavi çizgi: Sol thruster
            thr_left_vals = list(self.thruster_left_data)
            n_thr_left = min(len(relative_times), len(thr_left_vals))
            if n_thr_left > 1:
                self.thruster_ax.plot(relative_times[-n_thr_left:], thr_left_vals[-n_thr_left:], 'b-', linewidth=2, alpha=0.9, label='Sol Motor')
            # Kırmızı çizgi: Sağ thruster
            thr_right_vals = list(self.thruster_right_data)
            n_thr_right = min(len(relative_times), len(thr_right_vals))
            if n_thr_right > 1:
                self.thruster_ax.plot(relative_times[-n_thr_right:], thr_right_vals[-n_thr_right:], 'r-', linewidth=2, alpha=0.9, label='Sağ Motor')
            self.thruster_ax.set_title('Thruster: Mavi=Sol, Kırmızı=Sağ', fontsize=8, pad=2)
            self.thruster_ax.set_xlabel('Zaman (s)', fontsize=6)
            self.thruster_ax.set_ylabel('%', fontsize=6)
            self.thruster_ax.grid(True, alpha=0.2)
            self.thruster_ax.tick_params(labelsize=5, pad=1)
            self.thruster_ax.set_xlim(-30, 0)  # Son 30 saniye
            # Y ekseni otomatik ölçekleme - thruster değerlerine göre
            thruster_values = list(self.thruster_left_data) + list(self.thruster_right_data)
            if thruster_values:
                valid_thrusters = [v for v in thruster_values if v is not None and v >= 0]
                if valid_thrusters:
                    max_thruster = max(valid_thrusters)
                    self.thruster_ax.set_ylim(0, max(max_thruster + 10, 100))  # En az 100% göster
                else:
                    self.thruster_ax.set_ylim(0, 100)
            else:
                self.thruster_ax.set_ylim(0, 100)
            self.thruster_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.2)
            self.thruster_canvas.draw()
            
        except Exception as e:
            self.log_message_received.emit(f"Grafik güncelleme hatası: {e}")

    def clear_graph_data(self):
        """Grafik verilerini temizler"""
        self.speed_data.clear()
        self.speed_setpoint_data.clear()
        self.heading_data.clear()
        self.heading_setpoint_data.clear()
        self.thruster_left_data.clear()
        self.thruster_right_data.clear()
        self.time_data.clear()
        
        # Grafikleri temizle
        if hasattr(self, 'speed_ax'):
            self.speed_ax.clear()
            self.speed_ax.set_title('Hız', fontsize=8, pad=2)
            self.speed_ax.set_ylabel('m/s', fontsize=6)
            self.speed_ax.grid(True, alpha=0.2)
            self.speed_ax.tick_params(labelsize=5, pad=1)
            self.speed_ax.set_xticklabels([])
            self.speed_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.1)
            self.speed_canvas.draw()
            
        if hasattr(self, 'heading_ax'):
            self.heading_ax.clear()
            self.heading_ax.set_title('Heading', fontsize=8, pad=2)
            self.heading_ax.set_ylabel('°', fontsize=6)
            self.heading_ax.grid(True, alpha=0.2)
            self.heading_ax.tick_params(labelsize=5, pad=1)
            self.heading_ax.set_xticklabels([])
            self.heading_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.1)
            self.heading_canvas.draw()
            
        if hasattr(self, 'thruster_ax'):
            self.thruster_ax.clear()
            self.thruster_ax.set_title('Thruster', fontsize=8, pad=2)
            self.thruster_ax.set_xlabel('Zaman (s)', fontsize=6)
            self.thruster_ax.set_ylabel('%', fontsize=6)
            self.thruster_ax.grid(True, alpha=0.2)
            self.thruster_ax.tick_params(labelsize=5, pad=1)
            self.thruster_figure.subplots_adjust(left=0.15, right=0.95, top=0.8, bottom=0.2)
            self.thruster_canvas.draw()

    def start_telemetry_logging(self):
        """Telemetri verisini CSV'ye 1 Hz ile kaydetmeye başlar."""
        try:
            if self._telemetry_csv_file is not None:
                return
            logs_dir = os.path.join(os.path.dirname(__file__), 'logs')
            os.makedirs(logs_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            csv_path = os.path.join(logs_dir, f'telemetry_{ts}.csv')
            self._telemetry_csv_file = open(csv_path, 'w', newline='')
            import io
            if isinstance(self._telemetry_csv_file, io.TextIOBase):
                pass
            self._telemetry_csv_writer = csv.writer(self._telemetry_csv_file)
            self._telemetry_csv_writer.writerow([
                'timestamp', 'lat', 'lon', 'groundspeed_mps',
                'roll_deg', 'pitch_deg', 'heading_deg',
                'speed_setpoint_mps', 'heading_setpoint_deg'
            ])
            self._telemetry_log_timer.start(1000)
            self.log_message_received.emit(f"Telemetri CSV kaydı başladı: {csv_path}")
        except Exception as e:
            self.log_message_received.emit(f"Telemetri kaydı başlatılamadı: {e}")

    def stop_telemetry_logging(self):
        """Telemetri CSV kaydını durdurur."""
        try:
            self._telemetry_log_timer.stop()
            if self._telemetry_csv_file:
                try:
                    self._telemetry_csv_file.flush()
                except Exception:
                    pass
                self._telemetry_csv_file.close()
            self._telemetry_csv_file = None
            self._telemetry_csv_writer = None
            self.log_message_received.emit("Telemetri CSV kaydı durduruldu")
        except Exception as e:
            self.log_message_received.emit(f"Telemetri kaydı durdurulamadı: {e}")

    def _handle_mode_change(self, old_mode, new_mode):
        """Araç modu değişikliğini işler - otonom kayıt sistemi için"""
        try:
            # Otonom modlara geçiş kontrolü
            autonomous_modes = ["AUTO", "GUIDED", "RTL"]
            
            # Manuel modlara geçiş kontrolü
            manual_modes = ["MANUAL", "STABILIZE", "ACRO", "ALT_HOLD"]
            
            # Otonom moda geçiş
            if (old_mode not in autonomous_modes and new_mode in autonomous_modes):
                self.log_message_received.emit(f"🚀 OTONOM MODA GEÇİŞ: {old_mode} → {new_mode}")
                self.autonomous_recording.start_recording()
                
            # Manuel moda geçiş
            elif (old_mode in autonomous_modes and new_mode in manual_modes):
                self.log_message_received.emit(f"⏹️ MANUEL MODA GEÇİŞ: {old_mode} → {new_mode}")
                self.autonomous_recording.stop_recording()
                
        except Exception as e:
            self.log_message_received.emit(f"❌ Mod değişikliği işleme hatası: {e}")

    def _log_telemetry_row(self):
        """1 Hz telemetri satırı yazar."""
        if not self.is_connected or not self.vehicle or self._telemetry_csv_writer is None:
            return
        try:
            ts = datetime.utcnow().isoformat()
            lat = None
            lon = None
            if hasattr(self.vehicle, 'location') and self.vehicle.location and hasattr(self.vehicle.location, 'global_frame'):
                gf = self.vehicle.location.global_frame
                lat = getattr(gf, 'lat', None)
                lon = getattr(gf, 'lon', None)
            groundspeed = getattr(self.vehicle, 'groundspeed', None)
            heading = getattr(self.vehicle, 'heading', None)
            roll_deg = None
            pitch_deg = None
            if hasattr(self.vehicle, 'attitude') and self.vehicle.attitude is not None:
                try:
                    roll_deg = math.degrees(self.vehicle.attitude.roll)
                    pitch_deg = math.degrees(self.vehicle.attitude.pitch)
                except Exception:
                    pass
            speed_setpoint = None
            heading_setpoint = None
            mode_name = getattr(getattr(self.vehicle, 'mode', None), 'name', None)
            if mode_name in ["AUTO", "GUIDED", "RTL"]:
                wpnav_speed = self.vehicle.parameters.get('WPNAV_SPEED', None)
                wp_speed = self.vehicle.parameters.get('WP_SPEED', None)
                speed_param = wpnav_speed if wpnav_speed is not None else wp_speed
                if speed_param is not None:
                    speed_setpoint = speed_param / 100.0
                if len(self.waypoints) > 0 and isinstance(heading, (int, float)):
                    heading_setpoint = self.get_target_heading_from_mission(heading)
            self._telemetry_csv_writer.writerow([
                ts,
                f"{lat:.7f}" if isinstance(lat, float) else "",
                f"{lon:.7f}" if isinstance(lon, float) else "",
                f"{groundspeed:.3f}" if isinstance(groundspeed, float) else "",
                f"{roll_deg:.2f}" if isinstance(roll_deg, float) else "",
                f"{pitch_deg:.2f}" if isinstance(pitch_deg, float) else "",
                f"{heading:.1f}" if isinstance(heading, (int, float)) else "",
                f"{speed_setpoint:.3f}" if isinstance(speed_setpoint, float) else "",
                f"{heading_setpoint:.1f}" if isinstance(heading_setpoint, (int, float)) else ""
            ])
            try:
                self._telemetry_csv_file.flush()
            except Exception:
                pass
        except Exception as e:
            self.log_message_received.emit(f"Telemetri satırı yazılamadı: {e}")

    def load_predefined_mission(self):
        """Önceden tanımlanmış görev koordinatlarını yükler ve haritaya gönderir"""
        try:
            # Başka bir göreve geçerken Görev 2'yi kapat
            self.terminate_gorev2()
            
            # Mevcut waypoint'leri temizle
            self.waypoints.clear()
            self.bridge.clearMap.emit()
            
            # Önceden tanımlanmış koordinatları waypoint listesine ekle
            for i, (lat, lon) in enumerate(self.mission_coordinates):
                waypoint_num = i + 1
                self.waypoints.append({"lat": lat, "lng": lon, "num": waypoint_num})
                self.bridge.addWaypoint.emit(lat, lon)
            
            # Koordinatları haritaya gönder ve log'a yaz
            self.log_message_received.emit(f"Görev yüklendi: {len(self.mission_coordinates)} waypoint eklendi")
            for i, (lat, lon) in enumerate(self.mission_coordinates):
                self.log_message_received.emit(f"Waypoint {i+1}: {lat:.6f}, {lon:.6f}")
            
            # Eğer araç bağlıysa, otomatik olarak görevi araca gönder
            if self.is_connected and self.vehicle:
                self.send_mission_to_vehicle()
                
        except Exception as e:
            self.log_message_received.emit(f"Görev yükleme hatası: {str(e)}")
            QMessageBox.warning(self, "Hata", f"Görev yüklenirken hata oluştu: {str(e)}")

if __name__ == '__main__':
    # qputenv("QTWEBENGINE_REMOTE_DEBUGGING", "9223") # Debug için
    app = QApplication(sys.argv)
    gcs_app = GCSApp()
    gcs_app.show()
    sys.exit(app.exec_())