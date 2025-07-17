#!/usr/bin/env python3
"""
YKİ Gelişmiş Telemetri Görselleştirme Sistemi
Bu modül, eksik telemetri verilerini (hız setpoint, heading setpoint, thruster kuvvetleri)
profesyonel bir şekilde görselleştirir.
"""

import sys
import math
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QFrame, QGridLayout, QProgressBar, QGroupBox)
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QPolygonF, QFont
from PyQt5.QtCore import Qt, QPointF, QTimer, pyqtSignal, QRect
import numpy as np

class CircularGauge(QWidget):
    """Dairesel gösterge widget'ı - hız, heading setpoint vb. için"""
    
    def __init__(self, title="", min_val=0, max_val=100, unit="", parent=None):
        super().__init__(parent)
        self.title = title
        self.min_val = min_val
        self.max_val = max_val
        self.unit = unit
        self.current_value = 0
        self.target_value = 0
        self.setMinimumSize(150, 150)
        
    def set_values(self, current, target=None):
        """Mevcut ve hedef değerleri ayarla"""
        self.current_value = max(self.min_val, min(self.max_val, current))
        if target is not None:
            self.target_value = max(self.min_val, min(self.max_val, target))
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        side = min(width, height)
        
        # Merkeze taşı
        painter.translate(width / 2, height / 2)
        
        # Dış çerçeve
        radius = side / 2 - 10
        painter.setPen(QPen(QColor(60, 63, 65), 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(int(-radius), int(-radius), int(radius * 2), int(radius * 2))
        
        # Arka plan yay
        painter.setPen(QPen(QColor(200, 200, 200), 8))
        start_angle = 225 * 16  # Qt açı sistemi (16ths of a degree)
        span_angle = 270 * 16
        painter.drawArc(int(-radius + 15), int(-radius + 15), 
                       int((radius - 15) * 2), int((radius - 15) * 2), 
                       start_angle, span_angle)
        
        # Mevcut değer yayı
        current_percentage = (self.current_value - self.min_val) / (self.max_val - self.min_val)
        current_span = int(270 * 16 * current_percentage)
        painter.setPen(QPen(QColor(46, 204, 113), 8))  # Yeşil
        painter.drawArc(int(-radius + 15), int(-radius + 15), 
                       int((radius - 15) * 2), int((radius - 15) * 2), 
                       start_angle, current_span)
        
        # Hedef değer işaretçisi (eğer varsa)
        if hasattr(self, 'target_value') and self.target_value != self.current_value:
            target_percentage = (self.target_value - self.min_val) / (self.max_val - self.min_val)
            target_angle = math.radians(225 + 270 * target_percentage)
            
            # Hedef nokta
            target_x = (radius - 5) * math.cos(target_angle)
            target_y = (radius - 5) * math.sin(target_angle)
            
            painter.setPen(QPen(QColor(231, 76, 60), 4))  # Kırmızı
            painter.setBrush(QBrush(QColor(231, 76, 60)))
            painter.drawEllipse(int(target_x - 5), int(target_y - 5), 10, 10)
        
        # Merkez değer metni
        painter.setPen(QPen(Qt.black))
        painter.setFont(QFont('Arial', 16, QFont.Bold))
        text = f"{self.current_value:.1f}"
        painter.drawText(QRect(-50, -10, 100, 20), Qt.AlignCenter, text)
        
        # Birim
        painter.setFont(QFont('Arial', 10))
        painter.drawText(QRect(-50, 5, 100, 20), Qt.AlignCenter, self.unit)
        
        # Başlık
        painter.setFont(QFont('Arial', 12, QFont.Bold))
        painter.drawText(QRect(-75, int(-radius + 5), 150, 20), Qt.AlignCenter, self.title)

class ThrusterIndicator(QWidget):
    """Thruster kuvvet göstergesi - çoklu thruster için"""
    
    def __init__(self, thruster_count=4, parent=None):
        super().__init__(parent)
        self.thruster_count = thruster_count
        self.thruster_forces = [0.0] * thruster_count  # -100 ile +100 arası
        self.setMinimumSize(200, 150)
        
    def set_thruster_forces(self, forces):
        """Thruster kuvvetlerini ayarla (liste veya tuple)"""
        for i, force in enumerate(forces[:self.thruster_count]):
            self.thruster_forces[i] = max(-100, min(100, force))
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        
        # Başlık
        painter.setFont(QFont('Arial', 12, QFont.Bold))
        painter.drawText(10, 20, "Thruster Kuvvetleri")
        
        # Her thruster için çubuk çiz
        bar_width = (width - 40) / self.thruster_count
        bar_height = height - 50
        
        for i in range(self.thruster_count):
            x = 20 + i * bar_width
            
            # Arka plan çubuğu
            painter.fillRect(int(x), 30, int(bar_width - 5), int(bar_height), 
                           QColor(240, 240, 240))
            
            # Orta çizgi (sıfır)
            mid_y = 30 + bar_height / 2
            painter.setPen(QPen(Qt.black, 1))
            painter.drawLine(int(x), int(mid_y), int(x + bar_width - 5), int(mid_y))
            
            # Kuvvet çubuğu
            force = self.thruster_forces[i]
            if force > 0:  # İleri
                fill_height = (force / 100) * (bar_height / 2)
                fill_rect = QRect(int(x + 2), int(mid_y - fill_height), 
                                int(bar_width - 9), int(fill_height))
                painter.fillRect(fill_rect, QColor(46, 204, 113))  # Yeşil
            elif force < 0:  # Geri
                fill_height = (-force / 100) * (bar_height / 2)
                fill_rect = QRect(int(x + 2), int(mid_y), 
                                int(bar_width - 9), int(fill_height))
                painter.fillRect(fill_rect, QColor(231, 76, 60))  # Kırmızı
            
            # Thruster numarası ve değer
            painter.setFont(QFont('Arial', 10))
            painter.drawText(int(x), int(30 + bar_height + 15), f"T{i+1}")
            painter.drawText(int(x), int(30 + bar_height + 30), f"{force:.0f}%")

class CompassRose(QWidget):
    """Gelişmiş pusula - mevcut heading ve setpoint'i gösterir"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_heading = 0
        self.target_heading = 0
        self.setMinimumSize(120, 120)
        
    def set_headings(self, current, target=None):
        """Mevcut ve hedef heading'leri ayarla"""
        self.current_heading = current % 360
        if target is not None:
            self.target_heading = target % 360
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        side = min(width, height)
        radius = side / 2 - 10
        
        # Merkeze taşı
        painter.translate(width / 2, height / 2)
        
        # Pusula arka plan
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(QBrush(QColor(250, 250, 250)))
        painter.drawEllipse(int(-radius), int(-radius), int(radius * 2), int(radius * 2))
        
        # Ana yönler
        directions = ['K', 'D', 'G', 'B']
        angles = [0, 90, 180, 270]
        
        painter.setFont(QFont('Arial', 12, QFont.Bold))
        for direction, angle in zip(directions, angles):
            painter.save()
            painter.rotate(angle)
            painter.drawText(QRect(-10, int(-radius + 5), 20, 20), Qt.AlignCenter, direction)
            painter.restore()
        
        # Derece işaretleri
        painter.setPen(QPen(Qt.black, 1))
        for i in range(0, 360, 30):
            painter.save()
            painter.rotate(i)
            if i % 90 == 0:
                painter.drawLine(0, int(-radius + 10), 0, int(-radius + 20))
            else:
                painter.drawLine(0, int(-radius + 15), 0, int(-radius + 20))
            painter.restore()
        
        # Hedef heading ok (kırmızı)
        if hasattr(self, 'target_heading'):
            painter.save()
            painter.rotate(self.target_heading)
            painter.setPen(QPen(QColor(231, 76, 60), 3))
            painter.drawLine(0, int(-radius + 25), 0, -5)
            # Ok ucu
            arrow = QPolygonF([QPointF(0, int(-radius + 25)), 
                              QPointF(-5, int(-radius + 35)), 
                              QPointF(5, int(-radius + 35))])
            painter.setBrush(QBrush(QColor(231, 76, 60)))
            painter.drawPolygon(arrow)
            painter.restore()
        
        # Mevcut heading ok (mavi)
        painter.save()
        painter.rotate(self.current_heading)
        painter.setPen(QPen(QColor(52, 152, 219), 4))
        painter.drawLine(0, int(-radius + 30), 0, 0)
        # Ok ucu
        arrow = QPolygonF([QPointF(0, int(-radius + 30)), 
                          QPointF(-6, int(-radius + 42)), 
                          QPointF(6, int(-radius + 42))])
        painter.setBrush(QBrush(QColor(52, 152, 219)))
        painter.drawPolygon(arrow)
        painter.restore()
        
        # Merkez nokta
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(QBrush(Qt.white))
        painter.drawEllipse(-3, -3, 6, 6)
        
        # Değer metni
        painter.setFont(QFont('Arial', 10))
        painter.drawText(QRect(-30, 15, 60, 15), Qt.AlignCenter, f"{self.current_heading:.0f}°")

class AdvancedTelemetryPanel(QWidget):
    """Gelişmiş telemetri paneli - tüm widget'ları bir araya getirir"""
    
    telemetry_updated = pyqtSignal(dict)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUI()
        self.setupTimer()
        
    def setupUI(self):
        layout = QVBoxLayout(self)
        
        # Üst sıra - Ana göstergeler
        top_layout = QHBoxLayout()
        
        # Hız göstergesi
        self.speed_gauge = CircularGauge("Hız", 0, 20, "m/s")
        top_layout.addWidget(self.speed_gauge)
        
        # Pusula
        self.compass = CompassRose()
        top_layout.addWidget(self.compass)
        
        # RPM/Motor göstergesi (eğer varsa)
        self.rpm_gauge = CircularGauge("Motor RPM", 0, 3000, "rpm")
        top_layout.addWidget(self.rpm_gauge)
        
        layout.addLayout(top_layout)
        
        # Orta sıra - Setpoint'ler
        middle_layout = QHBoxLayout()
        
        # Hız setpoint göstergesi
        speed_setpoint_group = QGroupBox("Hız Kontrol")
        speed_setpoint_layout = QVBoxLayout(speed_setpoint_group)
        
        self.speed_setpoint_gauge = CircularGauge("Hız Setpoint", 0, 20, "m/s")
        speed_setpoint_layout.addWidget(self.speed_setpoint_gauge)
        
        # Hız hatası
        self.speed_error_label = QLabel("Hata: 0.0 m/s")
        self.speed_error_label.setAlignment(Qt.AlignCenter)
        speed_setpoint_layout.addWidget(self.speed_error_label)
        
        middle_layout.addWidget(speed_setpoint_group)
        
        # Heading setpoint göstergesi
        heading_setpoint_group = QGroupBox("Yön Kontrol")
        heading_setpoint_layout = QVBoxLayout(heading_setpoint_group)
        
        self.heading_setpoint_gauge = CircularGauge("Heading SP", 0, 360, "°")
        heading_setpoint_layout.addWidget(self.heading_setpoint_gauge)
        
        # Heading hatası
        self.heading_error_label = QLabel("Hata: 0.0°")
        self.heading_error_label.setAlignment(Qt.AlignCenter)
        heading_setpoint_layout.addWidget(self.heading_error_label)
        
        middle_layout.addWidget(heading_setpoint_group)
        
        layout.addLayout(middle_layout)
        
        # Alt sıra - Thruster ve sistem durumu
        bottom_layout = QVBoxLayout()
        
        # Thruster göstergesi
        thruster_group = QGroupBox("Thruster Sistemleri")
        thruster_layout = QVBoxLayout(thruster_group)
        
        self.thruster_indicator = ThrusterIndicator(4)
        thruster_layout.addWidget(self.thruster_indicator)
        
        # Thruster durumları
        thruster_status_layout = QHBoxLayout()
        self.thruster_status_labels = []
        for i in range(4):
            label = QLabel(f"T{i+1}: OK")
            label.setStyleSheet("color: green; font-weight: bold;")
            thruster_status_layout.addWidget(label)
            self.thruster_status_labels.append(label)
        
        thruster_layout.addLayout(thruster_status_layout)
        bottom_layout.addWidget(thruster_group)
        
        # Sistem performans metrikleri
        performance_group = QGroupBox("Sistem Performansı")
        performance_layout = QGridLayout(performance_group)
        
        # CPU/GPU kullanımı
        performance_layout.addWidget(QLabel("CPU:"), 0, 0)
        self.cpu_progress = QProgressBar()
        performance_layout.addWidget(self.cpu_progress, 0, 1)
        
        # Telemetri frekansı
        performance_layout.addWidget(QLabel("Telemetri Hz:"), 1, 0)
        self.telemetry_freq_label = QLabel("0 Hz")
        performance_layout.addWidget(self.telemetry_freq_label, 1, 1)
        
        # Latency
        performance_layout.addWidget(QLabel("Latency:"), 2, 0)
        self.latency_label = QLabel("0 ms")
        performance_layout.addWidget(self.latency_label, 2, 1)
        
        bottom_layout.addWidget(performance_group)
        layout.addLayout(bottom_layout)
        
        self.setMaximumHeight(600)
    
    def setupTimer(self):
        """Performans metrikleri için timer"""
        self.perf_timer = QTimer()
        self.perf_timer.timeout.connect(self.update_performance_metrics)
        self.perf_timer.start(1000)  # Her saniye
        
        self.telemetry_count = 0
        self.last_telemetry_time = 0
    
    def update_telemetry(self, telemetry_data):
        """Ana telemetri güncelleme fonksiyonu"""
        # Hız
        current_speed = telemetry_data.get('speed', 0)
        speed_setpoint = telemetry_data.get('speed_setpoint', current_speed)
        self.speed_gauge.set_values(current_speed, speed_setpoint)
        self.speed_setpoint_gauge.set_values(speed_setpoint, current_speed)
        
        speed_error = speed_setpoint - current_speed
        self.speed_error_label.setText(f"Hata: {speed_error:.2f} m/s")
        self.speed_error_label.setStyleSheet(
            f"color: {'red' if abs(speed_error) > 0.5 else 'green'}; font-weight: bold;"
        )
        
        # Heading
        current_heading = telemetry_data.get('heading', 0)
        heading_setpoint = telemetry_data.get('heading_setpoint', current_heading)
        self.compass.set_headings(current_heading, heading_setpoint)
        self.heading_setpoint_gauge.set_values(heading_setpoint, current_heading)
        
        # Heading hatası (en kısa açı farkı)
        heading_error = self._calculate_heading_error(current_heading, heading_setpoint)
        self.heading_error_label.setText(f"Hata: {heading_error:.1f}°")
        self.heading_error_label.setStyleSheet(
            f"color: {'red' if abs(heading_error) > 10 else 'green'}; font-weight: bold;"
        )
        
        # Motor RPM
        rpm = telemetry_data.get('rpm', 0)
        self.rpm_gauge.set_values(rpm)
        
        # Thruster kuvvetleri
        thruster_forces = telemetry_data.get('thruster_forces', [0, 0, 0, 0])
        self.thruster_indicator.set_thruster_forces(thruster_forces)
        
        # Thruster durumları
        thruster_status = telemetry_data.get('thruster_status', ['OK'] * 4)
        for i, (label, status) in enumerate(zip(self.thruster_status_labels, thruster_status)):
            label.setText(f"T{i+1}: {status}")
            color = "green" if status == "OK" else "red"
            label.setStyleSheet(f"color: {color}; font-weight: bold;")
        
        # Telemetri sayacını artır
        self.telemetry_count += 1
        
        # Sinyal gönder
        self.telemetry_updated.emit(telemetry_data)
    
    def _calculate_heading_error(self, current, target):
        """En kısa heading farkını hesapla"""
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error
    
    def update_performance_metrics(self):
        """Sistem performans metriklerini güncelle"""
        import time
        import psutil
        
        # CPU kullanımı
        cpu_percent = psutil.cpu_percent()
        self.cpu_progress.setValue(int(cpu_percent))
        
        # Telemetri frekansı
        current_time = time.time()
        if hasattr(self, 'last_perf_time'):
            time_diff = current_time - self.last_perf_time
            if time_diff > 0:
                freq = self.telemetry_count / time_diff
                self.telemetry_freq_label.setText(f"{freq:.1f} Hz")
        
        self.last_perf_time = current_time
        self.telemetry_count = 0

# Test fonksiyonu
if __name__ == "__main__":
    from PyQt5.QtWidgets import QApplication
    import random
    
    app = QApplication(sys.argv)
    
    panel = AdvancedTelemetryPanel()
    panel.show()
    
    # Test verisi gönder
    def send_test_data():
        test_data = {
            'speed': random.uniform(0, 15),
            'speed_setpoint': random.uniform(8, 12),
            'heading': random.uniform(0, 360),
            'heading_setpoint': random.uniform(0, 360),
            'rpm': random.uniform(1000, 2500),
            'thruster_forces': [
                random.uniform(-80, 80),
                random.uniform(-80, 80),
                random.uniform(-80, 80),
                random.uniform(-80, 80)
            ],
            'thruster_status': ['OK', 'OK', 'FAULT' if random.random() < 0.1 else 'OK', 'OK']
        }
        panel.update_telemetry(test_data)
    
    # Test timer
    test_timer = QTimer()
    test_timer.timeout.connect(send_test_data)
    test_timer.start(500)  # 0.5 saniyede bir
    
    sys.exit(app.exec_()) 