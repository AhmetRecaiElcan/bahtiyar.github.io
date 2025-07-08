import sys
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QPolygonF, QFont
from PyQt5.QtCore import Qt, QPointF
import math

class AttitudeIndicator(QWidget):
    """
    Aracın yatış (roll) ve yunuslama (pitch) açılarını gösteren
    bir yapay ufuk (attitude indicator) bileşeni.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self._pitch = 0.0
        self._roll = 0.0
        
        self.setMinimumSize(120, 120)

    def set_attitude(self, pitch_rad, roll_rad):
        """Açıları radyan cinsinden ayarlar ve bileşeni yeniden çizer."""
        # Radyanı dereceye çevir
        self._pitch = math.degrees(pitch_rad)
        self._roll = math.degrees(roll_rad)
        self.update() # repaint() çağrısı

    def paintEvent(self, event):
        """Bileşenin çizim mantığı."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        side = min(width, height)
        
        # Orijini merkeze taşı
        painter.translate(width / 2, height / 2)
        
        # Dış çerçeve dairesi çiz
        self._draw_outer_circle(painter, side)
        
        # Gökyüzü ve yer arka planını çiz (daire içinde)
        self._draw_sky_ground(painter, side)
        
        # Statik referans işaretlerini çiz
        self._draw_fixed_references(painter, side)
    
    def _draw_outer_circle(self, painter, side):
        """Dış çerçeve dairesini çizer."""
        radius = side / 2 - 5
        painter.setPen(QPen(Qt.black, 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(int(-radius), int(-radius), int(radius * 2), int(radius * 2))

    def _draw_sky_ground(self, painter, side):
        """Aracın açılarına göre hareket eden gökyüzü, yer ve pitch merdivenini çizer."""
        painter.save()
        
        # Daire içinde çizim yapmak için clipping ayarla
        radius = side / 2 - 5
        from PyQt5.QtGui import QPainterPath
        clip_path = QPainterPath()
        clip_path.addEllipse(int(-radius), int(-radius), int(radius * 2), int(radius * 2))
        painter.setClipPath(clip_path)
        
        # Roll açısına göre döndür
        painter.rotate(-self._roll)
        
        # Pitch açısına göre dikey olarak kaydır
        # Bir derece pitch, belirli bir piksel kaymasına karşılık gelir.
        pixels_per_degree = side / 180.0 
        painter.translate(0, self._pitch * pixels_per_degree)

        # Yer (kahverengi)
        ground_color = QColor(139, 69, 19)
        painter.setBrush(QBrush(ground_color))
        painter.setPen(Qt.NoPen)
        painter.drawRect(int(-side), 0, int(side * 2), int(side))

        # Gökyüzü (mavi)
        sky_color = QColor(135, 206, 235)
        painter.setBrush(QBrush(sky_color))
        painter.drawRect(int(-side), int(-side), int(side * 2), int(side))
        
        # Pitch merdiveni ve ufuk çizgisi
        painter.setPen(QPen(Qt.white, 2))
        
        # Ufuk çizgisi
        painter.drawLine(int(-side), 0, int(side), 0)
        
        # Pitch çizgileri (her 10 derecede bir)
        for angle in range(-90, 91, 10):
            if angle == 0: continue
            y_pos = -angle * pixels_per_degree
            line_length = side / 8 if angle % 30 == 0 else side / 16
            painter.drawLine(int(-line_length), int(y_pos), int(line_length), int(y_pos))

        painter.restore()

    def _draw_fixed_references(self, painter, side):
        """Ekranın ortasında sabit duran uçak sembolünü ve roll göstergesini çizer."""
        # Roll göstergesi (üstteki yay)
        painter.setPen(QPen(Qt.white, 2))
        painter.drawArc(int(-side/3), int(-side/3), int(side*2/3), int(side*2/3), 30*16, 120*16)
        
        # Roll işaretçisi
        painter.save()
        painter.rotate(-self._roll)
        pointer = QPolygonF([QPointF(0, -side/3), QPointF(-5, -side/3+10), QPointF(5, -side/3+10)])
        painter.setBrush(Qt.white)
        painter.drawPolygon(pointer)
        painter.restore()

        # Ortadaki sabit uçak sembolü
        pen = QPen(Qt.yellow, 3)
        painter.setPen(pen)
        
        # Sol kanat
        painter.drawLine(int(-side / 4), 0, int(-side / 8), 0)
        # Sağ kanat
        painter.drawLine(int(side / 8), 0, int(side / 4), 0)
        # Merkez nokta
        painter.drawPoint(0, 0)
        # Kuyruk
        painter.drawLine(0, 5, 0, 15)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QWidget()
    layout = QVBoxLayout(window)
    
    indicator = AttitudeIndicator()
    layout.addWidget(indicator)
    
    # Test için slider'lar eklenebilir veya timer ile değiştirilebilir
    indicator.set_attitude(math.radians(15), math.radians(-30)) # 15 derece pitch, -30 derece roll
    
    window.resize(300, 300)
    window.setWindowTitle("Attitude Indicator Test")
    window.show()
    sys.exit(app.exec_())