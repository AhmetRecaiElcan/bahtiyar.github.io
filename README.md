# İDA Yer Kontrol İstasyonu (İDA Ground Control Station)

İnsansız Deniz Aracı (İDA) için PyQt5 tabanlı yer kontrol istasyonu uygulaması.

## 🚢 Özellikler

### 📡 Telemetri ve Bağlantı
- **Gerçek zamanlı telemetri**: Hız, yükseklik, heading, GPS durumu, mod, batarya
- **USB/Serial bağlantı**: Pixhawk uçuş kontrolcüsü ile DroneKit üzerinden haberleşme
- **Otomatik port tarama**: Mevcut COM portlarını otomatik tespit
- **Bağlantı durumu göstergesi**: Anlık bağlantı durumu ve sistem mesajları

### 🗺️ Harita ve Navigasyon
- **Interaktif harita**: OpenStreetMap tabanlı gerçek zamanlı harita
- **Araç takibi**: Gemi ikonu ile gerçek zamanlı konum ve yön gösterimi
- **Waypoint yönetimi**: 
  - Çift tıklama ile waypoint ekleme
  - Waypoint'e çift tıklama ile silme
  - Otomatik rota çizimi
  - Waypoint numaralandırması

### 🎛️ Kontrol Paneli
- **Uçuş modları**: STABILIZE, AUTO, GUIDED mod değiştirme
- **Görev yönetimi**: Rota gönderme ve temizleme
- **Attitude Indicator**: Gerçek zamanlı roll/pitch göstergesi
  - Yapay ufuk (gökyüzü/yer ayrımı)
  - Pitch merdiveni ve ufuk çizgisi
  - Roll göstergesi yayı
  - Daire çerçeveli kompakt tasarım

### 📊 Sistem İzleme
- **Canlı log sistemi**: Tüm sistem olayları zaman damgalı
- **Batarya göstergesi**: Görsel progress bar ile batarya seviyesi
- **Durum paneli**: Anlık sistem durumu bilgileri

## 🛠️ Kurulum

### Gereksinimler
```bash
pip install -r requirements.txt
```

### Bağımlılıklar
- PyQt5
- PyQtWebEngine  
- dronekit
- pyserial

### Çalıştırma
```bash
python gcs_app.py
```

## 📁 Proje Yapısı

```
ida_ground_control_station/
├── gcs_app.py              # Ana uygulama dosyası
├── attitude_indicator.py   # Yapay ufuk bileşeni
├── map.html               # Harita arayüzü (HTML/JS)
├── ui_manager.py          # UI yönetim modülü (opsiyonel)
├── vehicle_manager.py     # Araç bağlantı modülü (opsiyonel)
├── requirements.txt       # Python bağımlılıkları
└── README.md             # Bu dosya
```

## 🎮 Kullanım

1. **Bağlantı Kurma**:
   - Port listesini yenileyin
   - Uygun COM portunu seçin (genellikle Pixhawk)
   - Baud rate'i ayarlayın (varsayılan: 57600)
   - "BAĞLAN" butonuna tıklayın

2. **Harita Kullanımı**:
   - Haritaya çift tıklayarak waypoint ekleyin
   - Waypoint'e çift tıklayarak silin
   - Araç konumu otomatik olarak takip edilir

3. **Görev Kontrolü**:
   - Uçuş modunu değiştirin (STABILIZE/AUTO/GUIDED)
   - Waypoint'leri araca gönderin
   - Rotayı temizleyin

4. **Telemetri İzleme**:
   - Gerçek zamanlı telemetri verilerini izleyin
   - Attitude indicator'da roll/pitch açılarını görün
   - Sistem loglarını takip edin

## 🔧 Teknik Detaylar

- **Framework**: PyQt5 + QWebEngine
- **Harita**: Leaflet.js + OpenStreetMap
- **İletişim**: DroneKit (MAVLink protokolü)
- **Grafik**: QPainter ile özel çizim bileşenleri
- **Thread Safety**: Qt sinyal-slot mekanizması

## 📝 Notlar

- Pixhawk bağlantısı için uygun sürücülerin yüklü olması gerekir
- İlk bağlantıda "PreArm: RC not calibrated" uyarıları normal karşılanmalıdır
- Harita için internet bağlantısı gereklidir (OpenStreetMap tiles)

## 🚀 Geliştirme

Proje modüler yapıda tasarlanmıştır:
- `gcs_app.py`: Ana koordinasyon ve UI
- `attitude_indicator.py`: Özel grafik bileşeni
- `map.html`: Web tabanlı harita arayüzü
- QWebChannel ile Python-JavaScript köprüsü

---

**Geliştirici**: Erkan (Cursor)  
**Versiyon**: 1.0  
**Tarih**: 2025 
