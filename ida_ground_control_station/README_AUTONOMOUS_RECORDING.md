# IDA Otonom Kayıt Sistemi Kullanım Kılavuzu

## Genel Bakış

IDA Ground Control Station'a entegre edilmiş otonom kayıt sistemi, aracınız otonom moda geçtiğinde otomatik olarak başlayan ve manuel moda geçtiğinde duran bir kayıt sistemidir.

## Özellikler

### 🎥 Otomatik Kayıt Başlatma/Durdurma
- **Otonom moda geçiş**: AUTO, GUIDED, RTL modlarına geçildiğinde kayıt otomatik başlar
- **Manuel moda geçiş**: MANUAL, STABILIZE, ACRO, ALT_HOLD modlarına geçildiğinde kayıt otomatik durur
- **Görev 2**: Görev 2 butonuna basıldığında kayıt otomatik başlar

### 📁 Kayıt Dosyaları (3 Dosya Türü)

#### 1. İşlenmiş Kamera Verisi (`processed_camera_data.mp4`)
- **Format**: MP4
- **Frekans**: 1 Hz (her saniye 1 frame)
- **İçerik**: 
  - Zaman etiketli frame'ler
  - Engel tespiti ve konumu (sarı renk algılama)
  - ROI (Region of Interest) çerçevesi
  - Araç modu ve ARM durumu
  - Engel yoğunluğu yüzdesi

#### 2. Lokal Harita/Cost Map (`local_obstacle_map.mp4`)
- **Format**: MP4
- **Frekans**: 1 Hz
- **İçerik**:
  - Engel haritası overlay'i
  - Bölge ayırıcıları (sol/orta/sağ)
  - Engel yoğunluğu görselleştirmesi
  - Zaman etiketli frame'ler

#### 3. Araç Telemetri Verisi (`vehicle_telemetry.csv`)
- **Format**: CSV
- **Frekans**: 1 Hz
- **İçerik**:
  - Timestamp (UTC)
  - Konum (lat, lon)
  - Hız (groundspeed_mps)
  - Yönelim açıları (roll_deg, pitch_deg, heading_deg)
  - Hız setpoint (speed_setpoint_mps)
  - Yön setpoint (heading_setpoint_deg)
  - Araç modu (vehicle_mode)
  - ARM durumu (armed_status)

## Kullanım

### Otomatik Kayıt (Önerilen)
1. Aracınızı bağlayın
2. Herhangi bir otonom moda geçin (AUTO, GUIDED, RTL)
3. Kayıt otomatik olarak başlayacak
4. Manuel moda geçtiğinizde kayıt otomatik olarak duracak

### Manuel Kayıt
1. "Otonom Kayıt Sistemi" panelinde "🎥 Kayıt Başlat" butonuna basın
2. Kayıt manuel olarak başlayacak
3. "⏹️ Kayıt Durdur" butonuna basarak kaydı durdurun

### Görev 2 ile Kayıt
1. "Görev 2" butonuna basın
2. Kayıt otomatik olarak başlayacak
3. Görev 2 durdurulduğunda kayıt da duracak

## Kayıt Dosyalarının Konumu

Kayıt dosyaları şu dizinde oluşturulur:
```
logs/autonomous_recording_YYYYMMDD_HHMMSS/
├── processed_camera_data.mp4
├── local_obstacle_map.mp4
└── vehicle_telemetry.csv
```

## Sistem Gereksinimleri

### Python Paketleri
```bash
pip install -r requirements.txt
```

### Donanım Gereksinimleri
- **Kamera**: USB kamera veya dahili kamera
- **Disk Alanı**: Kayıt süresine bağlı olarak değişir (yaklaşık 1MB/dakika)
- **RAM**: Minimum 2GB (OpenCV için)

## Sorun Giderme

### Kamera Açılamıyor
- Kamera başka bir uygulama tarafından kullanılıyor olabilir
- Kamera sürücülerini kontrol edin
- OpenCV kurulumunu kontrol edin: `pip install opencv-python`

### Kayıt Başlamıyor
- Araç bağlantısını kontrol edin
- Mod değişikliğinin başarılı olduğunu kontrol edin
- Sistem loglarını kontrol edin

### Dosya Boyutu Çok Büyük
- Kayıt süresini kontrol edin
- Gereksiz kayıtları silin
- Disk alanını kontrol edin

## Teknik Detaylar

### Engel Algılama
- **Renk Aralığı**: HSV (18-38, 140-255, 80-255) - Sarı renk
- **Eşik Değeri**: %15 sarı piksel yoğunluğu
- **Bölge Tespiti**: Sol/Orta/Sağ bölge analizi

### Video Codec
- **Codec**: MP4V (MPEG-4)
- **FPS**: 1.0 (1 frame/saniye)
- **Kalite**: Orijinal kamera çözünürlüğü

### CSV Format
- **Encoding**: UTF-8
- **Ayırıcı**: Virgül
- **Header**: İlk satırda sütun isimleri

## Güvenlik Notları

- Kayıt dosyaları hassas bilgiler içerebilir
- Dosyaları güvenli bir şekilde saklayın
- Gereksiz kayıtları düzenli olarak silin
- Kamera görüntülerini paylaşırken dikkatli olun

## Destek

Herhangi bir sorun yaşarsanız:
1. Sistem loglarını kontrol edin
2. Hata mesajlarını not edin
3. Teknik destek ekibiyle iletişime geçin
