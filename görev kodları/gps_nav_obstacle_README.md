## GPS NAV + Engel Kaçınma — Kullanım ve Notlar

Bu belge `gps_nav_obstacle.py` betiğinin nasıl kullanılacağını, gerekli ayarları, kalibrasyon önerilerini ve güvenlik notlarını özetler.

### Gereksinimler
- **ArduPilot Rover** (Pixhawk vb.) ve RC override ile sürüş desteği
- Aracın önüne bakan bir **kamera** (OpenCV ile erişilebilir)
- Python 3.8+
- Python paketleri: `dronekit`, `opencv-python`, `numpy`

Kurulum (örnek):
```bash
pip install dronekit opencv-python numpy
```

### Donanım/Ayarlar
- ArduPilot tarafında **MANUAL** moda geçiş yapılabilir ve araç **arm** edilebilir olmalı.
- RC kanal eşleşmeleri ArduRover varsayılanı kabul edilmiştir: `CH1 = direksiyon`, `CH3 = gaz`.
- Betik RC override gönderir; Mission Planner açıkken RC override çakışmasından kaçının.
- Bağlantı dizesi örnekleri:
  - Seri: `COM18`, `COM5`
  - UDP: `udp:127.0.0.1:14550`

### Çalıştırma
Temel kullanım (hedef koordinat vererek):
```bash
python3 "görev kodları/gps_nav_obstacle.py" \
  --lat 40.7710894 \
  --lon 29.9784 \
  --conn COM18 \
  --baud 57600 \
  --show
```

Pencereyi kapatmak için `q`. GPS fix yoksa araç durur. Hedefe varış yarıçapı varsayılan olarak 3 m.

### Argümanlar
- **--lat**: Hedef enlem (derece)
- **--lon**: Hedef boylam (derece)
- **--conn**: Bağlantı dizesi (örn. `COM18`, `udp:127.0.0.1:14550`)
- **--baud**: Baud hızı (varsayılan 57600)
- **--show**: Görsel pencereyi açar
- **--no-show**: Görsel pencereyi kapatır

Argüman verilmezse dosya içindeki varsayılan hedef koordinatlar kullanılır.

### Çalışma Mantığı (Özet)
- GPS konumundan hedefe doğru **bearing** hesaplanır.
- Mevcut **heading** ile hedef bearing arasındaki hata derece cinsinden bulunur.
- Hata -> direksiyon PWM; sabit hızda **RC override** ile sürüş yapılır.
- Kameranın alt-orta ROI bölgesinde **Canny** kenar yoğunluğuna göre basit bir engel skoru üretilir.
- Engel algılanırsa kısa bir kaçınma manevrası uygulanır: sağa kır → düz → sola kır → tekrar hedefe yönel.

### Ayarlanabilir Parametreler (dosya içinden)
`görev kodları/gps_nav_obstacle.py` içinde:
- **Sürüş**
  - `PWM_CRUISE`: Seyir gazı
  - `PWM_AVOID`: Kaçınma sırasında gaz
  - `STEER_GAIN_DEG_PER_PWM`: Heading hata → direksiyon PWM dönüşüm kazancı
  - `STEER_MIN/STEER_MAX`: Direksiyon PWM sınırları
- **Varış**
  - `ARRIVAL_RADIUS_M`: Hedefe varmış sayılacak yarıçap (m)
- **Kaçınma**
  - `AVOID_TURN_RIGHT_S`, `AVOID_STRAIGHT_S`, `AVOID_TURN_LEFT_S`: Manevra süreleri (s)
  - `AVOID_STEER_OFFSET`: Manevra sırasında direksiyon PWM ofseti
- **Engel algılama**
  - `OBS_ROI_W_RATIO`, `OBS_ROI_H_RATIO`: ROI boyutu (genişlik/yükseklik oranları)
  - `OBS_CANNY1`, `OBS_CANNY2`: Canny eşikleri
  - `OBS_MIN_EDGE_RATIO`: Kenar piksel oranı eşiği (engel hassasiyeti)

### Kalibrasyon Önerileri
- İlk denemeleri düşük hızda ve geniş alanlarda yapın.
- Işık koşulları değiştikçe `OBS_CANNY*` ve `OBS_MIN_EDGE_RATIO` değerlerini ayarlayın.
- Araç dönüş yönü ters gözlenirse direksiyon kazancının işaretini/ölçeğini uyarlayın.
- Varış yarıçapını GPS doğruluğunuza göre belirleyin (GNSS/RTK yoksa 2–5 m arası önerilir).

### Güvenlik Notları
- Aracı **havada/askıda** veya tekerler yükseltilmiş şekilde ilk test edin.
- Boş ve güvenli bir bölgede düşük hızla saha testi yapın.
- Acil durumda güç kesmek/RC’dan kontrolü devralmak için hazır olun.
- Kamera engel algısı yalındır; parlak zeminlerde sahte pozitif/negatifler olabilir.

### Sorun Giderme
- Bağlantı hatası: `--conn` ve `--baud` değerlerini doğrulayın; Mission Planner bağlantı çakışmalarını kapatın.
- GPS yok/fix zayıf: Anten konumunu/çevresini iyileştirin; açık alanda deneyin.
- Araç dönmüyor/ters dönüyor: Direksiyon yönünü ve `STEER_GAIN_DEG_PER_PWM` değerini gözden geçirin.
- Sürekli engel algısı: ROI’yi küçültün, `OBS_MIN_EDGE_RATIO` eşiklerini arttırın.

### Bilinen Kısıtlar
- Engel algılama yalnızca basit kenar yoğunluğuna dayanır; derinlik bilgisi içermez.
- Manevra deterministiktir; karmaşık engel/geometri durumlarında geliştirme gerektirir.

### Dosyalar
- Betik: `görev kodları/gps_nav_obstacle.py`
- Bu belge: `görev kodları/gps_nav_obstacle_README.md`


