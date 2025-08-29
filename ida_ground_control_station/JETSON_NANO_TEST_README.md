# Jetson Nano Test Fonksiyonu

Bu dokümantasyon, IDA Ground Control Station'da Jetson Nano ile test yapma özelliğini açıklar.

## Özellikler

- **Test Butonu**: Görev kontrolü panelinde "Test" butonu eklendi
- **Jetson Nano Bağlantısı**: SSH veya USB üzerinden bağlantı
- **Otomatik Test**: test.py dosyası otomatik olarak çalıştırılır
- **Gerçek Zamanlı Log**: Test sonuçları anlık olarak görüntülenir

## Kurulum

### 1. Gerekli Kütüphaneler

```bash
pip install paramiko
```

### 2. Jetson Nano Hazırlığı

Jetson Nano'da SSH servisinin aktif olduğundan emin olun:

```bash
sudo systemctl enable ssh
sudo systemctl start ssh
```

### 3. Bağlantı Ayarları

`gcs_app.py` dosyasında `_run_ssh_command_on_jetson` fonksiyonunda bağlantı bilgilerini güncelleyin:

```python
jetson_host = "192.168.1.100"  # Jetson Nano IP adresi
jetson_user = "nvidia"         # Jetson Nano kullanıcı adı
jetson_password = "password"   # Jetson Nano şifresi
```

## Kullanım

### 1. Test Butonunu Aktif Etme

1. Ground Control Station'ı başlatın
2. Pixhawk 2.4.8 ile bağlantı kurun
3. "Test" butonuna tıklayın

### 2. Test Süreci

Test butonuna tıkladığınızda:

1. **Bağlantı Kontrolü**: Araç bağlantısı kontrol edilir
2. **Dosya Kopyalama**: test.py dosyası Jetson Nano'ya kopyalanır
3. **Test Çalıştırma**: test.py dosyası Jetson Nano'da çalıştırılır
4. **Sonuç Görüntüleme**: Test sonuçları log panelinde görüntülenir

### 3. Test Sonuçları

Test başarılı olduğunda şu bilgiler görüntülenir:

- Sistem bilgileri (CPU, RAM)
- Python ortamı kontrolü
- Dosya sistemi erişimi
- USB bağlantısı kontrolü
- Pixhawk 2.4.8 iletişimi
- Telemetri verisi alma

## Bağlantı Yöntemleri

### SSH Bağlantısı (Önerilen)

SSH bağlantısı için `_run_ssh_command_on_jetson` fonksiyonunu kullanın:

```python
# gcs_app.py dosyasında bu satırı aktif edin:
self._run_ssh_command_on_jetson(test_file_path)
```

### USB Bağlantısı

USB bağlantısı için `_run_usb_command_on_jetson` fonksiyonunu geliştirin:

```python
# Bu fonksiyon şu anda placeholder
# USB seri bağlantısı veya ADB protokolü eklenebilir
```

## Test Dosyası (test.py)

`test.py` dosyası Jetson Nano'da çalıştırılacak test kodunu içerir:

- Sistem bilgilerini gösterir
- Python ortamını kontrol eder
- Dosya sistemi erişimini test eder
- USB bağlantısını kontrol eder
- Pixhawk 2.4.8 iletişimini test eder

## Hata Ayıklama

### Yaygın Hatalar

1. **Bağlantı Hatası**: Jetson Nano IP adresini kontrol edin
2. **Kimlik Doğrulama Hatası**: Kullanıcı adı ve şifreyi kontrol edin
3. **Dosya Bulunamadı**: test.py dosyasının varlığını kontrol edin
4. **Paramiko Hatası**: `pip install paramiko` komutunu çalıştırın

### Log Mesajları

- 🚁 JETSON NANO TEST BAŞLATILIYOR...
- 📡 Pixhawk 2.4.8 ile USB bağlantısı kuruluyor...
- 🔧 Test dosyası Jetson Nano'ya kopyalanıyor...
- ⚡ Test dosyası çalıştırılıyor...
- ✅ TEST BAŞARIYLA TAMAMLANDI!

## Geliştirme

### Yeni Test Fonksiyonları Ekleme

1. `test.py` dosyasına yeni test fonksiyonları ekleyin
2. `_execute_jetson_test` fonksiyonunu güncelleyin
3. Log mesajlarını ekleyin

### USB Bağlantısı Geliştirme

USB bağlantısı için şu yöntemler kullanılabilir:

- Seri port bağlantısı (pyserial)
- ADB protokolü
- USB tethering
- Network over USB

## Güvenlik

- SSH bağlantısı için güçlü şifreler kullanın
- Jetson Nano'da firewall ayarlarını kontrol edin
- Test dosyalarını güvenli konumlarda saklayın

## Destek

Sorun yaşadığınızda:

1. Log mesajlarını kontrol edin
2. Bağlantı ayarlarını doğrulayın
3. Jetson Nano'da SSH servisinin çalıştığını kontrol edin
4. Test dosyasının doğru konumda olduğunu kontrol edin
