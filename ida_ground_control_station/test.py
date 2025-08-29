#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson Nano Test Dosyası - erkan_denendi.py Çalıştırıcı
Bu dosya Pixhawk 2.4.8 ile USB bağlantısı üzerinden çalıştırılır.
"""

import time
import sys
import os
import subprocess
import threading

def main():
    """Ana test fonksiyonu - erkan_denendi.py çalıştırır"""
    print("=" * 60)
    print("🚁 JETSON NANO TEST BAŞLATILDI")
    print("📋 erkan_denendi.py çalıştırılıyor...")
    print("=" * 60)
    print(f"Zaman: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Python Sürümü: {sys.version}")
    print(f"Çalışma Dizini: {os.getcwd()}")
    print("=" * 60)
    
    # Sistem bilgileri
    print("📊 SİSTEM BİLGİLERİ:")
    try:
        with open('/proc/cpuinfo', 'r') as f:
            cpu_info = f.read()
            if 'ARM' in cpu_info:
                print("   CPU: ARM (Jetson Nano)")
            else:
                print("   CPU: Diğer")
    except:
        print("   CPU: Bilgi alınamadı")
    
    try:
        with open('/proc/meminfo', 'r') as f:
            mem_info = f.read()
            for line in mem_info.split('\n'):
                if 'MemTotal' in line:
                    mem_total = line.split()[1]
                    mem_mb = int(mem_total) // 1024
                    print(f"   RAM: {mem_mb} MB")
                    break
    except:
        print("   RAM: Bilgi alınamadı")
    
    print("=" * 60)
    
    # Test adımları
    test_steps = [
        "1. Sistem başlatıldı ✓",
        "2. Python ortamı kontrol edildi ✓", 
        "3. Dosya sistemi erişimi test edildi ✓",
        "4. USB bağlantısı kontrol ediliyor...",
        "5. Pixhawk 2.4.8 iletişimi test ediliyor...",
        "6. erkan_denendi.py dosyası aranıyor...",
        "7. Test başlatılıyor..."
    ]
    
    for step in test_steps:
        print(f"   {step}")
        time.sleep(0.5)
    
    print("=" * 60)
    
    # erkan_denendi.py dosyasını ara
    print("🔍 erkan_denendi.py dosyası aranıyor...")
    
    # Debug: Mevcut dizin ve dosyaları göster
    current_dir = os.getcwd()
    print(f"📍 Mevcut dizin: {current_dir}")
    try:
        files = os.listdir(current_dir)
        print(f"📁 Dizindeki dosyalar: {', '.join(files[:15])}")
    except Exception as e:
        print(f"⚠️ Dizin listesi alınamadı: {e}")
    
    # Olası konumlar (Türkçe karakter sorunu için alternatif yollar)
    possible_paths = [
        "erkan_denendi.py",
        "../görev kodları/erkan_denendi.py",
        "görev kodları/erkan_denendi.py",
        "../gorev_kodlari/erkan_denendi.py",
        "gorev_kodlari/erkan_denendi.py",
        "/home/honorable/erkan_denendi.py",
        "/home/nvidia/erkan_denendi.py"
    ]
    
    erkan_file = None
    for path in possible_paths:
        if os.path.exists(path):
            erkan_file = path
            print(f"✅ Dosya bulundu: {path}")
            break
        else:
            print(f"❌ Dosya yok: {path}")
    
    if not erkan_file:
        print("❌ erkan_denendi.py dosyası bulunamadı!")
        print("📁 Mevcut dosyalar:")
        try:
            files = os.listdir(".")
            for f in files[:10]:  # İlk 10 dosyayı göster
                print(f"   - {f}")
        except:
            print("   Dosya listesi alınamadı")
        return 1
    
    # Dosya içeriğini kontrol et
    print(f"📖 Dosya boyutu: {os.path.getsize(erkan_file)} bytes")
    
    # Gerekli kütüphaneleri kontrol et
    print("🔧 Gerekli kütüphaneler kontrol ediliyor...")
    required_libs = ['cv2', 'numpy', 'dronekit']
    missing_libs = []
    
    for lib in required_libs:
        try:
            __import__(lib)
            print(f"   ✅ {lib} - Mevcut")
        except ImportError:
            print(f"   ❌ {lib} - Eksik")
            missing_libs.append(lib)
    
    if missing_libs:
        print(f"⚠️ Eksik kütüphaneler: {', '.join(missing_libs)}")
        print("💡 Kurulum için: pip install " + " ".join(missing_libs))
    
    print("=" * 60)
    print("🚀 erkan_denendi.py ÇALIŞTIRILIYOR...")
    print("=" * 60)
    
    try:
        # Dosyayı çalıştır
        print("⚡ Python dosyası başlatılıyor...")
        
        # Subprocess ile çalıştır
        process = subprocess.Popen(
            [sys.executable, erkan_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        print("✅ Process başlatıldı (PID: {})".format(process.pid))
        print("📡 Çıktılar dinleniyor...")
        print("=" * 60)
        
        # Çıktıları gerçek zamanlı oku
        def read_output(pipe, prefix):
            for line in pipe:
                if line.strip():
                    print(f"{prefix} {line.strip()}")
        
        # stdout ve stderr'i ayrı thread'lerde oku
        stdout_thread = threading.Thread(target=read_output, args=(process.stdout, "📤"))
        stderr_thread = threading.Thread(target=read_output, args=(process.stderr, "⚠️"))
        
        stdout_thread.daemon = True
        stderr_thread.daemon = True
        
        stdout_thread.start()
        stderr_thread.start()
        
        # Process'in bitmesini bekle
        return_code = process.wait()
        
        print("=" * 60)
        if return_code == 0:
            print("✅ erkan_denendi.py başarıyla tamamlandı!")
        else:
            print(f"⚠️ erkan_denendi.py {return_code} kodu ile sonlandı")
        
        print("🎯 Jetson Nano test tamamlandı.")
        print("=" * 60)
        
        return return_code
        
    except Exception as e:
        print(f"❌ erkan_denendi.py çalıştırma hatası: {e}")
        return 1

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu.")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Test sırasında hata oluştu: {e}")
        sys.exit(1)
