#!/usr/bin/env python3
"""
YKİ Koordinat Debug Aracı
Bu araç, GPS koordinatlarının harita üzerinde doğru konumda gösterilip gösterilmediğini test eder.
"""

import math
import json
from typing import Tuple, List, Dict, Any

class CoordinateDebugger:
    """Koordinat dönüşümü ve harita projeksiyonu debug aracı"""
    
    def __init__(self):
        self.EARTH_RADIUS = 6378137.0  # WGS84 elipsoid yarıçapı (metre)
        self.WEB_MERCATOR_MAX = 20037508.342789244  # Web Mercator maksimum değer
    
    def wgs84_to_web_mercator(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        WGS84 (EPSG:4326) koordinatlarını Web Mercator (EPSG:3857) projektif koordinatlara dönüştürür.
        
        Args:
            lat: Enlem (derece)
            lon: Boylam (derece)
            
        Returns:
            x, y: Web Mercator koordinatları (metre)
        """
        # Boylam dönüşümü
        x = lon * math.pi * self.EARTH_RADIUS / 180.0
        
        # Enlem dönüşümü (Mercator projeksiyonu)
        lat_rad = lat * math.pi / 180.0
        y = self.EARTH_RADIUS * math.log(math.tan(math.pi / 4.0 + lat_rad / 2.0))
        
        return x, y
    
    def web_mercator_to_wgs84(self, x: float, y: float) -> Tuple[float, float]:
        """
        Web Mercator koordinatlarını WGS84'e geri dönüştürür.
        
        Args:
            x, y: Web Mercator koordinatları (metre)
            
        Returns:
            lat, lon: WGS84 koordinatları (derece)
        """
        lon = x * 180.0 / (math.pi * self.EARTH_RADIUS)
        lat = (2.0 * math.atan(math.exp(y / self.EARTH_RADIUS)) - math.pi / 2.0) * 180.0 / math.pi
        
        return lat, lon
    
    def calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        İki GPS koordinatı arasındaki mesafeyi Haversine formülü ile hesaplar.
        
        Returns:
            Mesafe (metre)
        """
        # Derece → Radyan
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formülü
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        a = (math.sin(dlat / 2) ** 2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
        c = 2 * math.asin(math.sqrt(a))
        
        return self.EARTH_RADIUS * c
    
    def calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        İki nokta arasındaki bearing'i (yön açısını) hesaplar.
        
        Returns:
            Bearing (derece, 0-360 arası)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon_rad = math.radians(lon2 - lon1)
        
        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))
        
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)
        
        # 0-360 arası normalize et
        return (bearing_deg + 360) % 360
    
    def test_coordinate_accuracy(self, gps_coords: List[Tuple[float, float]], 
                                tolerance_meters: float = 1.0) -> Dict[str, Any]:
        """
        GPS koordinatlarının dönüşüm doğruluğunu test eder.
        
        Args:
            gps_coords: [(lat, lon), ...] GPS koordinat listesi
            tolerance_meters: Kabul edilebilir hata toleransı (metre)
            
        Returns:
            Test sonuçları içeren dict
        """
        results = {
            "test_count": len(gps_coords),
            "passed": 0,
            "failed": 0,
            "max_error": 0.0,
            "avg_error": 0.0,
            "details": []
        }
        
        total_error = 0.0
        
        for i, (lat, lon) in enumerate(gps_coords):
            # WGS84 → Web Mercator → WGS84 dönüşümü test et
            x, y = self.wgs84_to_web_mercator(lat, lon)
            lat_back, lon_back = self.web_mercator_to_wgs84(x, y)
            
            # Hata mesafesini hesapla
            error_distance = self.calculate_distance(lat, lon, lat_back, lon_back)
            total_error += error_distance
            
            if error_distance > results["max_error"]:
                results["max_error"] = error_distance
            
            test_result = {
                "point": i + 1,
                "original": (lat, lon),
                "web_mercator": (x, y),
                "converted_back": (lat_back, lon_back),
                "error_meters": error_distance,
                "passed": error_distance <= tolerance_meters
            }
            
            if test_result["passed"]:
                results["passed"] += 1
            else:
                results["failed"] += 1
            
            results["details"].append(test_result)
        
        results["avg_error"] = total_error / len(gps_coords) if gps_coords else 0.0
        
        return results
    
    def generate_test_waypoints(self, center_lat: float, center_lon: float, 
                              radius_km: float = 1.0, count: int = 8) -> List[Tuple[float, float]]:
        """
        Belirtilen merkez etrafında test waypoint'leri oluşturur.
        
        Args:
            center_lat, center_lon: Merkez koordinatları
            radius_km: Yarıçap (kilometre)
            count: Waypoint sayısı
            
        Returns:
            [(lat, lon), ...] koordinat listesi
        """
        waypoints = []
        
        # Derece olarak yarıçap yaklaşımı (kabaca)
        radius_deg = radius_km / 111.0  # 1 derece ≈ 111 km
        
        for i in range(count):
            angle = 2 * math.pi * i / count
            
            lat = center_lat + radius_deg * math.cos(angle)
            lon = center_lon + radius_deg * math.sin(angle) / math.cos(math.radians(center_lat))
            
            waypoints.append((lat, lon))
        
        return waypoints
    
    def validate_coordinate_precision(self, lat: float, lon: float) -> Dict[str, Any]:
        """
        Koordinat precision'ını analiz eder.
        
        Returns:
            Precision analiz sonuçları
        """
        # Koordinat stringi analizi
        lat_str = f"{lat:.10f}"
        lon_str = f"{lon:.10f}"
        
        lat_decimal_places = len(lat_str.split('.')[-1]) if '.' in lat_str else 0
        lon_decimal_places = len(lon_str.split('.')[-1]) if '.' in lon_str else 0
        
        # Precision → Metre accuracy mapping (enlem için)
        # 1 decimal place ≈ 11.1 km
        # 2 decimal places ≈ 1.1 km  
        # 3 decimal places ≈ 110 m
        # 4 decimal places ≈ 11 m
        # 5 decimal places ≈ 1.1 m
        # 6 decimal places ≈ 0.11 m
        precision_accuracy = {
            1: 11100, 2: 1110, 3: 111, 4: 11.1, 5: 1.11, 6: 0.111, 7: 0.0111
        }
        
        lat_accuracy = precision_accuracy.get(lat_decimal_places, 0.001)
        lon_accuracy = precision_accuracy.get(lon_decimal_places, 0.001)
        
        return {
            "latitude": {
                "value": lat,
                "decimal_places": lat_decimal_places,
                "accuracy_meters": lat_accuracy
            },
            "longitude": {
                "value": lon,
                "decimal_places": lon_decimal_places,
                "accuracy_meters": lon_accuracy
            },
            "overall_accuracy_meters": max(lat_accuracy, lon_accuracy),
            "sufficient_for_marine": lat_accuracy <= 1.0 and lon_accuracy <= 1.0
        }

def run_debug_tests():
    """Debug testlerini çalıştırır ve sonuçları yazdırır."""
    debugger = CoordinateDebugger()
    
    print("=== YKİ Koordinat Debug Testleri ===\n")
    
    # Test 1: İstanbul Koordinatları (mevcut sistemde kullanılan başlangıç noktası)
    test_coords = [
        (41.015137, 28.979530),  # İstanbul başlangıç noktası
        (41.020000, 28.980000),  # Yakın nokta 1
        (41.010000, 28.985000),  # Yakın nokta 2
        (41.025000, 28.975000),  # Yakın nokta 3
    ]
    
    print("Test 1: Koordinat dönüşüm doğruluğu")
    results = debugger.test_coordinate_accuracy(test_coords, tolerance_meters=0.1)
    
    print(f"Toplam test: {results['test_count']}")
    print(f"Başarılı: {results['passed']}")
    print(f"Başarısız: {results['failed']}")
    print(f"Maksimum hata: {results['max_error']:.6f} metre")
    print(f"Ortalama hata: {results['avg_error']:.6f} metre")
    
    if results['failed'] > 0:
        print("\n⚠️  DİKKAT: Koordinat dönüşümünde hata tespit edildi!")
        for detail in results['details']:
            if not detail['passed']:
                print(f"Nokta {detail['point']}: {detail['error_meters']:.6f}m hata")
    else:
        print("✅ Koordinat dönüşümü başarılı")
    
    print("\n" + "="*50)
    
    # Test 2: Precision analizi
    print("\nTest 2: Koordinat precision analizi")
    for i, (lat, lon) in enumerate(test_coords[:2]):
        precision_result = debugger.validate_coordinate_precision(lat, lon)
        print(f"\nNokta {i+1}: ({lat}, {lon})")
        print(f"Enlem precision: {precision_result['latitude']['decimal_places']} basamak → {precision_result['latitude']['accuracy_meters']}m")
        print(f"Boylam precision: {precision_result['longitude']['decimal_places']} basamak → {precision_result['longitude']['accuracy_meters']}m")
        print(f"Genel accuracy: {precision_result['overall_accuracy_meters']}m")
        print(f"Denizcilik için yeterli: {'✅' if precision_result['sufficient_for_marine'] else '❌'}")
    
    print("\n" + "="*50)
    
    # Test 3: Test waypoint'leri oluştur
    print("\nTest 3: Test waypoint'leri (İstanbul merkez, 1km yarıçap)")
    center_lat, center_lon = 41.015137, 28.979530
    test_waypoints = debugger.generate_test_waypoints(center_lat, center_lon, radius_km=1.0, count=6)
    
    for i, (lat, lon) in enumerate(test_waypoints):
        distance = debugger.calculate_distance(center_lat, center_lon, lat, lon)
        bearing = debugger.calculate_bearing(center_lat, center_lon, lat, lon)
        print(f"Waypoint {i+1}: ({lat:.6f}, {lon:.6f}) - {distance:.1f}m, {bearing:.1f}°")

if __name__ == "__main__":
    run_debug_tests() 