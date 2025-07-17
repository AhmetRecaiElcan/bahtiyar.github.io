#!/usr/bin/env python3
"""
YKİ Test Senaryosu Oluşturucu
Bu script, belirli koordinat ve heading değerleriyle test senaryoları oluşturur
ve koordinat doğruluğunu test eder.
"""

import json
import math
import time
from typing import List, Dict, Tuple, Any
from coordinate_debug import CoordinateDebugger

class YKITestScenario:
    """Test senaryosu oluşturucu ve çalıştırıcı"""
    
    def __init__(self):
        self.debugger = CoordinateDebugger()
        self.test_results = []
        
    def create_precision_test_scenario(self) -> Dict[str, Any]:
        """Koordinat precision test senaryosu"""
        scenario = {
            "name": "Koordinat Precision Testi",
            "description": "Farklı precision seviyelerinde koordinat doğruluğunu test eder",
            "test_points": [
                # Yüksek precision (6+ ondalık)
                {"lat": 41.025631, "lon": 28.974298, "heading": 0, "name": "Galata Kulesi (Yüksek Precision)"},
                {"lat": 41.008238, "lon": 28.978359, "heading": 45, "name": "Eminönü (Yüksek Precision)"},
                
                # Orta precision (4-5 ondalık)
                {"lat": 41.0256, "lon": 28.9743, "heading": 90, "name": "Galata Kulesi (Orta Precision)"},
                {"lat": 41.0082, "lon": 28.9784, "heading": 135, "name": "Eminönü (Orta Precision)"},
                
                # Düşük precision (2-3 ondalık)
                {"lat": 41.03, "lon": 28.97, "heading": 180, "name": "Yaklaşık Konum (Düşük Precision)"},
                {"lat": 41.01, "lon": 28.98, "heading": 225, "name": "Yaklaşık Konum 2 (Düşük Precision)"},
            ],
            "expected_results": {
                "high_precision": {"accuracy_meters": "< 0.1", "acceptable": True},
                "medium_precision": {"accuracy_meters": "< 1.0", "acceptable": True},
                "low_precision": {"accuracy_meters": "< 100", "acceptable": False}
            }
        }
        return scenario
    
    def create_navigation_test_scenario(self) -> Dict[str, Any]:
        """Navigasyon test senaryosu - İstanbul Boğazı rotası"""
        scenario = {
            "name": "İstanbul Boğazı Navigasyon Testi",
            "description": "İstanbul Boğazı üzerinde gerçekçi deniz aracı rotası",
            "route_points": [
                {"lat": 41.013611, "lon": 28.949167, "heading": 0, "name": "Başlangıç - Karaköy", "speed": 0},
                {"lat": 41.017222, "lon": 28.967778, "heading": 45, "name": "Galata Köprüsü", "speed": 3.5},
                {"lat": 41.025631, "lon": 28.974298, "heading": 90, "name": "Galata Kulesi Yanı", "speed": 5.0},
                {"lat": 41.040556, "lon": 29.000000, "heading": 45, "name": "Üsküdar Yaklaşım", "speed": 4.2},
                {"lat": 41.044167, "lon": 29.013056, "heading": 0, "name": "Üsküdar Limanı", "speed": 2.0}
            ],
            "telemetry_data": {
                "speed_setpoint": 4.0,
                "heading_setpoint_offset": 5,  # Setpoint'ten sapma
                "thruster_scenario": "normal_navigation",
                "environmental_conditions": {
                    "current_speed": 0.5,  # Akıntı hızı
                    "current_direction": 180,  # Akıntı yönü
                    "wind_speed": 3.0,
                    "wind_direction": 225
                }
            }
        }
        return scenario
    
    def create_precision_challenge_scenario(self) -> Dict[str, Any]:
        """Hassas konumlandırma zorluk senaryosu"""
        scenario = {
            "name": "Hassas Konumlandırma Zorluğu",
            "description": "Dar geçit ve rıhtım yaklaşımları için hassas navigasyon",
            "challenge_points": [
                # Çok hassas konumlandırma gereken yerler
                {"lat": 41.016389, "lon": 28.983056, "heading": 270, "name": "Eminönü Rıhtımı", 
                 "tolerance_meters": 0.5, "approach_speed": 1.0},
                
                {"lat": 41.017500, "lon": 28.976111, "heading": 180, "name": "Galata Köprüsü Altı", 
                 "tolerance_meters": 1.0, "approach_speed": 0.8},
                
                {"lat": 41.025278, "lon": 28.975556, "heading": 90, "name": "Galata Rıhtımı", 
                 "tolerance_meters": 0.3, "approach_speed": 0.5},
                
                # Koordinat sistemi test noktaları
                {"lat": 41.000000, "lon": 29.000000, "heading": 0, "name": "Tam Koordinat", 
                 "tolerance_meters": 0.1, "approach_speed": 2.0},
                
                {"lat": 41.123456, "lon": 28.654321, "heading": 123, "name": "Ondalık Test", 
                 "tolerance_meters": 0.1, "approach_speed": 2.0},
            ]
        }
        return scenario
    
    def run_precision_test(self, scenario: Dict[str, Any]) -> Dict[str, Any]:
        """Precision test senaryosunu çalıştır"""
        results = {
            "scenario_name": scenario["name"],
            "timestamp": time.time(),
            "tests": [],
            "summary": {"passed": 0, "failed": 0, "total": 0}
        }
        
        for point in scenario["test_points"]:
            lat, lon = point["lat"], point["lon"]
            
            # Precision analizi
            precision = self.debugger.validate_coordinate_precision(lat, lon)
            
            # Koordinat dönüşüm testi
            coord_test = self.debugger.test_coordinate_accuracy([(lat, lon)], tolerance_meters=0.1)
            
            test_result = {
                "point_name": point["name"],
                "coordinates": {"lat": lat, "lon": lon},
                "heading": point["heading"],
                "precision_analysis": precision,
                "coordinate_accuracy": coord_test["details"][0] if coord_test["details"] else None,
                "test_passed": coord_test["passed"] > 0 and precision["sufficient_for_marine"]
            }
            
            results["tests"].append(test_result)
            
            if test_result["test_passed"]:
                results["summary"]["passed"] += 1
            else:
                results["summary"]["failed"] += 1
            results["summary"]["total"] += 1
        
        return results
    
    def run_navigation_simulation(self, scenario: Dict[str, Any], duration_seconds: int = 300) -> Dict[str, Any]:
        """Navigasyon simülasyonu çalıştır"""
        results = {
            "scenario_name": scenario["name"],
            "timestamp": time.time(),
            "route_data": [],
            "telemetry_log": [],
            "performance_metrics": {}
        }
        
        route_points = scenario["route_points"]
        telemetry_data = scenario["telemetry_data"]
        
        # Simülasyon parametreleri
        simulation_steps = duration_seconds * 2  # 2 Hz simülasyon
        current_point_index = 0
        
        for step in range(simulation_steps):
            simulation_time = step * 0.5  # 0.5 saniye intervals
            
            # Mevcut hedef nokta
            if current_point_index < len(route_points):
                current_target = route_points[current_point_index]
                
                # Basit linear interpolation ile konum hesapla
                if current_point_index < len(route_points) - 1:
                    next_target = route_points[current_point_index + 1]
                    
                    # Mesafe ve bearing hesapla
                    distance = self.debugger.calculate_distance(
                        current_target["lat"], current_target["lon"],
                        next_target["lat"], next_target["lon"]
                    )
                    
                    bearing = self.debugger.calculate_bearing(
                        current_target["lat"], current_target["lon"],
                        next_target["lat"], next_target["lon"]
                    )
                    
                    # Hız ve yön simülasyonu
                    speed = current_target.get("speed", 3.0)
                    travel_distance = speed * 0.5  # 0.5 saniyede gidilen mesafe
                    
                    # Yeni konum hesapla (basitleştirilmiş)
                    progress = min(1.0, travel_distance / distance)
                    
                    lat = current_target["lat"] + progress * (next_target["lat"] - current_target["lat"])
                    lon = current_target["lon"] + progress * (next_target["lon"] - current_target["lon"])
                    heading = bearing
                    
                    # Bir sonraki noktaya geçme kontrolü
                    if progress >= 0.9:  # %90 yaklaştığında sonraki noktaya geç
                        current_point_index += 1
                
                else:
                    # Son nokta
                    lat = current_target["lat"]
                    lon = current_target["lon"]
                    heading = current_target["heading"]
                    speed = 0
                
                # Çevresel faktörleri ekle
                env_conditions = telemetry_data.get("environmental_conditions", {})
                current_speed = env_conditions.get("current_speed", 0)
                current_direction = env_conditions.get("current_direction", 0)
                
                # Simulated noise/error ekle
                lat_noise = (math.sin(simulation_time * 0.1) * 0.000001)  # Çok küçük GPS noise
                lon_noise = (math.cos(simulation_time * 0.1) * 0.000001)
                heading_noise = math.sin(simulation_time * 0.2) * 2  # 2 derece heading noise
                
                # Telemetri verisi oluştur
                telemetry_point = {
                    "timestamp": simulation_time,
                    "position": {
                        "lat": lat + lat_noise,
                        "lon": lon + lon_noise,
                        "alt": 0.5
                    },
                    "heading": (heading + heading_noise) % 360,
                    "heading_setpoint": (heading + telemetry_data.get("heading_setpoint_offset", 0)) % 360,
                    "speed": speed + math.sin(simulation_time * 0.3) * 0.2,  # Speed variation
                    "speed_setpoint": telemetry_data.get("speed_setpoint", 4.0),
                    "thruster_forces": self._simulate_thruster_forces(speed, heading),
                    "environmental": env_conditions
                }
                
                results["telemetry_log"].append(telemetry_point)
        
        # Performans metrikleri hesapla
        results["performance_metrics"] = self._calculate_performance_metrics(results["telemetry_log"])
        
        return results
    
    def _simulate_thruster_forces(self, speed: float, heading: float) -> List[float]:
        """Thruster kuvvetlerini simüle et"""
        # 4 thrustern bir ROV/USV için basit kuvvet dağılımı
        # T1: Sol ön, T2: Sağ ön, T3: Sol arka, T4: Sağ arka
        
        base_thrust = speed * 20  # Hıza göre temel kuvvet
        
        # Yön değişimi için farklı thruster kuvvetleri
        if speed < 0.5:
            # Hovering/station keeping
            return [10, 10, 10, 10]
        else:
            # İleri hareket
            return [
                base_thrust + math.sin(math.radians(heading)) * 10,  # T1
                base_thrust - math.sin(math.radians(heading)) * 10,  # T2
                base_thrust + math.sin(math.radians(heading)) * 5,   # T3
                base_thrust - math.sin(math.radians(heading)) * 5    # T4
            ]
    
    def _calculate_performance_metrics(self, telemetry_log: List[Dict]) -> Dict[str, Any]:
        """Performans metriklerini hesapla"""
        if not telemetry_log:
            return {}
        
        # Hız hata analizi
        speed_errors = [abs(t["speed"] - t["speed_setpoint"]) for t in telemetry_log]
        
        # Heading hata analizi
        heading_errors = []
        for t in telemetry_log:
            error = abs(t["heading"] - t["heading_setpoint"])
            if error > 180:
                error = 360 - error  # En kısa açı farkı
            heading_errors.append(error)
        
        # Konum doğruluğu (GPS noise analizi)
        position_variations = []
        for i in range(1, len(telemetry_log)):
            prev_pos = telemetry_log[i-1]["position"]
            curr_pos = telemetry_log[i]["position"]
            
            distance = self.debugger.calculate_distance(
                prev_pos["lat"], prev_pos["lon"],
                curr_pos["lat"], curr_pos["lon"]
            )
            position_variations.append(distance)
        
        return {
            "speed_control": {
                "mean_error": sum(speed_errors) / len(speed_errors),
                "max_error": max(speed_errors),
                "std_error": self._calculate_std(speed_errors)
            },
            "heading_control": {
                "mean_error": sum(heading_errors) / len(heading_errors),
                "max_error": max(heading_errors),
                "std_error": self._calculate_std(heading_errors)
            },
            "position_accuracy": {
                "mean_variation": sum(position_variations) / len(position_variations) if position_variations else 0,
                "max_variation": max(position_variations) if position_variations else 0
            },
            "total_duration": telemetry_log[-1]["timestamp"] - telemetry_log[0]["timestamp"],
            "data_points": len(telemetry_log)
        }
    
    def _calculate_std(self, values: List[float]) -> float:
        """Standart sapma hesapla"""
        if len(values) < 2:
            return 0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
        return math.sqrt(variance)
    
    def save_test_results(self, results: Dict[str, Any], filename: str):
        """Test sonuçlarını dosyaya kaydet"""
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"Test sonuçları kaydedildi: {filename}")
    
    def generate_test_report(self, results: Dict[str, Any]) -> str:
        """Test raporu oluştur"""
        report = f"""
🚢 YKİ Test Raporu
================

Test Adı: {results['scenario_name']}
Tarih: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(results['timestamp']))}

"""
        
        if "tests" in results:
            # Precision test raporu
            report += f"""📊 Precision Test Sonuçları:
Toplam Test: {results['summary']['total']}
Başarılı: {results['summary']['passed']} ✅
Başarısız: {results['summary']['failed']} ❌

Detaylar:
"""
            for test in results["tests"]:
                status = "✅" if test["test_passed"] else "❌"
                report += f"  {status} {test['point_name']}\n"
                report += f"    Koordinat: {test['coordinates']['lat']:.6f}, {test['coordinates']['lon']:.6f}\n"
                report += f"    Precision: {test['precision_analysis']['overall_accuracy_meters']}m\n"
                if test['coordinate_accuracy']:
                    report += f"    Dönüşüm Hatası: {test['coordinate_accuracy']['error_meters']:.6f}m\n"
                report += "\n"
        
        elif "performance_metrics" in results:
            # Navigasyon test raporu
            metrics = results["performance_metrics"]
            report += f"""🧭 Navigasyon Test Sonuçları:
Test Süresi: {metrics.get('total_duration', 0):.1f} saniye
Veri Noktaları: {metrics.get('data_points', 0)}

Hız Kontrolü:
  Ortalama Hata: {metrics['speed_control']['mean_error']:.2f} m/s
  Maksimum Hata: {metrics['speed_control']['max_error']:.2f} m/s
  Standart Sapma: {metrics['speed_control']['std_error']:.2f} m/s

Yön Kontrolü:
  Ortalama Hata: {metrics['heading_control']['mean_error']:.1f}°
  Maksimum Hata: {metrics['heading_control']['max_error']:.1f}°
  Standart Sapma: {metrics['heading_control']['std_error']:.1f}°

Konum Doğruluğu:
  Ortalama Varyasyon: {metrics['position_accuracy']['mean_variation']:.3f}m
  Maksimum Varyasyon: {metrics['position_accuracy']['max_variation']:.3f}m
"""
        
        return report

def main():
    """Ana test senaryolarını çalıştır"""
    tester = YKITestScenario()
    
    print("🚢 YKİ Test Senaryoları Başlatılıyor...\n")
    
    # 1. Precision Test
    print("1. Koordinat Precision Testi çalıştırılıyor...")
    precision_scenario = tester.create_precision_test_scenario()
    precision_results = tester.run_precision_test(precision_scenario)
    
    precision_report = tester.generate_test_report(precision_results)
    print(precision_report)
    
    tester.save_test_results(precision_results, "precision_test_results.json")
    
    # 2. Navigation Simulation Test
    print("\n2. Navigasyon Simülasyonu çalıştırılıyor...")
    nav_scenario = tester.create_navigation_test_scenario()
    nav_results = tester.run_navigation_simulation(nav_scenario, duration_seconds=120)
    
    nav_report = tester.generate_test_report(nav_results)
    print(nav_report)
    
    tester.save_test_results(nav_results, "navigation_test_results.json")
    
    # 3. Örnek telemetri verisi oluştur (harita test için)
    print("\n3. Harita test verisi oluşturuluyor...")
    test_data = {
        "test_coordinates": [
            {"lat": 41.025631, "lon": 28.974298, "heading": 0, "name": "Galata Kulesi"},
            {"lat": 41.008238, "lon": 28.978359, "heading": 90, "name": "Eminönü"},
            {"lat": 41.040556, "lon": 29.000000, "heading": 180, "name": "Üsküdar"}
        ],
        "expected_results": {
            "all_points_visible": True,
            "vessel_icon_aligned": True,
            "route_line_accurate": True,
            "coordinate_precision": "< 1.0m"
        }
    }
    
    with open("map_test_data.json", 'w', encoding='utf-8') as f:
        json.dump(test_data, f, indent=2, ensure_ascii=False)
    
    print("✅ Tüm test senaryoları tamamlandı!")
    print("\nOluşturulan dosyalar:")
    print("- precision_test_results.json")
    print("- navigation_test_results.json") 
    print("- map_test_data.json")

if __name__ == "__main__":
    main() 