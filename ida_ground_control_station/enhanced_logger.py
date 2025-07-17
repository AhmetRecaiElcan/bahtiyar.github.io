#!/usr/bin/env python3
"""
YKİ Gelişmiş Logging Sistemi
Bu modül, koordinat doğruluğu, harita render, telemetri ve sistem performansı
için kapsamlı logging ve debug özellikleri sağlar.
"""

import logging
import json
import time
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
from enum import Enum
import threading
from collections import deque
import traceback

class LogLevel(Enum):
    """Log seviyeleri"""
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"

class LogCategory(Enum):
    """Log kategorileri"""
    COORDINATE = "COORDINATE"
    MAP_RENDER = "MAP_RENDER"
    TELEMETRY = "TELEMETRY"
    VEHICLE_CONTROL = "VEHICLE_CONTROL"
    COMMUNICATION = "COMMUNICATION"
    PERFORMANCE = "PERFORMANCE"
    USER_ACTION = "USER_ACTION"
    SYSTEM = "SYSTEM"

class EnhancedLogger:
    """Gelişmiş YKİ Logger Sistemi"""
    
    def __init__(self, log_dir: str = "logs", max_log_files: int = 10):
        self.log_dir = log_dir
        self.max_log_files = max_log_files
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Log dosyaları
        self.ensure_log_directory()
        
        # Ana logger
        self.main_logger = self.setup_main_logger()
        
        # Kategorik loggerlar
        self.category_loggers = {}
        for category in LogCategory:
            self.category_loggers[category] = self.setup_category_logger(category)
        
        # Memory buffer (gerçek zamanlı görüntüleme için)
        self.memory_buffer = deque(maxlen=1000)
        self.buffer_lock = threading.Lock()
        
        # Performans metrikleri
        self.performance_metrics = {
            "coordinate_updates": 0,
            "map_renders": 0,
            "telemetry_packets": 0,
            "errors": 0,
            "session_start": time.time()
        }
        
        # JSON log dosyası (makine okumaya uygun)
        self.json_log_file = os.path.join(self.log_dir, f"session_{self.session_id}.json")
        self.json_logs = []
        
        self.log_system_start()
    
    def ensure_log_directory(self):
        """Log dizinini oluştur"""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Eski log dosyalarını temizle
        self.cleanup_old_logs()
    
    def cleanup_old_logs(self):
        """Eski log dosyalarını temizle"""
        try:
            log_files = [f for f in os.listdir(self.log_dir) if f.endswith('.log')]
            log_files.sort(key=lambda x: os.path.getctime(os.path.join(self.log_dir, x)))
            
            while len(log_files) > self.max_log_files:
                old_file = log_files.pop(0)
                os.remove(os.path.join(self.log_dir, old_file))
        except Exception as e:
            print(f"Log temizleme hatası: {e}")
    
    def setup_main_logger(self) -> logging.Logger:
        """Ana logger'ı kurar"""
        logger = logging.getLogger(f"YKI_Main_{self.session_id}")
        logger.setLevel(logging.DEBUG)
        
        # Dosya handler
        file_handler = logging.FileHandler(
            os.path.join(self.log_dir, f"main_{self.session_id}.log"),
            encoding='utf-8'
        )
        file_handler.setLevel(logging.DEBUG)
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # Formatter
        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def setup_category_logger(self, category: LogCategory) -> logging.Logger:
        """Kategori bazlı logger kurar"""
        logger = logging.getLogger(f"YKI_{category.value}_{self.session_id}")
        logger.setLevel(logging.DEBUG)
        
        # Kategori-özel dosya
        file_handler = logging.FileHandler(
            os.path.join(self.log_dir, f"{category.value.lower()}_{self.session_id}.log"),
            encoding='utf-8'
        )
        file_handler.setLevel(logging.DEBUG)
        
        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        return logger
    
    def log(self, level: LogLevel, category: LogCategory, message: str, 
            data: Optional[Dict[str, Any]] = None, exception: Optional[Exception] = None):
        """Ana log fonksiyonu"""
        
        timestamp = time.time()
        log_entry = {
            "timestamp": timestamp,
            "datetime": datetime.fromtimestamp(timestamp).isoformat(),
            "level": level.value,
            "category": category.value,
            "message": message,
            "data": data or {},
            "session_id": self.session_id
        }
        
        if exception:
            log_entry["exception"] = {
                "type": type(exception).__name__,
                "message": str(exception),
                "traceback": traceback.format_exc()
            }
        
        # Memory buffer'a ekle
        with self.buffer_lock:
            self.memory_buffer.append(log_entry)
        
        # JSON log'a ekle
        self.json_logs.append(log_entry)
        
        # Kategori logger'ına yaz
        category_logger = self.category_loggers[category]
        log_message = message
        if data:
            log_message += f" | Data: {json.dumps(data, ensure_ascii=False)}"
        if exception:
            log_message += f" | Exception: {exception}"
        
        getattr(category_logger, level.value.lower())(log_message)
        
        # Ana logger'a da yaz
        getattr(self.main_logger, level.value.lower())(f"[{category.value}] {log_message}")
        
        # Performans metriklerini güncelle
        self.update_performance_metrics(category, level)
    
    def update_performance_metrics(self, category: LogCategory, level: LogLevel):
        """Performans metriklerini güncelle"""
        if category == LogCategory.COORDINATE:
            self.performance_metrics["coordinate_updates"] += 1
        elif category == LogCategory.MAP_RENDER:
            self.performance_metrics["map_renders"] += 1
        elif category == LogCategory.TELEMETRY:
            self.performance_metrics["telemetry_packets"] += 1
        
        if level in [LogLevel.ERROR, LogLevel.CRITICAL]:
            self.performance_metrics["errors"] += 1
    
    def log_coordinate_update(self, lat: float, lon: float, heading: float, 
                            source: str = "unknown", accuracy: Optional[float] = None):
        """Koordinat güncellemesi log'u"""
        data = {
            "lat": lat,
            "lon": lon,
            "heading": heading,
            "source": source,
            "accuracy_meters": accuracy
        }
        
        message = f"Koordinat güncellendi: {lat:.6f}, {lon:.6f}, {heading:.1f}° ({source})"
        self.log(LogLevel.INFO, LogCategory.COORDINATE, message, data)
    
    def log_coordinate_precision_test(self, lat: float, lon: float, 
                                    precision_result: Dict[str, Any]):
        """Koordinat precision test sonucu"""
        data = {
            "lat": lat,
            "lon": lon,
            "precision_analysis": precision_result
        }
        
        accuracy = precision_result.get("overall_accuracy_meters", "unknown")
        sufficient = precision_result.get("sufficient_for_marine", False)
        
        message = f"Precision testi: {lat:.6f}, {lon:.6f} → {accuracy}m ({'✅' if sufficient else '❌'})"
        level = LogLevel.INFO if sufficient else LogLevel.WARNING
        
        self.log(level, LogCategory.COORDINATE, message, data)
    
    def log_map_render_event(self, event_type: str, element: str, 
                           coordinates: Optional[Dict[str, float]] = None,
                           properties: Optional[Dict[str, Any]] = None):
        """Harita render eventi"""
        data = {
            "event_type": event_type,
            "element": element,
            "coordinates": coordinates,
            "properties": properties or {}
        }
        
        message = f"Harita eventi: {event_type} - {element}"
        if coordinates:
            message += f" @ {coordinates.get('lat', '?'):.6f}, {coordinates.get('lon', '?'):.6f}"
        
        self.log(LogLevel.DEBUG, LogCategory.MAP_RENDER, message, data)
    
    def log_telemetry_data(self, telemetry: Dict[str, Any], validation_result: Optional[Dict] = None):
        """Telemetri verisi log'u"""
        data = {
            "telemetry": telemetry,
            "validation": validation_result
        }
        
        speed = telemetry.get("speed", "N/A")
        heading = telemetry.get("heading", "N/A")
        position = telemetry.get("position", {})
        
        message = f"Telemetri: Hız={speed}, Heading={heading}, Pos=({position.get('lat', '?'):.6f}, {position.get('lon', '?'):.6f})"
        
        # Validasyon hatası varsa warning
        level = LogLevel.WARNING if validation_result and not validation_result.get("valid", True) else LogLevel.DEBUG
        
        self.log(level, LogCategory.TELEMETRY, message, data)
    
    def log_vehicle_command(self, command: str, parameters: Dict[str, Any], 
                          success: bool, response: Optional[str] = None):
        """Araç komutu log'u"""
        data = {
            "command": command,
            "parameters": parameters,
            "success": success,
            "response": response
        }
        
        message = f"Araç komutu: {command} ({'✅' if success else '❌'})"
        if response:
            message += f" → {response}"
        
        level = LogLevel.INFO if success else LogLevel.ERROR
        self.log(level, LogCategory.VEHICLE_CONTROL, message, data)
    
    def log_communication_event(self, event_type: str, direction: str, 
                              data_size: int, success: bool, latency: Optional[float] = None):
        """İletişim eventi log'u"""
        data = {
            "event_type": event_type,
            "direction": direction,  # "sent" veya "received"
            "data_size_bytes": data_size,
            "success": success,
            "latency_ms": latency
        }
        
        message = f"İletişim: {event_type} {direction} ({data_size} bytes)"
        if latency:
            message += f" - {latency:.1f}ms"
        if not success:
            message += " ❌"
        
        level = LogLevel.DEBUG if success else LogLevel.WARNING
        self.log(level, LogCategory.COMMUNICATION, message, data)
    
    def log_performance_metric(self, metric_name: str, value: float, unit: str = "", 
                             threshold: Optional[float] = None):
        """Performans metriği log'u"""
        data = {
            "metric_name": metric_name,
            "value": value,
            "unit": unit,
            "threshold": threshold,
            "exceeds_threshold": threshold and value > threshold
        }
        
        message = f"Performans: {metric_name} = {value}{unit}"
        if threshold and value > threshold:
            message += f" (⚠️ Eşik aşıldı: {threshold}{unit})"
        
        level = LogLevel.WARNING if (threshold and value > threshold) else LogLevel.DEBUG
        self.log(level, LogCategory.PERFORMANCE, message, data)
    
    def log_user_action(self, action: str, details: Optional[Dict[str, Any]] = None):
        """Kullanıcı eylemi log'u"""
        data = {
            "action": action,
            "details": details or {}
        }
        
        message = f"Kullanıcı eylemi: {action}"
        self.log(LogLevel.INFO, LogCategory.USER_ACTION, message, data)
    
    def log_system_error(self, error_type: str, error_message: str, 
                        exception: Optional[Exception] = None, 
                        system_state: Optional[Dict[str, Any]] = None):
        """Sistem hatası log'u"""
        data = {
            "error_type": error_type,
            "error_message": error_message,
            "system_state": system_state or {}
        }
        
        message = f"Sistem hatası [{error_type}]: {error_message}"
        self.log(LogLevel.ERROR, LogCategory.SYSTEM, message, data, exception)
    
    def log_system_start(self):
        """Sistem başlangıç log'u"""
        data = {
            "session_id": self.session_id,
            "log_directory": self.log_dir,
            "system_info": {
                "os": os.name,
                "python_version": ".".join(map(str, __import__("sys").version_info[:3]))
            }
        }
        
        self.log(LogLevel.INFO, LogCategory.SYSTEM, f"YKİ sistemi başlatıldı (Session: {self.session_id})", data)
    
    def get_recent_logs(self, count: int = 50, category: Optional[LogCategory] = None) -> List[Dict[str, Any]]:
        """Son log kayıtlarını getir"""
        with self.buffer_lock:
            recent_logs = list(self.memory_buffer)
        
        if category:
            recent_logs = [log for log in recent_logs if log["category"] == category.value]
        
        return recent_logs[-count:]
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Performans özetini getir"""
        current_time = time.time()
        session_duration = current_time - self.performance_metrics["session_start"]
        
        return {
            "session_id": self.session_id,
            "session_duration_seconds": session_duration,
            "metrics": self.performance_metrics.copy(),
            "rates": {
                "coordinate_updates_per_sec": self.performance_metrics["coordinate_updates"] / session_duration,
                "telemetry_packets_per_sec": self.performance_metrics["telemetry_packets"] / session_duration,
                "error_rate": self.performance_metrics["errors"] / max(1, self.performance_metrics["telemetry_packets"])
            }
        }
    
    def save_session_logs(self):
        """Session log'larını JSON dosyasına kaydet"""
        session_data = {
            "session_id": self.session_id,
            "session_start": self.performance_metrics["session_start"],
            "session_end": time.time(),
            "performance_summary": self.get_performance_summary(),
            "logs": self.json_logs
        }
        
        try:
            with open(self.json_log_file, 'w', encoding='utf-8') as f:
                json.dump(session_data, f, indent=2, ensure_ascii=False)
            
            self.main_logger.info(f"Session logları kaydedildi: {self.json_log_file}")
        except Exception as e:
            self.main_logger.error(f"Session log kaydetme hatası: {e}")
    
    def __del__(self):
        """Destructor - session loglarını kaydet"""
        try:
            self.save_session_logs()
        except:
            pass

# Global logger instance
_logger_instance = None

def get_logger() -> EnhancedLogger:
    """Global logger instance'ını getir"""
    global _logger_instance
    if _logger_instance is None:
        _logger_instance = EnhancedLogger()
    return _logger_instance

# Convenience functions
def log_coordinate_update(lat: float, lon: float, heading: float, source: str = "unknown"):
    """Koordinat güncelleme log'u (kısa fonksiyon)"""
    get_logger().log_coordinate_update(lat, lon, heading, source)

def log_map_event(event_type: str, element: str, coordinates: Optional[Dict] = None):
    """Harita eventi log'u (kısa fonksiyon)"""
    get_logger().log_map_render_event(event_type, element, coordinates)

def log_telemetry(telemetry: Dict[str, Any]):
    """Telemetri log'u (kısa fonksiyon)"""
    get_logger().log_telemetry_data(telemetry)

def log_error(error_type: str, message: str, exception: Optional[Exception] = None):
    """Hata log'u (kısa fonksiyon)"""
    get_logger().log_system_error(error_type, message, exception)

def log_user_action(action: str, details: Optional[Dict] = None):
    """Kullanıcı eylemi log'u (kısa fonksiyon)"""
    get_logger().log_user_action(action, details)

# Test fonksiyonu
if __name__ == "__main__":
    # Test logger
    logger = EnhancedLogger()
    
    # Test logları
    logger.log_coordinate_update(41.025631, 28.974298, 90, "GPS", 0.5)
    logger.log_map_render_event("marker_update", "vehicle", {"lat": 41.025631, "lon": 28.974298})
    
    test_telemetry = {
        "speed": 5.2,
        "heading": 90,
        "position": {"lat": 41.025631, "lon": 28.974298},
        "battery": {"level": 85}
    }
    logger.log_telemetry_data(test_telemetry)
    
    logger.log_vehicle_command("SET_MODE", {"mode": "AUTO"}, True, "Mode changed successfully")
    logger.log_user_action("waypoint_added", {"lat": 41.025631, "lon": 28.974298})
    
    # Performans özeti
    summary = logger.get_performance_summary()
    print("\nPerformans Özeti:")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    
    # Son loglar
    recent_logs = logger.get_recent_logs(5)
    print(f"\nSon {len(recent_logs)} log:")
    for log in recent_logs:
        print(f"[{log['level']}] {log['category']}: {log['message']}")
    
    print(f"\nLog dosyaları {logger.log_dir} dizininde oluşturuldu.") 