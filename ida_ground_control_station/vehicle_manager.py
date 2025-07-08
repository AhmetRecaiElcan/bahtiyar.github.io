import threading
from PyQt5.QtCore import QObject, pyqtSignal
from dronekit import connect, VehicleMode, Command, APIException
from pymavlink import mavutil
import serial.tools.list_ports

class VehicleManager(QObject):
    """
    Araç bağlantısını, telemetri alımını ve komut gönderimini yöneten sınıf.
    Tüm DroneKit işlemleri bu sınıfta toplanır.
    """
    connection_status_changed = pyqtSignal(bool, str)
    telemetry_updated = pyqtSignal(dict)
    log_message = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.vehicle = None
        self.is_connected = False

    def get_available_ports(self):
        """Mevcut seri portları döndürür."""
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect_to_vehicle(self, port, baud):
        """Araçla bağlantı kurmak için bir thread başlatır."""
        if self.is_connected:
            self.log_message.emit("Zaten bir bağlantı mevcut.")
            return
        
        self.log_message.emit(f"{port} üzerinden {baud} baud hızıyla bağlanılıyor...")
        thread = threading.Thread(target=self._connect_thread, args=(port, baud), daemon=True)
        thread.start()

    def _connect_thread(self, port, baud):
        """Arka planda bağlantı kurma işlemi."""
        try:
            self.vehicle = connect(port, baud=baud, wait_ready=True, heartbeat_timeout=30)
            self.is_connected = True
            self.connection_status_changed.emit(True, "Bağlantı başarılı!")
            self.log_message.emit(f"Araç bağlandı: {self.vehicle.version}")

            # Telemetri dinleyicilerini ekle
            self.vehicle.add_attribute_listener('attitude', self._telemetry_callback)
            self.vehicle.add_attribute_listener('location.global_relative_frame', self._telemetry_callback)
            self.vehicle.add_attribute_listener('gps_0', self._telemetry_callback)
            self.vehicle.add_attribute_listener('battery', self._telemetry_callback)
            self.vehicle.add_attribute_listener('mode', self._telemetry_callback)
            self.vehicle.add_attribute_listener('groundspeed', self._telemetry_callback)
            
        except APIException as e:
            self.log_message.emit(f"HATA: Bağlantı zaman aşımına uğradı. {e}")
            self.connection_status_changed.emit(False, "Bağlantı zaman aşımı.")
        except Exception as e:
            self.log_message.emit(f"HATA: Bağlantı kurulamadı. {e}")
            self.connection_status_changed.emit(False, f"Bağlantı hatası: {e}")

    def disconnect_from_vehicle(self):
        """Araçla bağlantıyı keser."""
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
            self.is_connected = False
            self.log_message.emit("Bağlantı sonlandırıldı.")
            self.connection_status_changed.emit(False, "Bağlantı kesildi.")

    def _telemetry_callback(self, vehicle, attr_name, value):
        """DroneKit'ten gelen herhangi bir telemetri güncellemesini işler."""
        if not self.is_connected:
            return

        telemetry_data = {
            "lat": self.vehicle.location.global_relative_frame.lat,
            "lon": self.vehicle.location.global_relative_frame.lon,
            "alt": self.vehicle.location.global_relative_frame.alt,
            "roll": self.vehicle.attitude.roll,
            "pitch": self.vehicle.attitude.pitch,
            "yaw": self.vehicle.attitude.yaw,
            "heading": self.vehicle.heading,
            "groundspeed": self.vehicle.groundspeed,
            "mode": self.vehicle.mode.name,
            "battery_level": self.vehicle.battery.level,
            "battery_voltage": self.vehicle.battery.voltage,
            "battery_current": self.vehicle.battery.current,
            "gps_fix": self.vehicle.gps_0.fix_type,
            "gps_sats": self.vehicle.gps_0.satellites_visible
        }
        self.telemetry_updated.emit(telemetry_data)
        
    def set_vehicle_mode(self, mode):
        """Aracın modunu değiştirir."""
        if not self.is_connected:
            self.log_message.emit("HATA: Mod değiştirmek için araç bağlantısı gerekli.")
            return

        def _set_mode():
            try:
                self.vehicle.mode = VehicleMode(mode)
                self.log_message.emit(f"Mod başarıyla değiştirildi: {mode}")
            except Exception as e:
                self.log_message.emit(f"HATA: Mod değiştirilemedi - {e}")
        
        threading.Thread(target=_set_mode, daemon=True).start()

    def upload_mission(self, waypoints):
        """Verilen waypoint listesini araca yükler."""
        if not self.is_connected:
            self.log_message.emit("HATA: Rota göndermek için araç bağlantısı gerekli.")
            return
        if not waypoints:
            self.log_message.emit("UYARI: Gönderilecek rota (waypoint) bulunmuyor.")
            return
            
        def _upload():
            try:
                cmds = self.vehicle.commands
                cmds.clear()
                for wp in waypoints:
                    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                                  wp["lat"], wp["lng"], 0)
                    cmds.add(cmd)
                
                cmds.upload()
                self.log_message.emit(f"Rota başarıyla araca yüklendi ({len(waypoints)} waypoint).")
            except Exception as e:
                self.log_message.emit(f"HATA: Rota gönderilemedi - {e}")

        threading.Thread(target=_upload, daemon=True).start() 