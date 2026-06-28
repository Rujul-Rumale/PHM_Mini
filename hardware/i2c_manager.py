import os
import time
import logging

log = logging.getLogger(__name__)

KNOWN_SENSOR_TYPES = {
    0x40: ("INA219", "Battery Monitor"),
    0x41: ("INA219", "Propulsion 1"),
    0x42: ("INA226", "Propulsion 2"),
    0x43: ("INA226", "Propulsion 3"),
    0x44: ("INA226", "Propulsion 4"),
    0x48: ("LM75", "ESC Temperature"),
    0x68: ("MPU6050", "IMU"),
    0x69: ("MPU6050", "IMU (alt)"),
    0x76: ("BME280", "Temperature/Pressure"),
    0x77: ("BME280", "Temperature/Pressure (alt)"),
    0x49: ("TMP117", "Temperature (alt)"),
}

I2C_BUS_PATHS = ["/dev/i2c-0", "/dev/i2c-1", "/dev/i2c-2", "/dev/i2c-3", "/dev/i2c-4", "/dev/i2c-5", "/dev/i2c-6", "/dev/i2c-7", "/dev/i2c-8", "/dev/i2c-9", "/dev/i2c-10"]


def _probe_smbus(bus_num: int, timeout_ms: float = 50) -> object:
    try:
        import smbus2
        bus = smbus2.SMBus(bus_num)
        bus._bus  # access to trigger init
        return bus
    except (ImportError, OSError, FileNotFoundError):
        return None


class I2CManager:
    def __init__(self):
        self._buses: dict[int, object] = {}
        self._detected_devices: dict[int, dict[int, str]] = {}
        self._enabled_features: dict[str, bool] = {}
        self._scan_results: dict = {}

    def scan_buses(self) -> list[int]:
        available = []
        for path in I2C_BUS_PATHS:
            if os.path.exists(path):
                bus_num = int(path.replace("/dev/i2c-", ""))
                bus = _probe_smbus(bus_num)
                if bus is not None:
                    self._buses[bus_num] = bus
                    available.append(bus_num)
        log.info("I2C buses available: %s", available)
        return available

    def scan_devices(self, bus_num: int) -> dict[int, str]:
        devices = {}
        bus = self._buses.get(bus_num)
        if bus is None:
            log.warning("Bus %d not available", bus_num)
            return devices
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                sensor_type = KNOWN_SENSOR_TYPES.get(addr, ("Unknown", f"Device 0x{addr:02X}"))
                label = f"{sensor_type[0]} {sensor_type[1]}"
                devices[addr] = label
                log.debug("I2C device found at 0x%02X: %s", addr, label)
            except OSError:
                pass
            except Exception as e:
                log.debug("I2C error at 0x%02X: %s", addr, e)
        self._detected_devices[bus_num] = devices
        return devices

    def scan_all_buses(self) -> dict[int, dict[int, str]]:
        buses = self.scan_buses()
        results = {}
        for b in buses:
            results[b] = self.scan_devices(b)
        return results

    def detect_sensors(self, hardware_config: dict) -> dict:
        self._scan_results = {"found": {}, "missing": {}, "features": {}}
        config_sensors = self._flatten_config(hardware_config)
        self.scan_all_buses()

        all_found = {}
        for bus_devices in self._detected_devices.values():
            all_found.update(bus_devices)

        for name, spec in config_sensors.items():
            addr_str = spec.get("address", "")
            try:
                addr = int(addr_str, 16) if addr_str else None
            except ValueError:
                addr = None
            if addr is not None and addr in all_found:
                self._scan_results["found"][name] = {"address": addr, "label": all_found[addr], "config": spec}
                self._scan_results["features"][name] = True
            else:
                self._scan_results["missing"][name] = {"address": addr_str, "config": spec}
                self._scan_results["features"][name] = False

        return self._scan_results

    def get_enabled_features(self) -> dict[str, bool]:
        return self._scan_results.get("features", {})

    def get_status_summary(self) -> str:
        found = self._scan_results.get("found", {})
        missing = self._scan_results.get("missing", {})
        lines = []
        if found:
            lines.append("Detected:")
            for name, info in found.items():
                lines.append(f"  0x{info['address']:02X} {info['label']}")
        if missing:
            lines.append("Missing:")
            for name, info in missing.items():
                lines.append(f"  {info['address'] or '??'} {name}")
        return "\n".join(lines)

    @staticmethod
    def _flatten_config(cfg: dict) -> dict:
        flat = {}
        if "sensors" in cfg:
            s = cfg["sensors"]
            if "battery_monitor" in s:
                flat["battery_monitor"] = s["battery_monitor"]
            if "imu" in s:
                flat["imu"] = s["imu"]
            if "temperature" in s:
                flat["temperature"] = s["temperature"]
            if "propulsion_units" in s:
                for pu in s["propulsion_units"]:
                    uid = pu.get("id", "?")
                    cs = pu.get("current_sensor", pu.get("esc_current_sensor"))
                    if cs:
                        flat[f"propulsion_{uid}_current"] = cs
                    esc = pu.get("esc_temperature")
                    if esc:
                        flat[f"propulsion_{uid}_esc_temp"] = esc
        return flat

    def close(self):
        for bus in self._buses.values():
            try:
                bus.close()
            except Exception:
                pass
        self._buses.clear()
