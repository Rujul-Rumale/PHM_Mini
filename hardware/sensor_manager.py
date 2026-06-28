import json
import time
import logging

from hardware.i2c_manager import I2CManager
from hardware.platform import is_raspberry_pi
from sensors import make_current_sensor, make_temperature_sensor

log = logging.getLogger(__name__)


class SensorManager:
    """Manages physical I2C and simulated sensors dynamically based on config."""
    def __init__(self, config_path: str = "config/hardware.json"):
        self._config_path = config_path
        self._config = self._load_config()
        self._i2c = I2CManager()
        self._battery = None
        self._current_sensors: list = []
        self._imu = None
        self._temperature_sensors: list = []
        self._scan_results = {}
        self._enabled_features = {}
        self._mode = "simulate"

    def _load_config(self) -> dict:
        try:
            with open(self._config_path) as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            log.warning("Could not load %s: %s. Using defaults.", self._config_path, e)
            return {"i2c_bus": 1, "sensors": {}}

    def init_hardware(self) -> dict:
        self._mode = "hardware"
        self._scan_results = self._i2c.detect_sensors(self._config)
        self._enabled_features = self._scan_results.get("features", {})
        log.info("Sensor scan complete. Found: %d, Missing: %d",
                 len(self._scan_results.get("found", {})),
                 len(self._scan_results.get("missing", {})))

        if not is_raspberry_pi():
            log.warning("Not running on Raspberry Pi — sensor init may fail")

        self._init_battery()
        self._init_current_sensors()
        self._init_imu()
        self._init_temperature()
        return self.get_status()

    def init_simulate(self, profile: dict = None):
        self._mode = "simulate"
        self._enabled_features = {
            "battery_monitor": True,
            "propulsion_current": True,
            "vibration": True,
            "temperature": True,
        }
        n_units = len(profile.get("propulsion_units", [])) if profile else 1
        bc = profile.get("battery", {}) if profile else {}
        sc = profile.get("simulation", {}) if profile else {}
        cell_count = bc.get("cell_count", 3)
        nominal_v = bc.get("nominal_voltage", 11.1)
        motor_imax = sc.get("motor_imax", 15.0)
        motor_i0 = sc.get("motor_i0", 0.45)
        batt_imax = motor_imax * max(n_units, 1) + 2.0

        # Check config type for battery monitor
        s_cfg = self._config.get("sensors", {})
        batt_type = s_cfg.get("battery_monitor", {}).get("type", "INA219")
        
        if batt_type == "INA219":
            from sensors.ina219 import MockINA219
            self._battery = MockINA219(
                base_voltage=nominal_v, base_current=batt_imax,
                label="battery", cell_count=cell_count,
                r_int=sc.get("r_int", 0.015),
            )
        else:
            from sensors.ina226 import MockINA226
            self._battery = MockINA226(
                base_voltage=nominal_v, base_current=batt_imax,
                label="battery", cell_count=cell_count,
                r_int=sc.get("r_int", 0.015),
            )
        self._battery.begin()

        # Check config type for propulsion current
        prop_units = s_cfg.get("propulsion_units", [])
        
        # Check if single INA configuration is used
        motor_addrs = self._config.get("motors", {}).get("i2c_addresses", [])
        if motor_addrs:
            for i in range(n_units):
                ptype = "INA219"
                if i < len(prop_units):
                    ptype = prop_units[i].get("current_sensor", prop_units[i].get("esc_current_sensor", {})).get("type", "INA219")
                    
                if ptype == "INA219":
                    from sensors.ina219 import MockINA219
                    s = MockINA219(
                        base_voltage=nominal_v * 0.95, base_current=motor_imax,
                        label=f"motor_{i}", no_load_i=motor_i0, motor_imax=motor_imax,
                    )
                else:
                    from sensors.ina226 import MockINA226
                    s = MockINA226(
                        base_voltage=nominal_v * 0.95, base_current=motor_imax,
                        label=f"motor_{i}", no_load_i=motor_i0, motor_imax=motor_imax,
                    )
                s.begin()
                self._current_sensors.append(s)

        # Mock temperature sensor (first ESC or Battery fallback)
        temp_ref_sensor = self._current_sensors[0] if self._current_sensors else self._battery
        if temp_ref_sensor:
            from sensors.lm75 import MockLM75
            t_sensor = MockLM75(temp_ref_sensor)
            t_sensor.begin()
            self._temperature_sensors.append(t_sensor)

        # Mock IMU
        from sensors.mpu6050 import MockMPU6050
        self._imu = MockMPU6050()
        self._imu.begin()

        log.info("Simulation sensors initialized for %d units", n_units)
        return self.get_status()

    def _init_battery(self):
        found = self._scan_results.get("found", {})
        if "battery_monitor" in found:
            addr = found["battery_monitor"]["address"]
            cfg = found["battery_monitor"]["config"]
            sensor_type = cfg.get("type", "INA219")
            shunt = cfg.get("shunt_ohms", cfg.get("shunt", 0.1))
            max_amps = cfg.get("max_amps", 16.0)
            try:
                import smbus2
                bus = smbus2.SMBus(int(self._config.get("i2c_bus", 1)))
                self._battery = make_current_sensor(sensor_type, bus, addr, shunt, max_amps)
                self._battery.begin()
                log.info(f"Battery monitor ({sensor_type}) online at 0x{addr:02X}")
            except Exception as e:
                log.error("Failed to init battery monitor: %s", e)
                self._enabled_features["battery_monitor"] = False
        else:
            log.warning("Battery monitor not detected — battery monitoring disabled")
            self._enabled_features["battery_monitor"] = False

    def _init_current_sensors(self):
        import smbus2
        found = self._scan_results.get("found", {})
        bus = smbus2.SMBus(int(self._config.get("i2c_bus", 1)))
        for name, info in found.items():
            if name.startswith("propulsion_") and "current" in name:
                addr = info["address"]
                cfg = info["config"]
                sensor_type = cfg.get("type", "INA219")
                shunt = cfg.get("shunt_ohms", cfg.get("shunt", 0.1))
                max_amps = cfg.get("max_amps", 16.0)
                try:
                    s = make_current_sensor(sensor_type, bus, addr, shunt, max_amps)
                    s.begin()
                    self._current_sensors.append(s)
                    log.info(f"Current sensor ({sensor_type}) at 0x{addr:02X} online")
                except Exception as e:
                    log.error("Failed to init current sensor at 0x%02X: %s", addr, e)
        if not self._current_sensors:
            self._enabled_features["propulsion_current"] = False
            log.warning("No propulsion current sensors detected")

    def _init_imu(self):
        found = self._scan_results.get("found", {})
        if "imu" in found:
            addr = found["imu"]["address"]
            sensor_type = found["imu"].get("label", "MPU6050 IMU")
            try:
                if "MPU6050" in sensor_type:
                    from sensors.mpu6050 import MPU6050
                    import smbus2
                    bus = smbus2.SMBus(int(self._config.get("i2c_bus", 1)))
                    self._imu = MPU6050(bus, addr)
                    self._imu.begin()
                    log.info("IMU online at 0x%02X", addr)
                    self._enabled_features["vibration"] = True
                else:
                    log.warning("Unsupported IMU type: %s", sensor_type)
                    self._enabled_features["vibration"] = False
            except Exception as e:
                log.error("Failed to init IMU: %s", e)
                self._enabled_features["vibration"] = False
        else:
            log.warning("IMU not detected — vibration analysis disabled")
            self._enabled_features["vibration"] = False

    def _init_temperature(self):
        import smbus2
        found = self._scan_results.get("found", {})
        bus = smbus2.SMBus(int(self._config.get("i2c_bus", 1)))
        for name, info in found.items():
            if "esc_temp" in name:
                addr = info["address"]
                cfg = info["config"]
                sensor_type = cfg.get("type", "LM75")
                try:
                    s = make_temperature_sensor(sensor_type, bus, addr)
                    s.begin()
                    self._temperature_sensors.append(s)
                    log.info(f"ESC Temperature sensor ({sensor_type}) at 0x{addr:02X} online")
                except Exception as e:
                    log.error(f"Failed to init temperature sensor at 0x{addr:02X}: {e}")
        self._enabled_features["temperature"] = len(self._temperature_sensors) > 0

    def get_battery(self):
        return self._battery

    def get_current_sensors(self):
        return self._current_sensors

    def get_imu(self):
        return self._imu

    def get_temperature_sensors(self):
        return self._temperature_sensors

    def get_enabled_features(self) -> dict:
        return dict(self._enabled_features)

    def get_mode(self) -> str:
        return self._mode

    def get_status(self) -> dict:
        return {
            "mode": self._mode,
            "battery": self._battery is not None,
            "current_sensors": len(self._current_sensors),
            "imu": self._imu is not None,
            "temperature": len(self._temperature_sensors),
            "enabled_features": dict(self._enabled_features),
            "scan_results": self._scan_results,
        }

    def close(self):
        self._i2c.close()
        log.info("Sensor manager closed")
