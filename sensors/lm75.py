import time
import logging

log = logging.getLogger(__name__)

REG_TEMP = 0x00


class LM75:
    """LM75 I2C Temperature Sensor Driver."""
    def __init__(self, bus, address: int = 0x48):
        self._bus = bus
        self._addr = address

    def begin(self):
        try:
            # Probe device by reading temperature
            self.read_temperature()
            log.info(f"LM75 temperature sensor online at 0x{self._addr:02X}")
        except Exception as e:
            log.error(f"Failed to communicate with LM75 at 0x{self._addr:02X}: {e}")
            raise

    def read_temperature(self) -> float:
        # Read 2 bytes from register 0x00
        data = self._bus.read_i2c_block_data(self._addr, REG_TEMP, 2)
        # Combine bytes. Temp is in MSB and the most significant 3 bits of LSB (11-bit total)
        raw = (data[0] << 8 | data[1]) >> 5
        # Handle 11-bit signed twos-complement
        if raw > 1023:
            raw -= 2048
        # Standard LM75 step is 0.125 degrees C
        return round(raw * 0.125, 2)


class MockLM75:
    """
    Mock counterpart for LM75 implementing a first-order thermal model:
    dT/dt = (P_heat - P_cool) / C_thermal
    where:
      P_heat = current^2 * R_esc
      P_cool = (T - T_ambient) / R_thermal
    """
    def __init__(self, current_sensor, ambient: float = 25.0,
                 r_esc: float = 0.05, r_thermal: float = 10.0, c_thermal: float = 5.0):
        self._current_sensor = current_sensor
        self._ambient = ambient
        self._r_esc = r_esc
        self._r_thermal = r_thermal
        self._c_thermal = c_thermal
        
        self._temp = ambient
        self._last_t = time.time()
        self.fault_mode = None

    def begin(self):
        self._last_t = time.time()
        self._temp = self._ambient
        log.info("[SIM] MockLM75 ready with first-order thermal model")

    def set_fault_mode(self, mode: str):
        self.fault_mode = mode

    def read_temperature(self) -> float:
        now = time.time()
        dt = now - self._last_t
        self._last_t = now
        
        # Clamp dt to prevent mathematical explosion if loop paused
        dt = min(max(dt, 0.0), 2.0)
        
        # Read current from motor supply sensor
        current = 0.0
        if self._current_sensor:
            try:
                current = self._current_sensor.read_current()
            except Exception:
                current = 0.0
                
        # Heat generation parameters
        r_esc = self._r_esc
        if self.fault_mode == "esc-degrade" or (self._current_sensor and getattr(self._current_sensor, "fault_mode", None) == "esc_degrade"):
            # Degraded ESC has higher internal resistance/heating
            r_esc *= 1.8
        elif self.fault_mode == "friction" or (self._current_sensor and getattr(self._current_sensor, "friction_level", 0) > 0):
            r_esc *= 1.2
            
        p_heat = (current ** 2) * r_esc
        p_cool = (self._temp - self._ambient) / self._r_thermal
        
        # First-order ODE integration (Euler method)
        dT = (p_heat - p_cool) / self._c_thermal
        self._temp += dT * dt
        
        # Keep temp within sensible limits
        self._temp = max(self._ambient - 5.0, min(self._temp, 150.0))
        return round(self._temp, 2)
