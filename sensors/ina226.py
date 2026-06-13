import time
import math
import logging
import random

from core.soc_estimator import soc_to_ocv

log = logging.getLogger(__name__)

REG_CONFIG      = 0x00
REG_SHUNT_V     = 0x01
REG_BUS_V       = 0x02
REG_POWER       = 0x03
REG_CURRENT     = 0x04
REG_CALIB       = 0x05
REG_MASK_EN     = 0x06
REG_ALERT_LIMIT = 0x07
REG_MFR_ID      = 0xFE
REG_DIE_ID      = 0xFF

CONFIG_DEFAULT = 0x4527


class INA226:
    def __init__(self, bus, address: int, shunt_ohms: float, max_amps: float = 20.0):
        self._bus = bus
        self._addr = address
        self._shunt = shunt_ohms
        self._max_amps = max_amps
        self._current_lsb = None
        self._calibrated = False

    def begin(self):
        self._write_reg(REG_CONFIG, CONFIG_DEFAULT)
        self._calibrate()
        self._calibrated = True
        log.debug(f"INA226 @ 0x{self._addr:02X} initialized, current_lsb={self._current_lsb:.6f}A")

    def _calibrate(self):
        self._current_lsb = self._max_amps / 32768.0
        calib = int(0.00512 / (self._current_lsb * self._shunt))
        self._write_reg(REG_CALIB, calib)

    def read_voltage(self) -> float:
        raw = self._read_reg_signed(REG_BUS_V)
        return raw * 1.25e-3

    def read_current(self) -> float:
        raw = self._read_reg_signed(REG_CURRENT)
        return raw * self._current_lsb

    def read_power(self) -> float:
        raw = self._read_reg(REG_POWER)
        return raw * 25 * self._current_lsb

    def _write_reg(self, reg: int, value: int):
        data = [(value >> 8) & 0xFF, value & 0xFF]
        self._bus.write_i2c_block_data(self._addr, reg, data)

    def _read_reg(self, reg: int) -> int:
        data = self._bus.read_i2c_block_data(self._addr, reg, 2)
        return (data[0] << 8) | data[1]

    def _read_reg_signed(self, reg: int) -> int:
        raw = self._read_reg(reg)
        if raw > 32767:
            raw -= 65536
        return raw


class MockINA226:
    def __init__(self, base_voltage: float, base_current: float, label: str = "mock",
                 cell_count: int = 4, r_int: float = 0.015,
                 no_load_i: float = None, motor_imax: float = None):
        self._vmax = base_voltage
        self._imax = base_current
        self._cell_count = cell_count
        self._label = label
        self._throttle = 0.0

        self._i_noload = (no_load_i if no_load_i is not None else
                          (base_current * 0.03 if "motor" in label else 0.5))
        self._motor_imax = motor_imax if motor_imax is not None else base_current
        self._is_propulsion = "battery" not in label
        self._r_int = r_int
        self._motor_currents = None

        self._soc = 100.0

        self.prop_attached = True
        self.friction_level = 0.0
        self.fault_mode = None
        self.fault_start_time = None

        self._ambient = 25.0

    def begin(self):
        log.info(f"[SIM] MockINA226 '{self._label}' ready")

    def set_throttle(self, t: float):
        self._throttle = max(0.0, min(1.0, t))

    def set_soc(self, soc_percent: float):
        self._soc = max(0.0, min(100.0, soc_percent))

    def set_motor_currents(self, currents: list):
        self._motor_currents = currents

    def set_ambient(self, temp: float):
        self._ambient = temp

    def _elapsed(self) -> float:
        if self.fault_start_time is None:
            return 0.0
        return time.time() - self.fault_start_time

    def _baseline_current(self) -> float:
        t = self._throttle
        return self._i_noload + (self._imax - self._i_noload) * (t ** 1.5)

    def _load_current(self) -> float:
        if self._motor_currents is not None:
            valid = [c for c in self._motor_currents if c is not None]
            return self._i_noload + sum(valid)
        i = self._baseline_current()
        if not self.prop_attached:
            i = self._i_noload
        if self.friction_level > 0:
            i += self._baseline_current() * self.friction_level
        if self.fault_mode == "bearing_wear":
            i *= 1.30
        if self.fault_mode == "prop_damage":
            i *= 1.10
        return i

    def read_current(self) -> float:
        i = self._load_current()
        if self.fault_mode == 'battery_aging' and not self._is_propulsion:
            i = i * 1.05
        return round(max(i, 0.0), 4)

    def read_voltage(self) -> float:
        if not self._is_propulsion:
            cell_soc = max(0.0, min(100.0, self._soc))
            cell_ocv = soc_to_ocv(cell_soc)
            v_pack = cell_ocv * self._cell_count
            load_i = self._load_current()
            r = self._r_int
            if self.fault_mode == 'battery_aging':
                elapsed = self._elapsed()
                r += min(0.06, elapsed / 60.0 * 0.06)
            if self.fault_mode == 'battery_old':
                r = 0.060
            v = v_pack - load_i * r
        else:
            v = self._vmax
        return round(v, 4)

    def read_power(self) -> float:
        return round(self.read_voltage() * self.read_current(), 3)

    def read_temperature(self) -> float:
        if self._is_propulsion:
            i = self._load_current()
            if not self.prop_attached:
                return round(self._ambient + 1.0, 1)
            temp_rise = i * 3.0
            if self.friction_level > 0:
                temp_rise *= (1.0 + self.friction_level * 0.5)
            if self.fault_mode == "friction":
                elapsed = self._elapsed()
                temp_rise *= (1.0 + min(0.5, elapsed / 60.0 * 0.5))
            if self.fault_mode == "esc_degrade":
                temp_rise *= 1.50
            if self.fault_mode == "bearing_wear":
                temp_rise *= 1.20
            return round(self._ambient + temp_rise, 1)
        else:
            i = self._load_current()
            temp = self._ambient + i * 0.5
            if self.fault_mode == 'battery_aging':
                temp += 5.0
            return round(temp, 1)

    def read_vibration(self, n_samples: int = 100) -> dict:
        if not self._is_propulsion:
            return {"rms": 0.0, "kurtosis": 0.0, "samples": []}

        base_noise = 0.1
        throttle_vib = self._throttle * 0.3
        fault_vib = 0.0

        if not self.prop_attached:
            fault_vib += 0.6
        if self.friction_level > 0:
            fault_vib += self.friction_level * 0.8

        if self.fault_mode == "prop_damage":
            fault_vib += 3.0 * self._throttle
        if self.fault_mode == "bearing_wear":
            fault_vib += 0.5

        rms = base_noise + throttle_vib + fault_vib

        samples = []
        if rms > 0:
            for _ in range(n_samples):
                v = random.gauss(0, rms / math.sqrt(2))
                samples.append(v)
        else:
            samples = [0.0] * n_samples

        mean = sum(samples) / len(samples)
        var = sum((s - mean) ** 2 for s in samples) / len(samples)
        std = math.sqrt(var)
        kurt = (sum(((s - mean) / max(std, 1e-6)) ** 4 for s in samples) / len(samples)) - 3.0 if std > 0 else 0.0

        if self.fault_mode == "friction":
            kurt = max(kurt, 6.0)

        return {
            "rms": round(rms, 4),
            "kurtosis": round(kurt, 3),
            "samples": [round(s, 6) for s in samples],
        }
