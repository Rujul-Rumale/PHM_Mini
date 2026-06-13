import math
import time
import random
import logging

log = logging.getLogger(__name__)

REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

AFS_SEL_2G = 0
AFS_SEL_4G = 1
AFS_SEL_8G = 2
AFS_SEL_16G = 3

ACCEL_SCALE = {AFS_SEL_2G: 16384.0, AFS_SEL_4G: 8192.0, AFS_SEL_8G: 4096.0, AFS_SEL_16G: 2048.0}


class MPU6050:
    def __init__(self, bus, address: int = 0x68, accel_range: int = AFS_SEL_2G):
        self._bus = bus
        self._addr = address
        self._accel_range = accel_range
        self._scale = ACCEL_SCALE[accel_range]
        self._initialized = False

    def begin(self):
        try:
            who = self._read_byte(REG_WHO_AM_I)
            if who != 0x68:
                log.warning("MPU6050 WHO_AM_I mismatch: 0x%02X (expected 0x68)", who)
        except Exception as e:
            raise RuntimeError(f"MPU6050 not responding at 0x{self._addr:02X}: {e}")

        self._write_byte(REG_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self._write_byte(REG_ACCEL_CONFIG, self._accel_range << 3)
        self._write_byte(REG_CONFIG, 0x00)
        self._initialized = True
        log.info("MPU6050 initialized at 0x%02X", self._addr)

    def read_accel(self) -> dict:
        data = self._read_bytes(REG_ACCEL_XOUT_H, 6)
        ax = self._twos_complement(data[0] << 8 | data[1], 16) / self._scale
        ay = self._twos_complement(data[2] << 8 | data[3], 16) / self._scale
        az = self._twos_complement(data[4] << 8 | data[5], 16) / self._scale
        return {"x": ax, "y": ay, "z": az}

    def read_vibration(self, n_samples: int = 100) -> dict:
        samples_x, samples_y, samples_z = [], [], []
        for _ in range(n_samples):
            a = self.read_accel()
            samples_x.append(a["x"])
            samples_y.append(a["y"])
            samples_z.append(a["z"])
            time.sleep(0.001)

        rms_x = math.sqrt(sum(v * v for v in samples_x) / len(samples_x))
        rms_y = math.sqrt(sum(v * v for v in samples_y) / len(samples_y))
        rms_z = math.sqrt(sum(v * v for v in samples_z) / len(samples_z))
        rms_total = math.sqrt(rms_x ** 2 + rms_y ** 2 + rms_z ** 2)

        mean = sum(samples_z) / len(samples_z)
        var = sum((v - mean) ** 2 for v in samples_z) / len(samples_z)
        std = math.sqrt(var) if var > 0 else 1e-6
        kurt = (sum(((v - mean) / std) ** 4 for v in samples_z) / len(samples_z)) - 3.0

        return {"rms": round(rms_total, 4), "kurtosis": round(kurt, 3), "samples": samples_z}

    def _write_byte(self, reg: int, value: int):
        self._bus.write_byte_data(self._addr, reg, value)

    def _read_byte(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)

    def _read_bytes(self, reg: int, length: int) -> list:
        return self._bus.read_i2c_block_data(self._addr, reg, length)

    @staticmethod
    def _twos_complement(val: int, bits: int) -> int:
        if val >= (1 << (bits - 1)):
            val -= (1 << bits)
        return val


class MockMPU6050:
    def __init__(self):
        self._base_noise = 0.1
        self._initialized = False

    def begin(self):
        self._initialized = True
        log.info("[SIM] MockMPU6050 ready")

    def read_accel(self) -> dict:
        return {
            "x": random.gauss(0, self._base_noise),
            "y": random.gauss(0, self._base_noise),
            "z": 1.0 + random.gauss(0, self._base_noise),
        }

    def read_vibration(self, n_samples: int = 100) -> dict:
        samples = [random.gauss(0, self._base_noise) for _ in range(n_samples)]
        rms = math.sqrt(sum(v * v for v in samples) / len(samples))
        mean = sum(samples) / len(samples)
        std = math.sqrt(sum((v - mean) ** 2 for v in samples) / len(samples)) or 1e-6
        kurt = (sum(((v - mean) / std) ** 4 for v in samples) / len(samples)) - 3.0
        return {"rms": round(rms, 4), "kurtosis": round(kurt, 3), "samples": samples}
