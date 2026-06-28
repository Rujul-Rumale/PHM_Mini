"""
Feature Extraction — vehicle-agnostic.
Operates on PropulsionUnit objects, scales to N units.
"""

import math
import statistics
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Optional, List
from scipy import stats

WINDOW = 50


@dataclass
class ElectricalBatteryFeatures:
    voltage: float
    current: float
    power: float
    voltage_variance: float
    voltage_trend: float
    current_variance: float
    load_voltage_drop: float
    ripple_amplitude: float

    def to_dict(self):
        return asdict(self)


@dataclass
class PropulsionFeatures:
    unit_id: int
    current: float
    current_variance: float
    current_trend: float
    spike_count: int
    imbalance_ratio: float = 0.0

    def to_dict(self):
        return asdict(self)

# Compatibility Alias for older tests
ElectricalMotorFeatures = PropulsionFeatures


@dataclass
class ThermalFeatures:
    temps: list[float] = field(default_factory=list)
    ambient_temp: float = 25.0
    temp_rise: list[float] = field(default_factory=list)

    @property
    def motor_temps(self):
        return self.temps

    @property
    def temp_rise_per_amp(self):
        return self.temp_rise

    def to_dict(self):
        return asdict(self)



@dataclass
class VibrationFeatures:
    unit_id: int
    rms: Optional[float] = None
    kurtosis: Optional[float] = None
    peak_freq: Optional[float] = None
    spectral_energy: Optional[float] = None

    @property
    def motor_id(self):
        return self.unit_id

    def to_dict(self):
        return asdict(self)


def _linear_slope(values: list) -> float:
    n = len(values)
    if n < 30:
        return 0.0
    result = stats.linregress(range(n), values)
    return result.slope


def _kurtosis(values: list) -> float:
    n = len(values)
    if n < 4:
        return 0.0
    mean = statistics.mean(values)
    std = statistics.stdev(values)
    if std == 0:
        return 0.0
    return (sum(((v - mean) / std) ** 4 for v in values) / n) - 3.0


class FeatureExtractor:
    def __init__(self, n_units: int = 4, motor_count: Optional[int] = None):
        n = motor_count if motor_count is not None else n_units
        self._n_units = n
        self._current_windows: List[deque] = [
            deque(maxlen=WINDOW) for _ in range(n)
        ]
        self._batt_v_window: deque = deque(maxlen=WINDOW)
        self._batt_i_window: deque = deque(maxlen=WINDOW)
        self._last_no_load_v: Optional[float] = None

        self._temp_windows: List[deque] = [
            deque(maxlen=WINDOW) for _ in range(n_units)
        ]
        self._vib_windows: List[deque] = [
            deque(maxlen=WINDOW) for _ in range(n_units)
        ]

    def electrical_battery(self, voltage: float, current: float, power: float) -> ElectricalBatteryFeatures:
        self._batt_v_window.append(voltage)
        self._batt_i_window.append(current)

        v_list = list(self._batt_v_window)
        i_list = list(self._batt_i_window)

        if current < 1.0:
            self._last_no_load_v = voltage

        load_drop = (self._last_no_load_v - voltage) if self._last_no_load_v else 0.0

        return ElectricalBatteryFeatures(
            voltage=round(voltage, 4),
            current=round(current, 4),
            power=round(power, 3),
            voltage_variance=round(statistics.variance(v_list) if len(v_list) > 1 else 0.0, 6),
            voltage_trend=round(_linear_slope(v_list), 6),
            current_variance=round(statistics.variance(i_list) if len(i_list) > 1 else 0.0, 6),
            load_voltage_drop=round(max(load_drop, 0.0), 4),
            ripple_amplitude=round(max(v_list) - min(v_list), 4),
        )

    def propulsion_currents(self, currents: list[Optional[float]]) -> list[PropulsionFeatures]:
        valid = [c for c in currents if c is not None]
        mean_i = statistics.mean(valid) if valid else 1.0

        features = []
        for idx in range(self._n_units):
            current = currents[idx] if idx < len(currents) else None
            win = self._current_windows[idx]

            if current is not None:
                win.append(current)

            win_list = list(win)
            n = len(win_list)

            if n < 2 or current is None:
                features.append(PropulsionFeatures(
                    unit_id=idx + 1, current=current or 0.0,
                    current_variance=0.0, current_trend=0.0, spike_count=0
                ))
                continue

            variance = statistics.variance(win_list)
            std = math.sqrt(variance)
            trend = _linear_slope(win_list)
            win_mean = statistics.mean(win_list)
            spike_threshold = win_mean + 3 * std if std > 0 else win_mean + 1.0
            spikes = sum(1 for v in win_list if v > spike_threshold)

            imbalance = 0.0
            if mean_i > 0:
                imbalance = abs(current - mean_i) / mean_i

            features.append(PropulsionFeatures(
                unit_id=idx + 1,
                current=round(current, 4),
                current_variance=round(variance, 6),
                current_trend=round(trend, 6),
                spike_count=spikes,
                imbalance_ratio=round(imbalance, 4),
            ))

        return features

    def thermal(self, temps: list[float], ambient: float = 25.0) -> ThermalFeatures:
        for i, t in enumerate(temps):
            if i < len(self._temp_windows):
                self._temp_windows[i].append(t)

        temp_rise = []
        for i in range(min(len(temps), self._n_units)):
            win = list(self._temp_windows[i])
            if win:
                avg_temp = statistics.mean(win)
                temp_rise.append(round(avg_temp - ambient, 2))
            else:
                temp_rise.append(0.0)

        return ThermalFeatures(
            temps=[round(t, 2) for t in temps],
            ambient_temp=ambient,
            temp_rise=temp_rise,
        )

    def vibration(self, unit_id: int, accel_samples: list[float],
                  sample_rate_hz: float = 100.0) -> Optional[VibrationFeatures]:
        if not accel_samples:
            return None

        rms = math.sqrt(statistics.mean(v ** 2 for v in accel_samples))
        kurt = _kurtosis(accel_samples)

        if unit_id - 1 < len(self._vib_windows):
            self._vib_windows[unit_id - 1].append(rms)

        peak_freq = None
        spectral_energy = None
        try:
            import numpy as np
            fft_vals = np.abs(np.fft.rfft(accel_samples))
            freqs = np.fft.rfftfreq(len(accel_samples), 1.0 / sample_rate_hz)
            peak_freq = float(freqs[np.argmax(fft_vals)])
            spectral_energy = float(np.sum(fft_vals ** 2))
        except ImportError:
            pass

        return VibrationFeatures(
            unit_id=unit_id,
            rms=round(rms, 5),
            kurtosis=round(kurt, 4),
            peak_freq=round(peak_freq, 2) if peak_freq else None,
            spectral_energy=round(spectral_energy, 2) if spectral_energy else None,
        )

    # Legacy method compatibility wrappers
    def electrical_features(self, voltage: float, current: float, power: float,
                            motor_currents: list) -> tuple[ElectricalBatteryFeatures, list[PropulsionFeatures]]:
        batt = self.electrical_battery(voltage, current, power)
        motors = self.propulsion_currents(motor_currents)
        return batt, motors

    def thermal_features(self, temps: list[float], ambient: float = 25.0) -> ThermalFeatures:
        return self.thermal(temps, ambient)

    def vibration_features(self, motor_id: int, accel_samples: list[float],
                           sample_rate_hz: float = 100.0) -> Optional[VibrationFeatures]:
        return self.vibration(motor_id, accel_samples, sample_rate_hz)
