"""
Baseline Manager — vehicle-agnostic.
Stores per-propulsion-unit current/vibration/thermal profiles.
"""

import json
import os
import logging
from dataclasses import dataclass, field, asdict
from typing import Optional
from collections import defaultdict

import numpy as np

logger = logging.getLogger(__name__)

BASELINE_FILE = "config/baseline.json"


@dataclass
class UnitBaseline:
    current_profile: dict = field(default_factory=dict)   # {throttle_pct: mean}
    current_std: dict = field(default_factory=dict)
    vibration_rms: float = 0.15
    vibration_std: float = 0.02
    thermal_response: float = 1.0  # °C per A above ambient
    thermal_std: float = 0.2


@dataclass
class BatteryBaseline:
    voltage_sag_per_amp: float = 0.045  # V/A (per cell * cell_count * R)
    voltage_sag_std: float = 0.01


class BaselineManager:
    def __init__(self, filepath: str = BASELINE_FILE, n_units: int = 4):
        self.filepath = filepath
        self.n_units = n_units
        self.units: dict[int, UnitBaseline] = {}
        self.battery_baseline = BatteryBaseline()
        self._loaded = False
        self._load()

    def _load(self):
        if os.path.exists(self.filepath):
            with open(self.filepath) as f:
                data = json.load(f)
            pu_data = data.get("propulsion_units", {})
            for uid_str, bl in pu_data.items():
                uid = int(uid_str)
                self.units[uid] = UnitBaseline(
                    current_profile=bl.get("current_profile", {}),
                    current_std=bl.get("current_std", {}),
                    vibration_rms=bl.get("vibration_rms", 0.15),
                    vibration_std=bl.get("vibration_std", 0.02),
                    thermal_response=bl.get("thermal_response", 1.0),
                    thermal_std=bl.get("thermal_std", 0.2),
                )
            bb = data.get("battery", {})
            self.battery_baseline = BatteryBaseline(
                voltage_sag_per_amp=bb.get("voltage_sag_per_amp", 0.045),
                voltage_sag_std=bb.get("voltage_sag_std", 0.01),
            )
            self._loaded = True
            logger.info("Baseline loaded for %d propulsion units", len(self.units))
        else:
            logger.info("No baseline file — using default values")

    def is_calibrated(self) -> bool:
        return self._loaded

    def compare_current(self, unit_id: int, throttle: float, current: float) -> Optional[dict]:
        if unit_id not in self.units:
            return None
        bl = self.units[unit_id]
        # Find nearest throttle point in profile
        pcts = [float(k) for k in bl.current_profile.keys()]
        if not pcts:
            return None
        nearest = min(pcts, key=lambda p: abs(p - throttle * 100))
        expected = bl.current_profile.get(str(int(nearest)), bl.current_profile.get(str(nearest), 0.0))
        std = bl.current_std.get(str(int(nearest)), bl.current_std.get(str(nearest), 0.1))
        deviation = (current - expected) / max(std, 0.01)
        return {"z_score": deviation, "expected": expected, "std": std}

    def compare_vibration(self, unit_id: int, rms: float) -> Optional[dict]:
        if unit_id not in self.units:
            return None
        bl = self.units[unit_id]
        deviation = (rms - bl.vibration_rms) / max(bl.vibration_std, 0.01)
        return {"z_score": deviation, "expected": bl.vibration_rms, "std": bl.vibration_std}

    def compare_temp_rise(self, unit_id: int, temp_rise: float, current: float) -> Optional[dict]:
        if unit_id not in self.units:
            return None
        bl = self.units[unit_id]
        expected = bl.thermal_response * max(current, 0.1)
        deviation = (temp_rise - expected) / max(bl.thermal_std, 0.01)
        return {"z_score": deviation, "expected": expected, "std": bl.thermal_std}
