"""
Health Engine — vehicle-agnostic.
propulsion_health(unit) replaces motor_health() + esc_health().
"""

import logging
from dataclasses import dataclass, field
from typing import Optional

from core.propulsion import PropulsionUnit

logger = logging.getLogger(__name__)


@dataclass
class ComponentHealth:
    score: float = 100.0
    age_factor: float = 0.0
    warnings: list[str] = field(default_factory=list)

    def to_dict(self):
        return {"score": self.score, "age_factor": self.age_factor, "warnings": self.warnings}


class HealthEngine:
    def __init__(self, r_internal_baseline: float = 0.020,
                 capacity_full: float = 3300.0, temp_max: float = 50.0):
        self.r_internal_baseline = r_internal_baseline
        self.capacity_full = capacity_full
        self.temp_max = temp_max

    def battery_health(self, r_internal: float, capacity_now: float,
                       temp: float, cycles: int = 0, age_days: int = 0) -> ComponentHealth:
        r_growth = (r_internal - self.r_internal_baseline) / max(self.r_internal_baseline, 1e-6)
        r_penalty = max(0, min(1, r_growth * 2.0))

        capacity_loss = (self.capacity_full - capacity_now) / max(self.capacity_full, 1e-6)
        cap_penalty = max(0, min(1, capacity_loss * 2.0))

        temp_stress = max(0, (temp - self.temp_max) / 30.0) if temp > self.temp_max else 0.0
        temp_penalty = min(1, temp_stress)

        cycles_penalty = min(1, cycles / 500.0)
        age_penalty = min(1, age_days / 730.0)

        score = 100 * (1 - 0.35 * r_penalty - 0.25 * cap_penalty -
                       0.20 * temp_penalty - 0.10 * cycles_penalty - 0.10 * age_penalty)
        score = max(0, min(100, score))

        age_factor = max(r_penalty, cap_penalty, cycles / 500.0)
        age_factor = min(1, age_factor)

        warnings = []
        if r_penalty > 0.3:
            warnings.append("Internal resistance growth exceeds 30%")
        if cap_penalty > 0.2:
            warnings.append("Capacity loss exceeds 20%")
        if temp_penalty > 0:
            warnings.append("Battery temperature exceeded max")
        if score < 60:
            warnings.append("Battery health critical — replace soon")

        return ComponentHealth(score=round(score, 1), age_factor=round(age_factor, 3),
                               warnings=warnings)

    def propulsion_health(self, unit: PropulsionUnit) -> float:
        """
        Compute single health score for a propulsion unit.
        Combines current deviation, vibration, and thermal.
        Returns score 0–100.
        """
        current_dev = 0.0
        if unit.baseline_current and unit.baseline_current.get("expected", 0) > 0:
            expected = unit.baseline_current["expected"]
            current_dev = abs(unit.current - expected) / max(expected, 0.01)

        vib_dev = 0.0
        if unit.baseline_vibration and unit.baseline_vibration.get("expected", 0) > 0:
            expected = unit.baseline_vibration["expected"]
            vib_dev = abs(unit.vibration_rms - expected) / max(expected, 0.01)

        temp_dev = 0.0
        if unit.baseline_temp_rise and unit.baseline_temp_rise.get("expected", 0) > 0:
            expected = unit.baseline_temp_rise["expected"]
            temp_dev = abs(unit.temp_rise - expected) / max(expected, 0.01)

        score = 100 * (1 - 0.35 * min(current_dev, 2.0) - 0.40 * min(vib_dev, 2.0) - 0.25 * min(temp_dev, 2.0))
        score = max(0, min(100, score))

        warnings = []
        if current_dev > 0.3:
            warnings.append(f"Current deviates {current_dev*100:.0f}% from baseline")
        if vib_dev > 0.5:
            warnings.append(f"Vibration deviates {vib_dev*100:.0f}% from baseline")
        if temp_dev > 0.5:
            warnings.append("Temperature rise abnormal")
        if score < 60:
            warnings.append("Unit health critical — inspect before flight")

        unit.health = round(score, 1)
        unit.age_factor = min(1, 0.35 * min(current_dev, 2.0) + 0.40 * min(vib_dev, 2.0) + 0.25 * min(temp_dev, 2.0))
        unit.health_warnings = warnings
        return unit.health

    def compute_overall(self, units: list[PropulsionUnit], batt_score: float) -> float:
        scores = [batt_score] + [u.health for u in units]
        return round(sum(scores) / len(scores), 1)
