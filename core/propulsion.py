"""
Propulsion Unit Model — vehicle-agnostic.
Quad = 4 units, fixed-wing = 1 unit.
All diagnostics operate on PropulsionUnit objects.
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PropulsionUnit:
    id: int
    name: str

    current: float = 0.0
    esc_temp: float = 25.0
    vibration_rms: float = 0.0
    vibration_kurtosis: float = 0.0
    temp_rise: float = 0.0

    health: float = 100.0
    health_warnings: list[str] = field(default_factory=list)
    age_factor: float = 0.0

    baseline_current: Optional[dict] = None
    baseline_vibration: Optional[dict] = None
    baseline_temp_rise: Optional[dict] = None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "current": round(self.current, 3),
            "esc_temp": round(self.esc_temp, 1),
            "vibration_rms": round(self.vibration_rms, 4),
            "vibration_kurtosis": round(self.vibration_kurtosis, 3),
            "temp_rise": round(self.temp_rise, 2),
            "health": round(self.health, 1),
            "warnings": self.health_warnings,
        }


def make_units(profile: dict) -> list[PropulsionUnit]:
    """Create PropulsionUnits from vehicle_profile.json propulsion_units list."""
    return [PropulsionUnit(id=u["id"], name=u["name"]) for u in profile["propulsion_units"]]
