from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict


@dataclass
class TelemetryFrame:
    """A single timestamped frame containing all raw, filtered, feature, and diagnostic telemetry."""
    timestamp: float
    flight_id: int
    flight_state: str

    # Raw battery metrics
    battery_voltage: float
    battery_current: float
    battery_power: float
    battery_sensor_quality: str        # ONLINE / STALE / OFFLINE / ERROR

    # Raw propulsion metrics (for single motor ESC supply current)
    esc_current: float
    esc_current_quality: str
    esc_temp: float
    esc_temp_quality: str

    # Raw airframe metrics
    imu_accel: dict                    # {'x': float, 'y': float, 'z': float}
    imu_quality: str

    # Control states
    throttle_pct: float
    throttle_source: str               # simulation / pwm / mavlink

    # Filtered outputs
    battery_current_filtered: float = 0.0
    esc_temp_filtered: float = 0.0
    imu_rms: float = 0.0
    imu_kurtosis: float = 0.0
    imu_peak_freq: Optional[float] = None

    # Derived features and health metrics
    features: Optional[dict] = None
    diagnoses: Optional[list] = None
    soc: Optional[float] = None

    def to_dict(self) -> dict:
        return asdict(self)
