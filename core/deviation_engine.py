"""
Deviation Engine — Physics-Based PHM.

Computes normalized deviations of every measured quantity from its calibrated
baseline expectation.  ALL health indices and condition classifiers operate on
DeviationFrame fields — never on raw sensor values.

Deviation formula (signed, so direction matters for PLI):
    deviation = (measured - expected) / expected

Absolute deviation is taken by callers where directionality is not needed.
"""

from dataclasses import dataclass, field
from typing import Optional
import logging

from core.baseline_manager import BaselineManager
from core.telemetry_frame import TelemetryFrame

log = logging.getLogger(__name__)


@dataclass
class DeviationFrame:
    """
    Normalized deviations from calibrated baseline at a given throttle point.
    All deviations are (measured − expected) / expected.
    Positive = higher than expected. Negative = lower than expected.
    """
    throttle_pct: float           # 0.0–100.0 throttle at measurement time
    calibrated: bool              # True = full calibration used, False = rolling fallback

    # ── Electrical deviations ─────────────────────────────────────────────
    current_deviation: float = 0.0        # ESC current
    power_deviation: float = 0.0          # Battery power
    voltage_sag_deviation: float = 0.0    # Voltage sag (BSI primary input)
    ripple_deviation: float = 0.0         # Current ripple amplitude

    # ── Thermal deviation ─────────────────────────────────────────────────
    temp_deviation: float = 0.0           # ESC temperature

    # ── Vibration deviations ──────────────────────────────────────────────
    vibration_deviation: float = 0.0      # IMU vibration RMS

    # ── Expected values (for evidence display) ────────────────────────────
    current_expected: float = 0.0
    power_expected: float = 0.0
    temp_expected: float = 0.0
    vibration_expected: float = 0.0
    voltage_sag_expected: float = 0.0

    # ── Confidence ────────────────────────────────────────────────────────
    confidence: float = 0.0              # 0.0–1.0 interpolation quality

    def deviation_pct(self, dev: float) -> str:
        """Format a deviation as a signed percentage string for evidence display."""
        return f"{dev * 100:+.0f}%"

    def to_dict(self) -> dict:
        return {
            "throttle_pct": self.throttle_pct,
            "calibrated": self.calibrated,
            "confidence": round(self.confidence, 3),
            "current_deviation": round(self.current_deviation, 4),
            "power_deviation": round(self.power_deviation, 4),
            "temp_deviation": round(self.temp_deviation, 4),
            "vibration_deviation": round(self.vibration_deviation, 4),
            "voltage_sag_deviation": round(self.voltage_sag_deviation, 4),
            "ripple_deviation": round(self.ripple_deviation, 4),
            "current_expected": round(self.current_expected, 3),
            "power_expected": round(self.power_expected, 3),
            "temp_expected": round(self.temp_expected, 2),
            "vibration_expected": round(self.vibration_expected, 4),
            "voltage_sag_expected": round(self.voltage_sag_expected, 3),
        }


def _safe_deviation(measured: float, expected: float) -> float:
    """
    Compute (measured - expected) / expected.
    Returns 0.0 if expected is effectively zero to avoid div/0.
    Clamps output to [-10, 10] to prevent runaway values from near-zero baselines.
    """
    if abs(expected) < 1e-6:
        return 0.0
    return max(-10.0, min(10.0, (measured - expected) / expected))


class DeviationEngine:
    """
    Computes a DeviationFrame from a TelemetryFrame and the BaselineManager.
    Feeds raw values into the baseline rolling window for uncalibrated fallback.
    """

    def compute(self, frame: TelemetryFrame, baseline: BaselineManager) -> DeviationFrame:
        throttle = (frame.throttle_pct or 0.0) * 100.0   # convert 0–1 → 0–100

        # Feed rolling fallback windows regardless of calibration state
        baseline.push_sample("esc_current", frame.esc_current)
        baseline.push_sample("battery_voltage", frame.battery_voltage)
        baseline.push_sample("battery_current", frame.battery_current)
        baseline.push_sample("battery_power", frame.battery_power)
        baseline.push_sample("esc_temp", frame.esc_temp_filtered)
        baseline.push_sample("vibration_rms", frame.imu_rms)
        if frame.imu_peak_freq:
            baseline.push_sample("vibration_peak_freq", frame.imu_peak_freq)

        # Estimate voltage sag: difference between no-load voltage and current voltage
        # We use battery_voltage as proxy. In calibration, sag = V_open - V_load.
        # Here we track deviation in battery_voltage directly (lower = more sag).
        voltage_sag_measured = frame.battery_voltage   # lower = more sag relative to baseline

        confidence = baseline.interpolation_confidence(throttle)
        is_calibrated = baseline.is_calibrated()

        # ── Get expected values ──────────────────────────────────────────
        def expected(field_name: str) -> Optional[float]:
            stats = baseline.get_expected(throttle, field_name)
            return stats.mean if stats else None

        exp_current = expected("esc_current") or frame.esc_current
        exp_power = expected("battery_power") or frame.battery_power
        exp_temp = expected("esc_temp") or frame.esc_temp_filtered
        exp_vib = expected("vibration_rms") or frame.imu_rms
        exp_sag = expected("battery_voltage") or frame.battery_voltage
        exp_ripple_raw = baseline.get_expected(throttle, "current_ripple")
        exp_ripple = exp_ripple_raw.mean if exp_ripple_raw else None

        # Current ripple: computed from battery_current variance proxy if available
        # We use frame.battery_current as a proxy (no separate ripple field in frame yet)
        # If not available, skip ripple deviation
        ripple_dev = 0.0
        if exp_ripple and exp_ripple > 1e-6:
            # We don't have a direct current ripple measurement in the frame currently.
            # Use the current variance as a proxy from features if available.
            ripple_measured = None
            if frame.features:
                bf = frame.features.get("battery_features", {})
                ripple_measured = bf.get("ripple_amplitude")
            if ripple_measured is not None:
                ripple_dev = _safe_deviation(ripple_measured, exp_ripple)

        dev = DeviationFrame(
            throttle_pct=throttle,
            calibrated=is_calibrated,
            confidence=confidence,

            current_deviation=_safe_deviation(frame.esc_current, exp_current),
            power_deviation=_safe_deviation(frame.battery_power, exp_power),
            temp_deviation=_safe_deviation(frame.esc_temp_filtered, exp_temp),
            vibration_deviation=_safe_deviation(frame.imu_rms, exp_vib),
            voltage_sag_deviation=_safe_deviation(voltage_sag_measured, exp_sag),
            ripple_deviation=ripple_dev,

            current_expected=exp_current,
            power_expected=exp_power,
            temp_expected=exp_temp,
            vibration_expected=exp_vib,
            voltage_sag_expected=exp_sag,
        )

        return dev
