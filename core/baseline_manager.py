"""
Baseline Manager — Physics-Based PHM.

New schema: baseline.json is indexed by integer throttle percentage.
Supports:
  - Throttle-indexed lookup with linear interpolation
  - Rolling statistical fallback when uncalibrated
  - Multi-run statistics (mean, std, min, max) per field per throttle step
"""

import json
import os
import logging
import statistics
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)

BASELINE_FILE = "config/baseline.json"

# Fields stored in baseline per throttle step
BASELINE_FIELDS = [
    "esc_current",
    "battery_voltage",
    "battery_current",
    "battery_power",
    "esc_temp",
    "vibration_rms",
    "vibration_peak_freq",
    "current_ripple",
    "voltage_sag",
]

# Rolling window size for uncalibrated fallback (seconds worth of samples at 5 Hz)
ROLLING_WINDOW = 300  # 60 seconds × 5 Hz


@dataclass
class FieldStats:
    mean: float
    std: float
    min: float = 0.0
    max: float = 0.0
    n: int = 0

    def to_dict(self):
        return {"mean": self.mean, "std": self.std, "min": self.min, "max": self.max, "n": self.n}


class BaselineManager:
    """
    Loads and serves calibration baseline data indexed by throttle percentage.
    Provides interpolated expected values and deviation confidence.
    Falls back to rolling statistics when uncalibrated.
    """

    def __init__(self, filepath: str = BASELINE_FILE):
        self.filepath = filepath
        self._calibrated = False
        self._calibrated_at: Optional[str] = None
        self._n_runs: int = 0
        self._throttle_steps: list[int] = []
        self._data: dict[int, dict[str, FieldStats]] = {}   # {throttle_pct: {field: FieldStats}}

        # Rolling fallback windows per field (deque of recent values)
        self._rolling: dict[str, deque] = {f: deque(maxlen=ROLLING_WINDOW) for f in BASELINE_FIELDS}

        self._load()

    # ── Persistence ──────────────────────────────────────────────────────────

    def _load(self):
        if not os.path.exists(self.filepath):
            logger.info("No baseline file found — operating in uncalibrated mode")
            return

        try:
            with open(self.filepath, encoding="utf-8") as fh:
                raw = json.load(fh)
        except (json.JSONDecodeError, OSError) as exc:
            logger.error("Failed to load baseline.json: %s", exc)
            return

        if not raw.get("calibrated", False):
            logger.info("Baseline file present but uncalibrated flag set — rolling fallback active")
            return

        # Detect old schema (has "motors" key) — reject gracefully
        if "motors" in raw:
            logger.warning(
                "Old baseline.json schema detected (has 'motors' key). "
                "Please re-run calibration with the new calibrate.py to generate a valid baseline."
            )
            return

        self._calibrated = True
        self._calibrated_at = raw.get("calibrated_at")
        self._n_runs = raw.get("n_runs", 1)
        self._throttle_steps = [int(k) for k in raw.get("data", {}).keys()]

        for step_str, step_data in raw.get("data", {}).items():
            step = int(step_str)
            self._data[step] = {}
            for field_name, stats in step_data.items():
                if isinstance(stats, dict):
                    self._data[step][field_name] = FieldStats(
                        mean=float(stats.get("mean", 0.0)),
                        std=max(float(stats.get("std", 0.0)), 1e-6),
                        min=float(stats.get("min", 0.0)),
                        max=float(stats.get("max", 0.0)),
                        n=int(stats.get("n", 0)),
                    )

        logger.info(
            "Baseline loaded: calibrated=%s, runs=%d, throttle steps=%s",
            self._calibrated_at, self._n_runs, self._throttle_steps,
        )

    def save(self, data: dict):
        """Save a freshly computed calibration data dict to file."""
        try:
            with open(self.filepath, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2)
            # Reload to apply in-memory
            self._calibrated = False
            self._data.clear()
            self._load()
            logger.info("Baseline saved and reloaded successfully.")
        except OSError as exc:
            logger.error("Failed to save baseline: %s", exc)

    # ── Rolling Fallback ─────────────────────────────────────────────────────

    def push_sample(self, field: str, value: float):
        """Feed a live measurement into the rolling fallback window."""
        if field in self._rolling:
            self._rolling[field].append(value)

    def _rolling_stats(self, field: str) -> Optional[FieldStats]:
        buf = list(self._rolling.get(field, []))
        n = len(buf)
        if n < 10:
            return None
        mean = statistics.mean(buf)
        std = statistics.stdev(buf) if n > 1 else 1e-6
        return FieldStats(mean=mean, std=max(std, 1e-6), min=min(buf), max=max(buf), n=n)

    # ── Lookup ───────────────────────────────────────────────────────────────

    def is_calibrated(self) -> bool:
        return self._calibrated and bool(self._data)

    def calibration_info(self) -> dict:
        return {
            "calibrated": self._calibrated,
            "calibrated_at": self._calibrated_at,
            "n_runs": self._n_runs,
            "throttle_steps": self._throttle_steps,
        }

    def get_expected(self, throttle_pct: Optional[float], field: str) -> Optional[FieldStats]:
        """
        Return interpolated FieldStats for a given throttle percentage and field name.
        If uncalibrated or field unknown, falls back to rolling statistics.
        Returns None if no data available.
        """
        if self._calibrated and self._throttle_steps and throttle_pct is not None:
            return self._interpolate(throttle_pct, field)

        # Rolling fallback
        return self._rolling_stats(field)

    def _interpolate(self, throttle_pct: float, field: str) -> Optional[FieldStats]:
        """Linear interpolation between the two nearest calibration throttle steps."""
        steps = sorted(self._throttle_steps)
        if not steps:
            return None

        # Clamp to calibrated range
        if throttle_pct <= steps[0]:
            step_data = self._data.get(steps[0], {})
            return step_data.get(field)
        if throttle_pct >= steps[-1]:
            step_data = self._data.get(steps[-1], {})
            return step_data.get(field)

        # Find bracketing steps
        lower = max(s for s in steps if s <= throttle_pct)
        upper = min(s for s in steps if s > throttle_pct)

        lo_data = self._data.get(lower, {})
        hi_data = self._data.get(upper, {})

        lo_stats = lo_data.get(field)
        hi_stats = hi_data.get(field)

        if lo_stats is None or hi_stats is None:
            return lo_stats or hi_stats

        # Linear interpolation weight
        span = upper - lower
        t = (throttle_pct - lower) / span if span > 0 else 0.0

        mean = lo_stats.mean + t * (hi_stats.mean - lo_stats.mean)
        std = lo_stats.std + t * (hi_stats.std - lo_stats.std)

        return FieldStats(mean=round(mean, 6), std=max(round(std, 6), 1e-6))

    def interpolation_confidence(self, throttle_pct: Optional[float]) -> float:
        """
        Returns 0.0–1.0 confidence in interpolation quality.
        1.0 = exact calibration point hit
        0.5 = midway between two known points
        0.0 = uncalibrated (rolling fallback)
        """
        if not self._calibrated or not self._throttle_steps or throttle_pct is None:
            return 0.3  # rolling fallback — low but non-zero

        steps = sorted(self._throttle_steps)
        # Check for exact hit
        nearest = min(steps, key=lambda s: abs(s - throttle_pct))
        dist = abs(nearest - throttle_pct)
        # Within 1% of a calibration step → full confidence
        if dist <= 1.0:
            return 1.0
        # Max meaningful gap: half the typical step spacing
        if len(steps) > 1:
            avg_step = (steps[-1] - steps[0]) / (len(steps) - 1)
            gap_fraction = min(dist / (avg_step / 2), 1.0)
            return round(1.0 - 0.4 * gap_fraction, 3)
        return 0.6
