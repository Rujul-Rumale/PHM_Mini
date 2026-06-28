"""
Health Index Engine — Physics-Based PHM.

Computes 4 physically meaningful health indices from a DeviationFrame.
No raw sensor values are read here — only normalised deviations.

Indices:
  PBI  Propeller Balance Index     — rotational imbalance
  PLI  Propulsion Load Index       — abnormal loading / missing prop
  ETI  ESC Thermal Index           — thermal behaviour anomaly
  BSI  Battery Stress Index        — battery stress

All indices are dimensionless, 0.0 = perfectly healthy.
Thresholds:
  < 0.20  Normal   (green)
  0.20–0.50  Elevated  (amber)
  > 0.50  Warning   (red)
  > 0.80  Critical  (deep red — can trigger FSM transition)

The overall Propulsion Health Score (0–100) is derived from indices.
"""

import time
import math
import logging
from dataclasses import dataclass, field
from collections import deque
from typing import Optional

from core.deviation_engine import DeviationFrame

log = logging.getLogger(__name__)

# ── Index thresholds ──────────────────────────────────────────────────────────
NORMAL_THRESHOLD = 0.20
ELEVATED_THRESHOLD = 0.50
WARNING_THRESHOLD = 0.80


def _band(value: float) -> str:
    """Return a human-readable band label."""
    v = abs(value)
    if v < NORMAL_THRESHOLD:
        return "Normal"
    if v < ELEVATED_THRESHOLD:
        return "Elevated"
    if v < WARNING_THRESHOLD:
        return "Warning"
    return "Critical"


def _clamp(v: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


@dataclass
class HealthIndices:
    """All four PHM health indices at a single point in time."""
    pbi: float = 0.0   # Propeller Balance Index
    pli: float = 0.0   # Propulsion Load Index  (signed: + high load, - missing prop)
    eti: float = 0.0   # ESC Thermal Index
    bsi: float = 0.0   # Battery Stress Index

    health_score: float = 100.0   # Overall propulsion health 0–100

    pbi_band: str = "Normal"
    pli_band: str = "Normal"
    eti_band: str = "Normal"
    bsi_band: str = "Normal"

    calibrated: bool = False
    confidence: float = 0.0

    # Short trend labels (populated by HealthIndexEngine over time)
    trend_1min: str = "Stable"
    trend_10min: str = "Stable"
    trend_flight: str = "Stable"

    def to_dict(self) -> dict:
        return {
            "pbi": round(self.pbi, 4),
            "pli": round(self.pli, 4),
            "eti": round(self.eti, 4),
            "bsi": round(self.bsi, 4),
            "pbi_band": self.pbi_band,
            "pli_band": self.pli_band,
            "eti_band": self.eti_band,
            "bsi_band": self.bsi_band,
            "health_score": round(self.health_score, 1),
            "calibrated": self.calibrated,
            "confidence": round(self.confidence, 3),
            "trend_1min": self.trend_1min,
            "trend_10min": self.trend_10min,
            "trend_flight": self.trend_flight,
        }


class HealthIndexEngine:
    """
    Computes health indices and maintains rolling trend histories.
    Sampled at the diagnostic rate (5 Hz).
    """

    def __init__(self):
        # Rolling history for trend analysis (sampled every 5 s ≈ 300 s window)
        # 12 samples/min × 10 min = 120 samples for 10-min window
        self._pbi_history: deque = deque(maxlen=120)
        self._pli_history: deque = deque(maxlen=120)
        self._eti_history: deque = deque(maxlen=120)
        self._bsi_history: deque = deque(maxlen=120)
        self._score_history: deque = deque(maxlen=120)

        # Flight-length history (no size cap — one entry every 5 s for entire flight)
        self._pbi_flight: list = []
        self._pli_flight: list = []
        self._eti_flight: list = []
        self._bsi_flight: list = []
        self._score_flight: list = []

        self._last_trend_sample = 0.0

    # ── Index computation ─────────────────────────────────────────────────────

    def compute(self, dev: DeviationFrame) -> HealthIndices:
        """
        Compute all four health indices from a DeviationFrame.
        """
        pbi = self._compute_pbi(dev)
        pli = self._compute_pli(dev)
        eti = self._compute_eti(dev)
        bsi = self._compute_bsi(dev)

        # Overall health score: weighted penalty from each index
        # Weights: PBI 40%, PLI 35%, ETI 15%, BSI 10%
        penalty = (
            0.40 * _clamp(abs(pbi))
            + 0.35 * _clamp(abs(pli))
            + 0.15 * _clamp(abs(eti))
            + 0.10 * _clamp(abs(bsi))
        )
        health_score = max(0.0, min(100.0, 100.0 * (1.0 - penalty)))

        indices = HealthIndices(
            pbi=round(pbi, 4),
            pli=round(pli, 4),
            eti=round(eti, 4),
            bsi=round(bsi, 4),
            health_score=round(health_score, 1),
            pbi_band=_band(pbi),
            pli_band=_band(pli),
            eti_band=_band(eti),
            bsi_band=_band(bsi),
            calibrated=dev.calibrated,
            confidence=dev.confidence,
        )

        self._update_trends(indices)
        return indices

    def _compute_pbi(self, dev: DeviationFrame) -> float:
        """
        Propeller Balance Index (PBI):
        Elevated vibration with a small current increase → classic rotational imbalance.

        PBI = 0.70 * vib_dev + 0.30 * ripple_dev
            Adjusted DOWN if current is also large (that's a load issue, not imbalance).
        Only positive deviations count — a prop that runs quieter is not imbalanced.
        """
        vib_dev = max(0.0, dev.vibration_deviation)      # only positive excursions
        ripple_dev = max(0.0, dev.ripple_deviation)

        raw = 0.70 * vib_dev + 0.30 * ripple_dev

        # Suppress PBI when current is also very high (that's PLI territory, not PBI)
        current_abs = abs(dev.current_deviation)
        if current_abs > 0.30 and vib_dev > 0:
            # Scale down PBI when current and vibration both rise together
            suppression = _clamp(current_abs / max(vib_dev, 0.01), 0.0, 0.80)
            raw *= (1.0 - 0.5 * suppression)

        return round(raw, 4)

    def _compute_pli(self, dev: DeviationFrame) -> float:
        """
        Propulsion Load Index (PLI):
        Signed — positive means overloaded, negative means underloaded.

        PLI = 0.60 * current_dev + 0.40 * power_dev
        Missing prop → strongly negative PLI.
        High drag / resistance → positive PLI.
        """
        pli = 0.60 * dev.current_deviation + 0.40 * dev.power_deviation
        return round(pli, 4)

    def _compute_eti(self, dev: DeviationFrame) -> float:
        """
        ESC Thermal Index (ETI):
        Temperature rising faster than current alone would explain.

        ETI = temp_dev - 0.5 * current_dev
            (If current is also high, some temp rise is expected — net out the expected portion)
        Only positive ETI values are meaningful (cooling problems; sub-zero means ESC cooler than normal).
        """
        # Account for expected thermal rise proportional to current increase
        current_contribution = 0.50 * dev.current_deviation
        net_temp_deviation = dev.temp_deviation - current_contribution
        eti = max(0.0, net_temp_deviation)   # only positive: excess heat
        return round(eti, 4)

    def _compute_bsi(self, dev: DeviationFrame) -> float:
        """
        Battery Stress Index (BSI):
        Battery working harder than calibrated conditions — voltage dropping more than expected.

        Note: voltage_sag_deviation is negative when battery is sagging more (lower voltage).
        We use the absolute value so BSI > 0 means stressed.
        BSI = 0.60 * abs(sag_dev) + 0.40 * abs(current_dev)
        """
        sag = abs(dev.voltage_sag_deviation)
        cur = abs(dev.current_deviation)
        bsi = 0.60 * sag + 0.40 * cur
        return round(bsi, 4)

    # ── Trend Analysis ────────────────────────────────────────────────────────

    def _update_trends(self, indices: HealthIndices):
        now = time.monotonic()
        # Sample trends once every 5 seconds
        if now - self._last_trend_sample < 5.0:
            return
        self._last_trend_sample = now

        self._pbi_history.append(indices.pbi)
        self._pli_history.append(abs(indices.pli))
        self._eti_history.append(indices.eti)
        self._bsi_history.append(indices.bsi)
        self._score_history.append(indices.health_score)

        self._pbi_flight.append(indices.pbi)
        self._pli_flight.append(abs(indices.pli))
        self._eti_flight.append(indices.eti)
        self._bsi_flight.append(indices.bsi)
        self._score_flight.append(indices.health_score)

        indices.trend_1min = self._slope_label(list(self._pbi_history)[-12:], higher_is_worse=True)
        indices.trend_10min = self._slope_label(list(self._pbi_history), higher_is_worse=True)
        indices.trend_flight = self._slope_label(self._pbi_flight, higher_is_worse=True)

    def get_trends(self) -> dict:
        """Return trend labels for all indices (called by diagnostics engine to push to StatusManager)."""
        def labels(history, flight, higher_is_worse=True):
            return {
                "trend_1min": self._slope_label(list(history)[-12:], higher_is_worse),
                "trend_10min": self._slope_label(list(history), higher_is_worse),
                "trend_flight": self._slope_label(flight, higher_is_worse),
            }

        return {
            "pbi": labels(self._pbi_history, self._pbi_flight),
            "pli": labels(self._pli_history, self._pli_flight),
            "eti": labels(self._eti_history, self._eti_flight),
            "bsi": labels(self._bsi_history, self._bsi_flight),
            "health_score": labels(self._score_history, self._score_flight, higher_is_worse=False),
        }

    def get_flight_summary(self) -> dict:
        """Returns per-flight averages and peaks for cross-flight trend storage."""
        def summary(lst):
            if not lst:
                return {"avg": 0.0, "max": 0.0, "n": 0}
            return {"avg": round(sum(lst) / len(lst), 4), "max": round(max(lst), 4), "n": len(lst)}

        return {
            "pbi": summary(self._pbi_flight),
            "pli": summary(self._pli_flight),
            "eti": summary(self._eti_flight),
            "bsi": summary(self._bsi_flight),
            "health_score": summary(self._score_flight),
        }

    def reset_flight(self):
        """Clear per-flight accumulations at flight start."""
        self._pbi_flight.clear()
        self._pli_flight.clear()
        self._eti_flight.clear()
        self._bsi_flight.clear()
        self._score_flight.clear()

    @staticmethod
    def _slope_label(history: list, higher_is_worse: bool = True) -> str:
        """Simple linear regression slope → human-readable trend label."""
        n = len(history)
        if n < 4:
            return "Stable"
        x_mean = (n - 1) / 2.0
        y_mean = sum(history) / n
        num = sum((i - x_mean) * (v - y_mean) for i, v in enumerate(history))
        den = sum((i - x_mean) ** 2 for i in range(n))
        slope = num / den if den > 0 else 0.0
        # Scale by 12 = samples/min (5 s apart)
        rate_per_min = slope * 12.0

        if higher_is_worse:
            if rate_per_min > 0.05:
                return "Increasing ▲"
            if rate_per_min < -0.05:
                return "Decreasing ▼"
        else:
            if rate_per_min < -1.0:
                return "Declining ▼"
            if rate_per_min > 1.0:
                return "Improving ▲"

        return "Stable"
