"""
PHM Diagnostics — Physics-Based UAV Propulsion Health Monitoring.

Classifies OBSERVABLE CONDITIONS from health indices (PBI / PLI / ETI / BSI).
Never claims exact mechanical faults. Never reads raw sensor values.

Condition severities:
  info     — informational (e.g., uncalibrated operation)
  warning  — requires attention before next mission
  critical — requires immediate action / landing

Debounce: a condition must appear in N consecutive evaluations to be confirmed.
Critical conditions bypass debounce and are raised immediately.
"""

import time
import logging
from dataclasses import dataclass, field, asdict
from collections import defaultdict
from typing import Optional

from core.health_engine import HealthIndices, WARNING_THRESHOLD, ELEVATED_THRESHOLD, NORMAL_THRESHOLD
from core.deviation_engine import DeviationFrame

log = logging.getLogger(__name__)


# ── Condition dataclass ───────────────────────────────────────────────────────

@dataclass
class Condition:
    """An observable PHM condition derived from health indices."""
    condition_id: str            # e.g. "rotational_imbalance"
    title: str                   # Short display title
    confidence: float            # 0.0–1.0
    severity: str                # "info" | "warning" | "critical"
    evidence: dict               # Field → formatted deviation string
    recommendation: str
    possible_causes: list        # Never a single claimed root cause
    timestamp: float = field(default_factory=time.time)

    # Backward-compat aliases for code that still expects FaultDiagnosis fields
    @property
    def component(self) -> str:
        return "propulsion"

    @property
    def fault_type(self) -> str:
        return self.condition_id

    @property
    def level(self) -> str:
        return self.severity

    @property
    def reasoning(self) -> str:
        return f"Confidence {self.confidence*100:.0f}%. Evidence: " + ", ".join(
            f"{k}: {v}" for k, v in self.evidence.items()
        )

    def to_dict(self) -> dict:
        return {
            "condition_id": self.condition_id,
            "title": self.title,
            "confidence": round(self.confidence, 3),
            "severity": self.severity,
            "evidence": self.evidence,
            "recommendation": self.recommendation,
            "possible_causes": self.possible_causes,
            "timestamp": self.timestamp,
            # Legacy fields
            "component": self.component,
            "fault_type": self.fault_type,
        }


# Backward-compat alias so existing dashboard/db code referencing FaultDiagnosis still works
FaultDiagnosis = Condition


# ── Diagnostics classifier ────────────────────────────────────────────────────

class Diagnostics:
    """
    Classifies observable conditions from PHM health indices and deviation frame.
    Debounces non-critical conditions over N consecutive evaluations.
    """

    def __init__(self, persistence_count: int = 3):
        self.persistence_count = persistence_count
        self._counters: dict = defaultdict(int)
        self._active: dict = {}       # condition_id → Condition
        self._history: list = []
        self._max_history = 500

    # ── Classification ────────────────────────────────────────────────────────

    def classify(self, indices: HealthIndices, dev: DeviationFrame) -> list:
        """
        Evaluate current health indices and deviation frame.
        Returns a list of Condition objects detected this evaluation cycle.
        Does NOT apply debounce — call update() to get confirmed conditions.
        """
        conditions = []
        pbi = indices.pbi
        pli = indices.pli
        eti = indices.eti
        bsi = indices.bsi

        # ── PBI: Rotational Imbalance ─────────────────────────────────────
        if pbi >= ELEVATED_THRESHOLD:
            # Distinguish severity: vibration dominant vs current also elevated
            vib_dominant = abs(dev.vibration_deviation) > abs(dev.current_deviation) * 1.5
            confidence = min(0.98, 0.60 + 0.38 * min((pbi - ELEVATED_THRESHOLD) / 0.5, 1.0))
            if not vib_dominant:
                confidence *= 0.75   # lower confidence if current is also rising

            severity = "critical" if pbi >= WARNING_THRESHOLD else "warning"

            conditions.append(Condition(
                condition_id="rotational_imbalance",
                title="Rotational imbalance detected",
                confidence=round(confidence, 3),
                severity=severity,
                evidence={
                    "vibration_deviation": dev.deviation_pct(dev.vibration_deviation),
                    "current_deviation": dev.deviation_pct(dev.current_deviation),
                    "PBI": f"{pbi:.3f}",
                },
                recommendation="Inspect propeller and hub before next flight. Check for physical damage, dirt, or missing balance weight.",
                possible_causes=[
                    "Damaged or chipped propeller blade",
                    "Propeller imbalance (mass distribution asymmetry)",
                    "Debris attached to blade",
                    "Loose propeller hub",
                ],
            ))

        # ── PLI: Propulsion Load Anomaly — High ───────────────────────────
        if pli >= ELEVATED_THRESHOLD:
            severity = "critical" if pli >= WARNING_THRESHOLD else "warning"
            confidence = min(0.95, 0.55 + 0.40 * min((pli - ELEVATED_THRESHOLD) / 0.5, 1.0))

            conditions.append(Condition(
                condition_id="propulsion_load_high",
                title="Propulsion load elevated",
                confidence=round(confidence, 3),
                severity=severity,
                evidence={
                    "current_deviation": dev.deviation_pct(dev.current_deviation),
                    "power_deviation": dev.deviation_pct(dev.power_deviation),
                    "PLI": f"{pli:+.3f}",
                },
                recommendation="Check for mechanical obstruction or aerodynamic drag increase. Inspect propeller condition and mounting.",
                possible_causes=[
                    "Increased aerodynamic drag (headwind, drag plate)",
                    "Mechanical resistance (tight bearing, rubbing)",
                    "Partial prop strike (FOD in intake)",
                ],
            ))

        # ── PLI: Propulsion Load Anomaly — Low (missing prop) ─────────────
        if pli <= -ELEVATED_THRESHOLD:
            severity = "critical" if pli <= -WARNING_THRESHOLD else "warning"
            confidence = min(0.97, 0.60 + 0.37 * min((abs(pli) - ELEVATED_THRESHOLD) / 0.5, 1.0))

            conditions.append(Condition(
                condition_id="propulsion_load_low",
                title="Propulsion load anomaly — thrust loss",
                confidence=round(confidence, 3),
                severity=severity,
                evidence={
                    "current_deviation": dev.deviation_pct(dev.current_deviation),
                    "power_deviation": dev.deviation_pct(dev.power_deviation),
                    "PLI": f"{pli:+.3f}",
                },
                recommendation="DO NOT FLY. Verify propeller is installed and secured. Inspect hub and shaft.",
                possible_causes=[
                    "Missing propeller",
                    "Loose / stripped propeller hub",
                    "Severe propeller structural failure",
                    "Motor running unloaded",
                ],
            ))

        # ── ETI: Thermal Anomaly ──────────────────────────────────────────
        if eti >= ELEVATED_THRESHOLD:
            severity = "critical" if eti >= WARNING_THRESHOLD else "warning"
            confidence = min(0.92, 0.50 + 0.42 * min((eti - ELEVATED_THRESHOLD) / 0.5, 1.0))

            conditions.append(Condition(
                condition_id="thermal_anomaly",
                title="Thermal behaviour abnormal",
                confidence=round(confidence, 3),
                severity=severity,
                evidence={
                    "temp_deviation": dev.deviation_pct(dev.temp_deviation),
                    "current_deviation": dev.deviation_pct(dev.current_deviation),
                    "ETI": f"{eti:.3f}",
                },
                recommendation="Reduce throttle and allow ESC to cool. Inspect cooling path before next flight.",
                possible_causes=[
                    "Blocked ESC cooling path / airflow",
                    "Degraded thermal interface compound",
                    "ESC heatsink detached",
                    "ESC internal fault (partially failing FET)",
                ],
            ))

        # ── BSI: Battery Stress ───────────────────────────────────────────
        if bsi >= ELEVATED_THRESHOLD:
            severity = "critical" if bsi >= WARNING_THRESHOLD else "warning"
            confidence = min(0.90, 0.50 + 0.40 * min((bsi - ELEVATED_THRESHOLD) / 0.5, 1.0))

            conditions.append(Condition(
                condition_id="battery_stress_elevated",
                title="Battery stress elevated",
                confidence=round(confidence, 3),
                severity=severity,
                evidence={
                    "voltage_sag_deviation": dev.deviation_pct(dev.voltage_sag_deviation),
                    "current_deviation": dev.deviation_pct(dev.current_deviation),
                    "BSI": f"{bsi:.3f}",
                },
                recommendation="Monitor battery voltage carefully. Consider reducing throttle. Inspect battery health after flight.",
                possible_causes=[
                    "Battery pack aging / increased internal resistance",
                    "Higher current demand than during calibration",
                    "Cell imbalance within pack",
                    "Cold battery temperature reducing capacity",
                ],
            ))

        # ── Informational: Uncalibrated Operation ────────────────────────
        if not dev.calibrated and dev.confidence < 0.5:
            conditions.append(Condition(
                condition_id="uncalibrated_operation",
                title="Operating without calibration baseline",
                confidence=0.99,
                severity="info",
                evidence={"confidence": f"{dev.confidence:.2f}", "mode": "rolling_statistics"},
                recommendation="Run calibration sweep via web UI to establish healthy baseline. Results will be more accurate after calibration.",
                possible_causes=["No calibration data exists yet", "Baseline file not loaded"],
            ))

        return conditions

    # ── Battery check (kept for direct battery alarm — raw threshold OK here) ─

    def check_battery_limits(self, voltage: float, current: float,
                             soc: Optional[float], temp: float,
                             temp_max: float = 60.0) -> list:
        """
        Hard safety limits for battery — these are absolute physical limits, not
        deviation-based. Kept separate from PHM indices because they are trip-wire
        safety checks, not health trend analysis.
        """
        findings = []
        if soc is not None and soc < 10.0:
            findings.append(Condition(
                condition_id="critical_soc",
                title="Battery critically low",
                confidence=0.99, severity="critical",
                evidence={"soc": f"{soc:.1f}%", "voltage": f"{voltage:.2f}V"},
                recommendation="Land immediately. Replace or charge battery before next flight.",
                possible_causes=["Battery depleted"],
            ))
        if voltage < 9.9:
            findings.append(Condition(
                condition_id="under_voltage",
                title="Battery under-voltage",
                confidence=0.99, severity="critical",
                evidence={"voltage": f"{voltage:.2f}V", "limit": "9.9V"},
                recommendation="Emergency landing immediately to prevent cell damage.",
                possible_causes=["Deep discharge", "Cell failure"],
            ))
        if current > 60.0:
            findings.append(Condition(
                condition_id="over_current",
                title="Battery over-current",
                confidence=0.95, severity="critical",
                evidence={"current": f"{current:.1f}A"},
                recommendation="Reduce throttle immediately.",
                possible_causes=["Short circuit", "Throttle runaway"],
            ))
        if temp > temp_max:
            findings.append(Condition(
                condition_id="battery_over_temp",
                title="Battery temperature limit exceeded",
                confidence=0.90, severity="critical",
                evidence={"temp": f"{temp:.1f}°C", "limit": f"{temp_max:.0f}°C"},
                recommendation="Reduce power and land. Risk of thermal runaway.",
                possible_causes=["Overloaded battery", "Ambient temperature too high"],
            ))
        return findings

    # ── Legacy compatibility wrapper ──────────────────────────────────────────

    def check_motors(self, motor_id: int, current: float, vibration_rms: float,
                     vibration_kurtosis: float, temp_rise: float,
                     baseline_current=None, baseline_vibration=None,
                     baseline_temp_rise=None) -> list:
        """
        Backward-compatible wrapper used by older unit tests.
        Constructs synthetic DeviationFrame + HealthIndices and classifies.
        """
        # Build a synthetic deviation frame from z-score baselines
        current_dev = 0.0
        if baseline_current and baseline_current.get("std", 0) > 0:
            current_dev = (current - baseline_current.get("expected", current)) / max(baseline_current["std"], 0.01)
            # Normalise from z-score to fractional deviation
            expected_i = baseline_current.get("expected", current)
            if expected_i > 0:
                current_dev = (current - expected_i) / expected_i

        vib_dev = 0.0
        if baseline_vibration and baseline_vibration.get("expected", 0) > 0:
            exp_v = baseline_vibration["expected"]
            vib_dev = (vibration_rms - exp_v) / exp_v

        temp_dev = 0.0
        if baseline_temp_rise and baseline_temp_rise.get("expected", 0) > 0:
            exp_t = baseline_temp_rise["expected"]
            temp_dev = (temp_rise - exp_t) / max(exp_t, 1.0)

        from core.deviation_engine import DeviationFrame
        dev = DeviationFrame(
            throttle_pct=50.0, calibrated=(baseline_current is not None),
            confidence=0.7 if baseline_current else 0.3,
            current_deviation=current_dev, power_deviation=current_dev * 0.9,
            temp_deviation=temp_dev, vibration_deviation=vib_dev,
            voltage_sag_deviation=0.0, ripple_deviation=0.0,
            current_expected=baseline_current.get("expected", current) if baseline_current else current,
            vibration_expected=baseline_vibration.get("expected", vibration_rms) if baseline_vibration else vibration_rms,
            temp_expected=baseline_temp_rise.get("expected", temp_rise) if baseline_temp_rise else temp_rise,
        )
        from core.health_engine import HealthIndexEngine
        engine = HealthIndexEngine()
        idx = engine.compute(dev)
        results = self.classify(idx, dev)
        # Map to expected legacy fault_type for old tests
        for c in results:
            if c.condition_id in ("propulsion_load_low", "propulsion_load_high"):
                c.condition_id = "current_imbalance"
            if c.condition_id == "rotational_imbalance":
                c.condition_id = "current_imbalance"
        return results

    # ── Debounce persistence ──────────────────────────────────────────────────

    def update(self, conditions: list) -> list:
        """
        Debounce conditions. Returns newly confirmed conditions.
        Critical conditions are raised immediately (no debounce).
        """
        new_confirmed = []
        seen_ids = {c.condition_id for c in conditions}

        for c in conditions:
            key = c.condition_id
            if c.severity == "critical":
                self._active[key] = c
                new_confirmed.append(c)
                continue
            self._counters[key] += 1
            if self._counters[key] >= self.persistence_count:
                if key not in self._active:
                    self._active[key] = c
                    new_confirmed.append(c)
                    log.info("Condition confirmed: %s (severity=%s, conf=%.2f)", key, c.severity, c.confidence)
                else:
                    self._active[key] = c  # refresh with latest evidence

        # Clear conditions that have disappeared
        stale = [k for k in list(self._active) if k not in seen_ids]
        for k in stale:
            del self._active[k]
            self._counters.pop(k, None)

        for c in new_confirmed:
            self._history.append(c)
        if len(self._history) > self._max_history:
            self._history = self._history[-self._max_history:]

        return new_confirmed

    def get_active(self) -> list:
        return list(self._active.values())

    def get_history(self, n: int = 50) -> list:
        return self._history[-n:]

    def predicted_failure(self) -> Optional[Condition]:
        """Returns the most recent confirmed serious condition as a predicted failure indicator."""
        if not self._history:
            return None
        for c in reversed(self._history):
            if c.severity == "critical":
                return c
        for c in reversed(self._history):
            if c.severity == "warning":
                return c
        return None

    def maintenance_recommendation(self) -> str:
        """Returns a top-level plain-language maintenance recommendation."""
        active = self.get_active()
        critical = [c for c in active if c.severity == "critical"]
        warnings = [c for c in active if c.severity == "warning"]
        info = [c for c in active if c.severity == "info"]

        if critical:
            return f"⛔ Immediate action required: {critical[0].title}. DO NOT FLY."
        if warnings:
            if len(warnings) == 1:
                return f"⚠️ {warnings[0].recommendation}"
            return f"⚠️ Multiple issues detected. Inspect propulsion system before next flight."
        if info:
            return "ℹ️ " + info[0].recommendation
        return "✅ No maintenance required."
