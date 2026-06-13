"""
Diagnostics — vehicle-agnostic.
All checks operate on PropulsionUnit objects.
Quad-only/fixed-wing-only checks gated by vehicle_features.
"""

import logging
from dataclasses import dataclass, field
from collections import defaultdict
from typing import Optional

from core.propulsion import PropulsionUnit

logger = logging.getLogger(__name__)

MAX_VIBRATION_RMS = 2.0
BEARING_KURTOSIS_THRESHOLD = 5.0
PROP_IMBALANCE_VIB_THRESHOLD = 0.8
PROP_IMBALANCE_CURRENT_Z = 2.0
ESC_TEMP_RISE_Z_THRESHOLD = 3.0
BATTERY_R_INTERNAL_GROWTH = 0.30
BATTERY_CAPACITY_LOSS = 0.20
BATTERY_TEMP_STRESS_HOURS = 30


@dataclass
class FaultDiagnosis:
    component: str
    fault_type: str
    confidence: float
    severity: str
    evidence: dict
    timestamp: float = 0.0


class Diagnostics:
    def __init__(self, persistence_count: int = 3):
        self.persistence_count = persistence_count
        self._counters: dict = defaultdict(int)
        self._active: dict = {}
        self._history: list[FaultDiagnosis] = []
        self._max_history = 500

    # ── Common diagnostics (all vehicle types) ────────────────

    def check_battery(self, voltage: float, current: float, soc: float,
                      r_internal: Optional[float], r_internal_baseline: Optional[float],
                      capacity_now: Optional[float], capacity_full: Optional[float],
                      temp: float, temp_max: float) -> list[FaultDiagnosis]:
        findings = []
        if soc is not None and soc < 10:
            findings.append(FaultDiagnosis(
                component="battery", fault_type="critical_soc",
                confidence=0.95, severity="critical",
                evidence={"soc": soc, "voltage": voltage}))
        if voltage < 9.9:
            findings.append(FaultDiagnosis(
                component="battery", fault_type="under_voltage",
                confidence=0.95, severity="critical",
                evidence={"voltage": voltage}))
        if current > 60.0:
            findings.append(FaultDiagnosis(
                component="battery", fault_type="over_current",
                confidence=0.90, severity="critical",
                evidence={"current": current}))
        if temp > temp_max:
            findings.append(FaultDiagnosis(
                component="battery", fault_type="over_temp",
                confidence=0.85, severity="warning",
                evidence={"temp": temp, "max": temp_max}))
        if r_internal is not None and r_internal_baseline is not None:
            growth = (r_internal - r_internal_baseline) / max(r_internal_baseline, 1e-6)
            if growth > BATTERY_R_INTERNAL_GROWTH:
                findings.append(FaultDiagnosis(
                    component="battery", fault_type="r_internal_degraded",
                    confidence=0.75, severity="warning",
                    evidence={"growth_pct": growth * 100,
                              "r_internal": r_internal, "baseline": r_internal_baseline}))
        if capacity_now is not None and capacity_full is not None:
            loss = (capacity_full - capacity_now) / max(capacity_full, 1e-6)
            if loss > BATTERY_CAPACITY_LOSS:
                findings.append(FaultDiagnosis(
                    component="battery", fault_type="capacity_loss",
                    confidence=0.70, severity="warning",
                    evidence={"loss_pct": loss * 100}))
        return findings

    def check_propulsion(self, unit: PropulsionUnit) -> list[FaultDiagnosis]:
        """Common propulsion diagnostics — prop damage, mechanical degradation, ESC faults."""
        findings = []
        comp = unit.name

        if unit.vibration_rms > MAX_VIBRATION_RMS:
            findings.append(FaultDiagnosis(
                component=comp, fault_type="excessive_vibration",
                confidence=0.90, severity="critical",
                evidence={"vibration_rms": unit.vibration_rms}))

        # Prop damage: vibration + current drop
        if unit.current < 0.3 and unit.vibration_rms > 0.1:
            findings.append(FaultDiagnosis(
                component=comp, fault_type="prop_damage",
                confidence=0.80, severity="warning",
                evidence={"current": unit.current, "vibration_rms": unit.vibration_rms}))

        # Current anomaly from baseline
        if unit.baseline_current and unit.baseline_current.get("z_score", 0) > PROP_IMBALANCE_CURRENT_Z:
            findings.append(FaultDiagnosis(
                component=comp, fault_type="current_anomaly",
                confidence=0.70, severity="warning",
                evidence={"z_score": unit.baseline_current["z_score"],
                          "expected": unit.baseline_current["expected"]}))

        # Vibration anomaly
        if unit.baseline_vibration and unit.baseline_vibration["z_score"] > 2.0:
            findings.append(FaultDiagnosis(
                component=comp, fault_type="vibration_anomaly",
                confidence=0.75, severity="warning",
                evidence={"z_score": unit.baseline_vibration["z_score"],
                          "expected": unit.baseline_vibration["expected"]}))

        # Bearing wear (kurtosis)
        if unit.vibration_kurtosis > BEARING_KURTOSIS_THRESHOLD:
            findings.append(FaultDiagnosis(
                component=comp, fault_type="bearing_wear",
                confidence=0.85, severity="warning",
                evidence={"kurtosis": unit.vibration_kurtosis}))

        # ESC thermal fault
        if unit.baseline_temp_rise and unit.baseline_temp_rise["z_score"] > ESC_TEMP_RISE_Z_THRESHOLD:
            findings.append(FaultDiagnosis(
                component=comp, fault_type="esc_thermal_fault",
                confidence=0.80, severity="warning",
                evidence={"temp_rise_z": unit.baseline_temp_rise["z_score"],
                          "expected": unit.baseline_temp_rise["expected"]}))

        return findings

    # ── Quad-only ─────────────────────────────────────────────

    def check_motor_imbalance(self, units: list[PropulsionUnit]) -> list[FaultDiagnosis]:
        """Only enabled when vehicle_profile.features.enable_motor_balance == true."""
        findings = []
        currents = [u.current for u in units if u is not None]
        if len(currents) < 2:
            return findings
        mean_i = sum(currents) / len(currents)
        if mean_i < 0.5:
            return findings
        for u in units:
            imbalance = abs(u.current - mean_i) / mean_i
            if imbalance > 0.40:
                findings.append(FaultDiagnosis(
                    component=u.name, fault_type="motor_imbalance",
                    confidence=0.80, severity="warning",
                    evidence={"imbalance_ratio": round(imbalance, 3),
                              "mean_current": round(mean_i, 3),
                              "unit_current": round(u.current, 3)}))
        return findings

    # ── Fixed-wing-only ───────────────────────────────────────

    def check_efficiency_loss(self, unit: PropulsionUnit, throttle: float,
                               gps_speed: Optional[float] = None) -> list[FaultDiagnosis]:
        """Only enabled when vehicle_profile.features.enable_efficiency_tracking == true."""
        findings = []
        if throttle < 0.1 or unit.current < 0.5:
            return findings
        # Efficiency proxy: thrust (inferred from throttle) / current
        eff = throttle / max(unit.current, 0.01)
        if unit.baseline_current:
            expected_current = unit.baseline_current.get("expected", unit.current)
            eff_expected = throttle / max(expected_current, 0.01)
            if eff_expected > 0 and eff / eff_expected < 0.7:
                findings.append(FaultDiagnosis(
                    component=unit.name, fault_type="propulsive_efficiency_loss",
                    confidence=0.70, severity="warning",
                    evidence={"efficiency_ratio": round(eff / eff_expected, 3),
                              "throttle": throttle, "current": unit.current}))
        return findings

    # ── Persistence ───────────────────────────────────────────

    def update(self, diagnoses: list[FaultDiagnosis]) -> list[FaultDiagnosis]:
        new_active = []
        for d in diagnoses:
            key = (d.component, d.fault_type)
            if d.severity == "critical":
                self._active[key] = d
                new_active.append(d)
                continue
            self._counters[key] += 1
            if self._counters[key] >= self.persistence_count:
                if key not in self._active:
                    self._active[key] = d
                    new_active.append(d)
                    logger.info("Fault confirmed: %s on %s (conf=%.2f)",
                                d.fault_type, d.component, d.confidence)
        cleanup_keys = set()
        for key in list(self._active):
            comp, ft = key
            if not any(k == key for k in [(d.component, d.fault_type) for d in diagnoses]):
                cleanup_keys.add(key)
        for k in cleanup_keys:
            del self._active[k]
            self._counters.pop(k, None)

        for d in new_active:
            self._history.append(d)
        if len(self._history) > self._max_history:
            self._history = self._history[-self._max_history:]

        return new_active

    def get_active(self) -> list[FaultDiagnosis]:
        return list(self._active.values())

    def get_history(self, n: int = 50) -> list[FaultDiagnosis]:
        return self._history[-n:]

    def predicted_failure(self) -> Optional[FaultDiagnosis]:
        if not self._history:
            return None
        for d in reversed(self._history):
            if d.severity == "critical":
                return d
        for d in reversed(self._history):
            if d.severity == "warning":
                return d
        return self._history[-1]
