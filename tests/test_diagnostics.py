"""
PHM Diagnostics Tests — condition classification from health indices.

Tests operate on health indices and deviation frames, NOT raw sensor values.
"""

import pytest
from core.deviation_engine import DeviationFrame
from core.health_engine import HealthIndexEngine, HealthIndices
from core.diagnostics import Diagnostics, Condition


def make_dev(
    throttle_pct=50.0, calibrated=True, confidence=0.9,
    current_deviation=0.0, power_deviation=0.0,
    temp_deviation=0.0, vibration_deviation=0.0,
    voltage_sag_deviation=0.0, ripple_deviation=0.0,
    current_expected=5.0, power_expected=60.0,
    temp_expected=35.0, vibration_expected=0.12,
    voltage_sag_expected=11.4,
):
    return DeviationFrame(
        throttle_pct=throttle_pct, calibrated=calibrated, confidence=confidence,
        current_deviation=current_deviation, power_deviation=power_deviation,
        temp_deviation=temp_deviation, vibration_deviation=vibration_deviation,
        voltage_sag_deviation=voltage_sag_deviation, ripple_deviation=ripple_deviation,
        current_expected=current_expected, power_expected=power_expected,
        temp_expected=temp_expected, vibration_expected=vibration_expected,
        voltage_sag_expected=voltage_sag_expected,
    )


def make_indices(pbi=0.0, pli=0.0, eti=0.0, bsi=0.0,
                 calibrated=True, confidence=0.9):
    from core.health_engine import _band
    score = max(0.0, 100.0 * (
        1.0 - 0.40 * min(abs(pbi), 1.0)
          - 0.35 * min(abs(pli), 1.0)
          - 0.15 * min(abs(eti), 1.0)
          - 0.10 * min(abs(bsi), 1.0)
    ))
    return HealthIndices(
        pbi=pbi, pli=pli, eti=eti, bsi=bsi,
        health_score=score,
        pbi_band=_band(pbi), pli_band=_band(pli),
        eti_band=_band(eti), bsi_band=_band(bsi),
        calibrated=calibrated, confidence=confidence,
    )


class TestHealthIndexEngine:
    def test_healthy_all_zero(self):
        engine = HealthIndexEngine()
        dev = make_dev()  # all deviations zero
        idx = engine.compute(dev)
        assert idx.pbi == 0.0
        assert idx.pli == 0.0
        assert idx.eti == 0.0
        assert idx.bsi == 0.0
        assert idx.health_score == pytest.approx(100.0, abs=0.1)
        assert idx.pbi_band == "Normal"

    def test_pbi_vibration_dominant(self):
        """Large vibration with small current → high PBI."""
        engine = HealthIndexEngine()
        dev = make_dev(vibration_deviation=3.5, current_deviation=0.05)
        idx = engine.compute(dev)
        assert idx.pbi > 0.50, f"Expected PBI > 0.5, got {idx.pbi}"
        assert idx.pbi_band in ("Warning", "Critical")

    def test_pbi_suppressed_when_current_also_high(self):
        """Both vibration and current high → PBI partially suppressed (it's PLI territory)."""
        engine = HealthIndexEngine()
        dev_pure_imbalance = make_dev(vibration_deviation=2.0, current_deviation=0.05)
        dev_both_high = make_dev(vibration_deviation=2.0, current_deviation=2.0)
        idx_imbalance = engine.compute(dev_pure_imbalance)
        idx_both = engine.compute(dev_both_high)
        assert idx_imbalance.pbi > idx_both.pbi, "PBI should be suppressed when current also high"

    def test_pli_high_drag_positive(self):
        """High current + high power → positive PLI (overloaded)."""
        engine = HealthIndexEngine()
        dev = make_dev(current_deviation=0.6, power_deviation=0.6)
        idx = engine.compute(dev)
        assert idx.pli > 0.50, f"Expected PLI > 0.5, got {idx.pli}"

    def test_pli_missing_prop_negative(self):
        """Very low current + power → strongly negative PLI."""
        engine = HealthIndexEngine()
        dev = make_dev(current_deviation=-0.9, power_deviation=-0.9)
        idx = engine.compute(dev)
        assert idx.pli < -0.50, f"Expected PLI < -0.5, got {idx.pli}"

    def test_eti_thermal_anomaly(self):
        """High temp deviation with normal current → ETI elevated."""
        engine = HealthIndexEngine()
        dev = make_dev(temp_deviation=1.5, current_deviation=0.0)
        idx = engine.compute(dev)
        assert idx.eti > 0.50, f"Expected ETI > 0.5, got {idx.eti}"

    def test_eti_suppressed_by_current(self):
        """Temp and current both high → ETI lower (expected thermal from load)."""
        engine = HealthIndexEngine()
        dev_curr_only = make_dev(temp_deviation=0.8, current_deviation=1.0)
        dev_temp_only = make_dev(temp_deviation=0.8, current_deviation=0.0)
        idx_curr = engine.compute(dev_curr_only)
        idx_temp = engine.compute(dev_temp_only)
        assert idx_temp.eti > idx_curr.eti, "ETI should be lower when current explains the temperature rise"

    def test_bsi_voltage_sag(self):
        """Large voltage sag deviation → elevated BSI."""
        engine = HealthIndexEngine()
        dev = make_dev(voltage_sag_deviation=-0.8)   # voltage much lower than expected
        idx = engine.compute(dev)
        assert idx.bsi > 0.40, f"Expected BSI > 0.4, got {idx.bsi}"

    def test_health_score_degraded_by_high_pbi(self):
        """High PBI reduces overall health score."""
        engine = HealthIndexEngine()
        dev_healthy = make_dev()
        dev_imbalanced = make_dev(vibration_deviation=3.0)
        idx_h = engine.compute(dev_healthy)
        idx_i = engine.compute(dev_imbalanced)
        assert idx_i.health_score < idx_h.health_score


class TestDiagnostics:
    def test_no_conditions_healthy(self):
        """Healthy deviations → no conditions raised."""
        d = Diagnostics()
        dev = make_dev()
        idx = make_indices()
        conditions = d.classify(idx, dev)
        # Filter out informational uncalibrated
        operational = [c for c in conditions if c.severity != "info"]
        assert len(operational) == 0

    def test_rotational_imbalance_detected(self):
        """High PBI → rotational_imbalance condition."""
        d = Diagnostics()
        dev = make_dev(vibration_deviation=3.5, current_deviation=0.05)
        idx = make_indices(pbi=0.70)
        conditions = d.classify(idx, dev)
        ids = [c.condition_id for c in conditions]
        assert "rotational_imbalance" in ids

    def test_rotational_imbalance_has_evidence(self):
        """rotational_imbalance condition must include vibration and current evidence."""
        d = Diagnostics()
        dev = make_dev(vibration_deviation=3.0, current_deviation=0.04)
        idx = make_indices(pbi=0.65)
        conditions = d.classify(idx, dev)
        ri = next((c for c in conditions if c.condition_id == "rotational_imbalance"), None)
        assert ri is not None
        assert "vibration_deviation" in ri.evidence
        assert "current_deviation" in ri.evidence
        assert len(ri.possible_causes) >= 2

    def test_propulsion_load_high(self):
        """High PLI → propulsion_load_high condition."""
        d = Diagnostics()
        dev = make_dev(current_deviation=0.7, power_deviation=0.7)
        idx = make_indices(pli=0.70)
        conditions = d.classify(idx, dev)
        ids = [c.condition_id for c in conditions]
        assert "propulsion_load_high" in ids

    def test_propulsion_load_low_missing_prop(self):
        """Strongly negative PLI → propulsion_load_low condition."""
        d = Diagnostics()
        dev = make_dev(current_deviation=-0.92, power_deviation=-0.92)
        idx = make_indices(pli=-0.85)
        conditions = d.classify(idx, dev)
        ids = [c.condition_id for c in conditions]
        assert "propulsion_load_low" in ids

    def test_thermal_anomaly_detected(self):
        """High ETI → thermal_anomaly condition."""
        d = Diagnostics()
        dev = make_dev(temp_deviation=2.0, current_deviation=0.0)
        idx = make_indices(eti=0.80)
        conditions = d.classify(idx, dev)
        ids = [c.condition_id for c in conditions]
        assert "thermal_anomaly" in ids

    def test_battery_stress_detected(self):
        """High BSI → battery_stress_elevated condition."""
        d = Diagnostics()
        dev = make_dev(voltage_sag_deviation=-0.9, current_deviation=0.6)
        idx = make_indices(bsi=0.70)
        conditions = d.classify(idx, dev)
        ids = [c.condition_id for c in conditions]
        assert "battery_stress_elevated" in ids

    def test_uncalibrated_condition(self):
        """Uncalibrated operation → info condition."""
        d = Diagnostics()
        dev = make_dev(calibrated=False, confidence=0.3)
        idx = make_indices(calibrated=False)
        conditions = d.classify(idx, dev)
        info = [c for c in conditions if c.condition_id == "uncalibrated_operation"]
        assert len(info) == 1
        assert info[0].severity == "info"

    def test_debounce_suppresses_early_warnings(self):
        """Warning-level conditions should not appear until persistence_count evaluations."""
        d = Diagnostics(persistence_count=3)
        dev = make_dev(vibration_deviation=3.5)
        idx = make_indices(pbi=0.70)
        raw = d.classify(idx, dev)
        # First two updates should not confirm
        confirmed1 = d.update(raw)
        confirmed2 = d.update(raw)
        confirmed3 = d.update(raw)
        assert len(confirmed1) == 0, "Should not confirm on first evaluation"
        assert len(confirmed2) == 0, "Should not confirm on second evaluation"
        assert len(confirmed3) >= 1, "Should confirm on third evaluation"

    def test_critical_bypasses_debounce(self):
        """Critical conditions are raised immediately without debounce."""
        d = Diagnostics(persistence_count=3)
        dev = make_dev(current_deviation=-0.99, power_deviation=-0.99)
        idx = make_indices(pli=-0.95)
        raw = d.classify(idx, dev)
        # Force severity to critical
        for c in raw:
            c.condition_id = "propulsion_load_low"
            object.__setattr__(c, "severity", "critical") if hasattr(c, "__dataclass_fields__") else setattr(c, "severity", "critical")
        confirmed = d.update(raw)
        # At least the critical one should be in active
        assert len(d.get_active()) >= 1

    def test_condition_clears_when_resolved(self):
        """Once an active condition disappears from input, it should be removed from active."""
        d = Diagnostics(persistence_count=1)
        dev_bad = make_dev(vibration_deviation=3.5)
        idx_bad = make_indices(pbi=0.70)
        raw = d.classify(idx_bad, dev_bad)
        d.update(raw)
        assert len(d.get_active()) >= 1

        # Now pass healthy data
        dev_ok = make_dev()
        idx_ok = make_indices()
        raw_ok = d.classify(idx_ok, dev_ok)
        d.update(raw_ok)
        # Operational conditions should be cleared
        remaining = [c for c in d.get_active() if c.severity != "info"]
        assert len(remaining) == 0

    def test_maintenance_recommendation_no_issues(self):
        """Clean system → no maintenance recommendation."""
        d = Diagnostics()
        rec = d.maintenance_recommendation()
        assert "No maintenance required" in rec

    def test_maintenance_recommendation_warning(self):
        """Active warning → maintenance advisory includes recommendation text."""
        d = Diagnostics(persistence_count=1)
        dev = make_dev(vibration_deviation=3.5)
        idx = make_indices(pbi=0.70)
        raw = d.classify(idx, dev)
        d.update(raw)
        rec = d.maintenance_recommendation()
        # Should be a non-empty warning advisory
        assert len(rec) > 5

    def test_legacy_check_motors_wrapper(self):
        """Backward-compat check_motors still returns Condition objects."""
        d = Diagnostics()
        results = d.check_motors(
            0, current=15.0, vibration_rms=0.1,
            vibration_kurtosis=1.0, temp_rise=5.0,
            baseline_current={"z_score": 3.0, "expected": 5.0, "std": 0.5},
            baseline_vibration=None, baseline_temp_rise=None,
        )
        # Should return a list (possibly empty — depends on PLI/PBI from mapped deviations)
        assert isinstance(results, list)

    def test_battery_limits_critical_soc(self):
        """Critical SOC triggers critical battery condition."""
        d = Diagnostics()
        conditions = d.check_battery_limits(
            voltage=11.0, current=5.0, soc=5.0, temp=30.0
        )
        ids = [c.condition_id for c in conditions]
        assert "critical_soc" in ids

    def test_battery_limits_under_voltage(self):
        """Under-voltage triggers critical battery condition."""
        d = Diagnostics()
        conditions = d.check_battery_limits(
            voltage=9.5, current=3.0, soc=30.0, temp=30.0
        )
        ids = [c.condition_id for c in conditions]
        assert "under_voltage" in ids

    def test_condition_to_dict(self):
        """Condition.to_dict() returns all expected fields."""
        d = Diagnostics()
        dev = make_dev(vibration_deviation=3.0)
        idx = make_indices(pbi=0.65)
        conditions = d.classify(idx, dev)
        for c in conditions:
            d_dict = c.to_dict()
            assert "condition_id" in d_dict
            assert "title" in d_dict
            assert "confidence" in d_dict
            assert "severity" in d_dict
            assert "evidence" in d_dict
            assert "recommendation" in d_dict
            assert "possible_causes" in d_dict
