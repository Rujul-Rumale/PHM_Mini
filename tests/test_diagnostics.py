import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from core.diagnostics import Diagnostics, FaultDiagnosis


class TestDiagnostics:
    def test_init(self):
        d = Diagnostics(persistence_count=3)
        assert d.get_active() == []

    def test_check_battery_undervoltage_critical(self):
        d = Diagnostics()
        findings = d.check_battery(voltage=9.0, current=30.0, soc=50,
                                   r_internal=None, r_internal_baseline=None,
                                   capacity_now=None, capacity_full=None,
                                   temp=30.0, temp_max=50.0)
        under = [f for f in findings if f.fault_type == "under_voltage"]
        assert len(under) == 1
        assert under[0].severity == "critical"

    def test_check_battery_voltage_ok(self):
        d = Diagnostics()
        findings = d.check_battery(voltage=11.5, current=10.0, soc=80,
                                   r_internal=None, r_internal_baseline=None,
                                   capacity_now=None, capacity_full=None,
                                   temp=30.0, temp_max=50.0)
        assert len(findings) == 0

    def test_check_battery_critical_soc(self):
        d = Diagnostics()
        findings = d.check_battery(voltage=10.0, current=10.0, soc=5,
                                   r_internal=None, r_internal_baseline=None,
                                   capacity_now=None, capacity_full=None,
                                   temp=30.0, temp_max=50.0)
        soc_f = [f for f in findings if f.fault_type == "critical_soc"]
        assert len(soc_f) == 1

    def test_check_battery_overcurrent(self):
        d = Diagnostics()
        findings = d.check_battery(voltage=10.5, current=70.0, soc=50,
                                   r_internal=None, r_internal_baseline=None,
                                   capacity_now=None, capacity_full=None,
                                   temp=30.0, temp_max=50.0)
        oc = [f for f in findings if f.fault_type == "over_current"]
        assert len(oc) == 1

    def test_check_battery_over_temp(self):
        d = Diagnostics()
        findings = d.check_battery(voltage=11.0, current=10.0, soc=50,
                                   r_internal=None, r_internal_baseline=None,
                                   capacity_now=None, capacity_full=None,
                                   temp=55.0, temp_max=50.0)
        ot = [f for f in findings if f.fault_type == "over_temp"]
        assert len(ot) == 1

    def test_check_battery_r_internal_degraded(self):
        d = Diagnostics()
        findings = d.check_battery(voltage=11.0, current=20.0, soc=50,
                                   r_internal=0.040, r_internal_baseline=0.020,
                                   capacity_now=None, capacity_full=None,
                                   temp=30.0, temp_max=50.0)
        r = [f for f in findings if f.fault_type == "r_internal_degraded"]
        assert len(r) == 1

    def test_check_motors_excessive_vibration(self):
        d = Diagnostics()
        findings = d.check_motors(0, current=10.0, vibration_rms=3.0,
                                  vibration_kurtosis=1.0, temp_rise=10.0,
                                  baseline_current=None, baseline_vibration=None,
                                  baseline_temp_rise=None)
        ev = [f for f in findings if f.fault_type == "excessive_vibration"]
        assert len(ev) == 1

    def test_check_motors_prop_damage(self):
        d = Diagnostics()
        findings = d.check_motors(0, current=0.2, vibration_rms=0.5,
                                  vibration_kurtosis=1.0, temp_rise=5.0,
                                  baseline_current=None, baseline_vibration=None,
                                  baseline_temp_rise=None)
        pd = [f for f in findings if f.fault_type == "prop_damage"]
        assert len(pd) == 1

    def test_check_motors_bearing_wear(self):
        d = Diagnostics()
        findings = d.check_motors(0, current=10.0, vibration_rms=0.5,
                                  vibration_kurtosis=6.0, temp_rise=5.0,
                                  baseline_current=None, baseline_vibration=None,
                                  baseline_temp_rise=None)
        bw = [f for f in findings if f.fault_type == "bearing_wear"]
        assert len(bw) == 1

    def test_persistence_debounce(self):
        d = Diagnostics(persistence_count=3)
        args = dict(voltage=11.0, current=30.0, soc=50,
                    r_internal=0.040, r_internal_baseline=0.020,
                    capacity_now=None, capacity_full=None,
                    temp=30.0, temp_max=50.0)
        findings = d.check_battery(**args)
        new_active = d.update(findings)
        assert len(d.get_active()) == 0
        assert len(new_active) == 0

        d.update(findings)
        assert len(d.get_active()) == 0

        new_active = d.update(findings)
        assert len(d.get_active()) >= 1
        assert len(new_active) >= 1

    def test_fault_clears_when_condition_lifts(self):
        d = Diagnostics(persistence_count=2)
        args = dict(voltage=11.0, current=30.0, soc=50,
                    r_internal=0.040, r_internal_baseline=0.020,
                    capacity_now=None, capacity_full=None,
                    temp=30.0, temp_max=50.0)
        good_args = dict(voltage=11.5, current=10.0, soc=80,
                         r_internal=0.020, r_internal_baseline=0.020,
                         capacity_now=3300, capacity_full=3300,
                         temp=30.0, temp_max=50.0)
        for _ in range(2):
            d.update(d.check_battery(**args))
        assert len(d.get_active()) >= 1

        for _ in range(2):
            d.update(d.check_battery(**good_args))
        assert len(d.get_active()) == 0

    def test_prediction_returns_most_recent_critical(self):
        d = Diagnostics()
        d.update([
            FaultDiagnosis("battery", "over_temp", 0.8, "warning", {}),
            FaultDiagnosis("motor_0", "excessive_vibration", 0.9, "critical", {}),
        ])
        pred = d.predicted_failure()
        assert pred is not None
        assert pred.fault_type == "excessive_vibration"

    def test_prediction_no_history(self):
        d = Diagnostics()
        assert d.predicted_failure() is None

    def test_get_history(self):
        d = Diagnostics()
        for _ in range(3):
            d.update([
                FaultDiagnosis("battery", "over_temp", 0.8, "warning", {"temp": 55}),
            ])
        hist = d.get_history()
        assert len(hist) == 1
        assert hist[0].component == "battery"

    def test_motor_overcurrent_via_zscore(self):
        d = Diagnostics()
        findings = d.check_motors(0, current=15.0, vibration_rms=0.1,
                                  vibration_kurtosis=1.0, temp_rise=5.0,
                                  baseline_current={"z_score": 3.0, "expected": 5.0, "std": 0.5},
                                  baseline_vibration=None, baseline_temp_rise=None)
        ci = [f for f in findings if f.fault_type == "current_imbalance"]
        assert len(ci) == 1
