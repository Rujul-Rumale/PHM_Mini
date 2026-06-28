import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from core.feature_extractor import (
    FeatureExtractor, ElectricalBatteryFeatures, ElectricalMotorFeatures,
    ThermalFeatures, VibrationFeatures,
    _linear_slope, _kurtosis,
)


class TestLinearSlope:
    def test_rising(self):
        v = list(range(50))
        s = _linear_slope(v)
        assert s > 0

    def test_falling(self):
        v = list(reversed(range(50)))
        s = _linear_slope(v)
        assert s < 0

    def test_flat(self):
        v = [5.0] * 50
        s = _linear_slope(v)
        assert abs(s) < 1e-6

    def test_too_few_points(self):
        v = [1, 2]
        s = _linear_slope(v)
        assert s == 0.0


class TestKurtosis:
    def test_normal_approx_zero(self):
        v = [0] * 5 + [1] * 5 + [-1] * 5
        k = _kurtosis(v)
        assert abs(k) < 3.0

    def test_too_few_points(self):
        v = [1, 2, 3]
        k = _kurtosis(v)
        assert k == 0.0

    def test_zero_stddev(self):
        v = [5.0] * 10
        k = _kurtosis(v)
        assert k == 0.0


class TestFeatureExtractor:
    def test_init(self):
        fe = FeatureExtractor(motor_count=4)
        assert fe is not None

    def test_electrical_features_battery(self):
        fe = FeatureExtractor(motor_count=4)
        for _ in range(60):
            batt_feat, _ = fe.electrical_features(11.1, 10.0, 111.0,
                                                  [5.0, 5.0, 5.0, 5.0])
        assert isinstance(batt_feat, ElectricalBatteryFeatures)
        assert batt_feat.voltage == 11.1
        assert batt_feat.current == 10.0
        assert batt_feat.power == 111.0

    def test_battery_variance_after_fill(self):
        fe = FeatureExtractor(motor_count=4)
        for _ in range(60):
            fe.electrical_features(11.1, 10.0, 111.0, [5.0, 5.0, 5.0, 5.0])
        batt_feat, _ = fe.electrical_features(10.8, 15.0, 162.0,
                                              [5.0, 5.0, 5.0, 5.0])
        assert batt_feat.voltage_variance > 0

    def test_electrical_motors_returns_list(self):
        fe = FeatureExtractor(motor_count=4)
        _, motor_feats = fe.electrical_features(11.1, 10.0, 111.0,
                                                [5.0, 5.0, 5.0, 5.0])
        assert len(motor_feats) == 4
        assert all(isinstance(f, ElectricalMotorFeatures) for f in motor_feats)

    def test_motor_imbalance_detected(self):
        fe = FeatureExtractor(motor_count=2)
        for _ in range(60):
            fe.electrical_features(11.1, 10.0, 111.0, [10.0, 2.0])
        _, motor_feats = fe.electrical_features(11.1, 10.0, 111.0, [10.0, 2.0])
        assert motor_feats[0].imbalance_ratio > 0
        assert motor_feats[1].imbalance_ratio > 0

    def test_motor_none_current_returns_safe_default(self):
        fe = FeatureExtractor(motor_count=2)
        _, motor_feats = fe.electrical_features(11.1, 10.0, 111.0,
                                                [None, 5.0])
        assert motor_feats[0].current == 0.0
        assert motor_feats[0].imbalance_ratio == 0.0

    def test_battery_load_voltage_drop(self):
        fe = FeatureExtractor(motor_count=4)
        fe.electrical_features(11.1, 0.5, 5.55, [5.0, 5.0, 5.0, 5.0])
        batt_feat, _ = fe.electrical_features(10.5, 30.0, 315.0,
                                              [5.0, 5.0, 5.0, 5.0])
        assert batt_feat.load_voltage_drop > 0

    def test_battery_ripple_amplitude(self):
        fe = FeatureExtractor(motor_count=4)
        for v in [11.1, 11.0, 10.9, 10.8, 11.1, 11.2]:
            fe.electrical_features(v, 10.0, 100.0, [5.0, 5.0, 5.0, 5.0])
        batt_feat, _ = fe.electrical_features(11.0, 10.0, 110.0,
                                              [5.0, 5.0, 5.0, 5.0])
        assert batt_feat.ripple_amplitude > 0

    def test_motor_trend_rising(self):
        fe = FeatureExtractor(motor_count=1)
        for c in [1.0 + i * 0.1 for i in range(60)]:
            fe.electrical_features(11.1, 10.0, 111.0, [c])
        _, motor_feats = fe.electrical_features(11.1, 10.0, 111.0, [6.0])
        assert motor_feats[0].current_trend > 0

    def test_thermal_features(self):
        fe = FeatureExtractor(motor_count=4)
        tf = fe.thermal_features([35.0, 36.0, 34.0, 35.0], ambient=25.0)
        assert isinstance(tf, ThermalFeatures)
        assert len(tf.motor_temps) == 4
        assert tf.temp_rise_per_amp[0] > 0

    def test_vibration_features(self):
        fe = FeatureExtractor(motor_count=4)
        samples = [0.1, 0.2, 0.15, 0.3, 0.12, 0.18, 0.22, 0.09, 0.25, 0.14]
        vf = fe.vibration_features(motor_id=1, accel_samples=samples)
        assert isinstance(vf, VibrationFeatures)
        assert vf.motor_id == 1
        assert vf.rms is not None
        assert vf.rms > 0

    def test_vibration_features_empty(self):
        fe = FeatureExtractor(motor_count=4)
        vf = fe.vibration_features(motor_id=1, accel_samples=[])
        assert vf is None

    def test_spike_count_zero_after_fill(self):
        fe = FeatureExtractor(motor_count=1)
        for _ in range(60):
            fe.electrical_features(11.1, 10.0, 111.0, [5.0])
        _, motor_feats = fe.electrical_features(11.1, 10.0, 111.0, [5.0])
        assert motor_feats[0].spike_count == 0


# ── DeviationEngine tests ──────────────────────────────────────────────────────

import json
import os
import tempfile
from core.baseline_manager import BaselineManager, FieldStats
from core.deviation_engine import DeviationEngine, DeviationFrame, _safe_deviation
from core.telemetry_frame import TelemetryFrame


def make_baseline_json(steps=None) -> str:
    """Create a minimal valid calibration baseline JSON string."""
    steps = steps or [0, 50, 100]
    data = {}
    for s in steps:
        factor = s / 100.0 + 0.1
        data[str(s)] = {
            "esc_current": {"mean": 5.0 * factor, "std": 0.15, "min": 4.5 * factor, "max": 5.5 * factor, "n": 50},
            "battery_voltage": {"mean": 11.4, "std": 0.05, "min": 11.2, "max": 11.6, "n": 50},
            "battery_current": {"mean": 6.0 * factor, "std": 0.20, "min": 5.5 * factor, "max": 6.5 * factor, "n": 50},
            "battery_power": {"mean": 70.0 * factor, "std": 2.0, "min": 65.0, "max": 78.0, "n": 50},
            "esc_temp": {"mean": 30.0 + s * 0.1, "std": 1.0, "min": 28.0, "max": 35.0, "n": 50},
            "vibration_rms": {"mean": 0.10 + s * 0.002, "std": 0.02, "min": 0.08, "max": 0.14, "n": 50},
            "voltage_sag": {"mean": 11.4 - s * 0.003, "std": 0.05, "min": 10.9, "max": 11.4, "n": 50},
        }
    return json.dumps({
        "calibrated": True,
        "calibrated_at": "2025-01-01T00:00:00+00:00",
        "n_runs": 3,
        "throttle_steps": steps,
        "data": data,
    })


def make_frame(esc_current=5.0, battery_voltage=11.4, battery_current=6.0,
               battery_power=70.0, esc_temp=32.0, imu_rms=0.12,
               imu_peak_freq=45.0, throttle_pct=0.5):
    frame = TelemetryFrame.__new__(TelemetryFrame)
    frame.timestamp = 1000.0
    frame.flight_id = 1
    frame.flight_state = "RUNNING"
    frame.battery_voltage = battery_voltage
    frame.battery_current = battery_current
    frame.battery_power = battery_power
    frame.battery_sensor_quality = "ONLINE"
    frame.esc_current = esc_current
    frame.esc_current_quality = "ONLINE"
    frame.esc_temp = esc_temp
    frame.esc_temp_filtered = esc_temp
    frame.esc_temp_quality = "ONLINE"
    frame.imu_accel = {"x": 0.0, "y": 0.0, "z": 1.0}
    frame.imu_quality = "ONLINE"
    frame.imu_rms = imu_rms
    frame.imu_kurtosis = 0.0
    frame.imu_peak_freq = imu_peak_freq
    frame.throttle_pct = throttle_pct
    frame.throttle_source = "operator"
    frame.battery_current_filtered = battery_current
    frame.soc = 80.0
    frame.features = None
    return frame


class TestBaselineManager:
    def _write_and_load(self, json_str):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as fh:
            fh.write(json_str)
            path = fh.name
        mgr = BaselineManager(filepath=path)
        os.unlink(path)
        return mgr

    def test_loads_valid_baseline(self):
        mgr = self._write_and_load(make_baseline_json([0, 50, 100]))
        assert mgr.is_calibrated()
        assert len(mgr._throttle_steps) == 3

    def test_exact_step_lookup(self):
        mgr = self._write_and_load(make_baseline_json([0, 50, 100]))
        stats = mgr.get_expected(50.0, "esc_current")
        assert stats is not None
        assert stats.mean > 0

    def test_interpolated_step_lookup(self):
        mgr = self._write_and_load(make_baseline_json([0, 50, 100]))
        stats_lo = mgr.get_expected(0.0, "esc_current")
        stats_hi = mgr.get_expected(100.0, "esc_current")
        stats_mid = mgr.get_expected(50.0, "esc_current")
        # Midpoint mean should be between lo and hi
        assert stats_lo.mean < stats_mid.mean < stats_hi.mean

    def test_rolling_fallback_when_uncalibrated(self):
        mgr = BaselineManager(filepath="nonexistent_file_zzz.json")
        assert not mgr.is_calibrated()
        # Feed some samples
        for v in [5.0, 5.1, 4.9, 5.2, 4.8] * 5:
            mgr.push_sample("esc_current", v)
        stats = mgr.get_expected(50.0, "esc_current")
        assert stats is not None
        assert abs(stats.mean - 5.0) < 0.5

    def test_confidence_at_exact_step(self):
        mgr = self._write_and_load(make_baseline_json([0, 50, 100]))
        conf = mgr.interpolation_confidence(50.0)
        assert conf == pytest.approx(1.0)

    def test_confidence_low_when_uncalibrated(self):
        mgr = BaselineManager(filepath="nonexistent_zzz.json")
        conf = mgr.interpolation_confidence(50.0)
        assert conf < 0.5

    def test_old_schema_rejected(self):
        old_schema = json.dumps({"motors": {"0": {}}, "battery": {}})
        mgr = self._write_and_load(old_schema)
        assert not mgr.is_calibrated()

    def test_uncalibrated_flag_false(self):
        """baseline.json with calibrated=false → uncalibrated mode."""
        stub = json.dumps({"calibrated": False, "data": {}})
        mgr = self._write_and_load(stub)
        assert not mgr.is_calibrated()


class TestDeviationEngine:
    def _make_calibrated_mgr(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as fh:
            fh.write(make_baseline_json([0, 50, 100]))
            path = fh.name
        mgr = BaselineManager(filepath=path)
        os.unlink(path)
        return mgr

    def test_healthy_deviations_near_zero(self):
        """Frame matching calibration exactly → all deviations ≈ 0."""
        mgr = self._make_calibrated_mgr()
        engine = DeviationEngine()
        # 50% throttle, expected esc_current ≈ 5.0 * 0.6 = 3.0 — use actual mean
        stats = mgr.get_expected(50.0, "esc_current")
        frame = make_frame(esc_current=stats.mean, throttle_pct=0.5)
        dev = engine.compute(frame, mgr)
        assert abs(dev.current_deviation) < 0.05

    def test_elevated_current_positive_deviation(self):
        """Current much higher than expected → positive current_deviation."""
        mgr = self._make_calibrated_mgr()
        engine = DeviationEngine()
        stats = mgr.get_expected(50.0, "esc_current")
        frame = make_frame(esc_current=stats.mean * 2.0, throttle_pct=0.5)
        dev = engine.compute(frame, mgr)
        assert dev.current_deviation > 0.5

    def test_missing_prop_negative_deviation(self):
        """Current drastically lower than expected → strongly negative current_deviation."""
        mgr = self._make_calibrated_mgr()
        engine = DeviationEngine()
        stats = mgr.get_expected(50.0, "esc_current")
        frame = make_frame(esc_current=stats.mean * 0.05, throttle_pct=0.5)
        dev = engine.compute(frame, mgr)
        assert dev.current_deviation < -0.5

    def test_calibrated_flag_set_correctly(self):
        """Calibrated manager → DeviationFrame.calibrated = True."""
        mgr = self._make_calibrated_mgr()
        engine = DeviationEngine()
        frame = make_frame()
        dev = engine.compute(frame, mgr)
        assert dev.calibrated is True

    def test_uncalibrated_falls_back(self):
        """Uncalibrated manager → DeviationFrame.calibrated = False."""
        mgr = BaselineManager(filepath="nonexistent_zzz.json")
        engine = DeviationEngine()
        frame = make_frame()
        dev = engine.compute(frame, mgr)
        assert dev.calibrated is False

    def test_safe_deviation_zero_expected(self):
        """_safe_deviation with near-zero expected returns 0."""
        result = _safe_deviation(5.0, 0.0)
        assert result == 0.0

    def test_safe_deviation_clamped(self):
        """_safe_deviation clamps to [-10, 10]."""
        result = _safe_deviation(1000.0, 0.001)
        assert result == 10.0

    def test_deviation_to_dict_fields(self):
        """DeviationFrame.to_dict() contains all expected keys."""
        mgr = self._make_calibrated_mgr()
        engine = DeviationEngine()
        frame = make_frame()
        dev = engine.compute(frame, mgr)
        d = dev.to_dict()
        for key in ["throttle_pct", "calibrated", "confidence",
                    "current_deviation", "power_deviation", "temp_deviation",
                    "vibration_deviation", "voltage_sag_deviation"]:
            assert key in d, f"Missing key: {key}"

