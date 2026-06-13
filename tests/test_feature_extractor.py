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
