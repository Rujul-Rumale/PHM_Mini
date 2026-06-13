import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import time
import pytest
from core.soc_estimator import SoCEstimator, ocv_to_soc, soc_to_ocv


class TestOCVLookup:
    def test_ocv_to_soc_fully_charged(self):
        assert ocv_to_soc(4.20) == 100.0
        assert ocv_to_soc(4.25) == 100.0

    def test_ocv_to_soc_fully_discharged(self):
        assert ocv_to_soc(3.30) == 0.0
        assert ocv_to_soc(3.20) == 0.0

    def test_ocv_to_soc_interpolate(self):
        soc = ocv_to_soc(3.85)
        assert 55 <= soc <= 65

    def test_ocv_to_soc_midpoint(self):
        soc = ocv_to_soc(3.70)
        assert soc == 36.0

    def test_soc_to_ocv_fully_charged(self):
        assert soc_to_ocv(100) == 4.20

    def test_soc_to_ocv_fully_discharged(self):
        assert soc_to_ocv(0) == 3.30

    def test_roundtrip(self):
        for v in [4.0, 3.85, 3.70, 3.55, 3.40]:
            soc = ocv_to_soc(v)
            v_back = soc_to_ocv(soc)
            assert abs(v_back - v) < 0.05


class TestSoCEstimator:
    def test_init_from_ocv(self):
        soc = SoCEstimator(capacity_mah=3300, cell_count=4)
        result = soc.update(16.8, 0.0)
        assert result == 100.0
        assert soc.initialized

    def test_init_low_voltage(self):
        soc = SoCEstimator(capacity_mah=3300, cell_count=4)
        result = soc.update(13.2, 0.0)
        assert result == 0.0

    def test_coulomb_counting_discharge(self):
        soc = SoCEstimator(capacity_mah=100, cell_count=4)
        soc.update(16.8, 0.0)
        time.sleep(0.5)
        result = soc.update(16.5, 30.0)
        assert result < 100.0

    def test_coulomb_counting_multiple_steps(self):
        soc = SoCEstimator(capacity_mah=100, cell_count=4)
        soc.update(16.8, 0.0)
        total = 100.0
        for _ in range(20):
            time.sleep(0.1)
            total = soc.update(16.5, 30.0)
        assert total < 85.0

    def test_soft_correction_near_zero_current(self):
        soc = SoCEstimator(capacity_mah=3300, cell_count=4)
        soc.update(16.8, 10.0)
        time.sleep(0.01)
        soc.update(16.5, 10.0)
        time.sleep(0.01)
        result = soc.update(16.8, 0.1)
        assert result > 0.0

    def test_discharge_then_recover_stays_in_bounds(self):
        soc = SoCEstimator(capacity_mah=3300, cell_count=4)
        soc.update(16.8, 0.0)
        for _ in range(100):
            time.sleep(0.001)
            result = soc.update(16.0, 30.0)
        assert result >= 0.0
        assert result <= 100.0

    def test_soc_stable_at_constant_load(self):
        soc = SoCEstimator(capacity_mah=100, cell_count=4)
        soc.update(16.8, 0.0)
        values = []
        for _ in range(20):
            time.sleep(0.05)
            values.append(soc.update(16.5, 10.0))
        assert values[-1] < values[0]
