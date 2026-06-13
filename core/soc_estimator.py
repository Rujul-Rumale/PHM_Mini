"""
Battery SoC Estimator
Method: Coulomb counting with OCV-based initial SoC and periodic correction.

OCV-SoC table is for LiPo (generic 4S values — adjust per cell chemistry).
Assumption: table is for a single cell; multiply voltage thresholds by cell_count.
"""

import time
from typing import Optional


# OCV → SoC lookup table for single LiPo cell (V, %)
# Source: typical LiPo discharge curve, rested voltage
_OCV_TABLE = [
    (4.20, 100), (4.15, 95), (4.10, 90), (4.05, 85),
    (4.00, 80),  (3.95, 75), (3.90, 68), (3.85, 60),
    (3.80, 52),  (3.75, 44), (3.70, 36), (3.65, 28),
    (3.60, 20),  (3.55, 13), (3.50, 7),  (3.40, 3),
    (3.30, 0),
]


def ocv_to_soc(cell_voltage: float) -> float:
    """Linear interpolation on OCV table."""
    if cell_voltage >= _OCV_TABLE[0][0]:
        return 100.0
    if cell_voltage <= _OCV_TABLE[-1][0]:
        return 0.0
    for i in range(len(_OCV_TABLE) - 1):
        v_hi, s_hi = _OCV_TABLE[i]
        v_lo, s_lo = _OCV_TABLE[i + 1]
        if v_lo <= cell_voltage <= v_hi:
            ratio = (cell_voltage - v_lo) / (v_hi - v_lo)
            return s_lo + ratio * (s_hi - s_lo)
    return 0.0


def soc_to_ocv(cell_soc: float) -> float:
    """Reverse lookup: SoC to cell OCV by interpolating OCV table."""
    if cell_soc >= _OCV_TABLE[0][1]:
        return _OCV_TABLE[0][0]
    if cell_soc <= _OCV_TABLE[-1][1]:
        return _OCV_TABLE[-1][0]
    for i in range(len(_OCV_TABLE) - 1):
        v_hi, s_hi = _OCV_TABLE[i]
        v_lo, s_lo = _OCV_TABLE[i + 1]
        if s_lo <= cell_soc <= s_hi:
            r = (cell_soc - s_lo) / (s_hi - s_lo) if s_hi != s_lo else 0
            return v_lo + r * (v_hi - v_lo)
    return _OCV_TABLE[-1][0]


class SoCEstimator:
    def __init__(self, capacity_mah: float, cell_count: int):
        self._capacity_as = capacity_mah * 3.6  # mAh → A·s
        self._cell_count = cell_count
        self._soc: Optional[float] = None
        self._charge_as: Optional[float] = None  # accumulated charge
        self._last_t: Optional[float] = None
        self._initialized = False

    def update(self, voltage: float, current: float) -> float:
        """
        voltage: pack voltage (V)
        current: discharge current (A, positive = discharging)
        Returns SoC in percent.
        """
        now = time.time()
        cell_v = voltage / self._cell_count

        if not self._initialized:
            # Initialize from OCV (assumes rested battery at startup)
            self._soc = ocv_to_soc(cell_v)
            self._charge_as = self._soc / 100.0 * self._capacity_as
            self._last_t = now
            self._initialized = True
            return self._soc

        dt = now - self._last_t
        self._last_t = now

        # Coulomb counting: subtract discharged charge
        self._charge_as -= current * dt
        self._charge_as = max(0.0, min(self._charge_as, self._capacity_as))
        self._soc = (self._charge_as / self._capacity_as) * 100.0

        # Soft correction toward OCV estimate when current is near zero
        if abs(current) < 0.5:
            ocv_soc = ocv_to_soc(cell_v)
            self._soc = 0.95 * self._soc + 0.05 * ocv_soc  # slow drift correction

        return round(self._soc, 2)

    @property
    def initialized(self) -> bool:
        return self._initialized
