"""Calibration — no-op for simulation. Was loading stale pyc that set CALIBRATING status."""


def apply_calibration_to_cfg(cfg: dict) -> dict:
    """Return config unchanged. Calibration requires hardware + sweep."""
    return cfg
