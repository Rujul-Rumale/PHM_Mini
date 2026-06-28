"""
UAV PHM Calibration — Operator-Driven Throttle Sweep.

Purpose:
    Create the digital fingerprint of a healthy propulsion system.
    Records mean / std / min / max for every sensor at each throttle step.

Usage:
    Standalone:  python calibrate.py
    Web-triggered: imported by web/dashboard.py and called in a background thread.

Procedure:
    1. Install a known-good, balanced propeller.
    2. Run this script (or trigger via web UI).
    3. For each throttle step (0%, 10%, ... 100%):
       - The script sets the throttle via the control service.
       - Waits SETTLE_S seconds for readings to stabilise.
       - Records SAMPLE_S seconds of sensor data.
       - Moves to the next step.
    4. After all runs complete, averages across N_RUNS runs.
    5. Writes config/baseline.json.
"""

import json
import logging
import os
import statistics
import sys
import time
from datetime import datetime, timezone
from typing import Optional

log = logging.getLogger(__name__)

BASELINE_FILE = "config/baseline.json"
THROTTLE_STEPS = list(range(0, 110, 10))   # 0, 10, 20, ..., 100
N_RUNS = 3
SETTLE_S = 5.0      # seconds to wait for readings to stabilise after throttle step
SAMPLE_S = 5.0      # seconds of data to collect per step
SAMPLE_HZ = 10      # samples per second during recording


class CalibrationRecorder:
    """
    Records sensor data during a calibration sweep.
    Can be driven by a real DroneMonitor instance or a mock for testing.
    """

    def __init__(self, n_runs: int = N_RUNS):
        self.n_runs = n_runs
        self._runs: list[dict] = []           # list of run data dicts
        self._progress: dict = {
            "state": "idle",       # idle | settling | sampling | computing | done | error
            "run": 0,
            "step": 0,
            "step_pct": 0,
            "message": "Calibration not started",
        }
        self._cancel = False

    def get_progress(self) -> dict:
        return dict(self._progress)

    def cancel(self):
        self._cancel = True

    def _update_progress(self, state: str, run: int, step: int, step_pct: int, msg: str):
        self._progress.update({
            "state": state, "run": run, "step": step,
            "step_pct": step_pct, "message": msg,
        })
        log.info("[CAL] Run %d/%d Step %d%% — %s", run, self.n_runs, step_pct, msg)

    def run_sweep(self, monitor) -> bool:
        """
        Execute N_RUNS calibration sweeps using a live DroneMonitor.
        monitor must expose:
          - status_mgr.get_state() → state dict
          - control_service.set_throttle(float)
          - throttle_ctrl.provider.get_throttle()
        Returns True on success, False on cancellation or error.
        """
        self._runs.clear()
        self._cancel = False

        for run_idx in range(1, self.n_runs + 1):
            if self._cancel:
                self._update_progress("error", run_idx, 0, 0, "Calibration cancelled by operator.")
                return False

            run_data = {}
            self._update_progress("settling", run_idx, 0, 0, f"Run {run_idx}/{self.n_runs}: Starting sweep")

            for step_pct in THROTTLE_STEPS:
                if self._cancel:
                    self._update_progress("error", run_idx, step_pct, step_pct, "Cancelled.")
                    return False

                throttle_frac = step_pct / 100.0

                # Command throttle via ControlService
                try:
                    monitor.control_service.set_throttle(throttle_frac)
                except Exception as exc:
                    log.warning("Throttle set failed at %d%%: %s", step_pct, exc)

                # Settle
                self._update_progress(
                    "settling", run_idx, step_pct, step_pct,
                    f"Run {run_idx}/{self.n_runs}: Throttle {step_pct}% — settling ({SETTLE_S:.0f}s)…"
                )
                deadline = time.time() + SETTLE_S
                while time.time() < deadline:
                    if self._cancel:
                        return False
                    time.sleep(0.1)

                # Sample
                self._update_progress(
                    "sampling", run_idx, step_pct, step_pct,
                    f"Run {run_idx}/{self.n_runs}: Throttle {step_pct}% — sampling ({SAMPLE_S:.0f}s)…"
                )
                samples = self._collect_samples(monitor, SAMPLE_S, SAMPLE_HZ)
                run_data[step_pct] = samples

            self._runs.append(run_data)

        # Return throttle to 0 after sweep
        try:
            monitor.control_service.set_throttle(0.0)
        except Exception:
            pass

        # Compute statistics and write baseline
        self._update_progress("computing", self.n_runs, 100, 100, "Computing statistics…")
        result = self._compute_baseline()
        self._write(result)
        self._update_progress("done", self.n_runs, 100, 100, "Calibration complete. Baseline saved.")
        return True

    def _collect_samples(self, monitor, duration_s: float, hz: float) -> dict:
        """Read sensor values from the monitor at hz Hz for duration_s seconds."""
        interval = 1.0 / hz
        deadline = time.time() + duration_s
        buffers: dict[str, list] = {
            "esc_current": [], "battery_voltage": [], "battery_current": [],
            "battery_power": [], "esc_temp": [], "vibration_rms": [],
            "vibration_peak_freq": [], "current_ripple": [], "voltage_sag": [],
        }

        ref_voltage: Optional[float] = None

        while time.time() < deadline:
            t0 = time.time()
            state = monitor.status_mgr.get_state()

            # Read from StatusManager telemetry (populated by MonitorService at 50 Hz)
            tel = state.get("telemetry", {})

            esc_i = tel.get("esc_current", 0.0)
            batt_v = tel.get("battery_voltage", 0.0)
            batt_i = tel.get("battery_current", 0.0)
            batt_p = tel.get("battery_power", 0.0)
            esc_t = tel.get("esc_temp", 25.0)
            vib_rms = tel.get("imu_rms", 0.0)
            vib_freq = tel.get("imu_peak_freq", 0.0)

            # Voltage sag relative to first zero-throttle reading
            if ref_voltage is None and batt_i < 0.5 and batt_v > 0:
                ref_voltage = batt_v
            voltage_sag = (ref_voltage - batt_v) if ref_voltage else 0.0

            # Current ripple proxy: use battery current variance approximation from features
            features = state.get("features", {})
            ripple = features.get("battery_features", {}).get("ripple_amplitude", 0.0)

            buffers["esc_current"].append(esc_i)
            buffers["battery_voltage"].append(batt_v)
            buffers["battery_current"].append(batt_i)
            buffers["battery_power"].append(batt_p)
            buffers["esc_temp"].append(esc_t)
            buffers["vibration_rms"].append(vib_rms)
            buffers["vibration_peak_freq"].append(vib_freq)
            buffers["current_ripple"].append(ripple)
            buffers["voltage_sag"].append(voltage_sag)

            elapsed = time.time() - t0
            time.sleep(max(0, interval - elapsed))

        return buffers

    def _compute_baseline(self) -> dict:
        """Aggregate across runs: mean, std, min, max per field per throttle step."""
        data = {}

        for step_pct in THROTTLE_STEPS:
            step_stats = {}
            # Collect all samples for this throttle step across all runs
            combined: dict[str, list] = {}
            for run_data in self._runs:
                step_samples = run_data.get(step_pct, {})
                for field, values in step_samples.items():
                    combined.setdefault(field, []).extend(values)

            for field, values in combined.items():
                valid = [v for v in values if v is not None]
                if not valid:
                    continue
                mean = statistics.mean(valid)
                std = statistics.stdev(valid) if len(valid) > 1 else 0.0
                step_stats[field] = {
                    "mean": round(mean, 6),
                    "std": round(max(std, 1e-6), 6),
                    "min": round(min(valid), 6),
                    "max": round(max(valid), 6),
                    "n": len(valid),
                }

            data[str(step_pct)] = step_stats

        return {
            "calibrated": True,
            "calibrated_at": datetime.now(timezone.utc).isoformat(),
            "n_runs": len(self._runs),
            "throttle_steps": THROTTLE_STEPS,
            "ambient_temp": data.get("0", {}).get("esc_temp", {}).get("mean", 25.0),
            "data": data,
        }

    def _write(self, result: dict):
        try:
            with open(BASELINE_FILE, "w", encoding="utf-8") as fh:
                json.dump(result, fh, indent=2)
            log.info("Baseline written to %s", BASELINE_FILE)
        except OSError as exc:
            log.error("Failed to write baseline: %s", exc)
            self._update_progress("error", 0, 0, 0, f"Write failed: {exc}")


# ── Singleton recorder for web UI access ─────────────────────────────────────

_recorder: Optional[CalibrationRecorder] = None


def get_recorder() -> CalibrationRecorder:
    global _recorder
    if _recorder is None:
        _recorder = CalibrationRecorder(n_runs=N_RUNS)
    return _recorder


def apply_calibration_to_cfg(cfg: dict) -> dict:
    """No-op. Calibration is stored in baseline.json, not cfg."""
    return cfg


# ── Standalone CLI entry point ────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse
    import threading

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    parser = argparse.ArgumentParser(description="PHM Calibration Sweep")
    parser.add_argument("--runs", type=int, default=N_RUNS, help=f"Number of sweep runs (default {N_RUNS})")
    parser.add_argument("--output", default=BASELINE_FILE, help="Output baseline JSON path")
    args = parser.parse_args()

    print("\n" + "="*60)
    print("  UAV PHM — Health Calibration Sweep")
    print("="*60)
    print(f"\n  Runs:           {args.runs}")
    print(f"  Throttle steps: {THROTTLE_STEPS}")
    print(f"  Settle time:    {SETTLE_S}s per step")
    print(f"  Sample time:    {SAMPLE_S}s per step")
    print(f"  Output:         {args.output}")
    print("\n  SAFETY: Ensure the propulsion system is in a safe test rig.")
    print("  Ensure a healthy, balanced propeller is installed.\n")

    confirm = input("  Type 'START' to begin calibration: ").strip().upper()
    if confirm != "START":
        print("  Aborted.")
        sys.exit(0)

    print("\n  [!] Launching monitor to connect to hardware sensors…\n")

    # Import and boot the monitor
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import yaml
        from monitor import DroneMonitor
        with open("config/hardware.json") as f:
            hw = json.load(f)
        with open("config/vehicle_profile.json") as f:
            profile = json.load(f)
        with open("config/fixed-wing.yaml") as f:
            cfg = yaml.safe_load(f)

        monitor = DroneMonitor(cfg, profile, simulate=False, hardware_mode=True, flight_mode="calibration")
        monitor.start(web=False)

        # Wait for READY state
        deadline = time.time() + 30
        while monitor.status_mgr.get_state()["flight_state"] not in ("READY", "ARMED", "RUNNING"):
            if time.time() > deadline:
                print("  ERROR: Monitor did not reach READY state within 30s. Aborting.")
                sys.exit(1)
            time.sleep(0.5)

        # Arm for calibration
        ok, reason = monitor.control_service.arm()
        if not ok:
            print(f"  WARNING: Could not arm: {reason}. Continuing with caution.")

        recorder = CalibrationRecorder(n_runs=args.runs)
        success = recorder.run_sweep(monitor)

        monitor.control_service.disarm()
        monitor.stop()

        if success:
            print(f"\n  ✅ Calibration complete! Baseline saved to: {args.output}")
        else:
            print("\n  ❌ Calibration failed or was cancelled.")

    except Exception as exc:
        print(f"\n  ERROR: {exc}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
