"""
UAV PHM Monitor — vehicle-agnostic, Raspberry Pi deployment ready.

Multithread architecture:
  Thread 1 (highest): fast_loop  — sensor reads, safety checks, blackbox buffer  (20-50 Hz)
  Thread 2 (medium):  health_loop — features, diagnostics, health engine, DB     (1 Hz)
  Thread 3 (low):     web_server  — Flask dashboard                              (on demand)
"""

import argparse
import json
import logging
import time
import sys
import os
import signal
import threading
from collections import deque
from datetime import datetime

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.soc_estimator import SoCEstimator
from core.logger import TelemetryDB
from core.feature_extractor import FeatureExtractor
from core.diagnostics import Diagnostics, FaultDiagnosis
from core.health_engine import HealthEngine, ComponentHealth
from core.baseline_manager import BaselineManager
from core.blackbox import Blackbox
from core.propulsion import PropulsionUnit, make_units
from hardware.sensor_manager import SensorManager
from hardware.platform import detect_platform, is_raspberry_pi, get_cpu_temperature, get_ram_usage, get_disk_usage, get_uptime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("logs/monitor.log")
    ]
)
log = logging.getLogger("monitor")

VEHICLE_PROFILE_PATH = "config/vehicle_profile.json"
HARDWARE_CONFIG_PATH = "config/hardware.json"


def load_vehicle_profile(path: str = VEHICLE_PROFILE_PATH) -> dict:
    with open(path) as f:
        return json.load(f)


def _next_flight_id(base_dir: str = "data/flights") -> int:
    os.makedirs(base_dir, exist_ok=True)
    existing = []
    for fname in os.listdir(base_dir):
        if fname.startswith("flight_") and fname.endswith(".db"):
            try:
                num = int(fname.replace("flight_", "").replace(".db", ""))
                existing.append(num)
            except ValueError:
                pass
    return max(existing) + 1 if existing else 1


class DroneMonitor:
    def __init__(self, cfg: dict, profile: dict, simulate: bool = False, hardware_mode: bool = False):
        self._cfg = cfg
        self._profile = profile
        self._simulate = simulate
        self._hardware_mode = hardware_mode
        self._running = False

        self._vehicle_type = profile.get("vehicle_type", "quad")
        self._vehicle_features = profile.get("features", {})
        self._units = make_units(profile)
        self._n_units = len(self._units)

        sc = cfg['system']
        self._fast_interval = sc.get('fast_loop_ms', 25) / 1000.0
        self._health_interval = sc.get('health_loop_ms', 1000) / 1000.0
        self._log_interval = sc['log_interval_ms'] / 1000.0
        self._persistence = sc.get('fault_persistence_count', 3)

        bc = cfg['battery']
        self._features = FeatureExtractor(n_units=self._n_units)
        self._baseline = BaselineManager(n_units=self._n_units)
        self._diagnostics = Diagnostics(persistence_count=self._persistence)
        self._health_engine = HealthEngine(
            r_internal_baseline=0.020,
            capacity_full=bc['capacity_mah'],
            temp_max=bc.get('max_temp', 50.0),
        )
        blackbox_samples = int(max(30.0 / max(self._fast_interval, 0.001), 100))
        self._blackbox = Blackbox(max_samples=blackbox_samples)
        self._soc = SoCEstimator(bc['capacity_mah'], bc['cell_count'])

        flight_id = _next_flight_id()
        db_dir = "data/flights"
        os.makedirs(db_dir, exist_ok=True)
        db_path = os.path.join(db_dir, f"flight_{flight_id:03d}.db")
        self._flight_id = flight_id
        self._db = TelemetryDB(db_path)

        self._v_history = deque(maxlen=int(1.0 / max(self._fast_interval, 0.001)) + 1)

        self._sensor_manager = SensorManager(HARDWARE_CONFIG_PATH)
        self._batt_sensor = None
        self._current_sensors = []
        self._imu = None
        self._enabled_features = {}

        self._armed = False
        self._throttle = 0.0
        self._prop_on = True
        self._friction_level = 0.0
        self._sim_fault_types = []

        self._sensor_degraded: dict[str, int] = {}
        self._degraded_threshold = 3

        self.latest: dict = {"ts": 0, "status": "INIT", "overall_health": 100}
        self._lock = threading.Lock()

    def set_throttle(self, fraction: float):
        self._throttle = max(0.0, min(1.0, fraction))

    def arm(self):
        self._armed = True

    def disarm(self):
        self._armed = False

    def set_prop(self, on: bool):
        self._prop_on = on

    def set_friction(self, level: float):
        self._friction_level = max(0.0, min(1.0, level))

    def status_snapshot(self) -> dict:
        with self._lock:
            return dict(self.latest)

    def get_telemetry(self, limit: int = 300) -> list:
        return self._db.get_recent_telemetry(limit=limit)

    def get_active_faults(self) -> list:
        return self._db.get_active_faults()

    def get_fault_history(self, limit: int = 50) -> list:
        return self._db.get_recent_fault_history(limit=limit)

    def get_fault_events(self, limit: int = 50) -> list:
        return self._db.get_recent_fault_events(limit=limit)

    def get_health(self, limit: int = 100) -> list:
        return self._db.get_recent_health(limit=limit)

    def get_active_diagnoses(self) -> list:
        with self._lock:
            return [d.__dict__ for d in self._diagnostics.get_active()]

    def get_prediction(self) -> dict:
        with self._lock:
            pred = self._diagnostics.predicted_failure()
            if pred:
                return pred.__dict__
            return {}

    def get_system_info(self) -> dict:
        return {
            "platform": detect_platform(),
            "is_raspberry_pi": is_raspberry_pi(),
            "cpu_temp_c": get_cpu_temperature(),
            "ram": get_ram_usage(),
            "disk": get_disk_usage(),
            "uptime_s": round(get_uptime(), 1),
            "flight_id": self._flight_id,
            "mode": "hardware" if self._hardware_mode else "simulate",
            "sensor_status": {
                "battery": self._batt_sensor is not None,
                "current_sensors": len(self._current_sensors),
                "imu": self._imu is not None,
            },
            "enabled_features": dict(self._enabled_features),
        }

    def init_sensors(self, fault_types: list = None):
        fault_types = fault_types or []
        if self._hardware_mode:
            log.info("Hardware mode — initializing real I2C sensors")
            status = self._sensor_manager.init_hardware()
            self._batt_sensor = self._sensor_manager.get_battery()
            self._current_sensors = self._sensor_manager.get_current_sensors()
            self._imu = self._sensor_manager.get_imu()
            self._enabled_features = self._sensor_manager.get_enabled_features()
            found = status.get("scan_results", {}).get("found", {})
            missing = status.get("scan_results", {}).get("missing", {})
            for name, info in found.items():
                log.info("  ONLINE  %s at 0x%02X", info["label"], info["address"])
            for name, info in missing.items():
                log.info("  MISSING %s at %s", name, info.get("address", "N/A"))
        else:
            log.info("Simulation mode — %d propulsion unit(s)", self._n_units)
            self._sim_fault_types = fault_types
            status = self._sensor_manager.init_simulate(self._profile)
            self._batt_sensor = self._sensor_manager.get_battery()
            self._current_sensors = self._sensor_manager.get_current_sensors()
            self._imu = self._sensor_manager.get_imu()
            self._enabled_features = self._sensor_manager.get_enabled_features()
            self._apply_faults(fault_types)

        if self._batt_sensor is None:
            log.error("No battery sensor available — PHM cannot operate")
            sys.exit(1)

    def _apply_faults(self, fault_types: list):
        if not fault_types:
            return
        t0 = time.time()
        sensors = self._current_sensors
        if "prop-loss" in fault_types:
            for s in sensors[:1]:
                s.prop_attached = False
            log.info("Fault injection: PROP_LOSS on unit 1")
        if "friction" in fault_types:
            for s in sensors[:max(1, self._n_units - 1)]:
                s.friction_level = 0.5
            log.info("Fault injection: FRICTION on units 1-%d", max(1, self._n_units - 1))
        if "battery-aging" in fault_types:
            self._batt_sensor.fault_mode = "battery_aging"
            self._batt_sensor.fault_start_time = t0
            log.info("Fault injection: BATTERY_AGING")
        if "battery-old" in fault_types:
            self._batt_sensor.fault_mode = "battery_old"
            log.info("Fault injection: BATTERY_OLD")
        if "prop-damage" in fault_types:
            for s in sensors[:1]:
                s.fault_mode = "prop_damage"
                s.fault_start_time = t0
            log.info("Fault injection: PROP_DAMAGE on unit 1")
        if "bearing-wear" in fault_types:
            for s in sensors[:1]:
                s.fault_mode = "bearing_wear"
                s.fault_start_time = t0
            log.info("Fault injection: BEARING_WEAR on unit 1")
        if "esc-degrade" in fault_types:
            for s in sensors[:1]:
                s.fault_mode = "esc_degrade"
                s.fault_start_time = t0
            log.info("Fault injection: ESC_DEGRADE on unit 1")

    def _read_with_retry(self, sensor, method: str, default=None, label: str = ""):
        if sensor is None:
            return default
        for attempt in range(3):
            try:
                return getattr(sensor, method)()
            except Exception as e:
                if attempt == 0:
                    log.warning("Sensor %s %s failed (attempt %d): %s", label, method, attempt + 1, e)
                time.sleep(0.001)
        key = f"{label}.{method}"
        self._sensor_degraded[key] = self._sensor_degraded.get(key, 0) + 1
        if self._sensor_degraded[key] >= self._degraded_threshold:
            log.error("Sensor %s %s permanently degraded — disabling", label, method)
        return default

    def _fast_loop(self):
        log.info("Fast loop started (%.0f Hz)", 1.0 / self._fast_interval)
        while self._running:
            loop_start = time.time()

            try:
                if self._simulate:
                    for s in self._current_sensors:
                        s.set_throttle(self._throttle)
                        s.prop_attached = self._prop_on
                        s.friction_level = self._friction_level

                batt_v = self._read_with_retry(self._batt_sensor, "read_voltage", default=0.0, label="battery")
                batt_i = self._read_with_retry(self._batt_sensor, "read_current", default=0.0, label="battery")
                batt_p = self._read_with_retry(self._batt_sensor, "read_power", default=0.0, label="battery")

                currents = []
                for idx, sensor in enumerate(self._current_sensors):
                    c = self._read_with_retry(sensor, "read_current", default=None, label=f"motor_{idx}")
                    currents.append(c)

                if self._simulate:
                    self._batt_sensor.set_motor_currents(currents)
                    self._batt_sensor.set_throttle(self._throttle)

                temps = []
                for idx, sensor in enumerate(self._current_sensors):
                    if hasattr(sensor, "read_temperature"):
                        t = self._read_with_retry(sensor, "read_temperature", default=25.0, label=f"motor_{idx}")
                        temps.append(t)
                    else:
                        temps.append(25.0)
                ambient = 25.0

                vib_data = []
                if self._enabled_features.get("vibration", False) and self._imu is not None:
                    try:
                        vib = self._imu.read_vibration(n_samples=20)
                        vib_data.append({"unit_id": 0, **vib})
                    except Exception as e:
                        log.warning("IMU read failed: %s", e)
                        vib_data.append({"unit_id": 0, "rms": 0.0, "kurtosis": 0.0, "samples": []})
                elif self._simulate:
                    for i, s in enumerate(self._current_sensors):
                        if hasattr(s, "read_vibration"):
                            try:
                                vib = s.read_vibration(n_samples=20)
                                vib_data.append({"unit_id": i + 1, **vib})
                            except Exception:
                                vib_data.append({"unit_id": i + 1, "rms": 0.0, "kurtosis": 0.0, "samples": []})
                if not vib_data:
                    vib_data = [{"unit_id": i + 1, "rms": 0.0, "kurtosis": 0.0, "samples": []} for i in range(max(1, self._n_units))]

                soc = self._soc.update(batt_v, batt_i)

                self._v_history.append((time.time(), batt_v))
                dv_dt = 0.0
                if len(self._v_history) >= 2:
                    t0, v0 = self._v_history[0]
                    t1, v1 = self._v_history[-1]
                    dt = t1 - t0
                    dv_dt = (v1 - v0) / dt if dt > 0 else 0.0

                currents_clean = [c if c is not None else 0.0 for c in currents]

                for i, unit in enumerate(self._units):
                    unit.current = currents_clean[i] if i < len(currents_clean) else 0.0
                    unit.esc_temp = temps[i] if i < len(temps) else 25.0
                    vd = vib_data[i] if i < len(vib_data) else {"rms": 0.0, "kurtosis": 0.0}
                    unit.vibration_rms = vd.get("rms", 0.0)
                    unit.vibration_kurtosis = vd.get("kurtosis", 0.0)
                    unit.temp_rise = round(temps[i] if i < len(temps) else 25.0 - 25.0, 2)

                self._blackbox.record_sample(
                    voltage=batt_v, current=batt_i,
                    motor_currents=currents_clean,
                    motor_vibrations=[v.get("rms", 0.0) for v in vib_data],
                    motor_temps=temps,
                    throttle=self._throttle,
                    temperature=temps[0] if temps else 25.0,
                )

                with self._lock:
                    self.latest["ts"] = time.time()
                    self.latest["battery"] = {
                        "voltage": round(batt_v, 3), "current": round(batt_i, 2),
                        "power": round(batt_p, 2), "soc": soc,
                    }
                    self.latest["propulsion_units"] = [u.to_dict() for u in self._units]
                    self.latest["throttle"] = round(self._throttle * 100, 1)
                    self.latest["armed"] = self._armed
                    self.latest["prop_on"] = self._prop_on
                    self.latest["friction_level"] = round(self._friction_level, 2)
                    self.latest["vibration"] = vib_data
                    self.latest["temperatures"] = {"temps": temps}
                    self.latest["dv_dt"] = round(dv_dt, 4)

            except Exception as e:
                log.error("Fast loop error: %s", e)

            elapsed = time.time() - loop_start
            time.sleep(max(0, self._fast_interval - elapsed))

    def _health_loop(self):
        log.info("Health loop started (%.1f Hz)", 1.0 / self._health_interval)
        last_log_t = 0
        last_print_t = 0

        while self._running:
            loop_start = time.time()

            try:
                snapshot = self.status_snapshot()
                batt = snapshot.get("battery", {})
                batt_v = batt.get("voltage", 0.0)
                batt_i = batt.get("current", 0.0)
                batt_p = batt.get("power", 0.0)
                soc = batt.get("soc", 0.0)
                currents = [u.get("current", 0.0) for u in snapshot.get("propulsion_units", [])]
                temps = snapshot.get("temperatures", {}).get("temps", [25.0])
                vib_data = snapshot.get("vibration", [])
                ambient = 25.0

                batt_feat = self._features.electrical_battery(batt_v, batt_i, batt_p)
                p_features = self._features.propulsion_currents(currents)
                therm_feat = self._features.thermal(temps, ambient)

                vib_features = []
                for vd in vib_data:
                    vf = self._features.vibration(
                        vd.get("unit_id", 0), vd.get("samples", []), sample_rate_hz=100.0
                    )
                    if vf:
                        vib_features.append(vf)

                diagnoses = []
                r_internal = getattr(self._soc, 'r_internal', None)
                capacity_now = self._cfg['battery']['capacity_mah']
                capacity_full = self._cfg['battery']['capacity_mah']

                batt_diag = self._diagnostics.check_battery(
                    voltage=batt_v, current=batt_i, soc=soc,
                    r_internal=r_internal,
                    r_internal_baseline=self._health_engine.r_internal_baseline,
                    capacity_now=capacity_now,
                    capacity_full=capacity_full,
                    temp=temps[0] if temps else 25.0,
                    temp_max=self._cfg['battery'].get('max_temp', 50.0),
                )
                diagnoses.extend(batt_diag)

                for unit in self._units:
                    if unit.current is not None:
                        unit.baseline_current = self._baseline.compare_current(
                            unit.id, self._throttle, unit.current)
                    unit.baseline_vibration = self._baseline.compare_vibration(
                        unit.id, unit.vibration_rms)
                    unit.baseline_temp_rise = self._baseline.compare_temp_rise(
                        unit.id, unit.temp_rise, unit.current)
                    diagnoses.extend(self._diagnostics.check_propulsion(unit))

                if self._vehicle_features.get("enable_motor_balance", False):
                    diagnoses.extend(self._diagnostics.check_motor_imbalance(self._units))

                if self._vehicle_features.get("enable_efficiency_tracking", False):
                    for unit in self._units:
                        diagnoses.extend(self._diagnostics.check_efficiency_loss(
                            unit, self._throttle))

                new_active = self._diagnostics.update(diagnoses)

                for d in new_active:
                    self._db.log_fault_event(
                        component=d.component, fault_type=d.fault_type,
                        confidence=d.confidence, severity=d.severity,
                        evidence=d.evidence,
                    )
                    self._blackbox.save_fault_event(
                        component=d.component, fault_type=d.fault_type,
                        confidence=d.confidence, severity=d.severity,
                        evidence=d.evidence,
                    )

                batt_health = ComponentHealth()
                if self._baseline.is_calibrated():
                    r_int = r_internal if r_internal else self._health_engine.r_internal_baseline
                    batt_health = self._health_engine.battery_health(
                        r_internal=r_int, capacity_now=capacity_now, temp=temps[0] if temps else 25.0,
                    )
                    for unit in self._units:
                        self._health_engine.propulsion_health(unit)

                overall = self._health_engine.compute_overall(self._units, batt_health.score)

                status = "OK"
                active_diag = self._diagnostics.get_active()
                if any(d.severity == "critical" for d in active_diag):
                    status = "CRITICAL"
                elif any(d.severity == "warning" for d in active_diag):
                    status = "WARNING"

                now = time.time()
                if now - last_log_t >= self._log_interval:
                    currents_clean = [u.current if u.current is not None else 0.0 for u in self._units]
                    self._db.log_telemetry(batt_v, batt_i, soc, batt_p, currents_clean, status)
                    self._db.log_health(
                        overall=overall, battery=batt_health.score,
                        motor_health=[u.health for u in self._units],
                        esc_health=[u.health for u in self._units],
                    )
                    last_log_t = now

                if now - last_print_t >= 5.0:
                    unit_summary = "  ".join(
                        f"{u.name}={u.current:.1f}A({u.health:.0f}%)"
                        for u in self._units
                    )
                    diag_summary = ""
                    if active_diag:
                        diag_summary = "  DIAG: " + " ".join(
                            f"{d.component}/{d.fault_type}" for d in active_diag[:3]
                        )
                    log.info(
                        f"STATUS={status} | V={batt_v:.2f}V I={batt_i:.1f}A "
                        f"SoC={soc:.0f}% | {unit_summary} | "
                        f"HEALTH={overall:.0f}%  BATT={batt_health.score:.0f}%{diag_summary}"
                    )
                    last_print_t = now

                pred = self._diagnostics.predicted_failure()
                with self._lock:
                    self.latest["status"] = status
                    self.latest["overall_health"] = overall
                    self.latest["battery"]["health"] = batt_health.score
                    self.latest["active_diagnoses"] = [d.__dict__ for d in active_diag]
                    self.latest["predicted_failure"] = pred.__dict__ if pred else None
                    self.latest["vehicle_type"] = self._vehicle_type
                    self.latest["n_units"] = self._n_units

            except Exception as e:
                log.error("Health loop error: %s", e)

            elapsed = time.time() - loop_start
            time.sleep(max(0, self._health_interval - elapsed))

    def print_startup_banner(self):
        mode_str = "HARDWARE" if self._hardware_mode else "SIMULATION"
        units_str = ", ".join(u.name for u in self._units)
        features = self._enabled_features
        banner = f"""
================================

UAV PHM SYSTEM ONLINE

Mode:
{mode_str}

Vehicle:
{self._vehicle_type.title()}

Propulsion Units:
{self._n_units} ({units_str})

Battery Monitor:
{'ONLINE' if self._batt_sensor else 'OFFLINE'}

Current Sensors:
{len(self._current_sensors)} connected

IMU:
{'ONLINE' if self._imu else 'DEGRADED' if self._enabled_features.get('vibration', False) else 'OFFLINE'}

Health Engine:
READY

Features:
  Battery:        {'ON' if features.get('battery_monitor', True) else 'OFF'}
  Current Sense:  {'ON' if features.get('propulsion_current', True) else 'OFF'}
  Vibration:      {'ON' if features.get('vibration', False) else 'OFF'}
  Temperature:    {'ON' if features.get('temperature', False) else 'OFF'}

Dashboard:
http://<host>:5000

Flight ID:
{self._flight_id:03d}

================================
"""
        for line in banner.splitlines():
            log.info(line)

    def run(self):
        self._running = True
        self.print_startup_banner()

        threads = []

        fast = threading.Thread(target=self._fast_loop, name="fast-loop", daemon=True)
        fast.start()
        threads.append(fast)

        health = threading.Thread(target=self._health_loop, name="health-loop", daemon=True)
        health.start()
        threads.append(health)

        port = int(os.environ.get("PHM_PORT", "5000"))
        try:
            from web.dashboard import create_app
            app = create_app(self)
            web = threading.Thread(
                target=lambda: app.run(host="0.0.0.0", port=port,
                                       debug=False, use_reloader=False),
                name="web-server", daemon=True
            )
            web.start()
            threads.append(web)
            log.info("Dashboard available at http://0.0.0.0:%d", port)
        except Exception as e:
            log.warning("Web dashboard failed to start: %s. Running headless.", e)

        try:
            while self._running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            log.info("KeyboardInterrupt received")
        finally:
            self.stop()

    def stop(self):
        if not self._running:
            return
        log.info("Shutting down...")
        self._running = False
        time.sleep(0.5)

        try:
            self._db._conn.commit()
            self._db._conn.close()
            log.info("Database flushed")
        except Exception as e:
            log.warning("DB close error: %s", e)

        try:
            state = {
                "ts": time.time(),
                "flight_id": self._flight_id,
                "mode": "hardware" if self._hardware_mode else "simulate",
                "vehicle_type": self._vehicle_type,
                "status": self.status_snapshot().get("status", "UNKNOWN"),
            }
            os.makedirs("data/baselines", exist_ok=True)
            with open("data/baselines/last_state.json", "w") as f:
                json.dump(state, f, indent=2)
            log.info("Final state saved")
        except Exception as e:
            log.warning("State save error: %s", e)

        self._sensor_manager.close()
        log.info("Sensors closed")
        log.info("Shutdown complete")


def main():
    parser = argparse.ArgumentParser(description="UAV PHM Monitor — Raspberry Pi Deployment")
    parser.add_argument("--config", default="config/thresholds.yaml")
    parser.add_argument("--profile", default=VEHICLE_PROFILE_PATH,
                        help="Path to vehicle_profile.json")
    parser.add_argument("--simulate", action="store_true",
                        help="Use simulated sensor data (no hardware)")
    parser.add_argument("--hardware", action="store_true",
                        help="Force real I2C sensor mode (requires Raspberry Pi)")
    parser.add_argument("--fault", default="",
                        help="Fault types: prop-loss, friction, battery-aging, "
                             "prop-damage, bearing-wear, esc-degrade (simulate only)")
    parser.add_argument("--hardware-config", default=HARDWARE_CONFIG_PATH,
                        help="Path to hardware.json sensor configuration")
    args = parser.parse_args()

    os.makedirs("logs", exist_ok=True)

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    profile = load_vehicle_profile(args.profile)

    if args.hardware and args.simulate:
        log.error("Cannot use both --hardware and --simulate")
        sys.exit(1)

    hardware_mode = args.hardware or (not args.simulate and is_raspberry_pi())
    simulate = args.simulate or (not hardware_mode and cfg['system'].get('simulate', False))

    if hardware_mode and simulate:
        hardware_mode = False

    fault_types = [f.strip() for f in args.fault.split(",") if f.strip()]
    if fault_types and not simulate:
        log.warning("--fault flags only take effect in --simulate mode")

    monitor = DroneMonitor(cfg, profile, simulate=simulate, hardware_mode=hardware_mode)
    monitor.init_sensors(fault_types=fault_types)

    def _shutdown(sig, frame):
        log.info("Shutdown signal received: %s", sig)
        monitor.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    monitor.run()


if __name__ == "__main__":
    main()
