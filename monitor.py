"""
Drone PHM Monitor — vehicle-agnostic.
Supports N propulsion units (quad=4, fixed-wing=1) via PropulsionUnit model.
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
from sensors.ina226 import INA226, MockINA226

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


def load_vehicle_profile(path: str = VEHICLE_PROFILE_PATH) -> dict:
    with open(path) as f:
        return json.load(f)


class DroneMonitor:
    def __init__(self, cfg: dict, profile: dict, simulate: bool = False):
        self._cfg = cfg
        self._profile = profile
        self._simulate = simulate
        self._running = False

        self._vehicle_type = profile.get("vehicle_type", "quad")
        self._vehicle_features = profile.get("features", {})
        self._units = make_units(profile)
        self._n_units = len(self._units)

        sc = cfg['system']
        self._poll_interval = sc['poll_interval_ms'] / 1000.0
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
        self._blackbox = Blackbox()

        self._soc = SoCEstimator(bc['capacity_mah'], bc['cell_count'])
        self._db = TelemetryDB(sc['db_path'])

        self._v_history = deque(maxlen=int(1.0 / self._poll_interval) + 1)

        self._batt_sensor = None
        self._sensors = []       # one MockINA226 per propulsion unit
        self._armed = False
        self._throttle = 0.0
        self._sim_fault_types = []

        self._prop_on = True
        self._friction_level = 0.0

        self.latest: dict = {}
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

    def init_sensors(self, fault_types: list = None):
        fault_types = fault_types or []
        if self._simulate:
            log.info(f"Simulation mode — {self._n_units} propulsion units  faults={fault_types or 'none'}")
            self._sim_fault_types = fault_types
            bc = self._cfg['battery']
            sim_cfg = self._profile.get('simulation', {})
            motor_imax = sim_cfg.get('motor_imax', 15.0)
            motor_i0 = sim_cfg.get('motor_i0', 0.45)
            batt_imax = motor_imax * self._n_units + 2.0

            self._batt_sensor = MockINA226(
                base_voltage=bc['nominal_voltage'],
                base_current=batt_imax,
                label="battery",
                cell_count=bc['cell_count'],
                r_int=sim_cfg.get('r_int', 0.015),
            )
            self._batt_sensor.begin()

            for i, unit in enumerate(self._units):
                s = MockINA226(base_voltage=bc['nominal_voltage'] * 0.95,
                               base_current=motor_imax,
                               label=unit.name,
                               no_load_i=motor_i0,
                               motor_imax=motor_imax)
                s.begin()
                self._sensors.append(s)

            t0 = time.time()
            if "prop-loss" in fault_types:
                for s in self._sensors[:1]:
                    s.prop_attached = False
                log.info("Fault injection: PROP_LOSS on unit 1")

            if "friction" in fault_types:
                for s in self._sensors[:max(1, self._n_units - 1)]:
                    s.friction_level = 0.5
                log.info("Fault injection: FRICTION on units 1-%d", max(1, self._n_units - 1))

            if "battery-aging" in fault_types:
                self._batt_sensor.fault_mode = "battery_aging"
                self._batt_sensor.fault_start_time = t0
                log.info("Fault injection: BATTERY_AGING")

            if "battery-old" in fault_types:
                self._batt_sensor.fault_mode = "battery_old"
                log.info("Fault injection: BATTERY_OLD (R_int=0.060)")

            if "prop-damage" in fault_types:
                for s in self._sensors[:1]:
                    s.fault_mode = "prop_damage"
                    s.fault_start_time = t0
                log.info("Fault injection: PROP_DAMAGE on unit 1")

            if "bearing-wear" in fault_types:
                for s in self._sensors[:1]:
                    s.fault_mode = "bearing_wear"
                    s.fault_start_time = t0
                log.info("Fault injection: BEARING_WEAR on unit 1")

            if "esc-degrade" in fault_types:
                for s in self._sensors[:1]:
                    s.fault_mode = "esc_degrade"
                    s.fault_start_time = t0
                log.info("Fault injection: ESC_DEGRADE on unit 1")
        else:
            try:
                import smbus2
                bc = self._cfg['battery']
                mc = self._cfg['motors']
                batt_addr = bc['i2c_address']
                unit_addrs = mc['i2c_addresses'][:self._n_units]

                if any(a == batt_addr for a in unit_addrs):
                    bus0 = smbus2.SMBus(0)
                    bus1 = smbus2.SMBus(1)
                    self._batt_sensor = INA226(bus1, batt_addr, bc['shunt_ohms'])
                    self._batt_sensor.begin()
                    for addr in unit_addrs:
                        bus = bus0 if addr == batt_addr else bus1
                        s = INA226(bus, addr, mc['shunt_ohms'],
                                   max_amps=mc['max_current_per_motor'] * 1.2)
                        s.begin()
                        self._sensors.append(s)
                else:
                    bus = smbus2.SMBus(1)
                    self._batt_sensor = INA226(bus, batt_addr, bc['shunt_ohms'])
                    self._batt_sensor.begin()
                    for addr in unit_addrs:
                        s = INA226(bus, addr, mc['shunt_ohms'],
                                   max_amps=mc['max_current_per_motor'] * 1.2)
                        s.begin()
                        self._sensors.append(s)
                log.info(f"Initialized battery + {len(self._sensors)} unit sensors via I2C")
            except ImportError:
                log.error("smbus2 not installed: pip3 install smbus2")
                sys.exit(1)
            except Exception as e:
                log.error(f"Sensor init failed: {e}")
                sys.exit(1)

    def _read_sensors(self):
        if self._simulate:
            for s in self._sensors:
                s.set_throttle(self._throttle)
                s.prop_attached = self._prop_on
                s.friction_level = self._friction_level

            currents = []
            for sensor in self._sensors:
                try:
                    currents.append(sensor.read_current())
                except Exception as e:
                    log.warning(f"Sensor read failed: {e}")
                    currents.append(None)

            self._batt_sensor.set_motor_currents(currents)
            self._batt_sensor.set_throttle(self._throttle)
        else:
            currents = []

        batt_v = self._batt_sensor.read_voltage()
        batt_i = self._batt_sensor.read_current()
        batt_p = self._batt_sensor.read_power()

        if not self._simulate:
            currents = []
            for sensor in self._sensors:
                try:
                    currents.append(sensor.read_current())
                except Exception as e:
                    log.warning(f"Sensor read failed: {e}")
                    currents.append(None)

        return batt_v, batt_i, batt_p, currents

    def _read_temperatures(self) -> tuple[list[float], float]:
        temps = []
        for s in self._sensors:
            if hasattr(s, 'read_temperature'):
                try:
                    temps.append(s.read_temperature())
                except Exception:
                    temps.append(25.0)
            else:
                temps.append(25.0)
        ambient = getattr(self._batt_sensor, '_ambient', 25.0)
        return temps, ambient

    def _read_vibration(self) -> list[dict]:
        vib_data = []
        for i, s in enumerate(self._sensors):
            if hasattr(s, 'read_vibration'):
                try:
                    vib = s.read_vibration(n_samples=50)
                    vib_data.append({"unit_id": i + 1, **vib})
                except Exception:
                    vib_data.append({"unit_id": i + 1, "rms": 0.0, "kurtosis": 0.0, "samples": []})
            else:
                vib_data.append({"unit_id": i + 1, "rms": 0.0, "kurtosis": 0.0, "samples": []})
        return vib_data

    def _compute_dv_dt(self, voltage: float) -> float:
        self._v_history.append((time.time(), voltage))
        if len(self._v_history) < 2:
            return 0.0
        t0, v0 = self._v_history[0]
        t1, v1 = self._v_history[-1]
        dt = t1 - t0
        return (v1 - v0) / dt if dt > 0 else 0.0

    def _update_units_from_sensors(self, currents, temps, vib_data):
        """Update PropulsionUnit objects from raw sensor readings."""
        for i, unit in enumerate(self._units):
            c = currents[i] if i < len(currents) and currents[i] is not None else 0.0
            t = temps[i] if i < len(temps) else 25.0
            vd = vib_data[i] if i < len(vib_data) else {"rms": 0.0, "kurtosis": 0.0}

            unit.current = c
            unit.esc_temp = t
            unit.vibration_rms = vd.get("rms", 0.0)
            unit.vibration_kurtosis = vd.get("kurtosis", 0.0)
            unit.temp_rise = round(t - 25.0, 2)

    def run(self):
        self._running = True
        log.info(f"Monitor started — {self._vehicle_type} ({self._n_units} propulsion units)")
        last_log_t = 0
        last_print_t = 0

        while self._running:
            loop_start = time.time()

            try:
                batt_v, batt_i, batt_p, currents = self._read_sensors()
                temps, ambient = self._read_temperatures()
                vib_data = self._read_vibration()
            except Exception as e:
                log.error(f"Sensor read error: {e}")
                time.sleep(self._poll_interval)
                continue

            soc = self._soc.update(batt_v, batt_i)
            if self._simulate:
                self._batt_sensor.set_soc(soc)
            dv_dt = self._compute_dv_dt(batt_v)

            self._update_units_from_sensors(currents, temps, vib_data)
            currents_clean = [c if c is not None else 0.0 for c in currents]

            # Feature extraction
            batt_feat = self._features.electrical_battery(batt_v, batt_i, batt_p)
            p_features = self._features.propulsion_currents(currents)
            therm_feat = self._features.thermal(temps, ambient)

            vib_features = []
            for vd in vib_data:
                vf = self._features.vibration(
                    vd["unit_id"], vd.get("samples", []), sample_rate_hz=100.0
                )
                if vf:
                    vib_features.append(vf)

            # Diagnostics
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

            # Per-unit baseline comparisons and diagnostics
            for unit in self._units:
                unit.baseline_current = self._baseline.compare_current(
                    unit.id, self._throttle, unit.current)
                unit.baseline_vibration = self._baseline.compare_vibration(
                    unit.id, unit.vibration_rms)
                unit.baseline_temp_rise = self._baseline.compare_temp_rise(
                    unit.id, unit.temp_rise, unit.current)

                diagnoses.extend(self._diagnostics.check_propulsion(unit))

            # Quad-only: motor imbalance
            if self._vehicle_features.get("enable_motor_balance", False):
                diagnoses.extend(self._diagnostics.check_motor_imbalance(self._units))

            # Fixed-wing-only: efficiency loss
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

            # Health engine
            batt_health = ComponentHealth()
            if self._baseline.is_calibrated():
                r_int = r_internal if r_internal else self._health_engine.r_internal_baseline
                batt_health = self._health_engine.battery_health(
                    r_internal=r_int,
                    capacity_now=capacity_now,
                    temp=temps[0] if temps else 25.0,
                )

                for unit in self._units:
                    self._health_engine.propulsion_health(unit)

            overall = self._health_engine.compute_overall(self._units, batt_health.score)

            self._blackbox.record_sample(
                voltage=batt_v, current=batt_i,
                motor_currents=currents_clean,
                motor_vibrations=[v.get("rms", 0.0) for v in vib_data],
                motor_temps=temps,
                throttle=self._throttle,
                temperature=temps[0] if temps else 25.0,
            )

            status = "OK"
            active_diag = self._diagnostics.get_active()
            if any(d.severity == "critical" for d in active_diag):
                status = "CRITICAL"
            elif any(d.severity == "warning" for d in active_diag):
                status = "WARNING"

            now = time.time()
            if now - last_log_t >= self._log_interval:
                self._db.log_telemetry(
                    batt_v, batt_i, soc, batt_p, currents_clean, status
                )
                self._db.log_health(
                    overall=overall,
                    battery=batt_health.score,
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
                self.latest = {
                    "ts": now,
                    "vehicle_type": self._vehicle_type,
                    "n_units": self._n_units,
                    "battery": {
                        "voltage": round(batt_v, 3),
                        "current": round(batt_i, 2),
                        "power": round(batt_p, 2),
                        "soc": soc,
                        "health": batt_health.score,
                    },
                    "propulsion_units": [u.to_dict() for u in self._units],
                    "prop_on": self._prop_on,
                    "friction_level": round(self._friction_level, 2),
                    "throttle": round(self._throttle * 100, 1),
                    "status": status,
                    "armed": self._armed,
                    "overall_health": overall,
                    "vibration": vib_data,
                    "temperatures": {"temps": temps},
                    "active_diagnoses": [d.__dict__ for d in active_diag],
                    "predicted_failure": pred.__dict__ if pred else None,
                }

            elapsed = time.time() - loop_start
            time.sleep(max(0, self._poll_interval - elapsed))

    def stop(self):
        self._running = False
        log.info("Monitor stopped")


def main():
    parser = argparse.ArgumentParser(description="Vehicle-Agnostic UAV PHM Monitor")
    parser.add_argument("--config", default="config/thresholds.yaml")
    parser.add_argument("--profile", default=VEHICLE_PROFILE_PATH,
                        help="Path to vehicle_profile.json")
    parser.add_argument("--simulate", action="store_true",
                        help="Use simulated sensor data (no hardware)")
    parser.add_argument("--fault", default="",
                        help="Fault types: prop-loss, friction, battery-aging, "
                             "prop-damage, bearing-wear, esc-degrade")
    args = parser.parse_args()

    os.makedirs("logs", exist_ok=True)

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    profile = load_vehicle_profile(args.profile)
    simulate = args.simulate or cfg['system'].get('simulate', False)

    fault_types = [f.strip() for f in args.fault.split(",") if f.strip()]
    if fault_types and not simulate:
        log.warning("--fault flags only take effect in --simulate mode")

    monitor = DroneMonitor(cfg, profile, simulate=simulate)
    monitor.init_sensors(fault_types=fault_types)

    def _shutdown(sig, frame):
        log.info("Shutdown signal received")
        monitor.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        from web.dashboard import create_app
        app = create_app(monitor)
        port = int(os.environ.get("PHM_PORT", "5050"))
        web_thread = threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=port,
                                   debug=False, use_reloader=False),
            daemon=True
        )
        web_thread.start()
        log.info(f"Dashboard available at http://0.0.0.0:{port}")
    except Exception as e:
        log.warning(f"Web dashboard failed to start: {e}. Running headless.")

    monitor.run()


if __name__ == "__main__":
    main()
