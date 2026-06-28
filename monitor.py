"""
UAV PHM System — Modular, Service-Oriented Embedded Computer.
"""

import argparse
import json
import logging
import time
import sys
import os
import signal
import threading
from datetime import datetime

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from core.event_bus import EventBus
from core.status_manager import StatusManager
from core.telemetry_frame import TelemetryFrame
from core.throttle import ThrottleController
from core.control_service import ControlService
from core.monitor_service import MonitorService
from core.feature_engine import FeatureEngine
from core.diagnostics_engine import DiagnosticsEngine
from core.database_service import DatabaseService, FlightStarted, FlightEnded
from core.watchdog import WatchdogService
from core.blackbox import Blackbox
from core.baseline_manager import BaselineManager
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


class DroneMonitor:
    """Orchestrates all thread-safe services, data loops, and safety watchdog."""
    def __init__(self, cfg: dict, profile: dict, simulate: bool = False, hardware_mode: bool = False, flight_mode: str = "simulation"):
        self._cfg = cfg
        self._profile = profile
        self._simulate = simulate
        self._hardware_mode = hardware_mode
        self._flight_mode = flight_mode
        self._running = False
        
        # 1. Event Bus
        self.event_bus = EventBus()
        
        # 2. Status Manager
        self.status_mgr = StatusManager(self.event_bus)
        self.status_mgr.update_state("config", cfg)
        self.status_mgr.update_state("vehicle_type", profile.get("vehicle_type", "fixed-wing"))
        self.status_mgr.update_state("controls.flight_mode", flight_mode)
        self.status_mgr.update_state("controls.deadman_timeout_sec", profile.get("safety", {}).get("deadman_timeout_sec", 10.0))
        
        # 3. Baseline profile manager
        self.baseline_mgr = BaselineManager(n_units=len(profile.get("propulsion_units", [1])))
        
        # 4. Throttle Controller
        mc = cfg.get("motors", {})
        self.throttle_ctrl = ThrottleController(
            status_mgr=self.status_mgr,
            max_throttle_limit=profile.get("throttle", {}).get("max_percent", 80) / 100.0,
            warn_current=mc.get("warn_current_per_motor", 12.0),
            max_current=mc.get("max_current_per_motor", 15.0),
            ramp_rate_per_sec=profile.get("throttle", {}).get("ramp_rate_per_sec", 20.0) / 100.0
        )
        
        # 5. Blackbox circular recorder
        self.blackbox = Blackbox()
        
        # 6. Database and sessions service
        flight_id = self._next_flight_id()
        self.status_mgr.update_state("system_health.flight_id", flight_id)
        db_path = "data/flights/flight_monitor.db"
        self.db_service = DatabaseService(self.status_mgr, self.event_bus, db_path)
        
        # 7. Pre-arm and control service
        self.control_service = ControlService(
            status_mgr=self.status_mgr,
            event_bus=self.event_bus,
            throttle_ctrl=self.throttle_ctrl,
            blackbox=self.blackbox,
            db_service=self.db_service
        )
        
        # 8. Sensor Manager
        self.sensor_mgr = SensorManager(HARDWARE_CONFIG_PATH)
        
        # 9. Hardware read loops
        self.monitor_service = MonitorService(
            status_mgr=self.status_mgr,
            event_bus=self.event_bus,
            sensor_mgr=self.sensor_mgr,
            throttle_ctrl=self.throttle_ctrl
        )
        
        # 10. Feature Engine
        self.feature_engine = FeatureEngine(self.status_mgr, self.event_bus, n_units=len(profile.get("propulsion_units", [1])))
        
        # 11. Diagnostics Engine
        self.diagnostics_engine = DiagnosticsEngine(self.status_mgr, self.event_bus, self.baseline_mgr)
        self.diagnostics_engine.db_service = self.db_service # wire DB to diagnostics log
        
        # 12. Watchdog
        self.watchdog = WatchdogService(
            status_mgr=self.status_mgr,
            services={
                "monitor": self.monitor_service,
                "database": self.db_service
            }
        )

        # Wire blackbox ring buffer recorder to frame updates
        from core.database_service import HealthUpdated
        self.event_bus.subscribe(HealthUpdated, self._record_blackbox_frame)

    def _next_flight_id(self, base_dir: str = "data/flights") -> int:
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

    def _record_blackbox_frame(self, event: HealthUpdated):
        frame = event.frame
        self.blackbox.record_sample(
            voltage=frame.battery_voltage,
            current=frame.battery_current,
            motor_currents=[frame.esc_current],
            motor_vibrations=[frame.imu_rms],
            motor_temps=[frame.esc_temp],
            throttle=frame.throttle_pct,
            temperature=frame.esc_temp,
            timestamp=frame.timestamp
        )

    def init_sensors(self, fault_types: list = None):
        fault_types = fault_types or []
        if self._hardware_mode:
            log.info("Hardware mode — initializing real I2C sensors")
            status = self.sensor_mgr.init_hardware()
            self._apply_fault_types_to_mock(fault_types)
        else:
            log.info(f"Simulation mode — initializing virtual sensors")
            status = self.sensor_mgr.init_simulate(self._profile)
            self._apply_fault_types_to_mock(fault_types)

    def _apply_fault_types_to_mock(self, fault_types: list):
        if not fault_types:
            return
            
        # Register simulation fault injections in controls state
        self.status_mgr.update_state("controls.sim_fault_inject", fault_types)
        
        # Set fault mode on mock battery
        batt = self.sensor_mgr.get_battery()
        if batt and hasattr(batt, "fault_mode"):
            if "battery-aging" in fault_types:
                batt.fault_mode = "battery_aging"
                batt.fault_start_time = time.time()
            elif "battery-old" in fault_types:
                batt.fault_mode = "battery_old"
                
        # Set fault mode on mock currents / esc temp
        currents = self.sensor_mgr.get_current_sensors()
        if currents and hasattr(currents[0], "fault_mode"):
            for f in fault_types:
                if f in ["prop-loss", "prop-damage", "bearing-wear", "esc-degrade", "friction"]:
                    # map cli hyphens to underscores for mock driver enum
                    m_f = f.replace("-", "_")
                    currents[0].fault_mode = m_f
                    currents[0].fault_start_time = time.time()
                    
                    # Set ESC Temp mock fault
                    temps = getattr(self.sensor_mgr, "_temperature_sensors", [])
                    if temps and hasattr(temps[0], "set_fault_mode"):
                        temps[0].set_fault_mode(f)

    def status_snapshot(self) -> dict:
        # Fetch current dashboard snapshot
        state = self.status_mgr.get_state()
        
        # Query hardware system resource stats
        system_stats = {
            "cpu_temp_c": get_cpu_temperature(),
            "ram": get_ram_usage(),
            "disk": get_disk_usage(),
            "uptime_s": round(get_uptime(), 1),
            "platform": detect_platform(),
            "is_raspberry_pi": is_raspberry_pi()
        }
        state["system_health"].update(system_stats)
        
        # Unpack unit dictionary list for backward compatibility
        prop = state["propulsion"]
        state["propulsion_units"] = [{
            "id": 1,
            "name": "main_motor",
            "current": prop["current"],
            "esc_temp": prop["esc_temp"],
            "vibration_rms": prop["vibration_rms"],
            "vibration_kurtosis": prop["vibration_kurtosis"],
            "health": prop["health"],
            "warnings": prop["warnings"],
        }]
        
        # Add dynamic vibration payload mapping
        state["vibration"] = [{
            "unit_id": 1,
            "rms": prop["vibration_rms"],
            "kurtosis": prop["vibration_kurtosis"]
        }]
        
        return state

    def get_telemetry(self, limit: int = 300) -> list:
        if self.db_service and self.db_service.db:
            return self.db_service.db.get_recent_telemetry(limit=limit)
        return []

    def get_active_faults(self) -> list:
        if self.db_service and self.db_service.db:
            return self.db_service.db.get_active_faults()
        return []

    def get_fault_history(self, limit: int = 50) -> list:
        if self.db_service and self.db_service.db:
            return self.db_service.db.get_recent_fault_history(limit=limit)
        return []

    def get_fault_events(self, limit: int = 50) -> list:
        if self.db_service and self.db_service.db:
            return self.db_service.db.get_recent_fault_events(limit=limit)
        return []

    def get_health(self, limit: int = 100) -> list:
        if self.db_service and self.db_service.db:
            return self.db_service.db.get_recent_health(limit=limit)
        return []

    def get_active_diagnoses(self) -> list:
        return self.status_mgr.get_state().get("active_faults", [])

    def get_prediction(self) -> dict:
        return self.status_mgr.get_state().get("predicted_failure") or {}

    def get_system_info(self) -> dict:
        state = self.status_mgr.get_state()
        sys_h = state["system_health"]
        
        # Poll RPi metrics
        sys_h.update({
            "cpu_temp_c": get_cpu_temperature(),
            "ram": get_ram_usage(),
            "disk": get_disk_usage(),
            "uptime_s": round(get_uptime(), 1),
            "platform": detect_platform(),
            "is_raspberry_pi": is_raspberry_pi(),
            "flight_id": state["system_health"].get("flight_id", 1),
            "mode": state["controls"]["flight_mode"],
            "sensor_status": state["sensor_status"],
            "enabled_features": state["capabilities"]
        })
        return sys_h

    def print_startup_banner(self):
        state = self.status_mgr.get_state()
        caps = state["capabilities"]
        sensors = state["sensor_status"]
        
        banner = f"""
====================================
UAV PHM SYSTEM
Vehicle: {state.get('vehicle_type', 'fixed-wing').upper()}    Flight: {state['system_health'].get('flight_id', 1):03d}
====================================
Hardware
  {'✓' if sensors.get('battery') == 'ONLINE' else '✗'} Battery Monitor    INA219 @ 0x40
  {'✓' if str(sensors.get('propulsion_current', '')).startswith('ONLINE') else '✗'} ESC Supply Current INA219 (Shared)
  {'✓' if sensors.get('imu') == 'ONLINE' else '✗'} Airframe IMU       MPU6050@ 0x68

Throttle Source: {self.throttle_ctrl.provider.source_type.title()}

Enabled Diagnostics
  {'✓' if caps.get('battery_health') else '✗'} Battery Health
  {'✓' if caps.get('propulsion_current') else '✗'} Prop Damage
  {'✓' if caps.get('vibration') else '✗'} Mechanical Degradation
  {'✓' if caps.get('esc_thermal') else '✗'} ESC Thermal
  {'✓' if caps.get('flight_recorder') else '✗'} Flight Recorder

Disabled
  ✗ Motor Balance (single propulsion unit)

Mode: {state['controls']['flight_mode'].upper()}
State: {state['flight_state']}
====================================
"""
        for line in banner.splitlines():
            log.info(line)

    def run(self):
        self._running = True
        
        # Start Service Threads
        self.monitor_service.start()
        self.db_service.start()
        self.watchdog.start()
        
        # Print Capability Report banner
        self.print_startup_banner()

        # Start Dashboard Web server
        port = int(os.environ.get("PHM_PORT", "5000"))
        try:
            from web.dashboard import create_app
            app = create_app(self)
            
            # Start dashboard in a daemon thread so it does not block main loop exit
            web_thread = threading.Thread(
                target=lambda: app.run(host="0.0.0.0", port=port, debug=False, use_reloader=False),
                name="web-server", daemon=True
            )
            web_thread.start()
            log.info(f"Dashboard available at http://0.0.0.0:{port}")
        except Exception as e:
            log.warning(f"Web dashboard failed to start: {e}. Running headless.")

        try:
            while self._running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            log.info("KeyboardInterrupt received.")
        finally:
            self.stop()

    def stop(self):
        if not self._running:
            return
            
        log.info("Shutting down PHM computer...")
        self._running = False
        
        # Stop background loops
        self.monitor_service.stop()
        self.db_service.stop()
        self.watchdog.stop()
        
        time.sleep(0.5)
        
        # Close database connection
        if self.db_service and self.db_service.db:
            try:
                self.db_service.db._conn.commit()
                self.db_service.db._conn.close()
                log.info("Database cleanly closed.")
            except Exception as e:
                log.warning(f"DB close error: {e}")
                
        # Close sensors
        self.sensor_mgr.close()
        log.info("Shutdown complete.")


def main():
    parser = argparse.ArgumentParser(description="UAV PHM Computer")
    parser.add_argument("--config", default="config/thresholds.yaml")
    parser.add_argument("--profile", default=VEHICLE_PROFILE_PATH)
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--mode", default=None, choices=["simulation", "bench_test", "ground_test", "flight"])
    parser.add_argument("--fault", default="")
    parser.add_argument("--hardware-config", default=HARDWARE_CONFIG_PATH)
    args = parser.parse_args()

    os.makedirs("logs", exist_ok=True)

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    with open(args.profile) as f:
        profile = json.load(f)

    hardware_mode = args.hardware or (not args.simulate and is_raspberry_pi())
    simulate = args.simulate or (not hardware_mode and cfg.get('system', {}).get('simulate', False))

    if hardware_mode and simulate:
        hardware_mode = False

    # Determine default flight mode
    mode = args.mode
    if not mode:
        mode = "bench_test" if hardware_mode else "simulation"

    fault_types = [f.strip() for f in args.fault.split(",") if f.strip()]

    monitor = DroneMonitor(cfg, profile, simulate=simulate, hardware_mode=hardware_mode, flight_mode=mode)
    monitor.init_sensors(fault_types=fault_types)

    def _shutdown(sig, frame):
        log.info(f"Shutdown signal received: {sig}")
        monitor.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    monitor.run()


if __name__ == "__main__":
    main()
