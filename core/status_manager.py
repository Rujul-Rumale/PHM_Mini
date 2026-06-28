import time
import copy
import logging
import threading
from collections import deque
from typing import Dict, List, Optional, Any

log = logging.getLogger(__name__)


class StatusManager:
    """Thread-safe state container and Flight State Machine (FSM) manager."""
    def __init__(self, event_bus=None):
        self._lock = threading.RLock()
        self._event_bus = event_bus
        
        self._state = {
            "flight_state": "BOOT",   # BOOT, INITIALIZING, READY, ARMED, RUNNING, FAULT, DISARMED, SHUTDOWN
            "battery": {
                "voltage": 0.0,
                "current": 0.0,
                "power": 0.0,
                "soc": 100.0,
                "health": 100.0,
                "r_internal": 0.020,
            },
            "propulsion": {
                "current": 0.0,
                "esc_temp": 25.0,
                "vibration_rms": 0.0,
                "vibration_kurtosis": 0.0,
                "health": 100.0,
                "current_dev": 0.0,
                "vibration_dev": 0.0,
                "temp_dev": 0.0,
                "warnings": [],
            },
            "airframe": {
                "vibration": {
                    "rms": 0.0,
                    "kurtosis": 0.0,
                    "samples": [],
                }
            },
            "controls": {
                "armed": False,
                "throttle": 0.0,
                "target_throttle": 0.0,
                "sim_fault_inject": [],
                "flight_mode": "simulation",  # simulation, bench_test, ground_test, flight
                "deadman_timeout_sec": 10.0,
                "last_operator_heartbeat": time.time(),
            },
            "active_faults": [],      # List of active FaultDiagnosis dicts
            "fault_events": [],
            "config": {},
            "sensor_status": {
                "battery": "OFFLINE",
                "propulsion_current": "OFFLINE",
                "esc_temperature": "OFFLINE",
                "imu": "OFFLINE",
            },
            "loop_stats": {
                "frequency": 0.0,
                "missed_cycles": 0,
                "watchdog_pings": {},
            },
            "system_health": {
                "cpu_temp": 25.0,
                "ram_usage": 0.0,
                "disk_usage": 0.0,
                "uptime": 0.0,
            },
            "event_log": deque(maxlen=50), # Event console log
            "capabilities": {
                "battery_health": False,
                "propulsion_current": False,
                "esc_thermal": False,
                "vibration": False,
                "motor_balance": False,
                "efficiency_tracking": False,
                "flight_recorder": True,
            }
        }

    def get_state(self) -> dict:
        """Return a deep copy of the full state dict for serialization/read access."""
        with self._lock:
            state_copy = copy.deepcopy(self._state)
            state_copy["event_log"] = list(state_copy["event_log"])
            return state_copy

    def update_state(self, path: str, value: Any):
        """Update a nested dictionary key using a dot-separated path (e.g. 'battery.voltage')."""
        with self._lock:
            parts = path.split(".")
            d = self._state
            for p in parts[:-1]:
                if p not in d:
                    d[p] = {}
                d = d[p]
            d[parts[-1]] = value

    def log_event(self, msg: str):
        """Add a timestamped entry to the scrollable UI event log."""
        with self._lock:
            ts_str = time.strftime("%H:%M:%S")
            self._state["event_log"].append(f"{ts_str} {msg}")
            log.info(f"Event: {msg}")

    def get_event_log(self) -> List[str]:
        with self._lock:
            return list(self._state["event_log"])

    def transition_to(self, new_state: str, force: bool = False, reason: str = "") -> bool:
        """
        Transition the FSM state. Enforces valid FSM transitions unless force=True.
        Returns True if transition occurred, False otherwise.
        """
        with self._lock:
            curr = self._state["flight_state"]
            if curr == new_state:
                return True

            allowed = False
            if force:
                allowed = True
            elif curr == "BOOT" and new_state == "INITIALIZING":
                allowed = True
            elif curr == "INITIALIZING" and new_state == "READY":
                allowed = True
            elif curr == "INITIALIZING" and new_state == "FAULT":
                allowed = True
            elif curr == "READY" and new_state == "ARMED":
                allowed = True
            elif curr == "ARMED" and new_state == "RUNNING":
                allowed = True
            elif curr == "RUNNING" and new_state == "ARMED":
                # Back to ARMED when throttle goes to zero
                allowed = True
            elif curr in ["ARMED", "RUNNING", "FAULT"] and new_state == "DISARMED":
                allowed = True
            elif curr == "DISARMED" and new_state == "READY":
                allowed = True
            elif new_state == "FAULT":
                # Any state can go to FAULT if a critical failure occurs
                allowed = True
            elif new_state == "SHUTDOWN":
                # Shutdown is exit condition
                allowed = True

            if allowed:
                self._state["flight_state"] = new_state
                
                # FSM actions
                if new_state == "FAULT" or new_state == "DISARMED":
                    # Force throttle to 0
                    self._state["controls"]["throttle"] = 0.0
                    self._state["controls"]["target_throttle"] = 0.0
                
                if new_state == "ARMED":
                    self._state["controls"]["armed"] = True
                elif new_state in ["DISARMED", "FAULT", "READY", "BOOT"]:
                    self._state["controls"]["armed"] = False

                reason_str = f" ({reason})" if reason else ""
                self.log_event(f"State transition: {curr} -> {new_state}{reason_str}")
                
                if self._event_bus:
                    from core.event_bus import EventBus
                    # We can lazy-import or define event classes
                    # Let's import inside to avoid circular reference
                    try:
                        from core.monitor_service import FlightStateChanged
                        self._event_bus.publish(FlightStateChanged(curr, new_state))
                    except ImportError:
                        pass
                return True
            else:
                log.warning(f"FSM Transition REJECTED: {curr} -> {new_state}{f' ({reason})' if reason else ''}")
                return False

    def ping_watchdog(self, service_name: str):
        """Update last heartbeat timestamp for a service."""
        with self._lock:
            self._state["loop_stats"]["watchdog_pings"][service_name] = time.time()

    def update_telemetry(self, frame):
        """Thread-safely unpack and update state dictionary from frame."""
        with self._lock:
            self._state["battery"]["voltage"] = frame.battery_voltage
            self._state["battery"]["current"] = frame.battery_current
            self._state["battery"]["power"] = frame.battery_power
            if frame.soc is not None:
                self._state["battery"]["soc"] = frame.soc
                
            self._state["propulsion"]["current"] = frame.esc_current
            self._state["propulsion"]["esc_temp"] = frame.esc_temp
            self._state["propulsion"]["vibration_rms"] = frame.imu_rms
            self._state["propulsion"]["vibration_kurtosis"] = frame.imu_kurtosis
            
            self._state["airframe"]["vibration"]["rms"] = frame.imu_rms
            self._state["airframe"]["vibration"]["kurtosis"] = frame.imu_kurtosis
            
            self._state["controls"]["throttle"] = frame.throttle_pct

