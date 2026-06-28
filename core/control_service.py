import time
import logging

from core.status_manager import StatusManager
from core.throttle import ThrottleController
from core.database_service import FlightStarted, FlightEnded

log = logging.getLogger(__name__)


class ControlService:
    """Handles operator commands, arming checks, disarm sequences, and fault recovery."""
    def __init__(self, status_mgr: StatusManager, event_bus, throttle_ctrl: ThrottleController,
                 blackbox, db_service):
        self.status_mgr = status_mgr
        self.event_bus = event_bus
        self.throttle_ctrl = throttle_ctrl
        self.blackbox = blackbox
        self.db_service = db_service
        self._flight_counter = 0
        
        # Load next flight ID from data/flights folder to persist counter
        self._init_flight_counter()

    def _init_flight_counter(self):
        try:
            import os
            base_dir = "data/flights"
            os.makedirs(base_dir, exist_ok=True)
            existing = []
            for fname in os.listdir(base_dir):
                if fname.startswith("flight_") and fname.endswith(".db"):
                    try:
                        num = int(fname.replace("flight_", "").replace(".db", ""))
                        existing.append(num)
                    except ValueError:
                        pass
            self._flight_counter = max(existing) if existing else 0
        except Exception as e:
            log.warning(f"Could not load flight ID history: {e}")
            self._flight_counter = 0

    def arm(self) -> tuple[bool, str]:
        """
        Executes pre-arm checklist. Transitions READY -> ARMED on success.
        Throttle is forced to 0.0.
        """
        state = self.status_mgr.get_state()
        curr_state = state["flight_state"]
        
        if curr_state != "READY":
            return False, f"Arming rejected: System is in state '{curr_state}' (must be READY)"

        # Pre-arm checklist
        sensors = state["sensor_status"]
        if sensors.get("battery") != "ONLINE":
            return False, "Arming rejected: Battery monitor is offline or in error"
        if not str(sensors.get("propulsion_current", "")).startswith("ONLINE"):
            return False, "Arming rejected: ESC current sensor is offline or in error"
        if sensors.get("imu") != "ONLINE":
            # IMU optional check (ground/bench mode can waive this, but flight mode requires it)
            if state["controls"].get("flight_mode") == "flight":
                return False, "Arming rejected: IMU is offline (required for Flight mode)"
                
        # Check active critical faults
        critical_faults = [f for f in state["active_faults"] if f.get("level") == "critical" or f.get("severity") == "critical"]
        if critical_faults:
            return False, f"Arming rejected: {len(critical_faults)} active critical fault(s) present"

        # Check battery SoC
        soc = state["battery"]["soc"]
        if soc < 20.0:
            return False, f"Arming rejected: Battery critically low ({soc:.1f}%)"

        # Preconditions passed -> Transition
        self._flight_counter += 1
        self.status_mgr.update_state("system_health.flight_id", self._flight_counter)
        
        success = self.status_mgr.transition_to("ARMED", reason="Pre-arm checklist passed")
        if success:
            # Force throttle setpoint to zero
            self.throttle_ctrl.emergency_stop()
            self.status_mgr.update_state("controls.last_operator_heartbeat", time.time())
            
            # Publish FlightStarted event
            self.event_bus.publish(FlightStarted(self._flight_counter))
            self.status_mgr.log_event(f"Pre-flight checklist PASSED. Flight {self._flight_counter:03d} started.")
            return True, "Armed successfully"
            
        return False, "Arming transition failed"

    def disarm(self) -> tuple[bool, str]:
        """
        Disarm sequence: forces throttle to 0, stops output, flushes blackbox, and registers session end.
        """
        state = self.status_mgr.get_state()
        curr_state = state["flight_state"]
        
        # Stop throttle outputs immediately
        self.throttle_ctrl.emergency_stop()
        
        if curr_state not in ["ARMED", "RUNNING", "FAULT"]:
            return False, f"Disarming rejected: System is in state '{curr_state}' (cannot disarm)"

        # Transition to DISARMED -> READY lifecycle
        self.status_mgr.transition_to("DISARMED", reason="Operator disarmed command")
        
        # Flush Blackbox file
        self.blackbox.flush_pending()
        
        # Compute flight statistics summary
        duration = 0.0
        # If database service is logged, get flight stats
        if self.db_service:
            # Get duration
            db_sessions = self.db_service.get_recent_flights(limit=1)
            if db_sessions:
                s = db_sessions[0]
                duration = time.time() - s.get("start_ts", time.time())
                
        summary = {
            "duration": duration,
            "avg_health": state["propulsion"]["health"],
            "fault_count": len(state["active_faults"])
        }
        
        # Publish FlightEnded event to persist stats
        self.event_bus.publish(FlightEnded(self._flight_counter, summary))
        
        # Transition DISARMED -> READY automatically after grace period
        self.status_mgr.transition_to("READY", force=True, reason="Cleanup complete")
        return True, "Disarmed successfully"

    def set_throttle(self, val: float) -> tuple[bool, str]:
        """Send operator throttle command (updates dead-man timer)."""
        self.status_mgr.update_state("controls.last_operator_heartbeat", time.time())
        return self.throttle_ctrl.set_target_throttle(val)

    def heartbeat(self):
        """Operator dead-man safety pulse."""
        self.status_mgr.update_state("controls.last_operator_heartbeat", time.time())

    def reset_fault(self) -> tuple[bool, str]:
        """Attempts to reset FSM out of FAULT back to READY if fault conditions have cleared."""
        state = self.status_mgr.get_state()
        curr_state = state["flight_state"]
        
        if curr_state != "FAULT":
            return False, f"Fault reset rejected: System is in state '{curr_state}' (not in FAULT)"

        # Verify active critical faults are cleared
        critical_faults = [f for f in state["active_faults"] if f.get("level") == "critical" or f.get("severity") == "critical"]
        if critical_faults:
            return False, f"Fault reset rejected: {len(critical_faults)} active critical fault(s) still present"

        # Transition FAULT -> READY
        success = self.status_mgr.transition_to("READY", force=True, reason="Operator fault acknowledgment")
        if success:
            self.throttle_ctrl.emergency_stop()
            self.status_mgr.log_event("FAULT reset acknowledged. Returning to READY.")
            return True, "Fault reset successful"
            
        return False, "FSM fault reset failed"

    def emergency_stop(self):
        """Immediate manual throttle cut and FAULT state trip."""
        self.throttle_ctrl.emergency_stop()
        self.status_mgr.transition_to("FAULT", force=True, reason="Manual emergency stop")
        self.status_mgr.log_event("EMERGENCY STOP TRIGGERED!")
