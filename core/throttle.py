import time
import logging

log = logging.getLogger(__name__)


class ThrottleProvider:
    """Interface abstraction for physical or simulated throttle output."""
    def __init__(self, source_type: str = "simulation"):
        self.source_type = source_type
        self._current_throttle = 0.0

    def get_throttle(self) -> float:
        return self._current_throttle

    def set_throttle(self, val: float):
        self._current_throttle = max(0.0, min(1.0, val))

    def emergency_stop(self):
        self._current_throttle = 0.0


class ThrottleController:
    """Dedicated throttle manager enforcing limits, ramping, and safety interlocks."""
    def __init__(self, status_mgr, max_throttle_limit: float = 0.8,
                 warn_current: float = 12.0, max_current: float = 15.0,
                 ramp_rate_per_sec: float = 0.5):
        self.status_mgr = status_mgr
        self.max_throttle_limit = max_throttle_limit    # e.g., 0.8 (80%)
        self.warn_current = warn_current                # current at which progressive limit starts
        self.max_current = max_current                  # absolute current limit
        self.ramp_rate_per_sec = ramp_rate_per_sec      # throttle rate limit per second
        self.provider = ThrottleProvider("simulation")
        self._current_actual_throttle = 0.0
        self._last_time = time.time()

    def update(self, current_measured_i: float):
        now = time.time()
        dt = now - self._last_time
        self._last_time = now
        
        # Limit dt
        dt = min(max(dt, 0.0), 1.0)
        
        state = self.status_mgr.get_state()
        flight_state = state["flight_state"]
        
        # Dead-man timer check (Operator Safety)
        if flight_state in ["ARMED", "RUNNING"]:
            timeout = state["controls"].get("deadman_timeout_sec", 10.0)
            last_hb = state["controls"].get("last_operator_heartbeat", now)
            if now - last_hb > timeout:
                log.error(f"Dead-man timeout: no operator heartbeat for {now - last_hb:.1f}s. Emergency Stop!")
                self.status_mgr.transition_to("FAULT", force=True, reason="Dead-man timer timeout")
                self.status_mgr.log_event("WATCHDOG: Dead-man timeout expired - cutting throttle")
                self.emergency_stop()
                return

        # Safe Cut state overrides
        if flight_state not in ["ARMED", "RUNNING"]:
            self._current_actual_throttle = 0.0
            self.provider.set_throttle(0.0)
            self.status_mgr.update_state("controls.throttle", 0.0)
            return

        target = state["controls"]["target_throttle"]
        
        # 1. Enforce soft absolute limit from vehicle profile
        target = min(target, self.max_throttle_limit)
        
        # 2. Check safety interlocks
        if self._is_safety_interlock_active(state):
            target = 0.0
            if self._current_actual_throttle > 0.0:
                self.status_mgr.log_event("Throttle cut: Safety interlock triggered (critical fault or battery low)")

        # 3. Soft Progressive Current Limiter
        if current_measured_i >= self.max_current:
            # Over absolute critical current limit -> FAULT transition
            log.error(f"Critical overcurrent detected: {current_measured_i:.1f}A >= {self.max_current:.1f}A. Tripping FAULT.")
            self.status_mgr.transition_to("FAULT", force=True, reason=f"Critical overcurrent: {current_measured_i:.1f}A")
            self.emergency_stop()
            return
        elif current_measured_i > self.warn_current:
            # Between warn and max: linearly scale back maximum allowed throttle
            over = current_measured_i - self.warn_current
            span = self.max_current - self.warn_current
            reduction_ratio = min(1.0, over / max(span, 0.1))
            
            # Gradually reduce target throttle towards 0
            max_allowed = self.max_throttle_limit * (1.0 - reduction_ratio)
            if target > max_allowed:
                target = max_allowed
                self.status_mgr.log_event(f"Progressive Current Limiting active: Throttle capped at {target*100:.1f}%")

        # 4. Enforce throttle ramp rate limit
        max_delta = self.ramp_rate_per_sec * dt
        diff = target - self._current_actual_throttle
        
        if abs(diff) > max_delta:
            delta = max_delta if diff > 0 else -max_delta
            self._current_actual_throttle += delta
        else:
            self._current_actual_throttle = target
            
        self.provider.set_throttle(self._current_actual_throttle)
        self.status_mgr.update_state("controls.throttle", self._current_actual_throttle)
        
        # Automatic RUNNING <-> ARMED transition based on active throttle
        if flight_state == "ARMED" and self._current_actual_throttle > 0.0:
            self.status_mgr.transition_to("RUNNING", reason="Throttle active")
        elif flight_state == "RUNNING" and self._current_actual_throttle == 0.0:
            self.status_mgr.transition_to("ARMED", reason="Throttle zero")

    def _is_safety_interlock_active(self, state) -> bool:
        # Check active critical faults
        for f in state["active_faults"]:
            if f.get("severity") == "critical" or f.get("level") == "critical":
                return True
                
        # Check battery critical
        batt_v = state["battery"]["voltage"]
        batt_soc = state["battery"]["soc"]
        # Allow running with 0 in simulator if not initialized
        if batt_soc <= 10.0 or (batt_v > 0 and batt_v < 9.9):
            return True
            
        return False

    def set_target_throttle(self, val: float) -> tuple[bool, str]:
        state = self.status_mgr.get_state()
        if state["flight_state"] not in ["ARMED", "RUNNING"]:
            return False, f"Rejected: Cannot set throttle in state {state['flight_state']}"
            
        if self._is_safety_interlock_active(state):
            return False, "Rejected: Safety interlocks active"
            
        self.status_mgr.update_state("controls.target_throttle", max(0.0, min(1.0, val)))
        # Update heartbeat on command
        self.status_mgr.update_state("controls.last_operator_heartbeat", time.time())
        return True, "Throttle target set"

    def emergency_stop(self):
        self._current_actual_throttle = 0.0
        self.provider.emergency_stop()
        self.status_mgr.update_state("controls.throttle", 0.0)
        self.status_mgr.update_state("controls.target_throttle", 0.0)
