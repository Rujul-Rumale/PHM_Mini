import time
import logging
import threading

log = logging.getLogger(__name__)


class WatchdogService(threading.Thread):
    """Monitors heartbeat timestamps of loops and services. Transition to FAULT on critical failure."""
    def __init__(self, status_mgr, services: dict):
        super().__init__(name="watchdog-service", daemon=True)
        self.status_mgr = status_mgr
        self.services = services  # dict of name -> thread object reference
        self._running = False

    def start(self):
        self._running = True
        super().start()

    def stop(self):
        self._running = False

    def run(self):
        log.info("Watchdog service started.")
        while self._running:
            time.sleep(1.0)
            now = time.time()
            state = self.status_mgr.get_state()
            pings = state.get("loop_stats", {}).get("watchdog_pings", {})
            flight_state = state["flight_state"]
            
            # 1. Check MonitorService heartbeat (critical loop)
            # If armed or running, watchdog is very strict (2.0s). Otherwise 5.0s.
            last_monitor_ping = pings.get("monitor")
            if last_monitor_ping:
                timeout = 2.0 if flight_state in ["ARMED", "RUNNING"] else 5.0
                if now - last_monitor_ping > timeout:
                    log.error(f"WATCHDOG: MonitorService loop has HUNG! Last ping was {now - last_monitor_ping:.1f}s ago. Transitioning to FAULT.")
                    self.status_mgr.transition_to("FAULT", force=True, reason="Watchdog: MonitorService hung")
                    self.status_mgr.log_event("WATCHDOG: MonitorService failed heartbeat check - emergency shutdown triggered")
            
            # 2. Check DatabaseService heartbeat
            last_db_ping = pings.get("database")
            if last_db_ping and now - last_db_ping > 10.0:
                log.warning(f"WATCHDOG: DatabaseService has not pinged in {now - last_db_ping:.1f}s.")
                
            # 3. Check thread liveness and attempt restart for crashed background loops
            for name, thread in list(self.services.items()):
                if thread and not thread.is_alive():
                    log.error(f"WATCHDOG: Thread '{name}' has CRASHED! Attempting transition to safety.")
                    self.status_mgr.transition_to("FAULT", force=True, reason=f"Watchdog: Thread {name} crashed")
