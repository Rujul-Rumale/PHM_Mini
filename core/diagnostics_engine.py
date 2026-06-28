"""
PHM Diagnostics Engine — orchestrates the deviation → index → condition pipeline.

Pipeline per HealthUpdated event:
  1. DeviationEngine.compute()        → DeviationFrame
  2. HealthIndexEngine.compute()      → HealthIndices
  3. Diagnostics.classify()           → list[Condition]
  4. Diagnostics.update()             → confirmed conditions (debounced)
  5. Diagnostics.check_battery_limits() → hard safety limits
  6. StatusManager update             → push indices + conditions
  7. FSM trip                         → critical conditions trigger FAULT state
  8. Watchdog ping
"""

import logging
from typing import Optional

from core.telemetry_frame import TelemetryFrame
from core.deviation_engine import DeviationEngine, DeviationFrame
from core.health_engine import HealthIndexEngine, HealthIndices
from core.diagnostics import Diagnostics, Condition
from core.database_service import HealthUpdated

log = logging.getLogger(__name__)


class DiagnosticsEngine:
    """
    Subscribes to HealthUpdated events.
    Runs the full PHM pipeline and publishes results to StatusManager.
    """

    def __init__(self, status_mgr, event_bus, baseline_mgr):
        self.status_mgr = status_mgr
        self.event_bus = event_bus
        self.baseline_mgr = baseline_mgr

        self.deviation_engine = DeviationEngine()
        self.index_engine = HealthIndexEngine()
        self.diagnostics = Diagnostics(persistence_count=3)

        # Latest computed values — exposed for REST API
        self.latest_deviation: Optional[DeviationFrame] = None
        self.latest_indices: Optional[HealthIndices] = None

        # Register event subscriber
        self.event_bus.subscribe(HealthUpdated, self._on_health_updated)

    # ── Event Handler ─────────────────────────────────────────────────────────

    def _on_health_updated(self, event: HealthUpdated):
        frame = event.frame

        # ── 1. Compute deviations ─────────────────────────────────────────
        dev = self.deviation_engine.compute(frame, self.baseline_mgr)
        self.latest_deviation = dev

        # ── 2. Compute health indices ──────────────────────────────────────
        indices = self.index_engine.compute(dev)
        self.latest_indices = indices

        # Append trends from index engine into indices object
        trends = self.index_engine.get_trends()
        indices.trend_1min = trends["pbi"]["trend_1min"]
        indices.trend_10min = trends["pbi"]["trend_10min"]
        indices.trend_flight = trends["pbi"]["trend_flight"]

        # ── 3. Classify observable conditions ─────────────────────────────
        raw_conditions = self.diagnostics.classify(indices, dev)

        # ── 4. Battery hard limits (absolute, not deviation-based) ─────────
        state = self.status_mgr.get_state()
        if frame.battery_sensor_quality == "ONLINE":
            battery_limits = self.diagnostics.check_battery_limits(
                voltage=frame.battery_voltage,
                current=frame.battery_current,
                soc=frame.soc,
                temp=frame.esc_temp_filtered,   # ESC temp as battery temp proxy
                temp_max=60.0,
            )
            raw_conditions.extend(battery_limits)

        # ── 5. Debounce — confirm persistent conditions ────────────────────
        new_confirmed = self.diagnostics.update(raw_conditions)
        active_conditions = self.diagnostics.get_active()

        # ── 6. Push to StatusManager ───────────────────────────────────────
        self.status_mgr.update_state("phm.indices", indices.to_dict())
        self.status_mgr.update_state("phm.deviation", dev.to_dict())
        self.status_mgr.update_state("phm.conditions", [c.to_dict() for c in active_conditions])
        self.status_mgr.update_state("phm.maintenance", self.diagnostics.maintenance_recommendation())
        self.status_mgr.update_state("phm.calibrated", dev.calibrated)
        self.status_mgr.update_state("phm.confidence", dev.confidence)

        # Legacy paths (dashboard tabs still expect these keys)
        self.status_mgr.update_state("propulsion.health", indices.health_score)
        self.status_mgr.update_state("overall_health", indices.health_score)
        self.status_mgr.update_state("active_faults", [c.to_dict() for c in active_conditions])

        # Index trend labels
        self.status_mgr.update_state("propulsion.trend_1min", indices.trend_1min)
        self.status_mgr.update_state("propulsion.trend_10min", indices.trend_10min)
        self.status_mgr.update_state("propulsion.trend_flight", indices.trend_flight)

        # Predicted failure indicator
        pred = self.diagnostics.predicted_failure()
        self.status_mgr.update_state("predicted_failure", pred.to_dict() if pred else None)

        # ── 7. Trip FSM on critical conditions ────────────────────────────
        for c in new_confirmed:
            if c.severity == "critical" and c.condition_id not in ("uncalibrated_operation",):
                if state["flight_state"] in ["ARMED", "RUNNING"]:
                    self.status_mgr.transition_to(
                        "FAULT", force=True,
                        reason=f"Critical PHM condition: {c.title}"
                    )
                    self.status_mgr.log_event(
                        f"SAFETY: Emergency cut! '{c.title}' (conf={c.confidence:.0%})"
                    )
                    break

        # ── 8. Enrich frame features dict for DatabaseService ─────────────
        if frame.features is not None:
            frame.features["phm"] = {
                "indices": indices.to_dict(),
                "deviation": dev.to_dict(),
                "conditions": [c.to_dict() for c in active_conditions],
                "maintenance": self.diagnostics.maintenance_recommendation(),
            }

        # ── 9. Watchdog ping ──────────────────────────────────────────────
        self.status_mgr.ping_watchdog("diagnostics")

    # ── Cross-flight summary (called by DatabaseService on FlightEnded) ───────

    def get_flight_summary(self) -> dict:
        """Called once per flight end to collect index averages for flight_health_indices table."""
        summary = self.index_engine.get_flight_summary()
        summary["n_conditions"] = len(self.diagnostics.get_history())
        return summary

    def reset_flight(self):
        """Called at flight start to clear per-flight index accumulations."""
        self.index_engine.reset_flight()
