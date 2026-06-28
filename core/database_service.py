import time
import json
import logging
import threading
import queue
from typing import Optional

from core.logger import TelemetryDB

log = logging.getLogger(__name__)


# Event classes referenced in pub/sub
class HealthUpdated:
    def __init__(self, frame):
        self.frame = frame


class FlightStarted:
    def __init__(self, flight_id: int):
        self.flight_id = flight_id


class FlightEnded:
    def __init__(self, flight_id: int, summary: dict):
        self.flight_id = flight_id
        self.summary = summary


class DatabaseService(threading.Thread):
    """Background database worker subscribing to events and persisting log entries at 2 Hz."""
    def __init__(self, status_mgr, event_bus, db_path: str):
        super().__init__(name="database-service", daemon=True)
        self.status_mgr = status_mgr
        self.event_bus = event_bus
        self.db_path = db_path
        self.db: Optional[TelemetryDB] = None
        self._running = False
        
        self._frame_queue = queue.Queue(maxsize=10)
        self._active_session_rowid: Optional[int] = None
        self._active_flight_id: int = 0
        
        # Max stats accumulators during active flight
        self._max_current = 0.0
        self._max_temp = 0.0
        self._max_vib = 0.0
        self._health_scores = []
        self._fault_count = 0

    def start(self):
        self._running = True
        self.db = TelemetryDB(self.db_path)
        self._init_extended_schema()
        
        # Subscribe to EventBus
        self.event_bus.subscribe(HealthUpdated, self._on_health_updated)
        self.event_bus.subscribe(FlightStarted, self._on_flight_started)
        self.event_bus.subscribe(FlightEnded, self._on_flight_ended)
        
        super().start()

    def stop(self):
        self._running = False

    def _init_extended_schema(self):
        c = self.db._conn.cursor()
        
        # 1. Derived features table
        c.execute("""
        CREATE TABLE IF NOT EXISTS derived_features (
            id                     INTEGER PRIMARY KEY AUTOINCREMENT,
            ts                     REAL NOT NULL,
            r_internal             REAL,
            current_deviation_pct  REAL,
            vib_rms                REAL,
            vib_peak_freq          REAL,
            thermal_slope          REAL,
            battery_health         REAL,
            propulsion_health      REAL,
            diag_confidence        REAL
        );
        """)
        
        # 2. Flight sessions table
        c.execute("""
        CREATE TABLE IF NOT EXISTS flight_sessions (
            id                  INTEGER PRIMARY KEY AUTOINCREMENT,
            flight_id           INTEGER NOT NULL,
            vehicle_type        TEXT,
            hw_config_hash      TEXT,
            sw_version          TEXT,
            git_commit          TEXT,
            calibration_version TEXT,
            operator            TEXT,
            flight_mode         TEXT,
            start_ts            REAL NOT NULL,
            end_ts              REAL,
            duration_s          REAL,
            max_batt_current    REAL,
            max_esc_temp        REAL,
            max_vibration_rms   REAL,
            avg_health          REAL,
            fault_count         INTEGER
        );
        """)
        
        # 3. Cross-flight PHM health index table
        c.execute("""
        CREATE TABLE IF NOT EXISTS flight_health_indices (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            flight_id       INTEGER NOT NULL,
            recorded_at     REAL NOT NULL,
            pbi_avg         REAL,
            pbi_max         REAL,
            pli_avg         REAL,
            pli_max         REAL,
            eti_avg         REAL,
            eti_max         REAL,
            bsi_avg         REAL,
            bsi_max         REAL,
            health_score_avg REAL,
            n_conditions    INTEGER
        );
        """)

        c.execute("CREATE INDEX IF NOT EXISTS idx_derived_ts ON derived_features(ts);")
        c.execute("CREATE INDEX IF NOT EXISTS idx_sessions_fid ON flight_sessions(flight_id);")
        c.execute("CREATE INDEX IF NOT EXISTS idx_phi_fid ON flight_health_indices(flight_id);")
        self.db._conn.commit()

    def _on_health_updated(self, event: HealthUpdated):
        # Enqueue frame for logging thread to process at 2Hz throttle rate
        try:
            self._frame_queue.put_nowait(event.frame)
        except queue.Full:
            pass

    def _on_flight_started(self, event: FlightStarted):
        self._active_flight_id = event.flight_id
        self._max_current = 0.0
        self._max_temp = 0.0
        self._max_vib = 0.0
        self._health_scores = []
        self._fault_count = 0
        
        state = self.status_mgr.get_state()
        mode = state["controls"]["flight_mode"]
        vehicle = state.get("vehicle_type", "fixed-wing")
        
        # Start flight session in DB
        with self.db._conn:
            c = self.db._conn.execute(
                """INSERT INTO flight_sessions 
                   (flight_id, vehicle_type, hw_config_hash, sw_version, git_commit, 
                    calibration_version, operator, flight_mode, start_ts)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (event.flight_id, vehicle, "default_hw", "1.0.0", "c3b9aa8", 
                 "v1.0", "UAV_OPERATOR", mode, time.time())
            )
            self._active_session_rowid = c.lastrowid
        log.info(f"DB registered flight session start for Flight {event.flight_id} (row {self._active_session_rowid})")

    def _on_flight_ended(self, event: FlightEnded):
        if not self._active_session_rowid:
            return

        summary = event.summary
        end_t = time.time()

        # Update flight session in DB
        with self.db._conn:
            self.db._conn.execute(
                """UPDATE flight_sessions SET
                   end_ts = ?,
                   duration_s = ?,
                   max_batt_current = ?,
                   max_esc_temp = ?,
                   max_vibration_rms = ?,
                   avg_health = ?,
                   fault_count = ?
                   WHERE id = ?""",
                (end_t, summary.get("duration", 0.0),
                 self._max_current, self._max_temp, self._max_vib,
                 summary.get("avg_health", 100.0), summary.get("fault_count", 0),
                 self._active_session_rowid)
            )
        log.info(f"DB updated flight session end for Flight {self._active_flight_id}. Duration: {summary.get('duration', 0.0):.1f}s")

        # Write PHM cross-flight health index summary
        phm_summary = summary.get("phm_indices", {})
        if phm_summary:
            self._write_flight_health_indices(self._active_flight_id, phm_summary)

        self._active_session_rowid = None

    def _write_flight_health_indices(self, flight_id: int, phm: dict):
        """Write one row to flight_health_indices per completed flight."""
        try:
            def get(index_key, stat):
                return phm.get(index_key, {}).get(stat, 0.0)

            with self.db._conn:
                self.db._conn.execute(
                    """INSERT INTO flight_health_indices
                       (flight_id, recorded_at,
                        pbi_avg, pbi_max, pli_avg, pli_max,
                        eti_avg, eti_max, bsi_avg, bsi_max,
                        health_score_avg, n_conditions)
                       VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                    (
                        flight_id, time.time(),
                        get("pbi", "avg"), get("pbi", "max"),
                        get("pli", "avg"), get("pli", "max"),
                        get("eti", "avg"), get("eti", "max"),
                        get("bsi", "avg"), get("bsi", "max"),
                        get("health_score", "avg"),
                        phm.get("n_conditions", 0),
                    )
                )
            log.info("Wrote flight_health_indices for flight %d", flight_id)
        except Exception as exc:
            log.error("Failed to write flight_health_indices: %s", exc)

    def run(self):
        log.info("DatabaseService thread started.")
        last_log_time = 0
        
        while self._running:
            self.status_mgr.ping_watchdog("database")
            
            # Fetch latest telemetry frame from queue (block with timeout)
            frame = None
            try:
                # We log at 2 Hz, so we consume the latest frame in queue every 500ms
                frame = self._frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue
                
            now = time.time()
            if now - last_log_time >= 0.5:  # 2 Hz logging rate
                last_log_time = now
                self._persist_frame(frame)
                
            # Keep queue clean (discard older frames)
            while not self._frame_queue.empty():
                try:
                    self._frame_queue.get_nowait()
                except queue.Empty:
                    break

    def _persist_frame(self, frame):
        try:
            # 1. Update flight stats if session is active
            if self._active_session_rowid:
                self._max_current = max(self._max_current, frame.battery_current)
                self._max_temp = max(self._max_temp, frame.esc_temp)
                self._max_vib = max(self._max_vib, frame.imu_rms)

            # 2. Log raw telemetry
            self.db.log_telemetry(
                batt_v=frame.battery_voltage,
                batt_i=frame.battery_current,
                batt_soc=frame.soc or 100.0,
                batt_power=frame.battery_power,
                motor_currents=[frame.esc_current],
                status=self.status_mgr.get_state()["flight_state"]
            )

            # 3. Log derived features (PHM-aware)
            phm = frame.features.get("phm", {}) if frame.features else {}
            indices = phm.get("indices", {})
            dev = phm.get("deviation", {})
            conditions = phm.get("conditions", [])
            health_score = indices.get("health_score", 100.0)
            confidence = dev.get("confidence", 1.0)
            current_dev_pct = dev.get("current_deviation", 0.0) * 100.0

            if self._active_session_rowid:
                self._health_scores.append(health_score)

            with self.db._conn:
                self.db._conn.execute(
                    """INSERT INTO derived_features
                       (ts, r_internal, current_deviation_pct, vib_rms, vib_peak_freq,
                        thermal_slope, battery_health, propulsion_health, diag_confidence)
                       VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                    (frame.timestamp, 0.0, current_dev_pct, frame.imu_rms,
                     frame.imu_peak_freq or 0.0, 0.0, 100.0, health_score, confidence)
                )

            self.db.log_health(
                overall=health_score,
                battery=100.0,
                motor_health=[health_score],
                esc_health=[health_score]
            )
            
        except Exception as e:
            log.error(f"Failed to persist telemetry frame: {e}", exc_info=True)
            
    def get_recent_flights(self, limit: int = 50) -> list:
        try:
            rows = self.db._conn.execute(
                "SELECT * FROM flight_sessions ORDER BY start_ts DESC LIMIT ?", (limit,)
            ).fetchall()
            return [dict(r) for r in rows]
        except Exception as e:
            log.error(f"Failed to query flight sessions: {e}")
            return []

    def get_flight_index_history(self, limit: int = 50) -> list:
        """Return cross-flight PHM index history for trend charting."""
        try:
            rows = self.db._conn.execute(
                """SELECT * FROM flight_health_indices
                   ORDER BY recorded_at DESC LIMIT ?""", (limit,)
            ).fetchall()
            return [dict(r) for r in reversed(rows)]  # chronological order
        except Exception as exc:
            log.error("Failed to query flight_health_indices: %s", exc)
            return []
