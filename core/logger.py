import sqlite3
import time
import logging
import os
from typing import List, Dict, Optional

log = logging.getLogger(__name__)

CREATE_TELEMETRY = """
CREATE TABLE IF NOT EXISTS telemetry (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    batt_v      REAL,
    batt_i      REAL,
    batt_soc    REAL,
    batt_power  REAL,
    m1_i        REAL,
    m2_i        REAL,
    m3_i        REAL,
    m4_i        REAL,
    status      TEXT DEFAULT 'OK'
);
"""

CREATE_FAULTS = """
CREATE TABLE IF NOT EXISTS faults (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    source      TEXT NOT NULL,
    code        TEXT NOT NULL,
    severity    TEXT NOT NULL,
    value       REAL,
    threshold   REAL,
    message     TEXT,
    cleared     INTEGER DEFAULT 0,
    cleared_ts  REAL
);
"""

CREATE_HEALTH_LOG = """
CREATE TABLE IF NOT EXISTS health_log (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    ts              REAL NOT NULL,
    overall_health  REAL,
    battery_health  REAL,
    motor1_health   REAL,
    motor2_health   REAL,
    motor3_health   REAL,
    motor4_health   REAL,
    esc1_health     REAL,
    esc2_health     REAL,
    esc3_health     REAL,
    esc4_health     REAL
);
"""

CREATE_FAULT_EVENTS = """
CREATE TABLE IF NOT EXISTS fault_events (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL NOT NULL,
    component   TEXT NOT NULL,
    fault_type  TEXT NOT NULL,
    confidence  REAL,
    severity    TEXT,
    evidence    TEXT
);
"""

CREATE_INDEXES = [
    "CREATE INDEX IF NOT EXISTS idx_telem_ts ON telemetry(ts);",
    "CREATE INDEX IF NOT EXISTS idx_faults_ts ON faults(ts);",
    "CREATE INDEX IF NOT EXISTS idx_faults_cleared ON faults(cleared);",
    "CREATE INDEX IF NOT EXISTS idx_health_ts ON health_log(ts);",
    "CREATE INDEX IF NOT EXISTS idx_fault_events_ts ON fault_events(ts);",
    "CREATE INDEX IF NOT EXISTS idx_fault_events_comp ON fault_events(component);",
]


class TelemetryDB:
    def __init__(self, db_path: str):
        os.makedirs(os.path.dirname(db_path) or ".", exist_ok=True)
        self._path = db_path
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._init_schema()
        self._active_fault_ids: Dict[str, int] = {}

    def _init_schema(self):
        c = self._conn.cursor()
        c.execute(CREATE_TELEMETRY)
        c.execute(CREATE_FAULTS)
        c.execute(CREATE_HEALTH_LOG)
        c.execute(CREATE_FAULT_EVENTS)
        for idx in CREATE_INDEXES:
            c.execute(idx)
        self._conn.commit()

    def log_telemetry(self, batt_v, batt_i, batt_soc, batt_power,
                      motor_currents: list, status: str):
        m = motor_currents + [None] * (4 - len(motor_currents))
        self._conn.execute(
            """INSERT INTO telemetry
               (ts, batt_v, batt_i, batt_soc, batt_power, m1_i, m2_i, m3_i, m4_i, status)
               VALUES (?,?,?,?,?,?,?,?,?,?)""",
            (time.time(), batt_v, batt_i, batt_soc, batt_power,
             m[0], m[1], m[2], m[3], status)
        )
        self._conn.commit()

    def log_health(self, overall: float, battery: float,
                   motor_health: list[float], esc_health: list[float]):
        m = motor_health + [0.0] * (4 - len(motor_health))
        e = esc_health + [0.0] * (4 - len(esc_health))
        self._conn.execute(
            """INSERT INTO health_log
               (ts, overall_health, battery_health,
                motor1_health, motor2_health, motor3_health, motor4_health,
                esc1_health, esc2_health, esc3_health, esc4_health)
               VALUES (?,?,?,?,?,?,?,?,?,?,?)""",
            (time.time(), overall, battery,
             m[0], m[1], m[2], m[3],
             e[0], e[1], e[2], e[3])
        )
        self._conn.commit()

    def log_fault_event(self, component: str, fault_type: str,
                        confidence: float, severity: str, evidence: dict):
        import json
        self._conn.execute(
            """INSERT INTO fault_events (ts, component, fault_type, confidence, severity, evidence)
               VALUES (?,?,?,?,?,?)""",
            (time.time(), component, fault_type, confidence,
             severity, json.dumps(evidence))
        )
        self._conn.commit()

    def log_fault(self, fault) -> int:
        key = f"{fault.source}:{fault.code}"
        if key in self._active_fault_ids:
            return self._active_fault_ids[key]
        c = self._conn.execute(
            """INSERT INTO faults (ts, source, code, severity, value, threshold, message)
               VALUES (?,?,?,?,?,?,?)""",
            (fault.timestamp, fault.source, fault.code, fault.severity.value,
             fault.value, fault.threshold, fault.message)
        )
        self._conn.commit()
        self._active_fault_ids[key] = c.lastrowid
        return c.lastrowid

    def clear_fault(self, source: str, code: str):
        key = f"{source}:{code}"
        if key in self._active_fault_ids:
            self._conn.execute(
                "UPDATE faults SET cleared=1, cleared_ts=? WHERE id=?",
                (time.time(), self._active_fault_ids[key])
            )
            self._conn.commit()
            del self._active_fault_ids[key]

    def get_recent_telemetry(self, limit: int = 300) -> List[Dict]:
        rows = self._conn.execute(
            "SELECT * FROM telemetry ORDER BY ts DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in reversed(rows)]

    def get_active_faults(self) -> List[Dict]:
        rows = self._conn.execute(
            "SELECT * FROM faults WHERE cleared=0 ORDER BY ts DESC"
        ).fetchall()
        return [dict(r) for r in rows]

    def get_recent_fault_history(self, limit: int = 50) -> List[Dict]:
        rows = self._conn.execute(
            "SELECT * FROM faults ORDER BY ts DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]

    def get_recent_health(self, limit: int = 100) -> List[Dict]:
        rows = self._conn.execute(
            "SELECT * FROM health_log ORDER BY ts DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]

    def get_recent_fault_events(self, limit: int = 50) -> List[Dict]:
        rows = self._conn.execute(
            "SELECT * FROM fault_events ORDER BY ts DESC LIMIT ?", (limit,)
        ).fetchall()
        return [dict(r) for r in rows]

    def get_stats(self) -> Dict:
        row = self._conn.execute(
            "SELECT COUNT(*) as total, MIN(ts) as start_ts FROM telemetry"
        ).fetchone()
        return dict(row)
