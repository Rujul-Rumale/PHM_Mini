import json
import os
import time
import logging
from collections import deque
from dataclasses import dataclass, asdict
from typing import Optional

logger = logging.getLogger(__name__)

BLACKBOX_DIR = "data/blackbox"
DEFAULT_MAX_SAMPLES = 3000  # 60s at 50Hz
POST_EVENT_SECONDS = 30.0
MAX_EVENTS = 200


@dataclass
class BlackboxSample:
    timestamp: float
    voltage: float
    current: float
    motor_currents: list[float]
    motor_vibrations: list[float]
    motor_temps: list[float]
    throttle: float
    temperature: float


@dataclass
class FaultEvent:
    timestamp: float
    component: str
    fault_type: str
    confidence: float
    severity: str
    evidence: dict
    before_snapshot: Optional[list] = None
    after_snapshot: Optional[list] = None

    def to_dict(self):
        return asdict(self)


class Blackbox:
    def __init__(self, directory: str = BLACKBOX_DIR, max_samples: int = DEFAULT_MAX_SAMPLES):
        self.directory = directory
        self.max_samples = max_samples
        self._ring = deque(maxlen=max_samples)
        self._post_ring = deque(maxlen=int(POST_EVENT_SECONDS * 100)) # oversized to avoid truncation
        self._events: list[FaultEvent] = []
        self._event_id = 0
        self._recording_after = False
        self._after_start = 0.0
        os.makedirs(directory, exist_ok=True)

    def record_sample(self, voltage: float, current: float,
                      motor_currents: list[float], motor_vibrations: list[float],
                      motor_temps: list[float], throttle: float,
                      temperature: float, timestamp: Optional[float] = None):
        ts = timestamp or time.time()
        sample = BlackboxSample(
            timestamp=ts, voltage=voltage, current=current,
            motor_currents=motor_currents, motor_vibrations=motor_vibrations,
            motor_temps=motor_temps, throttle=throttle,
            temperature=temperature,
        )
        self._ring.append(sample)
        if self._recording_after:
            self._post_ring.append(sample)
            if ts - self._after_start >= POST_EVENT_SECONDS:
                self._finalize_after_event()

    def save_fault_event(self, component: str, fault_type: str,
                         confidence: float, severity: str, evidence: dict) -> Optional[FaultEvent]:
        # Capture the before snapshot: slice exactly the last 30 seconds of samples from the 60s ring buffer
        now = time.time()
        cutoff_time = now - 30.0
        
        before = list(self._ring)
        before_sliced = [s for s in before if s.timestamp >= cutoff_time]
        snapshot = [asdict(s) for s in before_sliced] if before_sliced else None

        event = FaultEvent(
            timestamp=now,
            component=component, fault_type=fault_type,
            confidence=confidence, severity=severity,
            evidence=evidence, before_snapshot=snapshot,
            after_snapshot=None,
        )
        self._events.append(event)
        self._recording_after = True
        self._after_start = event.timestamp
        self._post_ring.clear()

        logger.info(f"Blackbox Fault Event: {component}/{fault_type} triggered. Sliced {len(before_sliced)} pre-samples. Recording 30s post-event.")
        return event

    def _finalize_after_event(self):
        if not self._recording_after:
            return
        self._recording_after = False
        if not self._events:
            return
        event = self._events[-1]
        event.after_snapshot = [asdict(s) for s in list(self._post_ring)]

        filename = f"fault_{self._event_id:04d}_{event.component}_{event.fault_type}_{int(event.timestamp)}.json"
        filepath = os.path.join(self.directory, filename)
        try:
            with open(filepath, "w") as f:
                json.dump(event.to_dict(), f, indent=2)
            self._event_id += 1
            logger.info(f"Fault blackbox file successfully saved: {filepath} ({len(event.before_snapshot or [])} pre + {len(event.after_snapshot or [])} post samples)")
        except Exception as e:
            logger.error(f"Failed to write fault blackbox file: {e}")

        if len(self._events) > MAX_EVENTS:
            self._events = self._events[-MAX_EVENTS:]

    def flush_pending(self):
        self._finalize_after_event()

    def get_recent_samples(self, n: int = 30) -> list[dict]:
        return [asdict(s) for s in list(self._ring)[-n:]]

    def get_events(self, n: int = 20) -> list[dict]:
        return [e.to_dict() for e in self._events[-n:]]

    def get_timeline(self) -> list[dict]:
        return [{"timestamp": e.timestamp, "component": e.component,
                 "fault_type": e.fault_type, "confidence": e.confidence,
                 "severity": e.severity} for e in self._events]
