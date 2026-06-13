import json
import os
import time
import logging
from collections import deque
from dataclasses import dataclass, asdict, field
from typing import Optional

logger = logging.getLogger(__name__)

BLACKBOX_DIR = "data/blackbox"
DEFAULT_MAX_SAMPLES = 1500
POST_EVENT_SECONDS = 30
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
        self._post_ring = deque(maxlen=int(POST_EVENT_SECONDS * 40))
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
        before = list(self._ring)
        snapshot = [asdict(s) for s in before] if before else None

        event = FaultEvent(
            timestamp=time.time(),
            component=component, fault_type=fault_type,
            confidence=confidence, severity=severity,
            evidence=evidence, before_snapshot=snapshot,
            after_snapshot=None,
        )
        self._events.append(event)
        self._recording_after = True
        self._after_start = event.timestamp
        self._post_ring.clear()

        logger.info("Fault event: %s/%s (conf=%.2f, sev=%s) — recording 30s post-event",
                    component, fault_type, confidence, severity)
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
        with open(filepath, "w") as f:
            json.dump(event.to_dict(), f, indent=2)
        self._event_id += 1
        logger.info("Fault event saved to %s (%d pre + %d post samples)",
                    filepath, len(event.before_snapshot or []),
                    len(event.after_snapshot or []))

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
