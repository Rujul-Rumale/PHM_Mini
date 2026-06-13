import json
import os
import logging
from collections import deque
from dataclasses import dataclass, asdict, field
from typing import Optional
from datetime import datetime

logger = logging.getLogger(__name__)

BLACKBOX_DIR = "blackbox"
MAX_SAMPLES = 100
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
    before_snapshot: Optional[dict] = None

    def to_dict(self):
        return asdict(self)


class Blackbox:
    def __init__(self, directory: str = BLACKBOX_DIR, max_samples: int = MAX_SAMPLES):
        self.directory = directory
        self.max_samples = max_samples
        self._ring = deque(maxlen=max_samples)
        self._events: list[FaultEvent] = []
        self._event_id = 0
        os.makedirs(directory, exist_ok=True)

    def record_sample(self, voltage: float, current: float,
                      motor_currents: list[float], motor_vibrations: list[float],
                      motor_temps: list[float], throttle: float,
                      temperature: float, timestamp: Optional[float] = None):
        ts = timestamp or datetime.now().timestamp()
        sample = BlackboxSample(
            timestamp=ts, voltage=voltage, current=current,
            motor_currents=motor_currents, motor_vibrations=motor_vibrations,
            motor_temps=motor_temps, throttle=throttle,
            temperature=temperature,
        )
        self._ring.append(sample)

    def save_fault_event(self, component: str, fault_type: str,
                         confidence: float, severity: str, evidence: dict) -> FaultEvent:
        before = list(self._ring)
        snapshot = [asdict(s) for s in before] if before else None

        event = FaultEvent(
            timestamp=datetime.now().timestamp(),
            component=component, fault_type=fault_type,
            confidence=confidence, severity=severity,
            evidence=evidence, before_snapshot=snapshot,
        )
        self._events.append(event)

        filename = f"fault_{self._event_id:04d}_{component}_{fault_type}_{int(event.timestamp)}.json"
        filepath = os.path.join(self.directory, filename)
        with open(filepath, "w") as f:
            json.dump(event.to_dict(), f, indent=2)
        self._event_id += 1
        logger.info("Fault event saved to %s", filepath)

        if len(self._events) > MAX_EVENTS:
            self._events = self._events[-MAX_EVENTS:]

        return event

    def get_recent_samples(self, n: int = 30) -> list[dict]:
        return [asdict(s) for s in list(self._ring)[-n:]]

    def get_events(self, n: int = 20) -> list[dict]:
        return [e.to_dict() for e in self._events[-n:]]

    def get_timeline(self) -> list[dict]:
        timeline = []
        for e in self._events:
            timeline.append({
                "timestamp": e.timestamp,
                "component": e.component,
                "fault_type": e.fault_type,
                "confidence": e.confidence,
                "severity": e.severity,
            })
        return timeline
