import logging
import threading
from collections import defaultdict
from typing import Callable, Any, Type

log = logging.getLogger(__name__)


class EventBus:
    """Thread-safe synchronous event bus for service communication."""
    def __init__(self):
        self._listeners = defaultdict(list)
        self._lock = threading.Lock()

    def subscribe(self, event_type: Type[Any], callback: Callable[[Any], None]):
        with self._lock:
            if callback not in self._listeners[event_type]:
                self._listeners[event_type].append(callback)
                log.debug(f"Subscribed {callback.__name__ if hasattr(callback, '__name__') else callback} to {event_type.__name__}")

    def unsubscribe(self, event_type: Type[Any], callback: Callable[[Any], None]):
        with self._lock:
            if callback in self._listeners[event_type]:
                self._listeners[event_type].remove(callback)

    def publish(self, event: Any):
        event_type = type(event)
        callbacks = []
        with self._lock:
            # Copy to avoid holding lock during callback execution
            callbacks = list(self._listeners[event_type])
            
        for callback in callbacks:
            try:
                callback(event)
            except Exception as e:
                log.error(f"Error executing callback for event {event_type.__name__}: {e}", exc_info=True)
