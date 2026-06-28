import math
from collections import deque
from typing import Optional


class MovingAverageFilter:
    """Simple moving average filter to smooth noisy signals (e.g. current)."""
    def __init__(self, window_size: int = 5):
        self._window_size = window_size
        self._buffer = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        self._buffer.append(value)
        return sum(self._buffer) / len(self._buffer)


class LowPassFilter:
    """First-order low-pass filter (RC filter representation) for temperature signals."""
    def __init__(self, tau: float = 10.0, initial_value: Optional[float] = None):
        self._tau = tau  # Time constant in seconds
        self._y = initial_value

    def update(self, x: float, dt: float) -> float:
        if self._y is None:
            self._y = x
            return x
        if dt <= 0:
            return self._y
            
        alpha = dt / (self._tau + dt)
        self._y = alpha * x + (1.0 - alpha) * self._y
        return self._y


class HighPassFilter:
    """First-order high-pass filter (used to eliminate DC gravity offset from raw accel)."""
    def __init__(self, tau: float = 0.5):
        self._tau = tau
        self._y = 0.0
        self._x_prev = None

    def update(self, x: float, dt: float) -> float:
        if self._x_prev is None:
            self._x_prev = x
            self._y = 0.0
            return 0.0
        if dt <= 0:
            return self._y
            
        alpha = self._tau / (self._tau + dt)
        self._y = alpha * (self._y + x - self._x_prev)
        self._x_prev = x
        return self._y
