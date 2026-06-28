from typing import Protocol

class CurrentSensor(Protocol):
    def begin(self):
        """Initialize the sensor registers and calibration."""
        ...

    def read_voltage(self) -> float:
        """Read bus voltage in Volts."""
        ...

    def read_current(self) -> float:
        """Read current in Amperes."""
        ...

    def read_power(self) -> float:
        """Read power consumption in Watts."""
        ...


class TemperatureSensor(Protocol):
    def begin(self):
        """Initialize the temperature sensor."""
        ...

    def read_temperature(self) -> float:
        """Read temperature in degrees Celsius."""
        ...
