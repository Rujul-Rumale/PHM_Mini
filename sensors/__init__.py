from sensors.base import CurrentSensor, TemperatureSensor
from sensors.ina226 import INA226, MockINA226
from sensors.ina219 import INA219, MockINA219
from sensors.lm75 import LM75, MockLM75
from sensors.mpu6050 import MPU6050, MockMPU6050

def make_current_sensor(sensor_type: str, bus, address: int, shunt_ohms: float, max_amps: float = 16.0) -> CurrentSensor:
    if sensor_type == "INA219":
        return INA219(bus, address, shunt_ohms, max_amps)
    elif sensor_type == "INA226":
        return INA226(bus, address, shunt_ohms, max_amps)
    else:
        raise ValueError(f"Unsupported current sensor type: {sensor_type}")

def make_temperature_sensor(sensor_type: str, bus, address: int) -> TemperatureSensor:
    if sensor_type == "LM75":
        return LM75(bus, address)
    else:
        raise ValueError(f"Unsupported temperature sensor type: {sensor_type}")
