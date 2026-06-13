# Drone Fault Monitor

Threshold-based battery and motor fault detection for RPi with INA226 sensors.

## Hardware Setup

### I2C Wiring (RPi → INA226)
```
RPi Pin 1  (3.3V)  → VCC
RPi Pin 6  (GND)   → GND
RPi Pin 3  (SDA1)  → SDA
RPi Pin 5  (SCL1)  → SCL
```

Enable I2C: `sudo raspi-config` → Interface Options → I2C

### INA226 Address Configuration (A0/A1 pins)
```
Battery sensor:  A1=GND, A0=GND → 0x40
Motor 1 ESC:     A1=GND, A0=VCC → 0x41
Motor 2 ESC:     A1=VCC, A0=GND → 0x42
Motor 3 ESC:     A1=VCC, A0=VCC → 0x43
Motor 4 ESC:     A1=GND, A0=SDA → 0x44  (check INA226 datasheet Table 2)
```

### Shunt Resistor Selection
- Battery (high current ~80A): 1mΩ shunt. Use a dedicated 4-terminal current shunt.
- Motors (up to 30A each): 10mΩ shunt is fine for a PCB trace or 2512 resistor.

INA226 shunt voltage range: ±81.92mV. Verify: I_max × R_shunt < 82mV.

### Placement
- Battery sensor: on main power rail AFTER the XT60 connector
- Motor sensors: each on the ESC output power trace (between ESC +V and motor common rail)

## Software Setup

```bash
# Install dependencies
pip3 install -r requirements.txt

# Verify I2C devices visible
i2cdetect -y 1

# Run with real hardware
# NOTE: Must run from project root (drone_monitor/) — paths are relative
python3 monitor.py

# Run in simulation mode (no hardware needed)
python3 monitor.py --simulate

# Custom config
python3 monitor.py --config config/thresholds.yaml --simulate
```

**IMPORTANT:** Run all commands from the `drone_monitor/` directory. Log and DB paths are relative.

Dashboard: http://<rpi-ip>:5050

## Configuration

Edit `config/thresholds.yaml`. Key parameters:

| Parameter | Meaning |
|-----------|---------|
| `min_voltage` | Hard undervoltage cutoff (V) |
| `max_dv_dt` | Max voltage sag rate (V/s, negative) |
| `soc_critical_percent` | Emergency low SoC level |
| `min_current_armed` | Below this = open circuit fault |
| `max_imbalance_ratio` | Motor current imbalance threshold |
| `fault_persistence_count` | Consecutive readings before fault declared |

## Fault Codes

### Battery
| Code | Severity | Condition |
|------|----------|-----------|
| `UNDERVOLTAGE_CRITICAL` | CRITICAL | V < min_voltage |
| `UNDERVOLTAGE_WARN` | WARNING | V < warn_voltage |
| `OVERCURRENT_CRITICAL` | CRITICAL | I > max_current |
| `OVERCURRENT_WARN` | WARNING | I > warn_current |
| `VOLTAGE_SAG_RATE` | WARNING | dV/dt < max_dv_dt |
| `LOW_SOC_CRITICAL` | CRITICAL | SoC < soc_critical_percent |
| `LOW_SOC_WARN` | WARNING | SoC < soc_warn_percent |

### Motors
| Code | Severity | Condition |
|------|----------|-----------|
| `OVERCURRENT_CRITICAL` | CRITICAL | I > max_current_per_motor |
| `OVERCURRENT_WARN` | WARNING | I > warn_current_per_motor |
| `OPEN_CIRCUIT` | CRITICAL | I < min_current_armed while armed |
| `CURRENT_IMBALANCE` | WARNING | >40% deviation from motor fleet mean |
| `SENSOR_MISSING` | CRITICAL | I2C read failure |

## API Endpoints

| Endpoint | Description |
|----------|-------------|
| `GET /api/latest` | Current readings + active faults (JSON) |
| `GET /api/telemetry` | Last 300 telemetry rows |
| `GET /api/faults` | Active fault list |
| `GET /api/fault_history` | Last 50 fault events |
| `POST /api/arm` | Set armed=True (enables open-circuit detection) |
| `POST /api/disarm` | Set armed=False |

## SoC Estimation Notes

Uses coulomb counting initialized from OCV at startup. Accuracy degrades over
time without resting voltage; acceptable for flight monitoring.
The OCV table in `core/soc_estimator.py` is for generic LiPo — replace with your
cell's datasheet curve for better accuracy.

## What Doesn't Work Yet

These features exist in the codebase but are incomplete or quarantined:

- **Anomaly Detection (ML):** IsolationForest + RandomForest detector is in `experimental/`. Requires `core/failure_simulator.py` (not yet built) to generate training data. See Phase 3 of the roadmap.
- **IMU / Vibration Pipeline:** `FeatureExtractor.inject_vibration()` exists but no IMU driver is wired. Vibration features return `None`.
- **Calibration REST API:** POST/GET calibration endpoints are documented in `calibrate.py` but not wired into the web dashboard.
- **PyThrust Propeller Database:** Physics-based expected-current solver falls back to a simplified model because the propeller aerodynamics database (`data/propellers/apc_202602`) is not bundled with this project.

## Extension Points

- **GPIO fault output**: Add `RPi.GPIO` output in monitor.py loop on `CRITICAL` status
- **MAVLink integration**: Inject `SYS_STATUS` or `BATTERY_STATUS` via pymavlink
- **More sensors**: Add Hall effect / encoder for RPM; wire via GPIO interrupt or I2C encoder IC
- **ESC telemetry**: Many ESCs output DSHOT telemetry or UART — replace motor INA226s with ESC telemetry parser for RPM + temperature
