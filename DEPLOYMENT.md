# UAV PHM Monitor — Raspberry Pi Deployment Guide

## Overview

Deploy the UAV Prognostics and Health Management (PHM) system on a Raspberry Pi for real-time fault detection, battery monitoring, and vibration analysis. The system boots headlessly, runs three parallel threads (fast sensor loop, health analysis loop, web dashboard), and stores per-flight telemetry databases.

## Hardware Requirements

### Supported Raspberry Pi Models

| Model | Notes |
|-------|-------|
| Pi Zero 2 W | Adequate for 1-2 propulsion units |
| Pi 3 Model B+ | Good for quadcopter (4 units) |
| Pi 4 Model B | Recommended — 2GB+ RAM |
| Pi 5 | Overkill but compatible |

### Required Sensors

| Sensor | Purpose | Interface | Address |
|--------|---------|-----------|---------|
| INA226 x2 | Battery voltage/current | I2C | 0x40 (battery) |
| INA226 xN | Propulsion unit current | I2C | 0x41-0x47 |
| MPU6050 | IMU / vibration | I2C | 0x68 |
| (Optional) BME280 | Ambient temp/pressure | I2C | 0x76 |

### Wiring Diagram

```
Raspberry Pi                  INA226 (0x40 — Battery)
┌──────────┐                  ┌──────────┐
│ Pin 1    │──── 3.3V ────────│ VCC      │
│ (3.3V)   │                  │          │
│ Pin 3    │──── SDA1 ────────│ SDA      │
│ (GPIO 2) │                  │          │
│ Pin 5    │──── SCL1 ────────│ SCL      │
│ (GPIO 3) │                  │          │
│ Pin 6    │──── GND ─────────│ GND      │
│ (GND)    │                  │          │
│          │                  │ A0=GND   │
│          │                  │ A1=GND   │
└──────────┘                  └──────────┘

INA226 (0x41 — Propulsion 1)     MPU6050 (0x68 — IMU)
┌──────────┐                     ┌──────────┐
│ VCC      │──── 3.3V ──────────│ VCC      │
│ SDA      │──── SDA1 ──────────│ SDA      │
│ SCL      │──── SCL1 ──────────│ SCL      │
│ GND      │──── GND ───────────│ GND      │
│ A0=VCC   │                     │ AD0=GND  │
│ A1=GND   │                     └──────────┘
└──────────┘
```

### I2C Address Configuration

| Sensor | A1 | A0 | Address |
|--------|----|----|---------|
| Battery INA226 | GND | GND | 0x40 |
| Propulsion 1 INA226 | GND | VCC | 0x41 |
| Propulsion 2 INA226 | VCC | GND | 0x42 |
| Propulsion 3 INA226 | VCC | VCC | 0x43 |
| Propulsion 4 INA226 | GND | SDA | 0x44 |
| MPU6050 IMU | GND | (AD0=GND) | 0x68 |

### Shunt Resistor Selection

| Location | Current Range | Shunt Value | Max Drop (@max I) |
|----------|--------------|-------------|-------------------|
| Battery rail | 0-80A | 1.0 mΩ | 80 mV (safe) |
| Motor per-ESC | 0-30A | 10 mΩ | 300 mV (exceeds ±81.92mV! use 2 mΩ) |

**Critical:** INA226 shunt voltage range is ±81.92 mV. Verify: `I_max × R_shunt < 0.08192V`. For 30A motor current, use 2.0 mΩ (60 mV) or lower.

## Installation

### Fresh Install from Scratch

```bash
# Clone repository
git clone <repository-url> /home/pi/drone_monitor
cd /home/pi/drone_monitor

# Run installer
./install.sh
```

The installer:
1. Updates system packages
2. Enables I2C interface
3. Creates data directories
4. Creates Python virtual environment
5. Installs Python dependencies
6. Registers systemd service
7. Verifies I2C bus

### After Installation

```bash
# Reboot to enable I2C
sudo reboot

# Check service status
sudo systemctl status uav-phm

# View live logs
journalctl -u uav-phm -f

# Stop/start/restart
sudo systemctl stop uav-phm
sudo systemctl start uav-phm
sudo systemctl restart uav-phm
```

## Calibration Procedure

### 1. Verify I2C Detection

```bash
i2cdetect -y 1
```

Expected output:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 41 -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- 68 -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

### 2. Configure hardware.json

Edit `config/hardware.json` to match your actual sensor addresses:

```json
{
    "i2c_bus": 1,
    "sensors": {
        "battery_monitor": {
            "type": "INA226",
            "address": "0x40",
            "shunt": 0.001
        },
        "propulsion_units": [
            {
                "id": 1,
                "current_sensor": {
                    "type": "INA226",
                    "address": "0x41",
                    "shunt": 0.01
                },
                "esc_temperature": null
            }
        ],
        "imu": {
            "type": "MPU6050",
            "address": "0x68"
        }
    }
}
```

### 3. Baseline Calibration

```bash
# Run in hardware mode to collect baseline data
sudo venv/bin/python monitor.py --hardware
```

Let the system run for 30-60 seconds at idle, then at several throttle points. The baseline is saved to `config/baseline.json`.

## Service Management

### systemd Commands

```bash
# View status
sudo systemctl status uav-phm

# Start on boot (enabled by default)
sudo systemctl enable uav-phm

# Disable auto-start
sudo systemctl disable uav-phm

# View logs
journalctl -u uav-phm -f --since "5 minutes ago"

# Reset service after config change
sudo systemctl restart uav-phm
```

### Manual Run

```bash
# Hardware mode (requires sensors)
cd /home/pi/drone_monitor
venv/bin/python monitor.py --hardware

# Simulation mode (no hardware needed)
venv/bin/python monitor.py --simulate

# Custom config
venv/bin/python monitor.py --hardware --config config/fixed-wing.yaml

# With fault injection (simulate only)
venv/bin/python monitor.py --simulate --fault "battery-aging,prop-damage"
```

## Startup Banner

When the system starts successfully, you will see:

```
================================

UAV PHM SYSTEM ONLINE

Mode:
HARDWARE

Vehicle:
Quad

Propulsion Units:
4 (front_left, front_right, rear_left, rear_right)

Battery Monitor:
ONLINE

Current Sensors:
4 connected

IMU:
ONLINE

Health Engine:
READY

Features:
  Battery:        ON
  Current Sense:  ON
  Vibration:      ON
  Temperature:    OFF

Dashboard:
http://<host>:5000

Flight ID:
001

================================
```

## Dashboard

Access the web dashboard at:

```
http://<raspberry-pi-ip>:5000
```

### API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | Dashboard HTML |
| GET | `/api/latest` | Current sensor readings + status |
| GET | `/api/system` | Pi health (CPU, RAM, disk, uptime) |
| GET | `/api/health` | Health scores |
| GET | `/api/diagnostics` | Active diagnoses |
| GET | `/api/prediction` | Predicted failures |
| GET | `/api/telemetry` | Telemetry history |
| GET | `/api/faults` | Active faults |
| GET | `/api/fault_events` | Fault event timeline |
| POST | `/api/arm` | Arm system |
| POST | `/api/disarm` | Disarm system |
| POST | `/api/throttle` | Set throttle (simulate only) |

## Data Storage

```
data/
├── flights/              # Per-boot flight databases
│   ├── flight_001.db     # Telemetry, faults, health
│   ├── flight_002.db
│   └── ...
├── blackbox/             # Fault event recordings (30s pre + 30s post)
│   ├── fault_0000_battery_over_current_1700000000.json
│   └── ...
└── baselines/            # Calibration profiles
    └── last_state.json   # Last shutdown state
```

Each boot creates a new flight database with an incrementing ID. Existing flight logs are never overwritten.

## System Architecture

### Thread Model

| Thread | Priority | Rate | Tasks |
|--------|----------|------|-------|
| **fast_loop** | Highest | 40 Hz | Sensor reads, safety thresholds, blackbox ring buffer |
| **health_loop** | Medium | 1 Hz | Feature extraction, diagnostics, health engine, DB |
| **web_server** | Low | on-demand | Flask dashboard, serves /api/* |

### Graceful Degradation

When a sensor fails:

1. First failure → log warning, retry
2. 3 consecutive failures → mark sensor as "degraded"
3. Disable diagnostics that depend on the failed sensor
4. System continues with reduced capabilities

Example: IMU lost:
- Disabled: vibration analysis
- Kept: battery monitoring, current monitoring

### Safe Shutdown

On shutdown (CTRL+C, systemctl stop, power loss risk):
1. Flush SQLite database
2. Save current health state to `data/baselines/last_state.json`
3. Close I2C bus handles

## Troubleshooting

### I2C Not Working

```bash
# Check I2C is enabled
ls /dev/i2c*
# Expected: /dev/i2c-1

# Check kernel module
lsmod | grep i2c

# Enable manually
sudo raspi-config  → Interface Options → I2C → Enable

# Check permissions
sudo usermod -a -G i2c pi
# Log out and back in
```

### Permission Denied on I2C

```bash
# Add pi user to i2c group
sudo usermod -a -G i2c $USER
# Reboot or re-login
```

### Service Fails to Start

```bash
# Check logs
journalctl -u uav-phm -n 50 --no-pager

# Common issues:
# 1. Working directory wrong — edit deploy/uav-phm.service
# 2. I2C permissions — ensure pi in i2c group
# 3. Missing dependencies — re-run install.sh

# Run manually to see errors
cd /home/pi/drone_monitor
sudo venv/bin/python monitor.py --hardware
```

### No Sensors Detected

```bash
# Verify I2C bus
i2cdetect -y 1

# Check wiring
# - All sensors share SDA, SCL, 3.3V, GND
# - Each sensor has unique address (A0/A1 pins)

# Check hardware.json matches actual addresses
cat config/hardware.json
```

### Database Too Large

Flight databases are stored per-session in `data/flights/`. To free space:
```bash
# List flight databases with sizes
ls -lh data/flights/

# Archive or delete old flights
rm data/flights/flight_001.db
```

## Development / Testing

```bash
# Run with mock sensors (no hardware)
python3 monitor.py --simulate

# Run with specific fault injection
python3 monitor.py --simulate --fault "prop-loss,battery-aging"

# Run tests
python3 -m pytest tests/
```

## License

Proprietary — UAV PHM Monitor
