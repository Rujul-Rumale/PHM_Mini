# UAV PHM Monitor — Raspberry Pi Deployment Guide

## Overview

Deploy the UAV Prognostics and Health Management (PHM) system on a Raspberry Pi for real-time physics-based fault detection, battery monitoring, and vibration analysis. The system boots headlessly, runs three parallel threads (fast sensor loop, health analysis loop, web dashboard), and stores per-flight telemetry databases.

## Hardware Requirements

### Supported Raspberry Pi Models

| Model | Notes |
|-------|-------|
| Pi Zero 2 W | Adequate for 1-2 propulsion units |
| Pi 3 Model B+ | Good for single motor setups |
| Pi 4 Model B | Recommended — 2GB+ RAM |
| Pi 5 | Overkill but compatible |

### Required Sensors

This architecture is currently optimized for a single-motor configuration:

| Sensor | Purpose | Interface | Address |
|--------|---------|-----------|---------|
| INA219 (or INA226) | Battery Monitor (Voltage/Current) | I2C | 0x40 |
| INA219 (or INA226) | ESC Monitor (Motor Current) | I2C | 0x41 |
| LM75 | ESC Thermal Probe | I2C | 0x48 |
| MPU6050 | Airframe Vibration IMU | I2C | 0x68 |

### Wiring Diagram

All sensors communicate over the standard Raspberry Pi I2C1 bus. They should be wired in parallel (daisy-chained).

```
Raspberry Pi                 Sensors (Parallel Bus)
┌──────────┐                 ┌────────────────────┐
│ Pin 1    │──── 3.3V ───────│ VCC (all sensors)  │
│ (3.3V)   │                 │                    │
│ Pin 3    │──── SDA1 ───────│ SDA (all sensors)  │
│ (GPIO 2) │                 │                    │
│ Pin 5    │──── SCL1 ───────│ SCL (all sensors)  │
│ (GPIO 3) │                 │                    │
│ Pin 6    │──── GND ────────│ GND (all sensors)  │
│ (GND)    │                 │                    │
└──────────┘                 └────────────────────┘
```

**Note:** If using 5V logic sensors, connect VCC to Pin 2 (5V), but ensure your Raspberry Pi is protected by a level shifter on the SDA/SCL lines if the sensor modules do not include them.

### I2C Address Configuration

You must solder the address jumpers on your breakout boards to match these specific addresses:

| Sensor | Jumper Configuration | Target Address |
|--------|----------------------|----------------|
| Battery INA219 | Default (No jumpers) | 0x40 |
| ESC INA219 | Bridge A0 | 0x41 |
| ESC LM75 | Default (A0, A1, A2 to GND) | 0x48 |
| MPU6050 IMU | Default (AD0 to GND) | 0x68 |

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

## Calibration Procedure (Required)

Because the system uses Physics-Based PHM indices, it must learn the baseline of your specific hardware.

### 1. Verify I2C Detection

```bash
i2cdetect -y 1
```

Expected output should show `40`, `41`, `48`, and `68`.

### 2. Configure Hardware Setup

Ensure `config/hardware.json` reflects your exact shunt resistor values for the INA219s. Do not hardcode these in Python.

### 3. Baseline Calibration via Web UI

1. Open the dashboard at `http://<pi-ip>:5000`
2. Mount a completely healthy, balanced propeller.
3. Navigate to the **Health** tab.
4. Click **▶ Start Calibration**.
5. Following the on-screen instructions, use the throttle slider to sweep from 0% to 100% in ~10% increments.
6. The system will collect data and save the fingerprint to `config/baseline.json`.

## Dashboard & Endpoints

Access the web dashboard at: `http://<raspberry-pi-ip>:5000`

### Key API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/v1/status` | Realtime raw telemetry and high-level states |
| GET | `/api/v1/phm` | Realtime PHM health indices and conditions |
| GET | `/api/v1/phm/history` | Cross-flight historical trend |
| POST | `/api/v1/calibrate` | Trigger the calibration routine |

## Data Storage

```
data/
├── flights/              # Per-boot flight SQLite databases
│   ├── flight_001.db     # Telemetry, faults, health
│   ├── flight_002.db
│   └── ...
├── blackbox/             # Fault event recordings
└── baselines/            # Calibration profiles
```

Each boot creates a new flight database with an incrementing ID. Existing flight logs are never overwritten.

## Troubleshooting

### I2C Not Working

```bash
# Check I2C is enabled
ls /dev/i2c*
# Expected: /dev/i2c-1

# Enable manually
sudo raspi-config  → Interface Options → I2C → Enable

# Add pi user to i2c group
sudo usermod -a -G i2c pi
```

### No Sensors Detected

```bash
# Verify I2C bus
i2cdetect -y 1

# Check wiring
# - All sensors share SDA, SCL, 3.3V, GND
# - Each sensor has unique address (A0/A1 pins)
```
