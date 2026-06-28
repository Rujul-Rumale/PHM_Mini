# UAV Propulsion Health Monitor

A Physics-Based Prognostics and Health Management (PHM) system for UAV propulsion systems.

This system replaces simple hard-coded thresholds with a **baseline-driven, physics-based health engine**. It continuously compares real-time telemetry against a calibrated "healthy" propulsion baseline to detect mechanical, thermal, and electrical degradation before catastrophic failure occurs.

## Hardware Configuration

The current architecture is designed for a single-motor propulsion unit monitored by the following I2C sensors:

*   **1x INA219 (or INA226) Current/Voltage Monitor:**
    *   Battery & ESC Monitor (`0x40`): Measures total pack voltage and current (motor draws ~99% of total current).
*   **1x LM75 Temperature Sensor (`0x48`):** Measures ESC/heatsink temperature.
*   **1x MPU6050 IMU (`0x68`):** Mounted on the motor base to measure high-frequency vibration.

## Core Concepts: Physics-Based Health Indices

Instead of raising alerts when a sensor hits an arbitrary limit, the system computes four core Health Indices based on the *deviations* from the calibrated baseline at the current throttle level:

1.  **Propeller Balance Index (PBI):** Correlates excessive vibration (from the IMU) with electrical ripple (from the INA219) to detect mechanical imbalance, damaged propellers, or bent shafts.
2.  **Propulsion Load Index (PLI):** Compares actual current draw to expected current draw. Positive PLI indicates excess drag (bearing friction, obstruction); negative PLI indicates loss of load (missing or loose propeller).
3.  **ESC Thermal Index (ETI):** Evaluates temperature rise *relative* to the expected thermal load for the current power draw. Detects cooling path failures or internal resistance degradation before hard thermal limits are reached.
4.  **Battery Stress Index (BSI):** Correlates voltage sag with current demand. Identifies aging batteries, high internal resistance, or excessive load.

## Software Setup & Execution

```bash
# Install dependencies
pip3 install -r requirements.txt

# Verify I2C devices are visible
i2cdetect -y 1

# Run the primary monitor
python3 monitor.py
```

### Dashboard

The system runs a responsive, high-frequency web dashboard accessible at:
`http://<raspberry-pi-ip>:5000`

## Calibration Procedure (Crucial Step!)

Because the PHM engine relies on detecting *deviations*, it must first learn what "healthy" looks like.

1.  **Install a perfectly balanced, undamaged propeller.**
2.  Open the web dashboard and navigate to the **Health** tab.
3.  Click **▶ Start Calibration**.
4.  Follow the on-screen instructions: Use the throttle slider to slowly sweep from 0% up to 100%, pausing for about 5 seconds at each 10% interval.
5.  The system automatically collects 3 baseline sweeps, computes the statistical normal bounds, and saves them to `config/baseline.json`.
6.  Once calibrated, the system will begin emitting active health scores and fault predictions.

## Project Structure

*   `core/`: The core processing pipeline.
    *   `baseline_manager.py`: Manages the throttle-indexed baseline and interpolates expected values.
    *   `deviation_engine.py`: Computes normalized deviations of real-time telemetry vs. the baseline.
    *   `health_engine.py`: Computes the 4 core health indices (PBI, PLI, ETI, BSI).
    *   `diagnostics.py`: Classifies abstract conditions (e.g., `rotational_imbalance`, `thermal_anomaly`) based on index severity, complete with a debounce filter.
*   `web/`: The Flask web server and dashboard UI.
*   `calibrate.py`: Operator-driven calibration routine.
*   `monitor.py`: The main entry point that wires all components together.

## Extension & Development

*   To run the test suite: `pytest tests/ -v`
*   Shunt resistor configuration should *not* be hardcoded; it is dynamically loaded from `hardware.json`.
*   Cross-flight health trends are logged to local SQLite databases per flight (`flight_health_indices` table).
