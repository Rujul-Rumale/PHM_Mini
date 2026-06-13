# Drone Fault Monitor — Deep Diagnostic Audit Report

---

## 1. PROJECT OVERVIEW

### Intended Purpose
A real-time battery and motor fault detection / prognostic health management (PHM) system for drones (quadcopter or fixed-wing) running on a Raspberry Pi with INA226 I2C current/voltage sensors. It provides telemetry logging, threshold-based fault detection, SoC/SoH estimation, a Flask dashboard, and optional ML-based anomaly detection.

### Intended Tech Stack
- **Language:** Python 3.11
- **Hardware Interface:** I2C via `smbus2` → INA226 sensors
- **Backend Web Server:** Flask (embedded, dev-grade)
- **Database:** SQLite3 (via `sqlite3`)
- **ML/Analytics:** scikit-learn (IsolationForest, RandomForest), numpy, scipy
- **Optional Physics Model:** PyThrust (`setuav-pythrust`) for expected-current estimation
- **Configuration:** YAML (`PyYAML`)
- **Frontend:** Single-page HTML+JS with Chart.js (CDN-loaded), CSS custom properties

### Entry Point
- `monitor.py` (line 359: `if __name__ == "__main__": main()`)
- `calibrate.py` is a standalone companion for sensor calibration (run separately)

### Current Run/Build State
- **All Python files compile without syntax errors.**
- **Monitor runs successfully in simulation mode** — starts up, creates DB, serves Flask dashboard at `http://0.0.0.0:5050`, prints telemetry summaries every 5s.
- **Calibrate script runs** but requires interactive stdin (fails with `EOFError` if piped).
- **On Windows:** `smbus2` fails to import (requires `fcntl`, Unix-only), but this is expected — the platform target is RPi (Linux).
- **Anomaly detector** cannot be trained on this system — requires `failure_simulator.py` which does not exist.
- **PyThrust** (`setuav-pythrust`) is installed and loads successfully, but the propeller database path (`data/propellers/apc_202602`) does not exist, so it falls back to a simplified physics model.

---

## 2. ARCHITECTURE & STRUCTURE

### Folder/Module Structure (as-is)
```
drone_monitor/
├── monitor.py              # Main entry point, DroneMonitor class, main loop
├── calibrate.py            # Calibration session state machine, sweep mode
├── requirements.txt        # 4 dependencies
├── README.md               # Docs
├── config/
│   ├── thresholds.yaml     # Quadcopter config (4 motors)
│   └── fixed-wing.yaml     # Single-motor config
├── core/
│   ├── __init__.py
│   ├── fault_detector.py   # Threshold-based fault detection (battery + motors)
│   ├── soc_estimator.py    # Coulomb-counting SoC with OCV initialization
│   ├── feature_extractor.py # Rolling-window statistical features
│   ├── health_manager.py   # Component health degradation model (0-100)
│   ├── battery_health.py   # SoH via internal resistance estimation
│   ├── anomaly_detector.py # Isolation Forest + Random Forest (unused at runtime)
│   └── logger.py           # SQLite telemetry + fault logging
├── sensors/
│   ├── __init__.py
│   └── ina226.py           # INA226 driver + MockINA226 simulator
├── web/
│   ├── __init__.py
│   └── dashboard.py        # Flask app, HTML template, API endpoints
└── logs/
    ├── monitor.log         # Live log (3MB generated in ~2hrs)
    ├── telemetry.db        # SQLite DB (16MB)
    └── telemetry_fw.db     # Fixed-wing DB (28KB)
```

### Architectural Pattern
- **Intended:** Modular monolith with a polling main loop, hardware abstraction layer (INA226/MockINA226), and a web layer on top
- **Pattern observed:** Pipeline architecture — sensor reads → feature extraction → fault detection → SoH estimation → health manager → DB logging → web serving — all in a single-threaded loop with a daemon Flask thread

### Structural Deviations & Anti-patterns
1. **Dead module:** `core/anomaly_detector.py` is never imported or used by `monitor.py` or any other module. It is a disconnected orphan.
2. **Missing dependency:** `anomaly_detector.py:179` imports `from core.failure_simulator import FailureSimulator`, but **`core/failure_simulator.py` does not exist**. The train path is broken.
3. **Strange directory name:** `{core,sensors,web,config,logs}` is a literal directory name (braces in the name), likely a shell glob that was mistakenly created instead of expanding. Contents unknown but likely empty or a duplicate of the actual directories.
4. **Duplicate OCV table:** The OCV→SoC lookup table is defined twice — in `core/soc_estimator.py:15-21` and again in `sensors/ina226.py:79-85`. This is a code duplication violation (DRY).
5. **Inverted dependency:** `sensors/ina226.py` contains `_soc_to_ocv()` (lines 88-100) — a pure calculation function that belongs in `soc_estimator.py`, not in a sensor driver.
6. **Circular dependency risk:** `calibrate.py` imports `from sensors.ina226 import INA226, MockINA226` (line 37), while `monitor.py` also imports from `sensors.ina226`. No actual cycle exists currently, but the architecture is flat enough that careless additions could create one.
7. **Hardcoded assumptions in MockINA226:** `sensors/ina226.py:191` hardcodes `cell_ocv * 4` (4S LiPo) — the mock battery is not configurable for different cell counts.
8. **Mixed naming conventions:** Some files use snake_case (`fault_detector.py`, `soc_estimator.py`), some use hyphenated in YAML paths (`fixed-wing.yaml`). Python module names use underscores consistently. Functions use snake_case. Some class attributes mix underscore prefixes with public access.

---

## 3. BREAKAGE DIAGNOSIS

### Known Errors and Failure Modes

| # | File | Line | Error | Type | Cause |
|---|------|------|-------|------|-------|
| 1 | `core/anomaly_detector.py` | 179 | `ModuleNotFoundError: No module named 'core.failure_simulator'` | **Broken import** | `failure_simulator.py` does not exist. The `--train` code path is dead. |
| 2 | `core/anomaly_detector.py` | 33-34 | Models directory and pickle files do not exist | **Missing runtime assets** | `models/anomaly_model.pkl` and `models/anomaly_scaler.pkl` are never generated. The `predict()` method (line 120) silently returns `None`. |
| 3 | `core/fault_detector.py` | 105 | `PropellerDatabase.load("data/propellers/apc_202602")` | **Missing data file** | The PyThrust propeller database path does not exist. Falls back to simplified model, but this is silent degradation. |
| 4 | `sensors/ina226.py` | (N/A if real hardware) | Hardware I2C bus not available | **Platform incompatibility** | `smbus2` requires `fcntl` (Unix-only). Fails on Windows. Expected behavior for RPi target, but any testing on non-Linux is impossible. |
| 5 | `monitor.py` | 353 | Web dashboard may fail to start | **Silent failure** | Any exception in `create_app()` or Flask startup is caught and logged as a warning; the monitor runs headless. User may not notice. |
| 6 | `calibrate.py` | 325 | `EOFError: EOF when reading a line` | **Interactive-only** | Calibration requires stdin input; fails when run non-interactively. |
| 7 | `calibrate.py` | 306 | Same `smbus2` issue on non-Linux | **Platform incompatibility** | Same as #4. |
| 8 | `web/dashboard.py` | 201 | Chart.js loaded from CDN | **External dependency risk** | Hardcoded `https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js` — dashboard breaks without internet. |
| 9 | `calibrate.py` | 400 | `cfg['battery']['warn_voltage']` may set key that doesn't exist | **Potential KeyError** | `apply_calibration_to_cfg` writes keys `warn_voltage`, `min_voltage` that already exist in the YAML. This works but is reassignment, not creation. However, if calibration produces different keys than expected, silent failure. |

### Missing Files
- `core/failure_simulator.py` — referenced by `anomaly_detector.py:179` for training data generation
- `models/anomaly_model.pkl` — expected by `anomaly_detector.py:107`
- `models/anomaly_scaler.pkl` — expected by `anomaly_detector.py:109`
- `data/propellers/apc_202602` — expected by `fault_detector.py:105`
- `config/calibration.json` — generated by `calibrate.py`, consumed by `apply_calibration_to_cfg()`; absent until calibration is run

### Broken Imports
- `core.failure_simulator` (line 179 of `anomaly_detector.py`) — **BROKEN**, file does not exist

### Unresolved References
- `STATUS` in `logger.py` — the telemetry table has a `status TEXT DEFAULT 'OK'` column; `monitor.py:248` passes `status` (a string from `self._detector.highest_severity().value`). This works correctly.
- `monitor._lock` in `dashboard.py:404,409` — accesses private attribute `_lock` of `DroneMonitor`; this is a design smell (cross-module access to private members) but not a runtime error.

### Environment/Config Issues
- `.env` file: None exists. Port is read from `os.environ.get("PHM_PORT", "5050")`.
- No hardcoded secrets found.
- **READ ME:** `os.makedirs("logs", exist_ok=True)` in `monitor.py:317` — uses relative path; must be run from the `drone_monitor/` directory.
- `calibrate.py:41` uses `CALIB_PATH = "config/calibration.json"` — also relative.
- Log directory and DB paths are relative (`logs/monitor.log`, `logs/telemetry.db`, `logs/telemetry_fw.db`). Running from a different CWD will break these.

---

## 4. CODE QUALITY & TECH DEBT

### High Complexity / Poor Readability
- **`sensors/ina226.py:186-204` (`read_voltage()`):** Combines battery voltage simulation, fault injection, and motor voltage in a single method with multiple branching paths. Moderate complexity.
- **`core/fault_detector.py:171-254` (`check_motors()`):** 84 lines with nested conditionals, three code paths (load curve, PyThrust, fixed threshold), and fault injection logic. High cyclomatic complexity.
- **`web/dashboard.py`:** The HTML/CSS/JS template is a single Python raw string (392 lines) embedded inline — not maintainable, no editor support, no minification.

### Duplicated Logic
1. **OCV➝SoC lookup table:** Defined in `core/soc_estimator.py:15-21` and duplicated in `sensors/ina226.py:79-85`.
2. **Linear interpolation logic:** `calibrate.py:97-108` (`CalibrationResult.from_file`) duplicates JSON deserialization patterns. The `ocv_to_soc()` function and `_soc_to_ocv()` function are inverses but independently implemented.
3. **`_linear_slope()` in `feature_extractor.py:54-63`** is a manual least-squares implementation. scipy is already a dependency (in `requirements.txt`), so `scipy.stats.linregress` could replace it.

### Commented-Out / Abandoned Code
- `sensors/ina226.py:117` — `fault_inject (str)` is marked as "legacy" in the docstring. The battery aging fault injection uses `fault_mode`, while `fault_inject` is used by `read_current()` (line 176-179) and `read_voltage()` (line 202-203). Two parallel fault injection systems.
- `core/health_manager.py` — extensive "[Inference]" comments (lines 14, 78, 222, etc.) suggesting many heuristics are placeholders that need calibration against real data.
- `core/battery_health.py` — lines 9-16 contain lengthy "[Inference]" disclaimers about R_int estimation accuracy being a simplified DC method.

### Inconsistent Patterns / Naming
- **Private attribute naming:** Some modules use `self._attr` convention, others expose properties. `dashboard.py` directly accesses `monitor._lock`, `monitor._armed`, `monitor._throttle`, `monitor._prop_on`, `monitor._friction_level`, `monitor._db` — all private attributes of `DroneMonitor`.
- **Fault code naming:** Some use underscores (`UNDERVOLTAGE_CRITICAL`), some use mixed case (`PROP_FAILURE`).
- **Config key naming:** `soc_warn_percent`, `soc_critical_percent` in YAML but `min_voltage`, `warn_voltage`. Inconsistent prefix vs. suffix for thresholds.

### Code Quality Ratings Per Module

| Module | File | Rating | Rationale |
|--------|------|--------|-----------|
| Main loop | `monitor.py` | **Good** | Clean, well-structured, reasonable error handling |
| Calibration | `calibrate.py` | **Good** | Well-organized state machine, clear flow |
| Fault detector | `core/fault_detector.py` | **Acceptable** | Complex but well-documented; could use refactoring |
| Soc estimator | `core/soc_estimator.py` | **Good** | Simple, focused, clean |
| Feature extractor | `core/feature_extractor.py` | **Good** | Clear separation of concerns |
| Health manager | `core/health_manager.py` | **Acceptable** | Clear intent but many "[Inference]" heuristics are uncalibrated |
| Battery health | `core/battery_health.py` | **Acceptable** | Same as health_manager — known heuristics |
| Anomaly detector | `core/anomaly_detector.py` | **Poor** | Dead code, broken import, unreachable |
| Logger | `core/logger.py` | **Good** | Clean, simple, well-structured |
| INA226 driver | `sensors/ina226.py` | **Acceptable** | Mock class is well-done; duplicated OCV table is a code smell |
| Dashboard | `web/dashboard.py` | **Poor** | 392-line inline HTML template is unmaintainable; no JS/CSS build step |
| Config | `config/thresholds.yaml` | **Good** | Well-commented, clear structure |

---

## 5. FEATURES & FUNCTIONALITY GAPS

### Intended but Incomplete Features

| Feature | Status | Evidence |
|---------|--------|----------|
| ML Anomaly Detection | **Broken** | `anomaly_detector.py` exists but is dead code. Training requires `failure_simulator.py` (missing). No model files present. `monitor.py` never calls it. |
| Calibration API | **Built, untested** | `calibrate.py` documents POST/GET endpoints for calibration via API (lines 17-20), but `web/dashboard.py` has no calibration routes. The API endpoints are unimplemented in the web layer. |
| Vibration/IMU Integration | **Stub** | `feature_extractor.py` has `inject_vibration()` (line 160) and `apply_pending_vibration()` (line 196) with FFT support via numpy, but no IMU driver exists and the method is never called by `monitor.py`. |
| PyThrust Physics Model | **Partial** | Code is complete and loads, but propeller database path (`data/propellers/apc_202602`) is missing. Falls back silently to a simplified `I0 + (Imax-I0)*T^1.5` model. |
| Load-curve Sweep | **Built** | `calibrate.py:431` (`run_sweep()`) works in simulation, requires GPIO 18 + ESC on real hardware. Interactive input required. |

### Features That Work vs. Not Built
- **Working:** Threshold-based fault detection, SoC estimation, SoH estimation, health scoring, telemetry DB logging, Flask dashboard with live charts, arm/disarm/throttle/prop/friction controls, MockINA226 simulation, dual-bus I2C detection, calibration state machine
- **Not Built:** Calibration REST API, MAVLink integration, GPIO fault output, ESC telemetry parser, IMU/vibration sensor driver, real failure data for anomaly model training

### TODOs / FIXMEs / Stubs

| File | Lines | Type | Content |
|------|-------|------|---------|
| `core/feature_extractor.py` | 160-208 | Stub | `inject_vibration()` — implemented but never called; FFT peak frequency "requires scipy.fft or numpy.fft" (numpy available, but IMU hardware not present) |
| `core/health_manager.py` | 14, 78, 222, 233 | Warning | Multiple `[Inference]` tags — heuristics are placeholders |
| `core/battery_health.py` | 9-16, 136, 152 | Warning | Multiple `[Inference]` tags — R_int and SoH methods are simplified |
| `sensors/ina226.py` | 117 | Legacy | `fault_inject` marked as legacy |
| `README.md` | 111-115 | Roadmap | Extension points (GPIO, MAVLink, sensors, ESC telemetry) listed but not implemented |

### Estimated Feature Completion: ~65%

- Core sensor reading, fault detection, SoC/SoH, DB logging — **working (100%)**
- Web dashboard — **working but crude (80%)** — inline template, no tests
- Calibration — **working in CLI mode (90%)** — API endpoints not wired to web
- Anomaly detection — **broken (10%)** — code present but unreachable
- PyThrust integration — **partial (60%)** — missing database files
- Vibration/IMU — **stub (15%)** — code present, no driver, not integrated
- Real hardware I2C — **untested** — no RPi available in audit environment

---

## 6. DEPENDENCIES & TOOLING

### Declared Dependencies (`requirements.txt`)
```
smbus2>=0.4.3
PyYAML>=6.0
Flask>=3.0.0
scipy>=1.12.0
```

### Undeclared Dependencies (used in code but not in requirements.txt)
| Package | Usage | File | Line |
|---------|-------|------|------|
| `numpy` | Feature vectors, FFT, anomaly detection | `feature_extractor.py:180`, `anomaly_detector.py:127,167`, `calibrate.py:467` | Required by scipy but should be explicit |
| `scikit-learn` | IsolationForest, RandomForest, StandardScaler | `anomaly_detector.py:168` | Not in requirements.txt |
| `setuav-pythrust` | Physics-based propulsion solver | `fault_detector.py:20-23` | Listed in YAML comments only |
| `gpiozero` | PWM output for ESC in sweep mode | `calibrate.py:469` | Not in requirements.txt |
| `smbus2` | I2C communication | `monitor.py:131`, `calibrate.py:305` | In requirements.txt ✓ |

### Missing / Potentially Missing Dependencies
- `scikit-learn` — required by `anomaly_detector.py` for training and prediction
- `setuav-pythrust` — optional physics solver, documented but not in requirements.txt
- `gpiozero` — optional (real hardware only), used in `calibrate.py:run_sweep()`
- `numpy` — technically brought in by scipy, but should be explicit

### Outdated / Vulnerable Packages
- No version pinning beyond minimums — `smbus2>=0.4.3`, `Flask>=3.0.0`. This could pull breaking changes.
- Flask 3.x+ is modern; no CVEs determinable from version constraints alone.

### Build Tooling
- **No build tooling present.** No `setup.py`, `pyproject.toml`, `Makefile`, or task runner.
- The project is meant to be run directly with `python3 monitor.py`.
- No test framework, no linting config, no type checking.

### Conflicting Versions
- None detected. All imports resolve correctly on Python 3.11.

---

## 7. DATA FLOW & INTEGRATION POINTS

### Data Flow Diagram
```
[INA226 Sensors] ──I2C──> [_read_sensors()]
                              │
                              ▼
                    ┌─────────────────────┐
                    │  _read_sensors()    │
                    │  batt_v, i, p      │
                    │  motor_currents[]   │
                    └─────────┬───────────┘
                              │
          ┌───────────────────┼───────────────────────┐
          ▼                   ▼                       ▼
  ┌──────────────┐   ┌──────────────┐   ┌──────────────────┐
  │   SoC Estim. │   │  FaultDetect │   │  FeatureExtract  │
  │ soc = f(V,I) │   │ batt_faults  │   │ motor_feats[]    │
  │              │   │ motor_faults │   │ batt_feat        │
  └──────┬───────┘   └──────┬───────┘   └────────┬─────────┘
         │                  │                     │
         │                  ▼                     │
         │          ┌──────────────┐              │
         │          │  TelemetryDB │              │
         │          │ log_fault()  │              │
         │          └──────┬───────┘              │
         │                                          │
         │                  ┌───────────────────────┘
         │                  ▼
         │          ┌──────────────────┐
         │          │  BatterySoH      │
         │          │  R_int, soh, sag │
         │          └──────┬───────────┘
         │                  │
         ▼                  ▼
   ┌─────────────────────────────┐
   │      HealthManager          │
   │  apply_fault_events()       │
   │  apply_motor_features()     │
   │  apply_battery_features()   │
   │  → phm dict per component   │
   └──────────────┬──────────────┘
                  │
                  ▼
   ┌────────────────────────────┐
   │  DroneMonitor.latest dict │  ← mutex-protected
   │  Polled by dashboard.py   │
   └────────────────────────────┘
                  │
                  ▼
   ┌────────────────────────────┐
   │  Flask Web Server          │
   │  /api/latest → JSON        │
   │  /api/telemetry → JSON     │
   │  / → DASHBOARD_HTML (SPA)  │
   └────────────────────────────┘
```

### Broken / Incomplete Integration Points
1. **Anomaly Detector** — never called from `monitor.py` main loop. The `AnomalyDetector` class exists, `predict()` returns `AnomalyResult`, but `monitor.run()` never invokes it.
2. **Calibration API** — `calibrate.py` documents REST endpoints for web-driven calibration, but `web/dashboard.py` has no calibration routes.
3. **Vibration Pipeline** — `FeatureExtractor.inject_vibration()` / `apply_pending_vibration()` are never called. No IMU driver exists.
4. **PyThrust Database** — `fault_detector.py:105` loads `data/propellers/apc_202602` — path does not exist in the project.

### External Services Referenced but Not Connected
- **MAVLink** (README line 113) — mentioned as extension point, no code
- **GPIO fault output** (README line 112) — mentioned, no code
- **ESC telemetry** (README line 114) — mentioned, no code

---

## 8. WHAT IS ACTUALLY WORKING

### Functional and Stable Components

1. **Main Loop (`monitor.py`):**
   - Starts, initializes all components, polls at configurable rate (10Hz default)
   - Simulation mode works perfectly — `MockINA226` generates realistic data
   - Shutdown via SIGINT/SIGTERM works
   - Thread-safe telemetry snapshot via `self._lock`

2. **Sensor Simulation (`sensors/ina226.py:MockINA226`):**
   - Deterministic throttle-based current model (`I0 + (Imax-I0)*T^1.5`)
   - Realistic OCV-based battery voltage with configurable R_int sag
   - Fault injection: prop loss (no-load), friction (extra load), battery aging (R_int ramps)
   - Manual controls: prop_attached, friction_level, throttle

3. **Fault Detection (`core/fault_detector.py`):**
   - Battery: undervoltage (warn/crit), overcurrent (warn/crit), voltage sag rate, low SoC (warn/crit)
   - Motor: overcurrent (warn/crit), open circuit, current imbalance, sensor missing, prop failure, load low/high
   - Persistence debounce (`fault_persistence_count` consecutive violations)
   - Active fault tracking with clear-on-normal logic
   - Highest-severity aggregation

4. **SoC Estimation (`core/soc_estimator.py`):**
   - OCV initialization at startup
   - Coulomb counting during operation
   - Soft OCV correction when current < 0.5A

5. **SoH Estimation (`core/battery_health.py`):**
   - R_int estimation via ΔV/ΔI on load transitions
   - Linear SoH mapping (R_new→100%, R_eol→0%)
   - Voltage sag tracking, cycle count estimate, failure probability heuristic

6. **Feature Extraction (`core/feature_extractor.py`):**
   - Rolling-window (50 samples @10Hz = 5s) variance, trend, spike count
   - Per-motor imbalance ratio, efficiency index
   - Battery voltage variance, trend, ripple amplitude, load voltage drop

7. **Health Manager (`core/health_manager.py`):**
   - Per-component health scoring (battery + per-motor)
   - Stress penalty system with per-fault-type weights
   - Passive aging drift, SoH anchoring for battery
   - Degradation rate, RUL estimation, failure probability, status labels

8. **Telemetry Logging (`core/logger.py`):**
   - SQLite schema creation (telemetry + faults tables)
   - Time-series telemetry logging at 2Hz
   - Fault event logging with clear tracking
   - Recent telemetry/fault queries for dashboard

9. **INA226 Hardware Driver (`sensors/ina226.py:INA226`):**
   - Register-level I2C communication
   - Calibration based on shunt resistance and max current
   - Signed register reads for bidirectional current
   - Dual-bus I2C support for address conflicts

10. **Flask Dashboard (`web/dashboard.py`):**
    - Serves DASHBOARD_HTML at `/` — dark theme, responsive grid
    - Real-time telemetry via `/api/latest` polled every 500ms
    - Chart.js voltage and current charts (300-point rolling window)
    - Interactive controls: arm/disarm, throttle slider, prop toggle, friction slider
    - PHM health gauges with color-coded bars and status badges
    - Active fault list with severity, source, code, message, value/threshold

11. **Calibration Script (`calibrate.py`):**
    - State machine: IDLE → BATTERY_REST → MOTOR_LOAD → COMPLETE
    - Battery resting voltage/no-load current sampling (5s @10Hz)
    - Motor load current sampling (2s warmup + 5s @10Hz)
    - Calibration save/load (config/calibration.json)
    - Load-curve sweep mode (0→100% in 5% steps with safety abort)
    - `apply_calibration_to_cfg()` — merges calibration into runtime config

12. **Config System (YAML):**
    - Dual configs: quadcopter (`thresholds.yaml`) and fixed-wing (`fixed-wing.yaml`)
    - Calibration override mechanism via `_motor_calib` injected key
    - PyThrust physics section (optional)

---

## 9. RAW FINDINGS LOG

### Surprises & Anomalies

1. **`{core,sensors,web,config,logs}` directory (568 bytes):** A literal directory named with curly braces sits alongside the actual module directories. This appears to be a shell glob expansion error — someone ran `mkdir {core,sensors,web,config,logs}` in a shell that didn't expand the braces. The directory contains a copy of all the actual subdirectories' contents (or is empty — permissions prevented reading). This should be deleted.

2. **Dashboard port mismatch:** README says port 5000, but `monitor.py:345` defaults to `5050`. The `os.environ.get("PHM_PORT", "5050")` defaults to 5050. The README is wrong (probably from Flask's default 5000 before being changed to 5050).

3. **`requirements.txt` is missing critical dependencies for full functionality:** `scikit-learn` (anomaly detection), `numpy` (FFT, feature vectors), `gpiozero` (ESC PWM).

4. **The PyThrust package** (`setuav-pythrust`) is installed in the environment but is **not listed anywhere** in `requirements.txt` or as a pip-installable dependency. It's mentioned only in YAML comments.

5. **Two independent telemetry DBs:** `telemetry.db` (16MB, from quad config runs) and `telemetry_fw.db` (28KB, from fixed-wing config) are both present in logs/. The 16MB DB suggests substantial testing.

6. **All `.pyc` files are present** in `__pycache__` directories, confirming the code has been executed at least once successfully.

7. **The `anomaly_detector.py` file is dated 2026-06-02** while most files are 2026-05-30 to 2026-06-09. It was added later but never integrated.

8. **`gpiozero` import is not guarded** in `calibrate.py:469` — the `except ImportError` on line 472 only exists. But `pwm = None` is set in the simulate branch (line 466), and the `if not simulate and pwm:` guard at line 540-542 prevents crash. This is correct but fragile — any future change that removes the `pwm = None` default would cause a `NameError`.

9. **Hardcoded 4S assumption** in `sensors/ina226.py:191` (`v_pack = cell_ocv * 4`). The MockINA226 does not read `cell_count` from config. This means the battery simulation always assumes 4S regardless of what the config says. The `DroneMonitor` passes `batt_imax` and `base_voltage=bc['nominal_voltage']` (which is 16.8V for 4S) so this works by coincidence.

10. **`monitor.py:324`** — `from calibrate import apply_calibration_to_cfg` is imported inside `main()` rather than at module top. This is intentional (lazy import to avoid circular issues at module load time) but is an unusual pattern.

11. **`core/__init__.py`** — The file is empty (`0 bytes`), so there's no package-level exports or documentation.

12. **`web/dashboard.py`** — The huge `DASHBOARD_HTML` string uses `r"""..."""` (raw triple-quoted string) which means the `\n`, `\t`, etc. inside the HTML/CSS/JS are literal backslash-n sequences, not newlines. This is actually incorrect — the raw string prevents `\n` from being interpreted as newline, so the HTML template is one long line. This works (browsers don't care) but makes the raw source unreadable. Any sequence like `\d` in JS regex or `\u` in CSS will also be affected.

---

## STATE OF THE CODEBASE SUMMARY

The Drone Fault Monitor codebase is **functional in simulation mode** but carries significant structural debt. The core pipeline (sensor reading → fault detection → SoC/SoH → logging → dashboard) is solid, well-organized, and demonstrably working. However, the codebase has three distinct problem zones: **(1) Dead/orphaned modules** — the anomaly detector is built but completely disconnected, with a broken import to a non-existent file; **(2) Feature-completeness gaps** — calibration API endpoints, IMU vibration pipeline, and PyThrust's propeller database are all stubbed but non-functional; and **(3) Maintainability issues** — an embedded 392-line HTML template, duplicated OCV table across two modules, platform-specific dependencies leaking into simulation paths, a shell-glob-broken directory, and a README documenting the wrong port number. The project appears to be an early-to-mid stage prototype that has been exercised in simulation but never validated on real hardware. About 65% of the intended feature set is working, 15% is partially built but broken, and 20% is unimplemented. The code is moderately clean for a prototype but would require significant refactoring before production deployment.
