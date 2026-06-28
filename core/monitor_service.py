import time
import math
import logging
import threading
from collections import deque
from typing import Optional

from core.telemetry_frame import TelemetryFrame
from core.signal_filter import MovingAverageFilter, LowPassFilter, HighPassFilter
from core.throttle import ThrottleController

log = logging.getLogger(__name__)


# Event dataclass definitions published on EventBus
class SensorFrameReady:
    def __init__(self, frame: TelemetryFrame):
        self.frame = frame


class MonitorService(threading.Thread):
    """Deterministic sensor reading and filtering thread (running at 50 Hz)."""
    def __init__(self, status_mgr, event_bus, sensor_mgr, throttle_ctrl: ThrottleController):
        super().__init__(name="monitor-service", daemon=True)
        self.status_mgr = status_mgr
        self.event_bus = event_bus
        self.sensor_mgr = sensor_mgr
        self.throttle_ctrl = throttle_ctrl
        self._running = False
        
        # Load sample rates from configuration
        self._freqs = {
            "battery": 50,
            "propulsion": 50,
            "imu": 50,
            "temperature": 5
        }
        self._last_sample_times = {
            "battery": 0.0,
            "propulsion": 0.0,
            "imu": 0.0,
            "temperature": 0.0
        }
        
        # Signal Filters
        self._batt_i_filter = MovingAverageFilter(window_size=5)
        self._esc_temp_filter = LowPassFilter(tau=10.0)
        self._imu_z_filter = HighPassFilter(tau=0.5)
        
        # Rolling acceleration buffer (avoids blocking 100ms reads)
        self._imu_z_buffer = deque(maxlen=100)
        self._imu_x_buffer = deque(maxlen=100)
        self._imu_y_buffer = deque(maxlen=100)
        
        # Loop Performance Metrics
        self._loop_starts = deque(maxlen=50)
        self._missed_cycles = 0
        
        # Latest cached readings
        self._latest_batt_v = 0.0
        self._latest_batt_i = 0.0
        self._latest_batt_p = 0.0
        self._latest_esc_i = 0.0
        self._latest_esc_t = 25.0
        self._latest_imu_accel = {"x": 0.0, "y": 0.0, "z": 1.0}

    def start(self):
        self._running = True
        # Read rates from status config if available
        cfg = self.status_mgr.get_state().get("config", {})
        sys_cfg = cfg.get("system", {})
        freqs = sys_cfg.get("frequencies")
        if freqs:
            self._freqs.update(freqs)
            
        super().start()

    def stop(self):
        self._running = False

    def run(self):
        log.info("MonitorService thread started.")
        interval = 1.0 / max(self._freqs.values())
        
        # Initial scan results
        status = self.sensor_mgr.get_status()
        self._update_sensor_quality_from_status(status)
        self._detect_capabilities(status)
        
        self.status_mgr.transition_to("INITIALIZING", reason="Startup check")
        
        # Determine simulation mode vs hardware
        is_simulate = status.get("mode") == "simulate"
        
        # Grace period for initialization
        time.sleep(0.5)
        self.status_mgr.transition_to("READY", reason="Initialization complete")
        self.status_mgr.log_event(f"UAV Doctor Online. Mode: {self.status_mgr.get_state()['controls']['flight_mode'].upper()}")
        
        last_t = time.time()
        
        while self._running:
            loop_start = time.time()
            self._loop_starts.append(loop_start)
            
            # Loop stats tracking
            if len(self._loop_starts) >= 2:
                dt_loop = loop_start - self._loop_starts[-2]
                if dt_loop > interval * 1.5:
                    self._missed_cycles += 1
                    self.status_mgr.update_state("loop_stats.missed_cycles", self._missed_cycles)
                
                avg_dt = (self._loop_starts[-1] - self._loop_starts[0]) / (len(self._loop_starts) - 1)
                freq = 1.0 / avg_dt if avg_dt > 0 else 0.0
                self.status_mgr.update_state("loop_stats.frequency", round(freq, 1))

            dt = loop_start - last_t
            last_t = loop_start
            dt = min(max(dt, 0.0), 1.0)
            
            # 1. Update simulation throttle/friction if simulate mode
            if is_simulate:
                state = self.status_mgr.get_state()
                thr = state["controls"]["throttle"]
                fric = state["controls"]["friction_level"]
                for s in self.sensor_mgr.get_current_sensors():
                    s.set_throttle(thr)
                    s.friction_level = fric
                batt_sensor = self.sensor_mgr.get_battery()
                if batt_sensor:
                    batt_sensor.set_throttle(thr)
            
            # 2. Query due sensors
            self._sample_sensors(loop_start, dt)
            
            # 3. Compile telemetry frame with single timestamp
            frame = TelemetryFrame(
                timestamp=loop_start,
                flight_id=self.status_mgr.get_state().get("system_health", {}).get("flight_id", 0),
                flight_state=self.status_mgr.get_state()["flight_state"],
                
                battery_voltage=self._latest_batt_v,
                battery_current=self._latest_batt_i,
                battery_power=self._latest_batt_p,
                battery_sensor_quality=self.status_mgr.get_state()["sensor_status"]["battery"],
                
                esc_current=self._latest_esc_i,
                esc_current_quality=self.status_mgr.get_state()["sensor_status"]["propulsion_current"],
                esc_temp=self._latest_esc_t,
                esc_temp_quality=self.status_mgr.get_state()["sensor_status"]["esc_temperature"],
                
                imu_accel=self._latest_imu_accel,
                imu_quality=self.status_mgr.get_state()["sensor_status"]["imu"],
                
                throttle_pct=self.throttle_ctrl.provider.get_throttle(),
                throttle_source=self.throttle_ctrl.provider.source_type
            )
            
            # Apply signal filters to compiled frame
            frame.battery_current_filtered = self._batt_i_filter.update(frame.battery_current)
            frame.esc_temp_filtered = self._esc_temp_filter.update(frame.esc_temp, dt)
            
            # Calculate vibration stats from rolling accel buffers
            if len(self._imu_z_buffer) >= 4:
                samples_z = list(self._imu_z_buffer)
                mean_z = sum(samples_z) / len(samples_z)
                var_z = sum((v - mean_z) ** 2 for v in samples_z) / len(samples_z)
                
                rms_x = math.sqrt(sum(v*v for v in self._imu_x_buffer) / len(self._imu_x_buffer)) if self._imu_x_buffer else 0.0
                rms_y = math.sqrt(sum(v*v for v in self._imu_y_buffer) / len(self._imu_y_buffer)) if self._imu_y_buffer else 0.0
                rms_z = math.sqrt(sum(v*v for v in samples_z) / len(samples_z))
                
                frame.imu_rms = math.sqrt(rms_x**2 + rms_y**2 + rms_z**2)
                std_z = math.sqrt(var_z) if var_z > 0 else 1e-6
                frame.imu_kurtosis = (sum(((v - mean_z) / std_z) ** 4 for v in samples_z) / len(samples_z)) - 3.0
                
                # Perform basic peak frequency detection if numpy is available
                try:
                    import numpy as np
                    fft_vals = np.abs(np.fft.rfft(samples_z))
                    freqs = np.fft.rfftfreq(len(samples_z), 1.0 / self._freqs["imu"])
                    frame.imu_peak_freq = float(freqs[np.argmax(fft_vals[1:]) + 1]) # skip DC
                except Exception:
                    frame.imu_peak_freq = 0.0
            else:
                frame.imu_rms = 0.0
                frame.imu_kurtosis = 0.0
                frame.imu_peak_freq = 0.0

            # 4. Write telemetry state to StatusManager
            self.status_mgr.update_telemetry(frame)
            
            # 5. Update controls safety / limiting
            self.throttle_ctrl.update(frame.esc_current)
            
            # 6. Publish Event SensorFrameReady
            self.event_bus.publish(SensorFrameReady(frame))
            
            # 7. Ping Watchdog
            self.status_mgr.ping_watchdog("monitor")
            
            elapsed = time.time() - loop_start
            time.sleep(max(0.001, interval - elapsed))

    def _sample_sensors(self, now: float, dt: float):
        # Battery monitor read
        if now - self._last_sample_times["battery"] >= 1.0 / self._freqs["battery"] - 0.001:
            self._last_sample_times["battery"] = now
            batt = self.sensor_mgr.get_battery()
            if batt:
                try:
                    self._latest_batt_v = batt.read_voltage()
                    self._latest_batt_i = batt.read_current()
                    self._latest_batt_p = batt.read_power()
                    self.status_mgr.update_state("sensor_status.battery", "ONLINE")
                except Exception as e:
                    log.warning(f"Battery monitor read error: {e}")
                    self.status_mgr.update_state("sensor_status.battery", "ERROR")

        # Propulsion ESC current sensor read
        if now - self._last_sample_times["propulsion"] >= 1.0 / self._freqs["propulsion"] - 0.001:
            self._last_sample_times["propulsion"] = now
            currents = self.sensor_mgr.get_current_sensors()
            if currents:
                try:
                    self._latest_esc_i = currents[0].read_current()
                    self.status_mgr.update_state("sensor_status.propulsion_current", "ONLINE")
                except Exception as e:
                    log.warning(f"ESC current sensor read error: {e}")
                    self.status_mgr.update_state("sensor_status.propulsion_current", "ERROR")
            else:
                # SINGLE INA FALLBACK: Use battery current directly
                self._latest_esc_i = self._latest_batt_i
                self.status_mgr.update_state("sensor_status.propulsion_current", "ONLINE (BATT)")

        # Propulsion ESC temperature sensor read
        if now - self._last_sample_times["temperature"] >= 1.0 / self._freqs["temperature"] - 0.001:
            self._last_sample_times["temperature"] = now
            # Find in manager's custom list
            temp_sensors = getattr(self.sensor_mgr, "_temperature_sensors", [])
            if temp_sensors:
                try:
                    self._latest_esc_t = temp_sensors[0].read_temperature()
                    self.status_mgr.update_state("sensor_status.esc_temperature", "ONLINE")
                except Exception as e:
                    log.warning(f"ESC temperature sensor read error: {e}")
                    self.status_mgr.update_state("sensor_status.esc_temperature", "ERROR")
            elif len(currents) > 0 and hasattr(currents[0], "read_temperature"):
                # Fallback to simulated current sensor temp (in simulation mode)
                try:
                    self._latest_esc_t = currents[0].read_temperature()
                    self.status_mgr.update_state("sensor_status.esc_temperature", "ONLINE")
                except Exception:
                    self.status_mgr.update_state("sensor_status.esc_temperature", "ERROR")

        # IMU accelerometer read (non-blocking)
        if now - self._last_sample_times["imu"] >= 1.0 / self._freqs["imu"] - 0.001:
            self._last_sample_times["imu"] = now
            imu = self.sensor_mgr.get_imu()
            if imu:
                try:
                    accel = imu.read_accel()
                    self._latest_imu_accel = accel
                    
                    # Apply high pass filter to remove gravity Z bias before buffering
                    az_filt = self._imu_z_filter.update(accel["z"], dt)
                    self._imu_z_buffer.append(az_filt)
                    self._imu_x_buffer.append(accel["x"])
                    self._imu_y_buffer.append(accel["y"])
                    
                    self.status_mgr.update_state("sensor_status.imu", "ONLINE")
                except Exception as e:
                    log.warning(f"IMU accel read error: {e}")
                    self.status_mgr.update_state("sensor_status.imu", "ERROR")

    def _update_sensor_quality_from_status(self, status: dict):
        results = status.get("scan_results", {})
        found = results.get("found", {})
        missing = results.get("missing", {})
        
        for name in found:
            self.status_mgr.update_state(f"sensor_status.{name}", "ONLINE")
        for name in missing:
            self.status_mgr.update_state(f"sensor_status.{name}", "OFFLINE")

    def _detect_capabilities(self, status: dict):
        results = status.get("scan_results", {})
        found = results.get("found", {})
        
        has_batt = "battery_monitor" in found or status.get("battery")
        has_current = any("current" in k for k in found.keys()) or len(status.get("current_sensors", [])) > 0
        has_temp = any("temp" in k for k in found.keys()) or status.get("temperature") > 0
        has_imu = "imu" in found or status.get("imu")
        
        caps = {
            "battery_health": has_batt,
            "propulsion_current": has_current,
            "esc_thermal": has_temp,
            "vibration": has_imu,
            "motor_balance": False,  # single propulsion unit setup
            "efficiency_tracking": True,
            "flight_recorder": True,
        }
        self.status_mgr.update_state("capabilities", caps)
