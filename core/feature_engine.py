import time
import logging
from typing import Optional

from core.telemetry_frame import TelemetryFrame
from core.feature_extractor import FeatureExtractor
from core.database_service import HealthUpdated

log = logging.getLogger(__name__)


class FeatureEngine:
    """Listens to raw SensorFrameReady events and computes sliding-window statistical features at 5 Hz."""
    def __init__(self, status_mgr, event_bus, n_units: int = 1):
        self.status_mgr = status_mgr
        self.event_bus = event_bus
        self.n_units = n_units
        self.extractor = FeatureExtractor(n_units=n_units)
        
        self._last_calc_time = 0.0
        self._calc_interval = 0.20  # 5 Hz health rate (200 ms)

        # Register event subscriber
        from core.monitor_service import SensorFrameReady
        self.event_bus.subscribe(SensorFrameReady, self._on_sensor_frame)

    def _on_sensor_frame(self, event: SensorFrameReady):
        frame = event.frame
        now = frame.timestamp
        
        # Check calculation rate throttle (5 Hz)
        if now - self._last_calc_time < self._calc_interval:
            return
            
        self._last_calc_time = now
        self._process_features(frame)

    def _process_features(self, frame: TelemetryFrame):
        # 1. Compute Battery Features
        # Ignore STALE/OFFLINE/ERROR battery quality
        if frame.battery_sensor_quality == "ONLINE":
            batt_feats = self.extractor.electrical_battery(
                voltage=frame.battery_voltage,
                current=frame.battery_current,
                power=frame.battery_power
            )
            # Update status manager
            self.status_mgr.update_state("battery.soc", frame.soc or 100.0)
        else:
            batt_feats = None

        # 2. Compute Propulsion Features (single-motor rail)
        prop_feats = []
        if str(frame.esc_current_quality).startswith("ONLINE"):
            prop_feats = self.extractor.propulsion_currents([frame.esc_current])
            
        # 3. Compute Thermal Features
        # ESC Temperature can be read from standalone LM75 or fallback simulated
        esc_t_rise = 0.0
        if frame.esc_temp_quality == "ONLINE":
            therm_feats = self.extractor.thermal([frame.esc_temp], ambient=25.0)
            if therm_feats and therm_feats.temp_rise:
                esc_t_rise = therm_feats.temp_rise[0]
        else:
            therm_feats = None

        # 4. Compute Vibration Features (vibration mapped to single propulsion unit)
        vib_feats = None
        if frame.imu_quality == "ONLINE" and frame.imu_rms > 0:
            # We map airframe vibration samples directly to propulsion unit ID 1
            # Wait, the extractor takes raw accel_samples. We can pass the frame's rms directly,
            # or feed mock raw accel samples if we want, but since frame has imu_rms:
            # let's look at FeatureExtractor.vibration which takes list[float] accel_samples.
            # In monitor_service we high-pass filtered and computed RMS directly on Z.
            # We can mock a list of samples for FeatureExtractor.vibration or call it directly.
            # Wait, FeatureExtractor.vibration computes RMS itself. To bypass double calculation,
            # we can construct the VibrationFeatures manually or pass Z buffer:
            # Since monitor_service maintains a Z buffer, we can check if it is accessible.
            # Actually, monitor_service built frame.imu_rms and frame.imu_kurtosis.
            # We can populate VibrationFeatures directly:
            from core.feature_extractor import VibrationFeatures
            vib_feats = VibrationFeatures(
                unit_id=1,
                rms=frame.imu_rms,
                kurtosis=frame.imu_kurtosis,
                peak_freq=frame.imu_peak_freq,
                spectral_energy=frame.imu_rms ** 2
            )

        # 5. Populate TelemetryFrame derived features dictionary
        features = {
            "battery_features": batt_feats.to_dict() if batt_feats else {},
            "propulsion_features": [f.to_dict() for f in prop_feats] if prop_feats else [],
            "thermal_features": therm_feats.to_dict() if therm_feats else {},
            "vibration_features": vib_feats.to_dict() if vib_feats else {},
        }
        frame.features = features
        
        # Publish HealthUpdated frame to alert DatabaseService and DiagnosticsEngine
        self.event_bus.publish(HealthUpdated(frame))
        self.status_mgr.ping_watchdog("feature_engine")
