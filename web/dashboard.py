import io
import csv
import json
import logging
import threading
from flask import Flask, jsonify, render_template, request, make_response, redirect, send_file

log = logging.getLogger("web")



def create_app(monitor):
    app = Flask(__name__)

    # --- UI Templates ---
    @app.route("/")
    def dashboard():
        return render_template("dashboard.html")

    # --- V1 Versioned REST API ---

    @app.route("/api/v1/status")
    def api_status_v1():
        return jsonify(monitor.status_snapshot())

    @app.route("/api/v1/system")
    def api_system_v1():
        return jsonify(monitor.get_system_info())

    @app.route("/api/v1/health")
    def api_health_v1():
        snap = monitor.status_snapshot()
        units = snap.get("propulsion_units", [])
        return jsonify({
            "overall": snap.get("overall_health", 100),
            "battery": snap.get("battery", {}).get("health", 100),
            "propulsion_units": [{
                "id": u["id"], "name": u["name"],
                "score": u["health"],
                "current": u["current"],
                "warnings": u.get("warnings", []),
            } for u in units],
            "vehicle_type": snap.get("vehicle_type", "fixed-wing"),
            "n_units": len(units),
        })

    @app.route("/api/v1/faults")
    def api_faults_v1():
        return jsonify(monitor.get_active_faults())

    @app.route("/api/v1/fault_history")
    def api_fault_history_v1():
        return jsonify(monitor.get_fault_history(limit=50))

    @app.route("/api/v1/fault_events")
    def api_fault_events_v1():
        return jsonify(monitor.get_fault_events(limit=50))

    @app.route("/api/v1/prediction")
    def api_prediction_v1():
        return jsonify(monitor.get_prediction())

    @app.route("/api/v1/sensors")
    def api_sensors_v1():
        snap = monitor.status_snapshot()
        return jsonify(snap.get("sensor_status", {}))

    @app.route("/api/v1/capability")
    def api_capability_v1():
        snap = monitor.status_snapshot()
        return jsonify(snap.get("capabilities", {}))

    @app.route("/api/v1/config")
    def api_config_v1():
        snap = monitor.status_snapshot()
        return jsonify(snap.get("config", {}))

    @app.route("/api/v1/history")
    def api_history_v1():
        limit = int(request.args.get("limit", 300))
        return jsonify(monitor.get_telemetry(limit=limit))

    # --- PHM Health Indices ---

    @app.route("/api/v1/phm")
    def api_phm_v1():
        """Current PHM health indices, deviations, active conditions, and maintenance advisory."""
        snap = monitor.status_snapshot()
        phm = snap.get("phm", {})
        calibration_info = {}
        if hasattr(monitor, "baseline_mgr"):
            calibration_info = monitor.baseline_mgr.calibration_info()
        return jsonify({
            "indices": phm.get("indices", {}),
            "deviation": phm.get("deviation", {}),
            "conditions": phm.get("conditions", []),
            "maintenance": phm.get("maintenance", "No maintenance required."),
            "calibrated": phm.get("calibrated", False),
            "confidence": phm.get("confidence", 0.0),
            "calibration_info": calibration_info,
        })

    @app.route("/api/v1/phm/history")
    def api_phm_history_v1():
        """Cross-flight PHM index history from flight_health_indices table."""
        limit = int(request.args.get("limit", 50))
        if monitor.db_service:
            return jsonify(monitor.db_service.get_flight_index_history(limit=limit))
        return jsonify([])

    @app.route("/api/v1/demo")
    def api_demo_guide_v1():
        """Return physical demo scenario guide."""
        try:
            with open("config/demo_profiles.json", encoding="utf-8") as fh:
                return jsonify(json.load(fh))
        except FileNotFoundError:
            return jsonify({"error": "demo_profiles.json not found"}), 404

    # --- Calibration ---

    _cal_thread: dict = {"thread": None}

    @app.route("/api/v1/calibrate", methods=["POST"])
    def api_calibrate_start_v1():
        """Trigger a calibration sweep in a background thread."""
        from calibrate import get_recorder
        recorder = get_recorder()
        progress = recorder.get_progress()
        if progress["state"] in ("settling", "sampling", "computing"):
            return jsonify({"status": "already_running", "progress": progress}), 409

        def _run():
            try:
                recorder.run_sweep(monitor)
                # Reload baseline manager after calibration
                if hasattr(monitor, "baseline_mgr"):
                    monitor.baseline_mgr._load()
            except Exception as exc:
                log.error("Calibration sweep error: %s", exc)

        t = threading.Thread(target=_run, name="calibration-sweep", daemon=True)
        _cal_thread["thread"] = t
        t.start()
        monitor.status_mgr.log_event("CAL: Calibration sweep started via web UI.")
        return jsonify({"status": "started"})

    @app.route("/api/v1/calibrate", methods=["GET"])
    def api_calibrate_status_v1():
        """Return current calibration progress."""
        from calibrate import get_recorder
        recorder = get_recorder()
        progress = recorder.get_progress()
        calibration_info = {}
        if hasattr(monitor, "baseline_mgr"):
            calibration_info = monitor.baseline_mgr.calibration_info()
        return jsonify({"progress": progress, "calibration_info": calibration_info})

    @app.route("/api/v1/calibrate/cancel", methods=["POST"])
    def api_calibrate_cancel_v1():
        """Cancel an in-progress calibration sweep."""
        from calibrate import get_recorder
        get_recorder().cancel()
        return jsonify({"status": "cancel_requested"})

    @app.route("/api/v1/flights")
    def api_flights_v1():
        if monitor.db_service:
            return jsonify(monitor.db_service.get_recent_flights())
        return jsonify([])

    @app.route("/api/v1/export")
    def api_export_v1():
        fmt = request.args.get("format", "csv")
        if fmt == "sqlite":
            if monitor.db_service and monitor.db_service.db_path:
                return send_file(monitor.db_service.db_path, as_attachment=True)
            return jsonify({"error": "No database path configured"}), 404
            
        elif fmt == "json":
            # Return all sessions + recent telemetry as JSON
            history = monitor.get_telemetry(limit=1000)
            return jsonify(history)
            
        else:
            # Default CSV
            output = io.StringIO()
            writer = csv.writer(output)
            history = monitor.get_telemetry(limit=1000)
            if history:
                writer.writerow(history[0].keys())
                for row in history:
                    writer.writerow(row.values())
            response = make_response(output.getvalue())
            response.headers["Content-Disposition"] = "attachment; filename=telemetry_export.csv"
            response.headers["Content-type"] = "text/csv"
            return response

    # --- Actions / Commands ---

    @app.route("/api/v1/arm", methods=["POST"])
    def api_arm_v1():
        success, reason = monitor.control_service.arm()
        return jsonify({"accepted": success, "reason": reason})

    @app.route("/api/v1/disarm", methods=["POST"])
    def api_disarm_v1():
        success, reason = monitor.control_service.disarm()
        return jsonify({"accepted": success, "reason": reason})

    @app.route("/api/v1/throttle", methods=["POST"])
    def api_throttle_v1():
        data = request.get_json(silent=True) or {}
        val = float(data.get("throttle", 0.0))
        success, reason = monitor.control_service.set_throttle(val)
        return jsonify({"accepted": success, "reason": reason})

    @app.route("/api/v1/reset_fault", methods=["POST"])
    def api_reset_fault_v1():
        success, reason = monitor.control_service.reset_fault()
        return jsonify({"accepted": success, "reason": reason})

    @app.route("/api/v1/heartbeat", methods=["POST"])
    def api_heartbeat_v1():
        monitor.control_service.heartbeat()
        return jsonify({"status": "heartbeat acknowledged"})

    @app.route("/api/v1/emergency_stop", methods=["POST"])
    def api_emergency_stop_v1():
        monitor.control_service.emergency_stop()
        return jsonify({"status": "emergency stop executed"})

    @app.route("/api/v1/config", methods=["POST"])
    def api_update_config_v1():
        # Live config parameter update
        data = request.get_json(silent=True) or {}
        key = data.get("key")
        val = data.get("value")
        if key:
            monitor.status_mgr.update_state(f"config.{key}", val)
            monitor.status_mgr.log_event(f"CONFIG: live update for '{key}' set to {val}")
            return jsonify({"status": "config updated"})
        return jsonify({"error": "Missing key or value"}), 400

    @app.route("/api/v1/fault/inject", methods=["POST"])
    def api_inject_fault_v1():
        data = request.get_json(silent=True) or {}
        fault_name = data.get("fault")
        if fault_name:
            monitor._apply_fault_types_to_mock([fault_name])
            monitor.status_mgr.log_event(f"SIM: injected fault '{fault_name}'")
            return jsonify({"status": f"fault '{fault_name}' injected"})
        return jsonify({"error": "Missing fault name"}), 400

    # --- Legacy Redirect Rules (301 Moved Permanently) ---

    @app.route("/api/latest")
    def api_latest_legacy():
        return redirect("/api/v1/status", code=301)

    @app.route("/api/system")
    def api_system_legacy():
        return redirect("/api/v1/system", code=301)

    @app.route("/api/health")
    def api_health_legacy():
        return redirect("/api/v1/health", code=301)

    @app.route("/api/diagnostics")
    def api_diagnostics_legacy():
        return redirect("/api/v1/prediction", code=301)

    @app.route("/api/prediction")
    def api_prediction_legacy():
        return redirect("/api/v1/prediction", code=301)

    @app.route("/api/faults")
    def api_faults_legacy():
        return redirect("/api/v1/faults", code=301)

    @app.route("/api/fault_events")
    def api_fault_events_legacy():
        return redirect("/api/v1/fault_events", code=301)

    @app.route("/api/arm", methods=["POST"])
    def api_arm_legacy():
        success, _ = monitor.control_service.arm()
        return jsonify({"armed": success})

    @app.route("/api/disarm", methods=["POST"])
    def api_disarm_legacy():
        success, _ = monitor.control_service.disarm()
        return jsonify({"armed": not success})

    @app.route("/api/throttle", methods=["POST"])
    def api_throttle_legacy():
        data = request.get_json(silent=True) or {}
        val = float(data.get("throttle", 0.0))
        monitor.control_service.set_throttle(val)
        return jsonify({"throttle": val})

    return app
