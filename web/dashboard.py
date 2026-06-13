from flask import Flask, jsonify, render_template, request


def create_app(monitor):
    app = Flask(__name__)

    @app.route("/")
    def dashboard():
        return render_template("dashboard.html")

    @app.route("/api/latest")
    def api_latest():
        return jsonify(monitor.status_snapshot())

    @app.route("/api/health")
    def api_health():
        snap = monitor.status_snapshot()
        units = snap.get("propulsion_units", [])
        return jsonify({
            "overall": snap.get("overall_health", 100),
            "battery": snap.get("battery", {}).get("health", 100),
            "propulsion_units": [{
                "id": u["id"], "name": u["name"],
                "score": u["health"],
                "current": u["current"],
                "warnings": u.get("health_warnings", []),
            } for u in units],
            "vehicle_type": snap.get("vehicle_type", "quad"),
            "n_units": snap.get("n_units", 0),
        })

    @app.route("/api/diagnostics")
    def api_diagnostics():
        return jsonify(monitor.get_active_diagnoses())

    @app.route("/api/prediction")
    def api_prediction():
        return jsonify(monitor.get_prediction())

    @app.route("/api/controls")
    def api_controls():
        snap = monitor.status_snapshot()
        return jsonify({
            "armed": snap.get("armed", False),
            "throttle": snap.get("throttle", 0),
            "prop_on": snap.get("prop_on", True),
            "friction_level": snap.get("friction_level", 0),
        })

    @app.route("/api/telemetry")
    def api_telemetry():
        return jsonify(monitor.get_telemetry(limit=300))

    @app.route("/api/health_history")
    def api_health_history():
        return jsonify(monitor.get_health(limit=100))

    @app.route("/api/faults")
    def api_faults():
        return jsonify(monitor.get_active_faults())

    @app.route("/api/fault_history")
    def api_fault_history():
        return jsonify(monitor.get_fault_history(limit=50))

    @app.route("/api/fault_events")
    def api_fault_events():
        return jsonify(monitor.get_fault_events(limit=50))

    @app.route("/api/arm", methods=["POST"])
    def api_arm():
        monitor.arm()
        return jsonify({"armed": True})

    @app.route("/api/disarm", methods=["POST"])
    def api_disarm():
        monitor.disarm()
        return jsonify({"armed": False})

    @app.route("/api/throttle", methods=["POST"])
    def api_throttle():
        data = request.get_json(silent=True) or {}
        val = float(data.get("throttle", 0.0))
        monitor.set_throttle(val)
        return jsonify({"throttle": monitor.status_snapshot().get("throttle", 0)})

    @app.route("/api/prop", methods=["POST"])
    def api_prop():
        data = request.get_json(silent=True) or {}
        monitor.set_prop(bool(data.get("on", True)))
        return jsonify({"prop_on": monitor.status_snapshot().get("prop_on", True)})

    @app.route("/api/friction", methods=["POST"])
    def api_friction():
        data = request.get_json(silent=True) or {}
        val = float(data.get("level", 0.0))
        monitor.set_friction(val)
        return jsonify({"friction_level": monitor.status_snapshot().get("friction_level", 0)})

    return app
