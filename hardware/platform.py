import os
import sys
import time
import logging

log = logging.getLogger(__name__)

_PLATFORM_CACHE = None


def detect_platform() -> str:
    global _PLATFORM_CACHE
    if _PLATFORM_CACHE:
        return _PLATFORM_CACHE
    if sys.platform == "linux":
        try:
            with open("/proc/cpuinfo") as f:
                data = f.read()
            if "Raspberry Pi" in data or "BCM" in data:
                _PLATFORM_CACHE = "raspberry-pi"
                return _PLATFORM_CACHE
        except OSError:
            pass
        _PLATFORM_CACHE = "linux"
    elif sys.platform == "darwin":
        _PLATFORM_CACHE = "darwin"
    elif sys.platform == "win32":
        _PLATFORM_CACHE = "win32"
    else:
        _PLATFORM_CACHE = "unknown"
    return _PLATFORM_CACHE


def is_raspberry_pi() -> bool:
    return detect_platform() == "raspberry-pi"


def get_raspberry_pi_version() -> str:
    if not is_raspberry_pi():
        return "N/A"
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("Model"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "Raspberry Pi (unknown)"


def get_cpu_temperature() -> float:
    if not is_raspberry_pi():
        return 0.0
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            raw = f.read().strip()
            return float(raw) / 1000.0
    except (OSError, ValueError):
        log.warning("Could not read CPU temperature")
        return 0.0


def get_ram_usage() -> dict:
    try:
        import psutil
        mem = psutil.virtual_memory()
        return {"total_gb": round(mem.total / (1024**3), 2), "used_gb": round(mem.used / (1024**3), 2), "percent": mem.percent}
    except ImportError:
        pass
    if is_raspberry_pi():
        try:
            with open("/proc/meminfo") as f:
                data = f.read()
            total_kb = 0
            avail_kb = 0
            for line in data.splitlines():
                if line.startswith("MemTotal:"):
                    total_kb = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    avail_kb = int(line.split()[1])
            if total_kb:
                used_kb = total_kb - avail_kb
                return {"total_gb": round(total_kb / (1024**2), 2), "used_gb": round(used_kb / (1024**2), 2), "percent": round(used_kb / total_kb * 100, 1)}
        except OSError:
            pass
    return {"total_gb": 0, "used_gb": 0, "percent": 0}


def get_disk_usage() -> dict:
    try:
        import psutil
        du = psutil.disk_usage("/")
        return {"total_gb": round(du.total / (1024**3), 1), "used_gb": round(du.used / (1024**3), 1), "percent": du.percent}
    except ImportError:
        pass
    try:
        st = os.statvfs("/")
        total = st.f_frsize * st.f_blocks
        free = st.f_frsize * st.f_bfree
        used = total - free
        return {"total_gb": round(total / (1024**3), 1), "used_gb": round(used / (1024**3), 1), "percent": round(used / total * 100, 1) if total else 0}
    except (OSError, AttributeError):
        return {"total_gb": 0, "used_gb": 0, "percent": 0}


def get_uptime() -> float:
    if is_raspberry_pi():
        try:
            with open("/proc/uptime") as f:
                return float(f.read().split()[0])
        except (OSError, ValueError):
            pass
    return time.time() - time.monotonic()
