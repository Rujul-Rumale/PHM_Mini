#!/usr/bin/env bash
set -e

# ──────────────────────────────────────────────────
# UAV PHM Monitor — Raspberry Pi Installation Script
# ──────────────────────────────────────────────────

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_NAME="drone_monitor"
VENV_DIR="$PROJECT_DIR/venv"
SERVICE_NAME="uav-phm"

echo "========================================"
echo " UAV PHM Monitor — Pi Installation"
echo "========================================"

# ── 1. System packages ────────────────────────
echo "[1/7] Updating system packages..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    python3 \
    python3-pip \
    python3-venv \
    i2c-tools \
    git \
    || true

# ── 2. Enable I2C ─────────────────────────────
echo "[2/7] Enabling I2C interface..."
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt > /dev/null
    echo "  I2C enabled in /boot/config.txt (reboot to activate)"
else
    echo "  I2C already enabled"
fi

if ! lsmod | grep -q "^i2c_dev"; then
    sudo modprobe i2c_dev || true
fi
if ! lsmod | grep -q "^i2c_bcm"; then
    sudo modprobe i2c_bcm2835 || true
fi

# Add pi user to i2c group
sudo usermod -a -G i2c "$USER" 2>/dev/null || true

# ── 3. Create directories ─────────────────────
echo "[3/7] Creating data directories..."
mkdir -p "$PROJECT_DIR/data/flights"
mkdir -p "$PROJECT_DIR/data/blackbox"
mkdir -p "$PROJECT_DIR/data/baselines"
mkdir -p "$PROJECT_DIR/logs"

# ── 4. Python virtual environment ─────────────
echo "[4/7] Creating Python virtual environment..."
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
    echo "  Virtual environment created at $VENV_DIR"
else
    echo "  Virtual environment already exists"
fi

# ── 5. Install Python dependencies ────────────
echo "[5/7] Installing Python dependencies..."
"$VENV_DIR/bin/pip" install --upgrade pip -qq
"$VENV_DIR/bin/pip" install -r "$PROJECT_DIR/requirements.txt" -qq
echo "  Dependencies installed"

# ── 6. Register systemd service ───────────────
echo "[6/7] Registering systemd service..."
SERVICE_SRC="$PROJECT_DIR/deploy/$SERVICE_NAME.service"
SERVICE_DST="/etc/systemd/system/$SERVICE_NAME.service"

sudo cp "$SERVICE_SRC" "$SERVICE_DST"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
echo "  Service $SERVICE_NAME registered and enabled"

# ── 7. Verify I2C ─────────────────────────────
echo "[7/7] Verifying I2C..."
if command -v i2cdetect &> /dev/null; then
    echo "  I2C bus scan:"
    sudo i2cdetect -y 1 || echo "  (run 'sudo i2cdetect -y 1' manually after reboot)"
else
    echo "  i2cdetect not found — install i2c-tools"
fi

echo ""
echo "========================================"
echo " Installation complete!"
echo ""
echo " Next steps:"
echo "   1. Reboot:  sudo reboot"
echo "   2. Check service:  sudo systemctl status $SERVICE_NAME"
echo "   3. View logs:  journalctl -u $SERVICE_NAME -f"
echo "   4. Dashboard:  http://<raspberry-pi-ip>:5000"
echo ""
echo " To test without service:"
echo "   cd $PROJECT_DIR"
echo "   $VENV_DIR/bin/python monitor.py --hardware"
echo ""
echo " For simulation (no hardware):"
echo "   $VENV_DIR/bin/python monitor.py --simulate"
echo "========================================"
