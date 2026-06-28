#!/bin/bash
# setup_ubuntu.sh - Auto Setup Script for UAV PHM Monitor on Ubuntu (Raspberry Pi)

echo "==============================================="
echo " UAV PHM Monitor - Ubuntu Setup Script "
echo "==============================================="

# 1. Update and install packages
echo "[1/4] Installing system packages (python3, i2c-tools)..."
sudo apt update
sudo apt install -y python3-pip python3-venv i2c-tools libffi-dev

# 2. Enable I2C in config.txt
echo "[2/4] Enabling I2C in /boot/firmware/config.txt..."
CONFIG_FILE="/boot/firmware/config.txt"
if [ -f "$CONFIG_FILE" ]; then
    if grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
        echo "I2C is already enabled in $CONFIG_FILE."
    else
        echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE" > /dev/null
        echo "I2C enabled in $CONFIG_FILE."
    fi
else
    echo "WARNING: $CONFIG_FILE not found. If this is not an Ubuntu Raspberry Pi, please enable I2C manually."
fi

# 3. Add user to i2c group
echo "[3/4] Adding user $USER to the i2c group..."
if groups $USER | grep &>/dev/null '\bi2c\b'; then
    echo "User $USER is already in the i2c group."
else
    sudo usermod -aG i2c $USER
    echo "Added to i2c group."
fi

# 4. Setup Python Virtual Environment
echo "[4/4] Setting up Python virtual environment..."
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

echo "==============================================="
echo "Setup Complete!"
echo "Please REBOOT your Raspberry Pi to apply the I2C group and boot config changes."
echo "Command: sudo reboot"
echo "==============================================="
