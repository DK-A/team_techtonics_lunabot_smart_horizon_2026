#!/usr/bin/env bash
# ==============================================================================
# LUNABOT RASPBERRY PI 4B AUTOMATED EDGE ENVIRONMENT SETUP
# Run this script on your Raspberry Pi 4 Model B (Ubuntu 22.04 LTS / Pi OS 64-bit)
# ==============================================================================

set -e

echo "========================================================================="
echo " 🌕 LUNABOT: CONFIGURING RASPBERRY PI 4B AS ROVER ONBOARD COMPUTER (OBC)"
echo "========================================================================="

# 1. Update and install core dependencies
echo "[1/4] Updating package index & essential tools..."
sudo apt-get update
sudo apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    python3-dev \
    build-essential \
    net-tools \
    iproute2

# 2. Install ROS 2 Humble (if not already installed)
if ! command -v ros2 >/dev/null 2>&1; then
    echo "[2/4] Installing ROS 2 Humble on ARM64..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs
else
    echo "[2/4] ROS 2 Humble already detected on this system. Skipping."
fi

# 3. Install lightweight Python ML libraries for ARM
echo "[3/4] Installing Python ML & Telemetry dependencies..."
pip3 install --no-cache-dir \
    numpy \
    scikit-learn \
    psutil

# 4. Copy trained ML models into edge directory if not present
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$SCRIPT_DIR/models"

if [ -f "$SCRIPT_DIR/../ml_models/isolation_forest_lunar_gas.pkl" ]; then
    cp "$SCRIPT_DIR/../ml_models/isolation_forest_lunar_gas.pkl" "$SCRIPT_DIR/models/"
    cp "$SCRIPT_DIR/../ml_models/terramechanics_slip_classifier.pkl" "$SCRIPT_DIR/models/"
    echo "[4/4] Copied trained .pkl machine learning models to $SCRIPT_DIR/models/"
else
    echo "[4/4] Please transfer isolation_forest_lunar_gas.pkl and terramechanics_slip_classifier.pkl into $SCRIPT_DIR/models/"
fi

# Make run script executable
chmod +x "$SCRIPT_DIR/run_edge_bridge.sh" 2>/dev/null || true
chmod +x "$SCRIPT_DIR/edge_bridge_node.py" 2>/dev/null || true

echo "========================================================================="
echo " ✅ RASPBERRY PI 4B SETUP COMPLETE!"
echo " Launch the Edge Bridge anytime by running: ./run_edge_bridge.sh"
echo "========================================================================="
