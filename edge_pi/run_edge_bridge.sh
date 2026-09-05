#!/usr/bin/env bash
# ==============================================================================
# LUNABOT RASPBERRY PI 4B EDGE GATEWAY LAUNCHER
# Runs on the Raspberry Pi 4B to bridge Gazebo telemetry & run edge ML.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================================================="
echo " 🌕 LUNABOT: STARTING RASPBERRY PI 4B EDGE GATEWAY & ONBOARD COMPUTER"
echo "========================================================================="

# Check for ROS 2 Humble or fallback to High-Performance Standalone Edge Agent
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    echo "   Mode:            Full ROS 2 DDS Multi-Machine Bridge"
    echo "   ROS Domain ID:   $ROS_DOMAIN_ID"
    echo "   Local Hostname:  $(hostname)"
    echo "   IP Address:      $(hostname -I 2>/dev/null | awk '{print $1}')"
    echo "========================================================================="
    exec python3 "$SCRIPT_DIR/edge_bridge_node.py" "$@"
else
    echo "   Mode:            High-Performance Standalone Edge Agent (Zero-ROS Required)"
    echo "   Target Laptop:   http://10.42.0.1:8080"
    echo "   Local Hostname:  $(hostname)"
    echo "   IP Address:      $(hostname -I 2>/dev/null | awk '{print $1}')"
    echo "========================================================================="
    exec python3 "$SCRIPT_DIR/edge_agent.py" "$@"
fi
