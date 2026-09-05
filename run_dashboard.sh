#!/usr/bin/env bash
# ==============================================================================
# LUNABOT WEB MISSION CONTROL LAUNCHER
# Runs the FastAPI + ROS 2 Web Mission Control Dashboard on port 8080.
# Can be run in a separate terminal while Gazebo/SLAM/Nav2 runs in another.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

echo "========================================================================="
echo " 🚀 LAUNCHING LUNABOT WEB MISSION CONTROL (SEPARATE PROCESS)"
echo "========================================================================="

# Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ Error: /opt/ros/humble/setup.bash not found."
    exit 1
fi

# Source workspace install
if [ -f "$WORKSPACE_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"
fi

export LUNA_PRO_ROOT="$WORKSPACE_ROOT"

# Free port 8080 if an older server instance is lingering
if command -v fuser >/dev/null 2>&1; then
    fuser -k 8080/tcp 2>/dev/null || true
fi

# Run dashboard app with simulation time enabled
exec python3 "$WORKSPACE_ROOT/tools/web_dashboard/app.py" --ros-args -p use_sim_time:=true "$@"
