#!/usr/bin/env bash
# ==============================================================================
# 🌕 LUNABOT: WEB MISSION CONTROL DASHBOARD LAUNCHER
# Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/run_dashboard.sh
# Team Techtonics • Smart Horizon 2026 Lunar Autonomy Challenge
#
# Launches the FastAPI + ROS 2 Web Mission Control Dashboard on port 8080.
# Runs separately in Terminal 2 while Gazebo/SLAM/Nav2 runs in Terminal 1.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

echo "========================================================================="
echo " 🚀 LAUNCHING LUNABOT WEB MISSION CONTROL (GROUND STATION)"
echo "========================================================================="

# 1. Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ Error: /opt/ros/humble/setup.bash not found. Please install ROS 2 Humble."
    exit 1
fi

# 2. Source workspace install
if [ -f "$WORKSPACE_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"
fi

export LUNA_PRO_ROOT="$WORKSPACE_ROOT"

# 3. Free port 8080 if an older server instance is lingering
if command -v fuser >/dev/null 2>&1; then
    fuser -k 8080/tcp 2>/dev/null || true
fi

# 4. Auto-open browser in background
(
    sleep 2
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "http://localhost:8080" >/dev/null 2>&1 || true
    fi
) &

echo "  💻 Local HUD:       http://localhost:8080"
echo "  📡 Network HUD:     http://10.42.0.1:8080"
echo "  ❄️ Real-Time Rule:  Rover & Simulation remain FROZEN until Pi connects"
echo "========================================================================="

# 5. Run dashboard app with simulation time enabled
exec python3 "$WORKSPACE_ROOT/tools/web_dashboard/app.py" --ros-args -p use_sim_time:=true "$@"
