#!/usr/bin/env bash
# ==============================================================================
# 🌕 LUNABOT: SINGLE-COMMAND TURNKEY DEMONSTRATION LAUNCHER
# Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/run_demo.sh
# Team Techtonics • Smart Horizon 2026 Lunar Autonomy Challenge
#
# Launches the full workstation stack in one command:
#  1. Gazebo Sim 8 Lunar Environment (1.62 m/s² gravity, crater regolith)
#  2. ROS 2 <-> Gazebo Bridge (Sensor topics, camera streams, cmd_vel)
#  3. SLAM Toolbox & EKF Odometry Localization
#  4. Nav2 Autonomous Navigation & Dynamic Keepout Zone Supervisor
#  5. Stereo SGBM 3D Depth Perception & Terramechanics ML
#  6. FastAPI Web Mission Control Ground Station HUD (Port 8080)
#  7. Automatic Browser Launch & Real-time Raspberry Pi 4B Freeze Lock
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

echo "========================================================================="
echo " 🌕 TEAM TECHTONICS • LUNABOT AUTONOMOUS EXPLORATION ROVER"
echo " 🚀 INITIALIZING FULL MISSION DEMONSTRATION STACK"
echo "========================================================================="

# 1. Source ROS 2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ Error: /opt/ros/humble/setup.bash not found. Please install ROS 2 Humble."
    exit 1
fi

# 2. Source Workspace Colcon Build
if [ -f "$WORKSPACE_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"
else
    echo "⚠️ Notice: Workspace setup.bash not found. Building workspace..."
    (cd "$WORKSPACE_ROOT/ros2_ws" && colcon build --symlink-install)
    source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"
fi

export LUNA_PRO_ROOT="$WORKSPACE_ROOT"

# 3. Clean up stale processes
echo "🧹 Cleaning up previous simulation and server processes..."
if command -v fuser >/dev/null 2>&1; then
    fuser -k 8080/tcp 2>/dev/null || true
fi
killall -9 gz-sim-server gz-sim-gui 2>/dev/null || true
sleep 1

# 4. Graceful Process Cleanup Handler on Exit
AUTONOMY_PID=""
cleanup() {
    echo ""
    echo "========================================================================="
    echo " 🛑 SHUTTING DOWN LUNABOT DEMONSTRATION STACK..."
    echo "========================================================================="
    if [ -n "$AUTONOMY_PID" ]; then
        kill -TERM "$AUTONOMY_PID" 2>/dev/null || true
    fi
    if command -v fuser >/dev/null 2>&1; then
        fuser -k 8080/tcp 2>/dev/null || true
    fi
    killall -9 gz-sim-server gz-sim-gui 2>/dev/null || true
    echo "✅ Clean shutdown complete. Farewell from Team Techtonics!"
}
trap cleanup EXIT INT TERM

# 5. Launch Gazebo Sim 8 + ROS 2 Autonomy Stack (Background)
echo "🚀 Spawning Gazebo Sim 8, SLAM Toolbox, Nav2, and Sensor Bridges..."
ros2 launch lunabot_bringup lunabot_bringup.launch.py \
    launch_gazebo:=true \
    slam:=true \
    nav2:=true \
    web:=false \
    use_sim_time:=true > /tmp/lunabot_autonomy.log 2>&1 &
AUTONOMY_PID=$!

echo "⏳ Waiting 4 seconds for Gazebo physics and ROS 2 graph to initialize..."
sleep 4

# 6. Auto-Open Web Browser in Background
(
    sleep 3
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "http://localhost:8080" >/dev/null 2>&1 || true
    fi
) &

# 7. Print Operator Guide & Evaluator Summary
echo ""
echo "========================================================================="
echo " 🌐 MISSION CONTROL GROUND STATION IS LIVE!"
echo "========================================================================="
echo "  💻 Local HUD:       http://localhost:8080"
echo "  📡 Network HUD:     http://10.42.0.1:8080 (or your host IP)"
echo "  ❄️ System State:    SIMULATION FROZEN (Awaiting Raspberry Pi 4B Link)"
echo ""
echo "  📲 MANUAL PI COMPONENT STEP:"
echo "     Open a separate terminal and log into your Raspberry Pi 4B:"
echo "     $ ssh techtonics@10.42.0.91"
echo "     $ cd ~/Desktop/SMART_HORIZON/LUNA_PRO/edge_pi"
echo "     $ python3 edge_agent.py"
echo ""
echo "  ⚡ BEHAVIOR:"
echo "     • Simulation & rover drives are FROZEN until the Pi 4B connects."
echo "     • Starting edge_agent.py on the Pi UNFREEZES Gazebo in real-time."
echo "     • Pressing Ctrl+C on the Pi instantly FREEZES everything."
echo "     • Press Ctrl+C in THIS terminal to stop the entire demo."
echo "========================================================================="
echo ""

# 8. Run Web Mission Control Server in Foreground
exec python3 "$WORKSPACE_ROOT/tools/web_dashboard/app.py" --ros-args -p use_sim_time:=true "$@"
