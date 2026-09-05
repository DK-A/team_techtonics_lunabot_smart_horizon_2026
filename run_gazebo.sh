#!/usr/bin/env bash
# ==============================================================================
# 🌕 LUNABOT: GAZEBO SIMULATION & ROS 2 AUTONOMY LAUNCHER
# Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/run_gazebo.sh
# Team Techtonics • Smart Horizon 2026 Lunar Autonomy Challenge
#
# Launches in Terminal 1:
#  1. Gazebo Sim 8 Lunar Environment (1.62 m/s² gravity, crater regolith)
#  2. ROS 2 <-> Gazebo Bridge (Sensor topics, camera streams, cmd_vel)
#  3. SLAM Toolbox & EKF Odometry Localization
#  4. Nav2 Autonomous Navigation & Dynamic Keepout Zone Supervisor
#  5. Stereo SGBM 3D Depth Perception & Terramechanics ML
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

echo "========================================================================="
echo " 🌕 TEAM TECHTONICS • LUNABOT GAZEBO SIMULATION & AUTONOMY"
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

# 3. Clean up stale Gazebo processes
echo "🧹 Cleaning up previous Gazebo processes..."
killall -9 gz-sim-server gz-sim-gui 2>/dev/null || true
sleep 1

cleanup() {
    echo ""
    echo "========================================================================="
    echo " 🛑 SHUTTING DOWN GAZEBO SIMULATION & AUTONOMY..."
    echo "========================================================================="
    killall -9 gz-sim-server gz-sim-gui 2>/dev/null || true
    echo "✅ Gazebo simulation stopped cleanly."
}
trap cleanup EXIT INT TERM

echo ""
echo "🚀 Spawning Gazebo Sim 8, SLAM Toolbox, Nav2, and Sensor Bridges..."
echo "👉 In Terminal 2, run:       ./run_dashboard.sh"
echo "👉 In Terminal 3 (on Pi), run: python3 edge_agent.py"
echo "========================================================================="
echo ""

exec ros2 launch lunabot_bringup lunabot_bringup.launch.py \
    launch_gazebo:=true \
    slam:=true \
    nav2:=true \
    web:=false \
    use_sim_time:=true "$@"
