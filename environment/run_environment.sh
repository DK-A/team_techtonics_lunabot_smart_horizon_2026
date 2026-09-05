#!/usr/bin/env bash
# ==============================================================================
# Standalone Lunar Simulation Environment Launcher
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LUNA_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Ensure clean state by terminating stale background Gazebo server processes
killall -9 gz-sim-server gz-sim-gui 2>/dev/null || true
sleep 0.5

# Clear cached GUI tracking state to prevent auto-rotation restore
rm -f ~/.gz/sim/8/gui.config 2>/dev/null || true

# Configure Gazebo Resource Paths using absolute paths for lunabot model resolution
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${LUNA_ROOT}/environment/models:${LUNA_ROOT}/environment/worlds:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="${GZ_SIM_RESOURCE_PATH}"

# Rendering Engine Configuration (ogre2 required for GPU LiDAR and stereo camera rendering)
export GZ_RENDERING_ENGINE_BACKEND=ogre2
export MESA_GL_VERSION_OVERRIDE=4.5
export MESA_GLSL_VERSION_OVERRIDE=450
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

# Rendering / Display configuration
export DISPLAY="${DISPLAY:-:1}"

WORLD_PATH="${SCRIPT_DIR}/worlds/moon.sdf"
if [ ! -f "${WORLD_PATH}" ]; then
    WORLD_PATH="${LUNA_ROOT}/environment/worlds/moon.sdf"
fi

echo "========================================================"
echo " Launching Lunar Simulation Environment"
echo " World File: ${WORLD_PATH}"
echo " Resource Path: ${GZ_SIM_RESOURCE_PATH}"
echo "========================================================"

exec gz sim -r "${WORLD_PATH}"
