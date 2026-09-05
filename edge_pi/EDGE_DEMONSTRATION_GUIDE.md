# 🌕 LunaBot: Raspberry Pi 4B Edge Computing & Gateway Demonstration Guide

This guide details how to configure, connect, and showcase your **Raspberry Pi 4 Model B** as the real-time **Edge Rover Onboard Computer (OBC)** bridging the Gazebo physics simulation and the Web Mission Control Dashboard.

---

## 🏛️ System Architecture Overview

In aerospace engineering (NASA JPL Perseverance / VIPER rovers), high-fidelity physics and rendering cannot run on a flight computer, while ground station commands cannot be trusted for real-time safety due to transmission latency (1.3s Moon-to-Earth delay).

```
   ┌────────────────────────┐                    ┌────────────────────────┐                    ┌────────────────────────┐
   │      HOST PC / GPU     │                    │  RASPBERRY PI 4B (OBC) │                    │ GROUND MISSION CONTROL │
   │   (Simulation Laptop)  │  ◄── ROS 2 DDS ──► │  (Edge Compute Node)   │  ◄── HTTP/WS ───►  │  (Web Dashboard / HUD) │
   │                        │                    │                        │                    │                        │
   │ • Gazebo Sim 8 Physics │                    │ • Hardware Telemetry   │                    │ • Real-Time 3D Map HUD │
   │ • 3D LiDAR & Cameras   │                    │ • Isolation Forest ML  │                    │ • XAI Decision Stream  │
   │ • SLAM Toolbox Mapping │                    │ • Terramechanics RF    │                    │ • Teleoperation & Wayp │
   │ • Nav2 Global Planner  │                    │ • Instant Tilt E-Stop  │                    │ • Edge Health Vitals   │
   └────────────────────────┘                    └────────────────────────┘                    └────────────────────────┘
```

---

## 🚀 3 Demonstration Steps (Turnkey Setup)

### Step 1: Connect the Raspberry Pi 4B to Your Network
Choose either of the two standard connection options:
- **Option A (Wi-Fi Router)**: Connect both your laptop and Raspberry Pi 4B to the same Wi-Fi network (or mobile hotspot).
- **Option B (Direct Ethernet Cable)**: Plug an Ethernet cable directly between your laptop and the Raspberry Pi 4B.

Verify both devices share the same ROS 2 Domain:
```bash
# Run on BOTH the Laptop and the Raspberry Pi:
export ROS_DOMAIN_ID=0
```

---

### Step 2: Run the Simulation & Dashboard on Your Laptop
Terminal 1 (Gazebo + SLAM + Nav2):
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/install/setup.bash
ros2 launch lunabot_bringup lunabot_bringup.launch.py launch_gazebo:=true slam:=true nav2:=true web:=false
```

Terminal 2 (Web Mission Control Dashboard):
```bash
cd ~/Desktop/SMART_HORIZON/LUNA_PRO
./run_dashboard.sh
```

---

### Step 3: Run the Edge Gateway on the Raspberry Pi 4B
Copy the `edge_pi/` folder and models to your Raspberry Pi, then execute:
```bash
cd edge_pi
./run_edge_bridge.sh
```

---

## 🎯 What Appears on the Dashboard During the Demo

1. **📟 RPi 4B Edge Gateway Card**:
   - Status badge turns green: `RPi 4B EDGE ONLINE`.
   - Displays real-time **ARM CPU Temperature**, **RAM utilization**, and **Load average** directly from the physical Raspberry Pi!
   - Confirms **Onboard ML Inference** active for both the Isolation Forest and Terramechanics Random Forest.
2. **🧠 Explainable AI Live Decision Stream**:
   - Streams transparent English reasoning logs explaining rover movement, dust conditions, radiation scans, and terramechanics slip states.
3. **🚨 Instant Tilt Safety / E-Stop**:
   - If the rover drives over an extreme crater slope (>28° tilt), the Raspberry Pi triggers an instantaneous onboard safety brake independently of the ground station!

---

## 🎤 Impressive Demonstration Talking Points (For Judges & Evaluators)

When demonstrating to judges or evaluators:
1. **"Why use an Edge Computer?"**:
   > *"In space exploration, radio signals between the Moon and Earth experience a 2.6-second round-trip latency. If a rover begins slipping down a crater wall, waiting for Earth ground control would result in a catastrophic rollover. By deploying our trained Scikit-Learn Isolation Forest and Terramechanics models directly onto this Raspberry Pi 4B ARM processor, all safety-critical anomaly detection and slip mitigation happen locally within 2 milliseconds."*
2. **"What role does the Host PC play?"**:
   > *"The host workstation simulates the harsh lunar regolith environment, lunar gravity (1.62 m/s²), and sensor photometrics in Gazebo Sim 8. The Raspberry Pi acts as the physical Rover Onboard Computer (OBC), receiving raw sensor packets over industrial ROS 2 DDS and streaming telemetry to the web dashboard."*
3. **"Look at the Hardware Vitals"**:
   > *"Notice the live hardware tile on our dashboard: it streams real-time CPU thermal health and memory metrics directly from the Raspberry Pi's internal Linux kernel thermal zone sysfs, proving true hardware-in-the-loop edge execution."*
