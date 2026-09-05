# 🏛️ LunaBot: Distributed Software Architecture & Dataflow

**Document Version**: 2.4  
**Framework**: ROS 2 Humble Hawksbill, Gazebo Sim 8, FastAPI, Scikit-Learn  

---

## 1. System Topology Overview

LunaBot employs a distributed robotics architecture designed to separate heavy physics rendering and global mapping from safety-critical onboard flight control:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           HOST WORKSTATION / SIMULATOR                          │
│                                                                                 │
│  ┌───────────────────────┐   ROS 2 gz_bridge   ┌─────────────────────────────┐  │
│  │     GAZEBO SIM 8      │ ◄─────────────────► │      ROS 2 CORE NODES       │  │
│  │ • Lunar Regolith (1.6m│                     │ • slam_toolbox (Karto SLAM) │  │
│  │ • Rocker-Bogie Joints │                     │ • nav2_controller (DWB)     │  │
│  │ • 360° LiDAR Raycast  │                     │ • nav2_planner (Navfn)      │  │
│  │ • Stereo Camera Pair  │                     │ • stereo_depth_node (SGBM)  │  │
│  └───────────────────────┘                     │ • zone_manager_node (Keepout│  │
│                                                └──────────────┬──────────────┘  │
│                                                               │                 │
│  ┌────────────────────────────────────────────────────────────┴──────────────┐  │
│  │                     FASTAPI MISSION CONTROL GROUND STATION                │  │
│  │  • Thread-safe ROS 2 Spin Daemon                                          │  │
│  │  • Multi-Camera MJPEG Streaming Engine (30 FPS)                           │  │
│  │  • Dynamic SVG SLAM Map & Click-to-Navigate Waypoint Dispatcher           │  │
│  │  • SQLite3 WAL Time-Series Telemetry & XAI Audit Logger                   │  │
│  │  • Explainable AI Semantic NLP Copilot (TF-IDF Vector Space + Gemini LLM) │  │
│  └────────────────────────────▲──────────────────────────────────────────────┘  │
└───────────────────────────────┼─────────────────────────────────────────────────┘
                                │ Direct Ethernet Bridge (10.42.0.1 <-> 10.42.0.91)
                                │ Sub-millisecond Ping (~0.18ms)
┌───────────────────────────────▼─────────────────────────────────────────────────┐
│                      RASPBERRY PI 4B ONBOARD COMPUTER (OBC)                     │
│                                                                                 │
│  ┌───────────────────────────────────────────────────────────────────────────┐  │
│  │                       SYSTEMD PERSISTENT EDGE SERVICE                     │  │
│  │  • edge_agent.service (ExecStartPre auto-fetch & auto-provisioning)       │  │
│  │  • Reads Linux Thermal Zone sysfs (/sys/class/thermal/thermal_zone0/temp) │  │
│  │  • Embedded Scikit-Learn / NumPy Terramechanics Inference (<0.8ms)        │  │
│  │  • Embedded Isolation Forest Volatile Anomaly Detection (<1.2ms)          │  │
│  │  • 2.5s Watchdog Fail-Safe: Instant Motor E-Stop on Signal Loss           │  │
│  └───────────────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Core ROS 2 Nodes & Dataflow

| Node Name | Source File | Subscribed Topics | Published Topics | Role |
| :--- | :--- | :--- | :--- | :--- |
| `stereo_depth_node` | `stereo_depth_node.py` | `/camera/left/image_raw`<br>`/camera/right/image_raw` | `/stereo/depth_sgbm`<br>`/stereo/depth_colored`<br>`/hazard/stereo_alert` | Computes dense 3D stereo disparity and depth matrix at 36 FPS using OpenCV SGBM |
| `terramechanics_ml_node` | `terramechanics_ml_node.py` | `/odom`<br>`/imu/data`<br>`/joint_states` | `/terramechanics_ml`<br>`/safety/traction_override` | Computes dynamic slip ratio and Bekker-Wong sinkage; executes Random Forest classification |
| `environmental_sensor_node` | `environmental_sensor_node.py` | `/odom` | `/environmental_sensor`<br>`/science/anomaly_alert` | Simulates lunar trace exosphere gases; runs Isolation Forest anomaly detection |
| `zone_manager_node` | `zone_manager_node.py` | `/map` | `/keepout_filter_mask`<br>`/costmap_filter_info` | Injects restricted geometric NO-GO polygons into Nav2 costmap layers |
| `web_telemetry_node` | `backend/app.py` | All sensor, map, odom, and ML topics | `/cmd_vel`<br>`/goal_pose` | Bridges ROS 2 DDS to the FastAPI Mission Control dashboard |

---

## 3. Hardware Fail-Safe Watchdog Mechanism

Because lunar rovers cannot rely on continuous ground station connectivity, LunaBot implements an active **2.5-second hardware watchdog**:

1. **Active Heartbeat**: The Raspberry Pi 4B streams physical hardware vitals (ARM SoC temperature, RAM, load) to `/api/edge_telemetry` at 1 Hz.
2. **Watchdog Evaluation**: `app.py` computes $\Delta t = t_{\text{current}} - t_{\text{last\_heartbeat}}$.
3. **Threshold Trigger ($\Delta t > 2.5\text{s}$)**:
   - Sets `edge_device.online = False`.
   - Triggers an **EMERGENCY XAI ALERT** in the audit feed: *"⚠️ ROVER HALTED — EDGE LINK LOST"*.
   - Issues zero-velocity `/cmd_vel` motor halt to prevent runaway or collision.
   - Updates dashboard UI card with red pulsing border: `❌ CONNECTION LOST / OFFLINE`.
4. **Auto-Reconnection**: As soon as Ethernet connectivity is restored, systemd re-engages within 2.0 seconds without requiring a reboot.
