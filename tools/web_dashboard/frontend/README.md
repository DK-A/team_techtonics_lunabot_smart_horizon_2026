# 🎨 LunaBot Web Ground Station — Frontend Architecture

This folder contains the client-side presentation layer for the **LunaBot Industrial Web Mission Control HUD**.

```
frontend/
├── index.html          # Main Mission Control HUD layout (Multi-camera grid, SVG SLAM, LiDAR scope, Telemetry)
├── css/
│   └── dashboard.css   # Dark-mode aerospace HUD styling, glassmorphism, responsive grid, status animations
└── js/
    └── dashboard.js    # Client-side telemetry stream handler, SVG SLAM rendering, LiDAR scope canvas, teleop
```

## Core Modules & Capabilities

1. **Dynamic SVG SLAM Occupancy Map**:
   - Renders real-time SLAM occupancy grid (`/map`) with high contrast obstacle frontiers.
   - Interactive waypoint dispatch: click anywhere on the lunar surface map to send target navigation coordinates to Nav2.
   - Live rover heading arrow, laser scans, and restricted NO-GO zone polygons.

2. **LiDAR Radar Scope (25m Sweep)**:
   - Real-time HTML5 Canvas polar radar sweeping at 10 Hz.
   - Color-coded proximity hazard detection:
     - 🔴 `< 1.5m`: Critical proximity alert
     - 🟠 `< 3.5m`: Obstacle warning
     - 🟡 `< 6.0m`: Detected object
     - 🟢 `> 6.0m`: Clear traversal

3. **Multi-Camera Composite Feeds**:
   - **Cam-L**: Left Stereo Camera (800x600 @ 30 FPS)
   - **Cam-R**: Right Stereo Camera (800x600 @ 30 FPS)
   - **Cam-B**: Rear Hazard Camera (800x600 wide-angle)
   - **3D Depth**: Real-time SGBM Stereo Disparity & Depth Cloud (36 FPS)

4. **Explainable AI (XAI) & Natural Language Copilot**:
   - Live stream of transparent autonomous decisions (traction throttle, hazard keepouts, gas plume detection).
   - Conversational question-answering modal querying the rover's real-time physical telemetry.
