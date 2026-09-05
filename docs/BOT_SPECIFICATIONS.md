# 🌕 LunaBot: Complete System & Hardware Specifications

**Project**: LunaBot Autonomous Lunar Exploration Rover  
**Challenge**: Smart Horizon 2026 Lunar Autonomy Challenge  
**Team**: Techtonics  

---

## 1. Physical Dimensions & Mass Properties

| Specification | Metric Value | Imperial Value | Engineering Notes |
| :--- | :--- | :--- | :--- |
| **Total Rover Mass** | **101.0 kg** | **222.7 lbs** | Flight-ready wet mass with full sensor payload |
| **Chassis & Avionics Frame** | 60.0 kg | 132.3 lbs | Aluminum 7075-T6 tubular spaceframe |
| **Suspension Assembly** | 23.0 kg | 50.7 lbs | Rockers (12kg), Bogies (8kg), Differential Bar (3kg) |
| **Wheel Assemblies (x6)** | 18.0 kg (3.0 kg/wheel) | 39.7 lbs | Direct-drive in-hub brushless motors |
| **Overall Length (Wheelbase)** | **1.30 m** | **51.2 in** | Front wheel axle to rear wheel axle |
| **Overall Width (Track)** | **0.94 m** | **37.0 in** | Left tire centerline to right tire centerline |
| **Overall Height** | **0.82 m** | **32.3 in** | Ground to highest point of sensor mast |
| **Ground Clearance** | **0.35 m (350 mm)** | **13.8 in** | Ample clearance over lunar rocks & craters |
| **Center of Mass (CoM)** | $[0.0, 0.0, -0.14\text{m}]$ | — | Ultra-low CoM ensures tip-over stability up to $38^\circ$ |

---

## 2. Suspension & Mobility Kinematics

- **Suspension Architecture**: True Passive **Rocker-Bogie** with pitch-averaging differential bar.
- **Differential Mechanism**: Top-mounted transverse differential pivot averaging left/right rocker angles $(\theta_{\text{body}} = \frac{\theta_L + \theta_R}{2})$.
- **Obstacle Climbing**: Traverses rocks and obstacles up to **$0.25\text{ m}$ ($25\text{ cm}$)**—greater than the wheel radius ($18.25\text{ cm}$)—without chassis body tilt.
- **Slope Gradeability**: Traverses slopes up to **$25^\circ$** in loose lunar regolith; static rollover threshold exceeds **$38^\circ$**.
- **Turing Radius**: **$0.0\text{ m}$ (Zero-radius skid-steer & pivot turn)** for tight maneuvering in crater fields.

### Wheel Specifications
- **Wheel Diameter**: **$0.365\text{ m}$ ($36.5\text{ cm}$)** ($r = 0.1825\text{ m}$).
- **Wheel Width / Contact Face**: **$0.160\text{ m}$ ($16.0\text{ cm}$)**.
- **Tread Design**: Cleated grousers ($15\text{ mm}$ chevron lugs) optimized for lunar regolith shear strength (Bekker-Wong mechanics).
- **Drive System**: 6 independent in-hub brushless DC motors with planetary gearboxes.

---

## 3. Sensor Suite & Perception Payload

```
                    ┌─────────────────────────┐
                    │ 360° LiDAR SENSOR MAST  │  (25m Sweep, 10 Hz)
                    └────────────┬────────────┘
                                 │
           ┌─────────────────────┼─────────────────────┐
           │                     │                     │
┌──────────┴──────────┐┌─────────┴─────────┐┌──────────┴──────────┐
│ STEREO CAM-L (800x6)││ STEREO CAM-R (800)││ REAR HAZARD CAM (800)│
└─────────────────────┘└───────────────────┘└─────────────────────┘
```

| Sensor Name | Model / Type | Update Rate | Resolution / Range | Primary Purpose |
| :--- | :--- | :--- | :--- | :--- |
| **360° Planar LiDAR** | RPLiDAR S2 / Solid-state | 10 Hz | $0.05\text{ m} - 25.0\text{ m}$ range, $0.25^\circ$ ang res | SLAM mapping, obstacle detection, proximity radar |
| **Stereo Vision Pair** | Dual Global Shutter CMOS | 30 FPS | $800 \times 600$, Baseline: $120\text{ mm}$, FOV: $80^\circ$ | Semi-Global Block Matching (SGBM) 3D depth & point cloud |
| **Rear Hazard Camera** | Wide-Angle CMOS | 30 FPS | $800 \times 600$, FOV: $90^\circ$ | Blind-spot hazard avoidance during autonomous reverse |
| **Inertial Measurement Unit** | 9-DOF IMU (Acc+Gyro+Mag) | 100 Hz | 3-axis Acc ($\pm 16g$), 3-axis Gyro ($\pm 2000^\circ/s$) | High-rate orientation, EKF odometry fusion, tilt safety |
| **Environmental Gas Spectrometer**| Multi-Channel Exosphere Sensor| 5 Hz | $O_2$ (ppm), Pressure (torr), Temp ($^\circ\text{C}$), Dust ($\mu\text{g/m}^3$) | Subsurface outgassing detection, habitat leak warning |
| **Radiation Dosimeter** | Solid-state Ionizing Sensor | 1 Hz | $0.01 - 100.0\text{ mSv/h}$ | Cosmic ray & solar particle event monitoring |

---

## 4. Distributed Dual-Architecture Compute

To overcome the **2.6-second round-trip radio latency** between Earth and the Moon, LunaBot employs a dual-tier compute topology:

```
┌─────────────────────────────────────────┐         Low-Latency Ethernet Bridge         ┌─────────────────────────────────────────┐
│     RASPBERRY PI 4B ONBOARD COMPUTER    │ ◄─────────────────────────────────────────► │        GROUND MISSION CONTROL HOST      │
│   (Physical Hardware-in-the-Loop OBC)   │           (10.42.0.1 <-> 10.42.0.91)        │           (Workstation / Laptop)        │
├─────────────────────────────────────────┤                                             ├─────────────────────────────────────────┤
│ • Broadcom BCM2711 ARM Cortex-A72 Quad  │                                             │ • Multi-core x86_64 CPU + GPU           │
│ • 4GB LPDDR4-3200 SDRAM                 │                                             │ • Gazebo Sim 8 (Lunar Regolith Physics) │
│ • Ubuntu 22.04 LTS                      │                                             │ • SLAM Toolbox (Graph SLAM Mapping)     │
│ • Systemd Auto-Start Service            │                                             │ • Nav2 Autonomous Path Planner          │
│ • Real-time SoC Thermal Sysfs Monitor   │                                             │ • FastAPI Industrial Mission Control    │
│ • Terramechanics ML (<0.8ms inference)  │                                             │ • Multi-Camera MJPEG Streaming          │
│ • Isolation Forest Anomaly Detection    │                                             │ • Explainable AI Natural Language Agent │
│ • 2.5s Hardware Watchdog Fail-Safe      │                                             │ • SQLite3 Time-Series Telemetry DB      │
└─────────────────────────────────────────┘                                             └─────────────────────────────────────────┘
```

---

## 5. Power & Thermal System

- **Battery Chemistry**: Space-qualified Lithium Iron Phosphate ($\text{LiFePO}_4$) pack.
- **Nominal Voltage**: $24.0\text{ V DC}$ ($48\text{ Ah}$, $1152\text{ Wh}$ capacity).
- **Operational Endurance**: $6.5\text{ hours}$ continuous autonomous traversal with all sensors and compute online.
- **Thermal Range**: Operational from $-55^\circ\text{C}$ to $+85^\circ\text{C}$ using multi-layer insulation (MLI) blankets and embedded radioisotope thermal units (RHUs).
