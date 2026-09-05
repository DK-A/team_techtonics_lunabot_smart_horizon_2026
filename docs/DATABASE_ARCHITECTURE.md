# 💾 LunaBot Mission Control: Multi-Tier Database & Telemetry Persistence Architecture

**Document Version**: 2.4  
**Classification**: Engineering Architecture Whitepaper  
**Target Systems**: Web Mission Control, ROS 2 Humble DDS, Raspberry Pi 4B Edge Gateway  

---

## 1. Executive Summary

Autonomous planetary rovers generate diverse data streams with fundamentally competing constraints:
1. **High-frequency control & visualization**: Needs sub-millisecond in-memory access for real-time web streaming at 10 Hz without disk I/O bottlenecks.
2. **Operational query & analytics**: Needs indexed relational / time-series queries for telemetry graphs, anomaly alerts, and astronaut question-answering.
3. **Lossless flight forensics**: Needs zero-copy, binary flight recording (`rosbag2`) capable of capturing high-throughput LiDAR scans and stereo image streams without dropping frames.

To solve this, LunaBot implements a **4-Tier Data Pipeline**:

```
 ┌─────────────────────────────────────────────────────────────────────────┐
 │               TIER 1: IN-MEMORY HOT TELEMETRY RING BUFFER               │
 │  • Sub-millisecond read/write in FastAPI async memory                   │
 │  • Real-time SSE / WebSocket streaming to web HUD                       │
 └────────────────────────────────────┬────────────────────────────────────┘
                                      │ (Decimated 1-2 Hz Snapshot)
                                      ▼
 ┌─────────────────────────────────────────────────────────────────────────┐
 │             TIER 2: OPERATIONAL RELATIONAL TIME-SERIES DB               │
 │  • SQLite3 with Write-Ahead Logging (WAL Mode)                          │
 │  • Tables: telemetry_records, xai_audit_logs, waypoint_dispatches       │
 │  • Fast SQL time-window queries, CSV/JSON export, historical analytics  │
 └─────────────────────────────────────────────────────────────────────────┘

 ┌─────────────────────────────────────────────────────────────────────────┐
 │            TIER 3: AEROSPACE FLIGHT BLACKBOX (ROS 2 MCAP)               │
 │  • High-throughput zero-copy serialization via rosbag2                  │
 │  • Records raw LiDAR (/scan), Stereo images, and 3D point clouds        │
 │  • Complete mission playback, digital twin replay, and model validation │
 └─────────────────────────────────────────────────────────────────────────┘

 ┌─────────────────────────────────────────────────────────────────────────┐
 │            TIER 4: SEMANTIC VECTOR STORE (EXPLAINABLE AI)               │
 │  • TF-IDF N-gram document embeddings & cosine distance matrices         │
 │  • Apollo LRV physics, NASA LADEE benchmarks, operational flight rules   │
 │  • Powers the Natural Language XAI Copilot                              │
 └─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Tier 2: Relational Time-Series Schema (SQLite3 WAL)

### Engine Configuration
```sql
PRAGMA journal_mode = WAL;
PRAGMA synchronous = NORMAL;
PRAGMA foreign_keys = ON;
```
*Why WAL Mode?* In WAL (Write-Ahead Logging) mode, reading processes never block writing processes, and writers never block readers. This guarantees that background ROS 2 telemetry ingestion threads can write at full frequency without causing UI lag in the browser.

### Key Table Schemas

#### A. `telemetry_records` (Time-Series Rover Telemetry)
Stores kinematic state, slip ratios, regolith sinkage, exosphere volatiles, and edge hardware vitals:
```sql
CREATE TABLE IF NOT EXISTS telemetry_records (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL NOT NULL,
    iso_time TEXT NOT NULL,
    robot_x REAL NOT NULL,
    robot_y REAL NOT NULL,
    robot_yaw_deg REAL NOT NULL,
    linear_speed REAL NOT NULL,
    angular_speed REAL NOT NULL,
    nav_status TEXT NOT NULL,
    target_name TEXT,
    target_dist_remaining REAL,
    slip_ratio REAL,
    sinkage_depth_mm REAL,
    terramechanics_class TEXT,
    gas_o2_pct REAL,
    ambient_pressure_hpa REAL,
    surface_temp_c REAL,
    dust_density_ug_m3 REAL,
    radiation_msv_h REAL,
    gas_anomaly_score REAL,
    is_gas_anomaly INTEGER DEFAULT 0,
    edge_online INTEGER DEFAULT 0,
    edge_device_name TEXT,
    edge_cpu_temp_c REAL,
    edge_ram_usage TEXT,
    edge_cpu_load TEXT,
    edge_latency_ms REAL
);
CREATE INDEX IF NOT EXISTS idx_telemetry_timestamp ON telemetry_records(timestamp);
```

#### B. `xai_audit_logs` (Autonomous Decision Audit Trail)
Maintains an immutable record of every autonomous action taken by the onboard supervisor:
```sql
CREATE TABLE IF NOT EXISTS xai_audit_logs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp REAL NOT NULL,
    iso_time TEXT NOT NULL,
    category TEXT NOT NULL,       -- MISSION, NAV, TERRA, SCIENCE, SAFETY, EDGE
    severity TEXT NOT NULL,       -- INFO, WARN, ALERT, CRITICAL, SUCCESS
    explanation TEXT NOT NULL,    -- Human-readable rationale
    actor TEXT DEFAULT 'AUTONOMOUS_SUPERVISOR',
    operator_ack INTEGER DEFAULT 0
);
CREATE INDEX IF NOT EXISTS idx_xai_timestamp ON xai_audit_logs(timestamp);
```

---

## 3. Tier 3: ROS 2 Bag (MCAP) Aerospace Blackbox

For high-throughput sensor telemetry that would overwhelm traditional SQL databases (such as raw $800 \times 600$ video frames and 360-point LiDAR scans), the system uses the official ROS 2 **MCAP** storage plugin:

```bash
# Record entire mission sensor telemetry into high-speed binary MCAP bag:
ros2 bag record -s mcap \
    /odom \
    /scan \
    /cmd_vel \
    /camera/left/image_raw \
    /camera/right/image_raw \
    /stereo/depth_sgbm \
    /imu/data \
    /environmental_sensor \
    /terramechanics_ml
```

### Advantages of the MCAP Format:
1. **Zero-Copy Serialization**: Minimal CPU overhead on the host computer.
2. **Self-Contained Schemas**: Stores full ROS 2 message definitions within the file header, ensuring files remain readable years later regardless of ROS distribution.
3. **Random-Access Indexing**: Supports instant seeking to any point in the mission timeline without decompressing the entire dataset.

---

## 4. API Endpoints for Data Access

The FastAPI backend exposes clean endpoints to query the database programmatically:

| Endpoint | Method | Description |
| :--- | :--- | :--- |
| `/api/db/stats` | `GET` | Returns total records count across all tables and active database file path |
| `/api/db/recent_telemetry` | `GET` | Fetches the most recent $N$ time-series telemetry records (supports `limit=50`) |
| `/api/db/recent_xai` | `GET` | Fetches recent Explainable AI decisions and audit logs |
| `/api/telemetry` | `GET` | Instantaneous Tier 1 in-memory state snapshot |
