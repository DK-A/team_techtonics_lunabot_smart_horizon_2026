# 💾 LunaBot Web Ground Station — Database & Telemetry Persistence

This directory manages persistent time-series telemetry storage, audit trails, and mission flight recordings.

```
database/
├── schema.sql          # DDL schema for SQLite3 time-series, XAI decision logs, and waypoints
├── models.py           # Dataclass serialization schemas (TelemetryRecord, XAIEvent, WaypointRecord)
├── db_manager.py       # High-performance SQLite3 WAL-mode database manager
├── mission_control.db  # Active embedded time-series database file
└── recordings/         # Saved flight recording manifests, JSON telemetry logs & MP4 captures
```

## Storage Architecture & Schema Design

1. **Write-Ahead Logging (WAL Mode)**:
   - Configured with `PRAGMA journal_mode = WAL;` and `PRAGMA synchronous = NORMAL;` allowing concurrent multi-threaded writes from ROS 2 background threads while servicing web API queries without lock contention.

2. **Tables**:
   - `telemetry_records`: High-resolution time-series storing robot coordinates $(x, y, \theta)$, velocity, slip ratio, sinkage depth, gas anomaly scores, and Raspberry Pi 4B physical SoC vitals (temperature, load, RAM).
   - `xai_audit_logs`: Immutable decision audit trail capturing every autonomous action (e.g. speed reduction, hazard detour, science plume dwell).
   - `waypoint_dispatches`: Log of all dispatched autonomous targets, completion timestamps, and detour metrics.
   - `science_plume_detections`: Coordinates and spectrometer readings for anomalous gas volatiles.
   - `mission_recordings`: Metadata manifests for recorded mission runs.

3. **Multi-Tier Mission Data Pipeline**:
   - **Hot Tier**: FastAPI in-memory state engine serving real-time 10 Hz web streams.
   - **Warm Tier**: SQLite3 time-series database storing operational logs for query, replay, and analytics.
   - **Cold Tier**: ROS 2 Bag (`rosbag2` MCAP) recording lossless binary sensor data for post-mission forensic analysis.
