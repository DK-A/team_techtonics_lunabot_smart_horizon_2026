-- ==============================================================================
-- LUNABOT MISSION CONTROL TIME-SERIES & TELEMETRY DATABASE SCHEMA
-- Engine: SQLite3 (WAL Mode) / Embedded Time-Series Telemetry Store
-- Target: tools/web_dashboard/database/schema.sql
-- ==============================================================================

PRAGMA journal_mode = WAL;
PRAGMA synchronous = NORMAL;
PRAGMA foreign_keys = ON;

-- 1. Real-Time Telemetry Time-Series
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
CREATE INDEX IF NOT EXISTS idx_telemetry_nav_status ON telemetry_records(nav_status);
CREATE INDEX IF NOT EXISTS idx_telemetry_edge_online ON telemetry_records(edge_online);

-- 2. Explainable AI (XAI) Decision Audit Log
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
CREATE INDEX IF NOT EXISTS idx_xai_category ON xai_audit_logs(category);
CREATE INDEX IF NOT EXISTS idx_xai_severity ON xai_audit_logs(severity);

-- 3. Autonomous Navigation Waypoint Dispatches
CREATE TABLE IF NOT EXISTS waypoint_dispatches (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    dispatched_at REAL NOT NULL,
    completed_at REAL,
    waypoint_name TEXT NOT NULL,
    target_x REAL NOT NULL,
    target_y REAL NOT NULL,
    target_yaw_deg REAL,
    status TEXT NOT NULL,         -- PENDING, EXECUTING, ARRIVED, ABORTED, BLOCKED
    obstacle_detour_executed INTEGER DEFAULT 0,
    science_dwell_sec REAL DEFAULT 0.0
);

-- 4. Volatiles & Subsurface Science Plume Detections
CREATE TABLE IF NOT EXISTS science_plume_detections (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    detected_at REAL NOT NULL,
    loc_x REAL NOT NULL,
    loc_y REAL NOT NULL,
    anomaly_score REAL NOT NULL,
    threshold REAL NOT NULL,
    o2_pct REAL,
    pressure_surge_hpa REAL,
    radiation_msv_h REAL,
    sample_analysis TEXT,
    hazard_classification TEXT    -- 'HABITAT_LEAK', 'VOLCANIC_FISSURE', 'TRACE_VOLATILE'
);

-- 5. Mission Flight Recordings Manifest
CREATE TABLE IF NOT EXISTS mission_recordings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    recording_id TEXT UNIQUE NOT NULL,
    started_at REAL NOT NULL,
    ended_at REAL,
    duration_sec REAL,
    total_frames INTEGER,
    file_path TEXT NOT NULL,
    file_size_mb REAL,
    video_channel TEXT DEFAULT 'STEREO_DEPTH_COMPOSITE',
    notes TEXT
);
