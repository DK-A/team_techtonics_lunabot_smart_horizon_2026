"""
==============================================================================
LUNABOT MISSION CONTROL DATABASE MANAGER
Location: tools/web_dashboard/database/db_manager.py

Handles persistent storage, time-series indexing, and querying of rover
telemetry, XAI decision audit trails, and mission flight events via SQLite3 (WAL mode).
==============================================================================
"""

import os
import sqlite3
import time
from datetime import datetime
from typing import List, Dict, Any, Optional

try:
    from .models import TelemetryRecord, XAIEvent, WaypointRecord, ScienceSample
except (ImportError, ValueError):
    from models import TelemetryRecord, XAIEvent, WaypointRecord, ScienceSample


class MissionDatabaseManager:
    """
    High-performance, embedded time-series and audit logger for LunaBot Mission Control.
    Uses SQLite3 with Write-Ahead Logging (WAL) for concurrency without blocking.
    """

    def __init__(self, db_path: Optional[str] = None):
        if db_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            db_path = os.path.join(current_dir, "mission_control.db")
        self.db_path = db_path
        self._init_db()

    def _get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, timeout=10.0)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode = WAL;")
        conn.execute("PRAGMA synchronous = NORMAL;")
        return conn

    def _init_db(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        schema_path = os.path.join(current_dir, "schema.sql")
        if os.path.exists(schema_path):
            with open(schema_path, "r") as f:
                schema_sql = f.read()
            with self._get_connection() as conn:
                conn.executescript(schema_sql)

    def log_telemetry(self, record: TelemetryRecord) -> int:
        query = """
        INSERT INTO telemetry_records (
            timestamp, iso_time, robot_x, robot_y, robot_yaw_deg,
            linear_speed, angular_speed, nav_status, target_name, target_dist_remaining,
            slip_ratio, sinkage_depth_mm, terramechanics_class,
            gas_o2_pct, ambient_pressure_hpa, surface_temp_c, dust_density_ug_m3,
            radiation_msv_h, gas_anomaly_score, is_gas_anomaly,
            edge_online, edge_device_name, edge_cpu_temp_c, edge_ram_usage, edge_cpu_load, edge_latency_ms
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        with self._get_connection() as conn:
            cursor = conn.execute(query, (
                record.timestamp, record.iso_time, record.robot_x, record.robot_y, record.robot_yaw_deg,
                record.linear_speed, record.angular_speed, record.nav_status, record.target_name, record.target_dist_remaining,
                record.slip_ratio, record.sinkage_depth_mm, record.terramechanics_class,
                record.gas_o2_pct, record.ambient_pressure_hpa, record.surface_temp_c, record.dust_density_ug_m3,
                record.radiation_msv_h, record.gas_anomaly_score, record.is_gas_anomaly,
                record.edge_online, record.edge_device_name, record.edge_cpu_temp_c, record.edge_ram_usage, record.edge_cpu_load, record.edge_latency_ms
            ))
            return cursor.lastrowid

    def log_xai_event(self, event: XAIEvent) -> int:
        query = """
        INSERT INTO xai_audit_logs (
            timestamp, iso_time, category, severity, explanation, actor, operator_ack
        ) VALUES (?, ?, ?, ?, ?, ?, ?)
        """
        with self._get_connection() as conn:
            cursor = conn.execute(query, (
                event.timestamp, event.iso_time, event.category,
                event.severity, event.explanation, event.actor, event.operator_ack
            ))
            return cursor.lastrowid

    def log_waypoint_dispatch(self, wp: WaypointRecord) -> int:
        query = """
        INSERT INTO waypoint_dispatches (
            dispatched_at, waypoint_name, target_x, target_y, target_yaw_deg, status, obstacle_detour_executed, science_dwell_sec
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """
        with self._get_connection() as conn:
            cursor = conn.execute(query, (
                wp.dispatched_at, wp.waypoint_name, wp.target_x, wp.target_y,
                wp.target_yaw_deg, wp.status, wp.obstacle_detour_executed, wp.science_dwell_sec
            ))
            return cursor.lastrowid

    def get_recent_telemetry(self, limit: int = 50) -> List[Dict[str, Any]]:
        query = "SELECT * FROM telemetry_records ORDER BY timestamp DESC LIMIT ?"
        with self._get_connection() as conn:
            rows = conn.execute(query, (limit,)).fetchall()
            return [dict(row) for row in rows]

    def get_recent_xai_logs(self, limit: int = 50) -> List[Dict[str, Any]]:
        query = "SELECT * FROM xai_audit_logs ORDER BY timestamp DESC LIMIT ?"
        with self._get_connection() as conn:
            rows = conn.execute(query, (limit,)).fetchall()
            return [dict(row) for row in rows]

    def get_stats(self) -> Dict[str, Any]:
        with self._get_connection() as conn:
            t_count = conn.execute("SELECT COUNT(*) FROM telemetry_records").fetchone()[0]
            x_count = conn.execute("SELECT COUNT(*) FROM xai_audit_logs").fetchone()[0]
            w_count = conn.execute("SELECT COUNT(*) FROM waypoint_dispatches").fetchone()[0]
            return {
                "telemetry_records_count": t_count,
                "xai_logs_count": x_count,
                "waypoints_count": w_count,
                "database_engine": "SQLite3 WAL",
                "database_path": self.db_path
            }


# Singleton instance
db_instance = MissionDatabaseManager()
