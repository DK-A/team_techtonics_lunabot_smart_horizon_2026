"""
==============================================================================
LUNABOT MISSION CONTROL DATA MODELS
Location: tools/web_dashboard/database/models.py

Defines type-safe dataclasses and serialization schemas for telemetry,
XAI audit decisions, science volatile detections, and waypoint dispatch.
==============================================================================
"""

from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any
import time


@dataclass
class TelemetryRecord:
    timestamp: float
    iso_time: str
    robot_x: float
    robot_y: float
    robot_yaw_deg: float
    linear_speed: float
    angular_speed: float
    nav_status: str
    target_name: Optional[str] = None
    target_dist_remaining: Optional[float] = None
    slip_ratio: Optional[float] = None
    sinkage_depth_mm: Optional[float] = None
    terramechanics_class: Optional[str] = None
    gas_o2_pct: Optional[float] = None
    ambient_pressure_hpa: Optional[float] = None
    surface_temp_c: Optional[float] = None
    dust_density_ug_m3: Optional[float] = None
    radiation_msv_h: Optional[float] = None
    gas_anomaly_score: Optional[float] = None
    is_gas_anomaly: int = 0
    edge_online: int = 0
    edge_device_name: Optional[str] = None
    edge_cpu_temp_c: Optional[float] = None
    edge_ram_usage: Optional[str] = None
    edge_cpu_load: Optional[str] = None
    edge_latency_ms: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class XAIEvent:
    timestamp: float
    iso_time: str
    category: str
    severity: str
    explanation: str
    actor: str = "AUTONOMOUS_SUPERVISOR"
    operator_ack: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class WaypointRecord:
    dispatched_at: float
    waypoint_name: str
    target_x: float
    target_y: float
    status: str
    target_yaw_deg: Optional[float] = None
    completed_at: Optional[float] = None
    obstacle_detour_executed: int = 0
    science_dwell_sec: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ScienceSample:
    detected_at: float
    loc_x: float
    loc_y: float
    anomaly_score: float
    threshold: float
    o2_pct: Optional[float] = None
    pressure_surge_hpa: Optional[float] = None
    radiation_msv_h: Optional[float] = None
    sample_analysis: Optional[str] = None
    hazard_classification: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
