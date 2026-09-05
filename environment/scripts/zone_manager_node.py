#!/usr/bin/env python3
"""
==============================================================================
LUNABOT ROS 2 ZONE MANAGER NODE
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/src/lunabot_bringup/scripts/zone_manager_node.py
Extensible node managing Static & Dynamic Nav2 Hazard Zones
==============================================================================
"""

import os
import sys
import yaml
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np

class ZoneManagerNode(Node):
    def __init__(self):
        super().__init__('zone_manager_node')
        
        # Dynamically determine workspace root
        workspace_root = os.environ.get('LUNA_PRO_ROOT', None)
        if not workspace_root or not os.path.exists(workspace_root):
            candidates = [
                os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")),
                "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO",
                os.getcwd()
            ]
            for c in candidates:
                if os.path.exists(os.path.join(c, "environment")):
                    workspace_root = c
                    break
        if not workspace_root:
            workspace_root = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO"
            
        default_config = os.path.join(workspace_root, 'environment', 'config', 'zones', 'static_zones.yaml')
        self.declare_parameter('config_path', default_config)
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        # Publishers
        self.zone_pub = self.create_publisher(String, '/zones/static_zones', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/zones/visualization_markers', 10)
        
        # Latched QoS so Nav2 receives the keepout grid once without triggering 1Hz costmap resizes
        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/zones/keepout_costmap', costmap_qos)
        
        # Load zone schema and build static keepout costmap
        self.zones_data = self.load_zones()
        self.keepout_msg = self.build_keepout_costmap()
        self.costmap_pub.publish(self.keepout_msg)
        
        # Publish timer (1 Hz for JSON & RViz markers)
        self.timer = self.create_timer(1.0, self.publish_zones)
        self.get_logger().info('Zone Manager Node initialized with latched Nav2 Keepout Costmap.')

    def load_zones(self):
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f)
                self.get_logger().info(f"Loaded {len(data.get('zones', []))} static zones from {self.config_path}")
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load static zones config: {str(e)}")
            return {"zones": []}

    def build_keepout_costmap(self):
        res = 0.05
        size_m = 40.0
        width = int(size_m / res)
        height = int(size_m / res)
        origin_x = -size_m / 2.0
        origin_y = -size_m / 2.0

        # Initialize with -1 (NO_INFORMATION) so keepout layer is transparent outside NO-GO zones
        grid = np.full((height, width), -1, dtype=np.int8)

        for zone in self.zones_data.get('zones', []):
            if zone.get('type') == 'NO_GO':
                pose = zone.get('pose', {})
                zx = float(pose.get('x', 0.0))
                zy = float(pose.get('y', 0.0))
                gtype = zone.get('geometry_type', 'CYLINDER')

                if gtype == 'CYLINDER':
                    radius = float(zone.get('dimensions', {}).get('radius', 2.0))
                    min_cx = max(0, int((zx - radius - origin_x) / res))
                    max_cx = min(width - 1, int((zx + radius - origin_x) / res))
                    min_cy = max(0, int((zy - radius - origin_y) / res))
                    max_cy = min(height - 1, int((zy + radius - origin_y) / res))

                    for cy in range(min_cy, max_cy + 1):
                        wy = origin_y + (cy + 0.5) * res
                        for cx in range(min_cx, max_cx + 1):
                            wx = origin_x + (cx + 0.5) * res
                            if (wx - zx)**2 + (wy - zy)**2 <= radius**2:
                                grid[cy, cx] = 100
                elif gtype == 'BOX':
                    sx = float(zone.get('dimensions', {}).get('size_x', 2.0)) / 2.0
                    sy = float(zone.get('dimensions', {}).get('size_y', 2.0)) / 2.0
                    min_cx = max(0, int((zx - sx - origin_x) / res))
                    max_cx = min(width - 1, int((zx + sx - origin_x) / res))
                    min_cy = max(0, int((zy - sy - origin_y) / res))
                    max_cy = min(height - 1, int((zy + sy - origin_y) / res))
                    grid[min_cy:max_cy+1, min_cx:max_cx+1] = 100

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = res
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.get_logger().info(f"Built Keepout Costmap Grid: {width}x{height} cells, {np.count_nonzero(grid==100)} lethal keepout cells.")
        return msg

    def publish_zones(self):
        # 1. Publish JSON string payload
        msg = String()
        msg.data = json.dumps(self.zones_data)
        self.zone_pub.publish(msg)
        
        # 2. Publish RViz2 / ROS 2 MarkerArray
        marker_array = MarkerArray()
        idx = 0
        
        for zone in self.zones_data.get('zones', []):
            marker = Marker()
            marker.header.frame_id = zone.get('pose', {}).get('frame_id', 'map')
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "static_zones"
            marker.id = idx
            idx += 1
            
            gtype = zone.get('geometry_type', 'CYLINDER')
            if gtype == 'CYLINDER':
                marker.type = Marker.CYLINDER
                marker.scale.x = zone.get('dimensions', {}).get('radius', 1.0) * 2.0
                marker.scale.y = zone.get('dimensions', {}).get('radius', 1.0) * 2.0
                marker.scale.z = zone.get('dimensions', {}).get('height', 1.0)
            else:
                marker.type = Marker.CUBE
                marker.scale.x = zone.get('dimensions', {}).get('size_x', 1.0)
                marker.scale.y = zone.get('dimensions', {}).get('size_y', 1.0)
                marker.scale.z = zone.get('dimensions', {}).get('size_z', 1.0)
                
            marker.action = Marker.ADD
            marker.pose.position.x = float(zone.get('pose', {}).get('x', 0.0))
            marker.pose.position.y = float(zone.get('pose', {}).get('y', 0.0))
            marker.pose.position.z = float(zone.get('pose', {}).get('z', 0.0))
            
            # Color mapping by zone type
            ztype = zone.get('type', 'SAFE')
            if ztype == 'BASE':
                marker.color.r = 0.0
                marker.color.g = 0.8
                marker.color.b = 1.0
                marker.color.a = 0.35
            elif ztype == 'SAFE':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.15
            elif ztype == 'NO_GO':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.45
            else:
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.5
                
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ZoneManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
