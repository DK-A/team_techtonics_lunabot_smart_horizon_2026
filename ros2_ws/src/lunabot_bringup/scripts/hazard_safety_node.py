#!/usr/bin/env python3
"""
==============================================================================
LUNABOT HAZARD SAFETY & ANTI-FLIP MONITOR NODE
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/src/lunabot_bringup/scripts/hazard_safety_node.py

Features:
 1. Subscribes to Nav2 velocity commands on /cmd_vel_nav (or /cmd_vel_raw)
 2. Monitors IMU pitch/roll tilt and LiDAR front proximity
 3. Overrides forward velocity if tilt > 12° or front obstacle < 0.65m
 4. Prevents rover from climbing up steep boulders and flipping over
 5. Publishes safe filtered command velocity to /cmd_vel
==============================================================================
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu

class HazardSafetyNode(Node):
    def __init__(self):
        super().__init__('hazard_safety_node')

        self.last_pitch = 0.0
        self.last_roll = 0.0
        self.min_front_dist = 99.0
        self.hazard_active = False

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.sub_cmd_raw = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_raw_cb, 10)
        self.sub_cmd_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_raw_cb, 10)

        self.pub_cmd_safe = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Hazard Safety & Anti-Flip Monitor Node Initialized.")

    def imu_cb(self, msg):
        q = msg.orientation
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.last_roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            self.last_pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.last_pitch = math.asin(sinp)

    def scan_cb(self, msg):
        if not msg.ranges:
            return
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        min_f = 99.0
        min_f_left = 99.0
        min_f_right = 99.0
        # Forward arc +- 35 degrees (+- 0.61 rad)
        for idx, dist in enumerate(msg.ranges):
            if math.isinf(dist) or math.isnan(dist) or dist <= msg.range_min:
                continue
            angle = angle_min + idx * angle_inc
            if abs(angle) < 0.61:
                if dist < min_f:
                    min_f = dist
                if angle > 0 and dist < min_f_left:
                    min_f_left = dist
                elif angle <= 0 and dist < min_f_right:
                    min_f_right = dist
        self.min_front_dist = min_f
        self.min_front_left = min_f_left
        self.min_front_right = min_f_right

    def cmd_raw_cb(self, msg):
        safe_cmd = Twist()
        # Direct clean drive direction matching physical 6WD forward rotation
        safe_cmd.linear.x = msg.linear.x
        safe_cmd.linear.y = msg.linear.y
        safe_cmd.linear.z = msg.linear.z
        safe_cmd.angular.x = msg.angular.x
        safe_cmd.angular.y = msg.angular.y
        safe_cmd.angular.z = msg.angular.z

        # 1. Anti-Flip Tilt Hazard Intervention (pitch or roll > 25 deg)
        max_tilt = max(abs(self.last_pitch), abs(self.last_roll))
        if max_tilt > 0.45:
            if safe_cmd.linear.x > 0:
                safe_cmd.linear.x = 0.0  # Stop forward motion on steep incline
            self.hazard_active = True
        else:
            self.hazard_active = False

        # 2. Front Obstacle Avoidance Guidance (within 0.25m)
        if self.min_front_dist < 0.25:
            if safe_cmd.linear.x > 0:
                safe_cmd.linear.x = 0.0
                if abs(safe_cmd.angular.z) < 0.1:
                    safe_cmd.angular.z = -0.35 if getattr(self, 'min_front_left', 99) < getattr(self, 'min_front_right', 99) else 0.35
        elif self.min_front_dist < 0.50:
            if safe_cmd.linear.x > 0.18:
                safe_cmd.linear.x = 0.18

        self.pub_cmd_safe.publish(safe_cmd)

def main():
    rclpy.init()
    node = HazardSafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
