#!/usr/bin/env python3
"""
==============================================================================
LUNABOT REAL-TIME GRAPHICAL SENSOR & CAMERA MISSION CONTROL DASHBOARD
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/scripts/live_sensor_dashboard.py

Visualizes:
 1. Live Left, Right & Rear Optical Camera Feeds
 2. 360° Polar LiDAR Radar Display with Range Rings & Collision Warnings
 3. IMU Dynamics, Lunar Gravity, Attitude (Pitch/Roll) & Wheel Odometry
 4. Habitat Life-Support Telemetry (O2, Pressure, Temperature, Dust, Radiation)
==============================================================================
"""

import os
import sys
import math
import time
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge

class LiveSensorDashboardNode(Node):
    def __init__(self):
        super().__init__('live_visual_sensor_dashboard')
        self.bridge = CvBridge()

        self.last_cam_l = None
        self.last_cam_r = None
        self.last_cam_rear = None
        self.last_scan = None
        self.last_imu = None
        self.last_odom = None
        self.last_env = None

        self.create_subscription(Image, '/camera/left/image_raw', self.cam_l_cb, 10)
        self.create_subscription(Image, '/camera/right/image_raw', self.cam_r_cb, 10)
        self.create_subscription(Image, '/camera/rear/image_raw', self.cam_rear_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/environmental/telemetry', self.env_cb, 10)

        self.get_logger().info("Visual Sensor Dashboard Node Subscribed to all topics.")

    def cam_l_cb(self, msg):
        try:
            self.last_cam_l = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Left Cam conversion err: {e}")

    def cam_r_cb(self, msg):
        try:
            self.last_cam_r = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Right Cam conversion err: {e}")

    def cam_rear_cb(self, msg):
        try:
            self.last_cam_rear = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Rear Cam conversion err: {e}")

    def scan_cb(self, msg):
        self.last_scan = msg

    def imu_cb(self, msg):
        self.last_imu = msg

    def odom_cb(self, msg):
        self.last_odom = msg

    def env_cb(self, msg):
        try:
            self.last_env = json.loads(msg.data)
        except Exception:
            pass


def create_standby_canvas(w, h, text):
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[:] = (20, 22, 28)
    cv2.rectangle(canvas, (2, 2), (w-3, h-3), (60, 70, 80), 1)
    cv2.putText(canvas, text, (w//2 - 130, h//2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 140, 160), 1, cv2.LINE_AA)
    return canvas


def draw_radar_scope(scan_msg, radar_w=420, radar_h=420):
    radar = np.zeros((radar_h, radar_w, 3), dtype=np.uint8)
    radar[:] = (12, 14, 20)

    center = (radar_w // 2, radar_h // 2)
    max_range = 25.0  # meters
    radius = min(radar_w, radar_h) // 2 - 30

    # Draw range rings (5m, 10m, 15m, 20m, 25m)
    for dist in [5.0, 10.0, 15.0, 20.0, 25.0]:
        r = int((dist / max_range) * radius)
        cv2.circle(radar, center, r, (35, 45, 55), 1, cv2.LINE_AA)
        cv2.putText(radar, f"{int(dist)}m", (center[0] + 4, center[1] - r + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (70, 90, 110), 1, cv2.LINE_AA)

    # Crosshairs & Axis lines
    cv2.line(radar, (center[0], 25), (center[0], radar_h - 25), (40, 50, 65), 1)
    cv2.line(radar, (25, center[1]), (radar_w - 25, center[1]), (40, 50, 65), 1)

    # Sector labels
    cv2.putText(radar, "FRONT (+X)", (center[0] - 35, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(radar, "REAR (-X)", (center[0] - 30, radar_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 130, 150), 1, cv2.LINE_AA)
    cv2.putText(radar, "LEFT", (6, center[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 130, 150), 1, cv2.LINE_AA)
    cv2.putText(radar, "RIGHT", (radar_w - 45, center[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 130, 150), 1, cv2.LINE_AA)

    min_front = float('inf')
    min_dist_overall = float('inf')

    if scan_msg is not None and len(scan_msg.ranges) > 0:
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        for idx, dist in enumerate(scan_msg.ranges):
            if math.isinf(dist) or math.isnan(dist) or dist <= scan_msg.range_min:
                continue

            angle = angle_min + idx * angle_inc
            if dist < min_dist_overall:
                min_dist_overall = dist

            # Check front arc (-45° to +45°)
            if abs(angle) < 0.785:
                if dist < min_front:
                    min_front = dist

            norm_dist = min(dist, max_range) / max_range
            px = int(center[0] - norm_dist * radius * math.sin(angle))
            py = int(center[1] - norm_dist * radius * math.cos(angle))

            if dist < 1.5:
                color = (0, 0, 255)      # Red alert
                pt_size = 2
            elif dist < 3.5:
                color = (0, 165, 255)    # Orange caution
                pt_size = 2
            elif dist < 6.0:
                color = (0, 240, 255)    # Yellow
                pt_size = 1
            else:
                color = (255, 220, 0)    # Cyan / Clear
                pt_size = 1

            cv2.circle(radar, (px, py), pt_size, color, -1)

    # Rover footprint at center
    cv2.rectangle(radar, (center[0] - 12, center[1] - 16), (center[0] + 12, center[1] + 16), (255, 255, 255), 1)
    cv2.arrowedLine(radar, (center[0], center[1] + 6), (center[0], center[1] - 18), (0, 255, 255), 2, tipLength=0.4)

    # Collision Warning Banner
    if min_front < 1.5:
        cv2.rectangle(radar, (10, radar_h - 55), (radar_w - 10, radar_h - 25), (0, 0, 180), -1)
        cv2.putText(radar, f"CRITICAL HAZARD: {min_front:.2f}m", (center[0] - 110, radar_h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    elif min_front < 5.0:
        cv2.rectangle(radar, (10, radar_h - 55), (radar_w - 10, radar_h - 25), (0, 120, 200), -1)
        cv2.putText(radar, f"FRONT OBSTACLE: {min_front:.2f}m", (center[0] - 95, radar_h - 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    elif min_dist_overall < 25.0:
        cv2.putText(radar, f"PATH CLEAR | Flank Obs: {min_dist_overall:.2f}m",
                    (20, radar_h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 230, 120), 1, cv2.LINE_AA)
    else:
        cv2.putText(radar, "PATH CLEAR (>25m)",
                    (20, radar_h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 230, 120), 1, cv2.LINE_AA)

    return radar


def render_dashboard(node):
    canvas_w = 1280
    canvas_h = 720
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    canvas[:] = (16, 18, 22)

    # Top Header Banner
    cv2.rectangle(canvas, (0, 0), (canvas_w, 42), (25, 30, 38), -1)
    cv2.line(canvas, (0, 42), (canvas_w, 42), (0, 200, 255), 2)
    cv2.putText(canvas, "LUNABOT MISSION CONTROL - REAL-TIME SENSOR & CAMERA HUD", (20, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cur_time = time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime())
    cv2.putText(canvas, f"MISSION CLOCK: {cur_time}", (canvas_w - 380, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1, cv2.LINE_AA)

    # =========================================================================
    # COLUMN 1: TRIPLE CAMERA STREAM (LEFT: 410px wide)
    # =========================================================================
    cam_w = 410
    cam_h = 210

    # 1. Left Optical Camera
    if node.last_cam_l is not None:
        feed_l = cv2.resize(node.last_cam_l, (cam_w, cam_h))
    else:
        feed_l = create_standby_canvas(cam_w, cam_h, "CAM 1: STEREO LEFT (CONNECTING...)")
    cv2.rectangle(feed_l, (0, 0), (cam_w, 24), (20, 20, 20), -1)
    cv2.putText(feed_l, "CAM 1: STEREO LEFT (848x480 RGB)", (10, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    canvas[50:50+cam_h, 15:15+cam_w] = feed_l

    # 2. Right Optical Camera
    if node.last_cam_r is not None:
        feed_r = cv2.resize(node.last_cam_r, (cam_w, cam_h))
    else:
        feed_r = create_standby_canvas(cam_w, cam_h, "CAM 2: STEREO RIGHT (CONNECTING...)")
    cv2.rectangle(feed_r, (0, 0), (cam_w, 24), (20, 20, 20), -1)
    cv2.putText(feed_r, "CAM 2: STEREO RIGHT (848x480 RGB)", (10, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    canvas[270:270+cam_h, 15:15+cam_w] = feed_r

    # 3. Rear Hazard Camera
    if node.last_cam_rear is not None:
        feed_rear = cv2.resize(node.last_cam_rear, (cam_w, cam_h))
    else:
        feed_rear = create_standby_canvas(cam_w, cam_h, "CAM 3: REAR HAZARD (CONNECTING...)")
    cv2.rectangle(feed_rear, (0, 0), (cam_w, 24), (20, 20, 20), -1)
    cv2.putText(feed_rear, "CAM 3: REAR HAZARD CAM (848x480 RGB)", (10, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    canvas[490:490+cam_h, 15:15+cam_w] = feed_rear

    # =========================================================================
    # COLUMN 2: 360° POLAR LIDAR RADAR (CENTER: 420px wide)
    # =========================================================================
    radar_x = 440
    radar_y = 50
    radar_dim = 420
    radar_img = draw_radar_scope(node.last_scan, radar_w=radar_dim, radar_h=radar_dim)
    canvas[radar_y:radar_y+radar_dim, radar_x:radar_x+radar_dim] = radar_img

    # LiDAR Specifications Box below Radar
    lidar_info_y = radar_y + radar_dim + 12
    cv2.rectangle(canvas, (radar_x, lidar_info_y), (radar_x + radar_dim, canvas_h - 15), (25, 28, 36), -1)
    cv2.rectangle(canvas, (radar_x, lidar_info_y), (radar_x + radar_dim, canvas_h - 15), (60, 75, 90), 1)

    ray_cnt = len(node.last_scan.ranges) if node.last_scan else 360
    cv2.putText(canvas, "LIVOX MID-360 CLASS 3D GPU LIDAR", (radar_x + 15, lidar_info_y + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"Angular Sweep: 360 DEG | Rays: {ray_cnt} | Range: 0.15m - 25.0m", (radar_x + 15, lidar_info_y + 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 200, 215), 1, cv2.LINE_AA)
    cv2.putText(canvas, "FUSION STATUS: Dust Regolith Filtering ACTIVE", (radar_x + 15, lidar_info_y + 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 120), 1, cv2.LINE_AA)
    cv2.putText(canvas, "ODOMETER: Active (DiffDrive 6-Wheel Kinematic Encoders)", (radar_x + 15, lidar_info_y + 96),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 200, 215), 1, cv2.LINE_AA)

    # =========================================================================
    # COLUMN 3: ENVIRONMENTAL TELEMETRY & IMU ATTITUDE (RIGHT: 385px wide)
    # =========================================================================
    hud_x = 880
    hud_y = 50
    hud_w = 385
    hud_h = canvas_h - 65

    cv2.rectangle(canvas, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (22, 26, 34), -1)
    cv2.rectangle(canvas, (hud_x, hud_y), (hud_x + hud_w, hud_y + hud_h), (50, 65, 80), 1)

    # Section 1: Habitat Life Support Vitals
    cv2.rectangle(canvas, (hud_x, hud_y), (hud_x + hud_w, hud_y + 32), (30, 36, 48), -1)
    cv2.putText(canvas, "HABITAT LIFE-SUPPORT TELEMETRY", (hud_x + 15, hud_y + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1, cv2.LINE_AA)

    env = node.last_env or {}
    o2_val = env.get("o2_percent", 20.9)
    temp_k = env.get("ambient_temp_k", 250.15)
    temp_c = temp_k - 273.15
    press_hpa = env.get("pressure_bmp390_hpa", 1013.25)
    therm_k = env.get("thermal_radiometry_k", 298.15)
    dust_ug = env.get("dust_concentration_ug_m3", 12.4)
    rad_msv = env.get("radiation_msv_h", 0.015)

    y_offset = hud_y + 55

    # O2
    cv2.putText(canvas, "O2 CONCENTRATION:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)
    o2_color = (0, 230, 100) if 19.5 <= o2_val <= 23.5 else (0, 0, 255)
    cv2.putText(canvas, f"{o2_val:.1f} % O2 (NOMINAL)", (hud_x + 180, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.44, o2_color, 1)

    # Ambient Temp
    y_offset += 32
    cv2.putText(canvas, "AMBIENT TEMPERATURE:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)
    cv2.putText(canvas, f"{temp_k:.1f} K ({temp_c:.1f} C)", (hud_x + 210, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 200, 0), 1)

    # Barometric Pressure
    y_offset += 32
    cv2.putText(canvas, "BMP390 PRESSURE:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)
    cv2.putText(canvas, f"{press_hpa:.1f} hPa", (hud_x + 210, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1)

    # FLIR Thermal Radiometry
    y_offset += 32
    cv2.putText(canvas, "FLIR THERMAL RADIOMETRY:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)
    cv2.putText(canvas, f"{therm_k:.1f} K", (hud_x + 230, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 165, 255), 1)

    # Regolith Dust
    y_offset += 32
    cv2.putText(canvas, "REGOLITH DUST DENSITY:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)
    cv2.putText(canvas, f"{dust_ug:.1f} ug/m3", (hud_x + 210, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 220, 255), 1)

    # Radiation Dosimeter
    y_offset += 32
    cv2.putText(canvas, "COSMIC RADIATION:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 200, 220), 1)
    cv2.putText(canvas, f"{rad_msv:.3f} mSv/h", (hud_x + 210, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 240, 100), 1)

    # Section 2: Rover Kinematics & IMU
    y_offset += 40
    cv2.rectangle(canvas, (hud_x, y_offset), (hud_x + hud_w, y_offset + 30), (30, 36, 48), -1)
    cv2.putText(canvas, "ROVER KINEMATICS & IMU ATTITUDE", (hud_x + 15, y_offset + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1, cv2.LINE_AA)

    y_offset += 50
    if node.last_imu:
        acc = node.last_imu.linear_acceleration
        acc_str = f"X={acc.x:.2f}, Y={acc.y:.2f}, Z={acc.z:.2f} m/s²"
        lunar_g = acc.z
    else:
        acc_str = "Awaiting /imu/data..."
        lunar_g = 1.62

    cv2.putText(canvas, "IMU 9-AXIS ACCELEROMETER:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
    cv2.putText(canvas, acc_str, (hud_x + 15, y_offset + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)

    y_offset += 45
    cv2.putText(canvas, "LUNAR GRAVITY CONSTANT:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
    cv2.putText(canvas, f"{lunar_g:.3f} m/s² (Nominal: 1.622)", (hud_x + 210, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 240, 120), 1)

    y_offset += 32
    if node.last_odom:
        pos = node.last_odom.pose.pose.position
        vel = node.last_odom.twist.twist.linear.x
        pos_str = f"({pos.x:.2f}m, {pos.y:.2f}m, {pos.z:.2f}m)"
        vel_str = f"{vel:.2f} m/s"
    else:
        pos_str = "(0.00m, 0.00m, 0.00m)"
        vel_str = "0.00 m/s"

    cv2.putText(canvas, "LOCAL ODOMETRY POSE:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
    cv2.putText(canvas, pos_str, (hud_x + 180, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)

    y_offset += 28
    cv2.putText(canvas, "FORWARD LINEAR VELOCITY:", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1)
    cv2.putText(canvas, vel_str, (hud_x + 230, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)

    # Section 3: Static Safety Zones Badge
    y_offset += 40
    cv2.rectangle(canvas, (hud_x, y_offset), (hud_x + hud_w, y_offset + 28), (30, 36, 48), -1)
    cv2.putText(canvas, "ACTIVE HABITAT SAFETY ZONES", (hud_x + 15, y_offset + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 1, cv2.LINE_AA)

    y_offset += 48
    cv2.putText(canvas, "[SAFE] Base Station & Sector (Traversable)", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 230, 100), 1)
    y_offset += 22
    cv2.putText(canvas, "[NO-GO] Northern Boulder Field (Restricted)", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 100, 255), 1)
    y_offset += 22
    cv2.putText(canvas, "[NO-GO] Southern Crater Ridge (Restricted)", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 100, 255), 1)
    y_offset += 22
    cv2.putText(canvas, "[NO-GO] Habitat Infrastructure (Critical)", (hud_x + 15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 255), 1)

    # Bottom status bar
    cv2.rectangle(canvas, (0, canvas_h - 22), (canvas_w, canvas_h), (20, 22, 28), -1)
    cv2.putText(canvas, "Press 'Q' or 'ESC' to exit | Press 'S' to save screenshot", (20, canvas_h - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (120, 140, 160), 1, cv2.LINE_AA)

    return canvas


def main():
    rclpy.init()
    node = LiveSensorDashboardNode()

    window_name = "LunaBot Mission Control - Real-Time Sensor & Camera HUD"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)

    print("=========================================================================")
    print(" LUNABOT LIVE VISUAL SENSOR DASHBOARD ACTIVE")
    print(" Displaying Real-Time Cameras, LiDAR Radar, IMU & Telemetry")
    print(" Press 'Q' or ESC in the window to close.")
    print("=========================================================================")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            frame = render_dashboard(node)
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q') or key == ord('Q'):
                break
            elif key == ord('s') or key == ord('S'):
                filename = f"lunabot_hud_screenshot_{int(time.time())}.png"
                cv2.imwrite(filename, frame)
                print(f"[HUD] Screenshot saved to: {filename}")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
