#!/usr/bin/env python3
"""
==============================================================================
LUNABOT ENVIRONMENTAL SENSOR MODULE NODE (PHASE 4)
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/src/lunabot_bringup/scripts/environmental_sensor_node.py
Publishes individual ROS 2 topics for Temp, O2, Pressure, Thermal, and Dust
==============================================================================
"""

import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import ChannelFloat32
from nav_msgs.msg import Odometry
import json
import os
import sys
import pickle
import numpy as np

class EnvironmentalSensorNode(Node):
    def __init__(self):
        super().__init__('environmental_sensor_node')
        
        self.frame_id = 'environmental_sensor_link'
        self.current_speed = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.filtered_dust = 12.4
        
        # Load trained Isolation Forest model (.pkl)
        self.iso_model = None
        self.iso_threshold = 0.5377
        pkl_path = "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ml_models/isolation_forest_lunar_gas.pkl"
        if os.path.exists(pkl_path):
            try:
                sys.path.insert(0, "/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ml_models")
                with open(pkl_path, "rb") as f:
                    meta = pickle.load(f)
                    self.iso_model = meta["model"]
                    self.iso_threshold = meta.get("threshold", 0.5377)
                self.get_logger().info(f"Loaded trained Isolation Forest ML model from {pkl_path} (Thresh: {self.iso_threshold:.4f})")
            except Exception as e:
                self.get_logger().warn(f"Could not load Isolation Forest .pkl: {e}")
        
        # Subscribe to odometry for real-time motion and location correlation
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # 1. Combined Telemetry Payload & ML Alerts
        self.telemetry_pub = self.create_publisher(String, '/environmental/telemetry', 10)
        self.ml_alert_pub = self.create_publisher(String, '/environmental/ml_anomaly_alert', 10)
        
        # 2. Individual Dedicated ROS 2 Topics for each sensor
        self.temp_pub = self.create_publisher(Float32, '/environmental/temperature', 10)
        self.o2_pub   = self.create_publisher(Float32, '/environmental/o2', 10)
        self.press_pub = self.create_publisher(Float32, '/environmental/pressure', 10)
        self.thermal_pub = self.create_publisher(Float32, '/environmental/thermal', 10)
        self.dust_pub  = self.create_publisher(ChannelFloat32, '/environmental/dust', 10)
        
        # Publish timer (10 Hz for ultra-smooth real-time responsiveness)
        self.timer = self.create_timer(0.1, self.publish_telemetry)
        self.get_logger().info('Dynamic Real-Time Environmental Sensor Node initialized with Isolation Forest ML.')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.current_speed = math.sqrt(vx * vx + vy * vy) + 0.3 * abs(wz)

    def publish_telemetry(self):
        now = self.get_clock().now().to_msg()
        sec = now.sec + now.nanosec * 1e-9
        
        # 1. Dynamic Lunar Regolith Temperature:
        # Surface temperature in low-sun lunar topography (-45°C to -65°C / 208 K - 228 K)
        spatial_thermal = 6.5 * math.sin(0.18 * self.current_x + 0.12 * self.current_y) + 1.8 * math.cos(0.04 * sec)
        temp_noise = random.gauss(0, 0.12)
        temp_val = round(228.15 + spatial_thermal + temp_noise, 2)  # around -45.0 °C
        
        # 2. Dynamic Regolith Dust Concentration:
        # Electrostatic levitation + 6WD wheel regolith churn
        target_dust = 11.2 + 75.0 * min(self.current_speed, 1.2) + 2.0 * math.sin(0.3 * sec) + random.gauss(0, 0.8)
        self.filtered_dust = max(6.0, 0.85 * self.filtered_dust + 0.15 * target_dust)
        dust_val = round(self.filtered_dust, 1)
        
        # 3. Dynamic Lunar Cosmic & Solar Radiation:
        # Unshielded lunar surface galactic cosmic rays (GCR) + solar wind particle flux
        # Apollo / Chang'e 4 measured ~0.06 to 0.35 mSv/h during lunar day
        rad_val = round(max(0.05, 0.315 + 0.035 * math.sin(0.08 * sec) + random.gauss(0, 0.008)), 3)
        
        # 4. Lunar Surface Exosphere Hard Vacuum & Zero Oxygen:
        # The Moon has no atmosphere: ultra-hard vacuum ~3.0 x 10^-10 hPa, O2 = 0.00% (vacuum)
        press_gauge_variation = 3.0 + 0.4 * math.sin(0.12 * sec) + random.gauss(0, 0.15)
        press_str = f"{press_gauge_variation:.1f}e-10"
        press_val = float(press_gauge_variation * 1e-10)
        o2_val = 0.00  # Hard lunar vacuum has 0.00% breathable oxygen
        therm_val = round(temp_val + 2.5 + random.gauss(0, 0.15), 2)
        solar_flux_w_m2 = round(1361.0 + 4.5 * math.sin(0.05 * sec), 1)
        
        # Publish Individual Standalone ROS 2 Topics
        msg_temp = Float32()
        msg_temp.data = temp_val
        self.temp_pub.publish(msg_temp)
        
        msg_o2 = Float32()
        msg_o2.data = float(o2_val)
        self.o2_pub.publish(msg_o2)
        
        msg_press = Float32()
        msg_press.data = float(press_val)
        self.press_pub.publish(msg_press)
        
        msg_therm = Float32()
        msg_therm.data = therm_val
        self.thermal_pub.publish(msg_therm)
        
        # Evaluate with Isolation Forest ML model (.pkl)
        feature_vec = np.array([[o2_val, press_val, temp_val - 273.15, dust_val, rad_val, solar_flux_w_m2]])
        ml_score = 0.05
        ml_anomaly = False
        if self.iso_model is not None:
            try:
                ml_score = float(self.iso_model.score_samples(feature_vec)[0])
                ml_anomaly = bool(ml_score >= self.iso_threshold)
            except Exception:
                pass
        
        # Publish Combined Telemetry JSON Payload
        payload = {
            "timestamp": round(sec, 2),
            "frame_id": self.frame_id,
            "ambient_temp_k": temp_val,
            "o2_percent": o2_val,
            "pressure_hpa": press_val,
            "pressure_display": f"{press_str} hPa",
            "environment_state": "LUNAR_VACUUM",
            "thermal_radiometry_k": therm_val,
            "dust_concentration_ug_m3": dust_val,
            "radiation_msv_h": rad_val,
            "solar_flux_w_m2": solar_flux_w_m2,
            "ml_anomaly_score": round(float(ml_score), 4),
            "ml_anomaly_detected": ml_anomaly
        }
        msg_str = String()
        msg_str.data = json.dumps(payload)
        self.telemetry_pub.publish(msg_str)
        
        if ml_anomaly:
            msg_alert = String()
            msg_alert.data = f"CRITICAL ENVIRONMENTAL ANOMALY (Isolation Forest Score: {ml_score:.4f})"
            self.ml_alert_pub.publish(msg_alert)
        
        # Publish Dust Channel
        msg_dust = ChannelFloat32()
        msg_dust.name = "regolith_dust"
        msg_dust.values = [float(dust_val)]
        self.dust_pub.publish(msg_dust)

def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentalSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
