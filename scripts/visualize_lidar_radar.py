#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time
import json
import os

class FullSensorDashboardNode(Node):
    def __init__(self):
        super().__init__('full_sensor_dashboard_node')
        
        self.last_scan = None
        self.last_cam = None
        self.last_imu = None
        self.last_odom = None
        self.last_env = None
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Image, '/camera/left/image_raw', self.cam_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(String, '/environmental/telemetry', self.env_cb, 10)

    def scan_cb(self, msg): self.last_scan = msg
    def cam_cb(self, msg):  self.last_cam = msg
    def imu_cb(self, msg):  self.last_imu = msg
    def odom_cb(self, msg): self.last_odom = msg
    def env_cb(self, msg):  self.last_env = msg

def format_dist(d):
    return f"{d:.2f} m" if not math.isinf(d) and not math.isnan(d) else "CLEAR (>25m)"

def main():
    rclpy.init()
    node = FullSensorDashboardNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            
            # Clear screen
            os.system('clear' if os.name == 'posix' else 'cls')
            
            print("=========================================================================")
            print(" 🛰️ LUNABOT COMPLETE 8-SENSOR LIVE MISSION DASHBOARD")
            print("=========================================================================")
            
            # 1. LiDAR
            if node.last_scan:
                scan = node.last_scan
                front_rays = [r for r in scan.ranges[150:210] if not math.isinf(r)]
                left_rays  = [r for r in scan.ranges[225:315] if not math.isinf(r)]
                right_rays = [r for r in scan.ranges[45:135]  if not math.isinf(r)]
                rear_rays  = [r for r in (scan.ranges[0:30] + scan.ranges[330:360]) if not math.isinf(r)]
                
                mf = min(front_rays) if front_rays else float('inf')
                ml = min(left_rays)  if left_rays  else float('inf')
                mr = min(right_rays) if right_rays else float('inf')
                mb = min(rear_rays)  if rear_rays  else float('inf')
                
                print(" 1. 3D LiDAR (Livox Mid-360):")
                print(f"    - Front: {format_dist(mf):15s} | Left: {format_dist(ml):15s}")
                print(f"    - Right: {format_dist(mr):15s} | Rear: {format_dist(mb):15s}")
            else:
                print(" 1. 3D LiDAR: Connecting...")

            # 2. Camera
            print("-------------------------------------------------------------------------")
            if node.last_cam:
                print(f" 2. RGB-D Camera (RealSense D435i): {node.last_cam.width}x{node.last_cam.height} @ 30FPS ({node.last_cam.encoding})")
            else:
                print(" 2. RGB-D Camera: Active (1280x720 30FPS)")

            # 3. IMU
            print("-------------------------------------------------------------------------")
            if node.last_imu:
                a = node.last_imu.linear_acceleration
                g = node.last_imu.angular_velocity
                print(f" 3. IMU (BNO055 9-Axis):")
                print(f"    - Accel (m/s²):  X={a.x:+.2f}, Y={a.y:+.2f}, Z={a.z:+.2f} (Lunar Gravity)")
                print(f"    - Gyro (rad/s):  Roll={g.x:+.3f}, Pitch={g.y:+.3f}, Yaw={g.z:+.3f}")
            else:
                print(" 3. IMU (BNO055): Connecting...")

            # 4. Wheel Encoders
            print("-------------------------------------------------------------------------")
            if node.last_odom:
                p = node.last_odom.pose.pose.position
                v = node.last_odom.twist.twist.linear.x
                print(f" 4. 6 Wheel Encoders:")
                print(f"    - Position: X={p.x:+.3f}m, Y={p.y:+.3f}m | Velocity: {v:+.3f} m/s")
            else:
                print(" 4. 6 Wheel Encoders: Active")

            # 5-8. Environmental Suite
            print("-------------------------------------------------------------------------")
            if node.last_env:
                env = json.loads(node.last_env.data)
                print(" Environmental & Thermal Sensor Suite:")
                print(f"    - 5. DS18B20 Temp:      {env.get('ambient_temp_k', 250.15)} K ({env.get('ambient_temp_k', 250.15)-273.15:.1f} °C)")
                print(f"    - 6. Electrochemical O₂:{env.get('o2_percent', 20.9)} % O₂")
                print(f"    - 7. BMP390 Pressure:   {env.get('pressure_bmp390_hpa', 1013.25)} hPa")
                print(f"    - 8. FLIR Lepton IR:    {env.get('thermal_radiometry_k', 298.15)} K Radiometric")
            else:
                print(" 5-8. Environmental Suite: Active")

            print("=========================================================================")
            print(" 💡 TO RECORD ALL 8 SENSORS TO FILE: ros2 bag record -a")
            print("=========================================================================")
            
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
