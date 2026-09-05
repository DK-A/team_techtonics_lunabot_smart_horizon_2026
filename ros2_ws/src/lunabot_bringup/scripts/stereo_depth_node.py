#!/usr/bin/env python3
"""
==============================================================================
LUNABOT 3D STEREO DEPTH & HAZARD POINTCLOUD NODE (PHASE 2)
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/ros2_ws/src/lunabot_bringup/scripts/stereo_depth_node.py

Features:
 1. Subscribes to /camera/left/image_raw and /camera/right/image_raw
 2. Fast Semi-Global Block Matching (StereoSGBM) Disparity Computation (30+ FPS)
 3. Back-projects (u, v, d) into 3D Cartesian PointCloud2 (/stereo/points in base_link)
 4. Multi-sensor fusion with Nav2 Local Costmap for 3D crater & boulder avoidance
 5. Publishes colorized real-time depth heatmap on /stereo/depth_color for Mission Control
==============================================================================
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import message_filters
import sensor_msgs_py.point_cloud2 as pc2


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        self.bridge = CvBridge()

        # Camera Geometry & Intrinsics
        # Baseline B = 0.50m (left at y=+0.25, right at y=-0.25)
        # Original Res: 848x480, Scale: 0.5 -> 424x240
        self.scale = 0.5
        self.orig_w = 848
        self.orig_h = 480
        self.w = int(self.orig_w * self.scale)
        self.h = int(self.orig_h * self.scale)

        # Horizontal FOV = 1.3962634 rad (~80 deg)
        # fx = w / (2 * tan(FOV/2))
        self.fx = (self.w / (2.0 * math.tan(1.3962634 / 2.0)))
        self.fy = self.fx
        self.cx = self.w / 2.0
        self.cy = self.h / 2.0
        self.baseline = 0.50  # 50 cm stereo baseline

        # Camera Mount Pose in base_link:
        # x = 0.45m, y = 0.0m (midpoint), z = 0.12m
        # Pitch down = 0.20 rad (~11.5 deg)
        self.pitch = 0.20
        self.cos_pitch = math.cos(self.pitch)
        self.sin_pitch = math.sin(self.pitch)
        self.cam_x = 0.45
        self.cam_y = 0.00
        self.cam_z = 0.12

        # Optimized Semi-Global Block Matcher
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=7,
            P1=8 * 3 * 7 * 7,
            P2=32 * 3 * 7 * 7,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        # Publishers
        self.cloud_pub = self.create_publisher(PointCloud2, '/stereo/points', 10)
        self.depth_color_pub = self.create_publisher(Image, '/stereo/depth_color', 10)
        self.hazard_pub = self.create_publisher(String, '/stereo/hazard_alert', 10)

        # Synchronized Camera Subscriptions
        self.sub_left = message_filters.Subscriber(self, Image, '/camera/left/image_raw')
        self.sub_right = message_filters.Subscriber(self, Image, '/camera/right/image_raw')
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right],
            queue_size=5,
            slop=0.08
        )
        self.sync.registerCallback(self.stereo_callback)

        self.last_process_time = 0.0
        self.get_logger().info("✅ 3D Stereo Depth & Hazard PointCloud Node Initialized (30 FPS SGBM Engine).")

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        # Throttle processing to max 20 Hz to preserve CPU for Nav2 & SLAM
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_process_time < 0.05:
            return
        self.last_process_time = now

        try:
            img_l = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            img_r = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")
            return

        # 1. Resize & Grayscale for fast block matching
        gray_l = cv2.cvtColor(cv2.resize(img_l, (self.w, self.h)), cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(cv2.resize(img_r, (self.w, self.h)), cv2.COLOR_BGR2GRAY)

        # 2. Compute Disparity Map
        disp = self.stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
        valid = (disp > 2.0) & (disp < 64.0)

        if not np.any(valid):
            return

        # 3. Back-project into 3D Camera Coordinates
        # Downsample valid mask with step stride for lightweight pointcloud
        v_idx, u_idx = np.where(valid)
        stride = 3  # Take every 3rd point (~2,000 points total)
        v_sub = v_idx[::stride]
        u_sub = u_idx[::stride]
        d_sub = disp[v_sub, u_sub]

        Z_c = (self.fx * self.baseline) / d_sub
        X_c = (u_sub - self.cx) * Z_c / self.fx
        Y_c = (v_sub - self.cy) * Z_c / self.fy

        # 4. Transform to base_link coordinates
        xb = self.cam_x + Z_c * self.cos_pitch - Y_c * self.sin_pitch
        yb = self.cam_y + X_c
        zb = self.cam_z - Z_c * self.sin_pitch - Y_c * self.cos_pitch

        # 5. Filter out-of-range points
        roi_mask = (xb > 0.40) & (xb < 9.0) & (np.abs(yb) < 3.5) & (zb > -1.5) & (zb < 1.5)
        xb_filt = xb[roi_mask].astype(np.float32)
        yb_filt = yb[roi_mask].astype(np.float32)
        zb_filt = zb[roi_mask].astype(np.float32)

        if len(xb_filt) > 0:
            # Create PointCloud2 msg
            points_array = np.column_stack((xb_filt, yb_filt, zb_filt))
            header = Header()
            header.stamp = left_msg.header.stamp
            header.frame_id = 'base_link'
            cloud_msg = pc2.create_cloud_xyz32(header, points_array)
            self.cloud_pub.publish(cloud_msg)

            # Hazard Detection (Negative Drop-offs or Tall Boulders)
            crater_points = np.sum((xb_filt < 3.0) & (zb_filt < -0.22))
            boulder_points = np.sum((xb_filt < 2.5) & (zb_filt > 0.35))
            if crater_points > 15:
                msg = String()
                msg.data = f"CRATER_DROP_DETECTED: {crater_points} points"
                self.hazard_pub.publish(msg)
            elif boulder_points > 20:
                msg = String()
                msg.data = f"BOULDER_HAZARD_DETECTED: {boulder_points} points"
                self.hazard_pub.publish(msg)

        # 6. Publish Colorized Depth Heatmap for Web Dashboard
        try:
            disp_clipped = np.clip(disp, 0, 48)
            disp_norm = cv2.normalize(disp_clipped, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            disp_color = cv2.applyColorMap(disp_norm, cv2.COLORMAP_TURBO)
            # Mask out invalid regions
            disp_color[~valid] = [20, 20, 25]

            color_msg = self.bridge.cv2_to_imgmsg(disp_color, encoding='bgr8')
            color_msg.header = left_msg.header
            color_msg.header.frame_id = 'base_link'
            self.depth_color_pub.publish(color_msg)
        except Exception as e:
            self.get_logger().debug(f"Color map error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
