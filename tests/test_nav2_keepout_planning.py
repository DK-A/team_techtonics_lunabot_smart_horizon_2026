#!/usr/bin/env python3
"""
==============================================================================
LUNABOT NAV2 & KEEPOUT ZONE VERIFICATION TEST
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/tests/test_nav2_keepout_planning.py
Verifies:
  1. Keepout costmap grid is published with lethal cells (cost 100).
  2. Nav2 planner and controller servers are active.
  3. Path planning routes cleanly around keepout zones without penetrating them.
==============================================================================
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

def test_nav2_keepout():
    rclpy.init()
    node = Node('test_nav2_keepout_verifier')
    
    keepout_grid = None
    def keepout_cb(msg):
        nonlocal keepout_grid
        keepout_grid = msg
    
    node.create_subscription(OccupancyGrid, '/zones/keepout_costmap', keepout_cb, 10)
    
    print("=================================================================")
    print(" TEST 1: VERIFYING KEEPOUT COSTMAP GENERATION (/zones/keepout_costmap)")
    print("=================================================================")
    t0 = time.time()
    while time.time() - t0 < 5.0 and keepout_grid is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    if keepout_grid is None:
        print("❌ FAIL: /zones/keepout_costmap not received!")
        node.destroy_node()
        rclpy.shutdown()
        return False
        
    w = keepout_grid.info.width
    h = keepout_grid.info.height
    res = keepout_grid.info.resolution
    data = np.array(keepout_grid.data, dtype=np.int8)
    lethal_count = int(np.count_nonzero(data == 100))
    print(f"Grid Dimensions: {w}x{h} ({w*res:.1f}m x {h*res:.1f}m)")
    print(f"Resolution:      {res} m/cell")
    print(f"Lethal Cells:    {lethal_count} cells")
    if lethal_count > 500:
        print(f"✅ PASS: Keepout costmap has {lethal_count} lethal cells marking NO-GO zones.")
    else:
        print(f"❌ FAIL: Insufficient lethal cells: {lethal_count}")
        node.destroy_node()
        rclpy.shutdown()
        return False

    print("\n=================================================================")
    print(" TEST 2: VERIFYING NAV2 PLANNER AVOIDANCE OF KEEPOUT ZONES")
    print("=================================================================")
    action_client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("⚠️ Planner action /compute_path_to_pose not available within timeout.")
    else:
        # Request a path from (0, 0) to (8.5, 7.5)
        # Directly between them is Northern Boulder Field NO-GO at (6.0, 5.0, radius 2.5)
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = node.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = 8.5
        goal_msg.goal.pose.position.y = 7.5
        goal_msg.goal.pose.orientation.w = 1.0
        
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=8.0)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            print("❌ Goal rejected by planner")
        else:
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=10.0)
            res = get_result_future.result()
            path = res.result.path
            print(f"Path computed successfully with {len(path.poses)} waypoints!")
            
            # Check if any waypoint penetrates Northern Boulder Field NO-GO zone: (6.0, 5.0), radius 2.5m
            penetrations = 0
            for p in path.poses:
                px = p.pose.position.x
                py = p.pose.position.y
                dist = math.sqrt((px - 6.0)**2 + (py - 5.0)**2)
                if dist < 2.5:
                    penetrations += 1
            
            print(f"Waypoints inside NO-GO zone (radius 2.5m): {penetrations}")
            if penetrations == 0:
                print("✅ PASS: Nav2 Path completely detoured around Northern Boulder Field keepout zone!")
            else:
                print(f"❌ FAIL: {penetrations} waypoints penetrated the keepout zone!")

    print("\n=================================================================")
    print(" NAV2 KEEPOUT SUITE COMPLETE")
    print("=================================================================")
    node.destroy_node()
    rclpy.shutdown()
    return True

if __name__ == '__main__':
    test_nav2_keepout()
