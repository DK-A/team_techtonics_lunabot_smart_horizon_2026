#!/usr/bin/env python3
"""
==============================================================================
LUNABOT STANDALONE MANUAL TELEOP TEST CONTROLLER (HIGH-SPEED EDITION)
Location: /home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/tools/manual_rover_control/manual_rover_control.py
Publishes geometry_msgs/msg/Twist to /cmd_vel over ROS 2 Humble
==============================================================================
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
==============================================================================
    LUNABOT STANDALONE MANUAL TEST CONTROLLER (HIGH SPEED: 0.80 m/s)
==============================================================================
  Controls:
    [W] : Move Forward              [S] : Move Backward
    [A] : Rotate Left               [D] : Rotate Right
    [Q] : Arc Forward-Left          [E] : Arc Forward-Right
    [SPACE] / [X] : Emergency Stop / Zero Velocity

  Speed Adjustments:
    [+] : Increase Linear Speed     [-] : Decrease Linear Speed
    [[] : Increase Angular Speed    []] : Decrease Angular Speed

  Press [Ctrl+C] or [ESC] to Exit.
==============================================================================
"""

class ManualRoverController(Node):
    def __init__(self):
        super().__init__('manual_rover_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.linear_speed = 0.80   # Default high linear speed: 0.80 m/s
        self.angular_speed = 0.60  # Default high angular speed: 0.60 rad/s
        
        self.current_lx = 0.0
        self.current_az = 0.0
        
        # 10 Hz continuous publisher timer
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info('High-Speed Manual Rover Controller initialized. Target topic: /cmd_vel')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = float(self.current_lx)
        msg.angular.z = float(self.current_az)
        self.pub.publish(msg)

    def stop_rover(self):
        self.current_lx = 0.0
        self.current_az = 0.0
        self.publish_velocity()

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    controller = ManualRoverController()
    
    print(BANNER)
    print(f"Current Speeds -> Linear: {controller.linear_speed:.2f} m/s | Angular: {controller.angular_speed:.2f} rad/s\n")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.01)
            key = get_key(settings)
            
            if key in ['w', 'W']:
                controller.current_lx = controller.linear_speed
                controller.current_az = 0.0
                print(f"\r[STATUS] MOVING FORWARD  ({controller.current_lx:.2f} m/s)         ", end="")
            elif key in ['s', 'S']:
                controller.current_lx = -controller.linear_speed
                controller.current_az = 0.0
                print(f"\r[STATUS] MOVING REVERSE  ({controller.current_lx:.2f} m/s)         ", end="")
            elif key in ['a', 'A']:
                controller.current_lx = 0.0
                controller.current_az = controller.angular_speed
                print(f"\r[STATUS] ROTATING LEFT   ({controller.current_az:.2f} rad/s)       ", end="")
            elif key in ['d', 'D']:
                controller.current_lx = 0.0
                controller.current_az = -controller.angular_speed
                print(f"\r[STATUS] ROTATING RIGHT  ({controller.current_az:.2f} rad/s)       ", end="")
            elif key in ['q', 'Q']:
                controller.current_lx = controller.linear_speed
                controller.current_az = controller.angular_speed * 0.75
                print(f"\r[STATUS] ARC FORWARD-LEFT ({controller.current_lx:.2f} m/s, {controller.current_az:.2f} rad/s) ", end="")
            elif key in ['e', 'E']:
                controller.current_lx = controller.linear_speed
                controller.current_az = -controller.angular_speed * 0.75
                print(f"\r[STATUS] ARC FORWARD-RIGHT ({controller.current_lx:.2f} m/s, {controller.current_az:.2f} rad/s)", end="")
            elif key in [' ', 'x', 'X']:
                controller.stop_rover()
                print(f"\r[STATUS] EMERGENCY STOP  (0.00 m/s)                          ", end="")
            elif key in ['+', '=']:
                controller.linear_speed = min(2.5, controller.linear_speed + 0.10)
                print(f"\r[CONFIG] Linear Speed Increased -> {controller.linear_speed:.2f} m/s    ", end="")
            elif key in ['-', '_']:
                controller.linear_speed = max(0.10, controller.linear_speed - 0.10)
                print(f"\r[CONFIG] Linear Speed Decreased -> {controller.linear_speed:.2f} m/s    ", end="")
            elif key in ['[']:
                controller.angular_speed = min(2.0, controller.angular_speed + 0.10)
                print(f"\r[CONFIG] Angular Speed Increased -> {controller.angular_speed:.2f} rad/s", end="")
            elif key in [']']:
                controller.angular_speed = max(0.10, controller.angular_speed - 0.10)
                print(f"\r[CONFIG] Angular Speed Decreased -> {controller.angular_speed:.2f} rad/s", end="")
            elif key in ['\x03', '\x1b']:  # Ctrl+C or ESC
                break
                
    except Exception as e:
        print(f"\nError in Manual Controller: {e}")
    finally:
        controller.stop_rover()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        controller.destroy_node()
        rclpy.shutdown()
        print("\n\nManual Rover Controller Safely Terminated. Zero Velocity Published.")

if __name__ == '__main__':
    main()
