import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class SymmetricRockTester(Node):
    def __init__(self):
        super().__init__('symmetric_rock_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.initial_z = None
        self.max_left_roll = 0.0
        self.max_right_roll = 0.0
        self.max_pitch = 0.0
        self.current_pos = (0.0, 0.0, 0.0)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        if self.initial_z is None:
            self.initial_z = pos.z
            
        sinr_cosp = 2 * (ori.w * ori.x + ori.y * ori.z)
        cosr_cosp = 1 - 2 * (ori.x * ori.x + ori.y * ori.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (ori.w * ori.y - ori.z * ori.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        if roll > 0 and roll > self.max_left_roll:
            self.max_left_roll = roll
        elif roll < 0 and abs(roll) > self.max_right_roll:
            self.max_right_roll = abs(roll)
            
        if abs(pitch) > self.max_pitch:
            self.max_pitch = abs(pitch)
            
        self.current_pos = (pos.x, pos.y, pos.z)

    def run_cmd(self, lin_x, ang_z, duration_sec):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

def main():
    rclpy.init()
    tester = SymmetricRockTester()
    
    print("=========================================================================")
    print(" LUNABOT LEFT VS RIGHT OBSTACLE TRAVERSAL SYMMETRY & ROLL TEST")
    print("=========================================================================")
    
    print("\n---> TEST 1: FORWARD SLOW 0.05 m/s (2.0s)")
    tester.run_cmd(0.05, 0.0, 2.0)
    
    print("\n---> TEST 2: FORWARD CRUISE 0.10 m/s (3.0s)")
    tester.run_cmd(0.10, 0.0, 3.0)

    print("\n---> TEST 3: LEFT/RIGHT TURN SYMMETRY (+0.10 / -0.10 rad/s)")
    tester.run_cmd(0.0, 0.10, 2.0)
    tester.run_cmd(0.0, -0.10, 2.0)

    print("\n---> TEST 4: STOP & SETTLE (1.0s)")
    tester.run_cmd(0.0, 0.0, 1.0)
    
    print(f"\n--- SYMMETRY MEASUREMENT RESULTS ---")
    print(f"   Max Left Roll:  {math.degrees(tester.max_left_roll):.2f}°")
    print(f"   Max Right Roll: {math.degrees(tester.max_right_roll):.2f}°")
    print(f"   Roll Delta:     {abs(math.degrees(tester.max_left_roll) - math.degrees(tester.max_right_roll)):.2f}°")
    print(f"   Max Pitch:      {math.degrees(tester.max_pitch):.2f}°")

    tester.destroy_node()
    rclpy.shutdown()
    print("\nSymmetric Rock Traversal & Roll Test Complete.")

if __name__ == '__main__':
    main()
