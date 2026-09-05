import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class RockTraversalTester(Node):
    def __init__(self):
        super().__init__('rock_traversal_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.initial_z = None
        self.max_z_displacement = 0.0
        self.max_pitch = 0.0
        self.max_roll = 0.0
        self.current_pos = (0.0, 0.0, 0.0)
        
    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        if self.initial_z is None:
            self.initial_z = pos.z
            
        z_disp = abs(pos.z - self.initial_z)
        if z_disp > self.max_z_displacement:
            self.max_z_displacement = z_disp
            
        # Convert quaternion to pitch & roll
        sinr_cosp = 2 * (ori.w * ori.x + ori.y * ori.z)
        cosr_cosp = 1 - 2 * (ori.x * ori.x + ori.y * ori.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (ori.w * ori.y - ori.z * ori.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        if abs(pitch) > self.max_pitch:
            self.max_pitch = abs(pitch)
        if abs(roll) > self.max_roll:
            self.max_roll = abs(roll)
            
        self.current_pos = (pos.x, pos.y, pos.z)

    def run_rock_test(self, speed, duration):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = 0.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            
        print(f"   [MEASUREMENT] Speed: {speed:.2f} m/s | Pos: ({self.current_pos[0]:.3f}, {self.current_pos[1]:.3f}, {self.current_pos[2]:.3f})")
        print(f"   [MEASUREMENT] Max Chassis Z Lift: {self.max_z_displacement:.4f} m | Max Pitch: {math.degrees(self.max_pitch):.2f}° | Max Roll: {math.degrees(self.max_roll):.2f}°")

def main():
    rclpy.init()
    tester = RockTraversalTester()
    
    print("=========================================================================")
    print(" LUNABOT ROCKER-BOGIE ROCK TRAVERSAL & CHASSIS LIFT AUDIT")
    print("=========================================================================")
    
    print("\n---> TEST 1: LOW SPEED 0.05 m/s OBSTACLE CLIMBING (2.0s)")
    tester.run_rock_test(0.05, 2.0)
    
    print("\n---> TEST 2: LOW SPEED 0.10 m/s OBSTACLE CLIMBING (3.0s)")
    tester.run_rock_test(0.10, 3.0)

    print("\n---> TEST 3: CRUISE SPEED 0.20 m/s OBSTACLE CLIMBING (3.0s)")
    tester.run_rock_test(0.20, 3.0)
    
    print("\n---> TEST 4: STOP & SETTLE (1.0s)")
    tester.run_rock_test(0.00, 1.0)
    
    tester.destroy_node()
    rclpy.shutdown()
    print("\nRock Traversal & Chassis Lift Audit Complete.")

if __name__ == '__main__':
    main()
