import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class MobilityTester(Node):
    def __init__(self):
        super().__init__('mobility_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.current_pos = (0.0, 0.0, 0.0)
        self.current_vel = (0.0, 0.0)

    def odom_cb(self, msg):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        self.current_vel = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )

    def send_cmd(self, lin_x, ang_z, duration_sec):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            
        print(f"     [ODOM] Pos: ({self.current_pos[0]:.3f}, {self.current_pos[1]:.3f}, {self.current_pos[2]:.3f}) | Vel: Lin={self.current_vel[0]:.3f} m/s, Ang={self.current_vel[1]:.3f} rad/s")

def main():
    rclpy.init()
    tester = MobilityTester()
    
    print("=========================================================================")
    print(" LUNABOT HIGH-SPEED MOBILITY & STABILITY TEST SUITE")
    print("=========================================================================")
    
    # 1. Medium Speed Forward (0.5 m/s)
    print("\n---> MEDIUM SPEED 0.50 m/s FORWARD: CMD (linear.x=0.50 m/s, angular.z=0.00 rad/s) for 2.0s")
    tester.send_cmd(0.50, 0.0, 2.0)
    
    # 2. High Speed Forward (0.80 m/s)
    print("\n---> HIGH SPEED 0.80 m/s FORWARD: CMD (linear.x=0.80 m/s, angular.z=0.00 rad/s) for 2.0s")
    tester.send_cmd(0.80, 0.0, 2.0)

    # 3. Maximum Cruise Speed Forward (1.20 m/s)
    print("\n---> MAXIMUM CRUISE SPEED 1.20 m/s FORWARD: CMD (linear.x=1.20 m/s, angular.z=0.00 rad/s) for 2.0s")
    tester.send_cmd(1.20, 0.0, 2.0)

    # 4. Sprint Speed Forward (1.50 m/s)
    print("\n---> SPRINT SPEED 1.50 m/s FORWARD: CMD (linear.x=1.50 m/s, angular.z=0.00 rad/s) for 2.0s")
    tester.send_cmd(1.50, 0.0, 2.0)

    # 5. Fast Turning Left (0.40 rad/s)
    print("\n---> FAST TURN 0.40 rad/s LEFT: CMD (linear.x=0.00 m/s, angular.z=0.40 rad/s) for 2.0s")
    tester.send_cmd(0.00, 0.40, 2.0)

    # 6. Fast Turning Right (0.60 rad/s)
    print("\n---> FAST TURN 0.60 rad/s RIGHT: CMD (linear.x=0.00 m/s, angular.z=-0.60 rad/s) for 2.0s")
    tester.send_cmd(0.00, -0.60, 2.0)

    # 7. High-Speed Curved Arc (0.80 m/s, 0.30 rad/s)
    print("\n---> HIGH-SPEED CURVED ARC: CMD (linear.x=0.80 m/s, angular.z=0.30 rad/s) for 2.0s")
    tester.send_cmd(0.80, 0.30, 2.0)

    # 8. Immediate Full Stop
    print("\n---> FULL STOP: CMD (linear.x=0.00 m/s, angular.z=0.00 rad/s) for 1.0s")
    tester.send_cmd(0.00, 0.0, 1.0)

    tester.destroy_node()
    rclpy.shutdown()
    print("\nHigh-Speed Mobility Test Suite Complete.")

if __name__ == '__main__':
    main()
