import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurnTestNode(Node):
    def __init__(self):
        super().__init__('turn_test_publisher')
        self.get_logger().info("Turn Test Publisher Node Started")

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(2)  # Allow subscriber to start

        self.turn_90_degrees()

    def turn_90_degrees(self):
        """Publishes a /cmd_vel command to turn the robot 90° to the right."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0  # No forward motion
        cmd_vel.angular.z = -1.5  # More rotation for carpet resistance

        turn_duration = 1.8  # Increased slightly from 1.5 to compensate for carpet
        self.get_logger().info("Turning 90 degrees to the right")

        # Publish turn command for set duration
        start_time = time.time()
        while time.time() - start_time < turn_duration:
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.1)

        # Stop rotation
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info("Turn complete, stopping rotation.")

        # ✅ **Move forward at lower speed (1000 PWM instead of 2000)**
        cmd_vel.linear.x = 0.5  # Lower speed to prevent excessive forward motion
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info("Moving forward after turn")

        time.sleep(1.0)  # Move forward briefly
        cmd_vel.linear.x = 0.0  # Stop movement
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info("Test complete, stopping motion.")

def main(args=None):
    rclpy.init(args=args)
    node = TurnTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
