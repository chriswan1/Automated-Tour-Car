import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import math

class AutoDecisionNode(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.get_logger().info("Obstacle Avoidance Node Started")

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_mode', 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to the /scan topic
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info("Subscribed to /scan topic")

        # Thresholds
        self.distance_threshold = 0.8  # Start avoiding at 0.8m
        self.emergency_threshold = 0.3  # Reverse if obstacle < 0.3m
        self.reverse_time = 2.0  # Reverse duration
        self.reverse_speed = -10.0  # Reverse speed
        self.max_speed = 10.0  # Max forward speed

        # Turn tracking
        self.current_turn_angle = 0  # Tracks total turn angle
        self.turning_away = False  # Whether turning away from obstacle
        self.turning_back = False  # Whether returning towards obstacle

        self.get_logger().info(f"Thresholds set: distance={self.distance_threshold}, emergency={self.emergency_threshold}, max_speed={self.max_speed}")

    def lidar_callback(self, data):
        self.get_logger().info("Received LiDAR scan data")
        self.number_of_readings = len(data.ranges)
        self.range_max = data.range_max
        ranges = [r if r != float("inf") else self.range_max for r in data.ranges]

        # Find the minimum distance in each front-facing region
        min_left = min(ranges[self.number_of_readings*3//4:self.number_of_readings])  # 270°-360°
        min_right = min(ranges[0:self.number_of_readings//4])  # 0°-90°

        cmd_vel = Twist()

        # Emergency Reverse
        if min_left < self.emergency_threshold or min_right < self.emergency_threshold:
            self.get_logger().warn(f"Emergency Reverse! Obstacle detected at {round(min(min_left, min_right), 2)}m")
            cmd_vel.linear.x = self.reverse_speed
            cmd_vel.angular.z = 0.0
            self.twist_pub.publish(cmd_vel)
            time.sleep(self.reverse_time)
            self.get_logger().info("Finished reversing. Resuming obstacle avoidance.")
            return  # Exit function to avoid further calculations

        # Detect obstacle and start turning away
        if min_left < self.distance_threshold and not self.turning_away and not self.turning_back:
            self.get_logger().info(f"Obstacle detected at {round(min_left, 2)}m - Turning Right")
            self.current_turn_angle = 0  # Reset turn angle
            self.turning_away = True
            cmd_vel.angular.z = -self.max_speed  # Turn Right
            cmd_vel.linear.x = self.max_speed / 2  # Slow movement while turning
        elif min_right < self.distance_threshold and not self.turning_away and not self.turning_back:
            self.get_logger().info(f"Obstacle detected at {round(min_right, 2)}m - Turning Left")
            self.current_turn_angle = 0  # Reset turn angle
            self.turning_away = True
            cmd_vel.angular.z = self.max_speed  # Turn Left
            cmd_vel.linear.x = self.max_speed / 2

        # Track how much the car has turned
        if self.turning_away:
            self.current_turn_angle += cmd_vel.angular.z * 0.1  # Accumulate turn angle
            if abs(self.current_turn_angle) >= 90:  # If turned ~90°
                self.get_logger().info("Reached perpendicular, now turning back")
                self.turning_away = False
                self.turning_back = True  # Start turning back

        # Turn back toward obstacle to check if it's still there
        if self.turning_back:
            cmd_vel.angular.z = self.max_speed if self.current_turn_angle < 0 else -self.max_speed  # Turn opposite
            cmd_vel.linear.x = self.max_speed / 2  # Move forward slowly
            self.current_turn_angle -= cmd_vel.angular.z * 0.1  # Reduce accumulated angle

            # If we return to straight position, stop turning back
            if abs(self.current_turn_angle) <= 5:
                self.get_logger().info("Back to original direction")
                self.turning_back = False

                # Check if the obstacle is still there
                if min_left < self.distance_threshold or min_right < self.distance_threshold:
                    self.get_logger().info("Obstacle still there, re-routing")
                    self.turning_away = True  # Restart avoidance
                else:
                    self.get_logger().info("Obstacle cleared, moving forward")

        # If no obstacle, move forward normally
        if not self.turning_away and not self.turning_back:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = 0.0
            self.get_logger().info("Moving Forward")

        # Publish the velocity command
        self.twist_pub.publish(cmd_vel)
        self.get_logger().info(f"Published /cmd_vel: linear.x={cmd_vel.linear.x}, angular.z={cmd_vel.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = AutoDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
