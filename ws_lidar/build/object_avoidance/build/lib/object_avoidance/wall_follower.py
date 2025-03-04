import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # Publishers & Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Constants
        self.target_distance = 0.3  # Desired distance from the wall (meters)
        self.safe_distance = 0.5  # Distance to trigger obstacle avoidance
        self.max_speed = 0.2  # Maximum forward speed
        self.max_turn_speed = 0.5  # Maximum turning speed

        # PID Parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.1  # Derivative gain

        # Tracking error history for PID
        self.last_error = 0.0
        self.integral = 0.0

        # Wall Side (None until first detection)
        self.wall_side = None  # 'left' or 'right'

        self.get_logger().info("Wall Follower Node Initialized")

    def scan_callback(self, msg):
        """
        Processes LiDAR scan data and controls the robot.
        """
        num_readings = len(msg.ranges)  # Total LiDAR readings

        # Extract readings for 90° (left) and 270° (right)
        index_90 = int(num_readings * 1/4)  # Left side
        index_270 = int(num_readings * 3/4)  # Right side

        min_left90 = msg.ranges[index_90] if msg.ranges[index_90] != float('inf') else self.safe_distance
        min_right270 = msg.ranges[index_270] if msg.ranges[index_270] != float('inf') else self.safe_distance

        front_distance = self.get_range(msg, angle=0)  # Front distance

        # Determine which wall to follow
        if self.wall_side is None:
            if min_left90 < self.safe_distance and min_right270 < self.safe_distance:
                self.wall_side = 'left' if min_left90 < min_right270 else 'right'
            elif min_left90 < self.safe_distance:
                self.wall_side = 'left'
            elif min_right270 < self.safe_distance:
                self.wall_side = 'right'

            if self.wall_side:
                self.get_logger().info(f"Following {self.wall_side} wall")

        # If no wall detected, move forward
        if self.wall_side is None:
            self.move_forward()
            return

        # Compute error based on the chosen wall
        error = self.target_distance - (min_left90 if self.wall_side == 'left' else min_right270)
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error

        # PID Control for steering
        turn_speed = self.kp * error + self.ki * self.integral + self.kd * derivative
        turn_speed = max(-self.max_turn_speed, min(self.max_turn_speed, turn_speed))

        # # Obstacle Avoidance: Stop or turn if something is too close in front
        # if front_distance < self.safe_distance:
        #     self.avoid_obstacle()
        #     return

        # Move forward while correcting direction
        twist_msg = Twist()
        twist_msg.linear.x = self.max_speed  # Constant forward speed
        twist_msg.angular.z = -turn_speed if self.wall_side == 'right' else turn_speed
        self.cmd_vel_publisher.publish(twist_msg)

        self.get_logger().info(f"{self.wall_side.capitalize()} Wall Distance: {min_left90 if self.wall_side == 'left' else min_right270:.2f}m | Error: {error:.2f} | Turn: {turn_speed:.2f}")

    def get_range(self, msg, angle):
        """
        Gets the distance measurement at a specific angle.
        """
        index = int((angle - msg.angle_min) / msg.angle_increment)
        if 0 <= index < len(msg.ranges):
            return msg.ranges[index]
        return float('inf')  # If out of range, assume no obstacle

    def move_forward(self):
        """
        Moves forward when no wall is detected.
        """
        twist_msg = Twist()
        twist_msg.linear.x = self.max_speed
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info("No wall detected. Moving forward.")

    def avoid_obstacle(self):
        """
        Stops the robot and turns away from an obstacle in front.
        """
        self.get_logger().info("Obstacle detected! Avoiding...")
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = self.max_turn_speed  # Turn in place
        self.cmd_vel_publisher.publish(twist_msg)
        time.sleep(1.0)  # Allow time to turn

    def stop_robot(self):
        """
        Stops the robot completely.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
