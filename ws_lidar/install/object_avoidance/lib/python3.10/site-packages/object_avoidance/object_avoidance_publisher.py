import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class AutoDecisionNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.get_logger().info("Obstacle Avoidance Node Started")

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_mode', 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to the /scan topic
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info("Subscribed to /scan topic")

        # **Thresholds**
        self.distance_threshold = 0.8  # Start avoiding at 0.8m
        self.emergency_threshold = 0.5  # Reverse if obstacle < 0.5m
        self.reverse_time = 2.0  # Reverse for 1 second
        self.reverse_speed = -0.5  # Reverse speed
        self.max_speed = 1.0  # Max forward speed

        self.get_logger().info(f"Thresholds set: distance={self.distance_threshold}, emergency={self.emergency_threshold}, max_speed={self.max_speed}")

    def lidar_callback(self, data):
        self.get_logger().info("Received LiDAR scan data")
        self.number_of_readings = len(data.ranges)
        self.range_max = data.range_max
        ranges = [r if r != float("inf") else self.range_max for r in data.ranges]

        # **Find the minimum distance in each front-facing zone**
        min_left1 = min(ranges[self.number_of_readings*11//12:self.number_of_readings])  # 330°-360°
        min_left2 = min(ranges[self.number_of_readings*5//6:self.number_of_readings*11//12])  # 300°-330°
        min_left3 = min(ranges[self.number_of_readings*3//4:self.number_of_readings*5//6])  # 270°-300°
        
        min_right1 = min(ranges[0:self.number_of_readings//12])  # 0°-30°
        min_right2 = min(ranges[self.number_of_readings//12:self.number_of_readings//6])  # 30°-60°
        min_right3 = min(ranges[self.number_of_readings//6:self.number_of_readings//4])  # 60°-90°

        cmd_vel = Twist()

        # **Step 1: PRIORITY EMERGENCY REVERSING**
        if (
            min_left1 < self.emergency_threshold or
            min_left2 < self.emergency_threshold or
            min_left3 < self.emergency_threshold or
            min_right1 < self.emergency_threshold or
            min_right2 < self.emergency_threshold or
            min_right3 < self.emergency_threshold
        ):
            self.get_logger().warn(f"🚨 Emergency Reverse! Obstacle detected at {round(min(min_left1, min_right1), 2)}m")

            # **Move backward**
            cmd_vel.linear.x = self.reverse_speed  # Reverse speed (-0.5)
            cmd_vel.angular.z = 0.0  # No turning while reversing
            self.twist_pub.publish(cmd_vel)
            self.get_logger().info(f"Reversing for {self.reverse_time}s | Speed: {self.reverse_speed}")

            # **Wait while reversing**
            time.sleep(self.reverse_time)

            # **Continue normal avoidance after reversing**
            self.get_logger().info("Finished reversing. Resuming obstacle avoidance.")

        # **Step 2: NORMAL OBSTACLE AVOIDANCE (Turn if needed)**
        if min_left1 < self.distance_threshold:  
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = -self.max_speed * 1.0  # Adjusted for softer turns
            self.get_logger().info(f"Turn left 1 - Obstacle at {round(min_left1, 2)}m")
        elif min_left2 < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = -self.max_speed * 1.0
            self.get_logger().info(f"Turn left 2 - Obstacle at {round(min_left2, 2)}m")
        elif min_left3 < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = -self.max_speed * 1.0
            self.get_logger().info(f"Turn left 3 - Obstacle at {round(min_left3, 2)}m")
        elif min_right1 < self.distance_threshold:  
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = self.max_speed * 1.0
            self.get_logger().info(f"Turn right 1 - Obstacle at {round(min_right1, 2)}m")
        elif min_right2 < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = self.max_speed * 1.0
            self.get_logger().info(f"Turn right 2 - Obstacle at {round(min_right2, 2)}m")
        elif min_right3 < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = self.max_speed * 1.0
            self.get_logger().info(f"Turn right 3 - Obstacle at {round(min_right3, 2)}m")
        else:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = 0.0
            self.get_logger().info("Moving Forward")

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
