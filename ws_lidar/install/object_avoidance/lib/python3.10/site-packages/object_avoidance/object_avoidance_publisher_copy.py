import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from itertools import chain
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
        self.distance_threshold = 0.5  # (0.8 indoor, 1.2 outdoor)
        self.emergency_threshold = 0.2  # **If obstacle < 0.5m, reverse**
        self.reverse_time = 1.0  # **Duration to reverse**
        self.reverse_speed = -0.5  # **Speed while reversing**
        self.max_speed = 1.0  # **Max speed for forward motion**

        self.get_logger().info(f"Thresholds set: distance={self.distance_threshold}, emergency={self.emergency_threshold}, max_speed={self.max_speed}")

    def lidar_callback(self, data):
        self.get_logger().info("Received LiDAR scan data")
        self.number_of_readings = len(data.ranges)
        self.range_max = data.range_max
        ranges = [r if r != float("inf") else self.range_max for r in data.ranges]

        # **Define front-facing obstacle detection zones**
        range_left1_avg = sum(ranges[self.number_of_readings*11//12:self.number_of_readings]) / (self.number_of_readings//12) #330-360
        range_left2_avg = sum(ranges[self.number_of_readings*5//6:self.number_of_readings*11//12]) / (self.number_of_readings//12) #300-330
        range_left3_avg = sum(ranges[self.number_of_readings*3//4:self.number_of_readings*5//6]) / (self.number_of_readings//12) #270-300
        
        range_right1_avg = sum(ranges[0:self.number_of_readings//12]) / (self.number_of_readings//12) #0-30
        range_right2_avg = sum(ranges[self.number_of_readings//12:self.number_of_readings//6]) / (self.number_of_readings//12) #30-60
        range_right3_avg = sum(ranges[self.number_of_readings//6:self.number_of_readings//4]) / (self.number_of_readings//12) #60-90

        cmd_vel = Twist()

        # **Check if any FRONT obstacle is within the EMERGENCY THRESHOLD**
        if (
            range_left1_avg < self.emergency_threshold or
            range_left2_avg < self.emergency_threshold or
            range_left3_avg < self.emergency_threshold or
            range_right1_avg < self.emergency_threshold or
            range_right2_avg < self.emergency_threshold or
            range_right3_avg < self.emergency_threshold
        ):
            self.get_logger().warn(f"🚨 Emergency Stop & Reverse! Front obstacle detected at {round(min(range_left1_avg, range_right1_avg), 2)}m")

            # **Move backward**
            cmd_vel.linear.x = self.reverse_speed
            cmd_vel.angular.z = 0.0
            self.twist_pub.publish(cmd_vel)
            self.get_logger().info(f"Reversing for {self.reverse_time}s | Speed: {self.reverse_speed}")

            # **Wait while reversing**
            time.sleep(self.reverse_time)

            # **Stop after reversing**
            cmd_vel.linear.x = 0.0
            self.twist_pub.publish(cmd_vel)
            self.get_logger().info("Stopped after reversing.")

            return  # **Exit to avoid publishing unnecessary movement commands**

        ## **Normal Obstacle Avoidance**
        if range_left1_avg < self.distance_threshold:  
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = -self.max_speed * 1.5
            self.get_logger().info(f"Turn left 1 - Obstacle at {round(range_left1_avg, 2)}m")
        elif range_left2_avg < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = -self.max_speed * 1.5
            self.get_logger().info(f"Turn left 2 - Obstacle at {round(range_left2_avg, 2)}m")
        elif range_left3_avg < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = -self.max_speed * 1.5
            self.get_logger().info(f"Turn left 3 - Obstacle at {round(range_left3_avg, 2)}m")
        elif range_right1_avg < self.distance_threshold:  
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = self.max_speed * 1.5
            self.get_logger().info(f"Turn right 1 - Obstacle at {round(range_right1_avg, 2)}m")
        elif range_right2_avg < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed / 2
            cmd_vel.angular.z = self.max_speed * 1.5
            self.get_logger().info(f"Turn right 2 - Obstacle at {round(range_right2_avg, 2)}m")
        elif range_right3_avg < self.distance_threshold:
            cmd_vel.linear.x = self.max_speed
            cmd_vel.angular.z = self.max_speed * 1.5
            self.get_logger().info(f"Turn right 3 - Obstacle at {round(range_right3_avg, 2)}m")
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
