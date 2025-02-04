import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')

        # Declare parameter for the topic name
        self.declare_parameter('input_topic', '/processed_lidar')
        input_topic = self.get_parameter('input_topic').value

        # Subscribe to the input topic
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.lidar_callback,
            10
        )
        self.get_logger().info(f"Lidar Subscriber Node started, subscribing to: {input_topic}")

    def lidar_callback(self, msg: LaserScan):
        self.get_logger().info("Received processed LiDAR data")

        # Log the first few ranges
        self.get_logger().info(f"First 10 Ranges: {msg.ranges[:10]}")

        # Calculate and log statistics
        valid_ranges = [r for r in msg.ranges if r < float('inf')]
        closest_distance = min(valid_ranges) if valid_ranges else None
        farthest_distance = max(valid_ranges) if valid_ranges else None
        self.get_logger().info(f"Closest distance: {closest_distance}, Farthest distance: {farthest_distance}")

        # Count how many ranges are equal to the max range
        max_range_count = sum(1 for r in msg.ranges if r == msg.range_max)
        self.get_logger().info(f"Number of ranges equal to max range: {max_range_count}")

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber_node = LidarSubscriber()

    try:
        rclpy.spin(lidar_subscriber_node)
    except KeyboardInterrupt:
        lidar_subscriber_node.get_logger().info("Node terminated by user")
    finally:
        lidar_subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

