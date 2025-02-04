import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarDataPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        # Declare parameters for topics
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/processed_lidar')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Subscriber to the input topic
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.lidar_callback,
            10
        )

        # Publisher for processed lidar data
        self.lidar_publisher = self.create_publisher(
            LaserScan,
            output_topic,
            10
        )
        self.get_logger().info(f"Lidar Publisher Node started with input: {input_topic} and output: {output_topic}")

    def lidar_callback(self, msg: LaserScan):
        self.get_logger().info("Received LiDAR data")

        # Process ranges: Replace 'inf' with max range value
        processed_ranges = [r if r < float('inf') else msg.range_max for r in msg.ranges]
        msg.ranges = processed_ranges

        # Log statistics
        valid_ranges = [r for r in processed_ranges if r < float('inf')]
        closest_distance = min(valid_ranges) if valid_ranges else None
        farthest_distance = max(valid_ranges) if valid_ranges else None
        self.get_logger().info(f"Closest distance: {closest_distance}, Farthest distance: {farthest_distance}")

        # Publish the processed data
        self.lidar_publisher.publish(msg)
        self.get_logger().info("Published processed LiDAR data")

def main(args=None):
    rclpy.init(args=args)

    lidar_publisher_node = LidarDataPublisher()

    try:
        rclpy.spin(lidar_publisher_node)
    except KeyboardInterrupt:
        lidar_publisher_node.get_logger().info("Node terminated by user")
    finally:
        lidar_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

