import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import numpy as np
from TurtleBot import rrt
import cv2

class RRTNode(Node):

    def __init__(self):
        super().__init__('rrt_node')
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_goal = self.create_subscription(Float64MultiArray, '/start_goal', self.start_goal_callback, 10)
        self.publisher_trajectory = self.create_publisher(Float64MultiArray, '/trajectory', 10)
        self.current_map = None
        self.resolution = None
        self.origin = None
        self.map_received = False

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.current_map = np.array(msg.data, dtype=np.int8).reshape(height, width)
        self.map_received = True
        self.get_logger().info('Map data has been received and processed.')

    def start_goal_callback(self, msg):
        if not self.map_received:
            self.get_logger().error('Map data has not been received yet')
            return

        x_start_real, y_start_real, x_goal_real, y_goal_real = msg.data
        self.get_logger().info(f'Received start and goal coordinates: {msg.data}')
        
        x_start_index = x_start_real #int(round((x_start_real - self.origin[0]) / self.resolution))
        y_start_index = y_start_real #int(round((y_start_real - self.origin[1]) / self.resolution))
        x_goal_index = x_goal_real# int(round((x_goal_real - self.origin[0]) / self.resolution))
        y_goal_index = y_goal_real# int(round((y_goal_real - self.origin[1]) / self.resolution))

        self.get_logger().info(f'Scaled goal coordinates: {x_goal_index, y_goal_index}')

        start = [x_start_index, y_start_index]
        goal = [x_goal_index, y_goal_index]
        self.get_logger().info(f'Calculating path from {start} to {goal}.')

        path, graph = rrt.find_path_RRT(start, goal, self.current_map)

        if path is not None:
            trajectory = Float64MultiArray()
            for point in path:
                trajectory.data.extend([point[0] * self.resolution + self.origin[0],
                                        point[1] * self.resolution + self.origin[1]])
            self.publisher_trajectory.publish(trajectory)
            self.get_logger().info(f'Trajectory published: {trajectory.data}')
        else:
            self.get_logger().error('RRT could not find a path')

        rrt.plot_path_on_map(self.current_map, path, start, goal)

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
