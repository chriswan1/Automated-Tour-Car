import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class MotionPlanner(Node):

    def __init__(self):
        super().__init__('motion_planner')
        
        # Subscriptions
        self.subscription_coords = self.create_subscription(Float64MultiArray, '/coords', self.coords_callback, 10)
        self.subscription_trajectory = self.create_subscription(Float64MultiArray, '/trajectory', self.trajectory_callback, 10)
        self.subscription_target_pose = self.create_subscription(Float64MultiArray, '/target_pose', self.target_pose_callback, 10)
        
        # Publishers
        self.publisher_start_goal = self.create_publisher(Float64MultiArray, '/start_goal', 10)
        self.publisher_reference_pose = self.create_publisher(Float64MultiArray, '/reference_pose', 10)

        # Position and path tracking
        self.current_position = None  # Initially unknown
        self.target_position = None
        self.trajectory = []
        self.current_trajectory_index = 0
        self.resolution = 1 / 14.29  # Converts pixels back to meters (0.07 meters per pixel)

    def coords_callback(self, msg):
        """ Updates the current position using GPS data. """
        self.current_position = [msg.data[0], msg.data[1]]
        self.get_logger().info(f'Updated current position: {self.current_position}')

        # Check if we now have both positions to compute the path
        if self.target_position is not None:
            self.send_start_goal()

    def target_pose_callback(self, msg):
        """ Receives the target position from external sources. """
        self.target_position = [msg.data[0], msg.data[1]]
        self.get_logger().info(f'Received new target position: {self.target_position}')

        # Check if we now have both positions to compute the path
        if self.current_position is not None:
            self.send_start_goal()

    def send_start_goal(self):
        """ Publishes start and goal positions for RRT pathfinding. """
        if self.current_position is None:
            self.get_logger().warn("Waiting for GPS position before computing path...")
            return

        if self.target_position is None:
            self.get_logger().warn("Waiting for target position before computing path...")
            return

        start_goal_msg = Float64MultiArray()
        start_goal_msg.data = [
            self.current_position[0], self.current_position[1],
            self.target_position[0], self.target_position[1]
        ]
        self.publisher_start_goal.publish(start_goal_msg)
        self.get_logger().info(f'Sent start goal: {start_goal_msg.data}')
        time.sleep(5)

    def trajectory_callback(self, msg):
        """ Receives and processes the RRT path. """
        self.trajectory = [(msg.data[i], msg.data[i+1]) for i in range(0, len(msg.data), 2)]
        self.get_logger().info(f'Parsed trajectory: {self.trajectory}')
        self.current_trajectory_index = 0
        self.send_next_reference_pose()

    def send_next_reference_pose(self):
        """ Sends the next waypoint to the PID controller. """
        if self.trajectory and self.current_trajectory_index < len(self.trajectory):
            next_pose = Float64MultiArray()
            next_pose.data = [
                self.trajectory[self.current_trajectory_index][0],
                self.trajectory[self.current_trajectory_index][1]
            ]
            self.publisher_reference_pose.publish(next_pose)
            self.get_logger().info(f'Sent reference pose: {next_pose.data[0]}, {next_pose.data[1]}]')
        else:
            self.get_logger().info("All waypoints reached, waiting for new target...")

    def process_trajectory(self):
        """ Ensures the car stays within 10 pixels of the planned path. """
        if not self.trajectory or self.current_trajectory_index >= len(self.trajectory) or self.current_position is None:
            return

        next_position = self.trajectory[self.current_trajectory_index]

        if self.is_close_to(next_position):
            self.current_trajectory_index += 1
            if self.current_trajectory_index < len(self.trajectory):
                self.send_next_reference_pose()
            else:
                self.get_logger().info("Reached final waypoint, waiting for new target...")
        else:
            self.get_logger().warn("Deviation detected! Recomputing path...")
            self.send_start_goal()  # Trigger re-planning with RRT

    def is_close_to(self, position):
        """ Checks if the car is within 10 pixels of the path. """
        pixel_threshold = 10  # Define threshold in pixels
        distance = math.sqrt((self.current_position[0] - position[0]) ** 2 + 
                             (self.current_position[1] - position[1]) ** 2)
        return distance < pixel_threshold * self.resolution  # Convert pixels to meters

def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanner()
    rclpy.spin(motion_planner)
    motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
