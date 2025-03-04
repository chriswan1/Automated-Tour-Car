import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pi
from std_msgs.msg import Float64MultiArray

class PIDController(Node):
    def __init__(self):
        super().__init__('PID_Controller')

        # PID Tuning Parameters
        self.Kp_linear = 0.5  
        self.Kd_linear = 0.1  
        self.Kp_angular = 2.0  
        self.Kd_angular = 0.1  

        # Position tracking
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.current_x = None  # Set to None initially
        self.current_y = None  
        self.prev_x = None
        self.prev_y = None
        self.current_theta = None

        # Subscriptions
        self.create_subscription(Float64MultiArray, '/reference_pose', self.reference_pose_callback, 10)
        self.create_subscription(Float64MultiArray, '/coords', self.gps_callback, 10)

        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoints = []

    def reference_pose_callback(self, msg):
        """ Receives next waypoint from motion planner. """
        self.waypoints.append((msg.data[0], msg.data[1]))
        if len(self.waypoints) == 1:  
            self.x_goal, self.y_goal = self.waypoints[0]
            self.get_logger().info(f'Received new goal: ({self.x_goal}, {self.y_goal})')

    def gps_callback(self, msg):
        """ Updates the current position using GPS. """
        self.prev_x, self.prev_y = self.current_x, self.current_y
        self.current_x, self.current_y = msg.data[0], msg.data[1]

        if self.prev_x is not None and self.prev_y is not None:
            self.current_theta = atan2(self.current_y - self.prev_y, self.current_x - self.prev_x)

        self.move_to_goal()

    def move_to_goal(self):
        """ Moves towards the next waypoint while ensuring smooth motion. """
        if not self.waypoints or self.current_x is None:
            return

        self.error_position = sqrt((self.x_goal - self.current_x)**2 + (self.y_goal - self.current_y)**2)
        if self.current_theta is None:
            return

        # Compute heading error
        desired_angle = atan2(self.y_goal - self.current_y, self.x_goal - self.current_x)
        self.error_angle = desired_angle - self.current_theta

        # Normalize angle
        while self.error_angle > pi:
            self.error_angle -= 2 * pi
        while self.error_angle < -pi:
            self.error_angle += 2 * pi

        # Generate control signals
        move = Twist()
        move.linear.x = min(self.Kp_linear * self.error_position, 1.0)
        move.angular.z = self.Kp_angular * self.error_angle
        self.vel_pub.publish(move)

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
