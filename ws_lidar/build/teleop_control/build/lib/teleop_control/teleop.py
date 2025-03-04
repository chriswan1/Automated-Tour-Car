import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import curses

class TeleopNode(Node):
    def __init__(self, stdscr):
        super().__init__('teleop_node')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.angle_publisher = self.create_publisher(Int32, '/steering_angle', 10)

        self.speed = 0.0
        self.steering_angle = 25  # Default straight position

        self.max_speed = 10.0
        self.min_speed = -10.0
        self.steering_limit_left = 40
        self.steering_limit_right = 10

        # Initialize curses
        self.stdscr = stdscr
        curses.curs_set(0)  # Hide cursor
        self.stdscr.nodelay(True)  # Non-blocking input
        curses.raw()  # Allows holding keys
        self.stdscr.timeout(100)  # Refresh rate

        self.run()

    def run(self):
        self.stdscr.addstr(0, 0, "Use W/S to move, A/D to turn, X to stop, Q to quit.")

        while rclpy.ok():
            key = self.stdscr.getch()
            if key != -1:  # If a key is detected
                self.process_key(chr(key))

            self.publish_velocity()
            self.publish_steering_angle()

    def process_key(self, key):
        """Handles key press logic"""
        if key == 'w':  # Move forward
            self.speed = self.max_speed
        elif key == 's':  # Move backward
            self.speed = self.min_speed
        elif key == 'a':  # Turn left (decrease angle)
            self.steering_angle = max(self.steering_angle - 1, self.steering_limit_right)
        elif key == 'd':  # Turn right (increase angle)
            self.steering_angle = min(self.steering_angle + 1, self.steering_limit_left)
        elif key == 'x':  # Full Stop
            self.speed = 0.0
            # self.steering_angle = 25  # Reset to straight
            self.publish_velocity()
            self.publish_steering_angle()
        elif key == 'q':  # Quit
            self.speed = 0.0
            self.publish_velocity()
            rclpy.shutdown()

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = 0.0  # No longer used
        self.vel_publisher.publish(msg)
        self.stdscr.addstr(2, 0, f"Speed: {self.speed:.2f}, Steering: {self.steering_angle}° ")

    def publish_steering_angle(self):
        msg = Int32()
        msg.data = self.steering_angle
        self.angle_publisher.publish(msg)

def curses_main(stdscr):
    rclpy.init()
    node = TeleopNode(stdscr)
    node.destroy_node()
    rclpy.shutdown()

def main():
    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()
