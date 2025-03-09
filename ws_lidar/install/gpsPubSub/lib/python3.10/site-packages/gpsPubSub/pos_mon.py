#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
from sensor_msgs.msg import Imu

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        
        # Map resolution (meters per pixel)
        self.map_resolution = 0.03
        
        # Coordinate transformation parameters
        self.x_offset = 0
        self.y_offset = 0
        
        # Create TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create timer for checking position
        self.timer = self.create_timer(1.0, self.check_position)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
            
        # IMU tracking variables
        self.imu_position = [35.5, 7.74]  # Start at origin
        self.imu_velocity = [0.0, 0.0]
        self.imu_heading = 0.0  # Radians
        self.last_imu_time = None
        
        # Create timer for checking position
        self.timer = self.create_timer(1.0, self.check_position)
        
        self.get_logger().info('Position monitor started - tracking IMU-only position')
        # For filtering
        self.position_history_x = []
        self.position_history_y = []
        self.history_size = 10
        self.initial_yaw = None
        self.current_yaw = None
        self.yaw_history = []
        
        self.get_logger().info('Position monitor')
    def imu_callback(self, msg):
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Initialize time on first callback
        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return
        # Calculate time step
        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time
        if self.is_stationary(accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z):
            #self.imu_velocity[0] = 0.0
            #self.imu_velocity[1] = 0.0
            i=0
        else:

            self.imu_heading += gyro_z *dt
            self.imu_heading = math.atan2(math.sin(self.imu_heading),math.cos(self.imu_heading))
            #if self.imu_heading > 0.3:
                #print("turning left")
            #if self.imu_heading < -0.3:
                #print("turning right")
            #print(self.imu_heading)
            
            #if self.imu_heading > 180:
                #print("90 degrees left hit")
            #if self.imu_heading < 
            # Convert to world frame using current heading
            world_accel_x = accel_x * math.cos(self.imu_heading) - accel_y * math.sin(self.imu_heading)
            world_accel_y = accel_x * math.sin(self.imu_heading) + accel_y * math.cos(self.imu_heading)
            
        
        # Update velocity by integrating acceleration
            self.imu_velocity[0] += world_accel_x * dt
            self.imu_velocity[1] += world_accel_y * dt
        if self.is_stationaryac(accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z):
            world_accel_x = accel_x * math.cos(self.imu_heading) - accel_y * math.sin(self.imu_heading)
            world_accel_y = accel_x * math.sin(self.imu_heading) + accel_y * math.cos(self.imu_heading)     
        # Update velocity by integrating acceleration
            self.imu_velocity[0] += world_accel_x * dt
            self.imu_velocity[1] += world_accel_y * dt
        # Apply damping to prevent velocity drift
        damping = 0.98  # Adjust as needed
        print(self.imu_velocity[0])
        self.imu_velocity[0] *= damping
        self.imu_velocity[1] *= damping
        
        # Update position by integrating velocity
        self.imu_position[0] += self.imu_velocity[0] * dt
        self.imu_position[1] += self.imu_velocity[1] * dt
        
    
        
    #   Stationary Function:
    #	Indicates whether the robot is stationary
    # 
    def is_stationary(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        accel_magnitude = math.sqrt(accel_x**2+accel_y**2+(accel_z-9.81)**2)
        #print(accel_magnitude)
        #print(gyro_z)
        gyro_magnitude = math.sqrt(gyro_z**2)
        #print(gyro_magnitude)
        return accel_magnitude < 4

    def is_stationaryac(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        accel_magnitude = math.sqrt(accel_x**2+accel_y**2+(accel_z-9.81)**2)
        #print(accel_magnitude)
        #print(gyro_z)
        gyro_magnitude = math.sqrt(gyro_z**2)
        #print(gyro_magnitude)
        return accel_magnitude < 1.5

    def is_turning(self, yaw):
        return yaw < 0.5  
        
    #   Check Position Function:
    #   
    #     
    def check_position(self):
        try:
            # Look up transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
                
            # Extract position and orientation
            x_meters = transform.transform.translation.x
            y_meters = transform.transform.translation.y

            
            
            # Convert to pixels
            x_pixels_ros = int(x_meters / self.map_resolution)
            y_pixels_ros = int(y_meters / self.map_resolution)
            
            # Apply offsets
            x_final = x_pixels_ros + self.x_offset
            y_final = abs(y_pixels_ros) + abs(self.y_offset)
            
            
            # Add to history for filtering
            self.position_history_x.append(x_final)
            self.position_history_y.append(y_final)
            
            # Keep history at desired length
            if len(self.position_history_x) > self.history_size:
                self.position_history_x.pop(0)
                self.position_history_y.pop(0)
            
            # Calculate smoothed position
            if len(self.position_history_x) > 0:
                x_smoothed = sum(self.position_history_x) / len(self.position_history_x)
                y_smoothed = sum(self.position_history_y) / len(self.position_history_y)
            else:
                x_smoothed = x_final
                y_smoothed = y_final
            imu_x_pixels = int(self.imu_position[0] / self.map_resolution)
            imu_y_pixels = int(self.imu_position[1] / self.map_resolution)
            #Print positions
            #self.get_logger().info(f'Meters: X: {x_meters:.2f}, Y: {y_meters:.2f}')
            #self.get_logger().info(f'Pixels: X: {x_pixels_ros}, Y: {y_pixels_ros}')
            #self.get_logger().info(f'Final: X: {int(x_final)}, Y: {int(y_final)}')
            #self.get_logger().info(f'Filtered: X: {int(x_smoothed)}, Y: {int(y_smoothed)}')
            #self.get_logger().info(f'Orientation: Roll: {roll:.1f}, Pitch: {pitch:.1f}, Yaw: {yaw:.1f} degrees')
            self.get_logger().info(f'IMU Position - Meters: X: {self.imu_position[0]:.2f}, Y: {self.imu_position[1]:.2f} | Pixels: X: {imu_x_pixels}, Y: {imu_y_pixels}')   
        except TransformException as ex:
            self.get_logger().warn(f'Could not get position: {ex}')

def main():
    rclpy.init()
    node = PositionMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
