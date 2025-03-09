#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
import smbus

class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')
        
        # Create publisher
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        # Parameters
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('frequency', 100.0)  # Hz
        self.declare_parameter('i2c_bus', 0)
        self.declare_parameter('i2c_address', 0x69)  # Default MPU6050 address
        
        self.frame_id = self.get_parameter('frame_id').value
        frequency = self.get_parameter('frequency').value
        bus_number = self.get_parameter('i2c_bus').value
        self.address = self.get_parameter('i2c_address').value
        
        # Initialize I2C bus
        try:
            self.bus = smbus.SMBus(bus_number)
            
            # Wake up MPU6050
            self.bus.write_byte_data(self.address, 0x6B, 0)
            
            # Configure accelerometer (±2g)
            self.bus.write_byte_data(self.address, 0x1C, 0)
            
            # Configure gyroscope (±250°/s)
            self.bus.write_byte_data(self.address, 0x1B, 0)
            
            self.get_logger().info('MPU6050 (GY-521) initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MPU6050: {str(e)}')
            raise
        
        # Create timer for periodic publishing
        timer_period = 1.0 / frequency
        self.timer = self.create_timer(timer_period, self.publish_imu_data)
        
        # For complementary filter
        self.last_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Constants
        self.G = 9.80665  # Gravitational acceleration in m/s²
        self.DEG_TO_RAD = math.pi / 180.0
        self.COMPLEMENTARY_FILTER_ALPHA = 0.98  # Filter coefficient
    
    def read_word(self, reg):
        """Read a word from the MPU6050 (two bytes)"""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) + low
        if value > 32767:
            value = value - 65536
        return value
    
    def read_accel_data(self):
        """Read accelerometer data from MPU6050"""
        x = self.read_word(0x3B) / 15000.0  # for ±2g
        y = self.read_word(0x3D) / 15000.0
        z = self.read_word(0x3F) / 15000.0
        
        # Convert to m/s²
        return [x * self.G, y * self.G, z * self.G]
    
    def read_gyro_data(self):
        """Read gyroscope data from MPU6050"""
        x = self.read_word(0x43) / 131.0  # for ±250°/s
        y = self.read_word(0x45) / 131.0
        z = self.read_word(0x47) / 131.0
        
        # Convert to rad/s
        return [x * self.DEG_TO_RAD, y * self.DEG_TO_RAD, z * self.DEG_TO_RAD]
    
    def complementary_filter(self, accel, gyro, dt):
        """Apply complementary filter to get roll and pitch estimates"""
        # Calculate roll and pitch from accelerometer (in radians)
        accel_x, accel_y, accel_z = accel
        roll_acc = math.atan2(accel_y, accel_z)
        pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        
        # Integrate gyroscope data
        gyro_x, gyro_y, gyro_z = gyro
        self.roll = self.COMPLEMENTARY_FILTER_ALPHA * (self.roll + gyro_x * dt) + \
                   (1 - self.COMPLEMENTARY_FILTER_ALPHA) * roll_acc
        self.pitch = self.COMPLEMENTARY_FILTER_ALPHA * (self.pitch + gyro_y * dt) + \
                    (1 - self.COMPLEMENTARY_FILTER_ALPHA) * pitch_acc
        
        # Yaw cannot be determined from accelerometer, only use gyro
        self.yaw += gyro_z * dt

        return self.roll, self.pitch, self.yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0, 0, 0, 0]
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w
	
  
        return q


    
    def publish_imu_data(self):
        try:
            # Get current time
            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            
            # Read raw data
            accel = self.read_accel_data()  # in m/s²
            gyro = self.read_gyro_data()  # in rad/s
            
            # Apply complementary filter to get orientation
            roll, pitch, yaw = self.complementary_filter(accel, gyro, dt)
            
            # Convert to quaternion
            quaternion = self.euler_to_quaternion(roll, pitch, yaw)
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Set orientation
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            # Set angular velocity
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
            print(accel[0], accel[1], accel[2])
            # Set covariance matrices
            orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            acceleration_covariance = [0.02, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.02]
            
            imu_msg.orientation_covariance = orientation_covariance
            imu_msg.angular_velocity_covariance = velocity_covariance
            imu_msg.linear_acceleration_covariance = acceleration_covariance
            
            # Publish the message
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.get_logger().info('Shutting down MPU6050 publisher')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
