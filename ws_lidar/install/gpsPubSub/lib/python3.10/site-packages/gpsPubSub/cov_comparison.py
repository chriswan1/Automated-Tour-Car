#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL, ToLL
import numpy as np

class SimpleGPSComparator(Node):
    def __init__(self):
        super().__init__('simple_gps_comparator')
        
        # QoS profile
        qos = QoSProfile(depth=10)
        
        # Subscribe to raw GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # Raw GPS topic
            self.gps_callback,
            qos
        )
        
        # Subscribe to EKF-filtered odometry
        self.ekf_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',  # EKF output topic
            self.ekf_callback,
            qos
        )
        
        # Create service clients for coordinate transformations
        self.to_ll_client = self.create_client(ToLL, 'toLL')
        
        # Store latest positions
        self.latest_gps_lat = None
        self.latest_gps_lon = None
        self.latest_ekf_lat = None
        self.latest_ekf_lon = None
        
        # Timer for comparison
        self.timer = self.create_timer(1.0, self.compare_positions)
        
        self.get_logger().info('Simple GPS comparator started')
    
    def gps_callback(self, msg):
        """Process GPS message"""
        self.latest_gps_lat = msg.latitude
        self.latest_gps_lon = msg.longitude
    
    def ekf_callback(self, msg):
        """Process EKF message and convert to lat/lon"""
        # Store EKF position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Convert EKF xyz to lat/lon
        if self.to_ll_client.service_is_ready():
            request = ToLL.Request()
            request.map_point.x = x
            request.map_point.y = y
            request.map_point.z = z
            
            future = self.to_ll_client.call_async(request)
            future.add_done_callback(self.to_ll_callback)
        else:
            if not hasattr(self, 'service_warning_printed'):
                self.get_logger().warn('ToLL service not available - cannot convert EKF position to lat/lon')
                self.service_warning_printed = True
    
    def to_ll_callback(self, future):
        """Process response from ToLL service"""
        try:
            response = future.result()
            self.latest_ekf_lat = response.ll_point.latitude
            self.latest_ekf_lon = response.ll_point.longitude
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def compare_positions(self):
        """Compare GPS and EKF positions"""
        if (self.latest_gps_lat is None or self.latest_gps_lon is None or 
            self.latest_ekf_lat is None or self.latest_ekf_lon is None):
            self.get_logger().info('Waiting for both GPS and EKF data...')
            return
        
        # Print comparison
        self.get_logger().info('======== Position Comparison ========')
        self.get_logger().info(f'Raw GPS: Lat={self.latest_gps_lat:.7f}, Lon={self.latest_gps_lon:.7f}')
        self.get_logger().info(f'EKF GPS: Lat={self.latest_ekf_lat:.7f}, Lon={self.latest_ekf_lon:.7f}')
        
        # Calculate difference
        lat_diff = abs(self.latest_gps_lat - self.latest_ekf_lat) * 111000  # approx meters
        lon_diff = abs(self.latest_gps_lon - self.latest_ekf_lon) * 111000 * np.cos(np.radians(self.latest_gps_lat))
        
        self.get_logger().info(f'Difference: Lat={lat_diff:.2f}m, Lon={lon_diff:.2f}m')
        self.get_logger().info('====================================')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGPSComparator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()