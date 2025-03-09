#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
import utm  # pip install utm

class GpsToOdomNode(Node):
    def __init__(self):
        super().__init__('gps_to_odom')
        
        # Get parameters
        self.declare_parameter('datum_lat', 33.670248)
        self.declare_parameter('datum_lon', -117.828590)
        
        self.datum_lat = self.get_parameter('datum_lat').value
        self.datum_lon = self.get_parameter('datum_lon').value
        
        # Convert datum to UTM
        self.datum_easting, self.datum_northing, self.datum_zone_number, self.datum_zone_letter = utm.from_latlon(self.datum_lat, self.datum_lon)
        
        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
    def gps_callback(self, msg):
        if not self.is_valid_fix(msg):
            return
            
        # Convert GPS to UTM
        easting, northing, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        
        # Check if we're in the same UTM zone
        if zone_number != self.datum_zone_number or zone_letter != self.datum_zone_letter:
            self.get_logger().warn('GPS coordinates in different UTM zone from datum')
            return
            
        # Calculate relative position
        x = easting - self.datum_easting
        y = northing - self.datum_northing
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Identity orientation
        odom.pose.pose.orientation.w = 1.0
        
        # Set covariance (from GPS accuracy if available)
        position_covariance = 1.0
        if msg.position_covariance_type > 0:
            position_covariance = max(msg.position_covariance[0], msg.position_covariance[4])
        
        odom.pose.covariance[0] = position_covariance  # x
        odom.pose.covariance[7] = position_covariance  # y
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
    def is_valid_fix(self, msg):
        # Check for valid GPS fix
        if msg.status.status < 0:  # No fix
            return False
        return True

def main():
    rclpy.init()
    node = GpsToOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()