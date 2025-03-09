import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import numpy as np
import utm

class GPSCovCalc(Node):
    def __init__(self):
        super().__init__('gps_covariance_calculator')
        self.subscription = self.create_subscription(
            NavSatFix, 'odom0', self.gps_callback, 10)
        self.lat_values = []
        self.lon_values = []
        self.alt_values = []
        self.x_values = []
        self.y_values = []
        self.z_values = []
        self.start_time = self.get_clock().now()
        self.collection_time = rclpy.duration.Duration(seconds=600)
        print("Iam here")
        # For UTM conversion
        self.utm_zone_number = None
        self.utm_zone_letter = None
        
        # For local origin
        self.origin_east = None
        self.origin_north = None
        
        self.reference_lat = 33.670137
        self.reference_lon = -117.828210
        east, north, self.utm_zone_number, self.utm_zone_letter = utm.from_latlon(
            self.reference_lat, self.reference_lon)
        self.origin_east = east
        self.origin_north = north

    def gps_callback(self, msg):
        if (self.get_clock().now() - self.start_time) > self.collection_time:
            self.calculate_covariance()
            rclpy.shutdown()
            return
        
        # Store raw lat/lon/alt
        self.lat_values.append(msg.latitude)
        self.lon_values.append(msg.longitude)
        self.alt_values.append(msg.altitude)
        # Convert to UTM coordinates
        east, north, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        
        # Store the zone info and origin from first reading
        if self.utm_zone_number is None:
            self.utm_zone_number = zone_number
            self.utm_zone_letter = zone_letter
            
            # Set the first point as origin if not using predefined reference
            if self.origin_east is None:
                self.origin_east = east
                self.origin_north = north
                self.get_logger().info(f'Setting origin at UTM coordinates: E:{east}, N:{north}')
        
        # Verify we're still in the same UTM zone
        if zone_number == self.utm_zone_number and zone_letter == self.utm_zone_letter:
            # Convert to local coordinates relative to origin
            local_x = east - self.origin_east
            local_y = north - self.origin_north
            local_z = msg.altitude
            
            self.x_values.append(local_x)
            self.y_values.append(local_y)
            self.z_values.append(local_z)
        else:
            self.get_logger().warning(f'GPS reading in different UTM zone')

    def calculate_covariance(self):
        # Calculate variance directly using NumPy
        if self.x_values:
            x_var = np.var(self.x_values)
            y_var = np.var(self.y_values)
            z_var = np.var(self.z_values)
            
            self.get_logger().info('GPS position variance (x,y,z): [{:.6f}, {:.6f}, {:.6f}]'.format(
                x_var, y_var, z_var))

def main(args=None):
    rclpy.init(args=args)
    gps_messages = GPSCovCalc()
    rclpy.spin(gps_messages)
    rclpy.shutdown()


if __name__ == '__main__':
    main()