#!/usr/bin/env python3

"""
Simple Binary Map Localizer for ROS2

This is a streamlined version for robot localization using a binary map.
It snaps GPS positions to the nearest road on the map without requiring IMU data.

Author: Claude
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import pyproj
import os
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray


class UTMConverter:
    """
    Handles conversion between GPS (lat, lon) and UTM coordinates.
    """
    def __init__(self, zone=None, northern=True, ref_lat=None, ref_lon=None):
        """Initialize with UTM zone if known."""
        self.zone = zone
        self.northern = northern
        self.proj_utm = None
        self.proj_latlon = pyproj.Proj(proj='latlong', datum='WGS84')
        
        # Reference point for local coordinates
        self.ref_lat = ref_lat
        self.ref_lon = ref_lon
        self.ref_easting = None
        self.ref_northing = None
        
        # If reference is provided, initialize UTM projection
        if ref_lat is not None and ref_lon is not None:
            self._initialize_utm_proj(ref_lat, ref_lon)
            self.ref_easting, self.ref_northing = self.gps_to_utm(ref_lat, ref_lon)
        
    def _initialize_utm_proj(self, lat, lon):
        """Initialize the UTM projection based on a reference point"""
        if self.zone is None:
            # Calculate UTM zone from longitude
            self.zone = int((lon + 180) / 6) + 1
        
        # Determine hemisphere from latitude if not specified
        if lat >= 0:
            self.northern = True
        else:
            self.northern = False
            
        # Create the projection
        hemisphere = 'north' if self.northern else 'south'
        self.proj_utm = pyproj.Proj(proj='utm', zone=self.zone, datum='WGS84', hemisphere=hemisphere)
        
    def gps_to_utm(self, lat, lon):
        """Convert GPS coordinates to UTM"""
        if self.proj_utm is None:
            self._initialize_utm_proj(lat, lon)
            
        # Convert to UTM
        east, north = pyproj.transform(self.proj_latlon, self.proj_utm, lon, lat)
        return east, north
    
    def gps_to_local(self, lat, lon):
        """Convert GPS coordinates directly to local coordinates."""
        easting, northing = self.gps_to_utm(lat, lon)
        
        # Calculate local coordinates relative to reference point
        x = easting - self.ref_easting
        y = northing - self.ref_northing
        
        return x, y
        
    def get_zone_info(self):
        """Return information about the UTM zone being used"""
        if self.proj_utm is None:
            return "UTM projection not initialized"
            
        hemisphere = "Northern" if self.northern else "Southern"
        return f"UTM Zone {self.zone}, {hemisphere} Hemisphere"


class BinaryMapLocalizer:
    """
    A simple localizer using a binary map where white (1) indicates road
    and black (0) indicates off-road areas.
    """
    
    def __init__(self, map_file, resolution=0.1, origin=(0, 0)):
        """Initialize with a binary map."""
        # Check if map file exists
        if not os.path.exists(map_file):
            raise FileNotFoundError(f"Map file not found: {map_file}")
            
        # Load the map from file
        self.map = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        if self.map is None:
            raise ValueError(f"Could not load map from {map_file}")
            
        # Binarize the map if it's not already binary
        _, self.binary_map = cv2.threshold(self.map, 127, 1, cv2.THRESH_BINARY)
            
        self.resolution = resolution
        self.origin = origin
        
        # Find all road pixels for nearest point calculation
        self.road_pixels = np.argwhere(self.binary_map == 1)  # (y, x) format
        if len(self.road_pixels) == 0:
            raise ValueError("No road pixels found in map! The map must contain white pixels for roads.")
            
        self.height, self.width = self.binary_map.shape
        
    def world_to_map(self, world_point):
        """Convert a world coordinate (meters) to map coordinates (pixels)."""
        # Adjust for the origin offset and convert from meters to pixels
        x_pixel = int((world_point[0] - self.origin[0]) / self.resolution)
        y_pixel = int((world_point[1] - self.origin[1]) / self.resolution)
        
        # Ensure the pixel is within map bounds
        x_pixel = max(0, min(x_pixel, self.width - 1))
        y_pixel = max(0, min(y_pixel, self.height - 1))
        
        return (x_pixel, y_pixel)
        
    def map_to_world(self, map_point):
        """Convert a map coordinate (pixels) to world coordinates (meters)."""
        x_world = map_point[0] * self.resolution + self.origin[0]
        y_world = map_point[1] * self.resolution + self.origin[1]
        
        return (x_world, y_world)
        
    def is_on_road(self, world_point):
        """Check if a point is on the road."""
        map_point = self.world_to_map(world_point)
        return self.binary_map[map_point[1], map_point[0]] == 1
        
    def localize(self, gps_position, covariance=None, previous_position=None):
        """
        Localize the robot given a GPS reading by snapping to the nearest road point.
        """
        # Convert GPS position to map coordinates
        map_point = self.world_to_map(gps_position)
        x_pixel, y_pixel = map_point
        
        # Check if the point is already on the road
        if self.binary_map[y_pixel, x_pixel] == 1:
            return gps_position, True, 1.0, False
        
        # Find the nearest road pixel using direct distance calculation
        min_dist = float('inf')
        closest_road_pixel = None
        
        for road_pixel in self.road_pixels:
            # road_pixel is (y, x)
            road_y, road_x = road_pixel
            dist = ((road_x - x_pixel)**2 + (road_y - y_pixel)**2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                closest_road_pixel = (road_x, road_y)
        
        # Convert back to world coordinates
        closest_world_point = self.map_to_world(closest_road_pixel)
        
        # Check for outliers if we have previous position
        is_outlier = False
        if previous_position is not None:
            dx = gps_position[0] - previous_position[0]
            dy = gps_position[1] - previous_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Outlier detection - adjust these parameters for your robot
            max_reasonable_speed = 5.0  # meters per second
            time_since_last_update = 0.1  # seconds (assume 10Hz update rate)
            max_reasonable_distance = max_reasonable_speed * time_since_last_update
            
            # If covariance available, adjust based on uncertainty
            if covariance is not None:
                # Extract standard deviation
                std_dev = math.sqrt(max(covariance[0, 0], covariance[1, 1]))
                # Allow more movement if uncertainty is high
                max_reasonable_distance = max(max_reasonable_distance, 3 * std_dev)
            
            if distance > max_reasonable_distance:
                is_outlier = True
        
        # Calculate distance to road and confidence
        distance_to_road = min_dist * self.resolution
        confidence = max(0.0, 1.0 - (distance_to_road / 10.0))  # 10 meters -> 0 confidence
        
        return closest_world_point, False, confidence, is_outlier


class SimpleLocalizationNode(Node):
    """
    ROS2 node that subscribes to GPS data and publishes corrected positions.
    """
    def __init__(self, 
                 map_file,
                 resolution=0.1, 
                 origin=(0, 0), 
                 initial_position=(0, 0),
                 ref_lat=33.6448461, 
                 ref_lon=-117.840944, 
                 utm_zone=11):
        """Initialize the ROS2 node with a binary map."""
        super().__init__('simple_map_localizer')
        
        # Check for required parameters
        if ref_lat is None or ref_lon is None:
            self.get_logger().error("Reference GPS coordinates (ref_lat, ref_lon) must be provided!")
            raise ValueError("Missing required parameters: ref_lat, ref_lon")
        
        # Set up the binary map localizer
        try:
            self.localizer = BinaryMapLocalizer(map_file=map_file, resolution=resolution, origin=origin)
            self.get_logger().info(f"Loaded map from {map_file}")
            self.get_logger().info(f"Map size: {self.localizer.width}x{self.localizer.height} pixels")
            self.get_logger().info(f"Road pixels: {len(self.localizer.road_pixels)}")
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {str(e)}")
            raise
        
        # Set up UTM converter for GPS coordinates
        self.utm_converter = UTMConverter(zone=utm_zone, ref_lat=ref_lat, ref_lon=ref_lon)
        
        # Store initial position and tracking variables
        self.current_position = initial_position
        self.last_position = None
        self.position_covariance = np.eye(2) * 1.0  # Default covariance (1m std dev)
        self.consecutive_outliers = 0
        self.max_outliers_warning = 5
        
        # If initial position is provided, snap it to the road
        if self.current_position is not None:
            snapped_position, on_road, confidence, _ = self.localizer.localize(self.current_position)
            self.current_position = snapped_position
            
            # Get pixel coordinates for logging
            map_pos = self.localizer.world_to_map(snapped_position)
            
            status = "on road" if on_road else f"snapped to road from ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})"
            self.get_logger().info(
                f"Initial position set: "
                f"World=({snapped_position[0]:.2f}, {snapped_position[1]:.2f})m, "
                f"Pixel=({map_pos[0]}, {map_pos[1]}) {status}"
            )
        
        self.publisher_coords = self.create_publisher(Float64MultiArray, '/coords', 10)
        
        # Create subscribers
        # NavSatFix GPS subscription
        self.navsat_subscription = self.create_subscription(
            NavSatFix,
            'gps',
            self.navsat_callback,
            10)
            
        # For standard ROS messages with covariance
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'robot_pose',
            self.pose_callback,
            10)
            
        # Output publishers
        self.corrected_publisher = self.create_publisher(
            String,
            'corrected_position',
            10)
            
        self.corrected_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            'corrected_pose',
            10)
            
        # Service to retrieve current position
        self.get_position_service = self.create_service(
            Trigger,
            'get_position',
            self.get_position_callback)
            
        # Log startup information
        self.get_logger().info(f"Simple map localizer initialized with resolution: {resolution}m/pixel")
        self.get_logger().info(f"Using reference GPS: {ref_lat:.6f}, {ref_lon:.6f}")
        self.get_logger().info(f"UTM: {self.utm_converter.get_zone_info()}")
        
    def get_position_callback(self, request, response):
        """Service to get the current position of the robot."""
        if self.current_position is None:
            response.success = False
            response.message = "No position available yet"
        else:
            map_pos = self.localizer.world_to_map(self.current_position)
            response.success = True
            response.message = (
                f"Current position: "
                f"World=({self.current_position[0]:.2f}, {self.current_position[1]:.2f})m, "
                f"Pixel=({map_pos[0]}, {map_pos[1]})"
            )
        return response
        
    def navsat_callback(self, msg):
        """Process GPS data from NavSatFix messages."""
        try:
            self.get_logger().info("Received GPS message")
            
            # Extract latitude, longitude
            lat = msg.latitude
            lon = msg.longitude
            
            self.get_logger().info(f"GPS coordinates: lat={lat:.6f}, lon={lon:.6f}")
            
            # Check for valid coordinates
            if abs(lat) < 0.01 and abs(lon) < 0.01:
                self.get_logger().warning("Received near-zero GPS coordinates. Ignoring.")
                return
                
            # Convert GPS coordinates to local coordinates using UTM
            try:
                x, y = self.utm_converter.gps_to_local(lat, lon)
                self.get_logger().info(f"Converted to local coordinates: x={x:.2f}, y={y:.2f}")
            except Exception as e:
                self.get_logger().error(f"Error converting GPS to local coordinates: {str(e)}")
                return
            
            # Create covariance matrix from NavSatFix covariance
            pos_cov = np.eye(2) * 4.0  # Default 2m std deviation squared
            
            # Check if covariance is available (not all -1's)
            if hasattr(msg, 'position_covariance_type') and msg.position_covariance_type > 0:
                try:
                    # Convert to our 2x2 format
                    pos_cov[0, 0] = msg.position_covariance[0]  # xx
                    pos_cov[0, 1] = msg.position_covariance[1]  # xy
                    pos_cov[1, 0] = msg.position_covariance[3]  # yx
                    pos_cov[1, 1] = msg.position_covariance[4]  # yy
                    
                    # Ensure the covariance matrix is positive definite
                    if np.linalg.det(pos_cov) <= 0:
                        pos_cov = np.eye(2) * 4.0  # Reset to default if invalid
                except:
                    # If any issue with covariance, use default
                    pass
            
            # Process the position update
            self._process_position_update((x, y), pos_cov)
            
        except Exception as e:
            self.get_logger().error(f'Error processing NavSatFix data: {str(e)}')
            
    def pose_callback(self, msg):
        """Process data from PoseWithCovarianceStamped messages."""
        try:
            # Extract position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            # Extract covariance (just the x,y components)
            cov_matrix = np.array(msg.pose.covariance).reshape(6, 6)
            position_covariance = cov_matrix[0:2, 0:2]
            
            # Process the position update
            self._process_position_update((x, y), position_covariance)
                
        except Exception as e:
            self.get_logger().error(f'Error processing pose data: {str(e)}')
            
    def _process_position_update(self, position, covariance):
        """Common processing for position updates."""
        try:
            # Get original position in both meters and pixels
            orig_world_pos = position
            orig_map_pos = self.localizer.world_to_map(position)
            
            self.get_logger().info(
                f"Original position: "
                f"World=({position[0]:.2f}, {position[1]:.2f})m, "
                f"Pixel=({orig_map_pos[0]}, {orig_map_pos[1]})"
            )
            
            # Use current position as reference if available
            prev_position = self.current_position
            
            # Check if on road and apply correction if needed
            corrected_pos, on_road, confidence, is_outlier = self.localizer.localize(
                position, 
                covariance,
                prev_position
            )
            
            # Track outliers for warning
            if is_outlier:
                self.consecutive_outliers += 1
                if self.consecutive_outliers >= self.max_outliers_warning:
                    self.get_logger().warning(
                        f'Detected {self.consecutive_outliers} consecutive GPS outliers! '
                        f'Check GPS signal or try repositioning robot.'
                    )
            else:
                self.consecutive_outliers = 0
            
            # Get corrected position in pixels
            corrected_map_pos = self.localizer.world_to_map(corrected_pos)
            
            # Calculate distance to road for logging
            if not on_road:
                diff_x = corrected_pos[0] - position[0]
                diff_y = corrected_pos[1] - position[1]
                distance_to_road = np.sqrt(diff_x**2 + diff_y**2)
            else:
                distance_to_road = 0.0
            
            # Update current position and covariance
            self.current_position = corrected_pos
            self.last_position = position  # Store for trajectory tracking
            
            # Adjust covariance based on correction confidence
            confidence_factor = max(0.1, confidence)  # Never reduce below 10%
            self.position_covariance = covariance / confidence_factor
            
            # Publish the corrected position as string
            corrected_msg = String()
            corrected_msg.data = f"{corrected_pos[0]:.6f}, {corrected_pos[1]:.6f}"
            self.corrected_publisher.publish(corrected_msg)
            
            # Also publish as PoseWithCovarianceStamped
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose.position.x = corrected_pos[0]
            pose_msg.pose.pose.position.y = corrected_pos[1]
            pose_msg.pose.pose.position.z = 0.0
            
            # Default orientation (no rotation)
            pose_msg.pose.pose.orientation.w = 1.0
            
            # Fill in covariance (6x6 matrix: x, y, z, roll, pitch, yaw)
            pose_covariance = np.zeros((6, 6))
            pose_covariance[0:2, 0:2] = self.position_covariance
            pose_covariance[2, 2] = 1.0  # Z uncertainty
            pose_covariance[3:6, 3:6] = np.eye(3)  # Angular uncertainty
            pose_msg.pose.covariance = pose_covariance.flatten().tolist()
            
            self.corrected_pose_publisher.publish(pose_msg)
            
            # Extract variance for logging
            variance_x = covariance[0, 0]
            std_dev = np.sqrt(variance_x)
            
            # Log the update with outlier indication
            outlier_str = " [OUTLIER REJECTED]" if is_outlier else ""
            
            if on_road:
                self.get_logger().info(
                    f'Position already on road: '
                    f'World=({position[0]:.2f}, {position[1]:.2f})m [±{std_dev:.2f}m], '
                    f'Pixel=({orig_map_pos[0]}, {orig_map_pos[1]}){outlier_str}'
                )
                self.publisher_coords.publish(Float64MultiArray(data=[orig_map_pos[0], abs(orig_map_pos[1])]))
                self.publisher_coords.publish(Float64MultiArray(data=[orig_map_pos[0], abs(1246 - orig_map_pos[1])]))
            else:
                self.get_logger().info(
                    f'Corrected position: '
                    f'World: ({position[0]:.2f}, {position[1]:.2f})m → ({corrected_pos[0]:.2f}, {corrected_pos[1]:.2f})m [±{std_dev:.2f}m], '
                    f'Pixel: ({orig_map_pos[0]}, {orig_map_pos[1]}) → ({corrected_map_pos[0]}, {corrected_map_pos[1]}), '
                    f'Distance: {distance_to_road:.2f}m, Confidence: {confidence:.2f}{outlier_str}'
                )
                self.publisher_coords.publish(Float64MultiArray(data=[corrected_map_pos[0], abs(corrected_map_pos[1])]))
                self.publisher_coords.publish(Float64MultiArray(data=[corrected_map_pos[0], abs(1246 - corrected_map_pos[1])]))                
        except Exception as e:
            self.get_logger().error(f'Error in position update: {str(e)}')


def main(args=None):
    """Main function for the localizer node."""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create and run the node with hardcoded parameters
    try:
        node = SimpleLocalizationNode(
            map_file='/home/cewan/Automated-Tour-Car/ws_lidar/src/gps_path_tracking/config/ramp.png',  # Replace with your actual map path
            resolution=0.03,                    # Map resolution in meters/pixel
            origin=(0.0, 0.0),                # Origin of the map
            initial_position=(2.0, 18.0),    # Starting position
            ref_lat=33.644499,                     # Reference latitude
            ref_lon=-117.840907,                    # Reference longitude
            utm_zone=11
        )
        
        # easting, northing = node.utm_converter.gps_to_utm(33.6448461,-117.840944)
        # node.utm_converter.ref_easting = easting - 2.0
        # node.utm_converter.ref_northing = northing - 18.0

        node.get_logger().info("Simple localizer is running. Press Ctrl+C to stop.")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        # Clean shutdown
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()