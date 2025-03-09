#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math
import numpy as np
from collections import deque

class CovarianceEvaluator(Node):
    def __init__(self):
        super().__init__('covariance_evaluator')
        
        # Store recent positions
        self.raw_positions = deque(maxlen=50)
        self.filtered_positions = deque(maxlen=50)
        
        # Subscribe to raw GPS
        self.raw_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.raw_callback,
            10
        )
        
        # Subscribe to filtered odometry
        self.filtered_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_callback,
            10
        )
        
        # Timer for periodic evaluation
        self.timer = self.create_timer(5.0, self.evaluate)
    
    def raw_callback(self, msg):
        self.raw_positions.append((msg.latitude, msg.longitude))
    
    def filtered_callback(self, msg):
        self.filtered_positions.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
    
    def calculate_jitter(self, positions):
        """Calculate average distance between consecutive points"""
        if len(positions) < 2:
            return 0.0
            
        total_distance = 0.0
        for i in range(1, len(positions)):
            p1 = positions[i-1]
            p2 = positions[i]
            # Simple Euclidean distance
            dist = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            total_distance += dist
            
        return total_distance / (len(positions) - 1)
    
    def evaluate(self):
        """Evaluate the effectiveness of the covariance matrix"""
        if len(self.raw_positions) < 2 or len(self.filtered_positions) < 2:
            self.get_logger().info('Not enough data for evaluation yet')
            return
            
        # For raw GPS, convert to meters first (rough approximation)
        raw_meters = []
        first_lat = self.raw_positions[0][0]
        for lat, lon in self.raw_positions:
            # Convert to meters (approximate)
            x = (lon - self.raw_positions[0][1]) * 111320 * math.cos(math.radians(first_lat))
            y = (lat - first_lat) * 111320
            raw_meters.append((x, y))
            
        raw_jitter = self.calculate_jitter(raw_meters)
        filtered_jitter = self.calculate_jitter(self.filtered_positions)
        
        improvement = (1 - (filtered_jitter / raw_jitter)) * 100 if raw_jitter > 0 else 0
        
        self.get_logger().info(f'Raw GPS jitter: {raw_jitter:.3f} meters')
        self.get_logger().info(f'Filtered jitter: {filtered_jitter:.3f} meters')
        self.get_logger().info(f'Improvement: {improvement:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceEvaluator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()