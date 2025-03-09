import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import csv
import time

class GatherData(Node):
    def __init__(self):
        super().__init__('data_for_ml')

        self.annotate = self.create_publisher(String, '/mldata', 10)
        self.csv_file = open('mldata.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow('time','x','y')