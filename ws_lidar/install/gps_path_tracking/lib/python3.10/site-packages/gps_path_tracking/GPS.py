#!/usr/bin/env python3
import rclpy as ros
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
import pynmea2
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker
from time import sleep
import time

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('GPS_publisher')
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(msg_type=NavSatFix,topic='gps', qos_profile=10)
        timer_period=1
        self.timer = self.create_timer(timer_period, self.get_lat_lon)
        
    
    def get_lat_lon(self):
        try:
            ser = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)
            while True:
                msg = NavSatFix()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()

                msg.status.status = NavSatStatus.STATUS_FIX
                msg.status.service = NavSatStatus.SERVICE_GPS

                data = ser.readline().decode('utf-8').strip()
                if data.startswith("$GNRMC"):
                    original_msg = pynmea2.parse(data)
                    msg.latitude = original_msg.latitude
                    msg.longitude = original_msg.longitude
                    self.get_logger().info('Publishing GPS data...')
                    self.publisher.publish(msg)
        except serial.SerialException as err:
            print(f"Error: {err}")
        except pynmea2.ParseError as err:
            print(f"Parsing Error: {err}")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
    

def main(args=None):
    ros.init(args=args)
    gps = GPSPublisher()
    ros.spin(gps)
    ros.shutdown()



if __name__ == '__main__':
    main() 

