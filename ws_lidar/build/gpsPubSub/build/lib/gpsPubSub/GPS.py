#!/usr/bin/env python3
import rclpy as ros
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
from serial import Serial
import pynmea2
from std_msgs.msg import String, Header
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker
from time import sleep
import time
import numpy as np

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('GPS_publisher')
        self.publisher = self.create_publisher(msg_type=NavSatFix,topic='/gps/fix', qos_profile=10)
        timer_period=0.5
        self.covariance_matrix = np.diag([0.0,0.0,0.0]).flatten()
        try:
            self.ser = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)
        except serial.SerialException as err:
            print(f"Error: {err}")
            raise
        self.timer = self.create_timer(timer_period, self.get_lat_lon)

    def get_lat_lon(self):
        try:
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps'

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            data2 = self.ser.readline()
            try:
                decoded_data2 = data2.decode('utf-8', errors='ignore').strip()
            except Exception:
                # If all else fails, just skip invalid bytes
                decoded_data2 = ''.join([chr(b) if b < 128 else ' ' for b in data2])


            if decoded_data2.startswith("$GNGGA"):
                try:
                    original_msg2 = pynmea2.parse(decoded_data2)

                    #if hasattr(original_msg2, 'horizontal_dilution_of_precision'):
                    hdop=original_msg2.horizontal_dil
                    print(hdop)
                    if hdop == '':
                        hdop = 0.0
                    sx = float(hdop) * 2
                    sy = float(hdop) * 2
                    sz = 5
                    self.covariance_matrix = np.diag([sx**2,sy**2,sz**2]).flatten()
                except pynmea2.ParseError as err:
                    self.get_logger().warn(f"NMEA Parsing error: {err}")
                    return

            data1 = self.ser.readline()
            try:
                decoded_data1 = data1.decode('utf-8', errors='ignore').strip()
            except Exception:
                # If all else fails, just skip invalid bytes
                decoded_data1 = ''.join([chr(b) if b < 128 else ' ' for b in data2])
            if decoded_data1.startswith("$GNRMC"):
                try:
                    original_msg1 = pynmea2.parse(decoded_data1)
                    msg.latitude = original_msg1.latitude
                    msg.longitude = original_msg1.longitude
                    msg.position_covariance = self.covariance_matrix
                    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                    self.get_logger().info('Publishing GPS data...')
                    print(str(original_msg1.latitude)+str(original_msg1.longitude))
                    self.publisher.publish(msg)
                except pynmea2.ParseError as err:
                    self.get_logger().warn(f"NMEA Parsing error: {err}")
                    return
        except Exception as err:
            self.get_logger().error(f"Timer Callback error: {err}")
    

def main(args=None):
    ser = Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    #Enable WAAS
    ser.write(b'$PMTK301,2*2E\r\n')
    sleep(1)
    ser.readline()

    #5Hz updates
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(b'$PMTK220,200*2C\r\n')
    sleep(1)
    ser.readline()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.close()
    ros.init(args=args)
    gps = GPSPublisher()
    ros.spin(gps)
    ros.shutdown()



if __name__ == '__main__':
    main() 

