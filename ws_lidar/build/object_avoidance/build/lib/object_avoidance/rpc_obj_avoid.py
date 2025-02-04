from ultralytics import YOLO
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import math
import smbus


class PCA9685:
    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        self.write(self.__MODE1, 0x00)

    def write(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, reg, value)

    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, reg)
        return result

    def setPWMFreq(self, freq):
        "Sets the PWM frequency"
        prescaleval = 25000000.0  # 25MHz
        prescaleval /= 4096.0  # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)  # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        "Sets a single PWM channel"
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(String, 'car_directions', 10)

    def publish_direction(self, pwm_command):
        msg = String()
        msg.data = pwm_command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published direction: {pwm_command}')


def avoid_obstacle(bboxes, frame, frame_width, publisher):
    left, center, right = 0, 0, 0
    for bbox in bboxes:
        x1, y1, x2, y2 = bbox
        bbox_midpoint = (x1 + x2) / 2
        if bbox_midpoint < frame_width // 3:
            left += 1
        elif bbox_midpoint > 2 * (frame_width // 3):
            right += 1
        else:
            center += 1

    pwm_command = "stop"
    if center > 0:
        if left <= right:
            pwm_command = "PWM.setMotorModel(-1500,-1500,2000,2000) left"  # Turn left
        else:
            pwm_command = "PWM.setMotorModel(2000,2000,-1500,-1500) right"  # Turn right
    elif left > 0:
        pwm_command = "PWM.setMotorModel(2000,2000,-1500,-1500) right"  # Turn right
    elif right > 0:
        pwm_command = "PWM.setMotorModel(-1500,-1500,2000,2000) left"  # Turn left
    else:
        pwm_command = "PWM.setMotorModel(1000,1000,1000,1000) forward"  # Move forward

    publisher.publish_direction(pwm_command)
    return pwm_command


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    # Load YOLOv8n model
    model = YOLO("yolov8n.pt")  # Ensure the yolov8n.pt file is downloaded

    # Video capture setup
    cap = cv2.VideoCapture(0)  # Replace 0 with your camera source
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret or frame is None:
                print("Error: Failed to grab frame.")
                continue

            # print(f"Frame shape before processing: {frame.shape}")

            # Ensure frame is properly formatted
            if len(frame.shape) != 3 or frame.shape[2] != 3:
                # print("Error: Invalid frame shape.")
                frame = frame.reshape((480, 640, 3))

            # Run YOLOv8n inference
            results = model.predict(frame, conf=0.5)
            bboxes = [
                box.xyxy[0].cpu().numpy()
                for box in results[0].boxes if box.cls == 0
            ]

            # Avoid obstacles
            avoid_obstacle(bboxes, frame, frame_width, node)

            # Display frame
            cv2.imshow("YOLOv8n Detection", frame)

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                PWM.setMotorModel(0,0,0,0)
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

