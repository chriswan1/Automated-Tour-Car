from ultralytics import YOLO
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import math
import smbus

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
            self.get_logger().info(f'Turning Left')
            pwm_command = "PWM.setMotorModel(-1500,-1500,2000,2000)"  # Turn left
        else:
            self.get_logger().info(f'Turning Right')
            pwm_command = "PWM.setMotorModel(2000,2000,-1500,-1500)"  # Turn right
    elif left > 0:
        self.get_logger().info(f'Turning Right')
        pwm_command = "PWM.setMotorModel(2000,2000,-1500,-1500)"  # Turn right
    elif right > 0:
        self.get_logger().info(f'Turning Left')
        pwm_command = "PWM.setMotorModel(-1500,-1500,2000,2000)"  # Turn left
    else:
        self.get_logger().info(f'Moving Forward')
        pwm_command = "PWM.setMotorModel(1000,1000,1000,1000)"  # Move forward

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
            #cv2.imshow("YOLOv8n Detection", frame)

            # Break on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info(f'Stopping')
                PWM.setMotorModel(0,0,0,0)
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


