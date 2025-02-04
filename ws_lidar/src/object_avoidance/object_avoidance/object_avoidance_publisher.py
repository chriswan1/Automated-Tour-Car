import time
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np

# Load YOLOv4-Tiny Model
net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Load class labels
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

print(f"✅ Loaded {len(classes)} class labels: {classes[:10]}...")  # Verify class names

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'car_directions', 10)

    def publish_direction(self, pwm_values, duration):
        msg = Int32MultiArray()
        msg.data = pwm_values
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published PWM values: {pwm_values} for {duration}s')
        time.sleep(duration)
        self.publisher_.publish(Int32MultiArray(data=[0, 0, 0, 0]))  # Stop motors

def detect_objects_yolov4(frame):
    """ Detects objects using YOLOv4-Tiny and applies NMS """

    if len(frame.shape) == 2 or frame.shape[0] == 1:
        frame = frame.reshape((416, 416, 3))

    if frame.shape[2] != 3:
        print("⚠️ Frame missing color channels, converting to RGB...")
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

    # Debugging: Print frame shape after conversion
    print(f"✅ Debug: Frame shape AFTER processing: {frame.shape}")

    # Preprocess the image for YOLO
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    output_layers = [net.getLayerNames()[i - 1] for i in net.getUnconnectedOutLayers()]
    detections = net.forward(output_layers)

    boxes, confs, class_ids = [], [], []
    for output in detections:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if class_id == 0 and confidence > 0.3: # confidence threshold
                center_x, center_y, width, height = (detection[0:4] * np.array(
                    [frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])).astype("int")
                x = int(center_x - width / 2)
                y = int(center_y - height / 2)

                boxes.append([x, y, int(width), int(height)])
                confs.append(float(confidence))
                class_ids.append(class_id)

    # Apply Non-Maximum Suppression (NMS)
    indices = cv2.dnn.NMSBoxes(boxes, confs, 0.3, 0.4)
    if len(indices) == 0:
        print("⚠️ No valid boxes after NMS.")
        return [], [], []  # Return empty results

    filtered_boxes, filtered_confs, filtered_class_ids = [], [], []
    for i in indices.flatten():  # Safely handle the indices
        filtered_boxes.append(boxes[i])
        filtered_confs.append(confs[i])
        filtered_class_ids.append(class_ids[i])

    return filtered_boxes, filtered_confs, filtered_class_ids

def avoid_obstacle(boxes, frame, frame_width, publisher):
    """ Decides motor commands based on detected objects """
    left, center, right = 0, 0, 0
    for box in boxes:
        x, _, w, _ = box
        bbox_midpoint = x + w / 2
        if bbox_midpoint < frame_width // 3:
            left += 1
        elif bbox_midpoint > 2 * (frame_width // 3):
            right += 1
        else:
            center += 1

    pwm_values = [0, 0, 0, 0]  # Default stop state
    duration = 0.5  # Duration to move
    if center > 0:
        if left <= right:
            pwm_values = [-1500, -1500, 2000, 2000]  # Turn left
            print("Turning Left")
        else:
            pwm_values = [2000, 2000, -1500, -1500]  # Turn right
            print("Turning Right")
    elif left > 0:
        pwm_values = [2000, 2000, -1500, -1500]  # Turn right
        print("Turning Right")
    elif right > 0:
        pwm_values = [-1500, -1500, 2000, 2000]  # Turn left
        print("Turning Left")
    else:
        pwm_values = [1000, 1000, 1000, 1000]  # Move forward
        print("Moving Forward")

    publisher.publish_direction(pwm_values, duration)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 416)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 416)

    if not cap.isOpened():
        print("❌ Error: Could not open camera.")
        exit()

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    last_capture_time = time.time()
    interval = 0.25  # Process every 0.5 seconds

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret or frame is None:
                print("❌ Error: Failed to grab frame.")
                continue

            if time.time() - last_capture_time < interval:
                continue  # Skip processing if interval hasn't elapsed
            last_capture_time = time.time()

            print(f"Processing image at {time.strftime('%H:%M:%S')}")

            boxes, confs, class_ids = detect_objects_yolov4(frame)

            if boxes:
                print("Detected Objects:")
                for i in range(len(boxes)):
                    x, y, w, h = boxes[i]
                    label = f"{classes[class_ids[i]]} ({confs[i]*100:.2f}%)"
                    print(f" - {label} at X:{x}, Y:{y}, W:{w}, H:{h}")
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            else:
                print("No objects detected.")

            avoid_obstacle(boxes, frame, frame_width, node)

    finally:
        cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
