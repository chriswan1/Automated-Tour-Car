import cv2
import time
import os
import numpy as np

# Load YOLOv4-Tiny Model
net = cv2.dnn.readNet("yolov4-tiny.weights", "yolov4-tiny.cfg")
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Load class labels
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

print(f"✅ Loaded {len(classes)} class labels: {classes[:10]}...")  # Verify class names

# Define thresholds
CONF_THRESHOLD = 0.5  # Confidence threshold
NMS_THRESHOLD = 0.3   # Non-Maximum Suppression threshold

def apply_edge_detection(frame):
    """Applies Canny edge detection and thickens the edges."""
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
    edges = cv2.Canny(gray, 50, 150)  # Apply Canny edge detection

    # **Thicken the edges using morphological dilation**
    kernel = np.ones((3, 3), np.uint8)  # Adjust kernel size for thicker edges
    edges_thick = cv2.dilate(edges, kernel, iterations=2)  # Increase iterations for even thicker edges

    return edges_thick


def capture_image():
    """ Captures a single image from the camera and saves it. """
    
    # Initialize the camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 416)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 416)

    if not cap.isOpened():
        print("❌ Error: Could not open camera.")
        return None

    # Wait for camera to initialize
    time.sleep(2)

    # Capture a single frame
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("❌ Error: Failed to capture image.")
        return None

    print(f"✅ Raw Frame Shape: {frame.shape}, Data Type: {frame.dtype}")

    # Validate frame format
    if len(frame.shape) == 2 or frame.shape[0] == 1:
        print("⚠️ Frame appears to be grayscale, converting to BGR...")
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    # Save the image
    image_path = os.path.join(os.getcwd(), "captured_image.jpg")
    cv2.imwrite(image_path, frame)
    print(f"✅ Image saved at: {image_path}")

    return image_path

def detect_objects(image_path):
    """ Loads an image, runs YOLOv4-Tiny object detection, applies edge detection, and saves the annotated image. """

    # Load the image
    frame = cv2.imread(image_path)

    if frame is None:
        print("❌ Error: Could not load image.")
        return

    print(f"✅ Image loaded: {image_path}, Shape: {frame.shape}")

    # Resize the image to YOLO format (416x416)
    frame = cv2.resize(frame, (416, 416))

    # Apply Canny edge detection
    edges = apply_edge_detection(frame)

    # Convert edges to 3-channel format for overlay
    edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

    # Preprocess the image for YOLO
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    output_layers = [net.getLayerNames()[i - 1] for i in net.getUnconnectedOutLayers()]
    
    detections = net.forward(output_layers)

    boxes = []
    confs = []
    class_ids = []

    print("\n🔍 Detected Objects:")

    for output in detections:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > CONF_THRESHOLD:
                center_x, center_y, width, height = (
                    detection[0:4] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
                ).astype("int")
                x = int(center_x - width / 2)
                y = int(center_y - height / 2)

                detected_class = classes[class_id] if class_id < len(classes) else "Unknown"
                print(f" - {detected_class} ({confidence*100:.2f}%) at X:{x}, Y:{y}, W:{width}, H:{height}")

                boxes.append([x, y, int(width), int(height)])
                confs.append(float(confidence))
                class_ids.append(class_id)

    if not boxes:
        print("No objects detected.")
        return

    # Apply Non-Maximum Suppression (NMS)
    indices = cv2.dnn.NMSBoxes(boxes, confs, CONF_THRESHOLD, NMS_THRESHOLD)

    # **Check if NMS found valid indices**
    if indices is None or len(indices) == 0:
        print("⚠️ No valid boxes after NMS.")
        return

    # **Ensure indices are properly handled**
    indices = indices.flatten() if isinstance(indices, np.ndarray) else []

    # Draw bounding boxes on the image
    for i in indices:
        x, y, w, h = boxes[i]
        label = f"{classes[class_ids[i]]}: {confs[i]*100:.2f}%"
        color = (0, 255, 0)  # Green box for detected objects
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # **Overlay edge detection on the image**
    frame_with_edges = cv2.addWeighted(frame, 0.8, edges_colored, 0.2, 0)

    # Save the annotated image
    annotated_image_path = os.path.join(os.getcwd(), "annotated_image.jpg")
    cv2.imwrite(annotated_image_path, frame_with_edges)
    print(f"✅ Annotated image with edges saved: {annotated_image_path}")

def main():
    """ Main function to execute the image capture and annotation process. """
    image_path = capture_image()
    if image_path:
        detect_objects(image_path)

if __name__ == "__main__":
    main()
