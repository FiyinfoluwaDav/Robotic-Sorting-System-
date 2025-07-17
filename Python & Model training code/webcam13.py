import cv2
import torch
import serial
import time
import logging
from ultralytics import YOLO

# Setup logging
logging.basicConfig(level=logging.INFO)

# Load YOLO model
try:
    model = YOLO("Yolo_v11_7.pt").to('cuda' if torch.cuda.is_available() else 'cpu')
except Exception as e:
    logging.error(f"Failed to load model: {e}")
    exit()

# Connect to ESP32 with retry mechanism
max_retries = 3
for attempt in range(max_retries):
    try:
        esp32 = serial.Serial("COM5", 115200, timeout=1)
        time.sleep(2)
        logging.info("Connected to ESP32")
        break
    except serial.SerialException as e:
        logging.error(f"Attempt {attempt+1} failed: {e}")
        if attempt == max_retries - 1:
            logging.error("Max retries reached. Exiting.")
            exit()
        time.sleep(1)

# Open webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    logging.error("Webcam not detected!")
    exit()

# === Detection Config ===
frame_skip = 2
frame_count = 0
object_counters = {"Chivita": 0, "Milo": 0}
detection_threshold = 3
detection_allowed = True
cooldown_start = None
cooldown_duration = 5  # Increased to match sorting time

# === Multi-object persistence tracking ===
active_detections = []  # List of dicts: {class, bbox, timestamp}
iou_threshold = 0.5
persistence_time = 2  # seconds

def compute_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])

    interWidth = max(0, xB - xA)
    interHeight = max(0, yB - yA)
    interArea = interWidth * interHeight

    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])

    unionArea = boxAArea + boxBArea - interArea + 1e-6
    return interArea / unionArea

def is_new_detection(current_box, current_class):
    current_time = time.time()
    for entry in active_detections:
        if entry['class'] == current_class:
            iou = compute_iou(current_box, entry['bbox'])
            if iou >= iou_threshold:
                entry['timestamp'] = current_time
                return False
    return True

def cleanup_old_detections():
    current_time = time.time()
    active_detections[:] = [entry for entry in active_detections
                            if current_time - entry['timestamp'] <= persistence_time]

try:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            logging.warning("Failed to capture frame!")
            continue

        frame_count += 1
        if frame_count % frame_skip != 0:
            continue

        small_frame = cv2.resize(frame, (640, 480))

        if detection_allowed:
            results = model(small_frame)
            cleanup_old_detections()

            for r in results:
                if r.boxes is None or len(r.boxes) == 0:
                    continue

                boxes = r.boxes.xyxy
                confs = r.boxes.conf
                cls = r.boxes.cls

                high_conf_indices = torch.where(confs >= 0.85)[0]

                for idx in high_conf_indices:
                    cls_id = int(cls[idx])
                    class_name = model.names[cls_id]
                    confidence = confs[idx].item()

                    if class_name not in object_counters:
                        continue

                    x1, y1, x2, y2 = map(int, boxes[idx])
                    current_bbox = (x1, y1, x2, y2)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{class_name} {confidence:.2f}"
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    if is_new_detection(current_bbox, class_name):
                        timestamp = int(time.time())
                        data = f"{class_name},{round(confidence, 2)},{timestamp}\n"
                        try:
                            esp32.write(data.encode())
                            logging.info(f"Sending to ESP32: {data.strip()}")
                            active_detections.append({
                                'class': class_name,
                                'bbox': current_bbox,
                                'timestamp': time.time()
                            })
                            time.sleep(0.1)  # Prevent serial buffer overflow
                        except serial.SerialException as e:
                            logging.error(f"Error sending to ESP32: {e}")

        # ESP32 response handling
        if esp32.in_waiting:
            try:
                response = esp32.readline().decode().strip()
                logging.info(f"ESP32 response: {response}")
                if response == "READY":
                    logging.info("ESP32 back to center. Resuming detection...")
                    detection_allowed = True
                    cooldown_start = None
            except serial.SerialException as e:
                logging.error(f"Error reading from ESP32: {e}")

        if not detection_allowed and cooldown_start:
            if time.time() - cooldown_start >= cooldown_duration:
                logging.info("Timeout reached. Resuming detection...")
                detection_allowed = True
                cooldown_start = None

        cv2.imshow("YOLO Sorting", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    logging.info("Script interrupted by user.")
finally:
    cap.release()
    cv2.destroyAllWindows()
    esp32.close()
    logging.info("Resources cleaned up.")