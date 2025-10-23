import cv2
import numpy as np
import ArducamDepthCamera as ac
import serial
import time

# =============================
# CONFIGURATION
# =============================
MAX_DISTANCE = 4000
CONFIDENCE_THRESHOLD = 30
OBJECT_DISTANCE_THRESHOLD = 500  # mm
WAIT_AFTER_LOST = 3
SCAN_DELAY = 0.05  # seconds
PORT = "/dev/ttyACM0"
BAUDRATE = 9600

X_MIN, X_MAX = 0, 180
Y_MIN, Y_MAX = 0, 90


# =============================
# SERVO CONTROL
# =============================
class ServoController:
    def __init__(self, port, baudrate):
        self.arduino = serial.Serial(port, baudrate, timeout=1)
        time.sleep(3)
        self.x_angle = 90
        self.y_angle = 45
        self.send_angles(self.x_angle, self.y_angle)

    def send_angles(self, x, y):
        x = int(np.clip(x, X_MIN, X_MAX))
        y = int(np.clip(y, Y_MIN, Y_MAX))
        cmd = f"{x},{y}\n"
        self.arduino.write(cmd.encode("utf-8"))
        print(f"Sent angles: X={x}, Y={y}")

    def move(self, dx=0, dy=0):
        self.x_angle = int(np.clip(self.x_angle + dx, X_MIN, X_MAX))
        self.y_angle = int(np.clip(self.y_angle + dy, Y_MIN, Y_MAX))
        self.send_angles(self.x_angle, self.y_angle)

    def close(self):
        self.arduino.close()


# =============================
# CAMERA HELPERS
# =============================
def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < CONFIDENCE_THRESHOLD] = (0, 0, 0)
    return preview


def getPreviewBW(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < CONFIDENCE_THRESHOLD] = 0
    return preview

"""
def get_nearest_object(depth, confidence):
    mask = confidence > CONFIDENCE_THRESHOLD
    valid_depth = np.where(mask, depth, np.inf)
    min_dist = np.min(valid_depth)
    if not np.isfinite(min_dist):
        return None, None, None
    if min_dist > OBJECT_DISTANCE_THRESHOLD:
        return None, None, None
    y, x = np.unravel_index(np.argmin(valid_depth), valid_depth.shape)
    return x, y, min_dist
"""

def get_nearest_object(depth, confidence):
    """
    Finds the nearest object region (confidence > threshold),
    and returns its center (x, y) and mean distance.
    """
    mask = confidence > CONFIDENCE_THRESHOLD
    valid_depth = np.where(mask, depth, np.inf)

    # Ignore invalid
    if not np.any(np.isfinite(valid_depth)):
        return None, None, None

    # Get minimum depth value (nearest object)
    min_dist = np.min(valid_depth)
    if min_dist > OBJECT_DISTANCE_THRESHOLD:
        return None, None, None

    # Segment region within ±5 cm of that depth
    near_mask = (valid_depth < min_dist + 50) & (valid_depth > min_dist - 50)

    # Find contours or centroid
    near_mask_uint8 = np.uint8(near_mask)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(near_mask_uint8, connectivity=8)

    if num_labels <= 1:
        return None, None, None

    # Find largest valid component (ignore background label 0)
    largest_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
    cx, cy = centroids[largest_idx]
    region_depth = np.mean(valid_depth[labels == largest_idx])

    return int(cx), int(cy), float(region_depth)


# =============================
# MAIN LOOP
# =============================
def main():
    print("Initializing Arducam ToF tracking system...")

    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error:", ret)
        return
    cam.start(ac.FrameType.DEPTH)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    r = cam.getControl(ac.Control.RANGE)

    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    cv2.namedWindow("rgb", cv2.WINDOW_AUTOSIZE)
    servo = ServoController(PORT, BAUDRATE)

    scanning = True
    direction_x = 1
    direction_y = 1
    last_seen = time.time()

    try:
        while True:
            frame = cam.requestFrame(2000)
            if frame is None or not isinstance(frame, ac.DepthData):
                continue

            depth_buf = frame.depth_data
            confidence_buf = frame.confidence_data
            height, width = depth_buf.shape

            # --- Visualization
            result_image = (depth_buf * (255.0 / r)).astype(np.uint8)
            RGB_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
            RGB_image = getPreviewRGB(RGB_image, confidence_buf)
            BW_image = getPreviewBW(result_image, confidence_buf)

            """
            # --- Object detection
            x, y, dist = get_nearest_object(depth_buf, confidence_buf)
            if x is not None:
                scanning = False
                last_seen = time.time()
                cv2.circle(RGB_image, (x, y), 10, (255, 255, 255), 2)

                # Move servos proportionally to keep object centered
                dx = (x - width / 2) / (width / 2)
                dy = (y - height / 2) / (height / 2)
                servo.move(dx * 5, dy * 5)
                print(f"Tracking object at {dist:.1f} mm")
            """

            # --- Object detection: find nearest object cluster
            mask = (confidence_buf > CONFIDENCE_THRESHOLD) & (depth_buf < OBJECT_DISTANCE_THRESHOLD)
            
            if np.any(mask):
                # Focus on the closest range
                min_dist = np.min(depth_buf[mask])
                if not np.isfinite(min_dist):
                    min_dist = OBJECT_DISTANCE_THRESHOLD
            
                # Keep only pixels close to min distance (±5 cm band)
                band_mask = mask & (depth_buf < min_dist + 50)
            
                # Find connected components (object segmentation)
                band_mask_uint8 = band_mask.astype(np.uint8)
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(band_mask_uint8, connectivity=8)
            
                if num_labels > 1:
                    # Find the largest valid object (excluding background)
                    largest_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1
                    cx, cy = centroids[largest_idx]
                    dist = np.mean(depth_buf[labels == largest_idx])
                    
                    # Draw bounding box and centroid
                    x, y, w, h, _ = stats[largest_idx]
                    cv2.rectangle(RGB_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.circle(RGB_image, (int(cx), int(cy)), 5, (0, 0, 0), -1)
            
                    scanning = False
                    last_seen = time.time()
            
                    # Move servos proportionally to center the object
                    dx = (cx - width / 2) / (width / 2)
                    dy = (cy - height / 2) / (height / 2)
                    servo.move(dx * 5, dy * 5)
                    print(f"Tracking object at {dist:.1f} mm | center=({cx:.1f},{cy:.1f})")
                else:
                    # No valid object found
                    if not scanning and time.time() - last_seen > WAIT_AFTER_LOST:
                        scanning = True
                        print("Object lost. Resuming scan...")
            else:
                # No object in view
                if not scanning and time.time() - last_seen > WAIT_AFTER_LOST:
                    scanning = True
                    print("Object lost. Resuming scan...")
            
                if scanning:
                    # Sweep scanning pattern
                    servo.x_angle += direction_x * 2
                    if servo.x_angle >= X_MAX or servo.x_angle <= X_MIN:
                        direction_x *= -1
                        servo.y_angle += direction_y * 2
                        if servo.y_angle >= Y_MAX or servo.y_angle <= Y_MIN:
                            direction_y *= -1
                    servo.send_angles(servo.x_angle, servo.y_angle)
                    time.sleep(SCAN_DELAY)


            else:
                # Lost tracking
                if not scanning and time.time() - last_seen > WAIT_AFTER_LOST:
                    scanning = True
                    print("Object lost. Resuming scan...")

                if scanning:
                    # Sweep scanning pattern
                    servo.x_angle += direction_x * 2
                    if servo.x_angle >= X_MAX or servo.x_angle <= X_MIN:
                        direction_x *= -1
                        servo.y_angle += direction_y * 2
                        if servo.y_angle >= Y_MAX or servo.y_angle <= Y_MIN:
                            direction_y *= -1
                    servo.send_angles(servo.x_angle, servo.y_angle)
                    time.sleep(SCAN_DELAY)

            cv2.imshow("rgb", RGB_image)
            cv2.imshow("bw", BW_image)
            cam.releaseFrame(frame)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        servo.close()
        cam.stop()
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
