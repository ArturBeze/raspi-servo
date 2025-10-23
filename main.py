import cv2
import numpy as np
import serial
import time
from arducam_depth_camera import ArducamCamera, DepthData

# =============================
# === CONFIGURATION ===========
# =============================
PORT = '/dev/ttyACM0'      # Arduino port
BAUDRATE = 9600
SCAN_DELAY = 0.05           # seconds between servo steps
OBJECT_DISTANCE_THRESHOLD = 500   # mm (50 cm)
CONFIDENCE_THRESHOLD = 30
WAIT_AFTER_LOST = 3         # seconds

# Servo limits
X_MIN, X_MAX = 0, 180
Y_MIN, Y_MAX = 0, 90

# =============================
# === ARDUINO SERVO CONTROL ===
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
        command = f"{x},{y}\n"
        self.arduino.write(command.encode('utf-8'))
        print(f"Sent angles: X={x}, Y={y}")

    def move(self, dx=0, dy=0):
        self.x_angle = int(np.clip(self.x_angle + dx, X_MIN, X_MAX))
        self.y_angle = int(np.clip(self.y_angle + dy, Y_MIN, Y_MAX))
        self.send_angles(self.x_angle, self.y_angle)

    def close(self):
        self.arduino.close()


# =============================
# === IMAGE HELPERS ===========
# =============================
def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray, confidence_value=CONFIDENCE_THRESHOLD) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < confidence_value] = (0, 0, 0)
    return preview


def getPreviewBW(preview: np.ndarray, confidence: np.ndarray, confidence_value=CONFIDENCE_THRESHOLD) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < confidence_value] = 0
    return preview


def get_nearest_object(depth, confidence):
    """Find coordinates (x, y) of nearest valid point."""
    mask = confidence > CONFIDENCE_THRESHOLD
    valid_depth = np.where(mask, depth, np.inf)
    min_dist = np.min(valid_depth)
    if not np.isfinite(min_dist):
        return None, None, None
    if min_dist > OBJECT_DISTANCE_THRESHOLD:
        return None, None, None
    y, x = np.unravel_index(np.argmin(valid_depth), valid_depth.shape)
    return x, y, min_dist


# =============================
# === MAIN LOOP ===============
# =============================
def main():
    cam = ArducamCamera()
    if not cam.open():
        raise RuntimeError("Failed to open Arducam ToF camera.")
    cam.start()
    servo = ServoController(PORT, BAUDRATE)
    print("System ready. Starting scan...")

    scanning = True
    direction_x = 1
    direction_y = 1
    last_seen = time.time()

    try:
        while True:
            frame = cam.requestFrame(2000)
            if frame is None or not isinstance(frame, DepthData):
                continue

            depth_buf = frame.depth_data
            confidence_buf = frame.confidence_data
            height, width = depth_buf.shape

            # For visualization
            r = np.max(depth_buf)
            result_image = (depth_buf * (255.0 / r)).astype(np.uint8)
            RGB_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
            RGB_image = getPreviewRGB(RGB_image, confidence_buf)
            BW_image = getPreviewBW(result_image, confidence_buf)

            # Object detection
            x, y, dist = get_nearest_object(depth_buf, confidence_buf)
            if x is not None:
                scanning = False
                last_seen = time.time()

                # Draw target
                cv2.circle(RGB_image, (x, y), 10, (255, 255, 255), 2)

                # Calculate offset from center
                dx = (x - width / 2) / (width / 2)
                dy = (y - height / 2) / (height / 2)

                # Adjust servo positions (proportional control)
                servo.move(dx * 5, dy * 5)
                print(f"Tracking object at {dist:.1f} mm")

            else:
                # Lost object
                if not scanning and time.time() - last_seen > WAIT_AFTER_LOST:
                    scanning = True
                    print("Object lost. Resuming scan...")

                if scanning:
                    servo.x_angle += direction_x * 2
                    if servo.x_angle >= X_MAX or servo.x_angle <= X_MIN:
                        direction_x *= -1
                        servo.y_angle += direction_y * 2
                        if servo.y_angle >= Y_MAX or servo.y_angle <= Y_MIN:
                            direction_y *= -1
                    servo.send_angles(servo.x_angle, servo.y_angle)
                    time.sleep(SCAN_DELAY)

            # Show preview
            cv2.imshow("Depth RGB", RGB_image)
            cv2.imshow("Depth BW", BW_image)

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
