import cv2
import numpy as np
import ArducamDepthCamera as ac
import serial
import time

# =========================
# CONFIGURATION
# =========================
PORT = "/dev/ttyACM0"
BAUDRATE = 9600

CONFIDENCE_THRESHOLD = 30
OBJECT_DISTANCE_THRESHOLD = 500   # mm (50 cm)
SCAN_DELAY = 0.05
WAIT_AFTER_LOST = 3.0
MAX_DISTANCE = 4000

X_MIN, X_MAX = 0, 180
Y_MIN, Y_MAX = 0, 90


# =========================
# SERVO CONTROL
# =========================
class ServoController:
    def __init__(self, port, baudrate):
        self.arduino = serial.Serial(port, baudrate, timeout=1)
        time.sleep(3)
        self.x_angle = 90
        self.y_angle = 45
        self.send_angles()

    def send_angles(self):
        cmd = f"{int(self.x_angle)},{int(self.y_angle)}\n"
        self.arduino.write(cmd.encode("utf-8"))

    def move_toward(self, dx, dy, scale=5.0):
        """dx/dy in normalized screen units (-1..1)."""
        self.x_angle = np.clip(self.x_angle + dx * scale, X_MIN, X_MAX)
        self.y_angle = np.clip(self.y_angle + dy * scale, Y_MIN, Y_MAX)
        self.send_angles()

    def scan_step(self, dir_x, dir_y):
        self.x_angle += dir_x * 2
        if self.x_angle >= X_MAX or self.x_angle <= X_MIN:
            dir_x *= -1
            self.y_angle += dir_y * 2
            if self.y_angle >= Y_MAX or self.y_angle <= Y_MIN:
                dir_y *= -1
        self.send_angles()
        time.sleep(SCAN_DELAY)
        return dir_x, dir_y

    def close(self):
        self.arduino.close()


# =========================
# CAMERA UTILITIES
# =========================
def filter_confidence(depth, confidence):
    depth = np.nan_to_num(depth)
    depth[confidence < CONFIDENCE_THRESHOLD] = np.inf
    return depth


def segment_closest_object(depth, confidence):
    """Return (cx, cy, avg_distance, mask) of closest segmented region."""
    depth = filter_confidence(depth, confidence)
    min_dist = np.min(depth)
    if not np.isfinite(min_dist) or min_dist > OBJECT_DISTANCE_THRESHOLD:
        return None, None, None, None

    # Segment pixels within ±5% of min distance
    mask = (depth < min_dist * 1.05)
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None, None, None, None

    cx, cy = int(np.mean(xs)), int(np.mean(ys))
    avg_dist = float(np.mean(depth[mask]))
    return cx, cy, avg_dist, mask


# =========================
# MAIN LOOP
# =========================
def main():
    print("Starting Arducam ToF Tracker...")

    # --- Camera setup
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera.")
        return

    cam.start(ac.FrameType.DEPTH)
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    r = cam.getControl(ac.Control.RANGE)
    info = cam.getCameraInfo()
    width, height = info.width, info.height

    # --- Servo setup
    servo = ServoController(PORT, BAUDRATE)
    dir_x, dir_y = 1, 1
    scanning = True
    last_seen = 0

    cv2.namedWindow("Depth", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frame = cam.requestFrame(2000)
            if frame is None or not isinstance(frame, ac.DepthData):
                continue

            depth = frame.depth_data
            conf = frame.confidence_data
            result_img = (depth * (255.0 / r)).astype(np.uint8)
            RGB = cv2.applyColorMap(result_img, cv2.COLORMAP_RAINBOW)

            # --- Object segmentation
            cx, cy, dist, mask = segment_closest_object(depth, conf)

            if cx is not None:
                scanning = False
                last_seen = time.time()

                # Highlight segmented object
                RGB[mask] = (0, 255, 255)
                cv2.circle(RGB, (cx, cy), 8, (255, 255, 255), 2)

                # Servo tracking (center the object)
                dx = (cx - width / 2) / (width / 2)
                dy = (cy - height / 2) / (height / 2)
                servo.move_toward(dx, dy, scale=5.0)
                print(f"Tracking object at {dist:.1f} mm")

            else:
                # No object visible
                if not scanning and time.time() - last_seen > WAIT_AFTER_LOST:
                    scanning = True
                    print("Object lost → Scanning mode")

                if scanning:
                    dir_x, dir_y = servo.scan_step(dir_x, dir_y)

            cv2.imshow("Depth", RGB)
            cam.releaseFrame(frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("Interrupted.")

    finally:
        servo.close()
        cam.stop()
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
