import cv2
import numpy as np
import ArducamDepthCamera as ac

import cv2.ximgproc as xip

# MAX_DISTANCE value modifiable  is 2000 or 4000
MAX_DISTANCE=4000


class UserRect:
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

    @property
    def rect(self):
        return (
            self.start_x,
            self.start_y,
            self.end_x - self.start_x,
            self.end_y - self.start_y,
        )

    @property
    def slice(self):
        return (slice(self.start_y, self.end_y), slice(self.start_x, self.end_x))

    @property
    def empty(self):
        return self.start_x == self.end_x and self.start_y == self.end_y


confidence_value = 30
selectRect, followRect = UserRect(), UserRect()


def getPreview(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    mask = confidence > confidence_value
    depth_masked = preview.copy()
    depth_masked[~mask] = 0
    return depth_masked


def on_mouse(event, x, y, flags, param):
    global selectRect, followRect

    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x = x - 4
        selectRect.start_y = y - 4
        selectRect.end_x = x + 4
        selectRect.end_y = y + 4
    else:
        followRect.start_x = x - 4
        followRect.start_y = y - 4
        followRect.end_x = x + 4
        followRect.end_y = y + 4


def on_confidence_changed(value):
    global confidence_value
    confidence_value = value


def usage(argv0):
    print("Usage: python " + argv0 + " [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


def main():
    print("Arducam Depth Camera Demo.")
    print("  SDK version:", ac.__version__)

    cam = ac.ArducamCamera()
    cfg_path = None
    # cfg_path = "file.cfg"

    black_color = (0, 0, 0)
    white_color = (255, 255, 255)

    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, 0)
    else:
        ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)

    r = cam.getControl(ac.Control.RANGE)
    print(f"r = {r}, 255 / r = {255.0 / r}")
    
    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    cv2.namedWindow("result image", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("result image", on_mouse)

    cv2.namedWindow("preview confidence", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preview confidence", on_mouse)

    cv2.namedWindow("rgb", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("rgb", on_mouse)

    cv2.namedWindow("bw", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("bw", on_mouse)

    if info.device_type == ac.DeviceType.HQVGA:
        # Only HQVGA support confidence
        cv2.createTrackbar(
            "confidence", "rgb", confidence_value, 255, on_confidence_changed
        )

    while True:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data # map of distances
            confidence_buf = frame.confidence_data # how reliable each depth measurement
            amplitude_buf = frame.amplitude_data # strength of the returned light signal

            result_image = (depth_buf * (255.0 / r)).astype(np.uint8)

            RGB_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)

            BW_image = getPreview(result_image, confidence_buf)
            RGB_image = getPreview(RGB_image, confidence_buf)

            cv2.normalize(confidence_buf, confidence_buf, 1, 0, cv2.NORM_MINMAX)

            BW_image = cv2.medianBlur(BW_image, ksize=5)
            BW_image = cv2.GaussianBlur(BW_image, (5, 5), 0)
            
            RGB_image = cv2.bilateralFilter(RGB_image, d=9, sigmaColor=50, sigmaSpace=9)

            cv2.imshow("result image", result_image)
            cv2.imshow("preview confidence", confidence_buf)

            cv2.rectangle(RGB_image, followRect.rect, white_color, 1)
            if not selectRect.empty:
                cv2.rectangle(RGB_image, selectRect.rect, black_color, 2)
                print("select Rect distance:", np.mean(depth_buf[selectRect.slice]))

            cv2.imshow("rgb", RGB_image)
            cv2.imshow("bw", BW_image)

            cam.releaseFrame(frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    cam.stop()
    cam.close()


if __name__ == "__main__":
    main()
