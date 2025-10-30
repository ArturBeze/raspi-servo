import os
import time
import cv2
import numpy as np
from colour import Color

from tof_camera import ToFCamera
from servo_controller import ServoController

from cv2_processing import combine_images

def show_camera_stream(save_dir="snapshots"):

    os.makedirs(save_dir, exist_ok=True)

    camera = ToFCamera()

    print("Нажмите 's', чтобы сохранить снимок")
    print("Нажмите 'q', чтобы выйти")

    prev_time = time.time()
    fps = 0

    while True:
        frame = camera.get_frame(normalize_depth=True)

        if frame is None:
            return None

        depth = frame["depth"].copy()
        depth_filtered = camera.get_filtered_depth(normalize_depth=True)
        confidence = frame["confidence"].copy()
        amplitude = frame["amplitude"].copy()

        confidence_vis = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        amplitude_vis = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        current_time = time.time()
        delta = current_time - prev_time
        if delta > 0:
            fps = 1 / delta
        prev_time = current_time

        depth_denoised = camera.denoise_gray_image(depth_filtered)

        images = [
            #depth, 
            #confidence_vis, 
            #amplitude_vis, 
            depth_filtered,
            depth_denoised
        ]

        captions = [
            #f"Depth. FPS: {fps:.1f}", 
            #f"Confidence" , 
            #f"Amplitude", 
            f"Filtered",
            f"Denoised"
        ]

        border_colors = [
            #(0, 0, 255),
            #(0, 255, 0),
            #(255, 0, 0),
            (255, 255, 0),
            (0, 255, 255)
        ]

        combined = combine_images(
            images=images,
            layout='horizontal',
            spacing=10,
            captions=captions,
            bg_color=(20, 20, 20),
            border_colors=border_colors,
            border_thickness=4
        )

        cv2.imshow("Combined", combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{timestamp}.jpg"
            filepath = os.path.join(save_dir, filename)
            cv2.imwrite(filepath, combined)
            print(f"📸 Снимок сохранён: {filepath}")

        elif key == ord('q'):
            break

    camera.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    show_camera_stream()
