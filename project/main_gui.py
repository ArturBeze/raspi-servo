import os
import time
import cv2
import numpy as np
from colour import Color

from tof_camera import ToFCamera
from servo_controller import ServoController

from cv2_processing import combine_images

class AutoTracker:
    save_dir="snapshots"

    def __init__(self):
        self.camera = ToFCamera()
        self.servo = ServoController()

        os.makedirs(AutoTracker.save_dir, exist_ok=True)

        self.x_angle = 90
        self.y_angle = 45
        self.servo.set_x(self.x_angle)
        self.servo.set_y(self.y_angle)

    def run(self):
        prev_time = time.time()
        fps = 0

        while True:
            frame = self.camera.get_frame(normalize_depth=True)

            if frame is None:
                return None

            depth = frame["depth"].copy()
            depth_filtered = self.camera.get_filtered_depth(normalize_depth=True)
            confidence = frame["confidence"].copy()
            amplitude = frame["amplitude"].copy()

            confidence_vis = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            amplitude_vis = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            current_time = time.time()
            delta = current_time - prev_time
            if delta > 0:
                fps = 1 / delta
            prev_time = current_time

            depth_denoised = self.camera.denoise_gray_image(depth_filtered)

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
                filepath = os.path.join(AutoTracker.save_dir, filename)
                cv2.imwrite(filepath, combined)
                print(f"📸 Снимок сохранён: {filepath}")

            elif key == ord('q'):
                break

        self.cleanup()

    def cleanup(self):
        self.camera.close()
        self.servo.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = AutoTracker()
    tracker.run()

"""
import time
import cv2
import numpy as np

from tof_camera import ToFCamera
from servo_controller import ServoController

from cv2_processing import combine_images

class AutoTracker:
    def __init__(self):
        self.camera = ToFCamera()
        self.servo = ServoController()

        # начальные позиции
        self.x_angle = 90
        self.y_angle = 45
        self.servo.set_x(self.x_angle)
        self.servo.set_y(self.y_angle)

        # параметры сканирования
        self.scan_step_x = 10
        self.scan_step_y = 10
        self.scan_delay = 0.3

        # ожидание при потере объекта
        self.lost_timeout = 3

        print("🤖 Автоматическое слежение запущено")

    def scan_environment(self):
        for y in range(0, 91, self.scan_step_y):
            self.servo.set_y(y)
            time.sleep(self.scan_delay)
            scan_range = range(0, 181, self.scan_step_x)
            if (y // self.scan_step_y) % 2 == 1:
                scan_range = reversed(scan_range)

            for x in scan_range:
                self.servo.set_x(x)
                time.sleep(self.scan_delay)

                result = self.custom()
                
                if result is not None:
                    depth, (bx, by, bw, bh), contour = result
                    print(f"✅ Объект найден при X={x}, Y={y}")
                    return depth, (bx, by, bw, bh), contour
        return None

    def track_object(self, bbox):
        x, y, w, h = bbox
        obj_cx = x + w // 2
        obj_cy = y + h // 2

        frame_w = self.camera.width
        frame_h = self.camera.height

        err_x = obj_cx - frame_w // 2
        err_y = obj_cy - frame_h // 2

        # чувствительность движения
        sensitivity_x = 0.05
        sensitivity_y = 0.05

        # изменение углов
        self.x_angle -= err_x * sensitivity_x
        self.y_angle -= err_y * sensitivity_y

        # ограничения
        self.x_angle = max(0, min(180, self.x_angle))
        self.y_angle = max(0, min(90, self.y_angle))

        self.servo.set_x(self.x_angle)
        self.servo.set_y(self.y_angle)

    def run(self):
        object_last_seen = 0
        object_lost = True

        while True:
            result = self.custom()
            
            if result is not None:
                depth, (x, y, w, h), contour = result
                self.track_object((x, y, w, h))
                object_last_seen = time.time()
                object_lost = False

                # визуализация
                output = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.imshow("Tracking", output)

            else:
                if not object_lost and (time.time() - object_last_seen) > self.lost_timeout:
                    print("❌ Объект потерян. Возврат к сканированию...")
                    object_lost = True

                if object_lost:
                    scan_result = self.scan_environment()
                    if scan_result:
                        depth, (x, y, w, h), contour = scan_result
                        self.track_object((x, y, w, h))
                        object_last_seen = time.time()
                        object_lost = False
                                  
                        # визуализация      
                        output = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
                        cv2.rectangle(output, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        cv2.imshow("Tracking", output)


            if cv2.waitKey(1) & 0xFF == 27:  # ESC для выхода
                break

        self.cleanup()

    def cleanup(self):
        print("🧹 Завершение работы...")
        self.camera.close()
        self.servo.close()
        cv2.destroyAllWindows()

    def custom(self):
        frame = self.camera.get_filtered_frame(normalize_depth=True)

        if frame is None:
            return None

        depth = frame["depth"].copy()
        confidence = frame["confidence"].copy()
        amplitude = frame["amplitude"].copy()

        confidence_vis = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        amplitude_vis = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                
        depth[depth == 0] = 255
        depth = 255 - self.camera.denoise_gray_image(depth)
        depth[depth < 128] = 0

        return self.camera.find_closest_object(depth)  


if __name__ == "__main__":
    tracker = AutoTracker()
    tracker.run()
"""
