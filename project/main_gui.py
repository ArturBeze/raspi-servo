import os
import time
import cv2
import numpy as np
from colour import Color

from tof_camera import ToFCamera
from servo_controller import ServoController

from cv2_processing import trim_image, combine_images

def nothing(x):
    pass

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

        self.scan_step_x = 10
        self.scan_step_y = 10
        self.scan_delay = 0.3

        self.lost_timeout = 3

        self.dir_x = 1
        self.dir_y = 1
        
        self.min_x = 60
        self.max_x = 120
        self.step_x = 5
        self.min_y = 30
        self.max_y = 60
        self.step_y = 5



    def track_object(self, bbox):
        x, y, w, h = bbox
        obj_cx = x + w // 2
        obj_cy = y + h // 2

        frame_w = self.camera.width
        frame_h = self.camera.height

        err_x = obj_cx - frame_w // 2
        err_y = obj_cy - frame_h // 2

        sensitivity_x = 0.05
        sensitivity_y = 0.05

        self.x_angle -= err_x * sensitivity_x
        self.y_angle -= err_y * sensitivity_y

        self.x_angle = max(60, min(120, self.x_angle))
        self.y_angle = max(30, min(60, self.y_angle))

        self.servo.set_x(self.x_angle)
        self.servo.set_y(self.y_angle)
        
        return obj_cx, obj_cy

    def step(self):
        new_x = self.x_angle + self.dir_x * self.step_x

        if self.min_x <= new_x <= self.max_x:
            self.x_angle = new_x
        else:
            self.dir_x *= -1
            new_y = self.y_angle + self.dir_y * self.step_y

            if not (self.min_y <= new_y <= self.max_y):
                self.dir_y *= -1
                new_y = self.y_angle + self.dir_y * self.step_y

            self.y_angle = new_y

        self.servo.set_x(self.x_angle)
        self.servo.set_y(self.y_angle)



    def run(self):
        cv2.namedWindow('Controls', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Controls', 400, 300)

        # Пороговое соотношение камеры
        cv2.createTrackbar('Threshold camera', 'Controls', 32, 100, nothing)

        # Медианный фильтр (размер ядра)
        cv2.createTrackbar('Median ksize', 'Controls', 5, 15, nothing)

        # Bilateral filter параметры
        cv2.createTrackbar('Bilateral d', 'Controls', 9, 15, nothing)
        cv2.createTrackbar('SigmaColor', 'Controls', 75, 150, nothing)
        cv2.createTrackbar('SigmaSpace', 'Controls', 75, 150, nothing)

        # Морфологическая очистка
        cv2.createTrackbar('Morph ksize', 'Controls', 3, 10, nothing)

        # Пороговое соотношение
        cv2.createTrackbar('Threshold ratio', 'Controls', 80, 100, nothing)

        object_last_seen = 0
        object_lost = True

        prev_time = time.time()
        fps = 0

        while True:
            threshold_camera = cv2.getTrackbarPos('Threshold camera', 'Controls')
            median_ksize = cv2.getTrackbarPos('Median ksize', 'Controls')
            bilateral_d = cv2.getTrackbarPos('Bilateral d', 'Controls')
            sigma_color = cv2.getTrackbarPos('SigmaColor', 'Controls')
            sigma_space = cv2.getTrackbarPos('SigmaSpace', 'Controls')
            morph_ksize = cv2.getTrackbarPos('Morph ksize', 'Controls')
            threshold_ratio = cv2.getTrackbarPos('Threshold ratio', 'Controls')

            median_ksize = max(1, median_ksize | 1)
            bilateral_d = max(1, bilateral_d)
            morph_ksize = max(1, morph_ksize | 1)
            threshold_ratio = max(0.1, threshold_ratio / 100.0)

            self.camera.set_confidence_threshold(threshold_camera)

            #frame = self.camera.get_frame(normalize_depth=True)
            frame = self.camera.get_filtered_frame(normalize_depth=True)

            if frame is None:
                return None

            #depth = frame["depth"].copy()
            #depth_filtered = self.camera.get_filtered_depth(normalize_depth=True)
            depth_filtered = frame["depth"].copy()            
            confidence = frame["confidence"].copy()
            amplitude = frame["amplitude"].copy()

            confidence_vis = cv2.normalize(confidence, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            amplitude_vis = cv2.normalize(amplitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

            current_time = time.time()
            delta = current_time - prev_time
            if delta > 0:
                fps = 1 / delta
            prev_time = current_time

            depth_denoised = ToFCamera.apply_denoise(
                depth_filtered,
                median_ksize=median_ksize,
                bilateral_d=bilateral_d,
                sigma_color=sigma_color,
                sigma_space=sigma_space,
                morph_ksize=morph_ksize
            )

            images = [
                #depth, 
                #confidence_vis, 
                #amplitude_vis, 
                depth_filtered,
                depth_denoised,
            ]

            captions = [
                #f"D. FPS: {fps:.1f}", 
                #f"Confidence" , 
                #f"Amplitude", 
                f"D. Filtered. FPS: {fps:.1f}",
                f"D. Denoised",
            ]

            border_colors = [
                #(0, 0, 255),
                #(0, 255, 0),
                #(255, 0, 0),
                (255, 255, 0),
                (0, 255, 255),
            ]

            depth_temp = depth_filtered.copy()
            depth_temp[depth_temp == 0] = 255
            depth_contoured = ToFCamera.apply_denoise(
                depth_temp,
                median_ksize=median_ksize,
                bilateral_d=bilateral_d,
                sigma_color=sigma_color,
                sigma_space=sigma_space,
                morph_ksize=morph_ksize
            )
            depth_contoured = 255 - depth_contoured
            depth_contoured = trim_image(depth_contoured, 196, 256)

            result = ToFCamera.find_closest_object(depth_contoured, threshold_ratio)

            if result:
                (x, y, w, h), contour = result
                depth_contoured = cv2.cvtColor(depth_contoured, cv2.COLOR_GRAY2BGR)
                cv2.rectangle(depth_contoured, (x, y), (x + w, y + h), (0, 0, 255), 2)

                images.append(depth_contoured)
                captions.append(f"D. Contoured")
                border_colors.append((255, 0, 255))

                obj_cx, obj_cy = self.track_object((x, y, w, h))
                cv2.line(depth_contoured, (obj_cx, obj_cy), (self.camera.width // 2, self.camera.height // 2), (255, 0, 0), 5)

                object_last_seen = time.time()
                print("✅ Объект найден. Завершение сканирования...")
                object_lost = False
            else:
                if not object_lost and (time.time() - object_last_seen) > self.lost_timeout:
                    print("❌ Объект потерян. Возврат к сканированию...")
                    object_lost = True

                if object_lost:
                    self.step()
                    pass

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
