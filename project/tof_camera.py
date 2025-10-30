import cv2
import numpy as np
import ArducamDepthCamera as ac

class ToFCamera:
    def __init__(self, max_distance=4000, cfg_path=None, confidence_threshold=30):
        """
        Initializes the Arducam ToF camera.
        :param max_distance: Maximum measurable distance in mm (2000 or 4000)
        :param cfg_path: Optional path to configuration file
        :param confidence_threshold: Minimum confidence value to keep a pixel
        """
        self.MAX_DISTANCE = max_distance
        self.cfg_path = cfg_path
        self.confidence_threshold = confidence_threshold

        self.cam = ac.ArducamCamera()
        self.is_open = False
        self.range_value = max_distance

        self._open_camera()
        self._start_camera()

    # ------------------- INTERNAL METHODS -------------------

    def _open_camera(self):
        """Opens the camera connection."""
        if self.cfg_path is not None:
            ret = self.cam.openWithFile(self.cfg_path, 0)
        else:
            ret = self.cam.open(ac.Connection.CSI, 0)

        if ret != 0:
            raise RuntimeError(f"Failed to open camera. Error code: {ret}")

        self.is_open = True

    def _start_camera(self):
        """Starts depth data streaming."""
        ret = self.cam.start(ac.FrameType.DEPTH)
        if ret != 0:
            self.cam.close()
            raise RuntimeError(f"Failed to start camera. Error code: {ret}")

        self.cam.setControl(ac.Control.RANGE, self.MAX_DISTANCE)
        self.range_value = self.cam.getControl(ac.Control.RANGE)

        info = self.cam.getCameraInfo()
        self.width = info.width
        self.height = info.height
        self.device_type = info.device_type

    # ------------------- FRAME CAPTURE -------------------

    def get_frame(self, normalize_depth=True):
        """
        Requests a single frame from the camera.
        :param normalize_depth: If True, depth scaled 0–255 (uint8)
        :return: dict with 'depth', 'confidence', 'amplitude'
        """
        frame = self.cam.requestFrame(2000)
        if frame is None or not isinstance(frame, ac.DepthData):
            return None

        depth = frame.depth_data.copy()
        confidence = frame.confidence_data.copy()
        amplitude = frame.amplitude_data.copy()

        if normalize_depth:
            depth = (depth * (255.0 / self.range_value)).astype(np.uint8)

        self.cam.releaseFrame(frame)

        return {
            "depth": depth,
            "confidence": confidence,
            "amplitude": amplitude
        }

    def get_filtered_depth(self, normalize_depth=True):
        """
        Returns a single depth frame filtered by confidence.
        :param normalize_depth: If True, result scaled 0–255 (uint8)
        :return: numpy array (filtered depth)
        """
        frame = self.get_frame(normalize_depth=False)
        if frame is None:
            return None

        depth = frame["depth"].copy()
        confidence = frame["confidence"]

        depth[confidence < self.confidence_threshold] = 0

        if normalize_depth:
            depth = (depth * (255.0 / self.range_value)).astype(np.uint8)

        return depth

    def get_filtered_frame(self, normalize_depth=True):
        """
        Returns all three images (depth, confidence, amplitude),
        with low-confidence pixels removed from all.
        :param normalize_depth: If True, depth scaled 0–255 (uint8)
        :return: dict with filtered 'depth', 'confidence', 'amplitude'
        """
        frame = self.get_frame(normalize_depth=False)
        if frame is None:
            return None

        depth = frame["depth"].copy()
        confidence = frame["confidence"].copy()
        amplitude = frame["amplitude"].copy()

        # Apply confidence mask
        mask = confidence >= self.confidence_threshold
        depth[~mask] = 0
        amplitude[~mask] = 0

        if normalize_depth:
            depth = (depth * (255.0 / self.range_value)).astype(np.uint8)

        return {
            "depth": depth,
            "confidence": confidence,
            "amplitude": amplitude
        }

    # ------------------- CONTROLS -------------------

    def set_range(self, max_distance):
        """Sets camera measurement range (2000 or 4000 mm)."""
        self.cam.setControl(ac.Control.RANGE, max_distance)
        self.range_value = self.cam.getControl(ac.Control.RANGE)

    def set_confidence_threshold(self, value):
        """Sets confidence filter threshold."""
        self.confidence_threshold = value

    # ------------------- CLEANUP -------------------

    def close(self):
        """Stops and closes the camera safely."""
        if self.is_open:
            self.cam.stop()
            self.cam.close()
            self.is_open = False

    def __del__(self):
        """Ensures safe closure when the object is destroyed."""
        self.close()

    # ------------------- MYSELF -------------------

    @staticmethod
    def denoise_gray_image(depth):
        # Проверяем, что изображение корректное
        if depth is None or len(depth.shape) != 2 or depth.dtype != np.uint8:
            raise ValueError("Ожидается одноканальное 8-битное изображение (np.uint8)")

        # 1. Медианный фильтр — эффективно удаляет точечный шум
        denoised = cv2.medianBlur(depth, 5)

        # 2. Билатеральный фильтр — сохраняет границы при сглаживании шумов
        denoised = cv2.bilateralFilter(denoised, d=7, sigmaColor=75, sigmaSpace=75)

        # 3. Морфологическая операция "открытие" — убирает мелкие пятна
        kernel = np.ones((3, 3), np.uint8)
        denoised = cv2.morphologyEx(denoised, cv2.MORPH_OPEN, kernel)

        return denoised
    
    @staticmethod
    def find_closest_object(depth, threshold_ratio=0.8):
        # Проверяем, что изображение корректное
        if len(depth.shape) != 2 or depth.dtype != np.uint8:
            raise ValueError("Изображение должно быть одноканальным 8-битным")

        # Игнорируем черные пиксели (значение 0)
        non_zero = depth[depth > 0]
        if non_zero.size == 0:
            return None  # нет объектов

        # Определяем порог
        max_val = non_zero.max()
        min_val = non_zero.min()
        threshold_val = max_val - int((max_val - min_val) * threshold_ratio)

        _, binary = cv2.threshold(depth, threshold_val, 255, cv2.THRESH_BINARY)

        # Находим контуры
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None  # объекты не найдены

        # Находим контур с максимальной средней яркостью
        max_mean = -1
        closest_contour = None
        for cnt in contours:
            mask = np.zeros_like(depth)
            cv2.drawContours(mask, [cnt], -1, 255, -1)
            mean_val = cv2.mean(depth, mask=mask)[0]
            if mean_val > max_mean:
                max_mean = mean_val
                closest_contour = cnt

        # Возвращаем координаты bounding box ближайшего объекта
        x, y, w, h = cv2.boundingRect(closest_contour)
        return depth, (x, y, w, h), closest_contour
