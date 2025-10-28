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
