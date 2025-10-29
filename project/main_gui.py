# gui_tof_servo.py
# Tkinter GUI that combines servo_controller.py and tof_camera.py
# - Manual / Scanning toggle (scan indicator)
# - X/Y servo sliders with live numeric labels
# - min_depth / max_depth sliders (defaults 300 / 600 mm) with overlay on image
# - dropdown to choose displayed image: raw depth, filtered depth, confidence
# - confidence_threshold slider
# - in scanning mode manual controls disabled
#
# Usage: run alongside servo_controller.py and tof_camera.py
# Requires: opencv (cv2), numpy, tkinter, threading

import threading
import time
import queue
import sys
import traceback

import tkinter as tk
from tkinter import ttk, StringVar, IntVar, DoubleVar

import cv2
import numpy as np

# try import hardware modules; fall back to simulation if they fail
SIMULATE_SERVO = False
SIMULATE_CAMERA = False
try:
    from servo_controller import ServoController
except Exception as e:
    print("⚠️  ServoController import failed — running in simulation mode. Error:", e)
    SIMULATE_SERVO = True

try:
    from tof_camera import ToFCamera
except Exception as e:
    print("⚠️  ToFCamera import failed — running in simulation mode. Error:", e)
    SIMULATE_CAMERA = True

# --- Simulation classes (used when hardware not available) ---
class DummyServo:
    def __init__(self):
        self.x = 90
        self.y = 45
        print("🔧 DummyServo initialized")

    def set_x(self, angle):
        self.x = max(0, min(180, int(angle)))
        print(f"[SIM] X set to {self.x}")

    def set_y(self, angle):
        self.y = max(0, min(90, int(angle)))
        print(f"[SIM] Y set to {self.y}")

    def close(self):
        print("[SIM] Servo closed")

class DummyCamera:
    def __init__(self, width=320, height=240):
        self.width = width
        self.height = height
        self.range_value = 4000
        self.confidence_threshold = 30
        print("🔧 DummyCamera initialized")

        # create synthetic depth & confidence (gradient + noise)
        xv = np.linspace(0, 1, self.width)[None, :].astype(np.float32)
        yv = np.linspace(0, 1, self.height)[:, None].astype(np.float32)
        self.base_depth = ((xv + yv) * (self.range_value / 2)).astype(np.float32)

    def get_frame(self, normalize_depth=True):
        # build synthetic frame
        noise = (np.random.randn(self.height, self.width) * 20).astype(np.float32)
        depth = np.clip(self.base_depth + noise, 0, self.range_value).astype(np.float32)
        confidence = (np.random.rand(self.height, self.width) * 255).astype(np.uint8)
        amplitude = np.zeros_like(confidence)
        if normalize_depth:
            depth_norm = (depth * (255.0 / self.range_value)).astype(np.uint8)
        else:
            depth_norm = depth.astype(np.float32)
        return {"depth": depth_norm, "confidence": confidence, "amplitude": amplitude}

    def get_filtered_depth(self, normalize_depth=True):
        f = self.get_frame(normalize_depth=True)
        d = f["depth"].copy()
        c = f["confidence"]
        d[c < self.confidence_threshold] = 0
        return d

    def get_filtered_frame(self, normalize_depth=True):
        f = self.get_frame(normalize_depth=True)
        d = f["depth"].copy()
        c = f["confidence"].copy()
        mask = c >= self.confidence_threshold
        d[~mask] = 0
        f["depth"] = d
        f["amplitude"] = np.zeros_like(d, dtype=np.uint8)
        return f

    def set_confidence_threshold(self, v):
        self.confidence_threshold = v

    def close(self):
        print("[SIM] Camera closed")

# --- Main GUI class ---
class ToFServoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ToF Camera + Servo Controller")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Devices
        self.servo = None
        self.camera = None
        self.camera_thread = None
        self.running = threading.Event()
        self.frame_queue = queue.Queue(maxsize=2)

        # default params
        self.x_angle = IntVar(value=90)
        self.y_angle = IntVar(value=45)
        self.mode_scan = IntVar(value=0)  # 0 = manual, 1 = scanning
        self.min_depth = IntVar(value=300)
        self.max_depth = IntVar(value=600)
        self.confidence_threshold = IntVar(value=30)
        self.image_choice = StringVar(value="depth_raw")  # depth_raw, depth_filtered, confidence

        # Setup hardware (or simulation)
        self.setup_devices()

        # Build UI
        self.build_ui()

        # Start camera thread
        self.running.set()
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

        # Start scan thread (handles servo motion in scanning mode)
        self.scan_thread = threading.Thread(target=self.scan_loop, daemon=True)
        self.scan_thread.start()

    def setup_devices(self):
        # Servo
        if not SIMULATE_SERVO:
            try:
                self.servo = ServoController()
            except Exception as e:
                print("⚠️  Failed to initialize ServoController — entering simulation. Error:", e)
                traceback.print_exc()
                self.servo = DummyServo()
        else:
            self.servo = DummyServo()

        # Camera
        if not SIMULATE_CAMERA:
            try:
                self.camera = ToFCamera()
                # ensure camera has attributes width/height
                if not hasattr(self.camera, "width"):
                    # try to fetch a frame to get sizes
                    f = self.camera.get_frame()
                    if f is not None:
                        self.camera.width = f["depth"].shape[1]
                        self.camera.height = f["depth"].shape[0]
            except Exception as e:
                print("⚠️  Failed to initialize ToFCamera — entering simulation. Error:", e)
                traceback.print_exc()
                self.camera = DummyCamera()
        else:
            self.camera = DummyCamera()

        # ensure camera's confidence threshold setter is available
        if not hasattr(self.camera, "set_confidence_threshold"):
            def set_conf(v):
                setattr(self.camera, "confidence_threshold", v)
            self.camera.set_confidence_threshold = set_conf

    def build_ui(self):
        frm = ttk.Frame(self.root, padding=8)
        frm.grid(row=0, column=0, sticky="nsew")

        # Mode toggle (manual / scanning) with visual indicator
        mode_frame = ttk.LabelFrame(frm, text="Mode", padding=6)
        mode_frame.grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        self.mode_toggle = ttk.Checkbutton(mode_frame, text="Scanning mode", variable=self.mode_scan,
                                           command=self.on_mode_change)
        self.mode_toggle.grid(row=0, column=0, sticky="w")

        self.mode_indicator = ttk.Label(mode_frame, text="Manual active", background="lightgray")
        self.mode_indicator.grid(row=0, column=1, padx=8, sticky="e")

        # Servo sliders
        servo_frame = ttk.LabelFrame(frm, text="Servo control (manual)", padding=6)
        servo_frame.grid(row=1, column=0, sticky="ew", padx=4, pady=4)

        ttk.Label(servo_frame, text="X angle (0-180):").grid(row=0, column=0, sticky="w")
        self.slider_x = ttk.Scale(servo_frame, from_=0, to=180, orient="horizontal",
                                  variable=self.x_angle, command=self.on_x_change)
        self.slider_x.grid(row=0, column=1, sticky="ew", padx=4)
        self.label_x_val = ttk.Label(servo_frame, textvariable=self.x_angle, width=4)
        self.label_x_val.grid(row=0, column=2, sticky="w", padx=4)

        ttk.Label(servo_frame, text="Y angle (0-90):").grid(row=1, column=0, sticky="w")
        self.slider_y = ttk.Scale(servo_frame, from_=0, to=90, orient="horizontal",
                                  variable=self.y_angle, command=self.on_y_change)
        self.slider_y.grid(row=1, column=1, sticky="ew", padx=4)
        self.label_y_val = ttk.Label(servo_frame, textvariable=self.y_angle, width=4)
        self.label_y_val.grid(row=1, column=2, sticky="w", padx=4)

        servo_frame.columnconfigure(1, weight=1)

        # Depth min/max and overlay info
        depth_frame = ttk.LabelFrame(frm, text="Depth range (mm)", padding=6)
        depth_frame.grid(row=2, column=0, sticky="ew", padx=4, pady=4)

        ttk.Label(depth_frame, text="Min depth:").grid(row=0, column=0, sticky="w")
        self.slider_min = ttk.Scale(depth_frame, from_=0, to=4000, orient="horizontal",
                                    variable=self.min_depth, command=self.on_min_change)
        self.slider_min.grid(row=0, column=1, sticky="ew", padx=4)
        self.label_min_val = ttk.Label(depth_frame, textvariable=self.min_depth, width=6)
        self.label_min_val.grid(row=0, column=2, sticky="w", padx=4)

        ttk.Label(depth_frame, text="Max depth:").grid(row=1, column=0, sticky="w")
        self.slider_max = ttk.Scale(depth_frame, from_=0, to=4000, orient="horizontal",
                                    variable=self.max_depth, command=self.on_max_change)
        self.slider_max.grid(row=1, column=1, sticky="ew", padx=4)
        self.label_max_val = ttk.Label(depth_frame, textvariable=self.max_depth, width=6)
        self.label_max_val.grid(row=1, column=2, sticky="w", padx=4)

        depth_frame.columnconfigure(1, weight=1)

        # Image choice and confidence threshold
        view_frame = ttk.LabelFrame(frm, text="View / Filtering", padding=6)
        view_frame.grid(row=3, column=0, sticky="ew", padx=4, pady=4)

        ttk.Label(view_frame, text="Show image:").grid(row=0, column=0, sticky="w")
        choices = {"Raw depth": "depth_raw", "Filtered depth": "depth_filtered", "Confidence": "confidence"}
        self.image_menu = ttk.OptionMenu(view_frame, self.image_choice, self.image_choice.get(), *choices.keys(),
                                         command=self.on_image_choice_change)
        # we need to map displayed label to internal value; easier to replace OptionMenu items with keys
        # but keep mapping by storing reverse mapping
        self.view_map_label_to_val = {k: v for k, v in choices.items()}
        # replace with actual keys in menu
        menu = self.image_menu["menu"]
        menu.delete(0, "end")
        for k in choices.keys():
            menu.add_command(label=k, command=lambda v=k: (self.image_choice.set(self.view_map_label_to_val[v]), self.on_image_choice_change(v)))
        # show default label correctly
        # find default label for current value:
        def_val_label = [k for k, v in self.view_map_label_to_val.items() if v == self.image_choice.get()]
        if def_val_label:
            self.image_choice.set(self.view_map_label_to_val[def_val_label[0]])
        self.image_menu.grid(row=0, column=1, sticky="ew", padx=4)

        ttk.Label(view_frame, text="Confidence threshold:").grid(row=1, column=0, sticky="w")
        self.slider_conf = ttk.Scale(view_frame, from_=0, to=255, orient="horizontal",
                                     variable=self.confidence_threshold, command=self.on_conf_change)
        self.slider_conf.grid(row=1, column=1, sticky="ew", padx=4)
        self.label_conf_val = ttk.Label(view_frame, textvariable=self.confidence_threshold, width=6)
        self.label_conf_val.grid(row=1, column=2, sticky="w", padx=4)

        view_frame.columnconfigure(1, weight=1)

        # Buttons row
        btn_frame = ttk.Frame(frm, padding=4)
        btn_frame.grid(row=4, column=0, sticky="ew")
        self.btn_close = ttk.Button(btn_frame, text="Close", command=self.on_close)
        self.btn_close.grid(row=0, column=0, sticky="w")

        # initial enabling/disabling
        self.on_mode_change()

        # info label
        self.info_label = ttk.Label(frm, text="OpenCV window will display images (press ESC or close window to exit).")
        self.info_label.grid(row=5, column=0, sticky="w", pady=(8,0))

    # ---------------- widget callbacks ----------------
    def on_mode_change(self):
        scanning = bool(self.mode_scan.get())
        if scanning:
            self.mode_indicator.config(text="Scanning active", background="#8f8")  # light green
            # disable manual controls
            self.slider_x.state(["disabled"])
            self.slider_y.state(["disabled"])
        else:
            self.mode_indicator.config(text="Manual active", background="lightgray")
            self.slider_x.state(["!disabled"])
            self.slider_y.state(["!disabled"])

    def on_x_change(self, val):
        # set servo X if manual mode
        if self.mode_scan.get() == 0:
            try:
                angle = int(float(val))
                # send to hardware
                try:
                    self.servo.set_x(angle)
                except Exception as e:
                    print("Error sending X to servo:", e)
            except Exception:
                pass  # ignore transition events

    def on_y_change(self, val):
        if self.mode_scan.get() == 0:
            try:
                angle = int(float(val))
                try:
                    self.servo.set_y(angle)
                except Exception as e:
                    print("Error sending Y to servo:", e)
            except Exception:
                pass

    def on_min_change(self, val):
        # just update var; overlay updated in camera image
        pass

    def on_max_change(self, val):
        pass

    def on_conf_change(self, val):
        try:
            v = int(float(val))
            self.camera.set_confidence_threshold(v)
        except Exception as e:
            print("Failed to set camera confidence threshold:", e)

    def on_image_choice_change(self, label_or_val=None):
        # If called via label in menu, we set image_choice already
        # No further action required — camera loop will read image_choice.get()
        pass

    # ---------------- camera & display ----------------
    def camera_loop(self):
        """
        Thread that continuously fetches frames from camera and shows them using OpenCV.
        Runs until self.running is cleared.
        """
        win_name = "ToF Camera View"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        try:
            while self.running.is_set():
                try:
                    choice = self.image_choice.get()
                    # note: we saved internal values in view_map_label_to_val; image_choice stores the internal value
                    # image_choice could be "depth_raw", "depth_filtered", "confidence"
                    if choice == "depth_raw" or choice == "" :
                        frame = self.camera.get_frame(normalize_depth=True)
                        if frame is None:
                            time.sleep(0.02)
                            continue
                        disp = frame["depth"]
                        disp = self.prepare_depth_for_display(disp)
                    elif choice == "depth_filtered":
                        # Use camera's get_filtered_depth if present
                        if hasattr(self.camera, "get_filtered_depth"):
                            disp = self.camera.get_filtered_depth(normalize_depth=True)
                            if disp is None:
                                time.sleep(0.02)
                                continue
                            disp = self.prepare_depth_for_display(disp)
                        else:
                            f = self.camera.get_frame(normalize_depth=True)
                            d = f["depth"]
                            c = f["confidence"]
                            d[c < self.confidence_threshold.get()] = 0
                            disp = self.prepare_depth_for_display(d)
                    elif choice == "confidence":
                        f = self.camera.get_frame(normalize_depth=True)
                        if f is None:
                            time.sleep(0.02)
                            continue
                        conf = f["confidence"]
                        # scale confidence to 0-255 if not already uint8
                        if conf.dtype != np.uint8:
                            conf = np.clip(conf, 0, 255).astype(np.uint8)
                        disp = conf
                        disp = cv2.equalizeHist(disp)
                    else:
                        # fallback raw
                        frame = self.camera.get_frame(normalize_depth=True)
                        if frame is None:
                            time.sleep(0.02)
                            continue
                        disp = frame["depth"]
                        disp = self.prepare_depth_for_display(disp)

                    # overlay min/max text on the image
                    disp_bgr = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
                    h, w = disp.shape[:2]
                    # overlay top-left
                    txt = f"min:{self.min_depth.get()}mm max:{self.max_depth.get()}mm conf_thr:{self.confidence_threshold.get()}"
                    cv2.putText(disp_bgr, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

                    # If scanning mode — show indicator overlay
                    if self.mode_scan.get():
                        cv2.putText(disp_bgr, "SCANNING ACTIVE", (8, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2, cv2.LINE_AA)

                    cv2.imshow(win_name, disp_bgr)
                    key = cv2.waitKey(1) & 0xFF
                    # allow closing by ESC in OpenCV window
                    if key == 27:
                        # emulate close
                        self.root.after(0, self.on_close)
                        break

                    # small sleep
                    time.sleep(0.01)

                except Exception as e:
                    print("Error in camera loop:", e)
                    traceback.print_exc()
                    time.sleep(0.2)
                    continue
        finally:
            try:
                cv2.destroyWindow(win_name)
            except Exception:
                pass

    def prepare_depth_for_display(self, depth_uint8):
        """Ensure a uint8 grayscale image is returned for display."""
        if depth_uint8 is None:
            return np.zeros((self.camera.height, self.camera.width), dtype=np.uint8)
        if depth_uint8.dtype != np.uint8:
            arr = np.clip(depth_uint8, 0, 255).astype(np.uint8)
        else:
            arr = depth_uint8
        return arr

    # ---------------- scanning behavior ----------------
    def scan_loop(self):
        """
        Thread that performs servo sweeping when scanning mode is active.
        Moves X from 0->180 and Y from 0->90 in a simple pattern.
        """
        x_dir = 1
        y_dir = 1
        x = self.x_angle.get()
        y = self.y_angle.get()
        while True:
            if not self.running.is_set():
                break
            if self.mode_scan.get():
                # step amounts
                x += x_dir * 2  # coarse steps
                if x > 180:
                    x = 180
                    x_dir = -1
                    y += 5 * y_dir
                elif x < 0:
                    x = 0
                    x_dir = 1
                    y += 5 * y_dir

                if y > 90:
                    y = 90
                    y_dir = -1
                elif y < 0:
                    y = 0
                    y_dir = 1

                # apply to servo
                try:
                    self.servo.set_x(int(x))
                    self.servo.set_y(int(y))
                except Exception as e:
                    print("Scan: servo set failed:", e)

                # update UI variables (must run in main thread via after)
                self.root.after(0, lambda xv=int(x): self.x_angle.set(xv))
                self.root.after(0, lambda yv=int(y): self.y_angle.set(yv))

                time.sleep(0.12)
            else:
                # not scanning -> small sleep
                time.sleep(0.12)

    # ---------------- cleanup ----------------
    def on_close(self):
        if self.running.is_set():
            self.running.clear()
            # small delay to let threads finish
            time.sleep(0.2)

        # close camera
        try:
            if self.camera:
                self.camera.close()
        except Exception as e:
            print("Failed closing camera:", e)

        # close servo
        try:
            if self.servo:
                self.servo.close()
        except Exception as e:
            print("Failed closing servo:", e)

        # destroy Tk and OpenCV windows
        try:
            self.root.destroy()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        # exit program
        try:
            sys.exit(0)
        except SystemExit:
            pass

# --- Run GUI ---
def main():
    root = tk.Tk()
    app = ToFServoGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
