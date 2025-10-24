import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import time
import threading
import cv2
import numpy as np
import ArducamDepthCamera as ac

# ==========================
# SERIAL / ARDUINO
# ==========================
PORT = "/dev/ttyACM0"
BAUD = 9600

def get_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"Device: {port.device}")
        print(f"Description: {port.description}")
        print(f"Hardware ID: {port.hwid}")
        print("-" * 30)

get_ports()

try:
    arduino = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print("✅ Connected to Arduino")
except serial.SerialException:
    arduino = None
    print("⚠️ Could not open serial port. Check connection.")

def send_to_arduino(x_val, y_val):
    if arduino and arduino.is_open:
        line = f"{x_val},{y_val}\n"
        arduino.write(line.encode("utf-8"))

# ==========================
# CAMERA / ARDUCAM
# ==========================

MAX_DISTANCE = 4000
confidence_value = 30

def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < confidence_value] = (0, 0, 0)
    return preview

def getPreviewBW(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < confidence_value] = 0
    return preview

def on_confidence_changed(value):
    global confidence_value
    confidence_value = value

def camera_loop():
    print("Starting Arducam Depth Camera...")
    cam = ac.ArducamCamera()

    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("❌ Failed to open camera. Error:", ret)
        return

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("❌ Failed to start camera. Error:", ret)
        cam.close()
        return

    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    r = cam.getControl(ac.Control.RANGE)
    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    cv2.namedWindow("rgb", cv2.WINDOW_AUTOSIZE)
    if info.device_type == ac.DeviceType.VGA:
        cv2.createTrackbar("confidence", "rgb", confidence_value, 255, on_confidence_changed)

    while True:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            confidence_buf = frame.confidence_data
            amplitude_buf = frame.amplitude_data

            result_image = (depth_buf * (255.0 / r)).astype(np.uint8)
            RGB_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
            RGB_image = getPreviewRGB(RGB_image, confidence_buf)
            BW_image = getPreviewBW(result_image, confidence_buf)

            cv2.normalize(confidence_buf, confidence_buf, 1, 0, cv2.NORM_MINMAX)

            cv2.imshow("preview_confidence", confidence_buf)
            cv2.imshow("rgb", RGB_image)
            cv2.imshow("bw", BW_image)

            cam.releaseFrame(frame)

        if cv2.waitKey(1) == ord("q"):
            break

    cam.stop()
    cam.close()
    cv2.destroyAllWindows()

# ==========================
# TKINTER GUI
# ==========================

root = tk.Tk()
root.title("Servo Control + Depth Camera")

main = ttk.Frame(root, padding=20)
main.pack()

x_value = tk.IntVar()
y_value = tk.IntVar()

ttk.Label(main, text="X Axis").grid(row=0, column=0, pady=10, sticky="e")
x_slider = ttk.Scale(main, from_=0, to=180, orient="horizontal", length=300, variable=x_value)
x_slider.grid(row=0, column=1, padx=10)
x_label = ttk.Label(main, text="90")
x_label.grid(row=0, column=2, padx=10)

ttk.Label(main, text="Y Axis").grid(row=1, column=0, pady=10, sticky="e")
y_slider = ttk.Scale(main, from_=0, to=135, orient="horizontal", length=300, variable=y_value)
y_slider.grid(row=1, column=1, padx=10)
y_label = ttk.Label(main, text="45")
y_label.grid(row=1, column=2, padx=10)

def update_servo(event=None):
    x = int(x_slider.get())
    y = int(y_slider.get())
    x_label.config(text=f"{x}")
    y_label.config(text=f"{y}")
    send_to_arduino(x, y)

for slider in (x_slider, y_slider):
    slider.bind("<B1-Motion>", update_servo)
    slider.bind("<ButtonRelease-1>", update_servo)

def init_position():
    x_slider.set(90)
    y_slider.set(45)
    update_servo()

root.after(1500, init_position)

# ==========================
# START CAMERA IN THREAD
# ==========================
camera_thread = threading.Thread(target=camera_loop, daemon=True)
camera_thread.start()

# ==========================
# RUN GUI
# ==========================
root.mainloop()

if arduino:
    arduino.close()
