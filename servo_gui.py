import tkinter as tk
from pyfirmata import Arduino, util
import time

# ----- Configuration -----
PORT = '/dev/ttyACM0'  # Change if your Arduino is on another port
SERVO_X_PIN = 9
SERVO_Y_PIN = 10
START_ANGLE = 90  # Start position in degrees

# ----- Setup -----
board = Arduino(PORT)
time.sleep(2)  # Allow Firmata to initialize

# Enable servo mode
board.digital[SERVO_X_PIN].mode = 4  # SERVO
board.digital[SERVO_Y_PIN].mode = 4  # SERVO

# Move both servos to start position
board.digital[SERVO_X_PIN].write(START_ANGLE)
board.digital[SERVO_Y_PIN].write(START_ANGLE)

# ----- GUI -----
root = tk.Tk()
root.title("Servo Control Panel")
root.geometry("400x250")
root.configure(bg="#f0f0f0")

tk.Label(root, text="Servo Motor Control", font=("Arial", 16, "bold"), bg="#f0f0f0").pack(pady=10)

# Servo control functions
def move_x(angle):
    angle = int(float(angle))
    board.digital[SERVO_X_PIN].write(angle)

def move_y(angle):
    angle = int(float(angle))
    board.digital[SERVO_Y_PIN].write(angle)

# X Axis slider
frame_x = tk.Frame(root, bg="#f0f0f0")
frame_x.pack(pady=10)
tk.Label(frame_x, text="X Axis", font=("Arial", 12), bg="#f0f0f0").pack()
x_slider = tk.Scale(frame_x, from_=0, to=180, orient="horizontal", length=300, command=move_x)
x_slider.set(START_ANGLE)
x_slider.pack()

# Y Axis slider
frame_y = tk.Frame(root, bg="#f0f0f0")
frame_y.pack(pady=10)
tk.Label(frame_y, text="Y Axis", font=("Arial", 12), bg="#f0f0f0").pack()
y_slider = tk.Scale(frame_y, from_=0, to=180, orient="horizontal", length=300, command=move_y)
y_slider.set(START_ANGLE)
y_slider.pack()

# Safe close
def on_close():
    board.exit()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
