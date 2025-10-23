import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import time

def get_ports():
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found.")
        return
        
    for port in ports:
            print(f"Device: {port.device}")
            print(f"Description: {port.description}")
            print(f"Hardware ID: {port.hwid}")
            if port.manufacturer:
                print(f"Manufacturer: {port.manufacturer}")
            if port.product:
                print(f"Product: {port.product}")
            if port.serial_number:
                print(f"Serial Number: {port.serial_number}")
            print("-" * 30)
            
get_ports()

# --- Serial connection ---
PORT = "/dev/ttyACM0"  # or "/dev/ttyUSB0"
BAUD = 9600

try:
    arduino = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # wait for Arduino reset
except serial.SerialException:
    arduino = None
    print("?? Could not open serial port. Check the connection and port name.")

# --- Function to send data ---
def send_to_arduino(x_val, y_val):
    if arduino and arduino.is_open:
        line = f"{x_val},{y_val}\n"
        arduino.write(line.encode("utf-8"))

# --- GUI setup ---
root = tk.Tk()
root.title("Servo Control GUI")

main = ttk.Frame(root, padding=20)
main.pack()

# --- Variables ---
x_value = tk.IntVar()
y_value = tk.IntVar()

# --- Labels ---
ttk.Label(main, text="X Axis").grid(row=0, column=0, pady=10, sticky="e")
x_slider = ttk.Scale(main, from_=0, to=180, orient="horizontal", length=300, variable=x_value)
x_slider.grid(row=0, column=1, padx=10)
x_label = ttk.Label(main, text="90")
x_label.grid(row=0, column=2, padx=10)

ttk.Label(main, text="Y Axis").grid(row=1, column=0, pady=10, sticky="e")
y_slider = ttk.Scale(main, from_=0, to=90, orient="horizontal", length=300, variable=y_value)
y_slider.grid(row=1, column=1, padx=10)
y_label = ttk.Label(main, text="90")
y_label.grid(row=1, column=2, padx=10)

# --- Update servos and labels ---
def update_servo(event=None):
    x = int(x_slider.get())
    y = int(y_slider.get())
    x_label.config(text=f"{x}")
    y_label.config(text=f"{y}")
    send_to_arduino(x, y)

# Bind movement and release
for slider in (x_slider, y_slider):
    slider.bind("<B1-Motion>", update_servo)
    slider.bind("<ButtonRelease-1>", update_servo)

# --- Initialize both to 90 ---
def init_position():
    x_slider.set(90)
    y_slider.set(45)
    update_servo()

root.after(1500, init_position)  # wait for Arduino to be ready

root.mainloop()

if arduino:
    arduino.close()
