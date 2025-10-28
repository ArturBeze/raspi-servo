import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import time

# --- Автоопределение порта Arduino ---
def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "ttyACM" in port.device or "ttyUSB" in port.device:
            return port.device
    raise RuntimeError("Arduino не найдено. Подключите устройство и перезапустите программу.")

arduino_port = find_arduino_port()
arduino = serial.Serial(arduino_port, 115200, timeout=1)
time.sleep(2)

# --- Начальные значения углов ---
angle_x = 90
angle_y = 45

# --- Параметры холста ---
canvas_size = 250
center_x = canvas_size // 2
center_y = canvas_size // 2
radius = 10
bar_width = 10
bar_height = 90  # вертикальные бары

# --- Текущие координаты камеры ---
current_x = center_x
current_y = center_y

# --- GUI ---
root = tk.Tk()
root.title("Servo Control via Arduino")
root.geometry("300x350")
root.resizable(False, False)

title = ttk.Label(root, text="Управление сервомоторами", font=("Arial", 14))
title.pack(pady=10)

# --- Слайдеры ---
frame_sliders = ttk.Frame(root)
frame_sliders.pack(pady=5)

# Слайдер X
ttk.Label(frame_sliders, text="X (0–180°):").grid(row=0, column=0, padx=5)
slider_x = ttk.Scale(frame_sliders, from_=0, to=180, orient="horizontal")
slider_x.set(angle_x)
slider_x.grid(row=0, column=1, padx=10)
label_x_val = ttk.Label(frame_sliders, text=f"{angle_x}°", width=5)
label_x_val.grid(row=0, column=2)

# Слайдер Y
ttk.Label(frame_sliders, text="Y (0–90°):").grid(row=1, column=0, padx=5)
slider_y = ttk.Scale(frame_sliders, from_=0, to=90, orient="horizontal")
slider_y.set(angle_y)
slider_y.grid(row=1, column=1, padx=10)
label_y_val = ttk.Label(frame_sliders, text=f"{angle_y}°", width=5)
label_y_val.grid(row=1, column=2)

# --- Холст ---
canvas = tk.Canvas(root, width=canvas_size, height=canvas_size, bg="#f0f0f0")
canvas.pack(pady=10)

# Сетка
canvas.create_line(center_x, 0, center_x, canvas_size, fill="#ccc", dash=(3,2))
canvas.create_line(0, center_y, canvas_size, center_y, fill="#ccc", dash=(3,2))

# Точка камеры
camera_dot = canvas.create_oval(center_x-radius, center_y-radius,
                                center_x+radius, center_y+radius, fill="blue")
# Текстовые значения
label_x_canvas = canvas.create_text(center_x - 40, 20, text=f"X: {angle_x}°", font=("Arial", 10), fill="black")
label_y_canvas = canvas.create_text(center_x - 40, 40, text=f"Y: {angle_y}°", font=("Arial", 10), fill="black")

# Вертикальные барграфы
bar_x = canvas.create_rectangle(center_x + 60, center_y - bar_height, center_x + 60 + bar_width, center_y, fill="blue")
bar_y = canvas.create_rectangle(center_x + 80, center_y - (angle_y/90)*bar_height, center_x + 80 + bar_width, center_y, fill="blue")

# --- Функции управления ---
def send_command(axis, value):
    cmd = f"{axis}:{int(value)}\n"
    arduino.write(cmd.encode('utf-8'))
    time.sleep(0.03)
    response = arduino.readline().decode().strip()
    if response:
        print(response)

def angle_to_canvas_coords(angle_x_val, angle_y_val):
    x = radius + (angle_x_val / 180) * (canvas_size - 2*radius)
    y = (canvas_size - radius) - (angle_y_val / 90) * (canvas_size - 2*radius)
    return x, y

def update_camera_color():
    dx = abs(angle_x - 90)/90
    dy = abs(angle_y - 45)/45
    deviation = max(dx, dy)
    if deviation < 0.1:
        color = "blue"
    elif deviation < 0.4:
        color = "green"
    elif deviation < 0.7:
        color = "yellow"
    else:
        color = "red"
    canvas.itemconfig(camera_dot, fill=color)
    canvas.itemconfig(bar_x, fill=color)
    canvas.itemconfig(bar_y, fill=color)

def update_angle_labels_on_canvas():
    canvas.itemconfig(label_x_canvas, text=f"X: {angle_x}°")
    canvas.itemconfig(label_y_canvas, text=f"Y: {angle_y}°")
    # Обновление вертикальных баров
    canvas.coords(bar_x,
                  center_x + 60,
                  center_y - (angle_x/180)*bar_height,
                  center_x + 60 + bar_width,
                  center_y)
    canvas.coords(bar_y,
                  center_x + 80,
                  center_y - (angle_y/90)*bar_height,
                  center_x + 80 + bar_width,
                  center_y)

def move_camera_smooth():
    global current_x, current_y
    target_x, target_y = angle_to_canvas_coords(angle_x, angle_y)
    step = 2
    dx = target_x - current_x
    dy = target_y - current_y
    if abs(dx)<1 and abs(dy)<1:
        current_x = target_x
        current_y = target_y
        canvas.coords(camera_dot, current_x-radius, current_y-radius, current_x+radius, current_y+radius)
        update_camera_color()
        update_angle_labels_on_canvas()
        return
    distance = (dx**2 + dy**2)**0.5
    current_x += dx/distance*min(step, distance)
    current_y += dy/distance*min(step, distance)
    canvas.coords(camera_dot, current_x-radius, current_y-radius, current_x+radius, current_y+radius)
    update_camera_color()
    update_angle_labels_on_canvas()
    root.after(20, move_camera_smooth)

def update_x(val):
    global angle_x
    angle_x = int(float(val))
    label_x_val.config(text=f"{angle_x}°")
    send_command('X', angle_x)
    move_camera_smooth()

def update_y(val):
    global angle_y
    angle_y = int(float(val))
    label_y_val.config(text=f"{angle_y}°")
    send_command('Y', angle_y)
    move_camera_smooth()

def on_close():
    arduino.close()
    root.destroy()

slider_x.config(command=update_x)
slider_y.config(command=update_y)

move_camera_smooth()
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()


"""
#include <Servo.h>

Servo servoX;
Servo servoY;

int posX = 90;  // старт X
int posY = 45;  // старт Y

void setup() {
  servoX.attach(9);
  servoY.attach(10);
  servoX.write(posX);
  servoY.write(posY);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("X:")) {
      int value = command.substring(2).toInt();
      value = constrain(value, 0, 180);
      servoX.write(value);
      posX = value;
      Serial.print("X=");
      Serial.println(posX);
    } 
    else if (command.startsWith("Y:")) {
      int value = command.substring(2).toInt();
      value = constrain(value, 0, 90);
      servoY.write(value);
      posY = value;
      Serial.print("Y=");
      Serial.println(posY);
    }
  }
}
"""
