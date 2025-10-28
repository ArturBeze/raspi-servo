import asyncio
import serial
import serial.tools.list_ports
import threading
import time

class AsyncServoController:
    def __init__(self, baudrate=115200, timeout=2):
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.port = self._auto_detect_port()
        self._connect()

        self._loop = asyncio.get_event_loop()
        self._read_thread = threading.Thread(target=self._read_from_arduino, daemon=True)
        self._read_thread.start()

        self.last_response = None
        self._lock = threading.Lock()

    def _auto_detect_port(self):
        """Automatically detect Arduino port."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "Arduino" in port.description or "usbmodem" in port.device or "ttyACM" in port.device:
                return port.device
        raise IOError("❌ Arduino not found. Connect the board and try again.")

    def _connect(self):
        """Connect to Arduino."""
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)
        print(f"✅ Connected to {self.port}")

    def _read_from_arduino(self):
        """Continuously read responses from Arduino."""
        while self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode().strip()
                if line:
                    with self._lock:
                        self.last_response = line
                    print(f"📨 {line}")
            except Exception:
                pass

    async def set_x(self, angle: int):
        """Set X servo (0–180)."""
        await self._send_command(f"X:{int(angle)}")

    async def set_y(self, angle: int):
        """Set Y servo (0–90)."""
        await self._send_command(f"Y:{int(angle)}")

    async def _send_command(self, command: str):
        """Send command asynchronously."""
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("❌ Arduino connection not open.")

        def write_command():
            self.ser.write((command + "\n").encode())

        await asyncio.to_thread(write_command)
        await asyncio.sleep(0.05)  # give Arduino time to respond

    def get_last_response(self):
        """Return last received line from Arduino."""
        with self._lock:
            return self.last_response

    def close(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 Connection closed.")

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
