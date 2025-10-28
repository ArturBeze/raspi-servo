import serial
import serial.tools.list_ports
import time

class ServoController:
    def __init__(self, baudrate=115200, timeout=2):
        self.ser = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.port = self._auto_detect_port()
        self._connect()

    def _auto_detect_port(self):
        """Автоматически определяет порт Arduino."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'Arduino' in port.description or 'usbmodem' in port.device or 'ttyACM' in port.device:
                return port.device
        raise IOError("❌ Arduino не найден. Подключи плату и попробуй снова.")

    def _connect(self):
        """Устанавливает соединение с Arduino."""
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)  # даем Ардуино время перезапуститься
        print(f"✅ Подключено к {self.port}")

    def set_x(self, angle):
        """Устанавливает угол по оси X (0–180)."""
        self._send_command(f"X:{int(angle)}")

    def set_y(self, angle):
        """Устанавливает угол по оси Y (0–90)."""
        self._send_command(f"Y:{int(angle)}")

    def _send_command(self, command):
        """Отправка команды на Arduino и чтение ответа."""
        if not self.ser or not self.ser.is_open:
            raise ConnectionError("❌ Соединение с Arduino не установлено.")
        
        self.ser.write((command + "\n").encode())
        response = self.ser.readline().decode().strip()
        if response:
            print(f"📨 Ответ: {response}")

    def get_status(self):
        """Возвращает последнее полученное состояние (X=.., Y=..)."""
        if self.ser and self.ser.in_waiting:
            return self.ser.readline().decode().strip()
        return None

    def close(self):
        """Закрывает соединение."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 Соединение с Arduino закрыто.")

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
