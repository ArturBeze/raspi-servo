import serial.tools.list_ports
import serial
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
            
def main():
    get_ports()

    PORT = '/dev/ttyACM0'
    BAUDRATE = 9600

    arduino = serial.Serial(PORT, BAUDRATE)
    time.sleep(2)  # wait for Arduino reset

    def set_servos(angle1, angle2):
        command = f"{angle1},{angle2}\n"
        arduino.write(command.encode('utf-8'))
        print(f"Sent: {command.strip()}")

    # Example usage
    set_servos(90, 45)
    time.sleep(1)
    set_servos(0, 180)

    arduino.close()

if __name__ == '__main__':
    main()
