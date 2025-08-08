import serial

# Replace with your correct port (e.g., "COM4" on Windows or "/dev/ttyUSB0" on Linux)
PORT = "COM2"
BAUDRATE = 230400

def read_serial():
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        print(f"Reading from {PORT} at {BAUDRATE} baud...\n")
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                print("Raw:", data.hex(" "))

if __name__ == "__main__":
    try:
        read_serial()
    except KeyboardInterrupt:
        print("\nStopped by user.")
