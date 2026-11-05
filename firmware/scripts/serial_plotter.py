import serial
import struct

PORT = "COM8"      # Change to your COM port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Listening on {PORT}...")

try:
    while True:
        # Wait all data is sent
        if ser.in_waiting >= 12:

            packet = ser.read(12)
            pitch, roll, yaw = struct.unpack("<fff", packet)
            
            print(f"Pitch: {pitch:8.4f}\tRoll: {roll:8.4f}\tYaw: {yaw:8.4f}\n", end="", flush=True)
except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()