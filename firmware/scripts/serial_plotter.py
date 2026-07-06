import serial
import struct

PORT = "COM11"      # Change to your COM port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Listening on {PORT}...")

try:
    while True:
        # Wait until a complete packet (3 floats = 12 bytes) is available
        if ser.in_waiting >= 4:
            packet = ser.read(4)

            # Unpack three little-endian floats
            # x, y, z = struct.unpack("<iii", packet)
            # x, y, z = struct.unpack("<fff", packet)
            p = struct.unpack("<f", packet)[0]

            # print(f"\rX: {x:8.3f}  Y: {y:8.3f}  Z: {z:8.3f}", end="", flush=True)
            print(f"\rPressure: {p:.2f}", end="", flush=True)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()