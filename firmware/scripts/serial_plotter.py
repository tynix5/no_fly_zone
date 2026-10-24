import serial
import struct

PORT = "COM9"      # Change to your COM port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Listening on {PORT}...")

try:
    while True:
        # Wait all data is sent
        if ser.in_waiting >= 32:

            packet = ser.read(32)
            q_w, q_x, q_y, q_z, dt, w_x, w_y, w_z = struct.unpack("<ffffffff", packet)
            
            print(f"q: {q_w:8.4f}, {q_x:8.4f}, {q_y:8.4f}, {q_z:8.4f}\tdt: {dt:8.4f}\tw_bias: {w_x:8.6f}, {w_y:8.6f}, {w_z:8.6f}\n", end="", flush=True)

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()