import serial
import struct

PORT = "COM11"      # Change to your COM port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Listening on {PORT}...")

try:
    while True:
        # Wait until a complete packet (3 floats = 12 bytes) is available
        if ser.in_waiting >= 16:
            packet = ser.read(16)

            # Unpack three little-endian floats
            # a_x, a_y, a_z, w_x, w_y, w_z, m_x, m_y, m_z, m_t, hpa, b_t = struct.unpack("<ffffffffffff", packet)
            q_w, q_x, q_y, q_z = struct.unpack("<ffff", packet)

            # print(f"\ra_x: {a_x:8.2f}\ta_y: {a_y:8.2f}\ta_z: {a_z:8.2f}\tw_x: {w_x:8.2f}\tw_y: {w_y:8.2f}\tw_z: {w_z:8.2f}\tm_x: {m_x:8.2f}\tm_y: {m_y:8.2f}\tm_z: {m_z:8.2f}\tmag_temp: {m_t:.2f}\tP: {hpa:8.2f}\tbar_temp: {b_t:8.2f}", end="", flush=True)
            print(f"\rq_w: {q_w:8.4f}\tq_x: {q_x:8.4f}\tq_y: {q_y:8.4f}\tq_z: {q_z:8.4f}")
except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()