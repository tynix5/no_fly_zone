import serial
import struct

ser = serial.Serial("COM8", 115200)

while True:
    data = ser.read(8)

    if len(data) == 8:
        throttle, yaw, pitch, roll = struct.unpack("<4H", data)

        print(f"Throttle: {throttle:4d}  "
              f"Yaw: {yaw:4d}  "
              f"Pitch: {pitch:4d}  "
              f"Roll: {roll:4d}")