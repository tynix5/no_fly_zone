import serial
import struct

ser = serial.Serial("COM8", 115200)

while True:
    data = ser.read(4)

    if len(data) == 4:
        raw, throttle = struct.unpack("<2H", data)

        print(
            f"Raw: {raw:4d}\n"
            f"Throttle: {throttle:4d}"
        )