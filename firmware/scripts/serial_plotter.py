import serial
import struct

PORT = "COM9"      # Change to your COM port
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Listening on {PORT}...")

try:
    while True:
        # Wait all data is sent
        # if ser.in_waiting >= 32:

        #     packet = ser.read(32)
        #     q_w, q_x, q_y, q_z, dt, w_x, w_y, w_z = struct.unpack("<ffffffff", packet)
            
        #     print(f"q: {q_w:8.4f}, {q_x:8.4f}, {q_y:8.4f}, {q_z:8.4f}\tdt: {dt:8.4f}\tw_bias: {w_x:8.6f}, {w_y:8.6f}, {w_z:8.6f}\n", end="", flush=True)
        # if ser.in_waiting >= 28:

        #             packet = ser.read(28)
        #             fl, bl, fr, br, w_x_err, w_y_err, w_z_err = struct.unpack("<fffffff", packet)
                    
        #             print(f"FL: {fl}, FR: {fr}, BL: {bl}, BR: {br}, w_x: {w_x_err:8.4f}, w_y: {w_y_err:8.4f}, w_z: {w_z_err:8.4f}\n", end="", flush=True)
        # if ser.in_waiting >= 24:
        
        #     packet = ser.read(24)
        #     w_x, w_y, w_z, w_x_bias, w_y_bias, w_z_bias = struct.unpack("<ffffff", packet)
            
        #     print(f"X: {w_x:8.4f}, Y: {w_y:8.4f}, Z: {w_z:8.4f}, X_bias: {w_x_bias:8.4f}, Y: {w_y_bias:8.4f}, Z: {w_z_bias:8.4f}\n", end="", flush=True)
        if ser.in_waiting >= 48:
        
            packet = ser.read(48)
            des_w, des_x, des_y, des_z, state_w, state_x, state_y, state_z, fl, bl, fr, br = struct.unpack("<ffffffffffff", packet)
            
            print(f"Desired: {des_w:8.4f}, {des_x:8.4f}, {des_y:8.4f}, {des_z:8.4f}, State: {state_w:8.4f}, {state_x:8.4f}, {state_y:8.4f}, {state_z:8.4f}, Speed: {fl}, {fr}, {bl}, {br}\n", end="", flush=True)
except KeyboardInterrupt:
    print("\nStopped.")

finally:
    ser.close()