# scripts
A collection of AI-generated Python scripts for code verification and validation.

## orientation_quat.py
Receives a four-float quaternion packet over serial (in order of q_w, q_x, q_y, q_z), and will display the quadcopter's orientation. Can be used to validate q_des packet coming from remote control, and can also be used to validate calculated IMU orientation from no_fly_zone.

## self_test.py
Requires no user interaction; simply running the script will verify the orientation_quat quaternion directionality.

## serial_plotter.py
A simple script that will receive a serial packet of any size and print them over terminal. Does not require 'sprintf' C function that microcontroller could use to print directly to terminal instead. For this same reason, it also supports receiving floats (since STM32 will not 'sprintf' with floats)