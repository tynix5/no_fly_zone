# no_fly_zone
A custom designed quadcopter and remote combination featuring a custom STM32-based flight controller PCB and STM32-based remote control PCB that combine aspects of sensor fusion, quaternion-based attitude control, wireless communication, and digital ESC communication.

<p align="center">
  <img src="screenshots/quad_top_6.jpeg" width="800"><br>
  <em>Fully-assembled quadcopter</em>
</p>

<p align="center">
  <img src="screenshots/quad_top_3.jpeg" width="800"><br>
  <em>Flight controller pictured with ESC mounted underneath</em>
</p>

<p align="center">
  <img src="screenshots/quad_pcb_2.jpeg" width="800"><br>
  <em>Custom flight controller PCB</em>
</p>

<p align="center">
  <img src="screenshots/remote_real.jpeg" width="800"><br>
  <em>Custom remote control frame</em>
</p>

<p align="center">
  <img src="screenshots/remote_internal.jpeg" width="800"><br>
  <em>Remote control PCB</em>
</p>

## Overview
This project focused on the design and implementation of a custom quadcopter flight controller paired with a complementary custom remote from the ground-up. Rather than buying an off-the-shelf flight controller and remote, almost all aspects of both the flight controller and remote were custom made. I decided to do this project because I have always been fascinated by drones, and I liked the challenge it would bring. It ties together all aspects of signal integrity, systems, low-level firmware, and digital signal processing. Below is a summary of some of the aspects this project entailed...

### Custom Features
- Flight controller PCB
- Remote control PCB
- Remote control frame
- Sensor fusion firmware
- Wireless nRF24L01 driver and handler
- Digital ESC communication implementation (DShot)
- Sensor libraries (IMU, Barometer, etc.)
- Encoder and OLED display libraries for remote control

No off-the-shelf software or firmware was used, but some aspects of flight controller design were inspired by PixHawk and Betaflight. Components purchased off-shelf include the ESC, four BLDC motors, quadcopter frame, and board components.

## Folder Overview
### /firmware
Contains source code, build settings, documentation, and additional information/diagrams for both the remote and quadcopter. A link to the STM32 submodule library can also be found, where most of the sensor implementation is found.

Note: The scripts subfolder in /firmware contains AI-generated Python scripts for validation and testing purposes. AI was NOT used to generate code found on either the remote or flight controller.

### /hardware
Contains KiCAD schematics and and PCB fabrication outputs for both the remote and quadcopter. Additional information on hardware selection and design can be found in each respective subfolder.

### /3d
Contains the 3D files for the spacers used to separate the STM32 flight controller board and ESC board, along with the 3D files for the remote control housing.

### /screenshots
Contains relevant screenshots for those in other READMEs

## BOM
All components purchased off-shelf can be found in the `bom.txt` file, and the BOMs for the PCBs can be found in the `.csv` files in their respective folders.

## Improvements
- Use of magnetometer or GPS unit for absolute heading
- Use of barometer for altitude hold
- Improved remote control PCB including nonvolatile storage (for PID parameters) and amplified transmission power
- Newer remote frame design