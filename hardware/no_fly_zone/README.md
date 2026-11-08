# no_fly_zone Schematic + PCB
## Overview

<p align="center">
  <img src="../../screenshots/no_fly_zone_rev_all_pcb.png" width="2400"><br>
  <em>Flight controller PCBs. From left to right: Revision 1.0, 2.0, 2.1</em>
</p>

### Microcontroller
The MCU chosen was the STM32F446RCT for its high maximum clock frequency, dedicated FPU (for quaternion calculations), and abundant peripherals (SPI, timers, etc.). 
### IMU/Sensor Suite
The sensor suite in Revision 1.0 consists of a 6DOF IMU, magnetometer, barometer, and GPS. The IMU provides information on pitch and roll, magnetometer on yaw, barometer on altitude, and GPS for extended yaw support as well as location tracking for future revisions. Onboard magnetometers are very susceptible to EMF fields from motors, other sensors, as well as PCB traces. Thus, the GPS can be used as a backup for heading. Each subsequent iteration of the PCB reduced the sensors complexity until only the IMU and (optional) barometer remained. The GPS was removed for size and complexity reasons, and the magnetometer was removed due to the reasons described above. 
### Radio Communication
The nRF24L01 was the chosen RF link between the flight controller and remote for availablity and simplicity of implementation. In the future, a dedicated PA + LNA chip may be added on board to extend quadcopter range.
### Power Regulation
The power regulation circuit regulates the battery voltage (~16.8V) down to a smooth 5V, using a buck converter, then down to 3.3V, using an LDO, for powering the flight controller.


## Revision 1.0
### Summary

The single schematic given here is separated into two boards. A singular board containing all components was not able to fit onto the quadcopter frame. Thus, a separate MCU board and sensor board were produced using KiKit and connectors were used to share power and communication.

### Schematics

#### Hierarchal Schematic
The hierarchal schematic provides a top-level overview of the two boards. The ESC connectors, power schematic, and RF schematic were all included on the MCU board. The sensor board contains only the sensor schematic.
![alt text](../../screenshots/quad_hierarchal.png)

#### MCU
<p align="center">
  <img src="../../screenshots/quad_mcu.png" width="2400" />
</p>

#### Sensors
<p align="center">
  <img src="../../screenshots/quad_sensors.png" width="2400" />
</p>

#### Power
<p align="center">
  <img src="../../screenshots/quad_power.png" width="2400" />
</p>

#### RF
Like the RF section on the remote, the nRF24L01 is surrounded by a continuous ground plane on the top layer as well as the layer directly underneath the antenna for stability. Stitching vias are placed about ~6mm apart (between lambda/10 - lambda/20 of 2.4 GHz wavelength) near antenna for shielding.
<p align="center">
  <img src="../../screenshots/quad_rf.png" width="2400" />
</p>

#### Misc
Contains 8 M3 screw holes, 4 for each board. Not shown for brevity.

#### PCB
<p align="center">
  <img src="../../screenshots/quad_pcb.png" width="2400" />
</p>

#### 3D
<p align="center">
  <img src="../../screenshots/quad_3d.png" width="2400" />
</p>

## Revision 2.0
### Summary 
This revision involved consolidating the sensor board and MCU and removing the onboard magnetometer and GPS. The goal of this project was to achieve simple flight manuevers and hovering, and using these sensors would require more in depth sensor fusion algorithms. The antenna section was improved with further RF shielding and stitching vias, as well as a taper from the RF unbalanced port to the SMA pad. The board stackup changed from SIG - GND - 3.3V - SIG to a SIG - GND - GND - SIG + 3.3V to reduce buck converter and RF noise. 

Revision 1.0 board worked, but there were some problems with the JST connectors, making it hard to keep communication with the sensors, and they would not have been very reliable in a noisy and harsh environment. Ground lines were needed next to every signal line, but only one ground line was used on the entire board (power connector).

### Changes
- Consolidated sensor and controller board
- Removed magnetometer and GPS
- Improved RF shielding and layout
- Layer stackup changed from SIG-GND-3.3V-SIG to SIG-GND-GND-3.3V

### Schematics

### Hierarchal Schematic
<p align="center">
  <img src="../../screenshots/quad_hierarchal_rev2.png" width="2400" />
</p>

#### MCU
<p align="center">
  <img src="../../screenshots/quad_mcu_rev2.png" width="2400" />
</p>

#### Sensors
<p align="center">
  <img src="../../screenshots/quad_sensors_rev2.png" width="2400" />
</p>

#### Power
<p align="center">
  <img src="../../screenshots/quad_power_rev2.png" width="2400" />
</p>

#### RF
<p align="center">
  <img src="../../screenshots/quad_rf_rev2.png" width="2400" />
</p>

#### PCB
<p align="center">
  <img src="../../screenshots/quad_pcb_front_rev2.png" width="45%" />
  <img src="../../screenshots/quad_pcb_back_rev2.png" width="45%" />
</p>


#### 3D
<p align="center">
  <img src="../../screenshots/quad_3d_front_rev2.png" width="41%" />
  <img src="../../screenshots/quad_3d_back_rev2.png" width="45%" />
</p>

## Revision 2.1

### Summary 
While working with the Revision 2.0 flight controller, it was too large to fit comfortably on the quadcopter frame, so changes were made to reduce size while keeping functionality

### Changes
- Board size reduced from 50mm x 50mm to 40mm x 40mm
- Most part footprints reduced from 0805 packages to 0603
- Added TVS diodes to power inputs and connectors
- Removed ferrite bead in series with RF power supply
- Changed debug connector to STLINK specific
- Moved STAT3 from PB2 to PB8

#### MCU
<p align="center">
  <img src="../../screenshots/quad_mcu_rev2_1.png" width="2400" />
</p>

#### Sensors
<p align="center">
  <img src="../../screenshots/quad_sensors_rev_2_1.png" width="2400" />
</p>

#### Power
<p align="center">
  <img src="../../screenshots/quad_power_rev2_1.png" width="2400" />
</p>

#### RF
<p align="center">
  <img src="../../screenshots/quad_rf_rev2_1.png" width="2400" />
</p>

#### 3D
<p align="center">
  <img src="../../screenshots/quad_pcb_front_rev2_1.png" width="39%" />
  <img src="../../screenshots/quad_pcb_back_rev_2_1.png" width="45%" />
</p>
