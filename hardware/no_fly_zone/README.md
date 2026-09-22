# no_fly_zone Schematic + PCB

## Overview

The quadcopter requires several components

1. Microcontroller
2. Sensor Suite
3. Radio Communication
4. Power Circuitry
5. ESC
6. Motors

The flight controller chosen was the STM32F446RCT for its high maximum clock frequency, dedicated FPU, and abundant communication modules. The sensor suite in Revision 1.0 consists of a 6DOF IMU, magnetometer, barometer, and GPS. Each of these sensors provides useful data for the flight controller. The IMU provides information on pitch and roll, magnetometer on yaw, barometer on altitude, and GPS for extended yaw support as well as location tracking for future revisions. Onboard magnetometers are very susceptible to EMF fields from motors, other sensors, as well as PCB traces. Thus, the GPS can be used as a backup for heading. 

The nRF24L01 was chosen to communicate with the remote, also using the nRF24L01. In the future, a dedicated PA + LNA chip may be added on board to extend quadcopter range.

The power circuitry consists of an onboard buck converter on the bottom side of the PCB in order to reduce noise to other components. More details below.

## Revision 1.0
### Summary

The single schematic given here is separated into two boards. A singular board containing all components was not able to fit onto the quadcopter frame. Thus, a separate MCU board and sensor board were produced using KiKit and connectors were used to share power and communication.

### Schematics

#### Hierarchal Schematic

The hierarchal schematic provides a top-level overview of the two boards. The ESC connectors, power schematic, and RF schematic were all included on the MCU board. The sensor board contains only the sensor schematic.

![alt text](../../screenshots/quad_hierarchal.png)

#### MCU

![alt text](../../screenshots/quad_mcu.png)

#### Sensors

![alt text](../../screenshots/quad_sensors.png)

#### Power

![alt text](../../screenshots/quad_power.png)

#### RF

Like the RF section on the remote, the nRF24L01 is surrounded by a continuous ground plane on the top layer as well as the layer directly underneath the antenna for stability. Stitching vias are placed about ~6mm apart (between lambda/10 - lambda/20 of 2.4 GHz wavelength) near antenna for shielding.

![alt text](../../screenshots/quad_rf.png)

#### Misc

Contains 8 M3 screw holes, 4 for each board. Not shown for brevity.

#### PCB

![alt text](../../screenshots/quad_pcb.png)

#### 3D

![alt text](../../screenshots/quad_3d.png)

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
![alt text](../../screenshots/quad_hierarchal_rev2.png)

#### MCU
![alt text](../../screenshots/quad_mcu_rev2.png)

#### Sensors
![alt text](../../screenshots/quad_sensors_rev2.png)

#### Power
![alt text](../../screenshots/quad_power_rev2.png)

#### RF
![alt text](../../screenshots/quad_rf_rev2.png)

#### PCB

![alt text](../../screenshots/quad_pcb_front_rev2.png)
![alt text](../../screenshots/quad_pcb_back_rev2.png)

#### 3D

![alt text](../../screenshots/quad_3d_front_rev2.png)
![alt text](../../screenshots/quad_3d_back_rev2.png)

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
![alt text](../../screenshots/quad_mcu_rev2_1.png)

#### Sensors
![alt text](../../screenshots/quad_sensors_rev_2_1.png)

#### Power
![alt text](../../screenshots/quad_power_rev2_1.png)

#### RF
![alt text](../../screenshots/quad_rf_rev2_1.png)

#### 3D
![alt text](../../screenshots/quad_pcb_front_rev2_1.png)
![alt text](../../screenshots/quad_pcb_back_rev_2_1.png)