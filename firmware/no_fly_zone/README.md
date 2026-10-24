# no_fly_zone
## Overview
The role of the STM32 onboard the flight controller PCB can be split into five responsibilities...

1. Read sensor data from IMU and barometer
2. Receive packets from nRF24L01
3. Calculate current orientation based on previous sensor data
4. Drive motors accordingly to achieve desired orientation
5. Monitor battery level

Sensor data is received from the LSM6DS3TR IMU and LPS25HB barometer over SPI, along with the nRF24L01. The IMU data is filtered onboard the sensor, then fused using the Madgwick filter to estimate orientation. The current orientation is compared to the desired orientation, from the remote, and the BLDC motors are driven accordingly. The BLDC motors are driven using the digital communication method known as DShot. In my implementation, I used a combination of timers and DMA in order to achieve the desired bit streams. Lastly, an ADC is used to sample the battery voltage and current sense pin coming from the ESC. 

Of course, there are a few more things present in the code, but as a general overview, it is fairly accurate. The amount of firmware required to fly a drone is surprisingly little. One of these things are some of the safety features onboard. If samples from the IMU and barometer are not sampled fast enough or communication fails, the quadcopter will failsafe. If communication with the remote is lost, the quadcopter will failsafe. If the battery level onboard reaches dangerously low voltage, the quadcopter will failsafe. In failsafe mode, motors will stop and the onboard LEDs indicate the source of the error.

## Block Diagram

## Additional Features
### Battery "Management" System (BMS)
The battery level is sampled using the ADC, but since current draw changes in accordance with motor speed, the battery voltage will fluctuate often. The solution to this is a running average, which will remain relatively indifferent to current draw. The quadcopter will also periodically send back ACK packets wirelessly to the remote, indicating its state and its battery level.

### Safety
1. Software watchdogs were used to monitor internal CPU state, resetting on timeouts.
2. Software timeouts were implemented on the quadcopter to stop all motors if communication with remote failed, or sensor data became stale.
3. In the event of low battery on either the quadcopter or remote, the quadcopter would immediately shut down.
4. Arm/Disarm modes on the remote control preventing inadvertent throttling.

## Improvements
Currently, the barometer is not used, but in later revisions, can be used for a form of altitude hold.