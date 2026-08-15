#ifndef RF_STRUCTS_H_
#define RF_STRUCTS_H_

#include <stdint.h>
#include "quaternion.h"

#define ARMED_KEY                   0xAA
#define DISARMED_KEY                0x55

// packet sent from remote to quadcopter
typedef struct {

    uint16_t throttle;
    Quaternion * q_des;
    float yaw_rate;
    float kp;
    float ki;
    float kd;
    
    uint8_t armed;

} PacketParams;

// packet sent from quadcopter to remote (as acknowledgement)
typedef struct {

    uint8_t key;
    uint8_t rx_batt_lvl;

} AckParams;


#endif