#ifndef RF_STRUCTS_H_
#define RF_STRUCTS_H_

#include <stdint.h>
#include "quaternion.h"

#define ARMED_KEY    0xAA
#define DISARMED_KEY 0x55

typedef enum
{
    QUAD_STATUS_DISARMED = 0,
    QUAD_STATUS_ARMED

} quad_arm_status_t;

// packet sent from remote to quadcopter
typedef struct
{

    uint16_t throttle;
    quaternion_t * q_des;
    float yaw_rate;
    float kp;
    float ki;
    float kd;

    quad_arm_status_t armed;

} rf_packet_params_t;

// packet sent from quadcopter to remote (as acknowledgement)
typedef struct
{

    quad_arm_status_t armed;
    uint8_t rx_batt_lvl;

} rf_ack_params_t;

#endif