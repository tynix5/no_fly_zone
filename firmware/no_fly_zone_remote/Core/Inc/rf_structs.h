#ifndef RF_STRUCTS_H_
#define RF_STRUCTS_H_

#include <stdint.h>
#include "quaternion.h"

#define ARMED_KEY    0xAA
#define DISARMED_KEY 0x55
#define FAILSAFE_KEY 0x2C

typedef enum : uint8_t
{
    MODE_STATUS_DISARMED = 0,
    MODE_STATUS_ARMED,
    MODE_STATUS_FAILSAFE,

} arm_status_t;

// packet sent from remote to quadcopter
typedef struct __attribute__((packed))
{
    uint16_t throttle;
    quaternion_t q_des;
    float kp;
    float ki;
    float kd;

    uint8_t key;

} rf_packet_params_t;

// packet sent from quadcopter to remote (as acknowledgement)
typedef struct
{
    uint8_t key;
    uint8_t quad_batt_lvl;
} rf_ack_params_t;

#endif