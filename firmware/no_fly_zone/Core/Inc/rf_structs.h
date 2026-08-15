#ifndef RF_STRUCTS_H_
#define RF_STRUCTS_H_

#include <stdint.h>

#define PACKET_SIZE                 sizeof(PacketParams)
#define ACK_SIZE                    sizeof(AckParams)

#define PACKET_KEY                  0xAA
#define ACK_KEY                     0x55

// packet sent from remote to quadcopter
typedef struct {

    // Quaternion quat_des;
    // float yaw_rate;
    // uint8_t armed;

    uint8_t pitch;
    uint8_t throttle;
    uint8_t yaw;
    uint8_t roll;
    float kp;
    float ki;
    float kd;
    uint8_t key;

} PacketParams;

// packet sent from quadcopter to remote (as acknowledgement)
typedef struct {

    uint8_t key;
    uint8_t rx_batt_lvl;

} AckParams;


#endif