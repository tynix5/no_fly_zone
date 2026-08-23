#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdint.h>

// from a top down view of the quad
//      front_left    front_right
//              \      /
//               \ _ /
//                | |
//                |_|
//               /   \
//              /     \     
//      back_left     back_right

typedef struct
{
    uint16_t front_left;
    uint16_t front_right;
    uint16_t back_left;
    uint16_t back_right;

} motor_speeds_t;

void motors_init();
void motors_update();
void motors_mix();
void motors_stop();

#endif