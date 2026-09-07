#ifndef BLDC_H_
#define BLDC_H_

#include "dshot.h"
#include "generic_types.h"

#define DSHOT_TIM_FREQ      84000000

/*
// from a top down view of the quad
//      front_left    front_right
//              \      /
//               \ _ /
//                | |
//                |_|
//               /   \
//              /     \     
//      back_left     back_right
*/

typedef struct {
    uint16_t speed_fl, speed_fr, speed_bl, speed_br;
} bldc_throttle_t;

typedef struct {
    dshot_handle_t motor_fl, motor_fr, motor_bl, motor_br;
    bldc_throttle_t throttles;
} bldc_handle_t;

status_t bldc_init(bldc_handle_t * bldc, TIM_HandleTypeDef * htim2, TIM_HandleTypeDef * htim5);
status_t bldc_disable(bldc_handle_t * bldc);
status_t bldc_enable(bldc_handle_t * bldc);
status_t bldc_stop(bldc_handle_t * bldc);
status_t bldc_mix(bldc_handle_t * bldc, uint16_t throttle, float tau_x, float tau_y, float tau_z);
status_t bldc_send(bldc_handle_t * bldc);
status_t bldc_clamp(uint16_t * throttle, uint16_t min, uint16_t max);

#endif