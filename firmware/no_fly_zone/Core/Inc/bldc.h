#ifndef BLDC_H_
#define BLDC_H_

#include "dshot.h"

#define DSHOT_TIM_FREQ 84000000
/*
// from a top down view of the quad
//  front_left (CCW)    front_right (CW)
//              \      /
//               \ _ /
//                | |
//                |_|
//               /   \
//              /     \     
// back_left (CW)     back_right (CCW)
*/
typedef struct
{
    uint16_t speed_fl, speed_fr, speed_bl, speed_br;
} bldc_throttle_t;

typedef struct
{
    dshot_handle_t motor_fl, motor_fr, motor_bl, motor_br;
    bldc_throttle_t throttles;
} bldc_handle_t;

/* Initialize DShot channels for all four motors */
/* [in/out] bldc - BLDC structure handle */
/* [in] htim2 - TIM2 handle */
/* [in] htim5 - TIM5 handle */
/* [return] DSHOT_OK if channels open successfully */
dshot_status_t bldc_init(bldc_handle_t * bldc, const TIM_HandleTypeDef * htim2, const TIM_HandleTypeDef * htim5);

/* Stop all motor DMA transfers */
/* [in/out] bldc - BLDC structure handle */
/* [return] DSHOT_OK if DMA channels disable */
dshot_status_t bldc_disable(bldc_handle_t * bldc);

/* Start all DMA transfers */
/* [in/out] bldc - BLDC structure handle */
/* [return] DSHOT_OK if start is successful */
dshot_status_t bldc_enable(bldc_handle_t * bldc);

/* Set all throttle values to zero (disarmed) */
/* [in/out] bldc - BLDC structure handle */
/* [return] DSHOT_OK if stop is successful */
dshot_status_t bldc_stop(bldc_handle_t * bldc);

/* Stop all motor DMA transfers */
/* [in/out] bldc - BLDC structure handle */
/* [in] throttle - DShot throttle value [0, 2048] */
/* [in] tau_x - DShot speed change in X direction */
/* [in] tau_y - DShot speed change in Y direction */
/* [in] tau_z - DShot speed change in Z direction */
/* [return] DSHOT_OK if successful */
dshot_status_t bldc_mix(bldc_handle_t * bldc, const uint16_t throttle, const float tau_x, const float tau_y, const float tau_z);

/* Queue new DShot packets for all motors */
/* [in/out] bldc - BLDC structure handle */
/* [return] DSHOT_OK if send is successful */
dshot_status_t bldc_send(bldc_handle_t * bldc);

/* Clamp DShot throttles to specified range */
/* [in/out] throttle - DShot throttle */
/* [in] min - minimum value in range */
/* [in] max - maximum value in range */
/* [return] DSHOT_OK */
dshot_status_t bldc_clamp(uint16_t * throttle, const uint16_t min, const uint16_t max);

#endif