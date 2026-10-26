#ifndef JOYSTICK_HELPER_H_
#define JOYSTICK_HELPER_H_

/*******************************************************************************************/
/*                     +1                                   +1                             */
/*                ----------                            ----------                         */
/*              --------------                        --------------                       */
/*            ------------------                    ------------------                     */
/*       -1 ---------------------- +1          -1 ---------------------- +1                */
/*          ----------------------                ----------------------                   */
/*            ------------------                    ------------------                     */
/*              --------------                        --------------                       */
/*                ----------                            ----------                         */
/*                    -1                                    -1                             */
/*******************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "quaternion.h"

#define MAX_PITCH_RATE_DEG 360.0f
#define MIN_PITCH_RATE_DEG -360.0f
#define MAX_ROLL_RATE_DEG  360.0f
#define MIN_ROLL_RATE_DEG  -360.0f
#define MAX_YAW_RATE_DEG   180.0f
#define MIN_YAW_RATE_DEG   -180.0f

#define MAX_PITCH_ANG_DEG 45.0f
#define MIN_PITCH_ANG_DEG -45.0f
#define MAX_ROLL_ANG_DEG  45.0f
#define MIN_ROLL_ANG_DEG  -45.0f

#define JOYSTICK_MIN_ADC 400
#define JOYSTICK_MID_ADC 1900
#define JOYSTICK_MAX_ADC 3600

#define JOYSTICK_CALIBRATION_T_MS 10000
#define JOYSTICK_TUNE_UPDATE_T_MS 250

#define ADC_RES_CNT   4096.0f
#define ADC_SAMP_FREQ 200.0f
#define ADC_SAMP_DT   1.0f / ADC_SAMP_FREQ

typedef struct
{
    uint16_t throttle, yaw, pitch, roll;

    uint16_t throttle_max, yaw_max, pitch_max, roll_max;
    uint16_t throttle_min, yaw_min, pitch_min, roll_min;
    uint16_t throttle_mid, yaw_mid, pitch_mid, roll_mid;

    uint32_t last_calibration_ms;
    uint32_t (*millis_get_tick)(void);

    bool is_calibrating;

} joysticks_t;
typedef enum : int32_t
{
    ACTIVE_TUNE_NONE = 0,
    ACTIVE_TUNE_KP,
    ACTIVE_TUNE_KI,
    ACTIVE_TUNE_KD
} active_tune_param_t;

void joysticks_init(joysticks_t * joystick);
void joysticks_calibration_start(joysticks_t * joystick);
void joysticks_condition(joysticks_t * joystick, float * throttle_cond, float * pitch_cond, float * roll_cond, float * yaw_cond);
void joysticks_pos_to_throttle(float throttle_norm, uint16_t * throttle);
void joysticks_pos_to_quat_acro(float pitch_norm, float roll_norm, float yaw_norm, quaternion_t * q_des);
void joysticks_pos_to_quat(float pitch_norm, float roll_norm, float yaw_norm, float * yaw_ang_des, quaternion_t * q_des);
void joysticks_calibrate(joysticks_t * joystick);
void joysticks_tune_pid(float pitch_norm, float roll_norm, float * kp, float * ki, float * kd, uint32_t * last_tune_ms, active_tune_param_t * active_tune_parameter);

#endif