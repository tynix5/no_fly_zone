#ifndef JOYSTICK_HELPER_H_
#define JOYSTICK_HELPER_H_

/*******************************************************************************************/
/******************************* Normalized Joystick Positions *****************************/
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
/*******************************************************************************************/
/*******************************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "quaternion.h"

// ACRO mode
#define MAX_PITCH_RATE_DEG 360.0f
#define MIN_PITCH_RATE_DEG -360.0f
#define MAX_ROLL_RATE_DEG  360.0f
#define MIN_ROLL_RATE_DEG  -360.0f

// NON-ACRO mode
#define MAX_PITCH_ANG_DEG 45.0f
#define MIN_PITCH_ANG_DEG -45.0f
#define MAX_ROLL_ANG_DEG  45.0f
#define MIN_ROLL_ANG_DEG  -45.0f

// ACRO and NON-ACRO mode
#define MAX_YAW_RATE_DEG 180.0f
#define MIN_YAW_RATE_DEG -180.0f

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
    // joystick positions
    uint16_t throttle, yaw, pitch, roll;

    // calibration parameters
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

/* Initialize joystick calibration values to defaults */
/* [input] joystick - joystick structure handle */
void joysticks_init(joysticks_t * joystick);

/* Start calibration process by setting flag and changing timestamp */
/* [output] joystick - joystick structure handle */
void joysticks_calibration_start(joysticks_t * joystick);

/* Convert raw ADC inputs from joysticks into normalized and shaped outputs */
/* [input] joystick - joystick structure handle */
/* [output] throttle_cond - sanitized throttle output [-1, 1] */
/* [output] pitch_cond - sanitized pitch output [-1, 1] */
/* [output] roll_cond - sanitized roll output [-1, 1] */
/* [output] yaw_cond - sanitized yaw output [-1, 1] */
void joysticks_condition(joysticks_t * joystick, float * throttle_cond, float * pitch_cond, float * roll_cond, float * yaw_cond);

/* Convert normalized joystick position to a DShot throttle */
/* [input] throttle_norm - normalized throttle input [-1, 1] */
/* [output] throttle - converted DShot output */
void joysticks_pos_to_throttle(float throttle_norm, uint16_t * throttle);

/* Convert normalized stick positions to a desired orientation (one that tracks integrates on previous orientations) */
/* [input] pitch_norm - normalized pitch input [-1, 1] */
/* [input] roll_norm - normalized roll input [-1, 1] */
/* [input] yaw_norm - normalized yaw input [-1, 1] */
/* [output] q_des - desired output quaternion */
void joysticks_pos_to_quat_acro(float pitch_norm, float roll_norm, float yaw_norm, quaternion_t * q_des);

/* Convert normalized stick positions to a desired orientation (one that DOES NOT integrate on previous orientations) */
/* [input] pitch_norm - normalized pitch input [-1, 1] */
/* [input] roll_norm - normalized roll input [-1, 1] */
/* [input] yaw_norm - normalized yaw input [-1, 1] */
/* [input/output] yaw_ang_des - desired yaw angle [-360, 360] integrated from previous iteration */
/* [output] q_des - desired output quaternion */
void joysticks_pos_to_quat(float pitch_norm, float roll_norm, float yaw_norm, float * yaw_ang_des, quaternion_t * q_des);

/* Calibrate joysticks - if new joystick reading exceeds marked thresholds, replace thresholds */
/* [input] joystick - joystick structure handle */
void joysticks_calibrate(joysticks_t * joystick);

/* Tune PID parameters using joysticks */
/* [input] joystick - joystick structure handle */
/* [input] pitch_norm - normalized pitch input [-1, 1] */
/* [input] roll_norm - normalized roll input [-1, 1] */
/* [input/output] kp - pointer to P constant */
/* [input/output] ki - pointer to I constant */
/* [input/output] kd - pointer to D constant */
/* [input/output] last_tune_ms - timestamp of last PID tuner update */
/* [input/output] active_tune_parameter - indicates parameter currently being tuned */
void joysticks_tune_pid(float pitch_norm, float roll_norm, float * kp, float * ki, float * kd, uint32_t * last_tune_ms, active_tune_param_t * active_tune_parameter);

#endif