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

#define MAX_PITCH_RATE_DEG 360
#define MIN_PITCH_RATE_DEG -360
#define MAX_ROLL_RATE_DEG  360
#define MIN_ROLL_RATE_DEG  -360
#define MAX_YAW_RATE_DEG   360
#define MIN_YAW_RATE_DEG   -360

#define JOYSTICK_MIN_ADC 400
#define JOYSTICK_MID_ADC 1900
#define JOYSTICK_MAX_ADC 3600

#define JOYSTICK_CALIBRATION_T_MS 10000
#define JOYSTICK_TUNE_UPDATE_T_MS 250

#define ADC_RES_CNT 4096.0f

typedef struct
{
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;
} joystick_t;

typedef struct
{
    uint16_t throttle_max, yaw_max, pitch_max, roll_max;
    uint16_t throttle_min, yaw_min, pitch_min, roll_min;
    uint16_t throttle_mid, yaw_mid, pitch_mid, roll_mid;

    uint32_t last_calibration_ms;
    uint32_t (*millis_get_tick)(void);

    bool is_calibrating;

} joystick_calibration_t;

typedef enum : int32_t
{
    ACTIVE_TUNE_NONE = 0,
    ACTIVE_TUNE_KP,
    ACTIVE_TUNE_KI,
    ACTIVE_TUNE_KD
} active_tune_param_t;

void joysticks_calibration_init(joystick_calibration_t * joystick_cal);
void joysticks_calibration_start(joystick_calibration_t * joystick_cal);
void joysticks_condition(joystick_t * joystick, joystick_calibration_t * joystick_cal, float * throttle_cond, float * pitch_cond, float * roll_cond, float * yaw_cond);
void normalize_input(uint16_t x, uint16_t x_min, uint16_t x_mid, uint16_t x_max, float deadzone, float beta, float * x_norm);
void pos_to_euler_rates(float pitch_norm, float roll_norm, float yaw_norm, float * pitch_rate, float * roll_rate, float * yaw_rate);
void pos_to_throttle(float throttle_norm, uint16_t * throttle);
void joysticks_calibrate(joystick_t * joystick, joystick_calibration_t * joystick_cal);
void joysticks_tune_pid(float pitch_norm, float roll_norm, float * kp, float * ki, float * kd, uint32_t * last_tune_ms, active_tune_param_t * active_tune_parameter);

#endif