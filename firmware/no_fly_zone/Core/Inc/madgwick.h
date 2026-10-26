#ifndef MADGWICK_H_
#define MADGWICK_H_

#include "quaternion.h"
#include "arm_math.h"

// all "private" variables
typedef struct
{
    quaternion_t _q_accel, _q_gyro;                     // accelerometer and gyroscope quaternion (body frame)
    quaternion_t _q_gyro_dot;                           // gyroscope derivative

    quaternion_t _q_state, _q_state_prev, _q_state_dot; // current state, previous state, current state derviative

    float _beta;                                        // accelerometer weight constant
    float _dt;                                          // time between successive updates
    float _k_att;                                       // attitude constant

} madgwick_state_t;

/* Initialize quaternion states to unit quaternion */
/* [out] state - madgwick structure handle */
void madgwick_init(madgwick_state_t * state);

/* Update current quaternion based on new IMU data */
/* [in] a_x - acceleration in X direction (unitless) */
/* [in] a_y - acceleration in Y direction (unitless) */
/* [in] a_z - acceleration in Z direction (unitless) */
/* [in] w_x - angular velocity in X direction (rad/s) */
/* [in] w_y - angular velocity in Y direction (rad/s) */
/* [in] w_z - angular velocity in Z direction (rad/s) */
/* [in] beta - accelerometer weight constant */
/* [in] dt - time between successive updates (seconds) */
/* [out] state - madgwick structure handle */
void madgwick_update(
    const float a_x, const float a_y, const float a_z, const float w_x, const float w_y, const float w_z, const float beta, const float dt, madgwick_state_t * state);

/* Update current quaternion based on new IMU data */
/* [in] state - madgwick structure handle */
/* [in] q_des - desired quaternion */
/* [in] k_att - attitude constant to convert from attitude error to desired angular rate */
/* [out] w_x_err - angular velocity error in _ direction (rad/s) */
/* [out] w_y_err - angular velocity error in _ direction (rad/s) */
/* [out] w_z_err - angular velocity error in _ direction (rad/s) */
void madgwick_curr_to_des(const madgwick_state_t * state, const quaternion_t q_des, const float k_att, float * w_x_err, float * w_y_err, float * w_z_err);

/* Clamp input between min and mix */
/* [in] in - input */
/* [in] min - minimum value */
/* [in] max - maximum value */
/* [return] clamped output */
float clamp(float in, float min, float max);

#endif