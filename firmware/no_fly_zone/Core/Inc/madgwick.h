#ifndef MADGWICK_H_
#define MADGWICK_H_

#include "quaternion.h"
#include "arm_math.h"

// all "private" variables
typedef struct
{
    quaternion_t _q_accel, _q_gyro;
    quaternion_t _q_gyro_dot;

    quaternion_t _q_state, _q_state_prev, _q_state_dot;

    float _beta;
    float _dt;
    float _k_att;

} madgwick_state_t;

// add a calculate error function?
void madgwick_init(madgwick_state_t * state);
void madgwick_update(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, float beta, float dt, madgwick_state_t * state);
void madgwick_curr_to_des(madgwick_state_t * state, quaternion_t q_des, float k_att, float * w_x_err, float * w_y_err, float * w_z_err);

#endif