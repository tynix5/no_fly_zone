#ifndef MADGWICK_H_
#define MADGWICK_H_

#include "quaternion.h"
#include "arm_math.h"

typedef struct {

    quaternion_t q_accel, q_gyro;
    quaternion_t q_gyro_dot;

    quaternion_t q_state, q_state_prev, q_state_dot;

    float beta;
    float dt;

} madgwick_state_t;

void madgwick_init(madgwick_state_t * state);
void madgwick_update(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, madgwick_state_t * state);

#endif