#include "madgwick.h"

static void accel_to_body_frame(float x, float y, float z, madgwick_state_t * state);            // dimensionless units (will be normalized)
static void gyro_to_body_frame(float x_rad, float y_rad, float z_rad, madgwick_state_t * state); // must be in radians per second
static void compute_err(madgwick_state_t * state, float * fg);
static void compute_jacobian_transpose(madgwick_state_t * state, float * JgT);
static void compute_gyro_dot(madgwick_state_t * state);
static void compute_gradient(madgwick_state_t * state, float * JgT, float * fg, quaternion_t * q_grad);
static void compute_estimate(madgwick_state_t * state, quaternion_t q_grad);

/* Body frame on current quadcopter
                    ^ X
                    |
                    |______> Y
                    Z (down)
*/
// Link to documentation used to build this filter
// https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#filter-gain

// In essence
// 1. Quaternions represent the rotation from the sensor frame to reference frame
// 2. Gyroscope is good for short term rotational information, downside is drift
// 3. Accelerometer is good for gravity reference, downside is inability to distinguish linear acceleration from gravity
// 4. Madgwick filter combines these two
// 5. If you know current orientation q, angular velocity measured by gyroscope will give you the rate of change of the quaternion (compute_gyro_dot())
// 6. You can therefore integrate and normalize to estimate attitude, but drift causes wild results
// 7. The accelerometer is brought in to introduce the notion of direction (of gravity), but not absolute orientation
// 8. The predicted gravity vector is computed (compute_err())
// 8a. The accelerometer data is normalized and compared with the predicted gravity vector
// 8b. If they agree (small error), the orientation estimate is consistent with gravity
// 9. The gradient is computed, which states the direction needed to move in order to reduce gravity error
// 10. A correction based on the normalized gradient is subtracted from the gyroscope-derived quaternion rate of change (compute estimate())
// 11. Integrate the corrected quaternion rate of change of orientation to find new orientation quatnernion (compute_estimate())
// 11. New quaternion is normalized such that is remains a valid unit quaternion (compute_estimate())
// Summary
// compute_err() - find the difference between the expected and measured gravity vector
// compute_gyro_dot() - based on gyroscope measurements, this is how estimatation of orientation should change
// compute_gradient() - based on the accelerometer sensor readings, move the orientation in this direction to achieve a better estimate
// B (beta) - controls strength of gradient-descent correction relative to gyro integration (high B, more accelerometer correction)
//          - small B --> smooth response, slower correction of drift; high B --> faster correction, susceptible to vibrations and acceleration
// dt - time elapsed between sensor updates

void madgwick_init(madgwick_state_t * state)
{
    state->q_state_prev.q1 = 1.0;
    state->q_state_prev.q2 = 0;
    state->q_state_prev.q3 = 0;
    state->q_state_prev.q4 = 0;
}

void madgwick_update(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, madgwick_state_t * state)
{
    float fg[3], JgT[12];
    quaternion_t q_grad;

    // convert sensor frames to body frames
    accel_to_body_frame(a_x, a_y, a_z, state);
    gyro_to_body_frame(w_x, w_y, w_z, state);

    // normalize acceleration
    quat_normalize(&state->q_accel);

    compute_err(state, fg);
    compute_jacobian_transpose(state, JgT);
    compute_gyro_dot(state);
    compute_gradient(state, JgT, fg, &q_grad);
    compute_estimate(state, q_grad);

    // update states
    state->q_state_prev.q1 = state->q_state.q1;
    state->q_state_prev.q2 = state->q_state.q2;
    state->q_state_prev.q3 = state->q_state.q3;
    state->q_state_prev.q4 = state->q_state.q4;
}

static void accel_to_body_frame(float x, float y, float z, madgwick_state_t * state)
{
    // align sensor coordinates to body frame coordinates
    // check these rotations line up correctly sign-wise
    state->q_accel.q1 = 0;
    state->q_accel.q2 = -y;
    state->q_accel.q3 = -x;
    state->q_accel.q4 = z;
}

static void gyro_to_body_frame(float x_rad, float y_rad, float z_rad, madgwick_state_t * state)
{
    // align sensor coordinates to body frame coordinates
    // check these rotations line up correctly sign-wise
    state->q_gyro.q1 = 0;
    state->q_gyro.q2 = y_rad;
    state->q_gyro.q3 = x_rad;
    state->q_gyro.q4 = z_rad;
}

static void compute_err(madgwick_state_t * state, float * fg)
{
    // compute f_g(q, s_a)
    // this is the error function: it calculates difference between predicted and measured state
    // the predicted state is everything in the equations but the subtraction of accelerometer quaternion
    fg[0] = 2.0 * (state->q_state_prev.q2 * state->q_state_prev.q4 - state->q_state_prev.q1 * state->q_state_prev.q3) - state->q_accel.q2;
    fg[1] = 2.0 * (state->q_state_prev.q1 * state->q_state_prev.q2 + state->q_state_prev.q3 * state->q_state_prev.q4) - state->q_accel.q3;
    fg[2] = 2.0 * (0.5 - state->q_state_prev.q2 * state->q_state_prev.q2 - state->q_state_prev.q3 * state->q_state_prev.q3) - state->q_accel.q4;
}

static void compute_jacobian_transpose(madgwick_state_t * state, float * JgT)
{
    /*
        Jg(q) = {   -2q_y, 2q_z, -2q_w, 2q_x,
                    2q_x, 2q_w, 2q_z, q_y,
                    0, -4q_x, -4q_y, 0          }
    */

    // if quaternion slightly changes, how does error change?
    JgT[0] = 2.0 * state->q_state_prev.q3;
    JgT[1] = 2.0 * state->q_state_prev.q2;
    JgT[2] = 0;
    JgT[3] = 2.0 * state->q_state_prev.q4;
    JgT[4] = 2.0 * state->q_state_prev.q1;
    JgT[5] = -4.0 * state->q_state_prev.q2;
    JgT[6] = -2.0 * state->q_state_prev.q1;
    JgT[7] = JgT[3];
    JgT[8] = -4.0 * state->q_state_prev.q3;
    JgT[9] = JgT[1];
    JgT[10] = JgT[0];
    JgT[11] = 0;
}

static void compute_gyro_dot(madgwick_state_t * state)
{
    // compute quaternion derivative for gyro
    // q_gyro_dot = 1/2 * q_state_prev * q_gyro
    quat_mult(state->q_state_prev, state->q_gyro, &state->q_gyro_dot);
    quat_mult_scalar(&state->q_gyro_dot, 0.5);
}

static void compute_gradient(madgwick_state_t * state, float * JgT, float * fg, quaternion_t * q_grad)
{
    arm_matrix_instance_f32 JgT_mat = { .numRows = 4, .numCols = 3, .pData = JgT };
    arm_matrix_instance_f32 fg_mat = { .numRows = 3, .numCols = 1, .pData = fg };

    float gradient[4];
    arm_matrix_instance_f32 gradient_mat = { .numRows = 4, .numCols = 1, .pData = gradient };

    // compute gradient = (J^T)f
    // linear estimate of how uncertainty in inputs propagates into uncertainty in output
    arm_mat_mult_f32(&JgT_mat, &fg_mat, &gradient_mat);

    q_grad->q1 = gradient[0];
    q_grad->q2 = gradient[1];
    q_grad->q3 = gradient[2];
    q_grad->q4 = gradient[3];
}

static void compute_estimate(madgwick_state_t * state, quaternion_t q_grad)
{
    // compute orientation estimation
    // q_t = q_t-1 + q_dot_t * dt
    quat_normalize(&q_grad);
    quat_mult_scalar(&q_grad, state->beta);
    quat_sub(state->q_gyro_dot, q_grad, &state->q_state_dot);
    quat_mult_scalar(&state->q_state_dot, state->dt);
    quat_add(state->q_state_prev, state->q_state_dot, &state->q_state);
    quat_normalize(&state->q_state);
}
