#include "madgwick.h"

/**************************************************** Madgwick Helper Functions *****************************************************/
/* Convert accelerometer world frame to body frame */
/* [in] x - accelerometer X */
/* [in] y - accelerometer Y */
/* [in] z - accelerometer Z */
/* [out] state - madgwick structure handle */
static void accel_to_body_frame(const float x, const float y, const float z, madgwick_state_t * state); // dimensionless units (will be normalized)

/* Convert gyroscope world frame to body frame */
/* [in] x_rad - gyroscope X (rad/s) */
/* [in] y_rad - gyroscope Y (rad/s) */
/* [in] z_rad - gyroscope Z (rad/s) */
/* [out] state - madgwick structure handle */
static void gyro_to_body_frame(const float x_rad, const float y_rad, const float z_rad, madgwick_state_t * state); // must be in radians per second

/* Calculate difference between predicted and measured state */
/* [in] state - madgwick structure handle */
/* [out] fg - error function */
static void compute_err(const madgwick_state_t * state, float * fg);

/* Calculate Jacobian to determine how output changes in relation to input and transpose matrix */
/* [in] state - madgwick structure handle */
/* [out] JgT - jacobian transpose matrix */
static void compute_jacobian_transpose(const madgwick_state_t * state, float * JgT);

/* Computer quaternion derivative for gyro */
/* [in/out] state - madgwick structure handle */
static void compute_gyro_dot(madgwick_state_t * state);

/* Compute gradient - estimates how uncertainty propagates from input to output */
/* [in] state - madgwick structure handle */
/* [in] JgT - jacobian transpose */
/* [in] fg - error function */
/* [out] q_grad - gradient */
static void compute_gradient(const madgwick_state_t * state, float * JgT, float * fg, quaternion_t * q_grad);

/* Compute final orientation estimation */
/* [in] state - madgwick structure handle */
/* [in] q_grad - gradient */
static void compute_estimate(madgwick_state_t * state, quaternion_t q_grad);
/************************************************************************************************************************************/

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
// 10. A correction based on the normalized gradient is subtracted from the gyroscope-derived quaternion rate of change (compute_estimate())
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
    state->_q_state_prev.q1 = 1.0f;
    state->_q_state_prev.q2 = 0.0f;
    state->_q_state_prev.q3 = 0.0f;
    state->_q_state_prev.q4 = 0.0f;
    state->_q_state.q1 = 1.0f;
    state->_q_state.q2 = 0.0f;
    state->_q_state.q3 = 0.0f;
    state->_q_state.q4 = 0.0f;
}

void madgwick_update(
    const float a_x, const float a_y, const float a_z, const float w_x, const float w_y, const float w_z, const float beta, const float dt, madgwick_state_t * state)
{
    float fg[3], JgT[12];
    quaternion_t q_grad;

    state->_beta = beta;
    state->_dt = dt;

    // convert sensor frames to body frames
    accel_to_body_frame(a_x, a_y, a_z, state);
    gyro_to_body_frame(w_x, w_y, w_z, state);

    // normalize acceleration
    quat_normalize(&state->_q_accel);

    compute_err(state, fg);
    compute_jacobian_transpose(state, JgT);
    compute_gyro_dot(state);
    compute_gradient(state, JgT, fg, &q_grad);
    compute_estimate(state, q_grad);

    // update states
    state->_q_state_prev.q1 = state->_q_state.q1;
    state->_q_state_prev.q2 = state->_q_state.q2;
    state->_q_state_prev.q3 = state->_q_state.q3;
    state->_q_state_prev.q4 = state->_q_state.q4;
}

void madgwick_curr_to_des(const madgwick_state_t * state, const quaternion_t q_des, const float k_att, float * w_x_err, float * w_y_err, float * w_z_err)
{
    // compute rotation needed to move from current orientation to desired orientation
    // calculate quaternion error
    quaternion_t q_state_conj, q_err;
    quat_copy(state->_q_state, &q_state_conj);
    quat_conjugate(&q_state_conj);
    quat_mult(q_state_conj, q_des, &q_err);

    // convert quaternion error to a 3D attitude error (approximation)
    // body frame convention, not sensor frame
    float sign = 1.0f;
    if (q_err.q1 < 0.0f)
        sign = -1.0f;

    // quaternion stores half the rotation angle inside sine and cosine
    // q_e = [q_w, q_x, q_y, q_z] = [cos(theta/2), u_x * sin(theta/2), u_y * sin(theta/2), u_z * sin(theta/2)]
    // Since sin(theta/2) ~= theta/2, 2 * q_x ~= u_x * theta
    // multiplication with sign results in shortest rotation
    float e_x = 2.0 * sign * q_err.q2;
    float e_y = 2.0 * sign * q_err.q3;
    float e_z = 2.0 * sign * q_err.q4;

    // convert attitude error into desired angular rates
    // k_att determines how aggressively the controller requests rotation back to desired attitude
    // Ex: k_att = 10 --> If attitude error is 1 degree, desired rate is 10 degrees/s
    // k_att = 2 --> If attitude error is 1 degree, desired rate is 2 degrees/s
    float w_x_des = e_x * k_att;
    float w_y_des = e_y * k_att;
    float w_z_des = e_z * k_att;

    // clamp rates
    const float max_rate_dps = 200.0f;
    const float max_rate_rps = max_rate_dps * M_PI / 180.0f;
    w_x_des = clamp(w_x_des, -max_rate_rps, max_rate_rps);
    w_y_des = clamp(w_y_des, -max_rate_rps, max_rate_rps);
    w_z_des = clamp(w_z_des, -max_rate_rps, max_rate_rps);

    // calculate difference between current and desired rotation rates
    *w_x_err = w_x_des - state->_q_gyro.q2;
    *w_y_err = w_y_des - state->_q_gyro.q3;
    *w_z_err = w_z_des - state->_q_gyro.q4;
}

float clamp(float in, float min, float max)
{
    if (in < min)
        return min;
    else if (in > max)
        return max;
    else
        return in;
}

static void accel_to_body_frame(const float x, const float y, const float z, madgwick_state_t * state)
{
    // align sensor coordinates to body frame coordinates
    // check these rotations line up correctly sign-wise
    state->_q_accel.q1 = 0;
    state->_q_accel.q2 = -y;
    state->_q_accel.q3 = -x;
    state->_q_accel.q4 = z;
}

static void gyro_to_body_frame(const float x_rad, const float y_rad, const float z_rad, madgwick_state_t * state)
{
    // align sensor coordinates to body frame coordinates
    // check these rotations line up correctly sign-wise
    state->_q_gyro.q1 = 0;
    state->_q_gyro.q2 = y_rad;
    state->_q_gyro.q3 = x_rad;
    state->_q_gyro.q4 = -z_rad;
}

static void compute_err(const madgwick_state_t * state, float * fg)
{
    // compute f_g(q, s_a)
    // this is the error function: it calculates difference between predicted and measured state
    // the predicted state is everything in the equations but the subtraction of accelerometer quaternion
    fg[0] = 2.0f * (state->_q_state_prev.q2 * state->_q_state_prev.q4 - state->_q_state_prev.q1 * state->_q_state_prev.q3) - state->_q_accel.q2;
    fg[1] = 2.0f * (state->_q_state_prev.q1 * state->_q_state_prev.q2 + state->_q_state_prev.q3 * state->_q_state_prev.q4) - state->_q_accel.q3;
    fg[2] = 2.0f * (0.5f - state->_q_state_prev.q2 * state->_q_state_prev.q2 - state->_q_state_prev.q3 * state->_q_state_prev.q3) - state->_q_accel.q4;
}

static void compute_jacobian_transpose(const madgwick_state_t * state, float * JgT)
{
    /*
        Jg(q) = {   -2q_y, 2q_z, -2q_w, 2q_x,
                    2q_x, 2q_w, 2q_z, 2q_y,
                    0, -4q_x, -4q_y, 0          }
    */

    // if quaternion slightly changes, how does error change?
    JgT[0] = -2.0f * state->_q_state_prev.q3;
    JgT[1] = 2.0f * state->_q_state_prev.q2;
    JgT[2] = 0.0f;

    JgT[3] = 2.0f * state->_q_state_prev.q4;
    JgT[4] = 2.0f * state->_q_state_prev.q1;
    JgT[5] = -4.0f * state->_q_state_prev.q2;

    JgT[6] = -2.0f * state->_q_state_prev.q1;
    JgT[7] = JgT[3];
    JgT[8] = -4.0f * state->_q_state_prev.q3;

    JgT[9] = JgT[1];
    JgT[10] = 2.0f * state->_q_state_prev.q3;
    JgT[11] = 0.0f;
}

static void compute_gyro_dot(madgwick_state_t * state)
{
    // compute quaternion derivative for gyro
    // _q_gyro_dot = 1/2 * _q_state_prev * _q_gyro
    quat_mult(state->_q_state_prev, state->_q_gyro, &state->_q_gyro_dot);
    quat_mult_scalar(&state->_q_gyro_dot, 0.5f);
}

static void compute_gradient(const madgwick_state_t * state, float * JgT, float * fg, quaternion_t * q_grad)
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
    quat_mult_scalar(&q_grad, state->_beta);
    quat_sub(state->_q_gyro_dot, q_grad, &state->_q_state_dot);
    quat_mult_scalar(&state->_q_state_dot, state->_dt);
    quat_add(state->_q_state_prev, state->_q_state_dot, &state->_q_state);
    quat_normalize(&state->_q_state);
}