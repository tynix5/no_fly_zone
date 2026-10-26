#include "joystick_helper.h"
#include "quaternion.h"
#include "stm32l4xx_hal.h"
#include <math.h>

static void joysticks_normalize_input(uint16_t x, uint16_t x_min, uint16_t x_mid, uint16_t x_max, float deadzone, float beta, float * x_norm);
static void joysticks_euler_rates_to_quat(float w_x, float w_y, float w_z, quaternion_t * q_des);
static void joysticks_pos_to_euler_rates(float pitch_norm, float roll_norm, float yaw_norm, float * pitch_rate, float * roll_rate, float * yaw_rate);
static void joysticks_pos_to_euler_ang(float pitch_norm, float roll_norm, float yaw_norm, float * yaw_ang_des, float * pitch_deg, float * roll_deg);

void joysticks_init(joysticks_t * joystick)
{
    // set default values for joystick range
    joystick->throttle_min = 510;
    joystick->pitch_min = 440;
    joystick->roll_min = 430;
    joystick->yaw_min = 500;

    joystick->throttle_max = 3450;
    joystick->pitch_max = 3390;
    joystick->roll_max = 3370;
    joystick->yaw_max = 3620;

    joystick->throttle_mid = 1940;
    joystick->pitch_mid = 1910;
    joystick->roll_mid = 1900;
    joystick->yaw_mid = 1950;

    joystick->is_calibrating = false;
    joystick->last_calibration_ms = 0;
}

void joysticks_calibration_start(joysticks_t * joystick)
{
    joystick->is_calibrating = true;
    joystick->last_calibration_ms = joystick->millis_get_tick();
}

void joysticks_condition(joysticks_t * joystick, float * throttle_cond, float * pitch_cond, float * roll_cond, float * yaw_cond)
{
    const float deadzone = 0.05; // 5% stick deadzone
    const float beta = 0.4;      // higher beta, higher exponential growth

    // throttle input is linear
    joysticks_normalize_input(joystick->throttle, joystick->throttle_min, joystick->throttle_mid, joystick->throttle_max, deadzone, 0, throttle_cond);
    joysticks_normalize_input(joystick->pitch, joystick->pitch_min, joystick->pitch_mid, joystick->pitch_max, deadzone, beta, pitch_cond);
    joysticks_normalize_input(joystick->roll, joystick->roll_min, joystick->roll_mid, joystick->roll_max, deadzone, beta, roll_cond);
    joysticks_normalize_input(joystick->yaw, joystick->yaw_min, joystick->yaw_mid, joystick->yaw_max, 0.10f, beta, yaw_cond); // slightly higher deadzone for yaw

    // correct sign changes
    *throttle_cond = -(*throttle_cond);
    *pitch_cond = -(*pitch_cond);
}

void joysticks_pos_to_throttle(float throttle_norm, uint16_t * throttle)
{
    // convert from throttle range to DShot values
    if (throttle_norm < 0)
        *throttle = 0;
    else
        *throttle = (uint16_t)(throttle_norm * (2047.0f - 48.0f) + 48.0f);
}

void joysticks_pos_to_quat_acro(float pitch_norm, float roll_norm, float yaw_norm, quaternion_t * q_des)
{
    // convert stick position to euler rates
    float pitch_rate_dps, roll_rate_dps, yaw_rate_dps;
    joysticks_pos_to_euler_rates(pitch_norm, roll_norm, yaw_norm, &pitch_rate_dps, &roll_rate_dps, &yaw_rate_dps);

    // convert to radians per second
    float pitch_rate_rps = pitch_rate_dps * M_PI / 180.0;
    float roll_rate_rps = roll_rate_dps * M_PI / 180.0;
    float yaw_rate_rps = yaw_rate_dps * M_PI / 180.0;

    // convert from euler rates to a quaternion
    joysticks_euler_rates_to_quat(roll_rate_rps, pitch_rate_rps, yaw_rate_rps, q_des);
}

void joysticks_pos_to_quat(float pitch_norm, float roll_norm, float yaw_norm, float * yaw_ang_des, quaternion_t * q_des)
{
    // covnvert stick position to euler angles
    float pitch_ang_deg, roll_ang_deg;
    joysticks_pos_to_euler_ang(pitch_norm, roll_norm, yaw_norm, yaw_ang_des, &pitch_ang_deg, &roll_ang_deg);

    // convert to radians
    float pitch_ang_rad = pitch_ang_deg * M_PI / 180.0;
    float roll_ang_rad = roll_ang_deg * M_PI / 180.0;
    float yaw_ang_rad = *yaw_ang_des * M_PI / 180.0;

    // convert from euler angles to a quaternion
    euler_to_quat(-pitch_ang_rad, roll_ang_rad, yaw_ang_rad, q_des);
}

void joysticks_calibrate(joysticks_t * joystick)
{
    if (joystick->is_calibrating && joystick->millis_get_tick() - joystick->last_calibration_ms < JOYSTICK_CALIBRATION_T_MS)
    {
        // add a buffer of 0.5% to every max value and min value
        uint16_t throttle_ceil = (uint16_t)((float)joystick->throttle * 1.005);
        uint16_t throttle_floor = (uint16_t)((float)joystick->throttle * 0.995);
        uint16_t pitch_ceil = (uint16_t)((float)joystick->pitch * 1.005);
        uint16_t pitch_floor = (uint16_t)((float)joystick->pitch * 0.995);
        uint16_t roll_ceil = (uint16_t)((float)joystick->roll * 1.005);
        uint16_t roll_floor = (uint16_t)((float)joystick->roll * 0.995);
        uint16_t yaw_ceil = (uint16_t)((float)joystick->yaw * 1.005);
        uint16_t yaw_floor = (uint16_t)((float)joystick->yaw * 0.995);

        if (throttle_ceil > joystick->throttle_max)
            joystick->throttle_max = throttle_ceil;
        else if (throttle_floor < joystick->throttle_min)
            joystick->throttle_min = throttle_floor;

        if (pitch_ceil > joystick->pitch_max)
            joystick->pitch_max = pitch_ceil;
        else if (pitch_floor < joystick->pitch_min)
            joystick->pitch_min = pitch_floor;

        if (roll_ceil > joystick->roll_max)
            joystick->roll_max = roll_ceil;
        else if (roll_floor < joystick->roll_min)
            joystick->roll_min = roll_floor;

        if (yaw_ceil > joystick->yaw_max)
            joystick->yaw_max = yaw_ceil;
        else if (yaw_floor < joystick->yaw_min)
            joystick->yaw_min = yaw_floor;
    }
    else if (joystick->millis_get_tick() - joystick->last_calibration_ms >= JOYSTICK_CALIBRATION_T_MS)
    {
        joystick->is_calibrating = false;
    }
}

void joysticks_tune_pid(float pitch_norm, float roll_norm, float * kp, float * ki, float * kd, uint32_t * last_tune_ms, active_tune_param_t * active_tune_parameter)
{
    float ** new_k;

    uint32_t curr_tick = HAL_GetTick(); // replace?

    // user-friendly update speed
    if (curr_tick - *last_tune_ms > JOYSTICK_TUNE_UPDATE_T_MS)
    {
        *last_tune_ms = curr_tick;

        if (roll_norm > 0.5f) // go to next parameter for tuning
        {
            if (++(*active_tune_parameter) > ACTIVE_TUNE_KD)
                *active_tune_parameter = ACTIVE_TUNE_NONE;
        }
        else if (roll_norm < -0.5f) // go to previous parameter for tuning
        {
            if (--(*active_tune_parameter) < ACTIVE_TUNE_NONE)
                *active_tune_parameter = ACTIVE_TUNE_KD;
        }

        switch (*active_tune_parameter)
        {
        case ACTIVE_TUNE_KP:
            // new_k holds address of kp
            // *new_k holds value of kp, which is address of x
            // **new_k holds value of *kp, which is x
            new_k = &kp;
            break;
        case ACTIVE_TUNE_KI:
            new_k = &ki;
            break;
        case ACTIVE_TUNE_KD:
            new_k = &kd;
            break;
        default:
            return;
        }

        if (pitch_norm >= 0.25f && pitch_norm < 0.40f)
            **new_k += 0.01f;
        else if (pitch_norm >= 0.40f && pitch_norm < 0.75f)
            **new_k += 0.10f;
        else if (pitch_norm >= 0.75f)
            **new_k += 1.00f;
        else if (pitch_norm > -0.40f && pitch_norm <= -0.25f)
            **new_k -= 0.01f;
        else if (pitch_norm > -0.75f && pitch_norm <= -0.4f)
            **new_k -= 0.10f;
        else if (pitch_norm <= -0.75f)
            **new_k -= 1.00f;

        if (**new_k > 99.99)
            **new_k = 99.985;
        else if (**new_k < 0.0)
            **new_k = 0.0;
    }
}

static void joysticks_normalize_input(uint16_t x, uint16_t x_min, uint16_t x_mid, uint16_t x_max, float deadzone, float beta, float * x_norm)
{
    // transform input x in range [x_min, x_max] to [-1, 1]
    // beta determines how sensitive control inputs are; beta = 1 results in cubic behavior
    // beta = 0 results in linear behavior
    // Small deviations from center of range result in smaller outputs, larger deviations result in larger swings

    // normalize each joystick input to a [-1, 1] range
    // apply a deadband
    // apply exponential weighting
    float norm, sign;

    // map input to [-1, 1] range
    if (x > x_mid)
    {
        norm = (float)(x - x_mid) / (float)(x_max - x_mid); // maps to [0, 1]
        sign = 1.0f;
    }
    else
    {
        norm = (float)(x - x_mid) / (float)(x_mid - x_min); // maps to [-1, 0]
        sign = -1.0f;
    }

    // fix discontinuity at deadzone boundaries
    if (fabsf(norm) < deadzone)
    {
        norm = 0;
    }
    else
    {
        // fix deadzone discontinuity, norm will be > 0 here always
        norm = (fabsf(norm) - deadzone) / (1.0 - deadzone);

        // scale output exponentially with input
        // complementary filter with exponential weighting
        norm = sign * ((1.0f - beta) * norm + powf(norm, 3.0) * beta);
    }

    // clamp output range in case input extends above/below maximum/minimum
    if (norm > 1.0f)
        norm = 1.0f;
    else if (norm < -1.0f)
        norm = -1.0f;

    *x_norm = norm;
}

static void joysticks_euler_rates_to_quat(float w_x, float w_y, float w_z, quaternion_t * q_des)
{
    // place gyro angular rates (radians per second) into quaternion
    quaternion_t q_rates = {
        .q1 = 0,
        .q2 = w_x,
        .q3 = -w_y,
        .q4 = w_z,
    };

    quaternion_t q_des_dot;

    // compute derivative
    // q_des_dot = 0.5 * q_des * [0, w_x, w_y, w_z]
    quat_mult(*q_des, q_rates, &q_des_dot);
    quat_mult_scalar(&q_des_dot, 0.5);

    // integrate with first-order approximation
    // q_des = q_des + q_des_dot * dt
    quat_mult_scalar(&q_des_dot, ADC_SAMP_DT);
    quat_add(*q_des, q_des_dot, q_des);

    // normalize
    quat_normalize(q_des);
}

static void joysticks_pos_to_euler_rates(float pitch_norm, float roll_norm, float yaw_norm, float * pitch_rate, float * roll_rate, float * yaw_rate)
{
    // _norm in range [-1, 1]
    *pitch_rate = pitch_norm * MAX_PITCH_RATE_DEG;
    *roll_rate = roll_norm * MAX_ROLL_RATE_DEG;
    *yaw_rate = yaw_norm * MAX_YAW_RATE_DEG;
}

static void joysticks_pos_to_euler_ang(float pitch_norm, float roll_norm, float yaw_norm, float * yaw_ang_des, float * pitch_deg, float * roll_deg)
{
    // _norm in range [-1, 1]
    *pitch_deg = pitch_norm * MAX_PITCH_ANG_DEG;
    *roll_deg = roll_norm * MAX_ROLL_ANG_DEG;
    // integrate to produce an angle
    *yaw_ang_des += yaw_norm * MAX_YAW_RATE_DEG * ADC_SAMP_DT;

    // handle wraps
    if (*yaw_ang_des > 360.0f)
        *yaw_ang_des -= 360.0f;
    else if (*yaw_ang_des < 0.0f)
        *yaw_ang_des += 360.0f;
}