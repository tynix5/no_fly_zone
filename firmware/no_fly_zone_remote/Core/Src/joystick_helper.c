#include "joystick_helper.h"
#include <math.h>

void joysticks_calibration_init(joystick_calibration_t * joystick_cal)
{
    // set default values for joystick range
    joystick_cal->throttle_min = 510;
    joystick_cal->pitch_min = 440;
    joystick_cal->roll_min = 430;
    joystick_cal->yaw_min = 500;

    joystick_cal->throttle_max = 3450;
    joystick_cal->pitch_max = 3390;
    joystick_cal->roll_max = 3370;
    joystick_cal->yaw_max = 3620;

    joystick_cal->throttle_mid = 1940;
    joystick_cal->pitch_mid = 1910;
    joystick_cal->roll_mid = 1900;
    joystick_cal->yaw_mid = 1950;

    joystick_cal->is_calibrating = false;
    joystick_cal->last_calibration_ms = 0;
}

void joysticks_calibration_start(joystick_calibration_t * joystick_cal)
{
    joystick_cal->is_calibrating = true;
    joystick_cal->last_calibration_ms = joystick_cal->millis_get_tick();
}

void joysticks_condition(joystick_t * joystick, joystick_calibration_t * joystick_cal, float * throttle_cond, float * pitch_cond, float * roll_cond, float * yaw_cond)
{
    const float deadzone = 0.05; // 5% stick deadzone
    const float beta = 0.4;      // higher beta, higher exponential growth

    // throttle input is linear
    normalize_input(joystick->throttle, joystick_cal->throttle_min, joystick_cal->throttle_mid, joystick_cal->throttle_max, deadzone, 0, throttle_cond);
    normalize_input(joystick->pitch, joystick_cal->pitch_min, joystick_cal->pitch_mid, joystick_cal->pitch_max, deadzone, beta, pitch_cond);
    normalize_input(joystick->roll, joystick_cal->roll_min, joystick_cal->roll_mid, joystick_cal->roll_max, deadzone, beta, roll_cond);
    normalize_input(joystick->yaw, joystick_cal->yaw_min, joystick_cal->yaw_mid, joystick_cal->yaw_max, deadzone, beta, yaw_cond);

    // correct sign changes
    *throttle_cond = -(*throttle_cond);
    *pitch_cond = -(*pitch_cond);
}

void normalize_input(uint16_t x, uint16_t x_min, uint16_t x_mid, uint16_t x_max, float deadzone, float beta, float * x_norm)
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

void pos_to_euler_rates(float pitch_norm, float roll_norm, float yaw_norm, float * pitch_rate, float * roll_rate, float * yaw_rate)
{
    // _norm in range [-1, 1]
    *pitch_rate = pitch_norm * MAX_PITCH_RATE_DEG;
    *roll_rate = roll_norm * MAX_ROLL_RATE_DEG;
    *yaw_rate = yaw_norm * MAX_YAW_RATE_DEG;
}

void pos_to_throttle(float throttle_norm, uint16_t * throttle)
{
    // check this range
    // convert from throttle range to DShot values
    if (throttle_norm < 0)
        *throttle = 0;
    else
        *throttle = (uint16_t)(throttle_norm * (2047.0f - 48.0f) + 48.0f);
}

void joysticks_calibrate(joystick_t * joystick, joystick_calibration_t * joystick_cal)
{
    if (joystick_cal->is_calibrating && joystick_cal->millis_get_tick() - joystick_cal->last_calibration_ms < JOYSTICK_CALIBRATION_T_MS)
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

        if (throttle_ceil > joystick_cal->throttle_max)
            joystick_cal->throttle_max = throttle_ceil;
        else if (throttle_floor < joystick_cal->throttle_min)
            joystick_cal->throttle_min = throttle_floor;

        if (pitch_ceil > joystick_cal->pitch_max)
            joystick_cal->pitch_max = pitch_ceil;
        else if (pitch_floor < joystick_cal->pitch_min)
            joystick_cal->pitch_min = pitch_floor;

        if (roll_ceil > joystick_cal->roll_max)
            joystick_cal->roll_max = roll_ceil;
        else if (roll_floor < joystick_cal->roll_min)
            joystick_cal->roll_min = roll_floor;

        if (yaw_ceil > joystick_cal->yaw_max)
            joystick_cal->yaw_max = yaw_ceil;
        else if (yaw_floor < joystick_cal->yaw_min)
            joystick_cal->yaw_min = yaw_floor;
    }
    else if (joystick_cal->millis_get_tick() - joystick_cal->last_calibration_ms >= JOYSTICK_CALIBRATION_T_MS)
    {
        joystick_cal->is_calibrating = false;
    }
}
