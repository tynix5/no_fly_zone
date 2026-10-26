#include "bldc.h"
#include "dshot.h"

dshot_status_t bldc_init(bldc_handle_t * bldc, TIM_HandleTypeDef * htim2, TIM_HandleTypeDef * htim5)
{
    // configure all channels
    dshot_status_t status;

    status = dshot_init(&bldc->motor_fl, htim5, TIM_CHANNEL_2, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    if (status != DSHOT_OK)
        return status;

    status = dshot_init(&bldc->motor_bl, htim2, TIM_CHANNEL_1, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    if (status != DSHOT_OK)
        return status;

    status = dshot_init(&bldc->motor_br, htim5, TIM_CHANNEL_3, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    if (status != DSHOT_OK)
        return status;

    status = dshot_init(&bldc->motor_fr, htim2, TIM_CHANNEL_4, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    if (status != DSHOT_OK)
        return status;

    status = bldc_stop(bldc);
    if (status != DSHOT_OK)
        return status;

    return bldc_enable(bldc);
}

dshot_status_t bldc_disable(bldc_handle_t * bldc)
{
    // stop DMA transfers
    dshot_status_t status;

    status = dshot_stop(&bldc->motor_fl);
    if (status != DSHOT_OK)
        return status;

    status = dshot_stop(&bldc->motor_bl);
    if (status != DSHOT_OK)
        return status;

    status = dshot_stop(&bldc->motor_br);
    if (status != DSHOT_OK)
        return status;

    return dshot_stop(&bldc->motor_fr);
}

dshot_status_t bldc_enable(bldc_handle_t * bldc)
{
    // start DMA transfers
    dshot_status_t status;

    status = dshot_start(&bldc->motor_fl);
    if (status != DSHOT_OK)
        return status;

    status = dshot_start(&bldc->motor_bl);
    if (status != DSHOT_OK)
        return status;

    status = dshot_start(&bldc->motor_br);
    if (status != DSHOT_OK)
        return status;

    return dshot_start(&bldc->motor_fr);
}

dshot_status_t bldc_stop(bldc_handle_t * bldc)
{
    // set all throttles to disarmed
    bldc->throttles.speed_fl = bldc->throttles.speed_fr = bldc->throttles.speed_bl = bldc->throttles.speed_br = 0;
    return bldc_send(bldc);
}

dshot_status_t bldc_mix(bldc_handle_t * bldc, uint16_t throttle, float tau_x, float tau_y, float tau_z)
{
    // determine directions for tau_x, tau_y, tau_z
    // tau_x requires left motors to match, right motors to match
    // tau_y requires front motors to match, back motors to match
    // tau_z requires diagonals to match
    // tau x if side to side filt
    // tau y is forward/back tilt
    // tau z is yaw

    const uint16_t throttle_min = 200;

    bldc->throttles.speed_fl = throttle;
    bldc->throttles.speed_fr = throttle;
    bldc->throttles.speed_bl = throttle;
    bldc->throttles.speed_br = throttle;

    // only add additional flight forces when throttle is above a certain amount
    if (throttle > throttle_min)
    {
        // potentially limit throttle to 1900?
        bldc->throttles.speed_fl += tau_x + tau_y + tau_z;
        bldc->throttles.speed_fr += -tau_x + tau_y - tau_z;
        bldc->throttles.speed_bl += tau_x - tau_y - tau_z;
        bldc->throttles.speed_br += -tau_x - tau_y + tau_z;

        // limit throttle to throttle_min to prevent huge jumps in motor speeds when moving past throttle_min
        bldc_clamp(&bldc->throttles.speed_fl, throttle_min, DSHOT_MAX_THROTTLE);
        bldc_clamp(&bldc->throttles.speed_fr, throttle_min, DSHOT_MAX_THROTTLE);
        bldc_clamp(&bldc->throttles.speed_bl, throttle_min, DSHOT_MAX_THROTTLE);
        bldc_clamp(&bldc->throttles.speed_br, throttle_min, DSHOT_MAX_THROTTLE);
    }

    bldc_clamp(&bldc->throttles.speed_fl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    bldc_clamp(&bldc->throttles.speed_fr, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    bldc_clamp(&bldc->throttles.speed_bl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    bldc_clamp(&bldc->throttles.speed_br, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

    return DSHOT_OK;
}

dshot_status_t bldc_send(bldc_handle_t * bldc)
{

    dshot_status_t status;

    status = dshot_queue(&bldc->motor_fl, bldc->throttles.speed_fl, 0);
    if (status != DSHOT_OK)
        return status;

    status = dshot_queue(&bldc->motor_bl, bldc->throttles.speed_bl, 0);
    if (status != DSHOT_OK)
        return status;

    status = dshot_queue(&bldc->motor_br, bldc->throttles.speed_br, 0);
    if (status != DSHOT_OK)
        return status;

    return dshot_queue(&bldc->motor_fr, bldc->throttles.speed_fr, 0);
}

dshot_status_t bldc_clamp(uint16_t * throttle, uint16_t min, uint16_t max)
{
    if (*throttle > max)
        *throttle = max;
    else if (*throttle < min)
        *throttle = min;

    return DSHOT_OK;
}