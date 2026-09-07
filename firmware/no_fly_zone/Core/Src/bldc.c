#include "bldc.h"

status_t bldc_init(bldc_handle_t * bldc, TIM_HandleTypeDef * htim2, TIM_HandleTypeDef * htim5)
{
    // configure all channels
    if (dshot_init(&bldc->motor_fl, htim5, TIM_CHANNEL_2, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_init(&bldc->motor_bl, htim2, TIM_CHANNEL_1, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_init(&bldc->motor_br, htim5, TIM_CHANNEL_3, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_init(&bldc->motor_fr, htim2, TIM_CHANNEL_4, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ) != STATUS_OK)
        return STATUS_ERR;

    if (bldc_stop(bldc) != STATUS_OK)
        return STATUS_ERR;

    if (bldc_enable(bldc) != STATUS_OK)
        return STATUS_ERR;

    return STATUS_OK;
}

status_t bldc_disable(bldc_handle_t * bldc)
{
    // stop DMA transfers
    if (dshot_stop(&bldc->motor_fl) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_stop(&bldc->motor_bl) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_stop(&bldc->motor_br) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_stop(&bldc->motor_fr) != STATUS_OK)
        return STATUS_ERR;

    return STATUS_OK;
}

status_t bldc_enable(bldc_handle_t * bldc)
{
    // start DMA transfers
    if (dshot_start(&bldc->motor_fl) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_start(&bldc->motor_bl) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_start(&bldc->motor_br) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_start(&bldc->motor_fr) != STATUS_OK)
        return STATUS_ERR;

    return STATUS_OK;
}

status_t bldc_stop(bldc_handle_t * bldc)
{
    // set all throttles to disarmed
    bldc->throttles.speed_fl = bldc->throttles.speed_fr = bldc->throttles.speed_bl = bldc->throttles.speed_br = 0;
    return bldc_send(bldc);
}

status_t bldc_mix(bldc_handle_t * bldc, uint16_t throttle, float tau_x, float tau_y, float tau_z)
{
    // determine directions for tau_x, tau_y, tau_z
    // tau_x requires left motors to match, right motors to match
    // tau_y requires front motors to match, back motors to match
    // tau_z requires diagonals to match
    bldc->throttles.speed_fl = throttle + tau_x - tau_y + tau_z;
    bldc->throttles.speed_fr = throttle - tau_x - tau_y - tau_z;
    bldc->throttles.speed_bl = throttle + tau_x + tau_y - tau_z;
    bldc->throttles.speed_br = throttle - tau_x + tau_y + tau_z;

    bldc_clamp(&bldc->throttles.speed_fl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    bldc_clamp(&bldc->throttles.speed_fr, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    bldc_clamp(&bldc->throttles.speed_bl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
    bldc_clamp(&bldc->throttles.speed_br, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

    return STATUS_OK;
}

status_t bldc_send(bldc_handle_t * bldc)
{
    if (dshot_queue(&bldc->motor_fl, bldc->throttles.speed_fl, 0) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_queue(&bldc->motor_fr, bldc->throttles.speed_fr, 0) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_queue(&bldc->motor_bl, bldc->throttles.speed_bl, 0) != STATUS_OK)
        return STATUS_ERR;
    if (dshot_queue(&bldc->motor_br, bldc->throttles.speed_br, 0) != STATUS_OK)
        return STATUS_ERR;

    return STATUS_OK;
}

status_t bldc_clamp(uint16_t * throttle, uint16_t min, uint16_t max)
{
    if (*throttle > max)
        *throttle = max;
    else if (*throttle < min)
        *throttle = min;

    return STATUS_OK;
}