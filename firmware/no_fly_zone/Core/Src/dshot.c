#include "dshot.h"

static void dshot_calculate_crc(uint16_t throttle, DShotChannel ch);

static uint32_t packets[DSHOT_MAX_N][DSHOT_PKT_W];
static uint32_t t1h_ccr[DSHOT_MAX_N];
static uint32_t t0h_ccr[DSHOT_MAX_N];

ErrorType dshot_init(DShotParams * dshot)
{
    // PREREQUISITE
    // MUST HAVE DMA STREAMS FOR EACH TIMER AND CHANNEL ENABLED IN CUBEMX
    if (dshot->n > DSHOT_MAX_N || dshot->bitrate > DSHOT_BR_1200)
        return RET_INVALID_CFG;
    
    for (uint8_t i = 0; i < dshot->n; i++)
    {
        __HAL_TIM_SET_COMPARE(dshot->htims[i], dshot->channels[i], 0);           // set duty cycle to 0
        __HAL_TIM_ENABLE_OCxPRELOAD(dshot->htims[i], dshot->channels[i]);        // enable output compare preload

        uint32_t tim_freq = dshot->f_tims[i];
        uint32_t dshot_freq;

        // set bit rate
        switch (dshot->bitrate)
        {
            case DSHOT_BR_150:
                dshot_freq = 150000;
                break;

            case DSHOT_BR_300:
                dshot_freq = 300000;
                break;

            case DSHOT_BR_600:
                dshot_freq = 600000;
                break;

            case DSHOT_BR_1200:
                dshot_freq = 1200000;
                break;

            default:
                return RET_INVALID_CFG;
        }

        uint32_t arr = (uint32_t) (tim_freq / dshot_freq);

        if (arr < 20)                                               // timer frequency needs to be higher to achieve accurate bit times
            return RET_INVALID_CFG;

        __HAL_TIM_SET_AUTORELOAD(dshot->htims[i], arr - 1);         // set ARR value to achieve desired frequency
        t1h_ccr[i] = (uint32_t) (DSHOT_T1H_FRAC * arr) - 1;         // calculate CCRx value for '1' bit
        t0h_ccr[i] = (uint32_t) (DSHOT_T0H_FRAC * arr) - 1;         // calculate CCRx value for '0' bit
    }

    return RET_OK;
}


ErrorType dshot_encode(DShotParams * dshot, uint16_t throttle, uint8_t tel_req, DShotChannel ch)
{
    if ((throttle > DSHOT_MAX_THROTTLE) || (throttle < DSHOT_MIN_THROTTLE && throttle != 0))
        return RET_INVALID_CFG;

    if (ch > dshot->n)
        return RET_INVALID_CFG;

    uint8_t ind = 0;

    for (uint8_t i = DSHOT_THROTTLE_W - 1; i < 255; i--)
    {
        // encode bits into timer compare values
        if (throttle & (1 << i))
            packets[ch][ind] = t1h_ccr[ch];
        else
            packets[ch][ind] = t0h_ccr[ch];

        ind++;
    }

    if (tel_req)
        packets[ch][ind] = t1h_ccr[ch];
    else
        packets[ch][ind] = t0h_ccr[ch];

    dshot_calculate_crc(throttle, ch);

    packets[ch][16] = 0;                    // 0 duty cycle for last bit

    return RET_OK;
}

ErrorType dshot_send(DShotParams * dshot, DShotChannel * selected_ch, uint8_t n)
{
    if (n > dshot->n)
        return RET_INVALID_CFG;

    for (uint8_t i = 0; i < n; i++)
    {
        DShotChannel ch = selected_ch[i];
        HAL_TIM_PWM_Start_DMA(dshot->htims[ch], dshot->channels[ch], packets[ch], DSHOT_PKT_W);
    }

    return RET_OK;
}

static void dshot_calculate_crc(uint16_t throttle, DShotChannel ch)
{
    // XOR all nibbles together
    uint8_t crc = ((throttle ^ (throttle >> 4)) ^ (throttle >> 8)) & 0x0f;

    uint8_t ind = 0;
    uint8_t offset = DSHOT_THROTTLE_W + DSHOT_TEL_REQ_W;

    // encode CRC into timer compare values
    for (uint8_t i = DSHOT_CRC_W - 1; i < 255; i--)
    {
        if (crc & (1 << i))
            packets[ch][ind + offset] = t1h_ccr[ch];
        else
            packets[ch][ind + offset] = t0h_ccr[ch];

        ind++;
    }
}