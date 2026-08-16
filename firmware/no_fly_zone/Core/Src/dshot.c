#include "dshot.h"
#include <string.h>

/************************* DShot Helper Functions **********************/
static void dshot_calculate_crc(uint16_t data, dshot_channel_t ch);
/***********************************************************************/

// index 0 in these arrays corresponds to DSHOT_CH_1
// index 1 corresponds to DSHOT_CH_2
// index z corresponds to DSHOT_CH_(z+1)
static uint32_t packets[DSHOT_MAX_N][DSHOT_MAX_PKT_W];               // stores encoded packets ready to be sent
static uint32_t packets_temp[DSHOT_MAX_N][DSHOT_MAX_PKT_W];          // stores encoded packets ready to be sent
static uint32_t packets_len[DSHOT_MAX_N];
static uint32_t t1h_ccr[DSHOT_MAX_N];                           // list of "on" times for '1' bit for each timer
static uint32_t t0h_ccr[DSHOT_MAX_N];                           // list of "on" times for '0' bit for each timer
static uint8_t channel_en[DSHOT_MAX_N];                         // is current channel being used

// Execution order
// 1. Enable TIM + DMA
// 2. Timer counts starting from 0
// 3. CNT reaches CCRx
// 4. TIM generates compare event
// 5. TIM generates DMA request
// 6. DMA transfers next bit from pkt into preload compare register
// 7. CNT reaches ARR, update event occurs
// 8. preload compare register is transferred to compare register
// 9. After packet width # of transfers, packets_temp is loaded into packets, restarting process

status_t dshot_init(dshot_handle_t * dshot)
{
    // PREREQUISITE
    // MUST HAVE DMA STREAMS FOR EACH TIMER AND CHANNEL ENABLED IN CUBEMX (CIRCULAR MODE)
    // Can use any channel with any timer frequency
    if (dshot->n > DSHOT_MAX_N || dshot->bitrate > DSHOT_BR_1200_KBPS || dshot->frequency > DSHOT_FREQ_8_KHZ)
        return STATUS_INVALID;
    
    for (uint8_t i = 0; i < dshot->n; i++)
    {
        channel_en[i] = 0;          // channel is not currently active

        __HAL_TIM_SET_COMPARE(dshot->stream[i].htim, dshot->stream[i].channel, 0);              // set duty cycle to 0
        __HAL_TIM_ENABLE_OCxPRELOAD(dshot->stream[i].htim, dshot->stream[i].channel);           // enable output compare preload

        uint32_t tim_freq = dshot->stream[i].freq;
        uint32_t dshot_pkt_freq, dshot_bit_freq;

        // set bit rate
        switch (dshot->bitrate)
        {
            case DSHOT_BR_150_KBPS:
                dshot_bit_freq = 150000;
                break;

            case DSHOT_BR_300_KBPS:
                dshot_bit_freq = 300000;
                break;

            case DSHOT_BR_600_KBPS:
                dshot_bit_freq = 600000;
                break;

            case DSHOT_BR_1200_KBPS:
                dshot_bit_freq = 1200000;
                break;

            default:
                return STATUS_INVALID;
        }

        // select packet frequency
        switch (dshot->frequency)
        {
            case DSHOT_FREQ_500_HZ:
                dshot_pkt_freq = 500;
                break;
                
            case DSHOT_FREQ_1_KHZ:
                dshot_pkt_freq = 1000;
                break;

            case DSHOT_FREQ_2_KHZ:
                dshot_pkt_freq = 2000;
                break;

            case DSHOT_FREQ_4_KHZ:
                dshot_pkt_freq = 4000;
                break;

            case DSHOT_FREQ_8_KHZ:
                dshot_pkt_freq = 8000;
                break;
            
            default:
                return STATUS_INVALID;
        }

        uint32_t arr = (uint32_t) (tim_freq / dshot_bit_freq);                      // calculate timer period for one bit
        uint32_t pkt_len = (uint32_t) (dshot_bit_freq / dshot_pkt_freq);            // calculate total timer cycles needed to achieve frequency

        if (pkt_len < DSHOT_MIN_PKT_W)                                  // dshot packet needs to be completed before next one begins
            return STATUS_INVALID;

        packets_len[i] = pkt_len;

        if (arr < 20)                                               // timer frequency needs to be higher to achieve accurate bit times
            return STATUS_INVALID;

        __HAL_TIM_SET_AUTORELOAD(dshot->stream[i].htim, arr - 1);   // set ARR value to achieve desired frequency
        t1h_ccr[i] = (uint32_t) (DSHOT_T1H_FRAC * arr) - 1;         // calculate CCRx value for '1' bit
        t0h_ccr[i] = (uint32_t) (DSHOT_T0H_FRAC * arr) - 1;         // calculate CCRx value for '0' bit
    }

    return STATUS_OK;
}

status_t dshot_start(dshot_handle_t * dshot, dshot_channel_t * selected_ch, uint8_t n)
{
    // send encoded packet on selected channels (n # of channels)
    if (n > dshot->n)
        return STATUS_INVALID;

    for (uint8_t i = 0; i < n; i++)
    {
        dshot_channel_t ch = selected_ch[i];
        channel_en[ch] = 1;                      // channel is active

        // if channel is already active, skip
        if ((HAL_TIM_PWM_Start_DMA(dshot->stream[ch].htim, dshot->stream[ch].channel, packets[ch], packets_len[ch]) != HAL_OK))
            continue;
    }

    return STATUS_OK;
}

status_t dshot_stop(dshot_handle_t * dshot, dshot_channel_t * selected_ch, uint8_t n)
{
    if (n > dshot->n)
        return STATUS_INVALID;

    // disable channel
    for (uint8_t i = 0; i < n; i++)
    {
        dshot_channel_t ch = selected_ch[i];
        channel_en[ch] = 0;
    }

    return STATUS_OK;
}

status_t dshot_queue(dshot_handle_t * dshot, uint16_t throttle, uint8_t tel_req, dshot_channel_t ch)
{
    if (throttle > DSHOT_MAX_THROTTLE)
        return STATUS_INVALID;

    if (ch >= dshot->n)
        return STATUS_INVALID;

    uint16_t data = (throttle << 1) | !!tel_req;
    uint8_t ind = 0;

    // encode binary packet into a series of on/off times
    for (uint8_t i = DSHOT_THROTTLE_W + DSHOT_TEL_REQ_W - 1; i < 255; i--)
    {
        // if channel is not currently running, write directly to packet address
        if (!channel_en[ch])
        {
            // encode bits into timer compare values
            if (data & (1 << i))
                packets[ch][ind] = t1h_ccr[ch];
            else
                packets[ch][ind] = t0h_ccr[ch];
        }
        
        // write directly to next packet regardless of channel status
        if (data & (1 << i))
            packets_temp[ch][ind] = t1h_ccr[ch];
        else
            packets_temp[ch][ind] = t0h_ccr[ch];

        ind++;
    }

    dshot_calculate_crc(data, ch);

    for (uint16_t i = DSHOT_MIN_PKT_W - 1; i < packets_len[ch]; i++)
    {
        if (!channel_en[ch])
            packets[ch][i] = 0;                    // 0 duty cycle for last bit
            
        packets_temp[ch][i] = 0;
    }

    return STATUS_OK;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t ch;

    switch (htim->Channel)
    {
        // queue next packet unless deactivated
        case HAL_TIM_ACTIVE_CHANNEL_1:

            ch = 0;
            if (!channel_en[ch])
                HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);

            break;

        case HAL_TIM_ACTIVE_CHANNEL_2:

            ch = 1;
            if (!channel_en[ch])
                HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);

            break;

        case HAL_TIM_ACTIVE_CHANNEL_3:

            ch = 2;
            if (!channel_en[ch])
                HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);

            break;

        case HAL_TIM_ACTIVE_CHANNEL_4:

            ch = 3;
            if (!channel_en[ch])
                HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);

            break;

        default:
            return;
    }

    memcpy(&packets[ch], &packets_temp[ch], packets_len[ch] * sizeof(packets[0][0]));            // must be completed before start of next packet
}

/************************* DShot Helper Functions **********************/
static void dshot_calculate_crc(uint16_t data, dshot_channel_t ch)
{
    // XOR all nibbles together
    uint8_t crc = ((data ^ (data >> 4)) ^ (data >> 8)) & 0x0f;

    uint8_t ind = 0;
    uint8_t offset = DSHOT_THROTTLE_W + DSHOT_TEL_REQ_W;

    // encode CRC into timer compare values
    for (uint8_t i = DSHOT_CRC_W - 1; i < 255; i--)
    {
        if (!channel_en[ch])
        {
            if (crc & (1 << i))
                packets[ch][ind + offset] = t1h_ccr[ch];
            else
                packets[ch][ind + offset] = t0h_ccr[ch];
        }
            
        if (crc & (1 << i))
            packets_temp[ch][ind + offset] = t1h_ccr[ch];
        else
            packets_temp[ch][ind + offset] = t0h_ccr[ch];

        ind++;
    }
}
/***********************************************************************/
