#include "dshot.h"
#include <string.h>

/************************* DShot Helper Functions **********************/
static void dshot_init_buffer(dshot_handle_t * dshot);
static status_t dshot_encode_pkt(dshot_handle_t * dshot, uint16_t throttle, uint8_t tel_req);
static void dshot_calculate_crc(dshot_handle_t * dshot, uint16_t data);
static status_t dshot_get_bitrate(dshot_bitrate_t bitrate, uint32_t * br);
static status_t dshot_get_pkt_freq(dshot_freq_t frequency, uint32_t * f);
static status_t dshot_set_timer_period(dshot_handle_t * dshot);
static status_t dshot_set_pkt_len(dshot_handle_t * dshot);
static status_t dshot_set_bit_times(dshot_handle_t * dshot);
/***********************************************************************/

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

// Specifics
// - _buff_len is the required amount of DShot bits in order to achieve f_pkt; first DSHOT_MIN_PKT_W elements are DShot timer compare widths
//   remaining elements are padding 0s
// - _dma_buff is twice the maximum length that DShot allows possible, allowing two packets to fit in the buffer
// - when DShot starts, the first packet (contained in the first _buff_len words in the buffer) is sent
// - after the first packet is sent, the HalfCpltCallback is called, allowing a new packet to be placed into the first half of the buffer
// - after the second packet is sent, the CpltCallback is called, allowing a new packet to be placed into the second half of the buffer

status_t dshot_init(dshot_handle_t * dshot, TIM_HandleTypeDef * htim, uint32_t ch, uint32_t f_tim, dshot_bitrate_t bitrate, dshot_freq_t f_pkt)
{
    // PREREQUISITE
    // MUST HAVE DMA STREAMS FOR EACH TIMER AND CHANNEL ENABLED IN CUBEMX (CIRCULAR MODE)
    // Can use any channel with any timer frequency
    if (bitrate > DSHOT_BR_1200_KBPS || f_pkt > DSHOT_FREQ_8_KHZ || htim == NULL)
        return STATUS_INVALID;

    dshot->htim = htim;
    dshot->channel = ch;
    dshot->bitrate = bitrate;
    dshot->f_tim = f_tim;
    dshot->f_pkt = f_pkt;
    dshot->_active = DSHOT_STATUS_PAUSE;

    // consider using dynamic memory allocation for buffer since its size varies so much based on configuration

    __HAL_TIM_SET_COMPARE(dshot->htim, dshot->channel, 0);    // set duty cycle to 0
    __HAL_TIM_ENABLE_OCxPRELOAD(dshot->htim, dshot->channel); // enable output compare preload

    dshot_init_buffer(dshot);

    if (dshot_set_timer_period(dshot) == STATUS_INVALID)
        return STATUS_INVALID;
    if (dshot_set_pkt_len(dshot) == STATUS_INVALID)
        return STATUS_INVALID;
    if (dshot_set_bit_times(dshot) == STATUS_INVALID)
        return STATUS_INVALID;

    return STATUS_OK;
}

status_t dshot_start(dshot_handle_t * dshot)
{
    dshot->_active = DSHOT_STATUS_ACTIVE; // indicate channel is active

    // attempt to start DMA transactions with TIM
    if (HAL_TIM_PWM_Start_DMA(dshot->htim, dshot->channel, dshot->_dma_buff, dshot->_buff_len * 2) != HAL_OK)
        return STATUS_BUSY;

    return STATUS_OK;
}

status_t dshot_stop(dshot_handle_t * dshot)
{
    // at completion of current packet, TIM will stop, stopping packets
    dshot->_active = DSHOT_STATUS_PAUSE;
    return STATUS_OK;
}

status_t dshot_queue(dshot_handle_t * dshot, uint16_t throttle, uint8_t tel_req)
{
    if (throttle > DSHOT_MAX_THROTTLE)
        return STATUS_INVALID;

    // if DShot is stopped, prepare initial packet
    // if DShot is running, next packet will contain new information
    return dshot_encode_pkt(dshot, throttle, tel_req);
}

void dshot_complete_callback(dshot_handle_t * dshot, TIM_HandleTypeDef * htim)
{
    // ensure triggered channel is DShot channel
    if (dshot->htim == htim)
    {
        uint32_t channel = 0;
        switch (htim->Channel)
        {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            channel = TIM_CHANNEL_1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            channel = TIM_CHANNEL_2;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
            channel = TIM_CHANNEL_3;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
            channel = TIM_CHANNEL_4;
            break;
        default:
            return;
        }

        // ensure channels match
        if (dshot->channel != channel)
            return;

        // if stop has been requested, stop immediately
        if (dshot->_active == DSHOT_STATUS_PAUSE)
            HAL_TIM_PWM_Stop_DMA(dshot->htim, dshot->channel);

        // if DShot is complete, second half of buffer is free to write to
        // copy next packet in queue to current buffer
        memcpy((uint8_t *)(dshot->_dma_buff + dshot->_buff_len), (uint8_t *)dshot->_next_pkt, sizeof(dshot->_next_pkt));
    }
}

void dshot_half_complete_callback(dshot_handle_t * dshot, TIM_HandleTypeDef * htim)
{
    // ensure triggered channel is DShot channel
    if (dshot->htim == htim)
    {
        uint32_t channel = 0;
        switch (htim->Channel)
        {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            channel = TIM_CHANNEL_1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            channel = TIM_CHANNEL_2;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
            channel = TIM_CHANNEL_3;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
            channel = TIM_CHANNEL_4;
            break;
        default:
            return;
        }

        // ensure channels match
        if (dshot->channel != channel)
            return;
        
        // do not request to stop DMA in middle of transfer to avoid errors

        // if DShot is half complete, first half of buffer is free to write to
        // copy next packet in queue to current buffer
        memcpy((uint8_t *)dshot->_dma_buff, (uint8_t *)dshot->_next_pkt, sizeof(dshot->_next_pkt));
    }
}

/************************* DShot Helper Functions **********************/
static void dshot_init_buffer(dshot_handle_t * dshot)
{
    // set all memory locations to zero to begin
    memset(dshot->_dma_buff, 0, sizeof(dshot->_dma_buff));
    memset(dshot->_next_pkt, 0, sizeof(dshot->_next_pkt));
}

static status_t dshot_encode_pkt(dshot_handle_t * dshot, uint16_t throttle, uint8_t tel_req)
{
    uint16_t data = (throttle << 1) | !!tel_req;
    uint8_t ind = 0;

    // encode binary packet into a series of on/off times
    for (uint8_t i = DSHOT_THROTTLE_W + DSHOT_TEL_REQ_W - 1; i < 255; i--)
    {
        // build packet data
        if (data & (1 << i))
            dshot->_next_pkt[ind] = dshot->_t1h;
        else
            dshot->_next_pkt[ind] = dshot->_t0h;

        ind++;
    }

    dshot_calculate_crc(dshot, data);
    dshot->_next_pkt[DSHOT_MIN_PKT_W - 1] = 0;

    return STATUS_OK;
}

static void dshot_calculate_crc(dshot_handle_t * dshot, uint16_t data)
{
    // XOR all nibbles together
    uint8_t crc = ((data ^ (data >> 4)) ^ (data >> 8)) & 0x0f;

    uint8_t ind = 0;
    uint8_t offset = DSHOT_THROTTLE_W + DSHOT_TEL_REQ_W;

    // encode CRC into timer compare values
    for (uint8_t i = DSHOT_CRC_W - 1; i < 255; i--)
    {
        if (crc & (1 << i))
            dshot->_next_pkt[ind + offset] = dshot->_t1h;
        else
            dshot->_next_pkt[ind + offset] = dshot->_t0h;

        ind++;
    }
}

static status_t dshot_get_bitrate(dshot_bitrate_t bitrate, uint32_t * br)
{
    // select bit rate
    switch (bitrate)
    {
    case DSHOT_BR_150_KBPS:
        *br = 150000;
        break;
    case DSHOT_BR_300_KBPS:
        *br = 300000;
        break;
    case DSHOT_BR_600_KBPS:
        *br = 600000;
        break;
    case DSHOT_BR_1200_KBPS:
        *br = 1200000;
        break;
    default:
        return STATUS_INVALID;
    }

    return STATUS_OK;
}

static status_t dshot_get_pkt_freq(dshot_freq_t frequency, uint32_t * f)
{
    // select packet frequency
    switch (frequency)
    {
    case DSHOT_FREQ_500_HZ:
        *f = 500;
        break;
    case DSHOT_FREQ_1_KHZ:
        *f = 1000;
        break;
    case DSHOT_FREQ_2_KHZ:
        *f = 2000;
        break;
    case DSHOT_FREQ_4_KHZ:
        *f = 4000;
        break;
    case DSHOT_FREQ_8_KHZ:
        *f = 8000;
        break;
    default:
        return STATUS_INVALID;
    }

    return STATUS_OK;
}

static status_t dshot_set_timer_period(dshot_handle_t * dshot)
{
    uint32_t f_bit;

    if (dshot_get_bitrate(dshot->bitrate, &f_bit) == STATUS_INVALID)
        return STATUS_INVALID;

    // TIM ARR is calculated based on timer frequency and bit frequency
    dshot->_arr = (uint32_t)(dshot->f_tim / f_bit);

    // TIM ARR must be high enough for accurate bit resolution
    if (dshot->_arr < DSHOT_TIM_MIN_ARR)
        return STATUS_INVALID;

    return STATUS_OK;
}

static status_t dshot_set_pkt_len(dshot_handle_t * dshot)
{
    uint32_t f_bit, f_pkt;

    if (dshot_get_pkt_freq(dshot->f_pkt, &f_pkt) == STATUS_INVALID)
        return STATUS_INVALID;
    if (dshot_get_bitrate(dshot->bitrate, &f_bit) == STATUS_INVALID)
        return STATUS_INVALID;

    // # of bits needed to achieve correct packet frequency
    dshot->_buff_len = (uint32_t)(f_bit / f_pkt);

    // DMA buffer length needs to be at least as long as DShot packet
    if (dshot->_buff_len < DSHOT_MIN_PKT_W)
        return STATUS_INVALID;

    return STATUS_OK;
}

static status_t dshot_set_bit_times(dshot_handle_t * dshot)
{
    __HAL_TIM_SET_AUTORELOAD(dshot->htim, dshot->_arr - 1);     // set ARR value to achieve desired frequency

    dshot->_t1h = (uint32_t)(DSHOT_T1H_FRAC * dshot->_arr) - 1; // calculate CCRx value for '1' bit
    dshot->_t0h = (uint32_t)(DSHOT_T0H_FRAC * dshot->_arr) - 1; // calculate CCRx value for '0' bit

    return STATUS_OK;
}
/***********************************************************************/
