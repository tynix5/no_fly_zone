#ifndef DSHOT_H_
#define DSHOT_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "generic_types.h"

/************************** DShot Packet Specifications ***********************/
#define DSHOT_FRAME_W        16
#define DSHOT_MIN_PKT_W      DSHOT_FRAME_W + 1 // include extra 0 duty cycle at end of packet for preload CMPRx
#define DSHOT_MAX_PKT_W      512               // > (min(DSHOT_BR_x) / min(DSHOT_FREQ_x))
#define DSHOT_TIM_MIN_ARR    20

#define DSHOT_THROTTLE_W     11
#define DSHOT_TEL_REQ_W      1
#define DSHOT_CRC_W          4

#define DSHOT_THROTTLE_SHIFT 5
#define DSHOT_TEL_REQ_SHIFT  4
#define DSHOT_CRC_SHIFT      0
/******************************************************************************/

/************************* DShot Protocol Specifications **********************/
// DShot '1' bit high times
#define DSHOT_150_T1H_US     5.00
#define DSHOT_300_T1H_US     2.50
#define DSHOT_600_T1H_US     1.25
#define DSHOT_1200_T1H_US    0.625
#define DSHOT_T1H_FRAC       0.75 // high time is 75% of total bit period

// DShot '0' bit high times
#define DSHOT_150_T0H_US     2.50
#define DSHOT_300_T0H_US     1.25
#define DSHOT_600_T0H_US     0.625
#define DSHOT_1200_T0H_US    0.313
#define DSHOT_T0H_FRAC       0.375 // high time is 37.5% of total bit period

// Dshot bit period
#define DSHOT_150_BIT_US     6.67
#define DSHOT_300_BIT_US     3.33
#define DSHOT_600_BIT_US     1.67
#define DSHOT_1200_BIT_US    0.83
/******************************************************************************/

#define DSHOT_MAX_N          6
#define DSHOT_MAX_THROTTLE   2047
#define DSHOT_MIN_THROTTLE   48

// commands from betaflight DShot
typedef enum : uint8_t
{
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1,
    DSHOT_CMD_BEEP2,
    DSHOT_CMD_BEEP3,
    DSHOT_CMD_BEEP4,
    DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO,
    DSHOT_CMD_SPIN_DIR_1,
    DSHOT_CMD_SPIN_DIR_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQ,
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_EXT_TEL_EN,
    DSHOT_CMD_EXT_TEL_DIS,
    DSHOT_CMD_SPIN_DIR_NORM = 20,
    DSHOT_CMD_SPIN_DIR_REV,
    DSHOT_CMD_LED0_ON,
    DSHOT_CMD_LED1_ON,
    DSHOT_CMD_LED2_ON,
    DSHOT_CMD_LED3_ON,
    DSHOT_CMD_LED0_OFF,
    DSHOT_CMD_LED1_OFF,
    DSHOT_CMD_LED2_OFF,
    DSHOT_CMD_LED3_OFF,
    DSHOT_CMD_SIG_LINE_TEL_DIS = 32,
    DSHOT_CMD_SIG_LIN_TEL_EN,
    DSHOT_CMD_SIG_LINE_CONT_ERPM_TEL,
    DSHOT_CMD_SIG_LIN_CONT_ERPM_PER_TEL,
    DSHOT_CMD_SIG_LINE_TEMP_TEL = 42,
    DSHOT_CMD_SIG_LINE_VOLT_TEL,
    DSHOT_CMD_SIG_LINE_CURR_TEL,
    DSHOT_CMD_SIG_LINE_CONSUMPTION_TEL,
    DSHOT_CMD_SIG_LINE_ERPM_TEL,
    DSHOT_CMD_SIG_LINE_ERPM_PER_TEL

} dshot_command_t;

typedef enum : uint8_t
{
    DSHOT_BR_150_KBPS = 0,
    DSHOT_BR_300_KBPS,
    DSHOT_BR_600_KBPS,
    DSHOT_BR_1200_KBPS

} dshot_bitrate_t;

typedef enum : uint8_t
{
    DSHOT_FREQ_500_HZ = 0,
    DSHOT_FREQ_1_KHZ,
    DSHOT_FREQ_2_KHZ,
    DSHOT_FREQ_4_KHZ,
    DSHOT_FREQ_8_KHZ,

} dshot_freq_t;

typedef enum : uint8_t
{
    DSHOT_STATUS_PAUSE = 0,
    DSHOT_STATUS_ACTIVE

} dshot_status_t;

typedef struct
{
    dshot_bitrate_t bitrate;  // DShot bit rate
    dshot_freq_t f_pkt;       // DShot packet sent rate
    TIM_HandleTypeDef * htim; // timer handle
    uint32_t channel;         // timer channel
    uint32_t f_tim;           // timer frequency (Hz)

    // private members
    uint32_t _dma_buff[2 * DSHOT_MAX_PKT_W]; // DMA buffer, stores 2 encoded DShot packets
                                             // packet 1 consists of first _buff_len bytes, packet 2 consists of second _buff_len bytes
    uint32_t _next_pkt[DSHOT_MIN_PKT_W];     // temporary DShot packet to be copied to either half of _dma_buff
    uint32_t _arr;                           // timer period
    uint32_t _buff_len;                      // total length of DShot packet + padding 0s
    uint32_t _t1h, _t0h;                     // CCRx timer values to encode 0's and 1's
    dshot_status_t _active;                  // channel active state

} dshot_handle_t;

/* Set timer ARR's, calculate CCRx values need for '1' and '0' bits, and determine length of DMA buffers */
status_t dshot_init(dshot_handle_t * dshot, TIM_HandleTypeDef * htim, uint32_t ch, uint32_t f_tim, dshot_bitrate_t bitrate, dshot_freq_t f_pkt);
/* Start continuously sending DShot packets on selected channels */
status_t dshot_start(dshot_handle_t * dshot);
/* Disable DShot packets */
status_t dshot_stop(dshot_handle_t * dshot);
/* Encode throttle amount into 17-byte packet */
status_t dshot_queue(dshot_handle_t * dshot, uint16_t throttle, uint8_t tel_req);
/* DShot DMA complete callback - occurs when first _buff_len bytes have been sent from DMA to TIM; place into HAL_TIM_PWM_PulseFinishedCallback */
void dshot_complete_callback(dshot_handle_t * dshot, TIM_HandleTypeDef * htim);
/* DShot DMA half complete callback - occurs second _buff_len bytes have been sent from DMA to TIM; place into HAL_TIM_PWM_PulseFinishedHalfCpltCallback */
void dshot_half_complete_callback(dshot_handle_t * dshot, TIM_HandleTypeDef * htim);

#endif