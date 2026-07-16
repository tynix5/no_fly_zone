#ifndef DSHOT_H_
#define DSHOT_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "error_types.h"

/************************** DShot Packet Specifications ***********************/
#define DSHOT_FRAME_W                       16
#define DSHOT_PKT_W                         DSHOT_FRAME_W + 1               // include extra 0 duty cycle at end of packet for preload CMPRx

#define DSHOT_THROTTLE_W                    11
#define DSHOT_TEL_REQ_W                     1
#define DSHOT_CRC_W                         4

#define DSHOT_THROTTLE_SHIFT                5
#define DSHOT_TEL_REQ_SHIFT                 4
#define DSHOT_CRC_SHIFT                     0
/******************************************************************************/

/************************* DShot Protocol Specifications **********************/
// DShot '1' bit high times
#define DSHOT_150_T1H_US                    5.00
#define DSHOT_300_T1H_US                    2.50
#define DSHOT_600_T1H_US                    1.25
#define DSHOT_1200_T1H_US                   0.625
#define DSHOT_T1H_FRAC                      0.75                            // high time is 75% of total bit period

// DShot '0' bit high times
#define DSHOT_150_T0H_US                    2.50
#define DSHOT_300_T0H_US                    1.25
#define DSHOT_600_T0H_US                    0.625
#define DSHOT_1200_T0H_US                   0.313
#define DSHOT_T0H_FRAC                      0.375                           // high time is 37.5% of total bit period

// Dshot bit period
#define DSHOT_150_BIT_US                    6.67
#define DSHOT_300_BIT_US                    3.33
#define DSHOT_600_BIT_US                    1.67
#define DSHOT_1200_BIT_US                   0.83
/******************************************************************************/

#define DSHOT_MAX_N                         6
#define DSHOT_MAX_THROTTLE                  2047
#define DSHOT_MIN_THROTTLE                  48

// commands from betaflight DShot
typedef enum : uint8_t {

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

} DShotCommand;

// channel number corresponds to the order in which they are listed in init()
typedef enum : uint8_t {

    DSHOT_CH_1 = 0,
    DSHOT_CH_2,
    DSHOT_CH_3,
    DSHOT_CH_4,
    DSHOT_CH_5,
    DSHOT_CH_6

} DShotChannel;

typedef enum : uint8_t {

    // kbit/s
    DSHOT_BR_150 = 0,
    DSHOT_BR_300,
    DSHOT_BR_600,
    DSHOT_BR_1200

} DShotBitrate;

typedef struct {

    TIM_HandleTypeDef * htim;                               // timer handles for each DShot channel
    uint32_t channel;                                       // channels used for each timer (channel # must line up with timer in htims)
    uint32_t freq;                                          // timer frequencies --> used to calculate ARR and CCRx

} DShotStream;

typedef struct {

    DShotBitrate bitrate;                                   // DShot bit rate
    uint8_t n;                                              // number of active DShot channels

    DShotStream stream[DSHOT_MAX_N];                        // unique DShot channel parameters

} DShotParams;

/* Set timer ARR's, calculate CCRx values need for '1' and '0' bits */
ErrorType dshot_init(DShotParams * dshot);
/* Encode throttle amount into 17-byte packet */
ErrorType dshot_encode(DShotParams * dshot, uint16_t throttle, uint8_t tel_req, DShotChannel ch);
/* Start DMA on selected channels */
ErrorType dshot_send(DShotParams * dshot, DShotChannel * selected_ch, uint8_t n);
/* DMA interrupt callback - calls when all 17 bytes have been transferred to CCRx */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

#endif