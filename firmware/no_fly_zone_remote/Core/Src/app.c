#include "app.h"
#include "main.h"
#include "stm32_hal_legacy.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_iwdg.h"
#include "stm32l4xx_hal_tim.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>

#include "bms.h"
#include "encoder.h"
#include "nrf24.h"
#include "oled_helper.h"
#include "quaternion.h"
#include "joystick_helper.h"
#include "rf_structs.h"
#include "ssd1306.h"

#define RF_TX_ADDR 0xE7E7E7E7
#define RF_RX_ADDR 0xE7E7E7E7

#define N_ADC_SAMPLES 5
#define ADC_SAMP_RATE 200 // Hz
#define ADC_SAMP_DT   (float)(1.0 / ADC_SAMP_RATE)

// add ms to end of each
#define OLED_UPDATE_T      50
#define ARM_STATE_UPDATE_T 500
#define ENC_UPDATE_T       100

#define OLED_RES_X 128
#define OLED_RES_Y 64
#define OLED_PAGES (OLED_RES_Y / 8)

uint16_t samples[N_ADC_SAMPLES];

// reference for buffer sizes found in ssd1306.h
uint8_t oled_buff[OLED_RES_X * OLED_PAGES];
uint8_t tx_buff[OLED_RES_X * OLED_PAGES + 13];

oled_pages_t page = PAGE_BATT_LVL;
uint8_t sw_state = 0;

volatile bool adc_dr = false;
float kp = 0, ki = 0, kd = 0;

void delay_us(uint32_t delay);
void delay_ms(uint32_t delay);
uint32_t micros_get_tick();
uint32_t millis_get_tick();

// nRF24L01 parameters, should match receiver params
nrf24_handle_t tx = {

    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .delay_ms = HAL_Delay,
    .delay_us = delay_us,

    .power_level = NRF24_TX_PWR_0DBM,
    .data_rate = NRF24_DATARATE_2MBPS,
    .freq_ch = 110,         // 2400 MHz + 110 = 2.510 GHz

    .nrf24_irqs_t = {
        .rx_dr_en = false,
        .tx_ds_en = false,
        .max_rt_en = false,
    },
};

nrf24_pipe_t ack_pipe = {

    .address = RF_RX_ADDR,
    .payload_type = NRF24_PAYLOAD_DYNAMIC,
    .ack = NRF24_AUTO_ACK_PAYLOAD,
    .num = NRF24_PIPE0,
};

ssd1306_handle_t oled = {

    .frame_buff = oled_buff,
    .tx_buff = tx_buff,
    .mode = SSD1306_PHY_MODE_DMA,
    .res_x = OLED_RES_X,
    .res_y = OLED_RES_Y,
};

encoder_handle_t enc = {

    .sw_gpio = ENC_SW_GPIO_Port,
    .sw_pin = ENC_SW_Pin,
    .mode = ENCODER_MODE_TIM,
    .t_update = 100,
};

bms_handle_t bms = {

    .type = BMS_LIPO_TYPE,
    .cell_cnt = 1,
    .div = 2.005,     // found by measuring resistor divider network
    .v_ref = 3.33,    // exact measurement
    .v_offset = 0.13, // measured
};

IWDG_HandleTypeDef * iwdg;

TIM_HandleTypeDef * tim_us_l;
uint16_t tim_us_cnt_h;

joystick_calibration_t joysticks_cal;

arm_state_t arm_state_remote = ARM_STATE_DISARMED;

void encoder_callback(encoder_handle_t * henc, encoder_event_t enc_event, encoder_event_t sw_event);
void bms_callback(bms_handle_t * bms, bms_event_t event);
static void euler_rates_to_quat(float w_x, float w_y, float w_z, quaternion_t * q_des);

void app_init(ADC_HandleTypeDef * hadc,
              DMA_HandleTypeDef * hdma,
              I2C_HandleTypeDef * hi2c,
              SPI_HandleTypeDef * hspi,
              TIM_HandleTypeDef * htim_adc,
              TIM_HandleTypeDef * henc,
              TIM_HandleTypeDef * htim_us_l,
              IWDG_HandleTypeDef * hiwdg)
{
    tx.hspi = hspi;
    oled.hi2c = hi2c;
    enc.htim = henc;
    tim_us_l = htim_us_l;
    iwdg = hiwdg;

    /******************************************** Configure nRF24L01 ******************************************/
    // microsecond timer must be started for rf_init()
    HAL_TIM_Base_Start_IT(tim_us_l);

    nrf24_init(&tx);
    nrf24_set_retransmit(&tx, NRF24_RETRY_DELAY_500, NRF24_RETRY_CNT_3); // no retransmits
    nrf24_open_rx_pipe(&tx, &ack_pipe);                                  // open RX pipe on 0
    nrf24_open_tx_pipe(&tx, RF_TX_ADDR);                                 // match TX with RX for auto acks
    /**********************************************************************************************************/

    /************************************** Configure battery management **************************************/
    bms_init(&bms, bms_callback);
    /**********************************************************************************************************/

    /****************************************** Configure encoder *********************************************/
    encoder_init(&enc);
    encoder_constrain(&enc, PAGE_BATT_LVL, PAGE_TUNE_PID); // set limits
    encoder_set_pos_mode(&enc, ENCODER_POS_WRAP);          // wrap from PAGE_BATT_LVL to PAGE_TUNE_PID
    encoder_register_callback(&enc, encoder_callback);     // set event callback function

    // start encoder
    HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
    /**********************************************************************************************************/

    /***************************************** Configure joystick *********************************************/
    // config joystick calibration values
    joysticks_calibration_init(&joysticks_cal);
    joysticks_cal.millis_get_tick = millis_get_tick;
    /**********************************************************************************************************/

    /****************************************** Configure ADC + DMA *******************************************/
    HAL_ADC_Start_DMA(hadc, (uint32_t *)samples, N_ADC_SAMPLES); // enable ADC with DMA transfers
    HAL_TIM_Base_Start(htim_adc);                                // start ADC conversions (200 Hz)
    /**********************************************************************************************************/

    /***************************************** Enable and start OLED ******************************************/
    ssd1306_init(&oled);
    ssd1306_clear(&oled);
    ssd1306_update(&oled);
    /**********************************************************************************************************/
}

void app(void)
{
    uint8_t remote_batt_lvl = 0;
    uint8_t quad_batt_lvl = 0;

    uint32_t last_tune_update = 0;
    active_tune_param_t curr_tune_param = ACTIVE_TUNE_NONE;

    joystick_t joysticks;

    uint32_t last_arm_state_update = 0;

    uint32_t adc_samples_cnt = 0;

    quaternion_t q_des = { .q1 = 1, .q2 = 0, .q3 = 0, .q4 = 0 };

    while (1)
    {
        // to arm, pull throttle all the way down and press encoder

        // flight mode
        // 1. read joysticks
        // 2. convert joysticks to desired angular rates
        // 3. convert rates to desired quaternion
        // 4. send over radio
        // 5. update oled --> figure out where to place update function

        // tune mode
        // must be disarmed
        // 1. read encoder
        // 2. control menu
        // 3. tune PID values
        // 4. send PID values over radio
        // 5. write PID values to non-volatile memory

        // pages 2 and 3 of OLED should only be able to be accessed when in DISARMED mode
        // if OLED is in ARMED mode, it is locked on page 1

        HAL_IWDG_Refresh(iwdg); // will cause system reset if refresh() not called every ~0.5s

        // stay disarmed until throttle is pulled all the way down while encoder
        // button is pressed
        if (sw_state && page == PAGE_BATT_LVL && joysticks.throttle > 3000 && millis_get_tick() - last_arm_state_update > ARM_STATE_UPDATE_T)
        {
            if (arm_state_remote == ARM_STATE_ARMED)
                arm_state_remote = ARM_STATE_DISARMED;
            else if (arm_state_remote == ARM_STATE_DISARMED)
                arm_state_remote = ARM_STATE_ARMED;

            curr_tune_param = ACTIVE_TUNE_NONE;
            last_arm_state_update = millis_get_tick();
        }

        if (sw_state && arm_state_remote == ARM_STATE_DISARMED && page == PAGE_JOYSTICK && !joysticks_cal.is_calibrating)
        {
            joysticks_calibration_start(&joysticks_cal);
        }

        if (adc_dr)
        {
            adc_dr = false;
            adc_samples_cnt++;

            // convert joysticks to euler angles and then quaternions
            memcpy((uint16_t *)&joysticks, (uint16_t *)samples, sizeof(joysticks));

            float throttle_norm, pitch_norm, roll_norm, yaw_norm;
            joysticks_condition(&joysticks, &joysticks_cal, &throttle_norm, &pitch_norm, &roll_norm, &yaw_norm);

            // float buff[4] = { throttle_norm, pitch_norm, roll_norm, yaw_norm };
            // uint16_t buff[4] = { joysticks.throttle, joysticks.pitch, joysticks.roll, joysticks.yaw };
            // CDC_Transmit_FS((uint8_t *)buff, sizeof(buff));

            float percent;
            bms_update(&bms, (float)samples[4] / ADC_RES_CNT, &percent);
            remote_batt_lvl = (uint8_t)percent;

            uint8_t key;
            uint16_t shaped_throttle = 0;

            switch (arm_state_remote)
            {
            case ARM_STATE_DISARMED:

                if (page == PAGE_JOYSTICK)
                    joysticks_calibrate(&joysticks, &joysticks_cal);
                else if (page == PAGE_TUNE_PID)
                    joysticks_tune_pid(pitch_norm, roll_norm, &kp, &ki, &kd, &last_tune_update, &curr_tune_param);

                key = DISARMED_KEY;
                break;

            case ARM_STATE_ARMED:

                // map joystick positions to euler angle rates
                float pitch_rate_dps, roll_rate_dps, yaw_rate_dps;
                pos_to_euler_rates(pitch_norm, roll_norm, yaw_norm, &pitch_rate_dps, &roll_rate_dps, &yaw_rate_dps);

                // convert to radians per second
                float pitch_rate_rps = pitch_rate_dps * M_PI / 180.0;
                float roll_rate_rps = roll_rate_dps * M_PI / 180.0;
                float yaw_rate_rps = yaw_rate_dps * M_PI / 180.0;

                // convert angular rates to desired quaternion
                euler_rates_to_quat(roll_rate_rps, pitch_rate_rps, yaw_rate_rps, &q_des);
                // CDC_Transmit_FS((uint8_t *)&q_des, sizeof(q_des));

                // if throttle < halfway, throttle = 0
                // handle jerking, accleration, etc
                /* Consider having centered throttle mean no vertical velocity, below center = descend, above center = ascend */
                pos_to_throttle(throttle_norm, &shaped_throttle);

                key = ARMED_KEY;
                break;

            case ARM_STATE_FAILSAFE:
            default:
                key = FAILSAFE_KEY;
                break;
            }

            // uint16_t data[2] = { joysticks.throttle, shaped_throttle };
            // CDC_Transmit_FS((uint8_t *)&data, sizeof(data));

            // instead of doing joysticks --> angles
            // do joysticks--> angle rates, then integrate and convert those into
            // quaternion

            rf_packet_params_t pkt = {
                .throttle = shaped_throttle,
                .q_des = {
                    .q1 = q_des.q1,
                    .q2 = q_des.q2,
                    .q3 = q_des.q3,
                    .q4 = q_des.q4,
                },
                .kp = kp,
                .ki = ki,
                .kd = kd,
                .key = key,
            };

            rf_ack_params_t ack;
            uint8_t ack_len;

            if (nrf24_send(&tx, (uint8_t *)&pkt, sizeof(rf_packet_params_t), true, (uint8_t *)&ack, &ack_len) == NRF24_OK)
            {
                if (ack_len == sizeof(rf_ack_params_t))
                {
                    quad_batt_lvl = ack.quad_batt_lvl;
                    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
                }
            }

            // update OLED after RF packet has been recently transmitted as not to disturb packet frequency
            // since ADC is running at 200Hz, OLED updates at 20Hz
            if (adc_samples_cnt % 10 == 0)
            {
                oled_params_t oled_info = {
                    .page = page,
                    .arm_state = arm_state_remote,
                    .rx_batt = quad_batt_lvl,
                    .tx_batt = remote_batt_lvl,
                    .joysticks = &joysticks,
                    .throttle_norm = throttle_norm,
                    .pitch_norm = pitch_norm,
                    .roll_norm = roll_norm,
                    .yaw_norm = yaw_norm,
                    .kp = kp,
                    .ki = ki,
                    .kd = kd,
                    .tuning_param = curr_tune_param,
                    .is_calibrating = joysticks_cal.is_calibrating,
                };

                oled_update(&oled, &oled_info);
            }
        }

        encoder_update(&enc);
    }
}

/************************************************* Interrupt Callbacks *****************************************************/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
    adc_dr = true;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    // from ssd1306.h
    ssd1306_callback(hi2c);
}
/***************************************************************************************************************************/

void encoder_callback(encoder_handle_t * henc, encoder_event_t enc_event, encoder_event_t sw_event)
{
    switch (enc_event)
    {
    case ENCODER_EVENT_CW_TURN:
    case ENCODER_EVENT_CCW_TURN:
        // do not allow page changes when in armed mode or during calibration process
        if (arm_state_remote == ARM_STATE_ARMED)
            encoder_set_position(henc, PAGE_BATT_LVL);
        if (arm_state_remote == ARM_STATE_DISARMED && joysticks_cal.is_calibrating)
            encoder_set_position(henc, PAGE_JOYSTICK);

        page = encoder_get_position(henc);
        break;

    case ENCODER_EVENT_NONE:
    default:
    }

    switch (sw_event)
    {
    case ENCODER_EVENT_SW_RELEASE:
        sw_state = 0;
        break;
    case ENCODER_EVENT_SW_PRESS:
        sw_state = 1;
        break;
    default:
    }
}

void bms_callback(bms_handle_t * bms, bms_event_t event)
{
    // if battery is empty, go to failsafe mode
    if (event == BMS_EVENT_CHARGE_EMPTY)
    {
        arm_state_remote = ARM_STATE_FAILSAFE;
    }
}

static void euler_rates_to_quat(float w_x, float w_y, float w_z, quaternion_t * q_des)
{
    // place gyro angular rates (radians per second) into quaternion
    quaternion_t q_rates = {
        .q1 = 0,
        .q2 = w_x,
        .q3 = -w_y,
        .q4 = w_z,
    };

    quaternion_t q_des_dot;

    // integrate
    // q_des_dot = 0.5 * q_des * [0, w_x, w_y, w_z]
    quat_mult(*q_des, q_rates, &q_des_dot);
    quat_mult_scalar(&q_des_dot, 0.5);

    // q_des = q_des + q_des_dot * dt
    quat_mult_scalar(&q_des_dot, ADC_SAMP_DT);
    quat_add(*q_des, q_des_dot, q_des);

    // normalize
    quat_normalize(q_des);
}

void delay_us(uint32_t delay)
{
    uint32_t start_us = micros_get_tick();
    while (micros_get_tick() - start_us < delay)
        ;
}

void delay_ms(uint32_t delay)
{
    delay_us(delay * 1000);
}

uint32_t micros_get_tick()
{
    uint16_t tim_low, tim_high;

    // check race conditions when doing non-atomic read over 2 registers
    do
    {
        tim_high = tim_us_cnt_h;
        tim_low = __HAL_TIM_GET_COUNTER(tim_us_l);

    } while (tim_us_cnt_h != tim_high);

    return ((uint32_t)tim_high << 16) | (uint32_t)tim_low;
}

uint32_t millis_get_tick()
{
    return micros_get_tick() / 1000;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
    if (htim == tim_us_l)
        tim_us_cnt_h++;
}