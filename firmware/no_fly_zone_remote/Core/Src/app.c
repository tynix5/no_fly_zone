#include "app.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_iwdg.h"
#include "stm32l4xx_hal_tim.h"
#include "usbd_cdc_if.h"

#include "bms.h"
#include "encoder.h"
#include "nrf24.h"
#include "oled_helper.h"
#include "quaternion.h"
#include "joystick_helper.h"
#include "rf_structs.h"
#include "ssd1306.h"

// change to 1 for acrobatic mode
#define ACROBATIC_MODE 0

#define RF_TX_ADDR 0xE7E7E7E7
#define RF_RX_ADDR 0xE7E7E7E7

#define N_ADC_SAMPLES         5
#define ARM_STATE_UPDATE_T_MS 500
#define ENCODER_UPDATE_T_MS   100

#define BMS_DIVISION_FACTOR 2.005f              // found by measuring resistor divider network
#define V_ADC_REF           3.33f               // exact measurement
#define V_ADC_OFFSET        0.13f               // measured

#define OLED_RES_X 128                          // pixels
#define OLED_RES_Y 64                           // pixels
#define OLED_PAGES (OLED_RES_Y / 8)

volatile bool adc_dr = false;                   // ADC data ready
uint16_t adc_samples[N_ADC_SAMPLES];            // DMA buffer for new ADC samples
uint32_t adc_samples_cnt = 0;                   // number of group conversions
uint8_t remote_batt_lvl = 0, quad_batt_lvl = 0; // battery levels

// reference for buffer sizes found in ssd1306.h
uint8_t oled_buff[OLED_RES_X * OLED_PAGES];                  // pixel buffer
uint8_t tx_buff[OLED_RES_X * OLED_PAGES + 13];               // TX buffer for DMA I2C transactions

oled_pages_t page = PAGE_BATT_LVL;                           // current OLED page
uint8_t sw_state = 0;                                        // encoder switch state

float kp = 15.0f, ki = 0.3f, kd = 0.5f;                      // preload K values for easier manipulation

IWDG_HandleTypeDef * iwdg;                                   // independent watchdog handler
TIM_HandleTypeDef * tim_us_l;                                // low 16-bits of 32-bit timer handle
uint16_t tim_us_cnt_h;                                       // high 16-bits of 32-bit timer

quaternion_t q_des = { .q1 = 1, .q2 = 0, .q3 = 0, .q4 = 0 }; // desired quaternion
arm_state_t arm_state_remote = ARM_STATE_DISARMED;           // current state of remote
joysticks_t joysticks;                                       // joystick structure handle
float yaw_ang_des;                                           // desired yaw angle (only NON-ACRO mode)

uint32_t last_arm_state_update = 0;                          // timestamp of last time mode changed betwen ARM/DISARM to DISARM/ARM
uint32_t last_tune_update = 0;                               // timestamp of last time tuner was updated (to prevent PID values from growing/shrinking too fast)
active_tune_param_t curr_tune_param = ACTIVE_TUNE_NONE;      // current parameter being tuned

void encoder_callback(encoder_handle_t * henc, encoder_event_t enc_event, encoder_event_t sw_event);
void bms_callback(bms_handle_t * bms, bms_event_t event);
static void monitor_arm_state_change_request();
static void monitor_calibration_request();
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
    .t_update = ENCODER_UPDATE_T_MS,
};

bms_handle_t bms = {

    .type = BMS_LIPO_TYPE,
    .cell_cnt = 1,
    .div = BMS_DIVISION_FACTOR,
    .v_ref = V_ADC_REF,
    .v_offset = V_ADC_OFFSET,
};

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
    joysticks_init(&joysticks);
    joysticks.millis_get_tick = millis_get_tick;
    /**********************************************************************************************************/

    /****************************************** Configure ADC + DMA *******************************************/
    HAL_ADC_Start_DMA(hadc, (uint32_t *)adc_samples, N_ADC_SAMPLES); // enable ADC with DMA transfers
    HAL_TIM_Base_Start(htim_adc);                                    // start ADC conversions (200 Hz)
    /**********************************************************************************************************/

    /***************************************** Enable and start OLED ******************************************/
    ssd1306_init(&oled);
    ssd1306_clear(&oled);
    ssd1306_update(&oled);
    /**********************************************************************************************************/
}

void app(void)
{
    while (1)
    {
        // PAGE 1
        // OLED is locked on this page when in ARM mode
        // PAGE 2
        // OLED is locked on this page when calibrating
        // PAGE 3
        // OLED is free to change

        HAL_IWDG_Refresh(iwdg); // will cause system reset if refresh() not called every ~0.5s

        monitor_arm_state_change_request();
        monitor_calibration_request();

        if (adc_dr)
        {
            adc_dr = false;
            adc_samples_cnt++;

            // extract joystick samples
            joysticks.throttle = adc_samples[0];
            joysticks.yaw = adc_samples[1];
            joysticks.pitch = adc_samples[2];
            joysticks.roll = adc_samples[3];

            // condition raw joystick inputs
            float throttle_norm, pitch_norm, roll_norm, yaw_norm;
            joysticks_condition(&joysticks, &throttle_norm, &pitch_norm, &roll_norm, &yaw_norm);

            // check battery level
            float percent;
            bms_update(&bms, (float)adc_samples[4] / ADC_RES_CNT, &percent);
            remote_batt_lvl = (uint8_t)percent;

            uint8_t key;
            uint16_t shaped_throttle = 0;

            switch (arm_state_remote)
            {
            case ARM_STATE_DISARMED:

                if (page == PAGE_JOYSTICK)
                    joysticks_calibrate(&joysticks);
                else if (page == PAGE_TUNE_PID)
                    joysticks_tune_pid(pitch_norm, roll_norm, &kp, &ki, &kd, &last_tune_update, &curr_tune_param);

                key = DISARMED_KEY;
                break;

            case ARM_STATE_ARMED:

                // ACROBATIC mode - Quadcopter will hold desired angle even after sticks return to center position
                // NON-ACROBATIC mode - Quadcopter will level when sticks return to center position, yaw angle will remain
                if (ACROBATIC_MODE)
                    joysticks_pos_to_quat_acro(pitch_norm, roll_norm, yaw_norm, &q_des);
                else
                    joysticks_pos_to_quat(pitch_norm, roll_norm, yaw_norm, &yaw_ang_des, &q_des); // keep track of current yaw position between updates

                // convert joystick position to DShot value
                joysticks_pos_to_throttle(throttle_norm, &shaped_throttle);

                // stream desired quaternion for debugging
                float data[] = { q_des.q1, q_des.q2, q_des.q3, q_des.q4 };
                CDC_Transmit_FS((uint8_t *)data, sizeof(data));

                key = ARMED_KEY;
                break;

            case ARM_STATE_FAILSAFE:
            default:
                key = FAILSAFE_KEY;
                break;
            }

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
                    .is_calibrating = joysticks.is_calibrating,
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
        if (arm_state_remote == ARM_STATE_DISARMED && joysticks.is_calibrating)
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

static void monitor_arm_state_change_request()
{
    // Monitor for an ARM state change request
    // Remote must be on battery level page, joystick throttle must be lowered all the way
    if (sw_state && page == PAGE_BATT_LVL && joysticks.throttle > 3000 && millis_get_tick() - last_arm_state_update > ARM_STATE_UPDATE_T_MS)
    {
        if (arm_state_remote == ARM_STATE_ARMED)
            arm_state_remote = ARM_STATE_DISARMED;
        else if (arm_state_remote == ARM_STATE_DISARMED)
        {
            arm_state_remote = ARM_STATE_ARMED;

            // reset
            q_des.q1 = 1.0f;
            q_des.q2 = 0.0f;
            q_des.q3 = 0.0f;
            q_des.q4 = 0.0f;
            yaw_ang_des = 0.0f;
        }

        curr_tune_param = ACTIVE_TUNE_NONE;
        last_arm_state_update = millis_get_tick();
    }
}

static void monitor_calibration_request()
{
    // Monitor for a calibration request
    // Remote must be disarmed, on joystick page with encoder pressed and currently not calibrating
    if (sw_state && arm_state_remote == ARM_STATE_DISARMED && page == PAGE_JOYSTICK && !joysticks.is_calibrating)
    {
        joysticks_calibration_start(&joysticks);
    }
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