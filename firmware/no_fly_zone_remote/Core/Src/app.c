#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "encoder.h"
#include "oled_helper.h"
#include "quaternion.h"
#include "rf_structs.h"
#include "ssd1306.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>

#define RF_TX_ADDR         0xE7E7E7E7
#define RF_RX_ADDR         0xE7E7E7E7

#define N_ADC_SAMPLES      5
#define ADC_SAMP_RATE      25 // Hz
#define ADC_SAMP_DT        (float)(1.0 / ADC_SAMP_RATE)

#define CELL_COUNT         1  // 1S LiPo
#define CELL_MAX_V         4.2
#define CELL_MIN_V         3.75
#define BAT_MAX_V          CELL_COUNT * CELL_MAX_V
#define BAT_MIN_V          CELL_COUNT * CELL_MIN_V

#define MAX_PITCH_RATE_DEG 360
#define MIN_PITCH_RATE_DEG -360
#define MAX_ROLL_RATE_DEG  360
#define MIN_ROLL_RATE_DEG  -360
#define MAX_YAW_RATE_DEG   360
#define MIN_YAW_RATE_DEG   -360

#define ENC_UPDATE_T       100

#define OLED_RES_X         128
#define OLED_RES_Y         64
#define OLED_PAGES         (OLED_RES_Y / 8)

uint16_t samples[N_ADC_SAMPLES];

// reference for buffer sizes found in ssd1306.h
uint8_t oled_buff[OLED_RES_X * OLED_PAGES];
uint8_t tx_buff[OLED_RES_X * OLED_PAGES + 13];

oled_pages_t page = PAGE_1;
uint8_t sw_state = 0;

volatile uint8_t adc_dr = 0;
float kp = 0, ki = 0, kd = 0;

// nRF24L01 parameters, should match receiver params
nrf_handle_t tx = {

    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .delay_ms = HAL_Delay,

    .this_addr = RF_RX_ADDR,
    .node_addr = RF_TX_ADDR,

    .power_level = NRF_TX_PWR_0DBM,
    .data_rate = NRF_DATARATE_2MBPS,
    .freq_ch = 2,
    .payload_type = NRF_PAYLOAD_DYNAMIC,
    .ack = FEAT_ENABLE,

    .nrf_irqs_t = {
        .rx_dr = FEAT_DISABLE,
        .tx_ds = FEAT_DISABLE,
        .max_rt = FEAT_DISABLE,
    },
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

void encoder_callback(encoder_handle_t * henc, encoder_event_t event);
void encoder_sw_callback(encoder_handle_t * henc, encoder_event_t event);
static void condition_throttle(uint16_t throttle, uint16_t * shaped_throttle);
static void
shape_input(uint16_t x, uint16_t x_min, uint16_t x_max, uint16_t x_mid, float deadzone, float x_shaped_min, float x_shaped_max, float * x_shaped);
static void euler_rates_to_quat(float w_x, float w_y, float w_z, quaternion_t * q_des);
static void pos_to_euler_rates(joystick_t * joystick, float * pitch_rate, float * roll_rate, float * yaw_rate);
static float map(float x, float in_min, float in_max, float out_min,
                 float out_max); // convert from [in_min, in_max] to [out_min, out_max]
static uint8_t compute_batt(uint16_t batt_lvl_adc);
static void tune(joystick_t * joysticks, float * kp, float * ki, float * kd, uint32_t * last_tune, oled_active_tune_param_t * active_tune);

void app_init(ADC_HandleTypeDef * hadc,
              DMA_HandleTypeDef * hdma,
              I2C_HandleTypeDef * hi2c,
              SPI_HandleTypeDef * hspi,
              TIM_HandleTypeDef * htim,
              TIM_HandleTypeDef * henc)
{
    tx.hspi = hspi;
    oled.hi2c = hi2c;
    enc.htim = henc;

    rf_init(&tx);
    ssd1306_init(&oled);

    encoder_init(&enc);
    encoder_constrain(&enc, PAGE_1, PAGE_3);
    encoder_set_pos_mode(&enc, ENCODER_POS_WRAP);
    encoder_register_callback(&enc, encoder_callback);
    encoder_register_sw_callback(&enc, encoder_sw_callback);

    // enable ADC with DMA transfers
    HAL_ADC_Start_DMA(hadc, (uint32_t *)samples, N_ADC_SAMPLES);

    // start ADC conversions
    HAL_TIM_Base_Start(htim); // 25Hz

    // start encoder
    HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);

    ssd1306_clear(&oled);
    ssd1306_update(&oled);

    // read previous kp, ki, kd values from non-volatile
}

void app(void)
{
    uint8_t remote_batt_lvl = 0;
    uint8_t quad_batt_lvl = 0;

    uint32_t last_tune_update = 0;
    oled_active_tune_param_t curr_tune = ACTIVE_TUNE_NONE;

    joystick_t joysticks;

    quaternion_t q_des = {
        .q1 = 1,
        .q2 = 0,
        .q3 = 0,
        .q4 = 0,
    };

    quad_arm_status_t mode = QUAD_STATUS_DISARMED;
    uint32_t last_mode_change = 0;

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

        // stay disarmed until throttle is pulled all the way down while encoder button is pressed
        // move button press funciton to encoder library
        if (sw_state && page != PAGE_3 && joysticks.throttle > 3000 && HAL_GetTick() - last_mode_change > 500)
        {
            if (mode == QUAD_STATUS_ARMED)
                mode = QUAD_STATUS_DISARMED;
            else if (mode == QUAD_STATUS_DISARMED)
                mode = QUAD_STATUS_ARMED;

            curr_tune = ACTIVE_TUNE_NONE;
            last_mode_change = HAL_GetTick();
        }

        if (adc_dr)
        {
            // convert joysticks to euler angles and then quaternions
            memcpy((uint16_t *)&joysticks, (uint16_t *)samples, sizeof(joysticks));

            // tune mode
            if (mode == QUAD_STATUS_DISARMED && page == PAGE_3)
            {
                // problem with tuning control
                tune(&joysticks, &kp, &ki, &kd, &last_tune_update, &curr_tune);
            }

            // map joystick positions to euler angle rates
            float pitch_rate_deg, roll_rate_deg, yaw_rate_deg;
            pos_to_euler_rates(&joysticks, &pitch_rate_deg, &roll_rate_deg, &yaw_rate_deg);

            // convert to radians
            float pitch_rate_rad = pitch_rate_deg * M_PI / 180.0;
            float roll_rate_rad = roll_rate_deg * M_PI / 180.0;
            float yaw_rate_rad = yaw_rate_deg * M_PI / 180.0;

            // convert angular rates to desired quaternion
            // this is an acrobatic mode of sorts
            // if user wants quadcopter to return to level position after releasing sticks, do euler_to_quat()
            // must match those in madgwick filter for quadcopter
            euler_rates_to_quat(roll_rate_rad, pitch_rate_rad, yaw_rate_rad, &q_des);
            // CDC_Transmit_FS((uint8_t *)&q_des, sizeof(q_des));

            // adc level is between 2.1V and 1.875V
            // make a library for this??? battery monitoring library --> bml
            // include method for callbacks that will shut down system if battery too low
            remote_batt_lvl = compute_batt(samples[4]);

            // if throttle < halfway, throttle = 0
            // otherwise throttle = throttle - 2048
            // add a condition_joysticks() function to convert ranges
            // handle jerking, accleration, etc
            uint16_t shaped_throttle;
            condition_throttle(joysticks.throttle, &shaped_throttle);

            uint16_t data[2] = { joysticks.throttle, shaped_throttle };
            CDC_Transmit_FS((uint8_t *)&data, sizeof(data));

            // instead of doing joysticks --> angles
            // do joysticks--> angle rates, then integrate and convert those into quaternion

            uint8_t key = (mode == QUAD_STATUS_ARMED) ? ARMED_KEY : DISARMED_KEY;

            rf_packet_params_t pkt = {
                .throttle = shaped_throttle,
                .kp = kp,
                .ki = ki,
                .kd = kd,
                .key = key,
            };

            pkt.q_des.q1 = q_des.q1;
            pkt.q_des.q2 = q_des.q2;
            pkt.q_des.q3 = q_des.q3;
            pkt.q_des.q4 = q_des.q4;

            rf_ack_params_t ack;
            uint8_t ack_len;

            rf_send(&tx, (uint8_t *)&pkt, sizeof(rf_packet_params_t), (uint8_t *)&ack, &ack_len);

            if (ack_len == sizeof(rf_ack_params_t))
            {
                quad_batt_lvl = ack.rx_batt_lvl;
            }

            adc_dr = 0;
        }

        encoder_update(&enc);

        oled_params_t oled_info = {
            .joysticks = &joysticks,
            .page = page,
            .rx_batt = quad_batt_lvl,
            .tx_batt = remote_batt_lvl,
            .mode = mode,
            .kp = kp,
            .ki = ki,
            .kd = kd,
            .active_tune = curr_tune, // change to tune_parameter
        };

        oled_update(&oled, &oled_info);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
    adc_dr = 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef * hi2c)
{
    ssd1306_callback(hi2c);
}

void encoder_callback(encoder_handle_t * henc, encoder_event_t event)
{
    switch (event)
    {
    case ENCODER_EVENT_CW_TURN:
    case ENCODER_EVENT_CCW_TURN:
        page = encoder_get_position(henc);
        break;
    case ENCODER_EVENT_NONE:
    case ENCODER_EVENT_SW_RELEASE:
    case ENCODER_EVENT_SW_PRESS:
    default:
    }
}

void encoder_sw_callback(encoder_handle_t * henc, encoder_event_t event)
{
    switch (event)
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

static void condition_throttle(uint16_t throttle, uint16_t * shaped_throttle)
{
    // clean up this implementation
    uint16_t throttle_min = 400;
    uint16_t throttle_center = 1900;
    float deadband = 0.05;

    uint16_t deadzone_start = throttle_center - throttle_center * deadband;
    if (throttle < deadzone_start)
        *shaped_throttle = (uint16_t)map(throttle, throttle_min, deadzone_start, 2047, 48);
    else
        *shaped_throttle = 0;
}

static void euler_rates_to_quat(float w_x, float w_y, float w_z, quaternion_t * q_des)
{
    // place gyro angular rates (radians per second) into quaternion
    quaternion_t q_rates = {
        .q1 = 0,
        .q2 = w_x,
        .q3 = w_y,
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

static void
shape_input(uint16_t x, uint16_t x_min, uint16_t x_max, uint16_t x_mid, float deadzone, float x_shaped_min, float x_shaped_max, float * x_shaped)
{
    // transform input x in range [x_min, x_max] to [x_shaped_min, x_shaped_max] in a quadratic fashion
    // small deviations from center of range result in smaller outputs, larger deviations result in larger swings

    // map input to [-1, 1] range
    if (x > x_mid)
        *x_shaped = (float)(x - x_mid) / (float)(x_max - x_mid); // maps to [0, 1]
    else
        *x_shaped = (float)(x - x_mid) / (float)(x_mid - x_min); // maps to [-1, 0]

    // fix discontinuity at deadzone boundaries
    if (fabsf(*x_shaped) < deadzone)
        *x_shaped = 0;
    else
        *x_shaped = (fabsf(*x_shaped) - deadzone) / (1.0 - deadzone);

    // scale output quadratically with input
    if (x > x_mid)
        *x_shaped = powf(*x_shaped, 2.0) * x_shaped_max;
    else
        *x_shaped = powf(*x_shaped, 2.0) * x_shaped_min;
}

static void pos_to_euler_rates(joystick_t * joystick, float * pitch_rate, float * roll_rate, float * yaw_rate)
{

    // calibrate()

    const uint16_t pitch_min = 400;
    const uint16_t roll_min = 400;
    const uint16_t yaw_min = 400;
    const uint16_t pitch_max = 3600;
    const uint16_t roll_max = 3600;
    const uint16_t yaw_max = 3600;
    const uint16_t pitch_center = 1900;
    const uint16_t roll_center = 1900;
    const uint16_t yaw_center = 1900;

    const float deadzone = 0.05; // 5% deadzone

    shape_input(joystick->pitch, pitch_min, pitch_max, pitch_center, deadzone, MIN_PITCH_RATE_DEG, MAX_PITCH_RATE_DEG, pitch_rate);
    shape_input(joystick->roll, roll_min, roll_max, roll_center, deadzone, MIN_ROLL_RATE_DEG, MAX_ROLL_RATE_DEG, roll_rate);
    shape_input(joystick->yaw, yaw_min, yaw_max, yaw_center, deadzone, MIN_YAW_RATE_DEG, MAX_YAW_RATE_DEG, yaw_rate);
}

static float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    // prevent division by 0
    if (in_max - in_min == 0)
    {
        return 0;
    }

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t compute_batt(uint16_t batt_lvl_adc)
{
    float adc_v = ((float)batt_lvl_adc / 4096.0) * 3.3;

    float adc_min_v = BAT_MIN_V * 0.5;
    float adc_max_v = BAT_MAX_V * 0.5;

    return (uint8_t)map(adc_v, adc_min_v, adc_max_v, 0, 100.0);
}

static void tune(joystick_t * joysticks, float * kp, float * ki, float * kd, uint32_t * last_tune, oled_active_tune_param_t * active_tune)
{
    float ** new_k;

    uint32_t curr_tick = HAL_GetTick();

    // user-friendly update speed
    if (curr_tick - *last_tune > 250)
    {
        *last_tune = curr_tick;

        if (joysticks->roll > 3000) // go to next parameter for tuning
        {
            if (++(*active_tune) > ACTIVE_TUNE_KD)
                *active_tune = ACTIVE_TUNE_NONE;
        }
        else if (joysticks->roll < 1000) // go to previous parameter for tuning
        {
            if (--(*active_tune) < ACTIVE_TUNE_NONE)
                *active_tune = ACTIVE_TUNE_KD;
        }

        switch (*active_tune)
        {
        case ACTIVE_TUNE_KP:
            // new_k holds address of kp
            // *new_k holds value of kp, which is address of x
            // **new_k holds value of *kp, which is x
            new_k = &kp;
            break;
        case ACTIVE_TUNE_KI:
            new_k = &ki;
            break;
        case ACTIVE_TUNE_KD:
            new_k = &kd;
            break;
        default:
            return;
        }
        if (joysticks->pitch < 1500 && joysticks->pitch > 500)
            **new_k += 0.01;
        else if (joysticks->pitch <= 500)
            **new_k += 0.05;
        else if (joysticks->pitch > 2500 && joysticks->pitch < 3250)
            **new_k -= 0.01;
        else if (joysticks->pitch >= 3250)
            **new_k -= 0.05;

        if (**new_k > 99.99)
            **new_k = 99.985;
        else if (**new_k < 0.0)
            **new_k = 0.0;
    }
}