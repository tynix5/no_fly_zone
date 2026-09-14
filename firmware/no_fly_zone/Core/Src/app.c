#include "app.h"
#include "arm_math.h"
#include "main.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

#include "bldc.h"
#include "bms.h"
#include "dshot.h"
#include "lps25hb.h"
#include "lsm6ds3tr.h"
#include "madgwick.h"
#include "nrf24.h"
#include "quaternion.h"
#include "rf_structs.h"

#define BETA    0.2f
#define DELTA_T 1 / 1660.0 // replace with actual variable

#define RF_TX_ADDR 0xC2C2C2C2
#define RF_RX_ADDR 0xE7E7E7E7

#define RF_FAILSAFE_TIMEOUT_MS  100
#define IMU_FAILSAFE_TIMEOUT_MS 50
#define BAR_FAILSAFE_TIMEOUT_MS 1000

#define PKT_PER_ACK 10

#define ADC_RES_CNT 4096.0f

void delay_us(uint32_t delay);

nrf_handle_t rx = {

    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .delay_ms = HAL_Delay,
    .delay_us = delay_us,

    .this_addr = RF_RX_ADDR,
    .node_addr = RF_TX_ADDR,

    .power_level = NRF_TX_PWR_0DBM,
    .data_rate = NRF_DATARATE_2MBPS,
    .freq_ch = 2,
    .payload_type = NRF_PAYLOAD_DYNAMIC,
    .ack = FEAT_ENABLE,

    .nrf_irqs_t = {
        .rx_dr = FEAT_ENABLE,
        .tx_ds = FEAT_DISABLE,
        .max_rt = FEAT_DISABLE,
    },
};

lps25_handle_t bar = {

    .cs_gpio = BAR_CS_GPIO_Port,
    .cs = BAR_CS_Pin,
    .delay_ms = HAL_Delay,

    .odr = BAR_ODR_7HZ,
    .it = BAR_INT_DRDY,

    .fifo_handle_t = {
        .mode = BAR_FIFO_BYPASS,
        .thresh = 31,
        .mov_smp = BAR_MOV_AVG_SMP_NONE,
    },
};

lsm6_handle_t imu = {

    .cs_gpio = IMU_CS_GPIO_Port,
    .cs = IMU_CS_Pin,
    .delay_ms = HAL_Delay,

    .gyro_handle_t =
        {
            .odr = IMU_ODR_G_1_66KHZ,
            .fs = IMU_FS_G_1000DPS,
            .filt = IMU_HPF_EN,
            .cutoff = IMU_HP_G_16MILHZ,
        },

    .accel_handle_t =
        {
            .odr = IMU_ODR_XL_1_66KHZ,
            .fs = IMU_FS_XL_8G,
            .filt = IMU_LPF_EN,
        },

    .fifo_handle_t =
        {
            .mode = IMU_FIFO_BYPASS,
            .odr = IMU_ODR_FIFO_DISABLE,
        },

    // accelerometer and gyroscope should trigger at about the same time (due to same ODR)
    .int_handle_t =
        {
            .int1 = IMU_INT1_DRDY_G,
            .int2 = IMU_INT2_NONE,
        }
};

bms_handle_t bms = {
    .type = BMS_LIPO_TYPE,
    .cell_cnt = 4,
    .div = 6, // ? make sure
    .v_ref = 3.3,
};

bldc_handle_t bldc;
madgwick_state_t state;

volatile uint8_t rf_dr = 0, imu_dr = 0, bar_dr = 0;
uint32_t last_pkt_tick, last_imu_tick, last_bar_tick;

IWDG_HandleTypeDef * iwdg;
TIM_HandleTypeDef * tim_us;

// sense_adc[0]: VBAT_SENSE
// sense_adc[1]: CURR_SENSE
uint16_t sense_adc[2];

/***********************************************************************************************/
typedef enum : uint8_t
{
    FAILSAFE_TYPE_RF = 1,
    FAILSAFE_TYPE_IMU = 2,
    FAILSAFE_TYPE_BAR = 4,
    FAILSAFE_TYPE_REMOTE = 8
} failsafe_type_t;

failsafe_type_t failsafe_cause = 0;
/***********************************************************************************************/

arm_status_t mode_quad = MODE_STATUS_DISARMED;

static void construct_ack(rf_ack_params_t * ack);
void bms_callback(bms_handle_t * bms, bms_event_t event);

void app_init(ADC_HandleTypeDef * hadc,
              SPI_HandleTypeDef * hspi_sensor,
              SPI_HandleTypeDef * hspi_rf,
              TIM_HandleTypeDef * htim2_motor,
              TIM_HandleTypeDef * htim_adc,
              TIM_HandleTypeDef * htim5_motor,
              TIM_HandleTypeDef * htim_us,
              IWDG_HandleTypeDef * hiwdg)
{
    // deselect all slaves at start
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BAR_CS_GPIO_Port, BAR_CS_Pin, GPIO_PIN_SET);

    // all motors no pwm
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    tim_us = htim_us;
    rx.hspi = hspi_rf;
    bar.hspi = hspi_sensor;
    imu.hspi = hspi_sensor;
    iwdg = hiwdg;

    /********************************************* Configure nRF24l01 *****************************************/
    HAL_TIM_Base_Start(htim_us); // start microsecond timer for rf_init()
    if (rf_init(&rx) == RF_SUCCESS)
        HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
    /**********************************************************************************************************/

    /******************************************** Configure barometer *****************************************/
    if (bar_init(&bar) == STATUS_OK)
        HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);
    /**********************************************************************************************************/

    /*********************************************** Configure IMU ********************************************/
    if (imu_init(&imu) == STATUS_OK)
        HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_SET);
    /**********************************************************************************************************/

    HAL_Delay(1000);

    HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_RESET);

    /******************************************** Configure Watchdog ******************************************/
    hiwdg->Instance = IWDG;
    hiwdg->Init.Prescaler = IWDG_PRESCALER_4;
    hiwdg->Init.Reload = 3999;
    HAL_IWDG_Init(hiwdg); // T = 0.5s
    /**********************************************************************************************************/

    /***************************************** Configure DShot channels ***************************************/
    bldc_init(&bldc, htim2_motor, htim5_motor);
    bldc_enable(&bldc);
    /**********************************************************************************************************/

    /************************************** Configure battery management **************************************/
    bms_init(&bms, bms_callback);
    /**********************************************************************************************************/

    /**************************************** Initiate Madgwick Filter ****************************************/
    madgwick_init(&state);
    /**********************************************************************************************************/

    /************************************** Configure and Start ADC + DMA *************************************/
    // make sure this is working
    HAL_ADC_Start_DMA(hadc, (uint32_t *)sense_adc, 2); // enable ADC conversion for VBAT_SENSE and CURR_SENSE
    HAL_TIM_Base_Start(htim_adc);                      // start triggering ADC (T = 0.2s)
    /**********************************************************************************************************/

    rf_listen_it(&rx);

    last_pkt_tick = last_imu_tick = last_bar_tick = HAL_GetTick();
}

void app(void)
{
    uint32_t pkt_cnt = 0;

    rf_packet_params_t pkt;
    rf_ack_params_t ack;

    const float k_att = 10.0;

    uint8_t has_been_disarmed = 0;

    while (1)
    {
        HAL_IWDG_Refresh(iwdg);

        if (rf_dr)
        {
            pkt_cnt++;
            last_pkt_tick = HAL_GetTick();
            construct_ack(&ack);

            rf_packet_params_t temp;
            uint8_t temp_size;

            // send acknowlegment every 10 packets
            if (pkt_cnt % PKT_PER_ACK == 0)
                rf_receive(&rx, (uint8_t *)&temp, &temp_size, (uint8_t *)&ack, sizeof(rf_ack_params_t));
            else
                rf_receive(&rx, (uint8_t *)&temp, &temp_size, (uint8_t *)&ack, 0);

            // discard packet if size doesnt match or keys dont match
            // copy to a valid packet if available
            if (temp_size == sizeof(rf_packet_params_t))
            {
                HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);

                switch (temp.key)
                {
                case DISARMED_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    has_been_disarmed = 1;
                    // do not go into disarm mode if something has failed
                    if (mode_quad != MODE_STATUS_FAILSAFE)
                        mode_quad = MODE_STATUS_DISARMED;
                    break;
                case ARMED_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    // prevent quadcopter from arming if it has never seen a disarmed
                    // packet
                    if (has_been_disarmed && mode_quad != MODE_STATUS_FAILSAFE)
                        mode_quad = MODE_STATUS_ARMED;
                    break;
                case FAILSAFE_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    mode_quad = MODE_STATUS_FAILSAFE;
                    // remote type doesn't really make sense, wouldn't receive anything
                    // from remote if it failed
                    failsafe_cause |= FAILSAFE_TYPE_REMOTE;
                    break;
                default:
                    // invalid packet, ignore
                }
            }

            // HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);

            rf_dr = 0;
            rf_listen_it(&rx);
        }

        // if quadcopter sensors or RF communication is disrupted, go into failsafe
        // mode RF can only go into failsafe if previous communication has been
        // established
        uint32_t tick = HAL_GetTick();

        if (tick - last_pkt_tick > RF_FAILSAFE_TIMEOUT_MS && pkt_cnt != 0)
        {
            mode_quad = MODE_STATUS_FAILSAFE;
            failsafe_cause |= FAILSAFE_TYPE_RF;
        }
        if (tick - last_imu_tick > IMU_FAILSAFE_TIMEOUT_MS)
        {
            mode_quad = MODE_STATUS_FAILSAFE;
            failsafe_cause |= FAILSAFE_TYPE_IMU;
        }
        // if (tick - last_bar_tick > BAR_FAILSAFE_TIMEOUT_MS)
        // {
        //     mode_quad = MODE_STATUS_FAILSAFE;
        //     failsafe_cause |= FAILSAFE_TYPE_BAR;
        // }

        if (imu_dr)
        {
            float a_x, a_y, a_z, w_x, w_y, w_z;

            // use more accurate timer for delta_t estimation
            // update orientation estimation
            imu_read_gyro_radps(&imu, &w_x, &w_y, &w_z);
            imu_read_accel_mps2(&imu, &a_x, &a_y, &a_z);
            madgwick_update(a_x, a_y, a_z, w_x, w_y, w_z, BETA, DELTA_T, &state);

            HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);

            // stream orientation over USB (for debugging)
            // float data[] = { state.q_state.q1, state.q_state.q2, state.q_state.q3,
            // state.q_state.q4 }; CDC_Transmit_FS((uint8_t *)data, sizeof(data));
            last_imu_tick = HAL_GetTick();
            imu_dr = 0;
        }

        // calculate orientation (only if IMU working)
        mode_quad = MODE_STATUS_ARMED; /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        switch (mode_quad)
        {
        case MODE_STATUS_DISARMED:
            // stop motors
            bldc_stop(&bldc);

            // uint16_t data[] = { 0, 0, 0, 0 };
            // CDC_Transmit_FS((uint8_t *)data, sizeof(data));

            HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
            break;

        case MODE_STATUS_ARMED:

            // calculate quaternion error and convert to a rate error
            float w_x_err, w_y_err, w_z_err;
            quaternion_t q_des_sample = { .q1 = 1, .q2 = 0, .q3 = 0, .q4 = 0 };
            madgwick_curr_to_des(&state, q_des_sample, k_att, &w_x_err, &w_y_err, &w_z_err);
            // madgwick_curr_to_des(&state, pkt.q_des, k_att, &w_x_err, &w_y_err, &w_z_err);

            // run PID on ew --> place these into a matrix for cleaner math?
            // convert rate error to torque/speed commands
            float test_kp = 50.0;
            float tau_x = test_kp * w_x_err;
            float tau_y = test_kp * w_y_err;
            float tau_z = test_kp * w_z_err;

            // mix torque/speed commands
            float throttle_test = 1000;
            bldc_mix(&bldc, throttle_test, tau_x, tau_y, tau_z);
            // bldc_mix(&bldc, pkt.throttle, tau_x, tau_y, tau_z);

            bldc_send(&bldc);

            uint16_t data[] = { bldc.throttles.speed_fl, bldc.throttles.speed_fr, bldc.throttles.speed_bl, bldc.throttles.speed_br };
            CDC_Transmit_FS((uint8_t *)data, sizeof(data));

            HAL_GPIO_TogglePin(STAT3_GPIO_Port, STAT3_Pin);
            break;

        case MODE_STATUS_FAILSAFE:

            // immediately stop and disable motors
            bldc_stop(&bldc);
            bldc_disable(&bldc);

            while (1)
            {
                // disable IMU, barometer, RF?
                // for initial versions, do not even attempt to recover from failure
                // must reset watchdog though
                HAL_IWDG_Refresh(iwdg);
                HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_RESET);
                HAL_Delay(100);
                HAL_IWDG_Refresh(iwdg);
                HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_SET);
                HAL_Delay(100);
            }

            break;

        default:
        }
    }
}

/************************************************* Interrupt Callbacks
 * *****************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // RF IRQ on falling edge
    if (GPIO_Pin == RF_IRQ_Pin)
    {
        rf_dr = 1;
    }
    // IMU IRQ on rising edge
    if (GPIO_Pin == IMU_IRQ1_Pin)
    {
        imu_dr = 1;
    }
    // BAR IRQ on rising edge
    if (GPIO_Pin == BAR_IRQ_Pin)
    {
        bar_dr = 1;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim)
{
    // found in dshot.h
    dshot_complete_callback(&bldc.motor_fl, htim);
    dshot_complete_callback(&bldc.motor_fr, htim);
    dshot_complete_callback(&bldc.motor_bl, htim);
    dshot_complete_callback(&bldc.motor_br, htim);
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef * htim)
{
    // found in dshot.h
    dshot_half_complete_callback(&bldc.motor_fl, htim);
    dshot_half_complete_callback(&bldc.motor_fr, htim);
    dshot_half_complete_callback(&bldc.motor_bl, htim);
    dshot_half_complete_callback(&bldc.motor_br, htim);
}
/***************************************************************************************************************************/

static void construct_ack(rf_ack_params_t * ack)
{
    float percent;
    if (bms_update(&bms, (float)sense_adc[0] / ADC_RES_CNT, &percent) != STATUS_OK)
        return;

    ack->quad_batt_lvl = (uint8_t)percent;

    switch (mode_quad)
    {
    case MODE_STATUS_DISARMED:
        ack->key = DISARMED_KEY;
        break;
    case MODE_STATUS_ARMED:
        ack->key = ARMED_KEY;
        break;
    case MODE_STATUS_FAILSAFE:
    default:
        ack->key = FAILSAFE_KEY;
    }
}

void bms_callback(bms_handle_t * bms, bms_event_t event)
{
    // if battery dies, immediately go into failsafe mode to prevent full
    // discharge
    if (event == BMS_EVENT_CHARGE_EMPTY)
    {
        mode_quad = MODE_STATUS_FAILSAFE;
    }
}

void delay_us(uint32_t delay)
{
    __HAL_TIM_SET_COUNTER(tim_us, 0);
    while (__HAL_TIM_GET_COUNTER(tim_us) < delay)
        ;
}