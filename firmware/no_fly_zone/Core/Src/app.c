#include "app.h"
#include "arm_math.h"
#include "main.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdbool.h>
#include <sys/types.h>

#include "bldc.h"
#include "bms.h"
#include "lps25hb.h"
#include "lsm6ds3tr.h"
#include "madgwick.h"
#include "nrf24.h"
#include "quaternion.h"
#include "rf_structs.h"

#define BETA    0.25f
#define DELTA_T 1 / 1660.0 // replace with actual variable

#define RF_TX_ADDR 0xC2C2C2C2
#define RF_RX_ADDR 0xE7E7E7E7

#define RF_FAILSAFE_TIMEOUT_MS  100
#define IMU_FAILSAFE_TIMEOUT_MS 50
#define BAR_FAILSAFE_TIMEOUT_MS 1000

#define PKT_PER_ACK 10

#define ADC_RES_CNT       4096.0f
#define N_VBAT_SENSE_BUFF 10

#define N_IMU_BIAS_SAMPLES 3000

uint32_t micros_get_tick();
uint32_t millis_get_tick();
void delay_us(uint32_t delay);
void delay_ms(uint32_t delay);

nrf24_handle_t rx = {

    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .delay_ms = delay_ms,
    .delay_us = delay_us,

    .power_level = NRF24_TX_PWR_0DBM,
    .data_rate = NRF24_DATARATE_2MBPS,
    .freq_ch = 110,           // 2400 MHz + 110 = 2.510 GHz

    .nrf24_irqs_t = {
        .rx_dr_en = true,
        .tx_ds_en = false,
        .max_rt_en = false,
    },
};

nrf24_pipe_t rx_pipe = {

    .address = RF_RX_ADDR,
    .ack = NRF24_AUTO_ACK_PAYLOAD,
    .payload_type = NRF24_PAYLOAD_DYNAMIC,
    .num = NRF24_PIPE0,
};

lps25hb_handle_t bar = {

    .cs_gpio = BAR_CS_GPIO_Port,
    .cs = BAR_CS_Pin,
    .delay_ms = delay_ms,

    .odr = LPS25HB_ODR_7HZ,
    .it = LPS25HB_INT_DRDY,

    .fifo_handle_t = {
        .mode = LPS25HB_FIFO_BYPASS,
        .thresh = 31,
        .mov_smp = LPS25HB_MOV_AVG_SMP_NONE,
    },
};

lsm6ds3tr_handle_t imu = {

    .cs_gpio = IMU_CS_GPIO_Port,
    .cs = IMU_CS_Pin,
    .delay_ms = delay_ms,

    // DO NOT use high pass filter on gyroscope. It suppresses sustained rotations and causes 
    // internal orienation model to drift back towards origin
    .gyro_handle_t = {
        .odr = LSM6DS3TR_ODR_G_1_66KHZ,
        .fs = LSM6DS3TR_FS_G_1000DPS,
        .filt = LSM6DS3TR_LPF_EN,   // use LPF for high-frequency vibrations from motors
    },

    .accel_handle_t = {
        .odr = LSM6DS3TR_ODR_XL_1_66KHZ,
        .fs = LSM6DS3TR_FS_XL_8G,
        .filt = LSM6DS3TR_LPF_EN,
    },

    .fifo_handle_t = {
        .mode = LSM6DS3TR_FIFO_BYPASS,
        .odr = LSM6DS3TR_ODR_FIFO_DISABLE,
    },

    // accelerometer and gyroscope should trigger at about the same time (due to same ODR)
    .int_handle_t = {
        .int1 = LSM6DS3TR_INT1_DRDY_G,
        .int2 = LSM6DS3TR_INT2_NONE,
    }
};

bms_handle_t bms = {
    .type = BMS_LIPO_TYPE,
    .cell_cnt = 4,
    .div = 6,
    .v_ref = 3.3,
    .v_offset = 0,
};

bldc_handle_t bldc;
madgwick_state_t state;

volatile bool rf_dr = false, imu_dr = false, bar_dr = false, adc_dr = false;
volatile uint32_t last_pkt_tick_ms, last_bar_tick_ms, last_imu_tick_us;
uint32_t this_imu_tick_us;

ADC_HandleTypeDef * hadc_sense;
IWDG_HandleTypeDef * iwdg;
TIM_HandleTypeDef *tim_us_l, *tim_us_h;

// sense_adc[0]: VBAT_SENSE
// sense_adc[1]: CURR_SENSE
uint16_t sense_adc[2];
uint32_t adc_samples_cnt = 0;
uint32_t vbat_sense_running_sum = 0;
uint16_t vbat_sense_buff[N_VBAT_SENSE_BUFF] = { 0 };
uint16_t last_vbat_sense = 0;
uint8_t vbat_sense_buff_ind = 0;
uint8_t quad_batt_lvl;

/***********************************************************************************************/
typedef enum : uint8_t
{
    FAILSAFE_TYPE_RF = 1,
    FAILSAFE_TYPE_IMU = 2,
    FAILSAFE_TYPE_BAR = 4,
    FAILSAFE_TYPE_REMOTE = 8,
    FAILSAFE_TYPE_BMS = 16,
} failsafe_type_t;

failsafe_type_t failsafe_cause = 0;
/***********************************************************************************************/

arm_state_t arm_state_quad = ARM_STATE_DISARMED;

static void construct_ack(rf_ack_params_t * ack);
void bms_callback(bms_handle_t * bms, bms_event_t event);

void app_init(ADC_HandleTypeDef * hadc,
              SPI_HandleTypeDef * hspi_sensor,
              SPI_HandleTypeDef * hspi_rf,
              TIM_HandleTypeDef * htim2_motor,
              TIM_HandleTypeDef * htim_adc,
              TIM_HandleTypeDef * htim5_motor,
              TIM_HandleTypeDef * htim_us_l,
              TIM_HandleTypeDef * htim_us_h,
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

    hadc_sense = hadc;
    bar.hspi = hspi_sensor;
    imu.hspi = hspi_sensor;
    rx.hspi = hspi_rf;
    tim_us_l = htim_us_l;
    tim_us_h = htim_us_h;
    iwdg = hiwdg;

    /********************************************* Configure nRF24l01 *****************************************/
    HAL_TIM_Base_Start(htim_us_l); // start microsecond timer for rf_init()
    HAL_TIM_Base_Start(htim_us_h); // start microsecond timer for rf_init()

    if (nrf24_init(&rx) == NRF24_OK && nrf24_open_rx_pipe(&rx, &rx_pipe) == NRF24_OK)
        HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
    /**********************************************************************************************************/

    /******************************************** Configure barometer *****************************************/
    if (lps25hb_init(&bar) == LPS25HB_OK)
        HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);
    /**********************************************************************************************************/

    /*********************************************** Configure IMU ********************************************/
    if (lsm6ds3tr_init(&imu) == LSM6DS3TR_OK)
        HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_SET);
    /**********************************************************************************************************/

    delay_ms(1000);

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

    nrf24_listen_it(&rx);

    last_pkt_tick_ms = last_bar_tick_ms = millis_get_tick();
    last_imu_tick_us = this_imu_tick_us = micros_get_tick();
}

void app(void)
{
    uint32_t pkt_cnt = 0;

    rf_packet_params_t pkt;
    rf_ack_params_t ack;

    const float k_att = 5.0;

    bool has_been_disarmed = 0;

    float last_w_x_err = 0, last_w_y_err = 0, last_w_z_err = 0;
    float w_x_err_sum = 0, w_y_err_sum = 0, w_z_err_sum = 0;
    float prev_dt = 0;

    uint32_t imu_samples_cnt = 0;
    float w_x_bias = 0.0f, w_y_bias = 0.0f, w_z_bias = 0.0f;
    float w_x_bias_accum = 0.0f, w_y_bias_accum = 0.0f, w_z_bias_accum = 0.0f;

    while (1)
    {
        HAL_IWDG_Refresh(iwdg);

        if (adc_dr)
        {
            adc_dr = false;
            adc_samples_cnt++;

            last_vbat_sense = vbat_sense_buff[vbat_sense_buff_ind]; // save last sample as it will be discarded
            vbat_sense_buff[vbat_sense_buff_ind++] = sense_adc[0];  // store previous VBAT ADC sample in buffer
            vbat_sense_running_sum += sense_adc[0];                 // add new sample to running sum

            // wrap index
            if (vbat_sense_buff_ind >= N_VBAT_SENSE_BUFF)
                vbat_sense_buff_ind = 0;

            // for first 10 samples, do not update BMS to get a baseline reading
            if (adc_samples_cnt > N_VBAT_SENSE_BUFF)
            {
                vbat_sense_running_sum -= last_vbat_sense; // remove discarded sample from running total

                // calculate average battery percentage
                float vbat_percent;
                float norm_adc = (float)vbat_sense_running_sum / (float)N_VBAT_SENSE_BUFF / ADC_RES_CNT;
                bms_update(&bms, norm_adc, &vbat_percent);
                quad_batt_lvl = (uint8_t)vbat_percent;
            }
        }

        if (rf_dr)
        {
            rf_dr = false;

            construct_ack(&ack);

            rf_packet_params_t temp;
            uint8_t temp_size;

            nrf24_pipe_num_t pipe_num;

            // send acknowlegment every 10 packets
            if (pkt_cnt % PKT_PER_ACK == 0)
                nrf24_receive(&rx, (uint8_t *)&ack, sizeof(rf_ack_params_t), (uint8_t *)&temp, &temp_size, &pipe_num);
            else
                nrf24_receive(&rx, (uint8_t *)&ack, 0, (uint8_t *)&temp, &temp_size, &pipe_num);

            // discard packet if size doesnt match or keys dont match
            // copy to a valid packet if available
            if (temp_size == sizeof(rf_packet_params_t))
            {
                pkt_cnt++;

                switch (temp.key)
                {
                case DISARMED_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    has_been_disarmed = true;
                    // do not go into disarm mode if something has failed
                    if (arm_state_quad != ARM_STATE_FAILSAFE)
                        arm_state_quad = ARM_STATE_DISARMED;
                    break;
                case ARMED_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    // prevent quadcopter from arming if it has never seen a disarmed
                    // packet
                    if (has_been_disarmed && arm_state_quad != ARM_STATE_FAILSAFE)
                        arm_state_quad = ARM_STATE_ARMED;
                    break;
                case FAILSAFE_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    arm_state_quad = ARM_STATE_FAILSAFE;
                    // remote type doesn't really make sense, wouldn't receive anything
                    // from remote if it failed
                    failsafe_cause |= FAILSAFE_TYPE_REMOTE;
                    break;
                default:
                    // invalid packet, ignore
                }
            }
            nrf24_listen_it(&rx);
        }

        if (imu_dr)
        {
            imu_dr = false;
            imu_samples_cnt++;

            float a_x, a_y, a_z, w_x, w_y, w_z;

            // update orientation estimation
            lsm6ds3tr_read_gyro_radps(&imu, &w_x, &w_y, &w_z);
            lsm6ds3tr_read_accel_mps2(&imu, &a_x, &a_y, &a_z);

            // discard first N samples to allow time for IMU to settle
            // accumulate average sensor reading for second N samples
            // quadcopter should be powered on while stationary on a level surface to calibrate
            if (imu_samples_cnt > N_IMU_BIAS_SAMPLES && imu_samples_cnt < 2 * N_IMU_BIAS_SAMPLES)
            {
                w_x_bias_accum += w_x;
                w_y_bias_accum += w_y;
                w_z_bias_accum += w_z;
            }
            else if (imu_samples_cnt == 2 * N_IMU_BIAS_SAMPLES) // calculate bias (average)
            {
                w_x_bias = w_x_bias_accum / (float)N_IMU_BIAS_SAMPLES;
                w_y_bias = w_y_bias_accum / (float)N_IMU_BIAS_SAMPLES;
                w_z_bias = w_z_bias_accum / (float)N_IMU_BIAS_SAMPLES;
            }

            w_x -= w_x_bias;
            w_y -= w_y_bias;
            w_z -= w_z_bias;

            prev_dt = (float)(this_imu_tick_us - last_imu_tick_us) / 1000000.0f; // convert sample delay from us to s
            madgwick_update(a_x, a_y, a_z, w_x, w_y, w_z, BETA, prev_dt, &state);

            // stream orientation over USB (for debugging)
            // float data[] = { state._q_state.q1, state._q_state.q2, state._q_state.q3, state._q_state.q4 };
            // CDC_Transmit_FS((uint8_t *)data, sizeof(data));

            last_imu_tick_us = this_imu_tick_us;
        }

        // barometer data currently not being used
        if (bar_dr)
        {
            bar_dr = false;

            float hpa;
            lps25hb_read_press_hpa(&bar, &hpa);
        }

        // if quadcopter sensors or RF communication is disrupted, go into failsafe
        // mode RF can only go into failsafe if previous communication has been established

        // save atomic copies before doing timeout checks
        // this way, last_*_tick cannot update after tick_ms/tick_us has been declared,
        // which would result in an unsigned overflow, and therefore, timeout
        uint32_t last_pkt_tick_ms_local = last_pkt_tick_ms;
        uint32_t last_imu_tick_us_local = last_imu_tick_us;
        uint32_t last_bar_tick_ms_local = last_bar_tick_ms;

        uint32_t tick_ms = millis_get_tick();
        uint32_t tick_us = micros_get_tick();

        // make sure tick_ms is > than last_tick_ms in case interrupt occurs between read and check
        // may not need the tick > last check
        if (tick_ms - last_pkt_tick_ms_local > RF_FAILSAFE_TIMEOUT_MS && pkt_cnt != 0)
        {
            arm_state_quad = ARM_STATE_FAILSAFE;
            failsafe_cause |= FAILSAFE_TYPE_RF;
        }
        if (((tick_us - last_imu_tick_us_local) / 1000) > IMU_FAILSAFE_TIMEOUT_MS)
        {
            arm_state_quad = ARM_STATE_FAILSAFE;
            failsafe_cause |= FAILSAFE_TYPE_IMU;
        }
        if (tick_ms - last_bar_tick_ms_local > BAR_FAILSAFE_TIMEOUT_MS)
        {
            arm_state_quad = ARM_STATE_FAILSAFE;
            failsafe_cause |= FAILSAFE_TYPE_BAR;
        }

        switch (arm_state_quad)
        {
        case ARM_STATE_DISARMED:
            // stop motors
            bldc_stop(&bldc);

            HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_RESET);
            break;

        case ARM_STATE_ARMED:

            // calculate quaternion error and convert to a rate error
            float w_x_err, w_y_err, w_z_err;
            madgwick_curr_to_des(&state, pkt.q_des, k_att, &w_x_err, &w_y_err, &w_z_err);

            // run PID on ew --> place these into a matrix for cleaner math?
            // convert rate error to torque/speed commands
            float tau_x = pkt.kp * w_x_err;
            float tau_y = pkt.kp * w_y_err;
            float tau_z = pkt.kp * w_z_err;

            // do not accumulate error when the drone is not moving
            if (pkt.throttle > 200)
            {
                w_x_err_sum += w_x_err * prev_dt;
                w_y_err_sum += w_y_err * prev_dt;
                w_z_err_sum += w_z_err * prev_dt;
            }

            // clamp integrals
            const float w_max_err_sum = 500.0f;

            if (w_x_err_sum > w_max_err_sum)
                w_x_err_sum = w_max_err_sum;
            else if (w_x_err_sum < -w_max_err_sum)
                w_x_err_sum = -w_max_err_sum;

            if (w_y_err_sum > w_max_err_sum)
                w_y_err_sum = w_max_err_sum;
            else if (w_y_err_sum < -w_max_err_sum)
                w_y_err_sum = -w_max_err_sum;

            if (w_z_err_sum > w_max_err_sum)
                w_z_err_sum = w_max_err_sum;
            else if (w_z_err_sum < -w_max_err_sum)
                w_z_err_sum = -w_max_err_sum;

            // do not divide by zero when performing PID
            if (prev_dt != 0.0f)
            {
                tau_x += pkt.ki * w_x_err_sum + pkt.kd * (w_x_err - last_w_x_err) / prev_dt;
                tau_y += pkt.ki * w_y_err_sum + pkt.kd * (w_y_err - last_w_y_err) / prev_dt;
                tau_z += pkt.ki * w_z_err_sum + pkt.kd * (w_z_err - last_w_z_err) / prev_dt;
            }

            last_w_x_err = w_x_err;
            last_w_y_err = w_y_err;
            last_w_z_err = w_z_err;

            // mix torque/speed commands
            bldc_mix(&bldc, pkt.throttle, tau_x, tau_y, tau_z);
            bldc_send(&bldc);

            // float data[] = {
            //     (float)bldc.throttles.speed_fl, (float)bldc.throttles.speed_bl, (float)bldc.throttles.speed_fr, (float)bldc.throttles.speed_br, w_x_err, w_y_err, w_z_err
            // };
            // CDC_Transmit_FS((uint8_t *)data, sizeof(data));
            float data[] = { pkt.q_des.q1,
                             pkt.q_des.q2,
                             pkt.q_des.q3,
                             pkt.q_des.q4,
                             state._q_state.q1,
                             state._q_state.q2,
                             state._q_state.q3,
                             state._q_state.q4,
                             (float)bldc.throttles.speed_fl,
                             (float)bldc.throttles.speed_bl,
                             (float)bldc.throttles.speed_fr,
                             (float)bldc.throttles.speed_br };
            CDC_Transmit_FS((uint8_t *)data, sizeof(data));

            HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_SET);

            break;

        case ARM_STATE_FAILSAFE:
        default:

            // immediately stop and disable motors
            bldc_stop(&bldc);
            bldc_disable(&bldc);

            HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_RESET);

            while (1)
            {
                // disable IMU, barometer, RF?
                // for initial versions, do not even attempt to recover from failure
                // must reset watchdog though

                if (failsafe_cause & FAILSAFE_TYPE_IMU)
                    HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);

                if (failsafe_cause & FAILSAFE_TYPE_BAR)
                    HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);

                if (failsafe_cause & FAILSAFE_TYPE_RF)
                    HAL_GPIO_TogglePin(STAT3_GPIO_Port, STAT3_Pin);

                if (failsafe_cause & FAILSAFE_TYPE_BMS)
                {
                    HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);
                    HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);
                    HAL_GPIO_TogglePin(STAT3_GPIO_Port, STAT3_Pin);
                }

                HAL_IWDG_Refresh(iwdg);

                delay_ms(100);
            }

            break;
        }
    }
}

/************************************************* Interrupt Callbacks *****************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // RF IRQ on falling edge
    if (GPIO_Pin == RF_IRQ_Pin)
    {
        rf_dr = true;
        last_pkt_tick_ms = millis_get_tick();
    }
    // IMU IRQ on rising edge
    if (GPIO_Pin == IMU_IRQ1_Pin)
    {
        imu_dr = true;
        this_imu_tick_us = micros_get_tick();
    }
    // BAR IRQ on rising edge
    if (GPIO_Pin == BAR_IRQ_Pin)
    {
        bar_dr = true;
        last_bar_tick_ms = millis_get_tick();
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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc)
{
    if (hadc == hadc_sense)
    {
        adc_dr = true;
    }
}
/***************************************************************************************************************************/

static void construct_ack(rf_ack_params_t * ack)
{
    ack->quad_batt_lvl = quad_batt_lvl;

    switch (arm_state_quad)
    {
    case ARM_STATE_DISARMED:
        ack->key = DISARMED_KEY;
        break;
    case ARM_STATE_ARMED:
        ack->key = ARMED_KEY;
        break;
    case ARM_STATE_FAILSAFE:
    default:
        ack->key = FAILSAFE_KEY;
    }
}

void bms_callback(bms_handle_t * bms, bms_event_t event)
{
    // if battery dies, immediately go into failsafe mode to prevent full discharge
    if (event == BMS_EVENT_CHARGE_EMPTY)
    {
        arm_state_quad = ARM_STATE_FAILSAFE;
        failsafe_cause |= FAILSAFE_TYPE_BMS;
    }
}

// chain 2 16-bit counters for a 32-bit counter
uint32_t micros_get_tick()
{
    uint16_t tim_low, tim_high;

    // check race conditions when doing non-atomic read over 2 registers
    do
    {
        tim_high = __HAL_TIM_GET_COUNTER(tim_us_h);
        tim_low = __HAL_TIM_GET_COUNTER(tim_us_l);

    } while (__HAL_TIM_GET_COUNTER(tim_us_h) != tim_high);

    return ((uint32_t)tim_high << 16) | (uint32_t)tim_low;
}

uint32_t millis_get_tick()
{
    return micros_get_tick() / 1000;
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