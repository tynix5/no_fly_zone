#include "app.h"
#include "main.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include <stdio.h>

#include "lsm6ds3tr.h"
#include "lps25hb.h"
#include "dshot.h"
#include "nrf24.h"
#include "rf_structs.h"
#include "quaternion.h"
#include "madgwick.h"

#define BETA                0.2f
#define MADGWICK_GAIN       BETA
#define DELTA_T             1 / 1660.0 // replace with actual variable
#define DSHOT_TIM_FREQ      84000000

#define RF_TX_ADDR          0xC2C2C2C2
#define RF_RX_ADDR          0xE7E7E7E7

#define FAILSAFE_TIMEOUT_MS 100

nrf_handle_t rx = {

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
        .rx_dr = FEAT_ENABLE, 
        .tx_ds = FEAT_DISABLE, 
        .max_rt = FEAT_DISABLE, }
};

lps25_handle_t bar = {

    .cs_gpio = BAR_CS_GPIO_Port,
    .cs = BAR_CS_Pin,
    .delay_ms = HAL_Delay,

    .odr = BAR_ODR_1HZ,
    .it = BAR_INT_DRDY,

    .fifo_handle_t = {  
        .mode = BAR_FIFO_BYPASS, 
        .thresh = 31, 
        .mov_smp = BAR_MOV_AVG_SMP_NONE, }
};

lsm6_handle_t imu = {

    .cs_gpio = IMU_CS_GPIO_Port,
    .cs = IMU_CS_Pin,
    .delay_ms = HAL_Delay,

    .gyro_handle_t = {  
        .odr = IMU_ODR_G_1_66KHZ, 
        .fs = IMU_FS_G_1000DPS, 
        .filt = IMU_HPF_EN, 
        .cutoff = IMU_HP_G_16MILHZ, },

    .accel_handle_t = { 
        .odr = IMU_ODR_XL_1_66KHZ, 
        .fs = IMU_FS_XL_8G, 
        .filt = IMU_LPF_EN, },

    .fifo_handle_t = {  
        .mode = IMU_FIFO_BYPASS, 
        .odr = IMU_ODR_FIFO_DISABLE, },

    // accelerometer and gyroscope should trigger at about the same time (due to same ODR)
    .int_handle_t = {   
        .int1 = IMU_INT1_DRDY_G, 
        .int2 = IMU_INT2_NONE, }

};

dshot_handle_t motor_fl, motor_fr, motor_bl, motor_br;

uint8_t rf_dr = 0, imu_dr = 0, bar_dr = 0;
uint32_t last_pkt_tick = 0, last_imu_tick = 0, last_bar_tick = 0;

static void clamp(uint16_t * x, uint16_t min, uint16_t max);

void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, TIM_HandleTypeDef * htim5)
{
    // deselect all slaves at start
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BAR_CS_GPIO_Port, BAR_CS_Pin, GPIO_PIN_SET);

    // all motors no pwm
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    // try to recover if one of these is not successful
    rx.hspi = hspi3;
    if (rf_init(&rx) == RF_SUCCESS)
        HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);

    bar.hspi = hspi2;
    if (bar_init(&bar) == STATUS_OK)
        HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);

    imu.hspi = hspi2;
    if (imu_init(&imu) == STATUS_OK)
        HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_SET);

    HAL_Delay(1000);

    HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT3_GPIO_Port, STAT3_Pin, GPIO_PIN_RESET);

    dshot_init(&motor_fl, htim5, TIM_CHANNEL_2, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    dshot_init(&motor_bl, htim2, TIM_CHANNEL_1, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    dshot_init(&motor_br, htim5, TIM_CHANNEL_3, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
    dshot_init(&motor_fr, htim2, TIM_CHANNEL_4, DSHOT_TIM_FREQ, DSHOT_BR_300_KBPS, DSHOT_FREQ_4_KHZ);
}

void app(void)
{
    uint32_t pkt_cnt = 0;

    madgwick_state_t state = {
        .beta = BETA,
        .dt = DELTA_T,
    };

    rf_packet_params_t pkt;
    rf_ack_params_t ack;

    const float k_att = 10.0;

    quad_arm_status_t mode = QUAD_STATUS_DISARMED;

    madgwick_init(&state);
    rf_listen_it(&rx);

    dshot_queue(&motor_fl, 0, 0);
    dshot_queue(&motor_bl, 0, 0);
    dshot_queue(&motor_fr, 0, 0);
    dshot_queue(&motor_br, 0, 0);

    dshot_start(&motor_fl);
    dshot_start(&motor_fr);
    dshot_start(&motor_bl);
    dshot_start(&motor_br);


    while (1)
    {
        /* Create timer to sample VBAT ADC every second or so */

        /*
        if (rf_dr)
        {
            pkt_cnt++;

            // build ack packet in here
            switch (mode)
            {
            case QUAD_STATUS_DISARMED:
                ack.key = DISARMED_KEY;
                break;
            case QUAD_STATUS_ARMED:
                ack.key = ARMED_KEY;
                break;
            case QUAD_STATUS_FAILSAFE:
            default:
                ack.key = FAILSAFE_KEY;
            }

            // add bml library
            ack.rx_batt_lvl = 32;

            rf_packet_params_t temp;
            uint8_t temp_size;

            // send acknowlegment every 10 packets
            if (pkt_cnt % 10 == 0)
                rf_receive(&rx, (uint8_t *)&temp, &temp_size, (uint8_t *)&ack, sizeof(rf_ack_params_t));
            else
                rf_receive(&rx, (uint8_t *)&temp, &temp_size, (uint8_t *)&ack, 0);

            // discard packet if size doesnt match or keys dont match
            // copy to a valid packet if available
            if (temp_size == sizeof(rf_packet_params_t))
            {
                switch (temp.key)
                {
                case DISARMED_KEY:
                case ARMED_KEY:
                case FAILSAFE_KEY:
                    memcpy((uint8_t *)&pkt, (uint8_t *)&temp, sizeof(rf_packet_params_t));
                    // how to fix this in event of failsafe
                    // do not arm if first packet received is an arm packet
                    mode = (pkt.key == ARMED_KEY) ? QUAD_STATUS_ARMED : QUAD_STATUS_DISARMED;
                    break;
                default:
                    // invalid packet
                }
            }

            HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);

            rf_dr = 0;
            rf_listen_it(&rx);
        }

        // if quadcopter sensors or RF communication is disrupted, go into failsafe mode
        uint32_t tick = HAL_GetTick();
        if (tick - last_pkt_tick > FAILSAFE_TIMEOUT_MS || tick - last_imu_tick > FAILSAFE_TIMEOUT_MS || tick - last_bar_tick > FAILSAFE_TIMEOUT_MS)
            mode = QUAD_STATUS_FAILSAFE;

        // use generic watchdog for failsafe
        // reset it every loop iteration in case something gets hung up

        if (imu_dr)
        {
            float a_x, a_y, a_z, w_x, w_y, w_z;

            // update orientation estimation
            imu_read_gyro_radps(&imu, &w_x, &w_y, &w_z);
            imu_read_accel_mps2(&imu, &a_x, &a_y, &a_z);
            madgwick_update(a_x, a_y, a_z, w_x, w_y, w_z, &state);

            // stream orientation over USB (for debugging)
            // float data[] = { state.q_state.q1, state.q_state.q2, state.q_state.q3, state.q_state.q4 };
            // CDC_Transmit_FS((uint8_t *)data, sizeof(data));
            imu_dr = 0;
        }

        if (mode == QUAD_STATUS_DISARMED)
        {
            dshot_queue(&motor_fl, 0, 0);
            dshot_queue(&motor_fr, 0, 0);
            dshot_queue(&motor_bl, 0, 0);
            dshot_queue(&motor_br, 0, 0);

            float data[] = { 0, 0, 0, 0 };
            CDC_Transmit_FS((uint8_t *)data, sizeof(data));

            HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
        }
        else if (mode == QUAD_STATUS_ARMED)
        {
            // calculate quaternion error
            // convert quaternion error to a rate error
            // convert rate error to torque/speed commands
            // mix torque/speed commands
            // send to motors

            // compute rotation needed to move from current orientation to desired orientation (AKA error)
            quaternion_t q_state_conj, q_err;
            quat_copy(state.q_state, &q_state_conj);
            quat_conjugate(&q_state_conj);
            quat_mult(q_state_conj, pkt.q_des, &q_err);

            // convert error to rate error
            float sign = 1.0;
            if (q_err.q1 < 0)
                sign = -1.0;

            // calculate 3 axis attitude error --> need more detail about this section
            // body frame convention, not sensor frame
            float e_x = 2.0 * sign * q_err.q2;
            float e_y = 2.0 * sign * q_err.q3;
            float e_z = 2.0 * sign * q_err.q4;

            float w_x_des = e_x * k_att;
            float w_y_des = e_y * k_att;
            float w_z_des = e_z * k_att;

            // clamp rates to certain range
            // clamp(w_x_des, max_rate, min_rate)

            // this needs to be fixed and placed into library
            // compute ew
            float w_x_err = w_x_des - state.q_gyro.q2;
            float w_y_err = w_y_des - state.q_gyro.q3;
            float w_z_err = w_z_des - state.q_gyro.q4;

            // run PID on ew
            // put all operations into a matrix/vector operation to clean up
            // leave ki = 0 for testing
            // float tau_x = pkt.kp * w_x_err + pkt.ki * w_x_sum + pkt.kd * w_x_err - last_w_x;
            // float tau_y = pkt.kp * w_y_err + pkt.ki * w_y_sum + pkt.kd * w_y_err - last_w_y;
            // float tau_z = pkt.kp * w_z_err + pkt.ki * w_z_sum + pkt.kd * w_z_err - last_w_z;
            float tau_x = pkt.kp * w_x_err;
            float tau_y = pkt.kp * w_y_err;
            float tau_z = pkt.kp * w_z_err;
            // mix torque commands for each motor

            // determine directions for tau_x, tau_y, tau_z
            // tau_x requires left motors to match, right motors to match
            // tau_y requires front motors to match, back motors to match
            // tau_z requires diagonals to match
            uint16_t motor_speed_fl = pkt.throttle + tau_x - tau_y + tau_z;
            uint16_t motor_speed_fr = pkt.throttle - tau_x - tau_y - tau_z;
            uint16_t motor_speed_bl = pkt.throttle + tau_x + tau_y - tau_z;
            uint16_t motor_speed_br = pkt.throttle - tau_x + tau_y + tau_z;
            // send

            clamp(&motor_speed_fl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
            clamp(&motor_speed_fr, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
            clamp(&motor_speed_bl, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
            clamp(&motor_speed_br, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

            uint16_t data[] = { motor_speed_fl, motor_speed_fr, motor_speed_bl, motor_speed_br };
            CDC_Transmit_FS((uint8_t *)data, sizeof(data));

            HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);
        }
        */
        /*
            // calculate orientation (only if IMU working)
            switch (mode)
            {
                case QUAD_STATUS_DISARMED:
                    // stop motors
                    break;
                case QUAD_STATUS_ARMED:
                    // drive motors
                    break;
                case QUAD_STATUS_FAILSAFE:
                    // stop motors
                    // attempt recovery
                default:

            }
            */
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // RF IRQ on falling edge
    if (GPIO_Pin == RF_IRQ_Pin)
    {
        rf_dr = 1;
        last_pkt_tick = HAL_GetTick();
    }
    // IMU IRQ on rising edge
    if (GPIO_Pin == IMU_IRQ1_Pin)
    {
        imu_dr = 1;
        last_imu_tick = HAL_GetTick();
    }
    // BAR IRQ on rising edge
    if (GPIO_Pin == BAR_IRQ_Pin)
    {
        bar_dr = 1;
        last_bar_tick = HAL_GetTick();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim)
{
    dshot_complete_callback(&motor_fl, htim);
    dshot_complete_callback(&motor_fr, htim);
    dshot_complete_callback(&motor_bl, htim);
    dshot_complete_callback(&motor_br, htim);
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef * htim)
{
    dshot_half_complete_callback(&motor_fl, htim);
    dshot_half_complete_callback(&motor_fr, htim);
    dshot_half_complete_callback(&motor_bl, htim);
    dshot_half_complete_callback(&motor_br, htim);
}

static void clamp(uint16_t * x, uint16_t min, uint16_t max)
{
    if (*x < min)
        *x = min;
    else if (*x > max)
        *x = max;
}