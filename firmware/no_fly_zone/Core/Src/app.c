#include "app.h"
#include "main.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"
#include <stdio.h>

#include "lsm6ds3tr.h"
#include "iis2mdc.h"
#include "lps25hb.h"
#include "dshot.h"
#include "nrf24.h"
#include "rf_structs.h"
#include "quaternion.h"

#define BETA                        0.2f
#define MADGWICK_GAIN               BETA
#define DELTA_T                     1 / 1660.0          // replace with actual variable

#define RF_TX_ADDR                  0xC2C2C2C2
#define RF_RX_ADDR                  0xE7E7E7E7

RadioParams rx = {

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

    .RadioIrqs = {
        .rx_dr = FEAT_ENABLE,
        .tx_ds = FEAT_DISABLE,
        .max_rt = FEAT_DISABLE
    }
};

uint8_t rf_dr = 0;          // data received from nRF24L01

uint8_t mag_dr = 0;
uint8_t gyro_dr = 0, accel_dr = 0;
uint8_t bar_dr = 0;


BarParams bar = {

    .cs_gpio = BAR_CS_GPIO_Port,
    .cs = BAR_CS_Pin,
    .delay_ms = HAL_Delay,

    .odr = BAR_ODR_1HZ,
    .it = BAR_INT_DRDY,

    .FifoParams = {
        .mode = BAR_FIFO_BYPASS,
        .thresh = 31,
        .mov_smp = BAR_MOV_AVG_SMP_NONE
    }
};

MagParams mag = {

    .cs_gpio = MAG_CS_GPIO_Port,
    .cs = MAG_CS_Pin,
    .delay_ms = HAL_Delay,

    .odr = MAG_ODR_10HZ,
    .mode = MAG_MODE_CONT,
    .lpf = FEAT_DISABLE,
    .low_pwr = FEAT_DISABLE,

    .IntParams = {
        .int_drdy = MAG_DRDY_ON_PIN,
        .it = 0
    }
};

ImuParams imu = {

    .cs_gpio = IMU_CS_GPIO_Port,
    .cs = IMU_CS_Pin,
    .delay_ms = HAL_Delay,

    .GyroParams = {
        .odr = IMU_ODR_G_1_66KHZ,
        .fs = IMU_FS_G_1000DPS,
        .filt = IMU_HPF_EN,
        .cutoff = IMU_HP_G_16MILHZ
    },

    .AccelParams = {
        .odr = IMU_ODR_XL_1_66KHZ,
        .fs = IMU_FS_XL_8G,
        .filt = IMU_LPF_EN
    },

    .FifoParams = {
        .mode = IMU_FIFO_BYPASS,
        .odr = IMU_ODR_FIFO_DISABLE
    },

    .IntParams = {
        .int1 = IMU_INT1_DRDY_G,
        .int2 = IMU_INT2_DRDY_XL
    }
};

DShotParams dshot = {
    .bitrate = DSHOT_BR_300,
    .n = 4
};


void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, TIM_HandleTypeDef * htim5, UART_HandleTypeDef * huart1)
{
    // deselect all slaves at start
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BAR_CS_GPIO_Port, BAR_CS_Pin, GPIO_PIN_SET);
    
    /*
    rx.hspi = hspi3;
    rf_init(&rx);

    bar.hspi = hspi1;
    if (bar_init(&bar) == RET_OK)
        HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
    */

    imu.hspi = hspi1;
    if (imu_init(&imu) == RET_OK)
        HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);

    // mag.hspi = hspi2;
    // if (mag_init(&mag) == RET_OK)
    //     HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_SET);
        
    HAL_Delay(3000);
    HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_RESET);
    
    /*
    dshot.stream[0].htim = dshot.stream[3].htim = htim2;
    dshot.stream[1].htim = dshot.stream[2].htim = htim5;
    dshot.stream[0].channel = TIM_CHANNEL_1;
    dshot.stream[1].channel = TIM_CHANNEL_2;
    dshot.stream[2].channel = TIM_CHANNEL_3;
    dshot.stream[3].channel = TIM_CHANNEL_4;
    dshot.stream[0].freq = dshot.stream[1].freq = dshot.stream[2].freq = dshot.stream[3].freq = 84000000;

    dshot_init(&dshot);

    uint16_t throttle1 = 0b10000010110;
    uint16_t throttle2 = 0b10000110110;
    uint16_t throttle3 = 0b10001110110;
    uint16_t throttle4 = 0b10011110110;

    dshot_encode(&dshot, throttle1, 0, DSHOT_CH_1);
    dshot_encode(&dshot, throttle2, 0, DSHOT_CH_2);
    dshot_encode(&dshot, throttle3, 0, DSHOT_CH_3);
    dshot_encode(&dshot, throttle4, 0, DSHOT_CH_4);
    */
}

void app(void)
{
    uint8_t count = 0;

    AckParams ack = {.key = ACK_KEY, .rx_batt_lvl = 84};
    PacketParams packet;

    uint8_t packet_len;

    uint8_t send = 0;
    
    float a_x, a_y, a_z, w_x, w_y, w_z;
    float m_x, m_y, m_z;
    float hpa, temp_bar, temp_mag;

    Quaternion q_accel, q_gyro;
    Quaternion q_gyro_dot;
    Quaternion q_state, q_state_prev, q_state_dot;

    // initial state
    q_state_prev.q1 = 1.0;
    q_state_prev.q2 = 0;
    q_state_prev.q3 = 0;
    q_state_prev.q4 = 0;

    volatile uint8_t busy = 0;

    rf_listen_it(&rx);

    while (1)
    {
        /* Create timer to sample VBAT ADC every second or so*/
        
        /*
        DShotChannel starts[4] = {DSHOT_CH_1, DSHOT_CH_2, DSHOT_CH_3, DSHOT_CH_4};
        dshot_send(&dshot, starts, 4);
        HAL_Delay(100);
        */

        // process remote sticks
        // compute target pitch, roll, yaw (rate)
        /*
        if (rf_dr)
        {
            count++;
            if (count == 10)
            {
                // send ack
                count = 0;
                rf_receive(&rx, (uint8_t *) &packet, &packet_len, (uint8_t *) &ack, ACK_SIZE);
            }
            else
            {
                // send no ack
                rf_receive(&rx, (uint8_t *) &packet, &packet_len, (uint8_t *) &ack, 0);
            }
            if (packet_len == PACKET_SIZE && packet.key == PACKET_KEY && packet.throttle < 100)
                HAL_GPIO_TogglePin(USER_GPIO_Port, USER_Pin);

            rf_dr = 0;
            rf_listen_it(&rx);
        } 
        if (gyro_dr)
        {
            q_gyro.q1 = 0;
            imu_read_gyro_degps(&imu, &q_gyro.q2, &q_gyro.q3, &q_gyro.q4);    
            send = 1;
            gyro_dr = 0;
            HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);
        }
        if (accel_dr)
        {
            q_accel.q1 = 0;
            imu_read_accel_mps2(&imu, &q_accel.q2, &q_accel.q3, &q_accel.q4); 
            send = 1;
            accel_dr = 0;
            HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);
        }  
        if (bar_dr)  
        {
            bar_read_press_hpa(&bar, &hpa);
            bar_read_temp(&bar, &temp_bar);
            send = 1;
            bar_dr = 0;
            HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);
        }
        if (mag_dr)
        {
            mag_read_mgauss(&mag, &m_x, &m_y, &m_z);
            mag_read_temp(&mag, &temp_mag);             
            send = 1;
            mag_dr = 0;
            HAL_GPIO_TogglePin(USER_GPIO_Port, USER_Pin);
        }

        if (send)
        {
            float data[] = {a_x, a_y, a_z, w_x, w_y, w_z, m_x, m_y, m_z, temp_mag, hpa, temp_bar};
            CDC_Transmit_FS((uint8_t *) data, sizeof(data));
            send = 0;
        }*/

        // yaw stick controls desired yaw rate not absolute angle
        // make sure axis are aligned with madgwick filter
        // body frame i am using
        /*
                            ^ X
                            |
                            |______> Y
                            Z (down)
        */
    //    https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#filter-gain

        if (accel_dr)
        {
            HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);

            accel_dr = 0;

            float meas_x, meas_y, meas_z;
            q_gyro.q1 = 0;
            imu_read_gyro_radps(&imu, &meas_x, &meas_y, &meas_z);  
            // align sensor coordinates to body frame coordinates
            q_gyro.q2 = meas_y;
            q_gyro.q3 = meas_x;
            q_gyro.q4 = meas_z;  

            q_accel.q1 = 0;
            imu_read_accel_mps2(&imu, &meas_x, &meas_y, &meas_z); 
            // sensor coordinates --> body frame
            q_accel.q2 = -meas_y;
            q_accel.q3 = -meas_x;
            q_accel.q4 = meas_z;  
            // normalize acceleration
            quat_normalize(&q_accel);
            
            
            // compute f_g(q, s_a)
            // this is the error function: it calculates difference between predicted and measured state
            float fg[3];
            fg[0] = 2.0 * (q_state_prev.q2 * q_state_prev.q4 - q_state_prev.q1 * q_state_prev.q3) - q_accel.q2;            
            fg[1] = 2.0 * (q_state_prev.q1 * q_state_prev.q2 + q_state_prev.q3 * q_state_prev.q4) - q_accel.q3;            
            fg[2] = 2.0 * (0.5 - q_state_prev.q2 * q_state_prev.q2 - q_state_prev.q3 * q_state_prev.q3) - q_accel.q4;
            
            // create Jacobian J_g(q)
            // if quaternion slightly changes, how does error change?
            // float Jg[3 * 4] = {-2.0 * q_state_prev.q3, 2.0 * q_state_prev.q4, -2.0 * q_state_prev.q1, 2.0 * q_state_prev.q2,
            //                     2.0 * q_state_prev.q2, 2.0 * q_state_prev.q1, 2.0 * q_state_prev.q4, 2 * q_state_prev.q3,
            //                     0.0, -4.0 * q_state_prev.q2, -4.0 * q_state_prev.q3, 0.0};
            
            float JgT[4 * 3] = {-2.0 * q_state_prev.q3, 2.0 * q_state_prev.q2, 0.0,
                                2.0 * q_state_prev.q4, 2.0 * q_state_prev.q1, -4.0 * q_state_prev.q2,
                                -2.0 * q_state_prev.q1, 2.0 * q_state_prev.q4, -4.0 * q_state_prev.q3,
                                2.0 * q_state_prev.q2, 2.0 * q_state_prev.q3, 0.0};
                
            // compute quaternion derivative for gyro
            // q_gyro_dot = 1/2 * q_state_prev * q_gyro 
            quat_mult(q_state_prev, q_gyro, &q_gyro_dot);
            quat_mult_scalar(&q_gyro_dot, 0.5);
            
            /*
            arm_matrix_instance_f32 Jg_mat = {
                .numRows = 3,
                .numCols = 4,
                .pData = Jg
                };
                */
                
            arm_matrix_instance_f32 JgT_mat = {
                .numRows = 4,
                .numCols = 3,
                .pData = JgT
            };
            
            arm_matrix_instance_f32 fg_mat = {
                .numRows = 3,
                .numCols = 1,
                .pData = fg
            };
            
            float gradient[4];
            arm_matrix_instance_f32 gradient_mat = {
                .numRows = 4,
                .numCols = 1,
                .pData = gradient
            };
            
            
            
            // arm_mat_trans_f32(&Jg_mat, &JgT_mat);             // replace with hard coded transpose matrix
            
            // compute gradient = (J^T)f
            // linear estimate of how uncertainty in inputs propagates into uncertainty in output
            arm_mat_mult_f32(&JgT_mat, &fg_mat, &gradient_mat);
            
            Quaternion q_grad = {.q1 = gradient[0], .q2 = gradient[1], .q3 = gradient[2], .q4 = gradient[3]};
            quat_normalize(&q_grad);
            quat_mult_scalar(&q_grad, BETA);
            quat_sub(q_gyro_dot, q_grad, &q_state_dot);
            quat_mult_scalar(&q_state_dot, DELTA_T);
            quat_add(q_state_prev, q_state_dot, &q_state);
            quat_normalize(&q_state);
            
            q_state_prev.q1 = q_state.q1;
            q_state_prev.q2 = q_state.q2;
            q_state_prev.q3 = q_state.q3;
            q_state_prev.q4 = q_state.q4;
            
            // float pitch, roll, yaw;
            // quat_to_euler(q_state, &pitch, &roll, &yaw);        // verify pitch, roll, yaw estimates
            // pitch *= 180.0 / PI;
            // roll *= 180.0 / PI;
            // yaw *= 180.0 / PI;
            // yaw = 0.0f;

            // char data[100];
            // uint8_t len = sprintf(data, "%d,%d,%d\r\n", (int) pitch, (int) roll, (int) yaw);
            
            float data[] = {q_state.q1, q_state.q2, q_state.q3, q_state.q4};
            // float data[] = {q_gyro.q2, q_gyro.q3, q_gyro.q4, 0.0f};
            CDC_Transmit_FS((uint8_t *) data, sizeof(data));
            // uint8_t status = CDC_Transmit_FS((uint8_t *) data, len);

        }
                    
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // RF_IRQ on falling edge
    if (GPIO_Pin == RF_IRQ_Pin)
    {
        rf_dr = 1;
    }
    if (GPIO_Pin == MAG_IRQ_Pin)
    {
        mag_dr = 1;
    }
    if (GPIO_Pin == IMU_IRQ1_Pin)
    {
        gyro_dr = 1;
    }
    if (GPIO_Pin == IMU_IRQ2_Pin)
    {
        accel_dr = 1;
    }
    if (GPIO_Pin == BAR_IRQ_Pin)
    {
        bar_dr = 1;
    }
}