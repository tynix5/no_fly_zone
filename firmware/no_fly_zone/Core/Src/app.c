#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "iis2mdc.h"
#include "lps25hb.h"
#include "lsm6ds3tr.h"
#include "dshot.h"
#include "rf_structs.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#define RF_TX_ADDR                 0xC2C2C2C2
#define RF_RX_ADDR                 0xE7E7E7E7

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
        .odr = IMU_ODR_G_12_5HZ,
        .fs = IMU_FS_G_1000DPS,
        .filt = IMU_HPF_EN,
        .cutoff = IMU_HP_G_260MILHZ
    },

    .AccelParams = {
        .odr = IMU_ODR_XL_12_5HZ,
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


void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, UART_HandleTypeDef * huart1)
{
    // deselect all slaves at start
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BAR_CS_GPIO_Port, BAR_CS_Pin, GPIO_PIN_SET);

    rx.hspi = hspi3;
    rf_init(&rx);
    /*

    bar.hspi = hspi1;
    if (bar_init(&bar) == RET_OK)
        HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
    
    imu.hspi = hspi1;
    if (imu_init(&imu) == RET_OK)
        HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);

    mag.hspi = hspi2;
    if (mag_init(&mag) == RET_OK)
        HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_SET);
        
    HAL_Delay(3000);
    HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_RESET);
    */
    uint16_t pkt[16];
    uint16_t throttle = 0b10000010110;
    dshot_encode(throttle, 0, pkt);
    __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_4, 0);
    HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);

    for (uint8_t i = 0; i < 16; i++)
    {
        while (__HAL_TIM_GET_FLAG(htim2, TIM_FLAG_UPDATE) == 0);
        __HAL_TIM_CLEAR_FLAG(htim2, TIM_FLAG_UPDATE);
        __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_4, pkt[i]);
            
    }

    while (__HAL_TIM_GET_FLAG(htim2, TIM_FLAG_UPDATE) == 0);
    __HAL_TIM_CLEAR_FLAG(htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_4, 0);
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

    rf_listen_it(&rx);

    while (1)
    {
        /* Create IMU library functions for filtering accel, gyro data */
        /* Take notes on complementary filter, madgwick filter, kalman filter */
        /* Create simple complementary filter */
        /* Create PWM generator for motor speeds */
        /* Create PID loop for quadcopter controller */
        /* Create timer to sample VBAT ADC every second or so*/
        

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
            imu_read_gyro_degps(&imu, &w_x, &w_y, &w_z);    
            send = 1;
            gyro_dr = 0;
            HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);
        }
        if (accel_dr)
        {
            imu_read_accel_mps2(&imu, &a_x, &a_y, &a_z); 
            send = 1;
            accel_dr = 0;
            // HAL_GPIO_TogglePin(STAT2_GPIO_Port, STAT2_Pin);
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
        }
            */
            
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