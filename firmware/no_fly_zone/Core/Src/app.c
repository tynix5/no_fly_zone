#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "iis2mdc.h"
#include "lps25hb.h"
#include "lsm6ds3tr.h"
#include "rf_structs.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"

#define RF_TX_ADDR                 0xC2C2C2C2
#define RF_RX_ADDR                 0xE7E7E7E7

static RadioParams rx = {

    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .this_addr = RF_RX_ADDR,
    .node_addr = RF_TX_ADDR,
    .delay_ms = HAL_Delay,
    .power_level = RF_PWR_0DBM,
    .data_rate = RF_DR_2MBPS,
    .freq_ch = 2,
    .payload_type = DYNAMIC_PAYLOAD,
    .ack = ENABLE_ACK
};

// only interrupt on data received
RadioIrqs irqs = {

    .rx_dr = INT_EN,
    .tx_ds = INT_DIS,
    .max_rt = INT_DIS
};

uint8_t rf_dr = 0;          // data received from nRF24L01


BarParams bar = {

    .cs_gpio = BAR_CS_GPIO_Port,
    .cs = BAR_CS_Pin,
    .odr = BAR_ODR_25HZ,
    .fifo_mode = BAR_BYPASS_MODE,
    .delay_ms = HAL_Delay
};

const GyroParams gyro = {

    .odr = IMU_ODR_1_66KHZ,
    .fs = IMU_FS_G_1000DPS,
    .lpf = IMU_LPF_EN,
    .hpf = IMU_HPF_DIS
};

const AccelParams accel = {

    .odr = IMU_ODR_1_66KHZ,
    .fs = IMU_FS_XL_8G,
    .filter_mode = IMU_NO_FILTER            // if high pass filter is enabled, it will turn z-axis into "0"
};

IMUParams imu = {

    .cs_gpio = IMU_CS_GPIO_Port,
    .cs = IMU_CS_Pin,
    .gyro = gyro,
    .xl = accel,
    .delay_ms = HAL_Delay
};

MagParams mag = {
    .cs_gpio = MAG_CS_GPIO_Port,
    .cs = MAG_CS_Pin,
    .delay_ms = HAL_Delay,
    .lpf = DISABLE,
    .odr = MAG_ODR_20HZ,
    .mode = MAG_MODE_CONT,
    .int_drdy = ENABLE
};


void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, UART_HandleTypeDef * huart1)
{
    // deselect all slaves at start
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BAR_CS_GPIO_Port, BAR_CS_Pin, GPIO_PIN_SET);

    rx.hspi = hspi3;
    rf_init(&rx);

    bar.hspi = hspi1;
    if (bar_init(&bar) == SUCCESS)
        HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
    
    imu.hspi = hspi1;
    if (imu_init(&imu) == SUCCESS)
        HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);

    mag.hspi = hspi2;
    if (mag_init(&mag) == SUCCESS)
        HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_SET);
    
    rf_set_irq(&rx, &irqs);
    
    HAL_Delay(3000);
    HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_RESET);
}

void app(void)
{
    uint8_t count = 0;

    AckParams ack = {.key = ACK_KEY, .rx_batt_lvl = 84};
    PacketParams packet;

    uint8_t packet_len;

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
        */
        float a_x, a_y, a_z, w_x, w_y, w_z;
        int16_t m_x = 2, m_y = 1, m_z = 1;
        float hpa;
        imu_read_accel_mps2(&imu, &a_x, &a_y, &a_z);         // correct
        imu_read_gyro_radps(&imu, &w_x, &w_y, &w_z);         // correct
        imu_read_gyro_degps(&imu, &w_x, &w_y, &w_z);            // correct
        if (mag_is_data_ready(&mag, 500) == SUCCESS)
        {
            mag_read_raw(&mag, &m_x, &m_y, &m_z);
        }
        if (bar_is_data_ready(&bar, 50) == SUCCESS)
        {
            hpa = bar_read_press_hpa(&bar);
        }

        float accel[3] = {a_x, a_y, a_z};
        float omega[3] = {w_x, w_y, w_z};
        int mag[3] = {(int) m_x, (int) m_y, (int) m_z};

        CDC_Transmit_FS((uint8_t *) &hpa, sizeof(hpa));
        HAL_Delay(50);
        HAL_GPIO_TogglePin(STAT1_GPIO_Port, STAT1_Pin);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // RF_IRQ on falling edge
    if (GPIO_Pin == RF_IRQ_Pin)
    {
        rf_dr = 1;
    }
}