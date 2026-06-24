#include "app.h"
#include "main.h"
#include "nrf24.h"
// #include "iis2mdc.h"
#include "lps25hb.h"
#include "lsm6ds3tr.h"
#include "rf_structs.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

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
static RadioIrqs irqs = {

    .rx_dr = INT_EN,
    .tx_ds = INT_DIS,
    .max_rt = INT_DIS
};

uint8_t rf_dr = 0;          // data received from nRF24L01


static BarParams bar = {

    .cs_gpio = BAR_CS_GPIO_Port,
    .cs = BAR_CS_Pin,
    .odr = BAR_ODR_25HZ,
    .fifo_mode = BAR_BYPASS_MODE
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
    .filter_mode = IMU_HPF_EN
};

IMUParams imu = {

    .cs_gpio = IMU_CS_GPIO_Port,
    .cs = IMU_CS_Pin,
    .gyro = gyro,
    .xl = accel
};

/* Sysclk running at 168 MHz */
/* HCLK running at 168 MHz */
/* Cortex system timer running at 21 MHz */
/* FCLK running at 168 MHz */
/* APB1 peripheral clocks running at 42 MHz */
/* APB1 timer clocks running at 84 MHz */
/* APB2 peripheral clocks running at 84 MHz */
/* APB2 timer clocks running at 168 MHz */
/* USB running at 48 MHz */

void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, UART_HandleTypeDef * huart1, PCD_HandleTypeDef * husb)
{
    rx.hspi = hspi3;
    rf_init(&rx);

    bar.hspi = hspi1;
    bar_init(&bar);
    HAL_GPIO_WritePin(STAT1_GPIO_Port, STAT1_Pin, GPIO_PIN_SET);
    
    imu.hspi = hspi1;
    imu_init(&imu);
    HAL_GPIO_WritePin(STAT2_GPIO_Port, STAT2_Pin, GPIO_PIN_SET);

    rf_set_irq(&rx, &irqs);
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