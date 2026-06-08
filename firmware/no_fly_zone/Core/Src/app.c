#include "app.h"
#include "main.h"
#include "nrf24.h"
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

/* Sysclk running at 168 MHz */
/* HCLK running at 168 MHz */
/* Cortex system timer running at 21 MHz */
/* FCLK running at 168 MHz */
/* APB1 peripheral clocks running at 42 MHz */
/* APB1 timer clocks running at 84 MHz */
/* APB2 peripheral clocks running at 84 MHz */
/* APB2 timer clocks running at 168 MHz */
/* USB running at 48 MHz */

void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, UART_HandleTypeDef * huart1, PCD_HandleTypeDef * husb);
{
    rx.hspi = hspi3;
    rf_init(&rx);

    rf_set_irq(rx, irqs);
}

void app(void)
{
    uint8_t count = 0;

    AckParams ack = {.key = ACK_KEY, .rx_batt_lvl = 43};
    PacketParams packet;

    uint8_t packet_len;
    while (1)
    {
        /******** Move all radio listen functions to EXTI */
        /* Create simple complementary filter */
        /* Create IMU library functions for filtering accel, gyro data */
        /* Create PWM generator for motor speeds */
        /* Create PID loop for quadcopter controller */
        if (rf_listen(&rx, 100000) == RF_SUCCESS)
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
            if (packet_len == PACKET_SIZE && packet.key == PACKET_KEY && packet.throttle > 128)
                HAL_GPIO_TogglePin(USER_GPIO_Port, USER_Pin);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}