#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"


#define RF_TX_ADDR                 0xC2C2C2C2
#define RF_RX_ADDR                 0xE7E7E7E7

uint8_t count = 0;

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

typedef struct {

    uint8_t batt_lvl;
} AckParams;

/* Sysclk running at 168 MHz */
/* HCLK running at 168 MHz */
/* Cortex system timer running at 21 MHz */
/* FCLK running at 168 MHz */
/* APB1 peripheral clocks running at 42 MHz */
/* APB1 timer clocks running at 84 MHz */
/* APB2 peripheral clocks running at 84 MHz */
/* APB2 timer clocks running at 168 MHz */
/* USB running at 48 MHz */

void blink(void)
{
    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
}

void app_init(SPI_HandleTypeDef * hspi)
{
    rx.hspi = hspi;
    rf_init(&rx);

    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(USER_GPIO_Port, USER_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);

}

void app(void)
{
    while (1)
    {
        
        uint8_t packet[3], len;
        uint8_t response[2] = {0x05, 0x02};

        if (rf_listen(&rx, 100000) == RF_SUCCESS)
        {
            count++;
            if (count == 10)
            {
                count = 0;
                rf_receive(&rx, packet, &len, response, 2);
            }
            else
            {
                rf_receive(&rx, packet, &len, response, 0);
            }
            if (len == 3 && packet[0] == 0x5e && packet[1] == 0x54 && packet[2] == 0x21)
                   HAL_GPIO_TogglePin(USER_GPIO_Port, USER_Pin);
        }
    }
}