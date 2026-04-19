#include "app.h"
#include "main.h"
#include "nrf24.h"
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
    .dynamic_payloads = USE_DYNAMIC_PAYLOAD,
    .use_acks = NO_ACKS
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

void app_init(SPI_HandleTypeDef * hspi)
{
    rx.hspi = hspi;
    rf_init(&rx);
}

void app(void)
{
    while (1)
    {
        if (rf_listen_no_ack(&rx, 0xffffffff))
        {
            uint8_t packet[3], len;
            rf_receive(&rx, packet, &len);

            if (len == 3 && packet[0] == 0x5e && packet[1] == 0x54 && packet[2] == 0x21)
                HAL_GPIO_TogglePin(USER_GPIO_Port, USER_Pin);
        }
    }
}