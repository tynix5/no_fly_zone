#include "app.h"
#include "main.h"
#include "nrf24.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"

#define RF_TX_ADDR                 0xE7E7E7E7
#define RF_RX_ADDR                 0xE7E7E7E7

static RadioParams tx = {

    .this_addr = RF_TX_ADDR,
    .node_addr = RF_RX_ADDR,
    .ce_gpio = RF_CE_GPIO_Port,
    .ce_pin = RF_CE_Pin,
    .cs_gpio = RF_CS_GPIO_Port,
    .cs_pin = RF_CS_Pin,
    .delay_ms = HAL_Delay,
};

void app_init(SPI_HandleTypeDef * hspi)
{
    tx.hspi = hspi;
    rf_init(&tx);

}

void app(void)
{
    while (1)
    {
        uint8_t packet = 0x5e;
        rf_send(&tx, &packet, 1);
        HAL_Delay(250);
    }
}