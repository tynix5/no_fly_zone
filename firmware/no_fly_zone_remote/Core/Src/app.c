#include "app.h"
#include "nrf24.h"
#include "drivers.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"

#define RF_TX_ADDR                 0xC2C2C2C2
#define RF_RX_ADDR                 0xE7E7E7E7

static RadioParams tx = {

    .this_addr = RF_TX_ADDR,
    .node_addr = RF_RX_ADDR,
    .spi_transfer = spi1_transfer,
    .spi_transfer_byte = spi1_transfer_byte,
    .rf_enable = ce_high,
    .rf_disable = ce_low,
    .delay_ms = HAL_Delay,
};

void app_init(void)
{
    spi1_init();
    rf_init(tx);

    // ce output
    GPIOA->MODER &= ~GPIO_MODER_MODE8;
    GPIOA->MODER |= GPIO_MODER_MODE8_0;

}

void app(void)
{
    while (1)
    {
        uint8_t packet = 0x5e;
        rf_send(tx, &packet, 1);
        HAL_Delay(250);
    }
}

void ce_low(void)
{
    GPIOA->ODR &= ~GPIO_ODR_OD8;
}

void ce_high(void)
{
    GPIOA->ODR |= GPIO_ODR_OD8;
}