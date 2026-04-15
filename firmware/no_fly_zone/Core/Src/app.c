#include "app.h"
#include "nrf24.h"
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

static RadioParams receiver;         // only app.c functions can access testing


/* Sysclk running at 168 MHz */
/* HCLK running at 168 MHz */
/* Cortex system timer running at 21 MHz */
/* FCLK running at 168 MHz */
/* APB1 peripheral clocks running at 42 MHz */
/* APB1 timer clocks running at 84 MHz */
/* APB2 peripheral clocks running at 84 MHz */
/* APB2 timer clocks running at 168 MHz */
/* USB running at 48 MHz */

void app_init(void)
{
    spi1_init();
    receiver.this_addr = 0xE7E7E7E7;
    receiver.node_addr = 0xC2C2C2C2;
    receiver.spi_transfer = spi1_transfer;
    receiver.spi_transfer_byte = spi1_transfer_byte;
    receiver.ce_high = ce_high;
    receiver.ce_low = ce_low;
    receiver.delay_ms = HAL_Delay;
    ce_low();

    // PD2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~GPIO_MODER_MODE2;
    GPIOD->MODER |= GPIO_MODER_MODE2_0;
}

void app(void)
{
    while (1)
    {
        nrf_listen(receiver, 0xffffffff);

        uint8_t packet, len;
        nrf_receive(receiver, &packet, &len);
        if (len == 1 && packet == 0x5e)
            GPIOD->ODR ^= GPIO_ODR_OD2;
    }
}

void spi1_init(void)
{
    // PB3 --> SPI1_SCK
    // PB4 --> SPI1_MISO
    // PB5 --> SPI1_MOSI
    
    // USER is PD2
    // CS is PC0
    // CE is PC1

    // SPI1 is on APB2 bus
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // alternate function mode
    GPIOB->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOB->MODER |= GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1;

    // select mode 5 for all
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL3 | GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5);
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL3_0 | GPIO_AFRL_AFRL3_2;
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL4_0 | GPIO_AFRL_AFRL4_2;
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL5_0 | GPIO_AFRL_AFRL5_2;

    // outputs
    GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOC->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0;

    // master mode
    SPI1->CR1 |= SPI_CR1_MSTR;

    // /64 prescaler
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
    SPI1->CR1 &= ~SPI_CR1_BR_1;

    // enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;

    cs_high();
}

uint8_t spi1_transfer_byte(uint8_t addr, uint8_t data)
{
    cs_low();

    SPI1->DR = addr;
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_TXE));

    cs_high();

    return SPI1->DR;
}

void spi1_transfer(uint8_t addr, uint8_t * src, uint8_t * dest, uint8_t len)
{
    cs_low();

    SPI1->DR = addr;
    while (!(SPI1->SR & SPI_SR_TXE));

    for (uint8_t i = 0; i < len; i++)
    {
        SPI1->DR = src[i];
        while (!(SPI1->SR & SPI_SR_TXE));
        dest[i] = SPI1->DR;
    }

    cs_high();
}

void cs_low(void)
{
    GPIOC->ODR &= ~GPIO_ODR_OD0;
}

void cs_high(void)
{
    GPIOC->ODR |= GPIO_ODR_OD0;
}

void ce_low(void)
{
    GPIOC->ODR &= ~GPIO_ODR_OD1;
}

void ce_high(void)
{
    GPIOC->ODR |= GPIO_ODR_OD1;
}