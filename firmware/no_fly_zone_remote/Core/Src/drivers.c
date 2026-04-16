#include "drivers.h"
#include "stm32l432xx.h"

void spi1_init(void)
{
    // A6 --> PA7 --> SPI1_MOSI
    // A5 --> PA6 --> SPI1_MISO
    // A4 --> PA5 --> SPI1_SCLK
    // A3 --> PA4 --> SPI1_NSS
    // D9 --> PA8 --> RF_CE

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // select alternate function mode
    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |= GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;

    // SPI1 on APB2 bus (80 MHz)

    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;       // /64

    SPI1->CR1 |= SPI_CR1_MSTR;      // master mode
    SPI1->CR1 |= SPI_CR1_SSM;       // software slave management

    // 8 bits
    SPI1->CR2 |= SPI_CR2_DS;
    SPI1->CR2 &= ~SPI_CR2_DS_3;

    SPI1->CR2 |= SPI_CR2_SSOE;      // slave enable

    SPI1->CR1 |= SPI_CR1_SPE;       // enable SPI
}

uint8_t spi1_transfer_byte(uint8_t addr, uint8_t data)
{
    SPI1->DR = addr;
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_TXE));

    return SPI1->DR;
}

void spi1_transfer(uint8_t addr, uint8_t * src, uint8_t * dest, uint8_t len)
{
    SPI1->DR = addr;
    while (!(SPI1->SR & SPI_SR_TXE));

    for (uint8_t i = 0; i < len; i++)
    {
        SPI1->DR = src[i];
        while (!(SPI1->SR & SPI_SR_TXE));
        dest[i] = SPI1->DR;
    }
}