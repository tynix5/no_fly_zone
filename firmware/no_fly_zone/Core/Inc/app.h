#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void dshot_next_bit(TIM_HandleTypeDef * htim);
void app_init(ADC_HandleTypeDef * hadc1, SPI_HandleTypeDef * hspi1, SPI_HandleTypeDef * hspi2, SPI_HandleTypeDef * hspi3, TIM_HandleTypeDef * htim2, UART_HandleTypeDef * huart1, DMA_HandleTypeDef * hdma);
void app(void);

#endif