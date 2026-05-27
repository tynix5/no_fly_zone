#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

void app_init(ADC_HandleTypeDef * hadc, DMA_HandleTypeDef * hdma, I2C_HandleTypeDef * hi2c, SPI_HandleTypeDef * hspi, TIM_HandleTypeDef * htim, PCD_HandleTypeDef * husb);
void app(void);

#endif