#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void app_init(ADC_HandleTypeDef * hadc1,
              SPI_HandleTypeDef * hspi2,
              SPI_HandleTypeDef * hspi3,
              TIM_HandleTypeDef * htim2,
              TIM_HandleTypeDef * htim3,
              TIM_HandleTypeDef * htim5,
              IWDG_HandleTypeDef * hiwdg);
void app(void);

#endif