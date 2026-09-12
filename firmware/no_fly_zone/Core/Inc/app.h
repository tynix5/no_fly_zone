#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void app_init(ADC_HandleTypeDef * hadc,
              SPI_HandleTypeDef * hspi_sensor,
              SPI_HandleTypeDef * hspi_rf,
              TIM_HandleTypeDef * htim2_motor,
              TIM_HandleTypeDef * htim_adc,
              TIM_HandleTypeDef * htim5_motor,
              TIM_HandleTypeDef * htim_us,
              IWDG_HandleTypeDef * hiwdg);
void app(void);

#endif