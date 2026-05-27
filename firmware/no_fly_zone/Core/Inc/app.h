#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void blink(void);
void app_init(SPI_HandleTypeDef * hspi);
void app(void);

#endif