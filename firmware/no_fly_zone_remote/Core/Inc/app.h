#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

void app_init(SPI_HandleTypeDef * hspi);
void app(void);

#endif