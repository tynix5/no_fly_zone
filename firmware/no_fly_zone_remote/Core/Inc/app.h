#ifndef APP_H_
#define APP_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

typedef struct
{
    uint16_t throttle;
    uint16_t yaw;
    uint16_t pitch;
    uint16_t roll;

} joystick_t;

typedef enum
{
    QUAD_STATUS_DISARMED = 0,
    QUAD_STATUS_ARMED

} quad_arm_status_t;

void app_init(ADC_HandleTypeDef * hadc,
              DMA_HandleTypeDef * hdma,
              I2C_HandleTypeDef * hi2c,
              SPI_HandleTypeDef * hspi,
              TIM_HandleTypeDef * htim,
              TIM_HandleTypeDef * henc);
void app(void);

#endif