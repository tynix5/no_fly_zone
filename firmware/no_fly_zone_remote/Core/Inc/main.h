/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PITCH_ADC_Pin GPIO_PIN_2
#define PITCH_ADC_GPIO_Port GPIOA
#define ROLL_ADC_Pin GPIO_PIN_3
#define ROLL_ADC_GPIO_Port GPIOA
#define THROTTLE_ADC_Pin GPIO_PIN_4
#define THROTTLE_ADC_GPIO_Port GPIOA
#define BATT_LVL_Pin GPIO_PIN_0
#define BATT_LVL_GPIO_Port GPIOB
#define YAW_ADC_Pin GPIO_PIN_1
#define YAW_ADC_GPIO_Port GPIOB
#define RF_CE_Pin GPIO_PIN_8
#define RF_CE_GPIO_Port GPIOA
#define RF_IRQ_Pin GPIO_PIN_9
#define RF_IRQ_GPIO_Port GPIOA
#define RF_CS_Pin GPIO_PIN_10
#define RF_CS_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_15
#define USER_LED_GPIO_Port GPIOA
#define ENC_SW_Pin GPIO_PIN_4
#define ENC_SW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
