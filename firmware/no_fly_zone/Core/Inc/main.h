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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VBAT_ADC_Pin GPIO_PIN_6
#define VBAT_ADC_GPIO_Port GPIOA
#define CURR_ADC_Pin GPIO_PIN_7
#define CURR_ADC_GPIO_Port GPIOA
#define STAT3_Pin GPIO_PIN_2
#define STAT3_GPIO_Port GPIOB
#define BAR_CS_Pin GPIO_PIN_12
#define BAR_CS_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOC
#define BAR_IRQ_Pin GPIO_PIN_7
#define BAR_IRQ_GPIO_Port GPIOC
#define BAR_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define IMU_IRQ1_Pin GPIO_PIN_8
#define IMU_IRQ1_GPIO_Port GPIOA
#define IMU_IRQ1_EXTI_IRQn EXTI9_5_IRQn
#define IMU_IRQ2_Pin GPIO_PIN_9
#define IMU_IRQ2_GPIO_Port GPIOA
#define IMU_IRQ2_EXTI_IRQn EXTI9_5_IRQn
#define RF_IRQ_Pin GPIO_PIN_15
#define RF_IRQ_GPIO_Port GPIOA
#define RF_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define STAT1_Pin GPIO_PIN_4
#define STAT1_GPIO_Port GPIOB
#define STAT2_Pin GPIO_PIN_5
#define STAT2_GPIO_Port GPIOB
#define RF_CS_Pin GPIO_PIN_6
#define RF_CS_GPIO_Port GPIOB
#define RF_CE_Pin GPIO_PIN_7
#define RF_CE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
