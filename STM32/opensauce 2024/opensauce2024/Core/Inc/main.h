/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
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
#define ACCEL_SELECT_Pin GPIO_PIN_4
#define ACCEL_SELECT_GPIO_Port GPIOA
#define LED_BLANK_Pin GPIO_PIN_5
#define LED_BLANK_GPIO_Port GPIOA
#define KEEP_POWER_ON_Pin GPIO_PIN_0
#define KEEP_POWER_ON_GPIO_Port GPIOB
#define LED_DATA_LATCH_Pin GPIO_PIN_5
#define LED_DATA_LATCH_GPIO_Port GPIOB
#define ACCEL_INTERRUPT_Pin GPIO_PIN_6
#define ACCEL_INTERRUPT_GPIO_Port GPIOB
#define LED_DATA_CLOCK_Pin GPIO_PIN_7
#define LED_DATA_CLOCK_GPIO_Port GPIOB
#define LED_DATA_Pin GPIO_PIN_8
#define LED_DATA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
