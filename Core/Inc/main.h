/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define CARD_DETECT_Pin GPIO_PIN_5
#define CARD_DETECT_GPIO_Port GPIOD
#define LCD_BK_Pin GPIO_PIN_4
#define LCD_BK_GPIO_Port GPIOD
#define LCD_TS_RST_Pin GPIO_PIN_3
#define LCD_TS_RST_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOC
#define LED_TS_INT_Pin GPIO_PIN_12
#define LED_TS_INT_GPIO_Port GPIOG
#define LED_TS_INT_EXTI_IRQn EXTI15_10_IRQn
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOI
#define KEY_Pin GPIO_PIN_4
#define KEY_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
