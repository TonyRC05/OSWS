/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define TFT_Pin GPIO_PIN_0
#define TFT_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_1
#define LIGHT_GPIO_Port GPIOA
#define SD_STATUS_Pin GPIO_PIN_0
#define SD_STATUS_GPIO_Port GPIOB
#define LED_SD_Pin GPIO_PIN_1
#define LED_SD_GPIO_Port GPIOB
#define BTN_SD_Pin GPIO_PIN_12
#define BTN_SD_GPIO_Port GPIOB
#define BTN_SD_EXTI_IRQn EXTI15_10_IRQn
#define BTN_INTERVAL_Pin GPIO_PIN_14
#define BTN_INTERVAL_GPIO_Port GPIOB
#define BTN_INTERVAL_EXTI_IRQn EXTI15_10_IRQn
#define SENSORS_STATUS_Pin GPIO_PIN_12
#define SENSORS_STATUS_GPIO_Port GPIOA
#define TFT_EN_Pin GPIO_PIN_4
#define TFT_EN_GPIO_Port GPIOB
#define TFT_EN_EXTI_IRQn EXTI4_IRQn
/* USER CODE BEGIN Private defines */
typedef enum {
	STATUS_OK = 1,
	STATUS_FAIL = 0,
}Status_Flag;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
