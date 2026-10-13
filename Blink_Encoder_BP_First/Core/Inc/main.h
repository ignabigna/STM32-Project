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
#define Escribe_IN1_Pin GPIO_PIN_2
#define Escribe_IN1_GPIO_Port GPIOA
#define Escribe_IN2_Pin GPIO_PIN_3
#define Escribe_IN2_GPIO_Port GPIOA
#define LASER_Pin GPIO_PIN_0
#define LASER_GPIO_Port GPIOB
#define BTN1_SCAN_Pin GPIO_PIN_12
#define BTN1_SCAN_GPIO_Port GPIOB
#define BTN2_SET_H_Pin GPIO_PIN_13
#define BTN2_SET_H_GPIO_Port GPIOB
#define BTN3_GOH_Pin GPIO_PIN_14
#define BTN3_GOH_GPIO_Port GPIOB
#define BTN4_STOP_Pin GPIO_PIN_15
#define BTN4_STOP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
