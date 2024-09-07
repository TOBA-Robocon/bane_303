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
#include "stm32f3xx_hal.h"

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
#define Servo1_signal_Pin GPIO_PIN_0
#define Servo1_signal_GPIO_Port GPIOA
#define Servo2_signal_Pin GPIO_PIN_1
#define Servo2_signal_GPIO_Port GPIOA
#define Limit_signal1_Pin GPIO_PIN_2
#define Limit_signal1_GPIO_Port GPIOA
#define Limit_signal2_Pin GPIO_PIN_3
#define Limit_signal2_GPIO_Port GPIOA
#define M1A_Pin GPIO_PIN_4
#define M1A_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOA
#define M1B_Pin GPIO_PIN_6
#define M1B_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOA
#define M2A_Pin GPIO_PIN_0
#define M2A_GPIO_Port GPIOB
#define M2B_Pin GPIO_PIN_1
#define M2B_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_8
#define LD3_GPIO_Port GPIOA
#define CAN_Select1_Pin GPIO_PIN_4
#define CAN_Select1_GPIO_Port GPIOB
#define CAN_Select2_Pin GPIO_PIN_5
#define CAN_Select2_GPIO_Port GPIOB
#define CAN_Select3_Pin GPIO_PIN_6
#define CAN_Select3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
