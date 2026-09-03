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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Internal_LED_Pin GPIO_PIN_13
#define Internal_LED_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_6
#define ENC2_A_GPIO_Port GPIOA
#define ENC2_B_Pin GPIO_PIN_7
#define ENC2_B_GPIO_Port GPIOA
#define IN1_1_Pin GPIO_PIN_12
#define IN1_1_GPIO_Port GPIOB
#define IN2_1_Pin GPIO_PIN_13
#define IN2_1_GPIO_Port GPIOB
#define IN3_1_Pin GPIO_PIN_14
#define IN3_1_GPIO_Port GPIOB
#define IN4_1_Pin GPIO_PIN_15
#define IN4_1_GPIO_Port GPIOB
#define ENA_1_Pin GPIO_PIN_8
#define ENA_1_GPIO_Port GPIOA
#define ENB_1_Pin GPIO_PIN_9
#define ENB_1_GPIO_Port GPIOA
#define ENA_2_Pin GPIO_PIN_10
#define ENA_2_GPIO_Port GPIOA
#define IN1_2_Pin GPIO_PIN_11
#define IN1_2_GPIO_Port GPIOA
#define IN2_2_Pin GPIO_PIN_12
#define IN2_2_GPIO_Port GPIOA
#define ENC3_B_Pin GPIO_PIN_6
#define ENC3_B_GPIO_Port GPIOB
#define ENC3_A_Pin GPIO_PIN_7
#define ENC3_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
