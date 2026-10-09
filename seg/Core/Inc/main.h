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
#define led_placa_Pin GPIO_PIN_13
#define led_placa_GPIO_Port GPIOC
#define ledon_Pin GPIO_PIN_1
#define ledon_GPIO_Port GPIOA
#define qtr1_Pin GPIO_PIN_2
#define qtr1_GPIO_Port GPIOA
#define qtr2_Pin GPIO_PIN_3
#define qtr2_GPIO_Port GPIOA
#define qtr3_Pin GPIO_PIN_4
#define qtr3_GPIO_Port GPIOA
#define qtr4_Pin GPIO_PIN_5
#define qtr4_GPIO_Port GPIOA
#define qtr5_Pin GPIO_PIN_6
#define qtr5_GPIO_Port GPIOA
#define qtr6_Pin GPIO_PIN_7
#define qtr6_GPIO_Port GPIOA
#define qtr7_Pin GPIO_PIN_0
#define qtr7_GPIO_Port GPIOB
#define qtr8_Pin GPIO_PIN_1
#define qtr8_GPIO_Port GPIOB
#define bin1_Pin GPIO_PIN_10
#define bin1_GPIO_Port GPIOB
#define bin2_Pin GPIO_PIN_11
#define bin2_GPIO_Port GPIOB
#define sd_cs_Pin GPIO_PIN_12
#define sd_cs_GPIO_Port GPIOB
#define ir_Pin GPIO_PIN_8
#define ir_GPIO_Port GPIOA
#define ir_EXTI_IRQn EXTI9_5_IRQn
#define stby_Pin GPIO_PIN_5
#define stby_GPIO_Port GPIOB
#define ain1_Pin GPIO_PIN_8
#define ain1_GPIO_Port GPIOB
#define ain2_Pin GPIO_PIN_9
#define ain2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
