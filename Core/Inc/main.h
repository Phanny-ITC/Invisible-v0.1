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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ls3_Pin GPIO_PIN_2
#define Ls3_GPIO_Port GPIOE
#define Ls4_Pin GPIO_PIN_3
#define Ls4_GPIO_Port GPIOE
#define Ls5_Pin GPIO_PIN_4
#define Ls5_GPIO_Port GPIOE
#define Ls6_Pin GPIO_PIN_5
#define Ls6_GPIO_Port GPIOE
#define Ls7_Pin GPIO_PIN_6
#define Ls7_GPIO_Port GPIOE
#define en_1_Pin GPIO_PIN_4
#define en_1_GPIO_Port GPIOA
#define step_1_Pin GPIO_PIN_5
#define step_1_GPIO_Port GPIOA
#define dir_1_Pin GPIO_PIN_6
#define dir_1_GPIO_Port GPIOA
#define en_2_Pin GPIO_PIN_7
#define en_2_GPIO_Port GPIOA
#define step_2_Pin GPIO_PIN_4
#define step_2_GPIO_Port GPIOC
#define dir_2_Pin GPIO_PIN_5
#define dir_2_GPIO_Port GPIOC
#define en_3_Pin GPIO_PIN_0
#define en_3_GPIO_Port GPIOB
#define step_3_Pin GPIO_PIN_1
#define step_3_GPIO_Port GPIOB
#define dir_3_Pin GPIO_PIN_2
#define dir_3_GPIO_Port GPIOB
#define en_4_Pin GPIO_PIN_7
#define en_4_GPIO_Port GPIOE
#define step_4_Pin GPIO_PIN_8
#define step_4_GPIO_Port GPIOE
#define dir_4_Pin GPIO_PIN_9
#define dir_4_GPIO_Port GPIOE
#define en_5_Pin GPIO_PIN_10
#define en_5_GPIO_Port GPIOE
#define step_5_Pin GPIO_PIN_11
#define step_5_GPIO_Port GPIOE
#define dir_5_Pin GPIO_PIN_12
#define dir_5_GPIO_Port GPIOE
#define en_6_Pin GPIO_PIN_13
#define en_6_GPIO_Port GPIOE
#define step_6_Pin GPIO_PIN_14
#define step_6_GPIO_Port GPIOE
#define dir_6_Pin GPIO_PIN_15
#define dir_6_GPIO_Port GPIOE
#define Ls1_Pin GPIO_PIN_0
#define Ls1_GPIO_Port GPIOE
#define Ls2_Pin GPIO_PIN_1
#define Ls2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
