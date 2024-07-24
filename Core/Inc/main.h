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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Library/Inc/lcd_i2c.h>
#include <Library/Inc/invicto_motor.h>
#include <Library/Inc/odometry.h>
#include <Library/Inc/pid.h>
#include <Library/Inc/robot_control.h>

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
#define EA_2_Pin GPIO_PIN_0
#define EA_2_GPIO_Port GPIOC
#define EB_2_Pin GPIO_PIN_1
#define EB_2_GPIO_Port GPIOC
#define EB_2_EXTI_IRQn EXTI1_IRQn
#define EinB_1_Pin GPIO_PIN_2
#define EinB_1_GPIO_Port GPIOC
#define EinB_1_EXTI_IRQn EXTI2_IRQn
#define EinA_1_Pin GPIO_PIN_3
#define EinA_1_GPIO_Port GPIOC
#define EinB_2_Pin GPIO_PIN_0
#define EinB_2_GPIO_Port GPIOA
#define EinB_2_EXTI_IRQn EXTI0_IRQn
#define EinA_2_Pin GPIO_PIN_1
#define EinA_2_GPIO_Port GPIOA
#define Button_1_Pin GPIO_PIN_6
#define Button_1_GPIO_Port GPIOA
#define Button_1_EXTI_IRQn EXTI9_5_IRQn
#define Button_2_Pin GPIO_PIN_8
#define Button_2_GPIO_Port GPIOE
#define Button_2_EXTI_IRQn EXTI9_5_IRQn
#define Button_3_Pin GPIO_PIN_10
#define Button_3_GPIO_Port GPIOE
#define Button_3_EXTI_IRQn EXTI15_10_IRQn
#define Button_4_Pin GPIO_PIN_15
#define Button_4_GPIO_Port GPIOB
#define Button_4_EXTI_IRQn EXTI15_10_IRQn
#define EinA_3_Pin GPIO_PIN_12
#define EinA_3_GPIO_Port GPIOD
#define EinB_3_Pin GPIO_PIN_13
#define EinB_3_GPIO_Port GPIOD
#define EinB_3_EXTI_IRQn EXTI15_10_IRQn
#define EinA_4_Pin GPIO_PIN_10
#define EinA_4_GPIO_Port GPIOA
#define EinB_4_Pin GPIO_PIN_14
#define EinB_4_GPIO_Port GPIOA
#define EinB_4_EXTI_IRQn EXTI15_10_IRQn
#define EA_1_Pin GPIO_PIN_15
#define EA_1_GPIO_Port GPIOA
#define EB_1_Pin GPIO_PIN_3
#define EB_1_GPIO_Port GPIOB
#define EB_1_EXTI_IRQn EXTI3_IRQn
#define EA_3_Pin GPIO_PIN_4
#define EA_3_GPIO_Port GPIOB
#define EB_3_Pin GPIO_PIN_5
#define EB_3_GPIO_Port GPIOB
#define EB_3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define retry 50
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
