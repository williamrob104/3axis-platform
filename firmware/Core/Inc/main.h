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

#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

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
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define MOTOR_EN_Pin LL_GPIO_PIN_12
#define MOTOR_EN_GPIO_Port GPIOB
#define ZERO_Z_Pin LL_GPIO_PIN_13
#define ZERO_Z_GPIO_Port GPIOB
#define ZERO_Y_Pin LL_GPIO_PIN_14
#define ZERO_Y_GPIO_Port GPIOB
#define ZERO_X_Pin LL_GPIO_PIN_15
#define ZERO_X_GPIO_Port GPIOB
#define PROBE_PULSE_Pin LL_GPIO_PIN_8
#define PROBE_PULSE_GPIO_Port GPIOA
#define PROBE_RESET_Pin LL_GPIO_PIN_9
#define PROBE_RESET_GPIO_Port GPIOA
#define DIR_Z_Pin LL_GPIO_PIN_10
#define DIR_Z_GPIO_Port GPIOA
#define STEP_Z_Pin LL_GPIO_PIN_15
#define STEP_Z_GPIO_Port GPIOA
#define DIR_Y_Pin LL_GPIO_PIN_3
#define DIR_Y_GPIO_Port GPIOB
#define STEP_Y_Pin LL_GPIO_PIN_4
#define STEP_Y_GPIO_Port GPIOB
#define DIR_X_Pin LL_GPIO_PIN_5
#define DIR_X_GPIO_Port GPIOB
#define STEP_X_Pin LL_GPIO_PIN_6
#define STEP_X_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define TIM_X TIM4
#define TIM_Y TIM3
#define TIM_Z TIM2
#define PROBE_PULSE_CHANNEL LL_TIM_CHANNEL_CH1
#define PROBE_RESET_CHANNEL LL_TIM_CHANNEL_CH2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
