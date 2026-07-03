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
#include "stm32wlxx_hal.h"

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
#define RTC_PREDIV_A ((1<<(15-RTC_N_PREDIV_S))-1)
#define RTC_PREDIV_S ((1<<RTC_N_PREDIV_S)-1)
#define RTC_N_PREDIV_S 10
#define VCC_5V_EN_Pin GPIO_PIN_3
#define VCC_5V_EN_GPIO_Port GPIOB
#define ADC_EN_Pin GPIO_PIN_5
#define ADC_EN_GPIO_Port GPIOB
#define RF_TXEN_Pin GPIO_PIN_6
#define RF_TXEN_GPIO_Port GPIOA
#define RF_RXEN_Pin GPIO_PIN_7
#define RF_RXEN_GPIO_Port GPIOA
#define GNSSEN_Pin GPIO_PIN_8
#define GNSSEN_GPIO_Port GPIOA
#define BTN_OK_Pin GPIO_PIN_2
#define BTN_OK_GPIO_Port GPIOB
#define BTN_OK_EXTI_IRQn EXTI2_IRQn
#define BTN_UP_Pin GPIO_PIN_12
#define BTN_UP_GPIO_Port GPIOB
#define BTN_UP_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BTN_DOWN_Pin GPIO_PIN_15
#define BTN_DOWN_GPIO_Port GPIOA
#define BTN_DOWN_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
