/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define EEPROM_I2C_ADDRESS (0b1010000 << 1)
#define EEPROM_DATA_ADDRESS 0x0000
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LIGHT_SENSOR_D0_Pin GPIO_PIN_0
#define LIGHT_SENSOR_D0_GPIO_Port GPIOA
#define LIGHT_SENSOR_D0_EXTI_IRQn EXTI0_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define BUTTON_TOGGLE_DISPLAY_Pin GPIO_PIN_4
#define BUTTON_TOGGLE_DISPLAY_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LED_E_Pin GPIO_PIN_5
#define LED_E_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOB
#define LED_KEY_3_Pin GPIO_PIN_10
#define LED_KEY_3_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOB
#define LED_F_Pin GPIO_PIN_14
#define LED_F_GPIO_Port GPIOB
#define LED_A_Pin GPIO_PIN_15
#define LED_A_GPIO_Port GPIOB
#define LED_D_Pin GPIO_PIN_6
#define LED_D_GPIO_Port GPIOC
#define LED_C_Pin GPIO_PIN_8
#define LED_C_GPIO_Port GPIOC
#define LED_KEY_4_Pin GPIO_PIN_8
#define LED_KEY_4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED_KEY_2_Pin GPIO_PIN_4
#define LED_KEY_2_GPIO_Port GPIOB
#define LED_KEY_1_Pin GPIO_PIN_5
#define LED_KEY_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
