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
#include "stm32g0xx_hal.h"

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
#define nI_O_INT_Pin GPIO_PIN_7
#define nI_O_INT_GPIO_Port GPIOB
#define VBAT_2_Pin GPIO_PIN_0
#define VBAT_2_GPIO_Port GPIOA
#define LED_Keypad_PWM_Pin GPIO_PIN_1
#define LED_Keypad_PWM_GPIO_Port GPIOA
#define UART_MCU_2_DFP_Pin GPIO_PIN_2
#define UART_MCU_2_DFP_GPIO_Port GPIOA
#define UART_DFP_2_MCU_Pin GPIO_PIN_3
#define UART_DFP_2_MCU_GPIO_Port GPIOA
#define DFP_Audio_en_Pin GPIO_PIN_4
#define DFP_Audio_en_GPIO_Port GPIOA
#define RTC_INT_Pin GPIO_PIN_5
#define RTC_INT_GPIO_Port GPIOA
#define LCD_BG_PWM_Pin GPIO_PIN_6
#define LCD_BG_PWM_GPIO_Port GPIOA
#define LED_Light_PWM_Pin GPIO_PIN_7
#define LED_Light_PWM_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_PIN_11
#define I2C_SCL_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_12
#define I2C_SDA_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_3
#define ENC_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define STATE_INITIALISATION 0
#define STATE_STANDBY 1
#define STATE_STANDBY_LIGHT 2
#define STATE_TOGGLE_LAMP 3
#define STATE_WDA_SHOW 4
#define STATE_WDA_TOGGLE 5
#define STATE_WDA_SET 6

#define STATE_TEMPLATE 99


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
