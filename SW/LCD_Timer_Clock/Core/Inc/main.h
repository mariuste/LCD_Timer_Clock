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
#define STATE_WDA_SET_HOUR 601
#define STATE_WDA_SET_MINUTE 602
#define STATE_WDA_SET_SAVE 603
#define STATE_OTA_SHOW 7
#define STATE_OTA_TOGGLE 8
#define STATE_OTA_SET 9
#define STATE_OTA_SET_HOUR 901
#define STATE_OTA_SET_MINUTE 902
#define STATE_OTA_SET_SAVE 903
#define STATE_TIME_DATE_SHOW 10
#define STATE_TIME_DATE_SET 11
#define STATE_TIME_DATE_SET_YEAR 1101
#define STATE_TIME_DATE_SET_DAY 1102
#define STATE_TIME_DATE_SET_MONTH 1103
#define STATE_TIME_DATE_SET_HOUR 1104
#define STATE_TIME_DATE_SET_MINUTE 1105
#define STATE_TIME_DATE_SET_SAVE 1106

#define STATE_TIMER1 12
#define STATE_TIMER1_SHOW 1201
#define STATE_TIMER1_SET 1202
#define STATE_TIMER1_SET_RUN 1204


#define STATE_TEMPLATE 99


#define EEPROM_WDA_HOUR_ADDR 1
#define EEPROM_WDA_MINUTE_ADDR 2
#define EEPROM_OTA_HOUR_ADDR 3
#define EEPROM_OTA_MINUTE_ADDR 4
#define EEPROM_TIMER1_ADDR 5
#define EEPROM_TIMER1_SECOND_ADDR 6
#define EEPROM_TIMER2_MINUTE_ADDR 7
#define EEPROM_TIMER2_SECOND_ADDR 8

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
