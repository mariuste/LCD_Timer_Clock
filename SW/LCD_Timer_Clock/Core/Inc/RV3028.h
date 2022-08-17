/**
 ******************************************************************************
 * @file           RV3028.h
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: Aug 14, 2022
 * Author: 	marius
 */
#ifndef INC_RV3028_H_
#define INC_RV3028_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32G0:
#include "stm32g0xx_hal.h"

// I2C Address
static const uint8_t RV3028_ADDR = 0xA4; // Use 8-bit address

/*
 * Registers
 */
#define RTC_REG_SECONDS 0x00
#define RTC_REG_MINUTES 0x01
#define RTC_REG_HOURS 0x02
#define RTC_REG_WEEKDAY 0x03
#define RTC_REG_DATE 0x04
#define RTC_REG_MONTH 0x05
#define RTC_REG_YEAR 0x06
#define RTC_REG_ALARM_MINUTES 0x07
#define RTC_REG_ALARM_HOURS 0x08
#define RTC_REG_ALARM_DATE 0x09
#define RTC_REG_TIMER_VALUE_0 0x0A
#define RTC_REG_TIMER_VALUE_1 0x0B
#define RTC_REG_TIMER_STATUS_0 0x0C
#define RTC_REG_TIMER_STATUS_1 0x0D
#define RTC_REG_STATUS 0x0E
#define RTC_REG_CONTROL1 0x0F
#define RTC_REG_CONTROL2 0x10
#define RTC_REG_GP_BITS 0x11
#define RTC_REG_CLOCK_INT_MASK 0x12
#define RTC_REG_EVENT_CONTROL 0x13
#define RTC_REG_TS_COUNT 0x14
#define RTC_REG_TS_SECONDS 0x15
#define RTC_REG_TS_MINUTES 0x16
#define RTC_REG_RS_HOURS 0x17
#define RTC_REG_TS_DATE 0x18
#define RTC_REG_TS_MONTH 0x19
#define RTC_REG_TS_YEAR 0x1A
#define RTC_REG_UNIX_TIME_0 0x1B
#define RTC_REG_UNIX_TIME_1 0x1C
#define RTC_REG_UNIX_TIME_2 0x1D
#define RTC_REG_UNIX_TIME_3 0x1E
#define RTC_REG_USER_RAM_1 0x1F
#define RTC_REG_USER_RAM_2 0x20
#define RTC_REG_PASSWORD_0 0x21
#define RTC_REG_PASSWORD_1 0x22
#define RTC_REG_PASSWORD_2 0x23
#define RTC_REG_PASSWORD_3 0x24
#define RTC_REG_EE_ADDRESS 0x25
#define RTC_REG_EE_DATA 0x26
#define RTC_REG_EE_COMMAND 0x27
#define RTC_REG_ID 0x28

// RTC variables
uint8_t RTC_Second;
uint8_t RTC_Minute;
uint8_t RTC_Hour;

uint8_t RTC_Day;
uint8_t RTC_Month;
uint8_t RTC_Year;

uint32_t RTC_UNIX_TIME;

// Alarm variables + constants

uint8_t WDA_Minute;
uint8_t WDA_Hour;

uint8_t ALARM_MODE;

#define ALARM_MODE_INACTIVE 0
#define ALARM_MODE_WORKINGDAYS 1
#define ALARM_MODE_ONETIME 2
#define ALARM_MODE_WORKINGDAYS_AND_ONETIME 3


/**
 * @struct HMI
 * @brief Structure for RV3028-C7 RTC
 *
 */
typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
	GPIO_TypeDef *Interrupt_PORT;	/**< GPIO Port of Interrupt pin */
	uint16_t Interrupt_PIN;			/**< GPIO Pin of Interrupt pin */
} RV3028;



/*
 * General Functions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

// TODO INIT RTC
void RTC_Setup(
		RV3028 *myRTC,
		I2C_HandleTypeDef *I2C_Handle,
		GPIO_TypeDef *INT_PORT,
		uint16_t INT_PIN
);

// TODO get unix time
void RTC_Get_Time(
		RV3028 *myRTC
);

// TODO convert a BCD value to a regular integer
uint8_t BCD_TO_unit8(uint8_t BCD_value);

#endif /* INC_RV3028_H_ */
