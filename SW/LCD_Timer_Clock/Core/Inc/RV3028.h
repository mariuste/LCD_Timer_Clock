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

#define ALARM_MODE_INACTIVE 0
#define ALARM_MODE_ACTIVE 1

#define ALARM_STATE_STANDBY 0
#define ALARM_STATE_RUNNING 1
#define ALARM_STATE_PRE_ALARM 2
#define ALARM_STATE_ALARM 3
#define ALARM_STATE_ALARM_SKIPPED 4

#define ALARM_PRE_ALARM_TIME 60 // 1200 seconds of pre-alarm
#define ALARM_AUDIO_RAMP 30 // time over which the audio level is increased

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
// TODO convert a integer value to a BCD value
uint8_t uint8_TO_BCD(uint8_t uint8_value);

// Getter
uint32_t get_RTC_UNIX_TIME(RV3028 *myRTC);
uint8_t get_RTC_Minute(RV3028 *myRTC);
uint8_t get_RTC_Hour(RV3028 *myRTC);
uint8_t get_RTC_Day(RV3028 *myRTC);
uint8_t get_RTC_Month(RV3028 *myRTC);
uint8_t get_RTC_Year(RV3028 *myRTC);

uint8_t get_WDA_Minute(RV3028 *myRTC);
uint8_t get_WDA_Hour(RV3028 *myRTC);
uint8_t get_WDA_State(RV3028 *myRTC);
float get_WDA_preAlarm_time (RV3028 *myRTC);
float get_WDA_Alarm_time (RV3028 *myRTC);
uint8_t get_ALARM_WDA_Mode(RV3028 *myRTC);

uint8_t get_OTA_Minute(RV3028 *myRTC);
uint8_t get_OTA_Hour(RV3028 *myRTC);
uint8_t get_OTA_State(RV3028 *myRTC);
float get_OTA_preAlarm_time (RV3028 *myRTC);
float get_OTA_Alarm_time (RV3028 *myRTC);
uint8_t get_ALARM_OTA_Mode(RV3028 *myRTC);

uint8_t get_TIMER1_State_Running(RV3028 *myRTC);
uint8_t get_TIMER1_RemainingTime_Minutes(RV3028 *myRTC);
uint8_t get_TIMER1_RemainingTime_Seconds(RV3028 *myRTC);

// Setter
void set_ALARM_WDA_Mode(RV3028 *myRTC, uint8_t AlarmState);
void set_ALARM_OTA_Mode(RV3028 *myRTC, uint8_t AlarmState);
void set_WDA_Minute(RV3028 *myRTC, uint8_t SET_WDA_MINUTE);
void set_WDA_Hour(RV3028 *myRTC, uint8_t SET_WDA_HOUR);
void set_WDA_ALARM_STOP(RV3028 *myRTC);
void set_WDA_ALARM_SKIP(RV3028 *myRTC);
void set_OTA_Minute(RV3028 *myRTC, uint8_t SET_OTA_MINUTE);
void set_OTA_Hour(RV3028 *myRTC, uint8_t SET_OTA_HOUR);

void set_RTC_Hour(RV3028 *myRTC, uint8_t hour);
void set_RTC_Minute(RV3028 *myRTC, uint8_t minute);
void set_RTC_Second(RV3028 *myRTC, uint8_t second);
void set_RTC_Year(RV3028 *myRTC, uint8_t year);
void set_RTC_Month(RV3028 *myRTC, uint8_t month);
void set_RTC_Day(RV3028 *myRTC, uint8_t day);
void set_TIMER1_State_Running(RV3028 *myRTC, uint8_t State);
void set_TIMER1_Minute(RV3028 *myRTC, uint8_t minute);
void set_TIMER1_Second(RV3028 *myRTC, uint8_t second);
void set_TIMER1_START(RV3028 *myRTC);
void set_TIMER1_ALARM_STOP(RV3028 *myRTC);

#endif /* INC_RV3028_H_ */
