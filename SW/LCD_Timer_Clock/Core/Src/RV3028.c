/**
 ******************************************************************************
 * @file           RV3028.c
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: Aug 14, 2022
 * Author: 	marius
 */

#include <RV3028.h>

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

uint8_t ALARM_MODE_RTC;

uint8_t ALARM_WDA_State;
uint8_t ALARM_OTA_State;

// TODO INIT RTC
void RTC_Setup(RV3028 *myRTC, I2C_HandleTypeDef *I2C_Handle,
		GPIO_TypeDef *INT_PORT, uint16_t INT_PIN) {
	/* Store I2C Handle */
	myRTC->I2C_Handle = I2C_Handle;

	/* Set I2C Address */
	myRTC->I2C_ADDRESS = RV3028_ADDR;

	/* Set Interrupt pin port */
	myRTC->Interrupt_PORT = INT_PORT;

	/* Set Interrupt pin port */
	myRTC->Interrupt_PIN = INT_PIN;

	/* Initialize  variables */

	// Set default alarm mode to inactive
	ALARM_MODE_RTC = ALARM_MODE_INACTIVE;
	ALARM_WDA_State = 0;
	ALARM_OTA_State = 0;

	// TODO load alarm times from EEPROM

}

// TODO get unix time
void RTC_Get_Time(RV3028 *myRTC) {

	// receive buffer
	uint8_t rx_buf[31];

	// read all important registers sequentially
	HAL_I2C_Mem_Read(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_SECONDS, 1, &rx_buf[0], 31, HAL_MAX_DELAY);


	// extract the data:

	// get time
	RTC_Second = BCD_TO_unit8(rx_buf[RTC_REG_SECONDS]);
	RTC_Minute = BCD_TO_unit8(rx_buf[RTC_REG_MINUTES]);
	RTC_Hour = BCD_TO_unit8(rx_buf[RTC_REG_HOURS]);

	// get date
	RTC_Day = BCD_TO_unit8(rx_buf[RTC_REG_DATE]);
	RTC_Month = BCD_TO_unit8(rx_buf[RTC_REG_MONTH]);
	RTC_Year = 2000 + BCD_TO_unit8(rx_buf[RTC_REG_YEAR]);

	// get UNIX time

	RTC_UNIX_TIME = (rx_buf[RTC_REG_UNIX_TIME_3] << 24)
					| (rx_buf[RTC_REG_UNIX_TIME_2] << 16)
					| (rx_buf[RTC_REG_UNIX_TIME_1] << 8)
					| (rx_buf[RTC_REG_UNIX_TIME_0]);

	// get weekday alarm
	WDA_Minute = BCD_TO_unit8(rx_buf[RTC_REG_ALARM_MINUTES]);
	WDA_Hour = BCD_TO_unit8(rx_buf[RTC_REG_ALARM_HOURS]);
}

uint32_t get_RTC_UNIX_TIME(RV3028 *myRTC) {
	return RTC_UNIX_TIME;
}


uint8_t BCD_TO_unit8(uint8_t BCD_value) {
	uint8_t result = 0;

	// resolve BCD coding:
	((BCD_value & 0b00000001) == 0) ? (result += 0) : (result += 1);
	((BCD_value & 0b00000010) == 0) ? (result += 0) : (result += 2);
	((BCD_value & 0b00000100) == 0) ? (result += 0) : (result += 4);
	((BCD_value & 0b00001000) == 0) ? (result += 0) : (result += 8);
	((BCD_value & 0b00010000) == 0) ? (result += 0) : (result += 10);
	((BCD_value & 0b00100000) == 0) ? (result += 0) : (result += 20);
	((BCD_value & 0b01000000) == 0) ? (result += 0) : (result += 40);

	return result;
}
