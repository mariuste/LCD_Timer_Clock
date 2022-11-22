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
uint8_t OTA_Minute;
uint8_t OTA_Hour;

uint8_t ALARM_MODE_RTC;

uint8_t ALARM_WDA_State;
uint8_t ALARM_OTA_State;

uint8_t TIMER1_Minute;
uint8_t TIMER1_Second;
uint8_t TIMER1_State_Running;

uint8_t TIMER2_Minute;
uint8_t TIMER2_Second;
uint8_t TIMER2_State_Running;

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
	TIMER1_State_Running = 0;
	TIMER2_State_Running = 0;

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
	RTC_Year = BCD_TO_unit8(rx_buf[RTC_REG_YEAR]);

	// get UNIX time

	RTC_UNIX_TIME = (rx_buf[RTC_REG_UNIX_TIME_3] << 24)
					| (rx_buf[RTC_REG_UNIX_TIME_2] << 16)
					| (rx_buf[RTC_REG_UNIX_TIME_1] << 8)
					| (rx_buf[RTC_REG_UNIX_TIME_0]);

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
	((BCD_value & 0b10000000) == 0) ? (result += 0) : (result += 80);

	return result;
}

uint8_t uint8_TO_BCD(uint8_t uint8_value) {
	uint8_t result = 0;
	if( uint8_value >= 80) {
		result |= 0b10000000;
		uint8_value -= 80;
	}
	if( uint8_value >= 40) {
		result |= 0b01000000;
		uint8_value -= 40;
	}
	if( uint8_value >= 20) {
		result |= 0b00100000;
		uint8_value -= 20;
	}
	if( uint8_value >= 10) {
		result |= 0b00010000;
		uint8_value -= 10;
	}
	if( uint8_value >= 8) {
		result |= 0b00001000;
		uint8_value -= 8;
	}
	if( uint8_value >= 4) {
		result |= 0b00000100;
		uint8_value -= 4;
	}
	if( uint8_value >= 2) {
		result |= 0b00000010;
		uint8_value -= 2;
	}
	if( uint8_value >= 1) {
		result |= 0b00000001;
		uint8_value -= 1;
	}

	return result;
}

// getter +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

uint32_t get_RTC_UNIX_TIME(RV3028 *myRTC) {
	return RTC_UNIX_TIME;
}
uint8_t get_RTC_Minute(RV3028 *myRTC) {
	return RTC_Minute;
}
uint8_t get_RTC_Year(RV3028 *myRTC) {
	return RTC_Year;
}
uint8_t get_RTC_Month(RV3028 *myRTC) {
	return RTC_Month;
}
uint8_t get_RTC_Day(RV3028 *myRTC) {
	return RTC_Day;
}
uint8_t get_RTC_Hour(RV3028 *myRTC) {
	return RTC_Hour;
}
uint8_t get_WDA_Minute(RV3028 *myRTC) {
	return WDA_Minute;
}
uint8_t get_WDA_Hour(RV3028 *myRTC) {
	return WDA_Hour;
}
uint8_t get_OTA_Minute(RV3028 *myRTC) {
	return OTA_Minute;
}
uint8_t get_OTA_Hour(RV3028 *myRTC) {
	return OTA_Hour;
}
uint8_t get_ALARM_WDA_State(RV3028 *myRTC) {
	return ALARM_WDA_State;
}
uint8_t get_ALARM_OTA_State(RV3028 *myRTC) {
	return ALARM_OTA_State;
}
uint8_t get_TIMER1_State_Running(RV3028 *myRTC) {
	return TIMER1_State_Running;
}
uint8_t get_TIMER2_State_Running(RV3028 *myRTC) {
	return TIMER2_State_Running;
}


// setter +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void set_ALARM_WDA_State(RV3028 *myRTC, uint8_t AlarmState){
	ALARM_WDA_State = AlarmState;
}
void set_ALARM_OTA_State(RV3028 *myRTC, uint8_t AlarmState){
	ALARM_OTA_State = AlarmState;
}
void set_WDA_Minute(RV3028 *myRTC, uint8_t SET_WDA_MINUTE) {
	WDA_Minute = SET_WDA_MINUTE;
}
void set_WDA_Hour(RV3028 *myRTC, uint8_t SET_WDA_HOUR) {
	WDA_Hour = SET_WDA_HOUR;
}
void set_OTA_Minute(RV3028 *myRTC, uint8_t SET_OTA_MINUTE) {
	OTA_Minute = SET_OTA_MINUTE;
}
void set_OTA_Hour(RV3028 *myRTC, uint8_t SET_OTA_HOUR) {
	OTA_Hour = SET_OTA_HOUR;
}

void set_RTC_Hour(RV3028 *myRTC, uint8_t hour) {
	// store new value locally
	RTC_Hour = hour;

	// send buffer
	uint8_t tx_buf[1];
	// convert value into BCD format
	tx_buf[0] = uint8_TO_BCD(RTC_Hour);

	// send value to RTC
	HAL_I2C_Mem_Write(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_HOURS, 1, &tx_buf[0], 1, HAL_MAX_DELAY);

}

void set_RTC_Minute(RV3028 *myRTC, uint8_t minute) {
	// store new value locally
	RTC_Minute = minute;

	// send buffer
	uint8_t tx_buf[1];
	// convert value into BCD format
	tx_buf[0] = uint8_TO_BCD(RTC_Minute);

	// send value to RTC
	HAL_I2C_Mem_Write(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_MINUTES, 1, &tx_buf[0], 1, HAL_MAX_DELAY);

}

void set_RTC_Second(RV3028 *myRTC, uint8_t second) {
	// TODO this function crashes the state machine for some reason

	// store new value locally
	RTC_Second = second;

	// send buffer
	uint8_t tx_buf[1];
	// convert value into BCD format
	tx_buf[0] = uint8_TO_BCD(RTC_Second);

	// send value to RTC
	HAL_I2C_Mem_Write(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_SECONDS, 1, &tx_buf[0], 1, HAL_MAX_DELAY);

}

void set_RTC_Year(RV3028 *myRTC, uint8_t year) {
	// store new value locally
	RTC_Year = year;

	// send buffer
	uint8_t tx_buf[1];
	// convert value into BCD format
	tx_buf[0] = uint8_TO_BCD(RTC_Year);

	// send value to RTC
	HAL_I2C_Mem_Write(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_YEAR, 1, &tx_buf[0], 1, HAL_MAX_DELAY);

}

void set_RTC_Month(RV3028 *myRTC, uint8_t month) {
	// store new value locally
	RTC_Month = month;

	// send buffer
	uint8_t tx_buf[1];
	// convert value into BCD format
	tx_buf[0] = uint8_TO_BCD(RTC_Month);

	// send value to RTC
	HAL_I2C_Mem_Write(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_MONTH, 1, &tx_buf[0], 1, HAL_MAX_DELAY);

}

void set_RTC_Day(RV3028 *myRTC, uint8_t day) {
	// store new value locally
	RTC_Day = day;

	// send buffer
	uint8_t tx_buf[1];
	// convert value into BCD format
	tx_buf[0] = uint8_TO_BCD(RTC_Day);

	// send value to RTC
	HAL_I2C_Mem_Write(myRTC->I2C_Handle, myRTC->I2C_ADDRESS,
			RTC_REG_DATE, 1, &tx_buf[0], 1, HAL_MAX_DELAY);

}

void set_TIMER1_State_Running(RV3028 *myRTC, uint8_t State_running) {
	TIMER1_State_Running = State_running;
}

void set_TIMER2_State_Running(RV3028 *myRTC, uint8_t State_running) {
	TIMER2_State_Running = State_running;
}

void set_TIMER1_Minute(RV3028 *myRTC, uint8_t minute) {
	// store new value locally
	TIMER1_Minute = minute;
}

void set_TIMER1_Second(RV3028 *myRTC, uint8_t second) {
	// store new value locally
	TIMER1_Second = second;
}

void set_TIMER2_Minute(RV3028 *myRTC, uint8_t minute) {
	// store new value locally
	TIMER2_Minute = minute;
}

void set_TIMER2_Second(RV3028 *myRTC, uint8_t second) {
	// store new value locally
	TIMER2_Second = second;
}
