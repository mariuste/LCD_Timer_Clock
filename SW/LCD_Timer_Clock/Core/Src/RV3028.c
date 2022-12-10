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
uint8_t RTC_Weekday;

uint32_t RTC_UNIX_TIME; // UNIX time
uint16_t RTC_UNIX_TIME_S; // current time in special unix time

// Alarm variables + constants
uint8_t WDA_Minute;
uint8_t WDA_Hour;
uint16_t WDA_Time_UNIX_S; // WDA time in special unix time

uint8_t OTA_Minute;
uint8_t OTA_Hour;
uint16_t OTA_Time_UNIX_S; // OTA time in special unix time

uint8_t ALARM_MODE_RTC;

uint8_t ALARM_WDA_Mode; // is alarm on or off
uint8_t ALARM_WDA_State; // is alarm in standby, running or ringing

uint8_t ALARM_OTA_Mode;
uint8_t ALARM_OTA_State;

uint8_t TIMER1_Minute;
uint8_t TIMER1_Second;
uint8_t TIMER1_State_Running;
uint32_t TIMER1_EndTime;
// uint8_t TIMER1_RemainingTime;


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
	ALARM_WDA_Mode = ALARM_MODE_INACTIVE;
	ALARM_WDA_State = ALARM_STATE_STANDBY;

	ALARM_OTA_Mode = ALARM_MODE_INACTIVE;
	ALARM_OTA_State = ALARM_STATE_STANDBY;

	TIMER1_State_Running = ALARM_STATE_STANDBY;

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

	// seconds since start of the day
	RTC_UNIX_TIME_S = RTC_Hour * 3600 + RTC_Minute * 60 + RTC_Second;

	// get date
	RTC_Day = BCD_TO_unit8(rx_buf[RTC_REG_DATE]);
	RTC_Month = BCD_TO_unit8(rx_buf[RTC_REG_MONTH]);
	RTC_Year = BCD_TO_unit8(rx_buf[RTC_REG_YEAR]);
	RTC_Weekday = BCD_TO_unit8(rx_buf[RTC_REG_WEEKDAY]);

	// get UNIX time

	RTC_UNIX_TIME = (rx_buf[RTC_REG_UNIX_TIME_3] << 24)
					| (rx_buf[RTC_REG_UNIX_TIME_2] << 16)
					| (rx_buf[RTC_REG_UNIX_TIME_1] << 8)
					| (rx_buf[RTC_REG_UNIX_TIME_0]);

	// check status of TIMER1
	if(
			(TIMER1_State_Running == ALARM_STATE_RUNNING) &&
			(get_TIMER1_RemainingTime_Minutes(myRTC) == 0) &&
			(get_TIMER1_RemainingTime_Seconds(myRTC) == 0)
	) {
		TIMER1_State_Running = ALARM_STATE_ALARM;
	}
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

uint8_t get_RTC_Hour(RV3028 *myRTC) {
	return RTC_Hour;
}

uint8_t get_RTC_Day(RV3028 *myRTC) {
	return RTC_Day;
}

uint8_t get_RTC_Month(RV3028 *myRTC) {
	return RTC_Month;
}

uint8_t get_RTC_Year(RV3028 *myRTC) {
	return RTC_Year;
}

uint8_t get_RTC_Weekday(RV3028 *myRTC) {
	return RTC_Weekday;
}

uint8_t get_WDA_Minute(RV3028 *myRTC) {
	return WDA_Minute;
}

uint8_t get_WDA_Hour(RV3028 *myRTC) {
	return WDA_Hour;
}

uint8_t get_WDA_State(RV3028 *myRTC){
	// TODO move to RTC_Get_Time
	// when the alarm is inactive the alarm is off
	if (ALARM_WDA_Mode == ALARM_MODE_INACTIVE) {
		// alarm is not active, return inactive alarm state
		ALARM_WDA_State = ALARM_STATE_STANDBY;
		return ALARM_WDA_State;
	}

	// when the alarm already went off do not change it
	if (ALARM_WDA_State == ALARM_STATE_ALARM) {
		return ALARM_WDA_State;
	}

	// when the pre-alarm went off and the alarm time is reached
	if (
			(ALARM_WDA_State == ALARM_STATE_PRE_ALARM) &&
			(RTC_UNIX_TIME_S >= WDA_Time_UNIX_S) )
	{
		ALARM_WDA_State = ALARM_STATE_ALARM;
		return ALARM_WDA_State;
	}

	// when the pre alarm went off and was manually stopped,
	// and the alarm time is reached, convert sate back to active
	if (
			(ALARM_WDA_State == ALARM_STATE_ALARM_SKIPPED) &&
			(RTC_UNIX_TIME_S >= WDA_Time_UNIX_S) )
	{
		ALARM_WDA_State = ALARM_STATE_STANDBY;
		return ALARM_WDA_State;
	}

	// triggering pre alarm on week days
	//TODO unit8_t dayIsWeekDay;
	//if (RTC_Weekday)

	if (
			(ALARM_WDA_State != ALARM_STATE_ALARM_SKIPPED) &&
			(RTC_UNIX_TIME_S < WDA_Time_UNIX_S) &&
			(RTC_UNIX_TIME_S >= WDA_Time_UNIX_S - ALARM_PRE_ALARM_TIME)) {
		ALARM_WDA_State = ALARM_STATE_PRE_ALARM;
		return ALARM_WDA_State;
	}

	return ALARM_WDA_State;
}

float get_WDA_preAlarm_time (RV3028 *myRTC) {
	// only return value when in pre alarm
	if(ALARM_WDA_State == ALARM_STATE_PRE_ALARM) {


		// number of seconds since the pre alarm started
		uint16_t seconds_since_preAlarm =
				ALARM_PRE_ALARM_TIME - (WDA_Time_UNIX_S - RTC_UNIX_TIME_S);

		// number between 0 and 1 to indicate how much time progressed of the pre alarm
		float progress = (float)seconds_since_preAlarm / (float)ALARM_PRE_ALARM_TIME;

		// output result
		return progress;
	}
	return 0;
}

float get_WDA_Alarm_time (RV3028 *myRTC) {
	// only return value when in alarm
	if(ALARM_WDA_State == ALARM_STATE_ALARM) {

		if(RTC_UNIX_TIME_S > (WDA_Time_UNIX_S + ALARM_AUDIO_RAMP)){
			// ramp finished, audio level at 100%
			return 1.0;
		} else {
			// number of seconds since the alarm started
			uint16_t seconds_since_Alarm = RTC_UNIX_TIME_S - WDA_Time_UNIX_S;

			// number between 0 and 1 to indicate how much time progressed of the audio ramp
			float progress = (float)seconds_since_Alarm / (float)ALARM_AUDIO_RAMP;

			// output result
			return progress;
		}
	}
	return 0.0;
}

uint8_t get_ALARM_WDA_Mode(RV3028 *myRTC) {
	return ALARM_WDA_Mode;
}


uint8_t get_OTA_Minute(RV3028 *myRTC) {
	return OTA_Minute;
}

uint8_t get_OTA_Hour(RV3028 *myRTC) {
	return OTA_Hour;
}

uint8_t get_OTA_State(RV3028 *myRTC){
	// TODO move to RTC_Get_Time
	// when the alarm is inactive the alarm is off
	if (ALARM_OTA_Mode == ALARM_MODE_INACTIVE) {
		// alarm is not active, return inactive alarm state
		ALARM_OTA_State = ALARM_STATE_STANDBY;
		return ALARM_OTA_State;
	}

	// when the alarm already went off do not change it
	if (ALARM_OTA_State == ALARM_STATE_ALARM) {
		return ALARM_OTA_State;
	}

	// when the pre-alarm went off and the alarm time is reached
	if (
			(ALARM_OTA_State == ALARM_STATE_PRE_ALARM) &&
			(RTC_UNIX_TIME_S >= OTA_Time_UNIX_S) )
	{
		ALARM_OTA_State = ALARM_STATE_ALARM;
		return ALARM_OTA_State;
	}

	// when the pre alarm went off and was manually stopped,
	// and the alarm time is reached, convert sate back to active
	// -- not used for OTA, only for WDA --
	/*if (
			(ALARM_OTA_State == ALARM_STATE_ALARM_SKIPPED) &&
			(RTC_UNIX_TIME_S >= OTA_Time_UNIX_S) )
	{
		ALARM_OTA_State = ALARM_STATE_STANDBY;
		return ALARM_OTA_State;
	}*/

	// triggering pre alarm
	if (
			(ALARM_OTA_State != ALARM_STATE_ALARM_SKIPPED) &&
			(RTC_UNIX_TIME_S < OTA_Time_UNIX_S) &&
			(RTC_UNIX_TIME_S >= OTA_Time_UNIX_S - ALARM_PRE_ALARM_TIME)) {
		ALARM_OTA_State = ALARM_STATE_PRE_ALARM;
		return ALARM_OTA_State;
	}

	return ALARM_OTA_State;
}

float get_OTA_preAlarm_time (RV3028 *myRTC) {
	// only return value when in pre alarm
	if(ALARM_OTA_State == ALARM_STATE_PRE_ALARM) {


		// number of seconds since the pre alarm started
		uint16_t seconds_since_preAlarm =
				ALARM_PRE_ALARM_TIME - (OTA_Time_UNIX_S - RTC_UNIX_TIME_S);

		// number between 0 and 1 to indicate how much time progressed of the pre alarm
		float progress = (float)seconds_since_preAlarm / (float)ALARM_PRE_ALARM_TIME;

		// output result
		return progress;
	}
	return 0;
}

float get_OTA_Alarm_time (RV3028 *myRTC) {
	// only return value when in alarm
	if(ALARM_OTA_State == ALARM_STATE_ALARM) {

		if(RTC_UNIX_TIME_S > (OTA_Time_UNIX_S + ALARM_AUDIO_RAMP)){
			// ramp finished, audio level at 100%
			return 1.0;
		} else {
			// number of seconds since the alarm started
			uint16_t seconds_since_Alarm = RTC_UNIX_TIME_S - OTA_Time_UNIX_S;

			// number between 0 and 1 to indicate how much time progressed of the audio ramp
			float progress = (float)seconds_since_Alarm / (float)ALARM_AUDIO_RAMP;

			// output result
			return progress;
		}
	}
	return 0.0;
}

uint8_t get_ALARM_OTA_Mode(RV3028 *myRTC) {
	return ALARM_OTA_Mode;
}


uint8_t get_TIMER1_State_Running(RV3028 *myRTC) {
	return TIMER1_State_Running;
}

uint8_t get_TIMER1_RemainingTime_Minutes(RV3028 *myRTC) {
	if(RTC_UNIX_TIME > TIMER1_EndTime) {
		// timer ended, return 0
		return 0;
	} else {
		// calculate remaining seconds
		uint32_t temp_remaining_timer = TIMER1_EndTime - RTC_UNIX_TIME;
		// extract minutes
		return (temp_remaining_timer / 60);
	}
}

uint8_t get_TIMER1_RemainingTime_Seconds(RV3028 *myRTC) {
	if(RTC_UNIX_TIME > TIMER1_EndTime) {
		// timer ended, return 0
		return 0;
	} else {
		// calculate remaining seconds
		uint32_t temp_remaining_timer = TIMER1_EndTime - RTC_UNIX_TIME;
		// extract seconds
		return (temp_remaining_timer % 60);
	}
}

// setter +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


void set_WDA_Minute(RV3028 *myRTC, uint8_t SET_WDA_MINUTE) {
	WDA_Minute = SET_WDA_MINUTE;
	// update WDA time
	WDA_Time_UNIX_S = WDA_Hour * 3600 + WDA_Minute * 60;
}

void set_WDA_Hour(RV3028 *myRTC, uint8_t SET_WDA_HOUR) {
	WDA_Hour = SET_WDA_HOUR;
	// update WDA time
	WDA_Time_UNIX_S = WDA_Hour * 3600 + WDA_Minute * 60;
}

void set_ALARM_WDA_Mode(RV3028 *myRTC, uint8_t AlarmState){
	ALARM_WDA_Mode = AlarmState;
}

void set_WDA_ALARM_SKIP(RV3028 *myRTC){
	// skip this alarm but keep it active
	ALARM_WDA_State = ALARM_STATE_ALARM_SKIPPED;
}

void set_WDA_ALARM_STOP(RV3028 *myRTC){
	// stop currently active alarm
	ALARM_WDA_State = ALARM_STATE_STANDBY;
}


void set_OTA_Minute(RV3028 *myRTC, uint8_t SET_OTA_MINUTE) {
	OTA_Minute = SET_OTA_MINUTE;
	// update OTA time
	OTA_Time_UNIX_S = OTA_Hour * 3600 + OTA_Minute * 60;
}

void set_OTA_Hour(RV3028 *myRTC, uint8_t SET_OTA_HOUR) {
	OTA_Hour = SET_OTA_HOUR;
	// update OTA time
	OTA_Time_UNIX_S = OTA_Hour * 3600 + OTA_Minute * 60;
}

void set_ALARM_OTA_Mode(RV3028 *myRTC, uint8_t AlarmState){
	ALARM_OTA_Mode = AlarmState;
}

void set_OTA_ALARM_SKIP(RV3028 *myRTC){
	// skip this alarm but keep it active
	ALARM_OTA_State = ALARM_STATE_ALARM_SKIPPED;
}

void set_OTA_ALARM_STOP(RV3028 *myRTC){
	// stop currently active alarm
	ALARM_OTA_State = ALARM_STATE_STANDBY;
	// This is only a one time alarm, set it to inactive
	ALARM_OTA_Mode = ALARM_MODE_INACTIVE;
}


void set_TIMER1_Second(RV3028 *myRTC, uint8_t second) {
	// store new value locally
	TIMER1_Second = second;
}

void set_TIMER1_Minute(RV3028 *myRTC, uint8_t minute) {
	// store new value locally
	TIMER1_Minute = minute;
}

void set_TIMER1_State_Running(RV3028 *myRTC, uint8_t State) {
	TIMER1_State_Running = State;
}

void set_TIMER1_START(RV3028 *myRTC) {
	// Set end time
	TIMER1_EndTime = RTC_UNIX_TIME + (TIMER1_Minute * 60) + TIMER1_Second;

	// start timer
	TIMER1_State_Running = ALARM_STATE_RUNNING;
}

void set_TIMER1_ALARM_STOP(RV3028 *myRTC) {
	// stop timer
	TIMER1_State_Running = ALARM_STATE_STANDBY;
}
