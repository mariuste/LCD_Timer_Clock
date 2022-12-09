/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BL55072A.h"	// LCD control
#include "HMI.h"		// HMI (LEDs and Buttons)
#include "RV3028.h" 	// RTC
#include "AT34C04.h"	// EEPROM

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Sate Machine ###############################################################
int currentState = STATE_INITIALISATION;
int nextState = STATE_INITIALISATION;

// Set timeout time
uint8_t TIMEOUT_SHORT = 1;
uint8_t TIMEOUT_MEDIUM = 2;
uint8_t TIMEOUT_LONG = 4;
uint8_t TIMEOUT_EXTRA_LONG = 30;
uint8_t TIMEOUT_ALARM = 180;

// counts loops for blinking segments
uint8_t loop_counter;
// blink intervals
uint8_t blink_slow_interval = 5;
uint8_t blink_fast_interval = 2;

// Toggling blink signals
uint8_t blink_signal_fast = 1;
uint8_t blink_signal_slow = 1;
// TODO create seconds blink pattern with rtc

uint8_t override_blink = 0;

// UNIX time stamp
uint32_t LastEvent = 0;

// Define external ICs ########################################################
// Human Machine Interface (buttons, led) -------------------
HMI myHMI;

// counter to detect long press of buttons
uint8_t HMI_BTN_ENCODER_LONG_COUNTER = 0;
uint8_t HMI_BTN_WDA_LONG_COUNTER = 0;
uint8_t HMI_BTN_OTA_LONG_COUNTER = 0;
uint8_t HMI_BTN_TIME_DATE_LONG_COUNTER = 0;
uint8_t HMI_BTN_TIMER1_LONG_COUNTER = 0;
uint8_t HMI_BTN_TIMER2_LONG_COUNTER = 0;

// counter to detect amount of edges of buttons
uint8_t HMI_BTN_WDA_FALLING_EDGE_COUNTER = 0;
uint8_t HMI_BTN_WDA_LAST_STATE = BUTTON_NOT_PRESSED;
uint8_t HMI_BTN_OTA_FALLING_EDGE_COUNTER = 0;
uint8_t HMI_BTN_OTA_LAST_STATE = BUTTON_NOT_PRESSED;
uint8_t HMI_BTN_TIMER1_FALLING_EDGE_COUNTER = 0;
uint8_t HMI_BTN_TIMER1_LAST_STATE = BUTTON_NOT_PRESSED;
uint8_t HMI_BTN_TIMER2_FALLING_EDGE_COUNTER = 0;
uint8_t HMI_BTN_TIMER2_LAST_STATE = BUTTON_NOT_PRESSED;

// LOCK buttons after press
uint8_t HMI_BTN_ENCODER_LOCK = 0;
uint8_t HMI_BTN_WDA_LOCK = 0;
uint8_t HMI_BTN_OTA_LOCK = 0;
uint8_t HMI_BTN_TIME_DATE_LOCK = 0;
uint8_t HMI_BTN_TIMER1_LOCK = 0;
uint8_t HMI_BTN_TIMER2_LOCK = 0;

// brightness of Lamp
float LAMP_brightness_setting = PWM_CH_LAMP_MIN;
float LAMP_brightness_current_level = 0;
float LAMP_brightness_ALARM = 5;
// state of Lamp
uint8_t LAMP_state = 0;

// default brightness
uint8_t brightness_backlight_default = 150;
uint8_t brightness_keypad_default = 100;
uint8_t brightness_LCD_backlight = 100;
uint8_t brightness_keypad = 0;

// Encoder position as temporary storage across states
float encoder_pos = 0;

// LCD interface --------------------------------------------
LCD myLCD;

/*// DFPlayer Data Packets:
static const uint8_t DFP_START = 0x7E;
static const uint8_t DFP_VER = 0xFF;
static const uint8_t DFP_LEN = 0x06;
static const uint8_t DFP_noFB = 0x00;
static const uint8_t DFP_STOP = 0xEF;*/

// RTC RV-3028 --------------------------------------------
RV3028 myRTC;

// for setting alarms and times
float TEMP_TIME_HOUR = 5;
float TEMP_TIME_MINUTE = 0;
float TEMP_TIME_SECONDS = 0;
float TEMP_TIMER_INDEX = 1;

float TEMP_DATE_YEAR = 22;
float TEMP_DATE_MONTH = 01;
float TEMP_DATE_DAY = 01;



// EEPROM -------------------------------------------------
AT34C04 myAT34C04;

// Analog intput -------------------------------------------------
// TODO maybe out source to HMI.h
// conversion Constant
const float VBAT_const = (3.0 / 4096) * 2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/*HAL_StatusTypeDef DFP_Send_CMD(uint8_t cmd, uint8_t payload1, uint8_t payload0) {
	// calculate CRC
	uint16_t DFT_CRC = 0x00;
	DFT_CRC = DFT_CRC - DFP_VER - DFP_LEN - cmd - DFP_noFB - payload1
			- payload0;
	// assemble transmission buffer
	uint8_t UART_buf[10] = { DFP_START, DFP_VER, DFP_LEN, cmd, DFP_noFB,
			payload1, payload0, DFT_CRC >> 8, DFT_CRC, DFP_STOP }; // Perform Reset

	// transmit packet
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_Reset() {
	uint8_t UART_buf[10] = { 0x7E, 0xFF, 0x06, 0x0C, 0x00, 0x00, 0x00, 0xFE,
			0xEF, 0xEF }; // Perform Reset
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_SetToSD() {
	uint8_t UART_buf[10] = { 0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE,
			0xF0, 0xEF }; // Specify micro SD
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_setVolume() {
	// for now fixed to 25%
	uint8_t UART_buf[10] = { 0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE,
			0xF0, 0xEF }; // Specify micro SD
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}*/

float my_roundl(float value) {
	// round down the easy way:
	int tempValue = value;
	float returnValue = tempValue * 1.0;
	return returnValue;
}

// State Machine States ####################################################### //
void ENTER_STATE_INITIALISATION() {
	// state newly entered; reset event timeout timer
	LastEvent = get_RTC_UNIX_TIME(&myRTC);

	// start loop counter
	loop_counter = 0;

	// load stored alarm times from EEPROM
	uint8_t hour_buffer = 0;
	uint8_t minute_buffer = 0;
	uint8_t timer_index_buffer = 0;

	// load WDA alarm times from EEPROM
	AT34C04_Read_VReg_unit8(&myAT34C04, EEPROM_WDA_HOUR_ADDR, &hour_buffer);
	AT34C04_Read_VReg_unit8(&myAT34C04, EEPROM_WDA_MINUTE_ADDR, &minute_buffer);
	// store in RTC
	set_WDA_Hour(&myRTC, hour_buffer);
	set_WDA_Minute(&myRTC, minute_buffer);

	// load OTA alarm times from EEPROM
	AT34C04_Read_VReg_unit8(&myAT34C04, EEPROM_OTA_HOUR_ADDR, &hour_buffer);
	AT34C04_Read_VReg_unit8(&myAT34C04, EEPROM_OTA_MINUTE_ADDR, &minute_buffer);
	// store in RTC
	set_OTA_Hour(&myRTC, hour_buffer);
	set_OTA_Minute(&myRTC, minute_buffer);

	// load TIMER 1 values from EEPROM
	AT34C04_Read_VReg_unit8(&myAT34C04, EEPROM_TIMER1_ADDR, &timer_index_buffer);
	// store locally
	TEMP_TIMER_INDEX = timer_index_buffer;


	// next state:
	nextState = STATE_STANDBY;

}

void ENTER_STATE_STANDBY(){
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// disable LCD background illumination
		brightness_LCD_backlight = 0;

		// disable Keypad Background illumination
		brightness_keypad = 0;

		// deactivate indicator LEDs
		HMI_reset_all_LED_b(&myHMI);
		HMI_Write(&myHMI);
		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------


	// display current time
	LCD_Write_Number(&myLCD, LCD_LEFT, get_RTC_Hour(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_RTC_Minute(&myRTC), 2);

	// blink colon roughly every 500 ms /TODO add seconds blink
	LCD_Write_Colon(&myLCD, blink_signal_slow);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// show TIMER1 status
	if (get_TIMER1_State_Running(&myRTC) == ALARM_STATE_RUNNING) {
		HMI_Write_LED_b(&myHMI, HMI_LED_TIMER1, 1);
		HMI_Write(&myHMI);
	} else {
		HMI_Write_LED_b(&myHMI, HMI_LED_TIMER1, 0);
		HMI_Write(&myHMI);
	}

	// C: conditions for changing the state ---------------------------

	// check for interrupts at HMI, but let the next state deal with it
	if (HMI_Read_Interrupt(&myHMI, HMI_BTN_ANY) == INTERRUPT) {
		// when any button is pressed, go to illuminated state
		nextState = STATE_STANDBY_LIGHT;
	}

	// check if encoder was turned
	encoder_pos = HMI_Encoder_position(&myHMI);

	if (encoder_pos != 0) {
		// encoder was moved
		nextState = STATE_STANDBY_LIGHT;
	}

	// D: timeout conditions ------------------------------------------

	// none, this is the default state
}

void ENTER_STATE_STANDBY_LIGHT() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// reset loop counter
		loop_counter = 0;

		// One time setup finished
		currentState = nextState;
	} else {
		// only called when this is not a state change

	}

	// B: Normal operations of the state ------------------------------

	// display current time
	LCD_Write_Number(&myLCD, LCD_LEFT, get_RTC_Hour(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_RTC_Minute(&myRTC), 2);

	// blink colon roughly every 500 ms /TODO add seconds blink
	LCD_Write_Colon(&myLCD, blink_signal_slow);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set Alarm LEDs
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_WDA, get_ALARM_WDA_State(&myRTC));
	HMI_Write_LED_b(&myHMI, HMI_LED_OTA, get_ALARM_OTA_State(&myRTC));
	// display Timer states
	if(get_TIMER1_State_Running(&myRTC) == ALARM_STATE_RUNNING) {
		HMI_Write_LED_b(&myHMI, HMI_LED_TIMER1, 1);
	}

	HMI_Write(&myHMI);

	// enable LCD Background illumination
	brightness_LCD_backlight = brightness_backlight_default;

	// enable Keypad Background illumination
	brightness_keypad = brightness_keypad_default;

	// if any button was pressed, reset the timeout timer
	if (HMI_Read_Interrupt(&myHMI, HMI_BTN_ANY) != 0x0000) {
		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);
	}

	// adjust brightness of Lamp
	if (LAMP_state == 1) {
		// lamp is on, adjust the brightness

		// check if encoder was turned
		int encoder_pos_temp = HMI_Encoder_position(&myHMI);
		if (encoder_pos_temp != 0) {
			// encoder was moved; adjust the brightness
			encoder_pos += encoder_pos_temp;

			// set brightness; /2 because of double steps of encoder
			LAMP_brightness_setting += (encoder_pos/2) * PWM_CH_LAMP_MANUAL_STEP;

			// ensure limits
			if (LAMP_brightness_setting < PWM_CH_LAMP_MIN) {
				LAMP_brightness_setting = PWM_CH_LAMP_MIN;
			}
			if (LAMP_brightness_setting > PWM_CH_LAMP_MAX) {
				LAMP_brightness_setting = PWM_CH_LAMP_MAX;
			}

			// reset encoder
			encoder_pos = 0;

			// reset event timeout timer
			LastEvent = get_RTC_UNIX_TIME(&myRTC);
		}

	} else {
		// lamp is off but movement of the encoder resets the timeout
		if (HMI_Encoder_position(&myHMI) != 0) {
			// reset event timeout timer
			LastEvent = get_RTC_UNIX_TIME(&myRTC);
		}
	}

	// C: conditions for changing the state ---------------------------

	// check if WDA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {

		// switch to STATE_WDA_SHOW
		nextState = STATE_WDA_SHOW;
	}

	// check if OTA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) {

		// switch to STATE_OTA_SHOW
		nextState = STATE_OTA_SHOW;
	}

	// check if Timer Date button is currently pressed
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) && (HMI_BTN_TIME_DATE_LOCK == 0) ) {

		// switch to STATE_TIME_DATE_SHOW
		nextState = STATE_TIME_DATE_SHOW;
	}

	// check if Timer1 button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) {

		// switch to STATE_TIMER1
		nextState = STATE_TIMER1;
	}


	// check if encoder button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) {
		//prevent timeout
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// increment encoder button counter if unlocked
		if (HMI_BTN_ENCODER_LOCK == 0) {
			HMI_BTN_ENCODER_LONG_COUNTER += 1;
		}
	}
	// if the threshold for a longpress is reached, set the new state and lock the encoder button
	if (HMI_BTN_ENCODER_LONG_COUNTER >= HMI_LONG_PRESS_THRESHOLD) {
		// reset the counter to prevent loop
		HMI_BTN_ENCODER_LONG_COUNTER = 0;

		// when one of the alarms is in pre-alarm, end the alarm
		// otherwise toggle the main lamp
		if(get_WDA_State(&myRTC) == ALARM_STATE_PRE_ALARM) {
			// skip this alarm but keep it active in general
			set_WDA_ALARM_SKIP(&myRTC);
		} else {
			// toggle LAMP in next state
			nextState = STATE_TOGGLE_LAMP;
		}
		// lock the encoder button
		HMI_BTN_ENCODER_LOCK = 1;
	}


	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY;
	}
}

void ENTER_STATE_TOGGLE_LAMP() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// start loop counter
		loop_counter = 0;

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// Toggle Lamp
	(LAMP_state) ? (LAMP_state = 0) : (LAMP_state = 1);

	// C: conditions for changing the state ---------------------------

	// none

	// D: timeout conditions ------------------------------------------

	// Immediate timeout, return to standby

	nextState = STATE_STANDBY_LIGHT;
}

void ENTER_STATE_WDA_SHOW() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// reset WDA button edge counter
		HMI_BTN_WDA_FALLING_EDGE_COUNTER = 0;
		// get current button state
		HMI_BTN_WDA_LAST_STATE = HMI_Read_BTN(&myHMI, HMI_BTN_WDA);

		// One time setup finished
		currentState = nextState;
	}

	// increment loop counter
	loop_counter += 1;
	if (loop_counter >= 10) {
		loop_counter = 0;
	}

	// B: Normal operations of the state ------------------------------
	// display week day alarm
	LCD_Write_Number(&myLCD, LCD_LEFT, get_WDA_Hour(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_WDA_Minute(&myRTC), 2);

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set Alarm LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_WDA, get_ALARM_WDA_State(&myRTC));
	HMI_Write(&myHMI);

	// C: conditions for changing the state ---------------------------

	// check if WDA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {
		// prevent timeout
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// increment WDA button
		HMI_BTN_WDA_LONG_COUNTER += 1;

	}


	// if the threshold for a longpress is reached, enter the next state
	if (HMI_BTN_WDA_LONG_COUNTER >= HMI_LONG_PRESS_THRESHOLD) {

		// enter next state
		nextState = STATE_WDA_SET;

		// reset long press counter
		HMI_BTN_WDA_LONG_COUNTER = 0;

		// lock WDA button
		HMI_BTN_WDA_LOCK = 1;
	}

	// if the threshold for a short press is reached, enter the next state (double press)
	uint8_t current_WDA_state = HMI_Read_BTN(&myHMI, HMI_BTN_WDA);

	// increase count when button was high and now is low
	if ((current_WDA_state == BUTTON_NOT_PRESSED) & (HMI_BTN_WDA_LAST_STATE == BUTTON_PRESSED)) {
		HMI_BTN_WDA_FALLING_EDGE_COUNTER += 1;
	}

	// double press detected, enter next state
	if (HMI_BTN_WDA_FALLING_EDGE_COUNTER >= 2) {

		// enter next state
		nextState = STATE_WDA_TOGGLE;
	}

	// update last button state
	HMI_BTN_WDA_LAST_STATE = current_WDA_state;

	// check if OTA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) {

		// switch to STATE_OTA_SHOW
		nextState = STATE_OTA_SHOW;
	}

	// check if Timer Date button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// display time
		nextState = STATE_STANDBY_LIGHT;
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_SHORT) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_WDA_TOGGLE(){
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// toggle the WDA alarm
	if(get_ALARM_WDA_State(&myRTC) == 0) {
		set_ALARM_WDA_Mode(&myRTC, ALARM_MODE_ACTIVE);
	} else if (get_ALARM_WDA_State(&myRTC) == 1) {
		set_ALARM_WDA_Mode(&myRTC, ALARM_MODE_INACTIVE);
	}

	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// instant timeout


	//return to main state
	nextState = STATE_WDA_SHOW;
}

void ENTER_STATE_WDA_SET() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// get current alarm time from RTC
		TEMP_TIME_HOUR = get_WDA_Hour(&myRTC);
		TEMP_TIME_MINUTE = get_WDA_Minute(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// noting to do here, procede to setting hour


	// C: conditions for changing the state ---------------------------

	// none

	// D: timeout conditions ------------------------------------------

	// Immediate timeout, return to standby

	nextState = STATE_WDA_SET_HOUR;
}

void ENTER_STATE_WDA_SET_HOUR() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed time
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIME_HOUR += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIME_HOUR < 0) {
			TEMP_TIME_HOUR = 23;
		}
		if (TEMP_TIME_HOUR > 23) {
			TEMP_TIME_HOUR = 0;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// display alarm time

	// blink hour value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_LEFT, DIGIT_EMPTY, 1);
	}
	// show minutes
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 2);

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink WDA LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_WDA, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// Time/Date button -> abort setting WDA and return to standby
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// WDA button -> confirm hour setting and continue with with setting minutes
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) && (HMI_BTN_WDA_LOCK == 0)) {

		// continue with setting minutes
		nextState = STATE_WDA_SET_MINUTE;

		// lock encoder button to prevent glitch
		HMI_BTN_WDA_LOCK = 1;
	}

	// Encoder button -> confirm hour setting and continue with with setting minutes
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with setting minutes
		nextState = STATE_WDA_SET_MINUTE;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_WDA_SET_MINUTE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed time
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIME_MINUTE += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIME_MINUTE < 0) {
			TEMP_TIME_MINUTE = 59;
		}
		if (TEMP_TIME_MINUTE > 59) {
			TEMP_TIME_MINUTE = 0;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// display alarm time
	// show hours
	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 2);

	// blink hour value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_RIGHT, DIGIT_EMPTY, 1);
	}

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink WDA LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_WDA, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// Time/Date button -> abort setting WDA and return to standby
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// WDA button -> confirm minute setting and continue with saving setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) && (HMI_BTN_WDA_LOCK == 0)) {

		// continue with setting minutes
		nextState = STATE_WDA_SET_SAVE;

		// lock encoder button to prevent glitch
		HMI_BTN_WDA_LOCK = 1;
	}

	// Encoder button -> confirm minute setting and continue with saving setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with saving setting
		nextState = STATE_WDA_SET_SAVE;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_WDA_SET_SAVE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// save WDA time locally
	set_WDA_Hour(&myRTC, TEMP_TIME_HOUR);
	set_WDA_Minute(&myRTC, TEMP_TIME_MINUTE);

	// save WDA time to EEPROM
	uint8_t temp_buffer_hour = TEMP_TIME_HOUR;
	uint8_t temp_buffer_minute = TEMP_TIME_MINUTE;
	// save hour to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_WDA_HOUR_ADDR, &temp_buffer_hour);
	// save minute to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_WDA_MINUTE_ADDR, &temp_buffer_minute);

	// display time
	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 2);
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 2);
	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink WDA LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_WDA, blink_signal_slow);
	HMI_Write(&myHMI);

	// blink background illumination
	brightness_LCD_backlight = brightness_backlight_default * blink_signal_slow;
	brightness_keypad = brightness_keypad_default * blink_signal_slow;


	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_SHORT) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}

}

void ENTER_STATE_OTA_SHOW() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// reset OTA button edge counter
		HMI_BTN_OTA_FALLING_EDGE_COUNTER = 0;
		// get current button state
		HMI_BTN_OTA_LAST_STATE = HMI_Read_BTN(&myHMI, HMI_BTN_OTA);

		// One time setup finished
		currentState = nextState;
	}

	// increment loop counter
	loop_counter += 1;
	if (loop_counter >= 10) {
		loop_counter = 0;
	}

	// B: Normal operations of the state ------------------------------
	// display week day alarm
	LCD_Write_Number(&myLCD, LCD_LEFT, get_OTA_Hour(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_OTA_Minute(&myRTC), 2);

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set Alarm LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_OTA, get_ALARM_OTA_State(&myRTC));
	HMI_Write(&myHMI);

	// C: conditions for changing the state ---------------------------

	// check if OTA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) {
		// prevent timeout
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// increment OTA button
		HMI_BTN_OTA_LONG_COUNTER += 1;

	}


	// if the threshold for a longpress is reached, enter the next state
	if (HMI_BTN_OTA_LONG_COUNTER >= HMI_LONG_PRESS_THRESHOLD) {

		// enter next state
		nextState = STATE_OTA_SET;

		// reset long press counter
		HMI_BTN_OTA_LONG_COUNTER = 0;

		// lock OTA button
		HMI_BTN_OTA_LOCK = 1;
	}

	// if the threshold for a short press is reached, enter the next state (double press)
	uint8_t current_OTA_state = HMI_Read_BTN(&myHMI, HMI_BTN_OTA);

	// increase count when button was high and now is low
	if ((current_OTA_state == BUTTON_NOT_PRESSED) & (HMI_BTN_OTA_LAST_STATE == BUTTON_PRESSED)) {
		HMI_BTN_OTA_FALLING_EDGE_COUNTER += 1;
	}

	// double press detected, enter next state
	if (HMI_BTN_OTA_FALLING_EDGE_COUNTER >= 2) {

		// enter next state
		nextState = STATE_OTA_TOGGLE;
	}

	// update last button state
	HMI_BTN_OTA_LAST_STATE = current_OTA_state;

	// check if WDA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {

		// switch to STATE_WDA_SHOW
		nextState = STATE_WDA_SHOW;
	}

	// check if Timer Date button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// display time
		nextState = STATE_STANDBY_LIGHT;
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_SHORT) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_WDA_ALARM() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// increment loop counter
	loop_counter += 1;
	if (loop_counter >= 10) {
		loop_counter = 0;
	}

	// B: Normal operations of the state ------------------------------
	// display current time
	LCD_Write_Number(&myLCD, LCD_LEFT, get_RTC_Hour(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_RTC_Minute(&myRTC), 2);

	// blink colon roughly every 500 ms /TODO add seconds blink
	LCD_Write_Colon(&myLCD, blink_signal_slow);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set LEDs
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_WDA, blink_signal_slow);
	HMI_Write(&myHMI);

	// blink background illumination
	brightness_LCD_backlight = brightness_backlight_default * blink_signal_slow;
	brightness_keypad = brightness_keypad_default * blink_signal_slow;

	// C: conditions for changing the state ---------------------------

	// Encoder button -> end alarm
	if (HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) {

		// end alarm
		set_WDA_ALARM_STOP(&myRTC);

		// return to standby
		nextState = STATE_STANDBY_LIGHT;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// WDA button -> end alarm
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {

		// end alarm
		set_WDA_ALARM_STOP(&myRTC);

		// re-enable background lights
		brightness_LCD_backlight = brightness_backlight_default;
		brightness_keypad = brightness_keypad_default;

		// return to standby
		nextState = STATE_STANDBY_LIGHT;

		// lock encoder button to prevent glitch
		HMI_BTN_WDA_LOCK = 1;
	}


	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_ALARM) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_OTA_TOGGLE(){
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// toggle the OTA alarm
	if(get_ALARM_OTA_State(&myRTC) == 0) {
		set_ALARM_OTA_Mode(&myRTC, ALARM_MODE_ACTIVE);
	} else if (get_ALARM_OTA_State(&myRTC) == 1) {
		set_ALARM_OTA_Mode(&myRTC, ALARM_MODE_INACTIVE);
	}

	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// instant timeout


	//return to main state
	nextState = STATE_OTA_SHOW;
}

void ENTER_STATE_OTA_SET() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// get current alarm time from RTC
		TEMP_TIME_HOUR = get_OTA_Hour(&myRTC);
		TEMP_TIME_MINUTE = get_OTA_Minute(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// noting to do here, procede to setting hour


	// C: conditions for changing the state ---------------------------

	// none

	// D: timeout conditions ------------------------------------------

	// Immediate timeout, return to standby

	nextState = STATE_OTA_SET_HOUR;
}

void ENTER_STATE_OTA_SET_HOUR() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed time
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIME_HOUR += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIME_HOUR < 0) {
			TEMP_TIME_HOUR = 23;
		}
		if (TEMP_TIME_HOUR > 23) {
			TEMP_TIME_HOUR = 0;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// display alarm time

	// blink hour value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_LEFT, DIGIT_EMPTY, 1);
	}
	// show minutes
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 2);

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink OTA LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_OTA, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// Time/Date button -> abort setting OTA and return to standby
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// OTA button -> confirm hour setting and continue with with setting minutes
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) && (HMI_BTN_OTA_LOCK == 0)) {

		// continue with setting minutes
		nextState = STATE_OTA_SET_MINUTE;

		// lock encoder button to prevent glitch
		HMI_BTN_OTA_LOCK = 1;
	}

	// Encoder button -> confirm hour setting and continue with with setting minutes
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with setting minutes
		nextState = STATE_OTA_SET_MINUTE;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_OTA_SET_MINUTE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed time
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIME_MINUTE += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIME_MINUTE < 0) {
			TEMP_TIME_MINUTE = 59;
		}
		if (TEMP_TIME_MINUTE > 59) {
			TEMP_TIME_MINUTE = 0;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// display alarm time
	// show hours
	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 2);

	// blink hour value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_RIGHT, DIGIT_EMPTY, 1);
	}

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink OTA LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_OTA, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// Time/Date button -> abort setting OTA and return to standby
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// OTA button -> confirm minute setting and continue with saving setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) && (HMI_BTN_OTA_LOCK == 0)) {

		// continue with setting minutes
		nextState = STATE_OTA_SET_SAVE;

		// lock encoder button to prevent glitch
		HMI_BTN_OTA_LOCK = 1;
	}

	// Encoder button -> confirm minute setting and continue with saving setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with saving setting
		nextState = STATE_OTA_SET_SAVE;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_OTA_SET_SAVE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// save OTA time locally
	set_OTA_Hour(&myRTC, TEMP_TIME_HOUR);
	set_OTA_Minute(&myRTC, TEMP_TIME_MINUTE);

	// save OTA time to EEPROM
	uint8_t temp_buffer_hour = TEMP_TIME_HOUR;
	uint8_t temp_buffer_minute = TEMP_TIME_MINUTE;
	// save hour to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_OTA_HOUR_ADDR, &temp_buffer_hour);
	// save minute to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_OTA_MINUTE_ADDR, &temp_buffer_minute);

	// display time
	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 2);
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 2);
	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink OTA LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_OTA, blink_signal_slow);
	HMI_Write(&myHMI);
	// blink background illumination
	brightness_LCD_backlight = brightness_backlight_default * blink_signal_slow;
	brightness_keypad = brightness_keypad_default * blink_signal_slow;


	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_SHORT) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}

}

void ENTER_STATE_TIME_DATE_SHOW() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// increment loop counter
	loop_counter += 1;
	if (loop_counter >= 10) {
		loop_counter = 0;
	}

	// B: Normal operations of the state ------------------------------
	// display date
	LCD_Write_Number(&myLCD, LCD_LEFT, get_RTC_Month(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_RTC_Day(&myRTC), 2);

	// show Dot to indicate date
	LCD_Write_Dot(&myLCD, POSITION_DOT_DAY);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, 1);
	HMI_Write(&myHMI);

	// C: conditions for changing the state ---------------------------

	// check if TIME_DATE button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {
		// prevent timeout
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// increment TIME_DATE button
		HMI_BTN_TIME_DATE_LONG_COUNTER += 1;

	}


	// if the threshold for a longpress is reached, enter the next state
	if (HMI_BTN_TIME_DATE_LONG_COUNTER >= HMI_LONG_PRESS_THRESHOLD) {

		// enter next state
		nextState = STATE_TIME_DATE_SET;

		// reset long press counter
		HMI_BTN_TIME_DATE_LONG_COUNTER = 0;

		// lock TIME_DATE button
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// check if WDA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {

		// switch to STATE_WDA_SHOW
		nextState = STATE_WDA_SHOW;
	}

	// check if OTA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) {

		// switch to STATE_OTA_SHOW
		nextState = STATE_OTA_SHOW;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_SHORT) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_TIME_DATE_SET() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// get current alarm time from RTC
		TEMP_DATE_YEAR = get_RTC_Year(&myRTC);
		TEMP_DATE_MONTH = get_RTC_Month(&myRTC);
		TEMP_DATE_DAY = get_RTC_Day(&myRTC);

		TEMP_TIME_HOUR = get_RTC_Hour(&myRTC);
		TEMP_TIME_MINUTE = get_RTC_Minute(&myRTC);


		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// noting to do here, procede to setting hour


	// C: conditions for changing the state ---------------------------

	// none

	// D: timeout conditions ------------------------------------------

	// Immediate timeout, return to standby

	nextState = STATE_TIME_DATE_SET_YEAR;
}

void ENTER_STATE_TIME_DATE_SET_YEAR() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed date
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the date value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_DATE_YEAR += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_DATE_YEAR < 0) {
			TEMP_DATE_YEAR = 0;
		}
		if (TEMP_DATE_YEAR > 99) {
			TEMP_DATE_YEAR = 99;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// display year
	// show first part of the year
	LCD_Write_Number(&myLCD, LCD_LEFT, 20, 2);
	// blink the second part of the year value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_DATE_YEAR, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_RIGHT, DIGIT_EMPTY, 1);
	}


	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink TIME/DATE LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// abort setting and return to standby with Buttons WDA, OTA, Timer 1 and Timer 2:
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER2) == BUTTON_PRESSED) ) {
		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// Time/Date button button -> confirm year setting and continue with with day setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) && (HMI_BTN_TIME_DATE_LOCK == 0)) {

		// continue with setting day
		nextState = STATE_TIME_DATE_SET_DAY;

		// lock encoder button to prevent glitch
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// Encoder button -> confirm year setting and continue with with day setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with setting day
		nextState = STATE_TIME_DATE_SET_DAY;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_TIME_DATE_SET_DAY() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed date
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the date value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_DATE_DAY += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_DATE_DAY < 1) {
			TEMP_DATE_DAY = 1;
		}
		if (TEMP_DATE_DAY > 31) {
			TEMP_DATE_DAY = 31;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// show date

	// blink the set value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_DATE_DAY, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_LEFT, DIGIT_EMPTY, 1);
	}
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_DATE_MONTH, 2);

	// show Dot to indicate date
	LCD_Write_Dot(&myLCD, POSITION_DOT_DAY);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink TIME/DATE LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// abort setting and return to standby with Buttons WDA, OTA, Timer 1 and Timer 2:
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER2) == BUTTON_PRESSED) ) {
		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// Time/Date button button -> confirm day setting and continue with with month setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) && (HMI_BTN_TIME_DATE_LOCK == 0)) {

		// continue with setting month
		nextState = STATE_TIME_DATE_SET_MONTH;

		// lock encoder button to prevent glitch
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// Encoder button -> confirm day setting and continue with with month setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with setting month
		nextState = STATE_TIME_DATE_SET_MONTH;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_TIME_DATE_SET_MONTH() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed date
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the date value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_DATE_MONTH += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_DATE_MONTH < 1) {
			TEMP_DATE_MONTH = 1;
		}
		if (TEMP_DATE_MONTH > 12) {
			TEMP_DATE_MONTH = 12;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// show date

	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_DATE_DAY, 2);
	// blink the set value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_DATE_MONTH, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_RIGHT, DIGIT_EMPTY, 1);
	}


	// show Dot to indicate date
	LCD_Write_Dot(&myLCD, POSITION_DOT_DAY);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink TIME/DATE LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// abort setting and return to standby with Buttons WDA, OTA, Timer 1 and Timer 2:
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER2) == BUTTON_PRESSED) ) {
		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// Time/Date button button -> confirm month setting and continue with with hour setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) && (HMI_BTN_TIME_DATE_LOCK == 0)) {

		// continue with setting hour
		nextState = STATE_TIME_DATE_SET_HOUR;

		// lock encoder button to prevent glitch
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// Encoder button -> confirm month setting and continue with with hour setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with setting hour
		nextState = STATE_TIME_DATE_SET_HOUR;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_TIME_DATE_SET_HOUR() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed date
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIME_HOUR += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIME_HOUR < 0) {
			TEMP_TIME_HOUR = 23;
		}
		if (TEMP_TIME_HOUR > 23) {
			TEMP_TIME_HOUR = 0;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// show time


	// blink the set value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_LEFT, DIGIT_EMPTY, 1);
	}
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 2);

	// show colon to indicate time
	LCD_Write_Dot(&myLCD, POSITION_COLON);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink TIME/DATE LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// abort setting and return to standby with Buttons WDA, OTA, Timer 1 and Timer 2:
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER2) == BUTTON_PRESSED) ) {
		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// Time/Date button button -> confirm hour setting and continue with with minute setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) && (HMI_BTN_TIME_DATE_LOCK == 0)) {

		// continue with setting minute
		nextState = STATE_TIME_DATE_SET_MINUTE;

		// lock encoder button to prevent glitch
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// Encoder button -> confirm hour setting and continue with with minute setting
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with setting minute
		nextState = STATE_TIME_DATE_SET_MINUTE;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_TIME_DATE_SET_MINUTE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed date
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIME_MINUTE += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIME_MINUTE < 0) {
			TEMP_TIME_MINUTE = 59;
		}
		if (TEMP_TIME_MINUTE > 59) {
			TEMP_TIME_MINUTE = 0;
		}

		// reset encoder
		encoder_pos = 0;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;
	} else {
		// reset override blink
		override_blink = 0;
	}

	// show time

	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 2);
	// blink the set value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 1);
	} else {
		LCD_Write_Number(&myLCD, LCD_RIGHT, DIGIT_EMPTY, 1);
	}

	// show colon to indicate time
	LCD_Write_Dot(&myLCD, POSITION_COLON);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink TIME/DATE LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, blink_signal_slow);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// abort setting and return to standby with Buttons WDA, OTA, Timer 1 and Timer 2:
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) ||
			(HMI_Read_BTN(&myHMI, HMI_BTN_TIMER2) == BUTTON_PRESSED) ) {
		// escape setting alarm and return to standby state
		nextState = STATE_STANDBY_LIGHT;
	}

	// Time/Date button button -> confirm minute setting and continue with savinf time and date
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) && (HMI_BTN_TIME_DATE_LOCK == 0)) {

		// continue with saving time and date
		nextState = STATE_TIME_DATE_SET_SAVE;

		// lock encoder button to prevent glitch
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// Encoder button -> confirm minute setting and continue with savinf time and date
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with saving time and date
		nextState = STATE_TIME_DATE_SET_SAVE;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

void ENTER_STATE_TIME_DATE_SET_SAVE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// save Time and Date to RTC
	set_RTC_Hour(&myRTC, TEMP_TIME_HOUR);
	set_RTC_Minute(&myRTC, TEMP_TIME_MINUTE);
	set_RTC_Second(&myRTC, 0);
	set_RTC_Year(&myRTC, TEMP_DATE_YEAR);
	set_RTC_Month(&myRTC, TEMP_DATE_MONTH);
	set_RTC_Day(&myRTC, TEMP_DATE_DAY);

	// save Time and Date time to EEPROM
	// TODO: save time and date to EEPROM

	// display time
	LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_HOUR, 2);
	LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_MINUTE, 2);
	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink Time/Date LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIME_DATE, blink_signal_slow);
	HMI_Write(&myHMI);
	// blink background illumination
	brightness_LCD_backlight = brightness_backlight_default * blink_signal_slow;
	brightness_keypad = brightness_keypad_default * blink_signal_slow;


	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_SHORT) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}

}

void ENTER_STATE_TIMER1() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// one of three timer states:
	if (get_TIMER1_State_Running(&myRTC) == ALARM_STATE_STANDBY) {
		// setting TIMER1
		nextState = STATE_TIMER1_SET;
	} else if (get_TIMER1_State_Running(&myRTC) == ALARM_STATE_RUNNING) {
		// display running timer
		nextState = STATE_TIMER1_SHOW;
	}  else {
		// should never reach
		nextState = -1;
	} // note: alarm state is triggered by main()

	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// no timeout

}

void ENTER_STATE_TIMER1_SHOW() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// increment loop counter
	loop_counter += 1;
	if (loop_counter >= 10) {
		loop_counter = 0;
	}

	// B: Normal operations of the state ------------------------------
	// remaining time of TIMER1
	LCD_Write_Number(&myLCD, LCD_LEFT, get_TIMER1_RemainingTime_Minutes(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_TIMER1_RemainingTime_Seconds(&myRTC), 2);

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set LEDs
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIMER1, blink_signal_slow);
	HMI_Write(&myHMI);

	// C: conditions for changing the state ---------------------------

	// check if WDA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {

		// switch to STATE_WDA_SHOW
		nextState = STATE_WDA_SHOW;
	}

	// check if OTA button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) {

		// switch to STATE_OTA_SHOW
		nextState = STATE_OTA_SHOW;
	}

	// check if Timer Date button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// display time
		nextState = STATE_STANDBY_LIGHT;
		HMI_BTN_TIME_DATE_LOCK = 1;
	}
	// D: timeout conditions ------------------------------------------

	// TODO new state: same as this bus whout background illumination
}

void ENTER_STATE_TIMER1_SET() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// get encoder position and update displayed time
	// check if encoder was turned
	int encoder_pos_temp = HMI_Encoder_position(&myHMI);
	if (encoder_pos_temp != 0) {
		// encoder was moved; adjust the time value
		encoder_pos += encoder_pos_temp;

		// set value; /2 because of double steps of encoder
		TEMP_TIMER_INDEX += (encoder_pos/2);

		// ensure limits, make the selection cyclic
		if (TEMP_TIMER_INDEX < 1) {
			TEMP_TIMER_INDEX = 1;
		}
		if (TEMP_TIMER_INDEX > 74) {
			TEMP_TIMER_INDEX = 74;
		}
		// ensure that the latest value will be displayed when encoder was turned
		override_blink = 1;

		// reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);
	} else {
		// reset override blink
		override_blink = 0;
	}
	/* Translate TEMP_TIMER_INDEX into minutes and seconds; this is not linear for convenience:
	 * 0 to 11: in 5 Second steps (starting at TEMP_TIMER_INDEX = 1 -> 5 seconds)
	 * 12 to 35 in 10 Second steps
	 * 36 to 61 in 1 minute steps
	 * 46 to 56 in 5 minute steps
	 *
	 * This translates to a range of 00m05s to 95m00s
	 */
	if(TEMP_TIMER_INDEX <= 11) {
		// 5 second steps
		TEMP_TIME_MINUTE = 0;
		TEMP_TIME_SECONDS = my_roundl(TEMP_TIMER_INDEX) * 5;
	} else if (TEMP_TIMER_INDEX <= 35) {
		// 10 second steps
		int TotalSeconds = (TEMP_TIMER_INDEX - 12) * 10 + 60;

		TEMP_TIME_MINUTE = my_roundl(TotalSeconds / 60);
		TEMP_TIME_SECONDS = my_roundl(TotalSeconds % 60);
	} else if (TEMP_TIMER_INDEX <= 61) {
		// 1 minute steps
		int TotalSeconds = (TEMP_TIMER_INDEX - 36) * 60 + 300;

		TEMP_TIME_MINUTE = my_roundl(TotalSeconds / 60);
		TEMP_TIME_SECONDS = 0;
	} else if (TEMP_TIMER_INDEX <= 74) {
		// 5 minute steps
		int TotalSeconds = (TEMP_TIMER_INDEX - 62) * 300 + 2100;

		TEMP_TIME_MINUTE = my_roundl(TotalSeconds / 60);
		TEMP_TIME_SECONDS = 0;
	}

	// reset encoder
	encoder_pos = 0;

	// display alarm time

	// blink value roughly every 500 ms
	if ((blink_signal_slow == 1) | (override_blink == 1)) {
		LCD_Write_Number(&myLCD, LCD_LEFT, TEMP_TIME_MINUTE, 1); // minutes
		LCD_Write_Number(&myLCD, LCD_RIGHT, TEMP_TIME_SECONDS, 2); // seconds
	} else {
		LCD_Write_Number(&myLCD, LCD_LEFT, DIGIT_EMPTY, 1);
		LCD_Write_Number(&myLCD, LCD_RIGHT, DIGIT_EMPTY, 1);
	}


	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// blink TIMER1 LED
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIMER1, blink_signal_fast);
	HMI_Write(&myHMI);


	// C: conditions for changing the state ---------------------------

	// check if Timer Date button is currently pressed
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_PRESSED) {

		// display time
		nextState = STATE_STANDBY_LIGHT;
		HMI_BTN_TIME_DATE_LOCK = 1;
	}

	// WDA button -> abort setting TIMER1 and show WDA
	if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_PRESSED) {

		// escape setting alarm and go to setting WDA
		nextState = STATE_WDA_SHOW;
	}

	// OTA button -> abort setting TIMER1 and show OTA
	if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_PRESSED) {

		// escape setting alarm and go to setting OTA
		nextState = STATE_OTA_SHOW;
	}

	// TIMER1 button -> load preset QUICKSET1
	/*if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) && (HMI_BTN_TIMER1_LOCK == 0)) {

		// continue with starting timer
		nextState = TBD;

		// lock encoder button to prevent glitch
		HMI_BTN_TIMER1_LOCK = 1;
	}*/

	// TIMER2 button -> load preset QUICKSET2
	/*if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) && (HMI_BTN_TIMER1_LOCK == 0)) {

		// continue with starting timer
		nextState = TBD;

		// lock encoder button to prevent glitch
		HMI_BTN_TIMER1_LOCK = 1;
	}*/

	// Encoder button -> confirm minute setting and continue
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) && (HMI_BTN_ENCODER_LOCK == 0)) {

		// continue with starting timer
		nextState = STATE_TIMER1_SET_RUN;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// TIMER1 button -> confirm minute setting and continue
	if ((HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) && (HMI_BTN_TIMER1_LOCK == 0)) {

		// continue with starting timer
		nextState = STATE_TIMER1_SET_RUN;

		// lock encoder button to prevent glitch
		HMI_BTN_TIMER1_LOCK = 1;
	}

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_EXTRA_LONG) {
		// timeout reached

		//return to other state // TODO implement standby sate without illumination
		nextState = STATE_STANDBY_LIGHT;
	}

	// not timeout
}

void ENTER_STATE_TIMER1_SET_RUN() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// save TIMER1 time locally
	set_TIMER1_Minute(&myRTC, TEMP_TIME_MINUTE);
	set_TIMER1_Second(&myRTC, TEMP_TIME_SECONDS);

	// save TIMER1 time to EEPROM
	uint8_t temp_buffer_index = TEMP_TIMER_INDEX;
	// save index to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_TIMER1_ADDR, &temp_buffer_index);


	// Start TIMER1
	set_TIMER1_START(&myRTC);

	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// go back to decider
	nextState = STATE_TIMER1;

}

void ENTER_STATE_TIMER1_ALARM() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// increment loop counter
	loop_counter += 1;
	if (loop_counter >= 10) {
		loop_counter = 0;
	}

	// B: Normal operations of the state ------------------------------
	// remaining time of TIMER1
	LCD_Write_Number(&myLCD, LCD_LEFT, get_TIMER1_RemainingTime_Minutes(&myRTC), 1);
	LCD_Write_Number(&myLCD, LCD_RIGHT, get_TIMER1_RemainingTime_Seconds(&myRTC), 2);

	// show colon
	LCD_Write_Colon(&myLCD, 1);

	// Send LCD Buffer
	LCD_SendBuffer(&myLCD);

	// set LEDs
	HMI_reset_all_LED_b(&myHMI);
	HMI_Write_LED_b(&myHMI, HMI_LED_TIMER1, blink_signal_slow);
	HMI_Write(&myHMI);

	// blink Lamp brightness
	LAMP_brightness_current_level = LAMP_brightness_ALARM * blink_signal_slow;

	// blink background illumination
	brightness_LCD_backlight = brightness_backlight_default * blink_signal_slow;
	brightness_keypad = brightness_keypad_default * blink_signal_slow;

	// C: conditions for changing the state ---------------------------

	// Encoder button -> end alarm
	if (HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_PRESSED) {

		// end alarm
		set_TIMER1_ALARM_STOP(&myRTC);

		// continue with setting timer
		nextState = STATE_TIMER1;

		// lock encoder button to prevent glitch
		HMI_BTN_ENCODER_LOCK = 1;
	}

	// TIMER1 button -> end alarm
	if (HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_PRESSED) {

		// end alarm
		set_TIMER1_ALARM_STOP(&myRTC);

		// re-enable background lights
		brightness_LCD_backlight = brightness_backlight_default;
		brightness_keypad = brightness_keypad_default;

		// continue with setting timer
		nextState = STATE_TIMER1;

		// lock encoder button to prevent glitch
		HMI_BTN_TIMER1_LOCK = 1;
	}


	// D: timeout conditions ------------------------------------------

	// TODO new state: same as this bus whout background illumination
}

void ENTER_STATE_TEMPLATE() {
	// A: One time operations when a state is newly entered -----------
	if (nextState != currentState) {
		// state newly entered; reset event timeout timer
		LastEvent = get_RTC_UNIX_TIME(&myRTC);

		// One time setup finished
		currentState = nextState;
	}

	// B: Normal operations of the state ------------------------------

	// C: conditions for changing the state ---------------------------

	// D: timeout conditions ------------------------------------------

	// check timeout
	if (get_RTC_UNIX_TIME(&myRTC) > LastEvent + TIMEOUT_LONG) {
		// timeout reached

		//return to other state
		nextState = STATE_STANDBY_LIGHT;
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	// Setup periphery ##############################################
	// Initialize HMI (Port Expander, Encoder and DFPlayer
	HMI_Setup(&myHMI, 		// SX1503 object
			&hi2c2,				// I2C Handle
			nI_O_INT_GPIO_Port,	// Interrupt pin port
			nI_O_INT_Pin,		// Interrupt pin
			&htim1,
			&huart2,
			DFP_Audio_en_GPIO_Port,
			DFP_Audio_en_Pin);

	// SET Inputs and Outputs to the default configuration (reset)
	HMI_defaultConfig(&myHMI);

	// Setup LED Lights #############################################
	// set brightness to 0 before starting the PWM timers
	HMI_set_PWM(&myHMI, PWM_CH_Keypad, 0);
	HMI_set_PWM(&myHMI, PWM_CH_LCD, 0);
	HMI_set_PWM(&myHMI, PWM_CH_LAMP, 0);

	// start PWM Timers
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // PWM_CH_Keypad
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PWM_CH_LCD
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PWM_CH_LAMP

	// Setup Encoder ################################################
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// Setup RTC ####################################################
	// Initialize RTC RV3028
	RTC_Setup(&myRTC,	 		// RV3028 handle
			&hi2c2,				// I2C Handle
			RTC_INT_GPIO_Port,	// Interrupt pin port
			RTC_INT_Pin			// Interrupt pin
	);

	// Setup LCD ####################################################
	// Setup LCD object
	LCD_Setup(&myLCD, 	// SX1503 object
			&hi2c2		// I2C Handle
	);

	// Initialize LCD
	LCD_INIT(&myLCD);

	// Enable LCD
	LCD_Enable(&myLCD);

	// blink all segments
	LCD_Segment_AllOn(&myLCD);
	HAL_Delay(1000);
	LCD_Segment_normal(&myLCD);

	// Setup EEPROM #################################################
	// Initialize EEPROM
	AT34C04_Initialize(
			&myAT34C04, // EEPROM object
			0x0,		// Address pin A0 value
			0x0,		// Address pin A1 value
			0x0,		// Address pin A2 value
			&hi2c2		// I2C Handle
	);

	// Setup MP3 ####################################################
	// disable MP3 Player
	//HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, 0);
	DFP_Disable(&myHMI);

	// Test Player:

	DFP_Enable(&myHMI); // enable DFPlayer

	 HAL_Delay(2000); // wait for startup

	 DFP_Setup(&myHMI); // setup player

	 // play track 1 in folder mp3
	 DFP_Play(&myHMI, 1, DFP_MODE_NO_REPEAT);
	 //DFP_Play(&myHMI, 1, DFP_MODE_SINGLE_REPEAT);

	// Setup ADC ####################################################
	// load preprogrammed calibration values
	HAL_ADCEx_Calibration_Start(&hadc1);

	// DEBUG code
	/*
	// set time and date of RTC to 9:00:45 05.07.2022
	set_RTC_Day(&myRTC, 5);
	set_RTC_Month(&myRTC, 7);
	set_RTC_Year(&myRTC, 22);
	set_RTC_Hour(&myRTC, 9);
	set_RTC_Minute(&myRTC, 0);
	set_RTC_Second(&myRTC, 45);

	// WDA time for test purpose to 9:02

	TEMP_TIME_HOUR = 9;
	TEMP_TIME_MINUTE = 2;

	set_WDA_Hour(&myRTC, TEMP_TIME_HOUR);
	set_WDA_Minute(&myRTC, TEMP_TIME_MINUTE);
	// save WDA time to EEPROM
	uint8_t temp_buffer_hour = TEMP_TIME_HOUR;
	uint8_t temp_buffer_minute = TEMP_TIME_MINUTE;
	// save hour to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_WDA_HOUR_ADDR, &temp_buffer_hour);
	// save minute to EEPROM
	AT34C04_Write_VReg_unit8(&myAT34C04, EEPROM_WDA_MINUTE_ADDR, &temp_buffer_minute);

	// enable WDA alarm
	set_ALARM_WDA_Mode(&myRTC, ALARM_MODE_ACTIVE);*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
		// create blink pattern #####################################

		// cyclic counter
		loop_counter += 1;
		if (loop_counter >= 255) {
			loop_counter = 0;
		}
		// set blink pattern based on counter
		if((loop_counter % blink_fast_interval) == 0) {
			blink_signal_fast = !blink_signal_fast;
		}

		if((loop_counter % blink_slow_interval) == 0) {
			blink_signal_slow = !blink_signal_slow;
		}

		// read battery level #######################################
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		uint32_t raw_battery_value = HAL_ADC_GetValue(&hadc1);
		// in volt
		float battery_voltage = raw_battery_value * VBAT_const;

		// TODO add low battery action, e.g. display "BAT"

		// Set LEDs #################################################
		// Set Backlight LEDs
		HMI_set_PWM(&myHMI, PWM_CH_LCD, brightness_LCD_backlight);
		// Set Keypad LEDs
		HMI_set_PWM(&myHMI, PWM_CH_Keypad, brightness_keypad);

		// Set Lamp Level
		// from high priority to low priority
		if(get_TIMER1_State_Running(&myRTC) == ALARM_STATE_ALARM) {
			// don't overwrite Lamp setting
		} else if (
				(get_WDA_State(&myRTC) == ALARM_STATE_ALARM) ||
				(get_WDA_State(&myRTC) == ALARM_STATE_PRE_ALARM)) {
			// don't overwrite Lamp setting
		} else {
			// no alarms, set default
			LAMP_brightness_current_level = LAMP_brightness_setting * LAMP_state;
		}

		HMI_set_PWM(&myHMI, PWM_CH_LAMP, LAMP_brightness_current_level);


		// Read RTC #################################################
		RTC_Get_Time(&myRTC);

		// Read Button evens ########################################
		/*
		 * Get button states, but not when it is waking up from unlit standby
		 * - this preserves the button presses while waking up improving the UX
		 */
		if((currentState == STATE_STANDBY) && (nextState == STATE_STANDBY_LIGHT)) {
			// don't get new button presses
		} else {
			// get current button states
			HMI_Read_GPIOs(&myHMI);
		}

		// reset button locks after long press or state changes
		if (HMI_Read_BTN(&myHMI, HMI_BTN_ENCODER) == BUTTON_NOT_PRESSED) {
			HMI_BTN_ENCODER_LOCK = 0;
			HMI_BTN_ENCODER_LONG_COUNTER = 0;
		}
		if (HMI_Read_BTN(&myHMI, HMI_BTN_WDA) == BUTTON_NOT_PRESSED) {
			HMI_BTN_WDA_LOCK = 0;
			HMI_BTN_WDA_LONG_COUNTER = 0;
		}
		if (HMI_Read_BTN(&myHMI, HMI_BTN_OTA) == BUTTON_NOT_PRESSED) {
			HMI_BTN_OTA_LOCK = 0;
			HMI_BTN_OTA_LONG_COUNTER = 0;
		}
		if (HMI_Read_BTN(&myHMI, HMI_BTN_TIME_DATE) == BUTTON_NOT_PRESSED) {
			HMI_BTN_TIME_DATE_LOCK = 0;
			HMI_BTN_TIME_DATE_LONG_COUNTER = 0;
		}
		if (HMI_Read_BTN(&myHMI, HMI_BTN_TIMER1) == BUTTON_NOT_PRESSED) {
			HMI_BTN_TIMER1_LOCK = 0;
			HMI_BTN_TIMER1_LONG_COUNTER = 0;
		}
		if (HMI_Read_BTN(&myHMI, HMI_BTN_TIMER2) == BUTTON_NOT_PRESSED) {
			HMI_BTN_TIMER2_LOCK = 0;
			HMI_BTN_TIMER2_LONG_COUNTER = 0;
		}

		// Check on timer and alarms ################################
		// Check Timer 1
		if(get_TIMER1_State_Running(&myRTC) == ALARM_STATE_ALARM) {
			// Enter Timer 1 state
			nextState = STATE_TIMER1_ALARM;
		}

		// Check alarm state
		if(get_WDA_State(&myRTC) == ALARM_STATE_PRE_ALARM) {
			// Dimm LED Lamp
			LAMP_brightness_current_level = (uint16_t) (get_WDA_preAlarm_time(&myRTC) * (float)PWM_CH_LAMP_MAX);


			nextState = STATE_STANDBY_LIGHT; // TODO temp light up
			float temp_a = get_WDA_preAlarm_time(&myRTC);
		}
		if(get_WDA_State(&myRTC) == ALARM_STATE_ALARM) {
			// enter alarm state
			nextState = STATE_WDA_ALARM;
		}

		// State Machine ############################################
		switch (nextState) {

		case STATE_INITIALISATION:
			ENTER_STATE_INITIALISATION();
			break;

		case STATE_STANDBY:
			ENTER_STATE_STANDBY();
			break;

		case STATE_STANDBY_LIGHT:
			ENTER_STATE_STANDBY_LIGHT();
			break;

		case STATE_TOGGLE_LAMP:
			ENTER_STATE_TOGGLE_LAMP();
			break;

		case STATE_WDA_SHOW:
			ENTER_STATE_WDA_SHOW();
			break;

		case STATE_WDA_ALARM:
			ENTER_STATE_WDA_ALARM();
			break;

		case STATE_WDA_TOGGLE:
			ENTER_STATE_WDA_TOGGLE();
			break;

		case STATE_WDA_SET:
			ENTER_STATE_WDA_SET();
			break;

		case STATE_WDA_SET_HOUR:
			ENTER_STATE_WDA_SET_HOUR();
			break;

		case STATE_WDA_SET_MINUTE:
			ENTER_STATE_WDA_SET_MINUTE();
			break;

		case STATE_WDA_SET_SAVE:
			ENTER_STATE_WDA_SET_SAVE();
			break;

		case STATE_OTA_SHOW:
			ENTER_STATE_OTA_SHOW();
			break;

		case STATE_OTA_TOGGLE:
			ENTER_STATE_OTA_TOGGLE();
			break;

		case STATE_OTA_SET:
			ENTER_STATE_OTA_SET();
			break;

		case STATE_OTA_SET_HOUR:
			ENTER_STATE_OTA_SET_HOUR();
			break;

		case STATE_OTA_SET_MINUTE:
			ENTER_STATE_OTA_SET_MINUTE();
			break;

		case STATE_OTA_SET_SAVE:
			ENTER_STATE_OTA_SET_SAVE();
			break;

		case STATE_TIME_DATE_SHOW:
			ENTER_STATE_TIME_DATE_SHOW();
			break;

		case STATE_TIME_DATE_SET:
			ENTER_STATE_TIME_DATE_SET();
			break;

		case STATE_TIME_DATE_SET_YEAR:
			ENTER_STATE_TIME_DATE_SET_YEAR();
			break;

		case STATE_TIME_DATE_SET_DAY:
			ENTER_STATE_TIME_DATE_SET_DAY();
			break;

		case STATE_TIME_DATE_SET_MONTH:
			ENTER_STATE_TIME_DATE_SET_MONTH();
			break;

		case STATE_TIME_DATE_SET_HOUR:
			ENTER_STATE_TIME_DATE_SET_HOUR();
			break;

		case STATE_TIME_DATE_SET_MINUTE:
			ENTER_STATE_TIME_DATE_SET_MINUTE();
			break;

		case STATE_TIME_DATE_SET_SAVE:
			ENTER_STATE_TIME_DATE_SET_SAVE();
			break;

		case STATE_TIMER1:
			ENTER_STATE_TIMER1();
			break;

		case STATE_TIMER1_SHOW:
			ENTER_STATE_TIMER1_SHOW();
			break;

		case STATE_TIMER1_SET:
			ENTER_STATE_TIMER1_SET();
			break;

		case STATE_TIMER1_SET_RUN:
			ENTER_STATE_TIMER1_SET_RUN();
			break;

		case STATE_TIMER1_ALARM:
			ENTER_STATE_TIMER1_ALARM();
			break;

		case STATE_TEMPLATE:
			ENTER_STATE_TEMPLATE();
			break;

		default:
			// display invalid sate number
			LCD_Write_Number(&myLCD, LCD_LEFT, my_roundl(nextState/100), 1);
			LCD_Write_Number(&myLCD, LCD_RIGHT, nextState%100, 2);
			LCD_Write_Colon(&myLCD, 0);
			// Send LCD Buffer
			LCD_SendBuffer(&myLCD);
			// set LEDs
			HMI_set_all_LED_b(&myHMI);
			HMI_Write(&myHMI);
		}

		HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000509;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 400, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8192-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8192-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : nI_O_INT_Pin */
  GPIO_InitStruct.Pin = nI_O_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nI_O_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DFP_Audio_en_Pin */
  GPIO_InitStruct.Pin = DFP_Audio_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DFP_Audio_en_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_INT_Pin */
  GPIO_InitStruct.Pin = RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
