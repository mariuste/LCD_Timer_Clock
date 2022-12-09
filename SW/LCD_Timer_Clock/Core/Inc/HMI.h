/**
 ******************************************************************************
 * @file           HMI.h
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: 12.08.2022
 * Author: 	marius
 */
#ifndef INC_HMI_H_
#define INC_HMI_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32G0:
#include "stm32g0xx_hal.h"


// Port Expander
static const uint8_t SX1503_ADDR = 0x20 << 1; // Use 8-bit address
static const uint8_t SX_1503_RegDataB = 0x00; // register address Pin State (0: LOW, 1: HIGH)
static const uint8_t SX_1503_RegDataA = 0x01; // register address Pin State
static const uint8_t SX_1503_RegDirB = 0x02; // register address Pin Direction (0: output, 1: Input)
static const uint8_t SX_1503_RegDirA = 0x03; // register address Pin Direction
static const uint8_t SX_1503_PullUpB = 0x04; // register address Pull Ups (0: pull up disabled, 1: pull up enabled)
static const uint8_t SX_1503_PullUpA = 0x05; // register address Pull Ups
static const uint8_t SX_1503_PullDownB = 0x06; // register address Pull Ups (0: pull up disabled, 1: pull up enabled)
static const uint8_t SX_1503_PullDownA = 0x07; // register address Pull Ups
static const uint8_t SX_1503_InterruptMaskB = 0x08; // register address Interrupt Mask (0: mask enabled, 1: mask disabled)
static const uint8_t SX_1503_InterruptMaskA = 0x09; // register address Interrupt Mask

static const uint8_t SX_1503_RegSenseHighB = 0x0A; // register address Interrupt Sense (00: none, 01: rising, 10: falling, 11: both)
static const uint8_t SX_1503_RegSenseHighA = 0x0B; // register address Interrupt Sense
static const uint8_t SX_1503_RegSenseLowB = 0x0C; // register address Interrupt Sense
static const uint8_t SX_1503_RegSenseLowA = 0x0D; // register address Interrupt Sense

static const uint8_t SX_1503_RegInterruptSourceB = 0x0E; // register address Interrupt Source (0: not the source of interrupt, 1: source of interrupt)
static const uint8_t SX_1503_RegInterruptSourceA = 0x0F; // register address Interrupt Source

static const uint8_t SX_1503_RegEventStatusB = 0x10; // register address Event status of all IOs (0: No event has occurred on this IO, 1: An event has occurred on this IO)
static const uint8_t SX_1503_RegEventStatusA = 0x11; // register address Event status ,,of all IOs

// Port Expander outputs:
static const uint16_t HMI_LED_WDA 		= 0b0000001000000000; // Week Day Alarm LED
static const uint16_t HMI_LED_OTA 		= 0b0000000100000000; // One Time Alarm LED
static const uint16_t HMI_LED_TIME_DATE = 0b0000000010000000; // Time/Date LED
static const uint16_t HMI_LED_TIMER1 	= 0b0000000001000000; // Timer1 LED
static const uint16_t HMI_LED_TIMER2 	= 0b0000000000100000; // Timer2 LED

#define HMI_LED_OFF 0
#define HMI_LED_ON 1

static const uint16_t HMI_BTN_ANY	 	= 0b0000010000011111; // Combined vector
static const uint16_t HMI_BTN_WDA 		= 0b0000000000010000; // Week Day Alarm Button
static const uint16_t HMI_BTN_OTA 		= 0b0000000000001000; // One Time Alarm Button
static const uint16_t HMI_BTN_TIME_DATE = 0b0000000000000100; // Time/Date Button
static const uint16_t HMI_BTN_TIMER1 	= 0b0000000000000010; // Timer1 Button
static const uint16_t HMI_BTN_TIMER2 	= 0b0000000000000001; // Timer2 Button
static const uint16_t HMI_BTN_ENCODER 	= 0b0000010000000000; // Encoder Button

// The buttons button states
#define BUTTON_PRESSED 1
#define BUTTON_NOT_PRESSED 0
// values for detected interrupt
#define NO_INTERRUPT 0
#define INTERRUPT 1

static const uint8_t HMI_LONG_PRESS_THRESHOLD = 15;

#define PWM_CH_Keypad	1
#define PWM_CH_LCD		2
#define PWM_CH_LAMP		3

#define PWM_CH_Keypad_MAX 8191
#define PWM_CH_Keypad_MIN 20
#define PWM_CH_LCD_MAX 8191
#define PWM_CH_LCD_MIN 20
#define PWM_CH_LAMP_MAX 4095 // 8191, but limit power
#define PWM_CH_LAMP_MIN 20
#define PWM_CH_LAMP_MANUAL_STEP 100

// DPFPlayer
// Commands
static const uint8_t DFP_START = 0x7E;
static const uint8_t DFP_VER = 0xFF;
static const uint8_t DFP_LEN = 0x06;
static const uint8_t DFP_noFB = 0x00;
static const uint8_t DFP_STOP = 0xEF;

// play modes
#define DFP_MODE_NO_REPEAT 0
#define DFP_MODE_REPEAT 1
#define DFP_MODE_FOLDER_REPEAT 2
#define DFP_MODE_SINGLE_REPEAT 3
#define DFP_MODE_RANDOM 4

/**
 * @struct HMI
 * @brief Structure for SX1503 port expander
 *
 */
typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address of Port Expander*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
	GPIO_TypeDef *Interrupt_PORT;	/**< GPIO Port of Interrupt pin */
	uint16_t Interrupt_PIN;			/**< GPIO Pin of Interrupt pin */
	TIM_HandleTypeDef *EncTimer; /**< Timer handle for encoder */
	UART_HandleTypeDef *UART_Handle;
	GPIO_TypeDef *DFP_EN_PORT;
	uint16_t DFP_EN_PIN;
} HMI;

// Return value
// HAL_StatusTypeDef ret;


/*
 * General Functions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

// TODO init port expander
void HMI_Setup(
		HMI *myHMI,
		I2C_HandleTypeDef *I2C_Handle,
		GPIO_TypeDef *INT_PORT,
		uint16_t INT_PIN,
		TIM_HandleTypeDef *EncTimerHandle,
		UART_HandleTypeDef *UART_Handle,
		GPIO_TypeDef *DFP_EN_PORT,
		uint16_t DFP_EN_PIN
);

// TODO set default config
HAL_StatusTypeDef HMI_defaultConfig(
		HMI *myHMI
);

// TODO set single LED
void HMI_Write_LED_b(
		HMI *myHMI,
		uint16_t LED,
		uint8_t state
);

// TODO write buffer to HMI
void HMI_Write(
		HMI *myHMI
);

// TODO read out button registers, safe to internal variables
void HMI_Read_GPIOs(
		HMI *myHMI
);

// TODO this function reads the current interrupt sate of the requested button; returns 0 or 1
uint8_t HMI_Read_Interrupt(
		HMI *myHMI,
		uint16_t button
);

// TODO this function reads the current sate of the requested button; returns 0 or 1
uint8_t HMI_Read_BTN(
		HMI *myHMI,
		uint16_t button
);

// TODO this function resets the the interrupt registers
void HMI_reset_INT(
		HMI *myHMI
);

// TODO set all LEDs
void HMI_set_all_LED_b(
		HMI *myHMI
);

// TODO reset all LEDs
void HMI_reset_all_LED_b(
		HMI *myHMI
);

// TODO control leds
void HMI_set_PWM(
		HMI *myHMI,
		uint8_t channel,
		uint16_t brightness
);

// TODO read encoder
int HMI_Encoder_position(
		HMI *myHMI
);

// Enable DFPlayer
void DFP_Enable(HMI *myHMI);

// Disable DFPlayer
void DFP_Disable(HMI *myHMI);

// Play song
HAL_StatusTypeDef DFP_Play(HMI *myHMI, uint8_t songNumber, uint8_t play_mode);

// Send command to DFPlayer
HAL_StatusTypeDef DFP_Send_CMD(HMI *myHMI, uint8_t cmd, uint8_t payload1, uint8_t payload0);

// Reset DFPlayer
HAL_StatusTypeDef DFP_Reset(HMI *myHMI);

//Switch to SD Card
HAL_StatusTypeDef DFP_SetToSD(HMI *myHMI);

// Set volume
HAL_StatusTypeDef DFP_setVolume(HMI *myHMI, uint8_t volume);

#endif /* INC_HMI_H_ */
