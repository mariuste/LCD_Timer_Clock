/**
 ******************************************************************************
 * @file           BL55072A.h
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: Aug 13, 2022
 * Author: 	marius
 */
#ifndef INC_BL55072A_H_
#define INC_BL55072A_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32G0:
#include "stm32g0xx_hal.h"

// I2C Address
static const uint8_t BL55072A_ADDR = 0x7C; // Use 8-bit address

// LCD commands ###############################################################

// ICSET ------------------------------------------------------------
static const uint8_t LCD_ICSET = 0xE8;
// ICSET Options:
static const uint8_t LCD_ICSET_SW_RESET = 0b00000010; // perform sw reset
static const uint8_t LCD_ICSET_EXT_OSC	= 0b00000001; // use external oscillator

// DISCTL ------------------------------------------------------------
static const uint8_t LCD_DISCTL = 0xA0;
// DISCTL Options:
static const uint8_t LCD_DISCTL_F80Hz	= 0b00000000; // set frequency to 80Hz
static const uint8_t LCD_DISCTL_F71Hz	= 0b00001000; // set frequency to 71Hz
static const uint8_t LCD_DISCTL_F64Hz	= 0b00010000; // set frequency to 64Hz
static const uint8_t LCD_DISCTL_F53Hz	= 0b00011000; // set frequency to 53Hz

static const uint8_t LCD_DISCTL_L_INV	= 0b00000000; // set update type to line inversion
static const uint8_t LCD_DISCTL_F_INV	= 0b00000100; // set update type to frame inversion

static const uint8_t LCD_DISCTL_PSM1	= 0b00000000; // set to power savings mode 1
static const uint8_t LCD_DISCTL_PSM2	= 0b00000001; // set to power savings mode 2
static const uint8_t LCD_DISCTL_NM		= 0b00000010; // set for normal mode
static const uint8_t LCD_DISCTL_HPM		= 0b00000011; // set to high power mode

static const uint8_t END_CMD_MASK	= 0b01111111;

uint8_t BL5502_BUFF[23]; // Display buffer

/**
 * @struct LCD
 * @brief Structure for BL55072A based LCD
 *
 */
typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address of the LCD driver*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
} LCD;

// Return value
HAL_StatusTypeDef ret;

/*
 * General Functions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

// TODO init lcd
void LCD_Setup(
		LCD *myLCD,
		I2C_HandleTypeDef *I2C_Handle
);


// TODO initialize LCD driver
HAL_StatusTypeDef LCD_INIT(LCD *myLCD);
HAL_StatusTypeDef LCD_Enable(LCD *myLCD);
HAL_StatusTypeDef LCD_AllOn(LCD *myLCD);
HAL_StatusTypeDef LCD_AllOff(LCD *myLCD);
HAL_StatusTypeDef LCD_Write(LCD *myLCD);
HAL_StatusTypeDef SEG_WriteBuffer(LCD *myLCD, uint8_t data);

#endif /* INC_BL55072A_H_ */
