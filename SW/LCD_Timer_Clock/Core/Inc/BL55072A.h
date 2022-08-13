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
