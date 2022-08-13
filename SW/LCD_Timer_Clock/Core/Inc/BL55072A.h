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
static const uint8_t BL55072A_ADDR = 0x20 << 1; // Use 8-bit address

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
void HMI_Setup(
		LCD *myLCD,
		I2C_HandleTypeDef *I2C_Handle
);

#endif /* INC_BL55072A_H_ */
