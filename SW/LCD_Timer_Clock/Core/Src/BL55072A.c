/**
 ******************************************************************************
 * @file           BL55072A.c
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: Aug 13, 2022
 * Author: 	marius
 */

#include <BL55072A.h>

// TODO init lcd
void LCD_Setup(LCD *myLCD, I2C_HandleTypeDef *I2C_Handle) {
	/* Store I2C Handle */
		myLCD->I2C_Handle = I2C_Handle;

		/* Set I2C Address */
		myLCD->I2C_ADDRESS = BL55072A_ADDR;
}
