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

// TODO initializes LCD controller
HAL_StatusTypeDef LCD_INIT(LCD *myLCD) {
	uint8_t buf[6]; // transmission buffer

	// perform soft reset
	buf[0] = LCD_ICSET | LCD_ICSET_SW_RESET;
	//buf[0] = 0xEA; // ICSET - software reset

	// set to 53Hz, line inversion, power safe mode 1
	//buf[1] = 0x38; // DISCTL - configure display
	buf[1] = LCD_DISCTL | LCD_DISCTL_F53Hz | LCD_DISCTL_L_INV | LCD_DISCTL_PSM1;

	// last byte gets a command bit:
	buf[1] &= END_CMD_MASK                   ;

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 2, 100);
}

HAL_StatusTypeDef LCD_Enable(LCD *myLCD) {
	uint8_t buf[4]; // transmission buffer

	buf[0] = 0x4C; // MODESET - Enable Display

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}

HAL_StatusTypeDef LCD_AllOn(LCD *myLCD) {
	uint8_t buf[4]; // transmission buffer

	buf[0] = 0xFE; // APCTL - All segments on
	buf[1] = 0x71; // BLKCTL - Blink all

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 2, 100);
}

HAL_StatusTypeDef LCD_AllOff(LCD *myLCD) {
	uint8_t buf[4]; // transmission buffer

	buf[0] = 0x7D; // APCTL - All segments off

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}

HAL_StatusTypeDef LCD_Write(LCD *myLCD) {
	uint8_t buf[4]; // transmission buffer

	buf[0] = 0xB6; // Set power save mode
	buf[1] = 0xF0; // Set blink
	buf[2] = 0xFC; // Close all pixels on/off function
	buf[3] = 0xC8; // Set display on
	buf[4] = 0xE8; // Set msb of ram address
	buf[5] = 0x00; // Set ram address

	for (int i = 6; i < 24; i++) {
		buf[i] = 0xFF; //
	}

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 24, 100);
}

HAL_StatusTypeDef SEG_WriteBuffer(LCD *myLCD, uint8_t data) {
	BL5502_BUFF[0] = 0xF0;
	BL5502_BUFF[1] = 0xA3;
	BL5502_BUFF[2] = 0xE8;
	BL5502_BUFF[3] = 0x00;

	for (int i = 4; i < 22; i++) {
		BL5502_BUFF[i] = data; //
	}
	HAL_StatusTypeDef return_value;

	return_value = HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS,
			(uint8_t*) BL5502_BUFF, 22, 100);
	BL5502_BUFF[0] = 0xC8;
	HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) BL5502_BUFF, 1,
			100);

	return return_value;
}
