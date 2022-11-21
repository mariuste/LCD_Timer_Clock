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

uint8_t BL5502_BUFF[23];

// TODO init lcd
void LCD_Setup(LCD *myLCD, I2C_HandleTypeDef *I2C_Handle) {
	/* Store I2C Handle */
		myLCD->I2C_Handle = I2C_Handle;

		/* Set I2C Address */
		myLCD->I2C_ADDRESS = BL55072A_ADDR;

		/* Fill sample data into buffer */
		myLCD->LCD_data[0] = 0x00; // a b
		myLCD->LCD_data[1] = 0x00; // c d
		myLCD->LCD_data[2] = 0x00; // e f
		myLCD->LCD_data[3] = 0x00; // g DP
		myLCD->LCD_data[4] = 0x00;
		myLCD->LCD_data[5] = 0x00;
		myLCD->LCD_data[6] = 0x00;
		myLCD->LCD_data[7] = 0x00;
		myLCD->LCD_data[8] = 0x00;
		myLCD->LCD_data[9] = 0x00;
		myLCD->LCD_data[10] = 0x00;
		myLCD->LCD_data[11] = 0x00;
		myLCD->LCD_data[12] = 0x00;
		myLCD->LCD_data[13] = 0x00;
		myLCD->LCD_data[14] = 0x00;
		myLCD->LCD_data[15] = 0x00;
}

// TODO initializes LCD controller
HAL_StatusTypeDef LCD_INIT(LCD *myLCD) {
	uint8_t buf[2]; // transmission buffer

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

// TODO activated LCD
HAL_StatusTypeDef LCD_Enable(LCD *myLCD) {
	uint8_t buf[1]; // transmission buffer

	// enable LCD and set Bias to 1/3
	buf[0] = LCD_MODESET | LCD_MODESET_LCD_ENABLE | LCD_MODESET_BIAS_3;

	// last byte gets a command bit:
	buf[0] &= END_CMD_MASK;

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}

HAL_StatusTypeDef LCD_Segment_AllOn(LCD *myLCD) {
	uint8_t buf[1]; // transmission buffer

	//Enable all segments
	buf[0] = LCD_APCTL | LCD_APCTL_ALL_ON | LCD_APCTL_nALL_OFF;

	// last byte gets a command bit:
	buf[0] &= END_CMD_MASK;

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}

HAL_StatusTypeDef LCD_Segment_AllOff(LCD *myLCD) {
	uint8_t buf[1]; // transmission buffer

	//Deactivate all segments
	buf[0] = LCD_APCTL | LCD_APCTL_nALL_ON | LCD_APCTL_ALL_OFF;

	// last byte gets a command bit:
	buf[0] &= END_CMD_MASK;

	// send initialization
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}

HAL_StatusTypeDef LCD_Segment_normal(LCD *myLCD) {
	uint8_t buf[1]; // transmission buffer

	// Reset all on or all off state
	buf[0] = LCD_APCTL | LCD_APCTL_nALL_ON | LCD_APCTL_nALL_OFF;

	// last byte gets a command bit:
	buf[0] &= END_CMD_MASK;

	// send command
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}

HAL_StatusTypeDef LCD_Blink(LCD *myLCD, uint8_t speed) {
	uint8_t buf[1]; // transmission buffer

	// set speed of blink pattern
	switch(speed) {
		case LCD_BLKCTL_OFF:	buf[0] = LCD_BLKCTL | LCD_BLKCTL_OFF; break;
		case LCD_BLKCTL_0HZ5:	buf[0] = LCD_BLKCTL | LCD_BLKCTL_0HZ5; break;
		case LCD_BLKCTL_1HZ:	buf[0] = LCD_BLKCTL | LCD_BLKCTL_1HZ; break;
		case LCD_BLKCTL_2HZ:	buf[0] = LCD_BLKCTL | LCD_BLKCTL_2HZ; break;
		case LCD_BLKCTL_0HZ3:	buf[0] = LCD_BLKCTL | LCD_BLKCTL_0HZ3; break;
		case LCD_BLKCTL_0HZ2:	buf[0] = LCD_BLKCTL | LCD_BLKCTL_0HZ2; break;
		default: return HAL_OK; break;
	}

	// last byte gets a command bit:
	buf[0] &= END_CMD_MASK;

	// send command
	return HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) buf, 1, 100);
}



void LCD_Set_Digit(LCD *myLCD, uint8_t position, uint8_t number) {
	// choose the correct sub-buffer
	uint32_t digitsegments = 0x00000000;
	// overwrite existing value
	uint8_t add_dot_or_colon = 0;
	uint8_t remove_dot_or_colon = 0;

	switch(number) {
	case  0:	digitsegments = 0x00888888; break;
	case  1:	digitsegments = 0x00008008; break;
	case  2:	digitsegments = 0x80800888; break;
	case  3:	digitsegments = 0x80008888; break;
	case  4:	digitsegments = 0x80088008; break;
	case  5:	digitsegments = 0x80088880; break;
	case  6:	digitsegments = 0x80888880; break;
	case  7:	digitsegments = 0x00008088; break;
	case  8:	digitsegments = 0x80888888; break;
	case  9:	digitsegments = 0x80088888; break;
	case SEGMENT_EMPTY:	digitsegments = 0x00000000; break; // empty character
	case SEGMENT_COLON:									// dot or colon
		// do not overwrite existing value
		add_dot_or_colon = 1;
		digitsegments = 0x08000000;
		break;
	case SEGMENT_NO_COLON:									// dot or colon
		// remove colon but keep existing data
		remove_dot_or_colon = 1;
		digitsegments = 0x00000000;
		break;
	}

	if ((add_dot_or_colon == 0) && (remove_dot_or_colon == 0)) {
		// overwrite data but preserve existing dot or colon
		myLCD->LCD_data[0 + position * 4] =  (myLCD->LCD_data[0 + position * 4] & 0x08000000) |  digitsegments;
		myLCD->LCD_data[1 + position * 4] =  (myLCD->LCD_data[1 + position * 4] & 0x08000000) | digitsegments >> 8;
		myLCD->LCD_data[2 + position * 4] =  (myLCD->LCD_data[2 + position * 4] & 0x08000000) | digitsegments >> 16;
		myLCD->LCD_data[3 + position * 4] =  (myLCD->LCD_data[3 + position * 4] & 0x08000000) | digitsegments >> 24;
	} else if (add_dot_or_colon == 1) {
		// just add the colon or dot
		myLCD->LCD_data[3 + position * 4] = (myLCD->LCD_data[3 + position * 4] | 0x08);
	} else if (remove_dot_or_colon == 1) {
		// just remove the colon or dot
		myLCD->LCD_data[3 + position * 4] = (myLCD->LCD_data[3 + position * 4] & 0x80);
	}

}


//myLCD->LCD_data[0] = 0x80; // a b

// TODO write Number (position 0:left / 1: right)
void LCD_Write_Number(LCD *myLCD, uint8_t position, uint8_t number, uint8_t leading_zero) {
	// first slit into two digits if necessary:

	uint8_t lower_digit;
	uint8_t upper_digit;
	uint8_t show_upper_digit = leading_zero;

	if (number == DIGIT_EMPTY) {
		// special case, draw empty digits
		LCD_Set_Digit(myLCD, POSITION_DIGIT_1+2*position, SEGMENT_EMPTY);
		LCD_Set_Digit(myLCD, POSITION_DIGIT_0+2*position, SEGMENT_EMPTY);
	} else if(number >= 10) {
		show_upper_digit = 1; // always show upper digit

		// split into two
		upper_digit = number / 10;
		upper_digit = upper_digit % 10;
		lower_digit = number % 10;

		// draw digits
		LCD_Set_Digit(myLCD, POSITION_DIGIT_1+2*position, lower_digit);
		LCD_Set_Digit(myLCD, POSITION_DIGIT_0+2*position, upper_digit);


	} else {
		// handle leading zero
		if (show_upper_digit == 0) {
			// hide leading zero
			LCD_Set_Digit(myLCD, POSITION_DIGIT_0+2*position, SEGMENT_EMPTY);
		} else {
			// show leading zero
			LCD_Set_Digit(myLCD, POSITION_DIGIT_0+2*position, 0);
		}
		// draw digit
		LCD_Set_Digit(myLCD, 1+2*position, number);

	}
}

void LCD_Write_Colon(LCD *myLCD, uint8_t enable) {
	if (enable == 1) {
		// enable colon
		LCD_Set_Digit(myLCD, POSITION_COLON, SEGMENT_COLON);
	} else if (enable == 0) {
		LCD_Set_Digit(myLCD, POSITION_COLON, SEGMENT_NO_COLON);
	}
}

void LCD_Write_Dot(LCD *myLCD, uint8_t position) {
	LCD_Set_Digit(myLCD, position, SEGMENT_COLON);
}

// TODO function to display/hide colon)

HAL_StatusTypeDef LCD_SendBuffer(LCD *myLCD) {
	BL5502_BUFF[0] = 0xF0;
	BL5502_BUFF[1] = 0xA3;
	BL5502_BUFF[2] = 0xE8;
	BL5502_BUFF[3] = 0x00;

	for (int i = 4; i < 20; i++) {
		BL5502_BUFF[i] = myLCD->LCD_data[i-4];
	}
	HAL_StatusTypeDef return_value;

	return_value = HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS,
			(uint8_t*) BL5502_BUFF, 22, 100);
	BL5502_BUFF[0] = 0xC8;
	HAL_I2C_Master_Transmit(myLCD->I2C_Handle, myLCD->I2C_ADDRESS, (uint8_t*) BL5502_BUFF, 1,
			100);

	return return_value;
}
