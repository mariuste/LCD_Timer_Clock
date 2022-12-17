/**
 ******************************************************************************
 * @file           BL55072A.h
 * @brief          Header file for BL55072A.c
 *
 * This is a driver for the LCD controller BL55072A
 *
 * It offers simple commands to utilize most functions of the driver.
 *
 * The implementation is currently application specific. It is configured to drive
 * an LCD with 32 segments
 *
 ******************************************************************************
 * Created on: Aug 13, 2022
 * Author: 	marius
 */
#ifndef INC_BL55072A_H_
#define INC_BL55072A_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32L4:
// #include "stm32l4xx_hal.h" // needed for unit types
#include "stm32g0xx_hal.h"
// TODO: generalize include for units so not every STM needs their own special include



#define BL55072A_ADDR 0x7C /**< I2C Base Address (already shifted into 8Bit format) */

// LCD commands ###############################################################
#define LCD_CMD_ICSET       0xE8   /**< Command to set address */
#define LCD_CMD_DISCTL      0xA0   /**< Command to set Frame Mode, Drive Mode and power saving mode */
#define LCD_CMD_MODESET     0xC0   /**< Command to enable/disable LCD and set bias */
#define LCD_CMD_APCTL       0xFC   /**< Command to set all segments on or off */
#define LCD_CMD_BLKCTL      0x70   /**< Command to control blink modes */

// LCD options ###############################################################

// ICSET Options:
#define LCD_OPT_ICSET_SW_RESET  0b00000010 /**< Option to perform sw reset */
#define LCD_OPT_ICSET_INT_OSC	0b00000000 /**< Option to use internal oscillator */
#define LCD_OPT_ICSET_EXT_OSC	0b00000001 /**< Option to use external oscillator */

// DISCTL Options:
#define LCD_OPT_DISCTL_F80Hz	0b00000000 /**< Option to set frequency to 80Hz */
#define LCD_OPT_DISCTL_F71Hz	0b00001000 /**< Option to set frequency to 71Hz */
#define LCD_OPT_DISCTL_F64Hz	0b00010000 /**< Option to set frequency to 64Hz */
#define LCD_OPT_DISCTL_F53Hz	0b00011000 /**< Option to set frequency to 53Hz */

#define LCD_OPT_DISCTL_L_INV	0b00000000 /**< Option to set update type to line inversion */
#define LCD_OPT_DISCTL_F_INV	0b00000100 /**< Option to set update type to frame inversion */

#define LCD_OPT_DISCTL_PSM1	    0b00000000 /**< Option to set to power savings mode 1 */
#define LCD_OPT_DISCTL_PSM2	    0b00000001 /**< Option to set to power savings mode 2 */
#define LCD_OPT_DISCTL_NM		0b00000010 /**< Option to set for normal mode */
#define LCD_OPT_DISCTL_HPM		0b00000011 /**< Option to set to high power mode */

// MODESET Options:
#define LCD_OPT_MODESET_LCD_DISABLE	0b00000000 /**< Option to disable the LCD */
#define LCD_OPT_MODESET_LCD_ENABLE	0b00001000 /**< Option to enable the LCD */

#define LCD_OPT_MODESET_BIAS_3		0b00000000 /**< Option to set bias to 1/3 */
#define LCD_OPT_MODESET_BIAS_2		0b00000100 /**< Option to set bias to 1/2 */

// APCTL Options:
#define LCD_OPT_APCTL_nALL_ON	    0b00000000 /**< Disable Option "all on" */
#define LCD_OPT_APCTL_ALL_ON	    0b00000010 /**< Enable Option "all on" */

#define LCD_OPT_APCTL_nALL_OFF	    0b00000000 /**< Disable Option "all off" */
#define LCD_OPT_APCTL_ALL_OFF	    0b00000001 /**< Enable Option "all off" */

// BLKCTL Options:
#define LCD_BLKCTL_OFF	            0b00000000 /**< Option to disable blinking */
#define LCD_BLKCTL_0HZ5	            0b00000001 /**< Option to set blinking to 0.5 Hz */
#define LCD_BLKCTL_1HZ	            0b00000010 /**< Option to set blinking to 1 Hz */
#define LCD_BLKCTL_2HZ	            0b00000011 /**< Option to set blinking to 2 Hz */
#define LCD_BLKCTL_0HZ3	            0b00000100 /**< Option to set blinking to 0.3 Hz */
#define LCD_BLKCTL_0HZ2	            0b00000101 /**< Option to set blinking to 0.2 Hz */


static const uint8_t END_CMD_MASK	= 0b01111111; /**< Mask to end ommands */


// LCD positions
#define POSITION_DIGIT_0 0 /**< First Digit Location in 4 Digit, 32 Segment display */
#define POSITION_DIGIT_1 1 /**< Second Digit Location in 4 Digit, 32 Segment display */
#define POSITION_DIGIT_2 2 /**< Third Digit Location in 4 Digit, 32 Segment display */
#define POSITION_DIGIT_3 3 /**< Forth Digit Location in 4 Digit, 32 Segment display */
#define POSITION_COLON 3 /**< Position of middle colon */
#define POSITION_DOT_DAY 1 /**< Position of second first dot */

#define SEGMENT_EMPTY 101 /**< Code for displaying empty segment */
#define SEGMENT_COLON 102 /**< Code for displaying colon or dot in this digit */
#define SEGMENT_NO_COLON 103 /**< Code for not displaying colon or dot in this digit */

#define DIGIT_EMPTY 100 /**< Code for displaying empty digit */


#define LCD_LEFT 0 /**< Code for the two left digts */
#define LCD_RIGHT 1 /**< Code for the two right digts */

#define NO_LEADING_ZERO 0 /**< Code for enabling/disabling leading zeros */
#define LEADING_ZERO 1 /**< Code for enabling/disabling leading zeros */

extern uint8_t BL5502_BUFF[23]; /**< 32 Segment display Buffer */

/**
 * @struct LCD
 * @brief Structure for BL55072A LCD driver. It is used to configure the LCD
 *
 */
typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address of the LCD driver*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
	uint8_t LCD_data[16];			/**< Display Buffer */
} LCD;

/*
 * General Functions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

// Doxygen Group start:
/** @name BL55072A General Functions
 */
/**@{*/

/**
 * @fn void LCD_Setup(LCD*, I2C_HandleTypeDef*)
 *
 * Basic setup BL5572A driver
 *
 * Sets up the I2C address and loads an empty display buffer
 *
 */
void LCD_Setup(
		LCD *myLCD, 					/**< Pointer to the LCD handle */
		I2C_HandleTypeDef *I2C_Handle	/**< Pointer to the I2C handle */
);

/**
 * @fn HAL_StatusTypeDef LCD_INIT(LCD*)
 * @brief Initialize the LCD controller
 *
 * Sets up the LCD controller with basic commands to prepare it for normal operation
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_INIT(
		LCD *myLCD	/**< Pointer to the LCD handle */
		);

/**@}*/
// Doxygen Group end

// Doxygen Group start:
/** @name LCD Internal Functions
 */
/**@{*/

/**
 * @fn HAL_StatusTypeDef LCD_Enable(LCD*)
 *
 * Enable LCD
 *
 * Enable LCD and set bias
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_Enable(
		LCD *myLCD	/**< Pointer to the LCD handle */
		);

/**
 * @fn HAL_StatusTypeDef LCD_Segment_AllOn(LCD*)
 *
 * Enable all segments
 *
 * Enables all segments. May be helpful for startup and debugging
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_Segment_AllOn(
		LCD *myLCD /**< Pointer to the LCD handle */
		);

/**
 * @fn HAL_StatusTypeDef LCD_Segment_AllOff(LCD*)
 *
 * Disable all segments
 *
 * Disables all segments. May be helpful for debugging
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_Segment_AllOff(
		LCD *myLCD /**< Pointer to the LCD handle */
		);

/**
 * @fn HAL_StatusTypeDef LCD_Segment_normal(LCD*)
 *
 * Enable normal mode
 *
 * Clears forced AllOn an AllOff state
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_Segment_normal(
		LCD *myLCD /**< Pointer to the LCD handle */
		);

/**
 * @fn HAL_StatusTypeDef LCD_Blink(LCD*, uint8_t)
 *
 * Enable/Disable blink and set blink speed
 *
 * Configure segment blink speed. ONly the entire panel can blink,
 * not single segments or portions. Usually this is done in software.
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_Blink(
		LCD *myLCD,		/**< Pointer to the LCD handle */
		uint8_t speed	/**< Desired blink speed: LCD_BLKCTL_XX */
		);

/**@}*/
// Doxygen Group end

// Doxygen Group start:
/** @name LCD Control Functions
 */
/**@{*/

/**
 * @fn void LCD_Set_Digit(LCD*, uint8_t, uint8_t)
 *
 * Write single digit to LCD
 *
 * Function to write one digit to the LCD. This function is mostly
 * used internally. Writing an entire number with LCD_Write_Number
 * is probably more useful
 *
 */
void LCD_Set_Digit(
		LCD *myLCD,			/**< Pointer to the LCD handle */
		uint8_t position,	/**< Position of digit (POSITION_DIGIT_0..3) */
		uint8_t number		/**< Digit to write (can be 0..9 and
							SEGMENT_EMPTY, SEGMENT_COLON, SEGMENT_NO_COLON*/
		);

/**
 * @fn void LCD_Write_Number(LCD*, uint8_t, uint8_t, uint8_t)
 *
 * Write a two digit number to the LCD
 *
 * This function writes a number to the LCD. The number can be up to two
 * digits long. It can be specified if there should be a leading zero or not.
 */
void LCD_Write_Number(
		LCD *myLCD,				/**< Pointer to the LCD handle */
		uint8_t position,		/**< Position of the number:
								LCD_LEFT / LCD_RIGHT */
		uint8_t number,			/**< Number to write: 0..99, DIGIT_EMPTY */
		uint8_t leading_zero	/**< Enable or disable leading zero:
		 	 	 	 	 	 	 NO_LEADING_ZERO, LEADING_ZERO */
		);

/**
 * @fn void LCD_Write_Colon(LCD*, uint8_t)
 *
 * Write the colon symbol
 *
 * Writes the colon of the display
 *
 */
void LCD_Write_Colon(
		LCD *myLCD,		/**< Pointer to the LCD handle */
		uint8_t enable	/**< Enable or Disable colon: 0, 1 */
		);

/**
 * @fn void LCD_Write_Dot(LCD*, uint8_t)
 *
 * Enables the dot
 *
 * Writes on of the dot of the display
 *
 */
void LCD_Write_Dot(
		LCD *myLCD,			/**< Pointer to the LCD handle */
		uint8_t position	/**< Position of the dot: POSITION_DIGIT_0..2 */
		);

/**
 * @fn HAL_StatusTypeDef LCD_SendBuffer(LCD*)
 *
 * Flush the display buffer
 *
 * Writs the internally storred display buffer to the LCD, therefore refreshes
 * the display.
 *
 * @return HAL response of the I2C Read/Write function
 */
HAL_StatusTypeDef LCD_SendBuffer(
		LCD *myLCD	/**< Pointer to the LCD handle */
		);

/**@}*/
// Doxygen Group end

#endif /* INC_BL55072A_H_ */
