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

static const uint8_t LCD_ICSET_INT_OSC	= 0b00000000; // use internal oscillator
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

// MODESET ------------------------------------------------------------
static const uint8_t LCD_MODESET = 0xC0;
// MODESET Options:
static const uint8_t LCD_MODESET_LCD_DISABLE	= 0b00000000; // disable the LCD
static const uint8_t LCD_MODESET_LCD_ENABLE		= 0b00001000; // enable the LCD

static const uint8_t LCD_MODESET_BIAS_3		= 0b00000000; // set bias to 1/3
static const uint8_t LCD_MODESET_BIAS_2		= 0b00000100; // set bias to 1/2

// APCTL ------------------------------------------------------------
static const uint8_t LCD_APCTL = 0xFC;
// APCTL Options:
static const uint8_t LCD_APCTL_nALL_ON	= 0b00000000; // all on disable
static const uint8_t LCD_APCTL_ALL_ON	= 0b00000010; // all on ebable

static const uint8_t LCD_APCTL_nALL_OFF	= 0b00000000; // all off disable
static const uint8_t LCD_APCTL_ALL_OFF	= 0b00000001; // all off ebable

// BLKCTL ------------------------------------------------------------
static const uint8_t LCD_BLKCTL = 0x70;
// BLKCTL Options:
#define LCD_BLKCTL_OFF	0b00000000 // disable blinking
#define LCD_BLKCTL_0HZ5	0b00000001 // set blinking to 0.5 Hz
#define LCD_BLKCTL_1HZ	0b00000010 // set blinking to 1 Hz
#define LCD_BLKCTL_2HZ	0b00000011 // set blinking to 2 Hz
#define LCD_BLKCTL_0HZ3	0b00000100 // set blinking to 0.3 Hz
#define LCD_BLKCTL_0HZ2	0b00000101 // set blinking to 0.2 Hz


static const uint8_t END_CMD_MASK	= 0b01111111;


// LCD positions
#define POSITION_DIGIT_0 0
#define POSITION_DIGIT_1 1
#define POSITION_DIGIT_2 2
#define POSITION_DIGIT_3 3
#define POSITION_COLON 3
#define POSITION_DOT_DAY 1

#define SEGMENT_EMPTY 101
#define SEGMENT_COLON 102
#define SEGMENT_NO_COLON 103

#define DIGIT_EMPTY 100


#define LCD_LEFT 0
#define LCD_RIGHT 1

#define NO_LEADING_ZERO 0
#define LEADING_ZERO 1

extern uint8_t BL5502_BUFF[23];

/**
 * @struct LCD
 * @brief Structure for BL55072A based LCD
 *
 */
typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address of the LCD driver*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
	uint8_t LCD_data[16];				/**< Display Buffer */
} LCD;

// Return value
// HAL_StatusTypeDef ret;

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
HAL_StatusTypeDef LCD_Segment_AllOn(LCD *myLCD);
HAL_StatusTypeDef LCD_Segment_AllOff(LCD *myLCD);
HAL_StatusTypeDef LCD_Segment_normal(LCD *myLCD);
HAL_StatusTypeDef LCD_Blink(LCD *myLCD, uint8_t speed);

void LCD_Set_Digit(LCD *myLCD, uint8_t position, uint8_t number);
void LCD_Write_Number(LCD *myLCD, uint8_t position, uint8_t number, uint8_t leading_zero);
void LCD_Write_Colon(LCD *myLCD, uint8_t enable);
void LCD_Write_Dot(LCD *myLCD, uint8_t position);

HAL_StatusTypeDef LCD_SendBuffer(LCD *myLCD);

#endif /* INC_BL55072A_H_ */
