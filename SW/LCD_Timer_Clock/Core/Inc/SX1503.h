/**
 ******************************************************************************
 * @file           SX1503.h
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: 12.08.2022
 * Author: 	marius
 */
#ifndef INC_SX1503_H_
#define INC_SX1503_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32G0:
#include "stm32g0xx_hal.h"


// Port Expander Test
static const uint8_t SX1503_ADDR = 0x20 << 1; // Use 8-bit address
static const uint8_t SX_1503_RegDataB = 0x00; // register address
static const uint8_t SX_1503_RegDataA = 0x01; // register address
static const uint8_t SX_1503_RegDirB = 0x02; // register address
static const uint8_t SX_1503_RegDirA = 0x03; // register address

// Port Expander outputs:
static const uint16_t HMI_LED_WDA	= 0b0000001000000000; // Week Day Alarm LED
static const uint16_t HMI_LED_OT 	= 0b0000000100000000; // One Time Alarm LED
static const uint16_t HMI_LED_TIME_DATE	= 0b0000000010000000; // Time/Date LED
static const uint16_t HMI_LED_TIMER1	= 0b0000000001000000; // Timer1 LED /TODO swap back (see findings List PT1)
static const uint16_t HMI_LED_TIMER2	= 0b0000000000100000; // Timer2 LED /TODO swap back (see findings List PT1)

// TODO uint8_t HMI_BANKA_Buffer[1]	= {0x00}; // Output Buffer of Bank A
// TODO uint8_t HMI_BANKB_Buffer[1]	= {0x00}; // Output Buffer of Bank B





/**
 * @struct SX1503
 * @brief Structure for SX1503 port expander
 *
 */
typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address of Port Expander*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
} SX1503;


// Return value
HAL_StatusTypeDef ret;


/*
 * General Functions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

/*
 * INITIALIZATION
 */
void SX1503_Setup(
		SX1503 *mySX1503,
		I2C_HandleTypeDef *I2C_Handle
);


#endif /* INC_SX1503_H_ */
