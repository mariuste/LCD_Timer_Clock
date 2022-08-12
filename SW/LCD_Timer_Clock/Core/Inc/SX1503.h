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

typedef struct {
	uint8_t I2C_ADDRESS;			/**< I2C Address of Port Expander*/
	I2C_HandleTypeDef *I2C_Handle;	/**< I2C Interface Handle */
} SX1503;


#endif /* INC_SX1503_H_ */
