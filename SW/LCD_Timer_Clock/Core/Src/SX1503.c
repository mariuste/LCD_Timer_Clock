/**
 ******************************************************************************
 * @file           SX1503.c
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: 12.08.2022
 * Author: 	marius
 */

#include "SX1503.h"

void HMI_Setup(SX1503 *mySX1503, I2C_HandleTypeDef *I2C_Handle) {
	/* Store I2C Handle */
	mySX1503->I2C_Handle = I2C_Handle;

	/* Set I2C Address*/
	mySX1503->I2C_ADDRESS = SX1503_ADDR;
}

// TODO set default config
HAL_StatusTypeDef HMI_defaultConfig(SX1503 *mySX1503) {
	uint8_t buf[12]; // transmission buffer

	// set Bank A outputs (I/O5, I/O6, I/O7)
	buf[0] = SX_1503_RegDirA;
	buf[1] = 0b00011111;
	HAL_I2C_Master_Transmit(mySX1503->I2C_Handle, mySX1503->I2C_ADDRESS, buf, 2,
			HAL_TIMEOUT);

	// set Bank B outputs (I/O8, I/O9)
	buf[0] = SX_1503_RegDirB;
	buf[1] = 0b11111100;
	HAL_I2C_Master_Transmit(mySX1503->I2C_Handle, mySX1503->I2C_ADDRESS, buf, 2,
			HAL_TIMEOUT);
}

/*
 void setup_HMILEDs() {
 uint8_t buf[12]; // transmission buffer

 // set Bank A outputs (I/O5, I/O6, I/O7)
 buf[0] = SX_1503_RegDirA;
 buf[1] = 0b00011111;
 HAL_I2C_Master_Transmit(&hi2c2, SX1503_ADDR, buf, 2, HAL_TIMEOUT);

 // set Bank B outputs (I/O8, I/O9)
 buf[0] = SX_1503_RegDirB;
 buf[1] = 0b11111100;
 HAL_I2C_Master_Transmit(&hi2c2, SX1503_ADDR, buf, 2, HAL_TIMEOUT);
 }

 void HMI_Write_LED_b(uint16_t LED, uint8_t state) {

 // decide if Bank A or Bank B is affected
 if(LED <= 0xFF) {
 // Bank A
 if (state == 1) {
 // set LED on
 HMI_BANKA_Buffer[0] |= LED;
 } else if (state == 0) {
 // set LED off
 HMI_BANKA_Buffer[0] &= ~LED;
 }
 } else {
 // Bank B
 if (state == 1) {
 // set LED on
 HMI_BANKB_Buffer[0] |= (LED >> 8);
 } else if (state == 0) {
 // set LED off
 HMI_BANKB_Buffer[0] &= ~(LED >> 8);
 }
 }
 }

 void HMI_LED_set_All_b() {
 // set all LEDs
 HMI_Write_LED_b(HMI_LED_WDA, 1);
 HMI_Write_LED_b(HMI_LED_OT, 1);
 HMI_Write_LED_b(HMI_LED_TIME_DATE, 1);
 HMI_Write_LED_b(HMI_LED_TIMER1, 1);
 HMI_Write_LED_b(HMI_LED_TIMER2, 1);
 }

 void HMI_LED_reset_All_b() {
 // reset buffer
 HMI_BANKA_Buffer[0] = 0x00;
 HMI_BANKB_Buffer[0] = 0x00;
 }

 void HMI_LED_Refresh() {
 HAL_I2C_Mem_Write(&hi2c2, SX1503_ADDR, SX_1503_RegDataA, 1, &HMI_BANKA_Buffer[0], 1, HAL_MAX_DELAY);
 HAL_I2C_Mem_Write(&hi2c2, SX1503_ADDR, SX_1503_RegDataB, 1, &HMI_BANKB_Buffer[0], 1, HAL_MAX_DELAY);
 }

 */

