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

void HMI_Setup(SX1503 *mySX1503, I2C_HandleTypeDef *I2C_Handle,
		GPIO_TypeDef *INT_PORT, uint16_t INT_PIN) {
	/* Store I2C Handle */
	mySX1503->I2C_Handle = I2C_Handle;

	/* Set I2C Address */
	mySX1503->I2C_ADDRESS = SX1503_ADDR;

	/* Set Interrupt pin port */
	mySX1503->Interrupt_PORT = INT_PORT;

	/* Set Interrupt pin port */
	mySX1503->Interrupt_PIN = INT_PIN;
}

// TODO set default config
HAL_StatusTypeDef HMI_defaultConfig(SX1503 *mySX1503) {
	uint8_t buf[14]; // transmission buffer

	// set Bank A output Levels (I/O5, I/O6, I/O7) (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataA] = 0b00000000;

	// set Bank B outputs levels (I/O8, I/O9) (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataB] = 0b00000000;

	// set Bank A outputs (I/O5, I/O6, I/O7) (0: output, 1: Input)
	buf[SX_1503_RegDirA] = 0b00011111;

	// set Bank B outputs (I/O8, I/O9) (0: output, 1: Input)
	buf[SX_1503_RegDirB] = 0b11111100;

	// activate Pull-up resistors for buttons

	// activate pull-ups for Bank A inputs (I/O0, I/O1, I/O2, I/O3, I/O4) (0: pull up disabled, 1: pull up enabled)
	buf[SX_1503_PullUpA] = 0b00011111;

	// activate pull-ups for Bank B inputs (I/10) (0: pull up disabled, 1: pull up enabled)
	buf[SX_1503_PullUpB] = 0b11111100;

	// deactivate pull-downs for Bank A inputs (0: pull down disabled, 1: pull downs enabled)
	buf[SX_1503_PullDownA] = 0b00000000;

	// deactivate pull-downs for Bank B inputs (0: pull down disabled, 1: pull downs enabled)
	buf[SX_1503_PullDownB] = 0b00000000;

	// activate Interrupt Mask
	uint16_t interruptMask = 0x0000; // all disabled
	interruptMask |= HMI_BTN_WDA;
	interruptMask |= HMI_BTN_OTA;
	interruptMask |= HMI_BTN_TIME_DATE;
	interruptMask |= HMI_BTN_TIMER1;
	interruptMask |= HMI_BTN_TIMER2;
	interruptMask |= HMI_BTN_ENCODER;

	// invert mask to fit register
	interruptMask = ~interruptMask;

	// set interrupt mask of Bank A
	buf[SX_1503_InterruptMaskA] = interruptMask; // lower byte

	// set interrupt mask of Bank B
	buf[SX_1503_InterruptMaskB] = (interruptMask >> 8); // upper byte

	// set Interrupt detection of buttons to falling edge of Bank A (00: none, 01: rising, 10: falling, 11: both)
	uint16_t SensePattern = 0x0000; // all none

	SensePattern |= 0b10; 		// I/00  falling (HMI_BTN_TIMER2)
	SensePattern |= 0b10 << 2;	// I/01  falling (HMI_BTN_TIMER1)
	SensePattern |= 0b10 << 4;	// I/02  falling (HMI_BTN_TIME_DATE)
	SensePattern |= 0b10 << 6;	// I/03  falling (HMI_BTN_OTA)
	SensePattern |= 0b10 << 8;	// I/04  falling (HMI_BTN_WDA)

	// set interrupt detection pattern of Bank A, Lower byte
	buf[SX_1503_RegSenseLowA] = SensePattern; // lower byte

	// set interrupt detection pattern of Bank A, Upper byte
	buf[SX_1503_RegSenseHighA] = (SensePattern >> 8); // upper byte

	// set Interrupt detection of buttons to falling edge of Bank B (00: none, 01: rising, 10: falling, 11: both)
	SensePattern = 0x0000; // all none
	SensePattern |= 0b10 << 4;	// I/010 falling (HMI_BTN_ENCODER)

	// set interrupt detection pattern of Bank B, Lower byte
	buf[SX_1503_RegSenseLowB] = SensePattern; // lower byte

	// set interrupt detection pattern of Bank B, Upper byte
	buf[SX_1503_RegSenseHighB] = (SensePattern >> 8); // upper byte

	// Send data packet, beginning with register 0x00 (SX_1503_RegDataB)
	return HAL_I2C_Mem_Write(mySX1503->I2C_Handle, mySX1503->I2C_ADDRESS,
			SX_1503_RegDataB, 1, &buf[SX_1503_RegDataB], 14, HAL_MAX_DELAY);
}

// TODO set single LED
void HMI_Write_LED_b(SX1503 *mySX1503, uint16_t LED, uint8_t state) {
	// decide if Bank A or Bank B is affected
	if (LED <= 0xFF) {
		// Bank A
		if (state == 1) {
			// set LED on
			//HMI_BANKA_Buffer[0] |= LED;
			HMI_BANKA_Buffer |= LED;
		} else if (state == 0) {
			// set LED off
			//HMI_BANKA_Buffer[0] &= ~LED;
			HMI_BANKA_Buffer &= ~LED;
		}
	} else {
		// Bank B
		if (state == 1) {
			// set LED on
			//HMI_BANKB_Buffer[0] |= (LED >> 8);
			HMI_BANKB_Buffer |= (LED >> 8);
		} else if (state == 0) {
			// set LED off
			//HMI_BANKB_Buffer[0] &= ~(LED >> 8);
			HMI_BANKB_Buffer &= ~(LED >> 8);
		}
	}
}

// TODO write buffer to HMI
void HMI_Write(SX1503 *mySX1503) {
	uint8_t buf[2]; // transmission buffer

	// set Bank A output Levels (I/O5, I/O6, I/O7) (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataA] = HMI_BANKA_Buffer;

	// set Bank B outputs levels (I/O8, I/O9) (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataB] = HMI_BANKB_Buffer;

	// Send data packet, beginning with register 0x00 (SX_1503_RegDataB)
	HAL_I2C_Mem_Write(mySX1503->I2C_Handle, mySX1503->I2C_ADDRESS,
			SX_1503_RegDataB, 1, &buf[SX_1503_RegDataB], 2, HAL_MAX_DELAY);
}

// TODO this function reads the interrupt pin. It returns the button last pressed
uint16_t HMI_Read_INT_BTN_press(SX1503 *mySX1503) {
	// check if the interrupt is active
	if (HAL_GPIO_ReadPin(mySX1503->Interrupt_PORT, mySX1503->Interrupt_PIN)
			== 0) {

		// read interrupt source:
		// Receive buffer
		uint8_t buf[2];
		// read register SX_1503_RegInterruptSourceB
			HAL_I2C_Mem_Read(mySX1503->I2C_Handle, mySX1503->I2C_ADDRESS,
					SX_1503_RegInterruptSourceB, 1, &buf[0], 2, HAL_MAX_DELAY);

		// assemble back 16bit result
		uint16_t result = 0x0000;
		result = buf[1];			// Bank A is lower byte of the result
		result |= (buf[0] << 8);	// Bank B is upper byte of the result

		// only consider buttons; use mask to gnore other results
		result &= 0b0000010000011111;
		HMI_reset_INT(mySX1503);
		// reset interrupt register


		return result;
	} else {
		return 0x0000;
	}
}

// TODO this function reads the current sate of the requested button
void HMI_Read_BTN(SX1503 *mySX1503, uint16_t button) {

}

// TODO this function resets the the interrupt register RegInterruptSource
void HMI_reset_INT(SX1503 *mySX1503) {
	uint8_t buf[2]; // transmission buffer
	// reset Bank A Interrupt source
	buf[0] = 0xFF;

	// reset Bank B Interrupt source
	buf[1] = 0xFF;

	// Send data packet, beginning with register SX_1503_RegInterruptSourceB
	HAL_I2C_Mem_Write(mySX1503->I2C_Handle, mySX1503->I2C_ADDRESS,
			SX_1503_RegInterruptSourceB, 1, &buf[0], 2, HAL_MAX_DELAY);
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

