/**
 ******************************************************************************
 * @file           HMI.c
 * @brief          Brief Description
 *
 * Long Description
 ******************************************************************************
 * Created on: 12.08.2022
 * Author: 	marius
 */

#include <HMI.h>

void HMI_Setup(HMI *myHMI, I2C_HandleTypeDef *I2C_Handle,
		GPIO_TypeDef *INT_PORT, uint16_t INT_PIN) {
	/* Store I2C Handle */
	myHMI->I2C_Handle = I2C_Handle;

	/* Set I2C Address */
	myHMI->I2C_ADDRESS = SX1503_ADDR;

	/* Set Interrupt pin port */
	myHMI->Interrupt_PORT = INT_PORT;

	/* Set Interrupt pin port */
	myHMI->Interrupt_PIN = INT_PIN;
}

// TODO set default config
HAL_StatusTypeDef HMI_defaultConfig(HMI *myHMI) {
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
	return HAL_I2C_Mem_Write(myHMI->I2C_Handle, myHMI->I2C_ADDRESS,
			SX_1503_RegDataB, 1, &buf[SX_1503_RegDataB], 14, HAL_MAX_DELAY);
}

// TODO set single LED
void HMI_Write_LED_b(HMI *myHMI, uint16_t LED, uint8_t state) {
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
void HMI_Write(HMI *myHMI) {
	uint8_t buf[2]; // transmission buffer

	// set Bank A output Levels (I/O5, I/O6, I/O7) (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataA] = HMI_BANKA_Buffer;

	// set Bank B outputs levels (I/O8, I/O9) (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataB] = HMI_BANKB_Buffer;

	// Send data packet, beginning with register 0x00 (SX_1503_RegDataB)
	HAL_I2C_Mem_Write(myHMI->I2C_Handle, myHMI->I2C_ADDRESS,
			SX_1503_RegDataB, 1, &buf[SX_1503_RegDataB], 2, HAL_MAX_DELAY);
}

// TODO this function reads the interrupt pin. It returns the button last pressed
uint16_t HMI_Read_INT_BTN_press(HMI *myHMI) {
	// check if the interrupt is active
	if (HAL_GPIO_ReadPin(myHMI->Interrupt_PORT, myHMI->Interrupt_PIN)
			== 0) {

		// read interrupt source:
		// Receive buffer
		uint8_t buf[2];
		// read register SX_1503_RegInterruptSourceB
			HAL_I2C_Mem_Read(myHMI->I2C_Handle, myHMI->I2C_ADDRESS,
					SX_1503_RegInterruptSourceB, 1, &buf[0], 2, HAL_MAX_DELAY);

		// assemble back 16bit result
		uint16_t result = 0x0000;
		result = buf[1];			// Bank A is lower byte of the result
		result |= (buf[0] << 8);	// Bank B is upper byte of the result

		// only consider buttons; use mask to gnore other results
		result &= 0b0000010000011111;
		HMI_reset_INT(myHMI);
		// reset interrupt register


		return result;
	} else {
		return 0x0000;
	}
}

// TODO this function reads the current sate of the requested button
void HMI_Read_BTN(HMI *myHMI, uint16_t button) {

}

// TODO this function resets the the interrupt register RegInterruptSource
void HMI_reset_INT(HMI *myHMI) {
	uint8_t buf[2]; // transmission buffer
	// reset Bank A Interrupt source
	buf[1] = 0xFF;

	// reset Bank B Interrupt source
	buf[0] = 0xFF;

	// Send data packet, beginning with register SX_1503_RegInterruptSourceB
	HAL_I2C_Mem_Write(myHMI->I2C_Handle, myHMI->I2C_ADDRESS,
			SX_1503_RegInterruptSourceB, 1, &buf[0], 2, HAL_MAX_DELAY);
}

// TODO set all LEDs
void HMI_set_all_LED(HMI *myHMI) {
	// fill buffer with LED to write to
	HMI_Write_LED_b(myHMI, HMI_LED_WDA,			1);
	HMI_Write_LED_b(myHMI, HMI_LED_OTA,			1);
	HMI_Write_LED_b(myHMI, HMI_LED_TIME_DATE,	1);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER1,		1);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER2,		1);

	// write buffer to activate LEDs
	HMI_Write(myHMI);
}

// TODO reset all LEDs
void HMI_reset_all_LED(HMI *myHMI) {
	// fill buffer with LED to write to
	HMI_Write_LED_b(myHMI, HMI_LED_WDA,			0);
	HMI_Write_LED_b(myHMI, HMI_LED_OTA,			0);
	HMI_Write_LED_b(myHMI, HMI_LED_TIME_DATE,	0);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER1,		0);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER2,		0);

	// write buffer to activate LEDs
	HMI_Write(myHMI);
}