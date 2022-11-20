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

// Variables ############################################

// Trafer Buffer
uint8_t HMI_BANKA_Buffer; // Output Buffer of Bank A
uint8_t HMI_BANKB_Buffer; // Output Buffer of Bank B

// Encoder
uint32_t Encoder_current_couter;
uint32_t Encoder_last_couter;
int Encoder_Position;

// Button States
uint16_t HMI_BTN_WDA_STATE = BUTTON_NOT_PRESSED;			// Week Day Alarm Button
uint16_t HMI_BTN_OTA_STATE = BUTTON_NOT_PRESSED;			// One Time Alarm Button
uint16_t HMI_BTN_TIME_DATE_STATE  = BUTTON_NOT_PRESSED;	// Time/Date Button
uint16_t HMI_BTN_TIMER1_STATE  = BUTTON_NOT_PRESSED;		// Timer1 Button
uint16_t HMI_BTN_TIMER2_STATE  = BUTTON_NOT_PRESSED;		// Timer2 Button
uint16_t HMI_BTN_ENCODER_STATE = BUTTON_NOT_PRESSED;		// Encoder Button

uint16_t HMI_BTN_WDA_Interrupt = NO_INTERRUPT;
uint16_t HMI_BTN_OTA_Interrupt = NO_INTERRUPT;
uint16_t HMI_BTN_TIME_DATE_Interrupt  = NO_INTERRUPT;
uint16_t HMI_BTN_TIMER1_Interrupt  = NO_INTERRUPT;
uint16_t HMI_BTN_TIMER2_Interrupt  = NO_INTERRUPT;
uint16_t HMI_BTN_ENCODER_Interrupt = NO_INTERRUPT;

void HMI_Setup(HMI *myHMI, I2C_HandleTypeDef *I2C_Handle,
		GPIO_TypeDef *INT_PORT, uint16_t INT_PIN, TIM_HandleTypeDef *EncTimerHandle) {
	/* Store I2C Handle */
	myHMI->I2C_Handle = I2C_Handle;

	/* Set I2C Address */
	myHMI->I2C_ADDRESS = SX1503_ADDR;

	/* Set Interrupt pin port */
	myHMI->Interrupt_PORT = INT_PORT;

	/* Set Interrupt pin port */
	myHMI->Interrupt_PIN = INT_PIN;

	/* Store encoder timer handle */
	myHMI->EncTimer = EncTimerHandle;

	/* Initialize encoder variables */
	Encoder_current_couter = __HAL_TIM_GET_COUNTER(myHMI->EncTimer);
	Encoder_last_couter = Encoder_current_couter;
	Encoder_Position = 0;
}

// TODO set default config
HAL_StatusTypeDef HMI_defaultConfig(HMI *myHMI) {
	// Set the PWM panels to 0%
	TIM3->CCR1 = 5; // LCD
	TIM3->CCR2 = 0; // LIGHT
	TIM2->CCR2 = 5; // Keypad

	uint8_t buf[14]; // transmission buffer

	// set Bank A output Levels (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataA] = 0b00000000;

	// set Bank B outputs levels (0: LOW, 1: HIGH)
	buf[SX_1503_RegDataB] = 0b00000000;

	// set Bank A outputs (0: output, 1: Input)
	buf[SX_1503_RegDirA] = 0b00011111;

	// set Bank B outputs (0: output, 1: Input)
	buf[SX_1503_RegDirB] = 0b11111100;

	// activate Pull-up resistors for buttons

	// activate pull-ups for Bank A inputs (0: pull up disabled, 1: pull up enabled)
	buf[SX_1503_PullUpA] = 0b00011111;

	// activate pull-ups for Bank B inputs (0: pull up disabled, 1: pull up enabled)
	buf[SX_1503_PullUpB] = 0b11111000;
	// note: HMI_BTN_ENCODER does not get a pull up because it would mess up the levels

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
	HAL_I2C_Mem_Write(myHMI->I2C_Handle, myHMI->I2C_ADDRESS, SX_1503_RegDataB,
			1, &buf[SX_1503_RegDataB], 2, HAL_MAX_DELAY);
}

// TODO read out button registers, safe to internal variables
void HMI_Read_GPIOs(HMI *myHMI){
	// Receive buffer
	uint8_t buf[2];

	/* Read registers for button states:
	 * SX_1503_RegDataB
	 * SX_1503_RegDataA
	 */
	// read register SX_1503_RegDatax
	HAL_I2C_Mem_Read(myHMI->I2C_Handle, myHMI->I2C_ADDRESS, SX_1503_RegDataB, 1,
			&buf[0], 2, HAL_MAX_DELAY);

	// assemble back 16bit result
	uint16_t result = 0x0000;
	result = buf[1];			// Bank A is lower byte of the result
	result |= (buf[0] << 8);	// Bank B is upper byte of the result

	// mask results with the buttons and store them
	HMI_BTN_WDA_STATE 		= (result & HMI_BTN_WDA)==0 		? BUTTON_NOT_PRESSED : BUTTON_PRESSED;
	HMI_BTN_OTA_STATE 		= (result & HMI_BTN_OTA)==0 		? BUTTON_NOT_PRESSED : BUTTON_PRESSED;
	HMI_BTN_TIME_DATE_STATE	= (result & HMI_BTN_TIME_DATE)==0 	? BUTTON_NOT_PRESSED : BUTTON_PRESSED;
	HMI_BTN_TIMER1_STATE 	= (result & HMI_BTN_TIMER1)==0 		? BUTTON_NOT_PRESSED : BUTTON_PRESSED;
	HMI_BTN_TIMER2_STATE 	= (result & HMI_BTN_TIMER2)==0 		? BUTTON_NOT_PRESSED : BUTTON_PRESSED;
	HMI_BTN_ENCODER_STATE 	= (result & HMI_BTN_ENCODER)==0 	? BUTTON_NOT_PRESSED : BUTTON_PRESSED;

	/* Read registers for button interrupt
	 * SX_1503_RegInterruptSourceB
	 * SX_1503_RegInterruptSourceA
	 */
	// read register SX_1503_RegInterruptSourceB
	HAL_I2C_Mem_Read(myHMI->I2C_Handle, myHMI->I2C_ADDRESS,
			SX_1503_RegInterruptSourceB, 1, &buf[0], 2, HAL_MAX_DELAY);

	// assemble back 16bit result
	result = 0x0000;
	result = buf[1];			// Bank A is lower byte of the result
	result |= (buf[0] << 8);	// Bank B is upper byte of the result

	// only consider buttons; use mask to ignore other results
	result &= 0b0000010000011111;

	// mask results with the buttons and store them
	HMI_BTN_WDA_Interrupt 		= (result & HMI_BTN_WDA) != 0		? INTERRUPT : NO_INTERRUPT;
	HMI_BTN_OTA_Interrupt 		= (result & HMI_BTN_OTA) != 0		? INTERRUPT : NO_INTERRUPT;
	HMI_BTN_TIME_DATE_Interrupt = (result & HMI_BTN_TIME_DATE) != 0	? INTERRUPT : NO_INTERRUPT;
	HMI_BTN_TIMER1_Interrupt 	= (result & HMI_BTN_TIMER1) != 0 	? INTERRUPT : NO_INTERRUPT;
	HMI_BTN_TIMER2_Interrupt 	= (result & HMI_BTN_TIMER2) != 0 	? INTERRUPT : NO_INTERRUPT;
	HMI_BTN_ENCODER_Interrupt 	= (result & HMI_BTN_ENCODER) != 0 	? INTERRUPT : NO_INTERRUPT;

	// reset interrupt register
	HMI_reset_INT(myHMI);
}

// TODO this function reads the interrupt pin. It returns the button last pressed
uint16_t HMI_Read_INT_BTN_press(HMI *myHMI) {
	// check if the interrupt is active
	if (HAL_GPIO_ReadPin(myHMI->Interrupt_PORT, myHMI->Interrupt_PIN) == 0) {

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

		// only consider buttons; use mask to ignore other results
		result &= 0b0000010000011111;

		// reset interrupt register
		HMI_reset_INT(myHMI);

		return result;
	} else {
		return 0x0000;
	}
}

// TODO this function reads the current sate of the requested button
uint8_t HMI_Read_BTN(HMI *myHMI, uint16_t button) {
	// read interrupt source:
	// Receive buffer
	uint8_t buf[2];
	// read register SX_1503_RegDatax
	HAL_I2C_Mem_Read(myHMI->I2C_Handle, myHMI->I2C_ADDRESS, SX_1503_RegDataB, 1,
			&buf[0], 2, HAL_MAX_DELAY);

	// assemble back 16bit result
	uint16_t result = 0x0000;
	result = buf[1];			// Bank A is lower byte of the result
	result |= (buf[0] << 8);	// Bank B is upper byte of the result

	// only consider requested pin; use it as mask
	result &= button;

	// return 0 fow low state and 1 for high state
	if (result == 0) {
		return 0;
	} else {
		return 1;
	}
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
	HMI_Write_LED_b(myHMI, HMI_LED_WDA, 1);
	HMI_Write_LED_b(myHMI, HMI_LED_OTA, 1);
	HMI_Write_LED_b(myHMI, HMI_LED_TIME_DATE, 1);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER1, 1);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER2, 1);

	// write buffer to activate LEDs
	HMI_Write(myHMI);
}

// TODO reset all LEDs
void HMI_reset_all_LED(HMI *myHMI) {
	// fill buffer with LED to write to
	HMI_Write_LED_b(myHMI, HMI_LED_WDA, 0);
	HMI_Write_LED_b(myHMI, HMI_LED_OTA, 0);
	HMI_Write_LED_b(myHMI, HMI_LED_TIME_DATE, 0);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER1, 0);
	HMI_Write_LED_b(myHMI, HMI_LED_TIMER2, 0);

	// write buffer to activate LEDs
	HMI_Write(myHMI);
}

void HMI_set_PWM(HMI *myHMI, uint8_t channel, uint8_t brightness) {
	switch (channel) {
	case PWM_CH_Keypad:
		TIM2->CCR2 = brightness;
		break;
	case PWM_CH_LCD:
		TIM3->CCR1 = brightness;
		break;
	case PWM_CH_LAMP:
		TIM3->CCR2 = brightness;
		break;
	default:
		// do nothing
		break;
	}

}

// TODO read encoder
int HMI_Encoder_position(HMI *myHMI) {
	// get current encoder timer value
	Encoder_current_couter = __HAL_TIM_GET_COUNTER(myHMI->EncTimer);

	// number of timer pulses between old value and new value
	int distance = 0;

	// calculate the distance between the current timer value and the last
	if (Encoder_current_couter > Encoder_last_couter) {
		// see if it was a negative overflow
		if (Encoder_current_couter > Encoder_last_couter + 40000) {
			// overflow, new value is smaller than old value
			distance = -((0xFFFF + 1 - Encoder_current_couter) + Encoder_last_couter);
		} else {
			// no overflow, new value is bigger than old value
			distance = Encoder_current_couter - Encoder_last_couter;
		}
	} else if (Encoder_current_couter < Encoder_last_couter) {
		// see if it was a positive overflow
		if (Encoder_last_couter > Encoder_current_couter + 40000) {
			// overflow, new value is bigger than old value
			distance = (0xFFFF + 1 - Encoder_last_couter) + Encoder_current_couter;
		} else {
			// no overflow, new value is smaller than old value
			distance = -(Encoder_last_couter - Encoder_current_couter);
		}

	} else if (Encoder_current_couter == Encoder_last_couter) {
		// position is unchanged
	}


	// calculate relative position to last measurement point
	Encoder_Position = distance;

	// update last counter
	Encoder_last_couter = Encoder_current_couter;
	return Encoder_Position;
}
