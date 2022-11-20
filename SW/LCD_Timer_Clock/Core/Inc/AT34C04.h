/**
 ******************************************************************************
 * @file           AT34C04.h
 * @brief          Header file for AT34C04.c
 ******************************************************************************
 * Created on: 18.12.2021
 * Author: 	Marius Tetard
 */


#ifdef EXAMPLE_CODE

/*
 * Example Usage ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */


/* USER CODE BEGIN Includes */

// EEPROM
#include "AT34C04.h"

/* USER CODE END Includes */

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* USER CODE BEGIN PV */

// EEPROM Object
AT34C04 myAT34C04;

/* USER CODE END PV */

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* USER CODE BEGIN 2 */

// Initialize EEPROM
AT34C04_Initialize(
		&myAT34C04, // EEPROM object
		0x0,		// Address pin A0 value
		0x0,		// Address pin A1 value
		0x0,		// Address pin A2 value
		&hi2c2		// I2C Handle
);

/* USER CODE END 2 */

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* USER CODE BEGIN WHILE */

// Variables
uint8_t myVirtualRegister = 0x10;

// Read_From EEPROM
uint8_t a8 = 0x00;
uint16_t a16 = 0x0000;
uint32_t a32 = 0x000000;
float f32 = 0.0f;


AT34C04_Read_VReg_unit8(&myAT34C04, myVirtualRegister, &a8);
AT34C04_Read_VReg_unit16(&myAT34C04, myVirtualRegister, &a16);
AT34C04_Read_VReg_unit32(&myAT34C04, myVirtualRegister, &a32);
AT34C04_Read_VReg_float(&myAT34C04, myVirtualRegister, &f32);

// Write to EEPROM
// TBD


/* USER CODE END WHILE */

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif


#ifndef INC_AT34C04_H_
#define INC_AT34C04_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32L4:
// #include "stm32l4xx_hal.h" // needed for unit types
#include "stm32g0xx_hal.h"

/**
 * @defgroup AT34C04_GROUP AT34C04 EEPROM commands and defines
 *
 * @{
 */
#define BaseI2CAddress1		0xA0 /**< I2C Base Address (already shifted into 8Bit format) */

// For setting Page Address RPA
#define CMD_ARRAY1  		0x6C;	/**< Command to select Array 1 */
#define CMD_ARRAY2   		0x6D;	/**< Command to select Array 2 */

#define ARRAY1				0x01	/**< ID of Array 1 */
#define ARRAY2				0x02	/**< ID of Array 2 */


#define MiniumWriteDelay	5 		/**< Delay after each write command in ms */

#define ADRESSLENGTH 		1		/**< Length of I2C commands in Bytes */
#define ByterPerPage		16		/**< Number of EEPROM Registers per Page */
#define PagesPerQuadrant	8		/**< Number of EEPROM Pages per Quadrant */
#define QuadrantsPerArray	2		/**< Number of EEPROM Quadrants per Array */
#define NoOfArray			2		/**< Number of EEPROM Array per EEPROM IC */

#define VRegSize			4		/**< Size of Virtual Register VREG in Bytes */
#define MaxVReg				(ByterPerPage*PagesPerQuadrant*QuadrantsPerArray*NoOfArray)/RegSize /**< Number of Virtual Registers VREG*/
/** @} */

/**
 * @struct  AT34C04
 * @brief Structure for AT34C04 EEPROM driver. It is used to configure and use the I2C EEPROM
 *
 */
typedef struct {
	uint8_t EEPROM_ADDRESS;			/**< I2C Address of EEPROM*/
	I2C_HandleTypeDef *EEPROM_I2C;	/**< I2C Interface Handle */
	uint32_t lastWrite;				/**< time since last EEOROM Write action */
} AT34C04;

// Return value
//HAL_StatusTypeDef ret;

/*
 * General Functions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

/*
 * INITIALIZATION
 */
HAL_StatusTypeDef AT34C04_Initialize(
		AT34C04 *myAT34C04,
		uint8_t EEPROM_A0,
		uint8_t EEPROM_A1,
		uint8_t EEPROM_A2,
		I2C_HandleTypeDef *EEPROM_I2C
);
/*
 * Write virtual register in various formats
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit8(AT34C04 *myAT34C04, uint8_t VREG, uint8_t *data);
HAL_StatusTypeDef AT34C04_Write_VReg_unit16(AT34C04 *myAT34C04, uint8_t VREG, uint16_t *data);
HAL_StatusTypeDef AT34C04_Write_VReg_unit32(AT34C04 *myAT34C04, uint8_t VREG, uint32_t *data);
HAL_StatusTypeDef AT34C04_Write_VReg_float(AT34C04 *myAT34C04, uint8_t VREG, float *data);
/*
 * Read virtual register in various formats
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit8(AT34C04 *myAT34C04, uint8_t VREG, uint8_t *data);
HAL_StatusTypeDef AT34C04_Read_VReg_unit16(AT34C04 *myAT34C04, uint8_t VREG, uint16_t *data);
HAL_StatusTypeDef AT34C04_Read_VReg_unit32(AT34C04 *myAT34C04, uint8_t VREG, uint32_t *data);
HAL_StatusTypeDef AT34C04_Read_VReg_float(AT34C04 *myAT34C04, uint8_t VREG, float *data);


/*
 * LOW LEVEL FUNCTIONS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// EEPROM Read/Write
HAL_StatusTypeDef AT34C04_MEM_Read(AT34C04 *myAT34C04, uint8_t EEPROMRegister, uint8_t *pData, uint8_t size);
HAL_StatusTypeDef AT34C04_MEM_Write(AT34C04 *myAT34C04, uint8_t EEPROMRegister, uint8_t *pData, uint8_t size);

// Set Array 1 or 2 and read it back; ARRAY1 or ARRAY2
HAL_StatusTypeDef AT34C04_Set_Array(AT34C04 *myAT34C04, uint8_t arrayNo);
uint8_t AT34C04_Get_Array(AT34C04 *myAT34C04);

// Covert Virtual register to actual register
uint8_t VREG_to_REG(uint8_t VREG);

// Convert Virtual register to array
uint8_t VREG_to_ARRAY(uint8_t VREG);

// Ensure the minimum delay after a EEPROM write operation
void AT34C04_ensure_min_write_delay(AT34C04 *myAT34C04);


#endif /* INC_AT34C04_H_ */