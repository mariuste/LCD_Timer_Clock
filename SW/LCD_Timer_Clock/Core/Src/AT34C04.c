/**
 ******************************************************************************
 * @file           AT34C04.c
 * @brief          Driver for AT34C04 I2C EEPROM
 *
 * This is a simplified driver for the AT34C04 I2C EEPROM. It offers the
 * functionality to safe some data types and retrieve them.
 *
 * This driver abstracts the Physical EEPROM Registers into virtual registers at
 * the cost of storage efficiency.
 *
 * Physical EEPROM Registers - "PREG"\n
 * Virtual EEPROM Registers - "VREG"
 *
 * It accepts a single register number and splits it automatically to
 * quadrants and pages.
 *
 * It is application specific and splits the storage
 * into 4 Byte virtual registers.
 *
 * It offers functions to safe
 * the data types unint8_t uint16_t, uint32_t and floats
 *
 * HW Level:\n
 * . 2 x ARRAY\n
 * .. with 2 Quadrants each\n
 * ... with 8 Pages each\n
 * .... with 16 Bytes each\n
 * . in total 512 Bytes\n
 *
 ******************************************************************************
 * Created on: 19.12.2021
 * Author: 	Marius Tetard
 */

#include "AT34C04.h"

// variables
HAL_StatusTypeDef ret;

// Doxygen Group start:
/** @name EEPROM General Functions
 */
/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_Initialize(AT34C04*, uint8_t, uint8_t, uint8_t, I2C_HandleTypeDef*)
 * @brief Initialize the I2C EEPROM IC
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param EEPROM_A0 Value of address pin 0
 * @param EEPROM_A1 Value of address pin 1
 * @param EEPROM_A2 Value of address pin 2
 * @param EEPROM_I2C Pointer to the I2C handle
 * @return HAL response based on simple register read test
 */
HAL_StatusTypeDef AT34C04_Initialize(
		AT34C04 *myAT34C04,
		uint8_t EEPROM_A0,
		uint8_t EEPROM_A1,
		uint8_t EEPROM_A2,
		I2C_HandleTypeDef *EEPROM_I2C
)
{
	/* initialize SysTic Timer */
	myAT34C04->lastWrite = HAL_GetTick();

	/* Store I2C Handle */
	myAT34C04->EEPROM_I2C = EEPROM_I2C;

	/* Calculate and set I2C address */

	/* Base I2C Address in 8Bit format is 0xA0. It can be modified with
	 * A0 (BIT 1), A1 (BIT 2) and A2 (BIT3) */

	// Address modifier:
	uint8_t addressMod = 0x00;
	if (EEPROM_A0 == 1) {
		addressMod |= 0b00000010;
	}
	if (EEPROM_A1 == 1) {
		addressMod |= 0b00000100;
	}
	if (EEPROM_A2 == 1) {
		addressMod |= 0b00001000;
	}


	myAT34C04->EEPROM_ADDRESS = BaseI2CAddress1 | addressMod;

	// Test read a register and return HAL Response
	uint8_t test;
	return AT34C04_Read_VReg_unit8(myAT34C04, 0x00, &test);
}

/**@}*/
// Doxygen Group end



// Doxygen Group start:
/** @name Public EEPROM Write Functions
 */

/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_unit8(AT34C04*, uint8_t, uint8_t*)
 * @brief Write a uint8_t value to to an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be written to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit8(AT34C04 *myAT34C04, uint8_t VREG, uint8_t *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	// read memory and return HAL status
	return AT34C04_MEM_Write(myAT34C04, VREG_to_REG(VREG), &data[0], 1);
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_unit16(AT34C04*, uint8_t, uint16_t*)
 * @brief Write a uint16_t value to to an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be written to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit16(AT34C04 *myAT34C04, uint8_t VREG, uint16_t *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	// convert one uint16_t into two uint8_t
	uint8_t EEPROMbuf[2];
	EEPROMbuf[0] = data[0] & 0xFF;	// set LSByte
	EEPROMbuf[1] = data[0] >> 8;	// set MSbyte

	// read memory and return HAL status
	HAL_StatusTypeDef ret = AT34C04_MEM_Write(myAT34C04, VREG_to_REG(VREG), &EEPROMbuf[0], 2);

	return ret;
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_unit32(AT34C04*, uint8_t, uint32_t*)
 * @brief Write a uint32_t value to to an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be written to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit32(AT34C04 *myAT34C04, uint8_t VREG, uint32_t *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	// convert one uint132_t into four uint8_t
	uint8_t EEPROMbuf[4];
	EEPROMbuf[0] = data[0] & 0xFF;	// set LSByte
	EEPROMbuf[1] = data[0] >> 8;	//
	EEPROMbuf[2] = data[0] >> 16;	//
	EEPROMbuf[3] = data[0] >> 24;	// set MSbyte

	// read memory and return HAL status
	HAL_StatusTypeDef ret = AT34C04_MEM_Write(myAT34C04, VREG_to_REG(VREG), &EEPROMbuf[0], 4);

	return ret;
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_float(AT34C04*, uint8_t, float*)
 * @brief Write a float value to to an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be written to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Write_VReg_float(AT34C04 *myAT34C04, uint8_t VREG, float *data)
{
	// convert Float to four uint8_t
	uint8_t EEPROMbuf[4];
	*((float *)EEPROMbuf) = *data;


	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));


	// read memory and return HAL status
	HAL_StatusTypeDef ret = AT34C04_MEM_Write(myAT34C04, VREG_to_REG(VREG), &EEPROMbuf[0], 4);

	return ret;
}

/**@}*/
// Doxygen Group end



// Doxygen Group start:
/** @name Public EEPROM Read Functions
 */
/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_unit8(AT34C04*, uint8_t, uint8_t*)
 * @brief Read a uint8_t value from an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be read to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit8(AT34C04 *myAT34C04, uint8_t VREG, uint8_t *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	// read memory and return HAL status
	return AT34C04_MEM_Read(myAT34C04, VREG_to_REG(VREG), &data[0], 1);
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_unit16(AT34C04*, uint8_t, uint16_t*)
 * @brief Read a uint16_t value from an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be read to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit16(AT34C04 *myAT34C04, uint8_t VREG, uint16_t *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	uint8_t EEPROMbuf[2];

	// read memory and return HAL status
	HAL_StatusTypeDef ret = AT34C04_MEM_Read(myAT34C04, VREG_to_REG(VREG), &EEPROMbuf[0], 2);

	// convert two uint8_t into one uint16_t
	uint16_t merged_byte = 0x0000;
	merged_byte |= EEPROMbuf[0];		// get LSByte
	merged_byte |= EEPROMbuf[1] << 8;	// get MSByte

	data[0] = merged_byte;

	return ret;
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_unit32(AT34C04*, uint8_t, uint32_t*)
 * @brief Read a uint32_t value from an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be read to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit32(AT34C04 *myAT34C04, uint8_t VREG, uint32_t *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	uint8_t EEPROMbuf[4];

	// read memory and return HAL status
	HAL_StatusTypeDef ret = AT34C04_MEM_Read(myAT34C04, VREG_to_REG(VREG), &EEPROMbuf[0], 4);

	// convert four uint8_t into one uint32_t
	uint32_t merged_byte = 0x00000000;
	merged_byte |= EEPROMbuf[0];		// get LSByte
	merged_byte |= EEPROMbuf[1] << 8;	//
	merged_byte |= EEPROMbuf[2] << 16;	//
	merged_byte |= EEPROMbuf[3] << 24;	// get MSByte

	data[0] = merged_byte;

	return ret;
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_float(AT34C04*, uint8_t, float*)
 * @brief Read a float value from an VREG EEPROM address
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param VREG Virtual Register Address
 * @param data Data to be read to the EEPROM
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Read_VReg_float(AT34C04 *myAT34C04, uint8_t VREG, float *data)
{
	// set corresponding array
	AT34C04_Set_Array(myAT34C04, VREG_to_ARRAY(VREG));

	uint8_t EEPROMbuf[4];

	// read memory and return HAL status
	HAL_StatusTypeDef ret = AT34C04_MEM_Read(myAT34C04, VREG_to_REG(VREG), &EEPROMbuf[0], 4);

	// convert four uint8_t into one float
	typedef unsigned char uchar;
	float merged_byte;
	*((uchar*)(&merged_byte) + 0) = EEPROMbuf[0]; // get LSByte
	*((uchar*)(&merged_byte) + 1) = EEPROMbuf[1];
	*((uchar*)(&merged_byte) + 2) = EEPROMbuf[2];
	*((uchar*)(&merged_byte) + 3) = EEPROMbuf[3]; // get MSByte

	// set provided float value
	data[0] = merged_byte;

	return ret;
}

/**@}*/
// Doxygen Group end



// Doxygen Group start:
/** @name Private EEPROM Low Level Functions
 */
// TODO make actually private
/**@{*/
/**
 * @fn HAL_StatusTypeDef AT34C04_MEM_Read(AT34C04*, uint8_t, uint8_t*, uint8_t)
 * @brief Low Level Function to read EEPROM registers
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param EEPROMRegister Physical Register which will be read
 * @param pData Pointer to data buffer to store read data
 * @param size Number of bytes to be read from the EEPROM (Note that only a single page break is possible)
 * @return HAL response of the I2C Read function
 */
HAL_StatusTypeDef AT34C04_MEM_Read(AT34C04 *myAT34C04, uint8_t EEPROMRegister, uint8_t *pData, uint8_t size)
{
	// ensure that enough time has passed since last write command
	AT34C04_ensure_min_write_delay(myAT34C04);

	ret = HAL_I2C_Mem_Read(
			myAT34C04->EEPROM_I2C,
			myAT34C04->EEPROM_ADDRESS,
			EEPROMRegister,
			ADRESSLENGTH,
			&pData[0],
			size,
			HAL_MAX_DELAY);
	return ret;
}

/**
 * @fn HAL_StatusTypeDef AT34C04_MEM_Write(AT34C04*, uint8_t, uint8_t*, uint8_t)
 * @brief Low Level Function to write EEPROM registers
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param EEPROMRegister Physical Register which will be read
 * @param pData Pointer to data buffer with the data to be written
 * @param size Number of bytes to be written to the EEPROM (Note that only a single page break is possible)
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_MEM_Write(AT34C04 *myAT34C04, uint8_t EEPROMRegister, uint8_t *pData, uint8_t size)
{
	// ensure that enough time has passed since last write command
	AT34C04_ensure_min_write_delay(myAT34C04);

	// all tests passed, update the lastWrite Time
	myAT34C04->lastWrite = HAL_GetTick();

	// write to EEPROM
	ret = HAL_I2C_Mem_Write(
			myAT34C04->EEPROM_I2C,
			myAT34C04->EEPROM_ADDRESS,
			EEPROMRegister,
			ADRESSLENGTH,
			&pData[0],
			size,
			HAL_MAX_DELAY);
	return ret;
}

/**
 * @fn HAL_StatusTypeDef AT34C04_Set_Array(AT34C04*, uint8_t)
 * @brief Low level function to set the EEPROM ARRAY
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @param arrayNo Selected ARRAY: "ARRAY1" or "ARRAY2"
 * @return HAL response of the I2C Write function
 */
HAL_StatusTypeDef AT34C04_Set_Array(AT34C04 *myAT34C04, uint8_t arrayNo)
{
	// ensure that enough time has passed since last write command
	AT34C04_ensure_min_write_delay(myAT34C04);

	uint8_t CMD_SET_ARRAYx = 0;
	if (arrayNo == ARRAY1) {
		CMD_SET_ARRAYx = CMD_ARRAY1;
	} else if (arrayNo == ARRAY2) {
		CMD_SET_ARRAYx = CMD_ARRAY2;
	}

	uint8_t dummyDataSend[2];
	ret = HAL_I2C_Master_Transmit(
			myAT34C04->EEPROM_I2C,
			CMD_SET_ARRAYx,
			&dummyDataSend[0],
			2,
			HAL_MAX_DELAY);
	return ret;
}

/**
 * @fn uint8_t AT34C04_Get_Array(AT34C04*)
 * @brief Low level function to get the currently set EEPROM ARRAY
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 * @return Currently set ARRAY: "ARRAY1" or "ARRAY2"
 */
uint8_t AT34C04_Get_Array(AT34C04 *myAT34C04)
{
	// ensure that enough time has passed since last write command
	AT34C04_ensure_min_write_delay(myAT34C04);

	// Read Page Address RPA - determines in which array we are
	uint8_t dummyData[2];
	ret = HAL_I2C_Master_Receive(
			myAT34C04->EEPROM_I2C,
			0x6D,				// Read Array
			&dummyData[0],
			2,
			HAL_MAX_DELAY);
	if (ret == HAL_OK) {
		return ARRAY1;
	} else {
		return ARRAY2;
	}
}

/**
 * @fn uint8_t VREG_to_REG(uint8_t)
 * @brief Conversion function: VREG -> PREG
 *
 * @param VREG Virtual Register Address
 * @return Physical EEPROM Address
 */
uint8_t VREG_to_REG(uint8_t VREG) {
	// convert virtual register into EEPROM register
	if (VREG < (uint8_t)64) {
		return VREG * 4;
	} else if (VREG < (uint8_t)128) {
		return VREG * 4 - 256;
	} else {
		return -1;
	}
}

/**
 * @fn uint8_t VREG_to_ARRAY(uint8_t)
 * @brief Conversion function: VREG -> Array
 *
 * @param VREG Virtual Register Address
 * @return ARRAY Number: "ARRAY1" or "ARRAY2"
 */
uint8_t VREG_to_ARRAY(uint8_t VREG) {
	// convert virtual register into array & EEPROM register
	if (VREG < (uint8_t)64) {
		return ARRAY1;
	} else if (VREG < (uint8_t)128) {
		return ARRAY2;
	} else {
		return -1;
	}
}

/**
 * @fn void AT34C04_ensure_min_write_delay(AT34C04*)
 * @brief Enforce minimum time delay after last EEPROM write operation
 *
 * @param myAT34C04 Pointer to the I2C EEPROM handle
 */
void AT34C04_ensure_min_write_delay(AT34C04 *myAT34C04) {
	// get current time
	uint32_t current_time = HAL_GetTick();

	// if there has be an overflow -> ignore
	if(current_time < myAT34C04->lastWrite) {
		// timer overflow, ignore and enforce delay
		myAT34C04->lastWrite = current_time;
		HAL_Delay(MiniumWriteDelay);
	} else {
		// no timer overflow, ensure that the minimum time has passed
		while(current_time < myAT34C04->lastWrite + MiniumWriteDelay){
			// wait a millisecond, no reason to max out the MCU Core
			HAL_Delay(1);
			// update time
			current_time = HAL_GetTick();
		}
	}
}
/**@}*/
// Doxygen Group end
