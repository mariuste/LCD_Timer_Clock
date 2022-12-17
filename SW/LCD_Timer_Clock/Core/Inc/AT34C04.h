/**
 ******************************************************************************
 * @file           AT34C04.h
 * @brief          Header file for AT34C04.c
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
 * Created on: 18.12.2021
 * Author: 	Marius Tetard
 */

#ifndef INC_AT34C04_H_
#define INC_AT34C04_H_

/*
 * Defines / Variables ++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
// For STM32L4:
// #include "stm32l4xx_hal.h" // needed for unit types
#include "stm32g0xx_hal.h"
// TODO: generalize include for units so not every STM needs their own special include

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


// Doxygen Group start:
/** @name EEPROM General Functions
 */
/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_Initialize(AT34C04*, uint8_t, uint8_t, uint8_t, I2C_HandleTypeDef*)
 * 
 * Initialize the I2C EEPROM IC
 * 
 * Sets up the I2C address based on the provided Address Pins. Additionally a
 * simple read test is performed and returns the HAL response.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Initialize(
		AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
		uint8_t EEPROM_A0, /**< EEPROM_A1 Value of address pin 1 */
		uint8_t EEPROM_A1, /**< EEPROM_A2 Value of address pin 2 */
		uint8_t EEPROM_A2, /**< EEPROM_I2C Pointer to the I2C handle */
		I2C_HandleTypeDef *EEPROM_I2C /**< Pointer to the I2C handle */
);

/**@}*/
// Doxygen Group end

// Doxygen Group start:
/** @name Public EEPROM Write Functions
 */

/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_unit8(AT34C04*, uint8_t, uint8_t*)
 *
 * Write a uint8_t value to an VREG EEPROM address
 *
 * This function allows to write a single uint8_t value to a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit8(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    uint8_t *data /**< Data to be written to the EEPROM */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_unit16(AT34C04*, uint8_t, uint16_t*)
 *
 * Write a uint16_t value to an VREG EEPROM address
 *
 * This function allows to write a single uint16_t value to a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit16(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    uint16_t *data /**< Data to be written to the EEPROM */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_unit32(AT34C04*, uint8_t, uint32_t*)
 *
 * Write a uint32_t value to an VREG EEPROM address
 *
 * This function allows to write a single uint32_t value to a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Write_VReg_unit32(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    uint32_t *data /**< Data to be written to the EEPROM */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Write_VReg_float(AT34C04*, uint8_t, float*)
 *
 * Write a float value to an VREG EEPROM address
 *
 * This function allows to write a single float value to a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Write_VReg_float(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    float *data /**< Data to be written to the EEPROM */
);

/**@}*/
// Doxygen Group end


// Doxygen Group start:
/** @name Public EEPROM Read Functions
 */
/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_unit8(AT34C04*, uint8_t, uint8_t*)
 *
 * Read a uint8_t value from an VREG EEPROM address
 * 
 * This function allows to read a single uint8_t value from a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit8(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    uint8_t *data /**< Data to be read from the EEPROM */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_unit16(AT34C04*, uint8_t, uint16_t*)
 *
 * Read a uint16_t value from an VREG EEPROM address
 * 
 * This function allows to read a single uint16_t value from a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit16(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    uint16_t *data /**< Data to be read from the EEPROM */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_unit32(AT34C04*, uint8_t, uint32_t*)
 *
 * Read a uint32_t value from an VREG EEPROM address
 * 
 * This function allows to read a single uint32_t value from a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Read_VReg_unit32(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    uint32_t *data /**< Data to be read from the EEPROM */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Read_VReg_float(AT34C04*, uint8_t, float*)
 *
 * Read a float value from an VREG EEPROM address
 * 
 * This function allows to read a single float value from a specified
 * VREG address. The corresponding array and PREG are set automatically.
 *
 * Page/Array breaks can be ignored because the PREGs are broken up in a way
 * to make page and array breaks unnecessary.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Read_VReg_float(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t VREG, /**< Virtual Register Address */
    float *data /**< Data to be read from the EEPROM */
);

/**@}*/
// Doxygen Group end



// Doxygen Group start:
/** @name Private EEPROM Low Level Functions
 */
/**@{*/

/**
 * @fn HAL_StatusTypeDef AT34C04_MEM_Read(AT34C04*, uint8_t, uint8_t*, uint8_t)
 *
 * Low Level Function to read EEPROM registers
 * 
 * This function is an simplified abstraction of the HAL_I2C_Mem_Read function.
 * The minimum delay to the last EEPROM write is ensured.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_MEM_Read(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t EEPROMRegister, /**< Physical Register which will be read */
    uint8_t *pData, /**< Pointer to data buffer to store read data */
    uint8_t size /**< Number of bytes to be read from the EEPROM (Note that only a single page break is possible) */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_MEM_Write(AT34C04*, uint8_t, uint8_t*, uint8_t)
 *
 * Low Level Function to write EEPROM registers
 * 
 * This function is an simplified abstraction of the HAL_I2C_Mem_Write function.
 * The minimum delay to the last EEPROM write is ensured.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_MEM_Write(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t EEPROMRegister, /**< Physical Register which will be written to */
    uint8_t *pData, /**< Pointer to data buffer with the data to be written */
    uint8_t size /**< Number of bytes to be written to the EEPROM (Note that only a single page break is possible) */
);

/**
 * @fn HAL_StatusTypeDef AT34C04_Set_Array(AT34C04*, uint8_t)
 *
 * Low level function to set the EEPROM ARRAY
 * 
 * This function selects the specified array in the AT34C04 EEPROM.
 * The minimum delay to the last EEPROM write is ensured.
 *
 * @return HAL response of the I2C Read/Write function
 * 
 */
HAL_StatusTypeDef AT34C04_Set_Array(
    AT34C04 *myAT34C04, /**< Pointer to the EEPROM handle */
    uint8_t arrayNo /**< Selected ARRAY: "ARRAY1" or "ARRAY2" */
);

/**
 * @fn uint8_t AT34C04_Get_Array(AT34C04*)
 * 
 * Low level function to get the currently set EEPROM ARRAY
 * 
 * This function reads the the currently selected array in the AT34C04 EEPROM
 * and returns it as return value.
 *
 * The minimum delay to the last EEPROM write is ensured.
 * 
 * @return Currently set ARRAY: "ARRAY1" or "ARRAY2"
 * 
 */
uint8_t AT34C04_Get_Array(
    AT34C04 *myAT34C04 /**<  */
);

/**
 * @fn uint8_t VREG_to_REG(uint8_t)
 * 
 * Conversion function: VREG -> PREG
 *
 * This conversion function calculates a physical EEPROM Register
 * address from a given virtual register address
 *
 * @return Physical EEPROM Address
 * 
 */
uint8_t VREG_to_REG(
    uint8_t VREG /**<  */
);

/**
 * @fn uint8_t VREG_to_ARRAY(uint8_t)
 * 
 * Conversion function: VREG -> Array
 * 
 * This conversion function calculates a physical Array Number
 * based on a given virtual register address
 * 
 * @return ARRAY Number: "ARRAY1" or "ARRAY2"
 * 
 */
uint8_t VREG_to_ARRAY(
    uint8_t VREG /**<  */
);

/**
 * @fn void AT34C04_ensure_min_write_delay(AT34C04*)
 * 
 * Enforce minimum time delay after last EEPROM write operation
 * 
 * This function enforces the minimum time delay after last EEPROM write operation.
 * It only requires the amount of time necessary to fulfill the minimum delay
 * specification.
 *
 * If there is a timer overflow of the SysTick timer a complete minimum delay time
 * is added. This may need slightly more time but simplifies the function.
 */
void AT34C04_ensure_min_write_delay(
    AT34C04 *myAT34C04 /**<  */
);

/**@}*/
// Doxygen Group end


#endif /* INC_AT34C04_H_ */
