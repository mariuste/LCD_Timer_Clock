/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BL55072A.h"
#include "HMI.h"
#include "RV3028.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Sate Machine ###############################################################
uint8_t currentState = STATE_INITIALISATION;
uint8_t nextState = STATE_INITIALISATION;

uint8_t TIMEOUT_1 = 5;
uint8_t TIMEOUT_2 = 2;

// unix time stamp
uint32_t LastEvent = 0;

// Define external ICs ########################################################
// Human Machine Interface (buttons, led) -------------------
HMI myHMI;

// LCD interface --------------------------------------------
LCD myLCD;

// DFPlayer Data Packets:
static const uint8_t DFP_START = 0x7E;
static const uint8_t DFP_VER = 0xFF;
static const uint8_t DFP_LEN = 0x06;
static const uint8_t DFP_noFB = 0x00;
static const uint8_t DFP_STOP = 0xEF;

// RTC RV-3028 --------------------------------------------
RV3028 myRTC;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef DFP_Send_CMD(uint8_t cmd, uint8_t payload1, uint8_t payload0) {
	// calculate CRC
	uint16_t DFT_CRC = 0x00;
	DFT_CRC = DFT_CRC - DFP_VER - DFP_LEN - cmd - DFP_noFB - payload1
			- payload0;
	// assemble transmission buffer
	uint8_t UART_buf[10] = { DFP_START, DFP_VER, DFP_LEN, cmd, DFP_noFB,
			payload1, payload0, DFT_CRC >> 8, DFT_CRC, DFP_STOP }; // Perform Reset

	// transmit packet
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_Reset() {
	uint8_t UART_buf[10] = { 0x7E, 0xFF, 0x06, 0x0C, 0x00, 0x00, 0x00, 0xFE,
			0xEF, 0xEF }; // Perform Reset
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_SetToSD() {
	uint8_t UART_buf[10] = { 0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE,
			0xF0, 0xEF }; // Specify micro SD
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_setVolume() {
	// for now fixed to 25%
	uint8_t UART_buf[10] = { 0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE,
			0xF0, 0xEF }; // Specify micro SD
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	// Setup periphery ########################################################
	// Setup Port Expander -------------------------------------------
	// Initialize Port Expander SX1503
	HMI_Setup(&myHMI, 		// SX1503 object
			&hi2c2,				// I2C Handle
			nI_O_INT_GPIO_Port,	// Interrupt pin port
			nI_O_INT_Pin,		// Interrupt pin
			&htim1);

	// SET Inputs and Outputs to the default configuration (reset)
	HMI_defaultConfig(&myHMI);

	// Setup LCD ---------------------------------------------------
	LCD_Setup(&myLCD, 	// SX1503 object
			&hi2c2		// I2C Handle
			);

	// disable MP3 Player
	HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, 0);

	// Setup LED Lights ###########################################
	// set brightness to 0 before starting the PWM timers
	HMI_set_PWM(&myHMI, PWM_CH_Keypad, 0);
	HMI_set_PWM(&myHMI, PWM_CH_LCD, 0);
	HMI_set_PWM(&myHMI, PWM_CH_LAMP, 0);

	// start PWM Timers
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // PWM_CH_Keypad
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PWM_CH_LCD
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PWM_CH_LAMP

	// Setup Encoder #############################################
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// Setup RTC #################################################
	// Initialize RTC RV3028
	RTC_Setup(&myRTC,	 		// RV3028 handle
			&hi2c2,				// I2C Handle
			RTC_INT_GPIO_Port,	// Interrupt pin port
			RTC_INT_Pin			// Interrupt pin
			);

	// Test Player:
	/*
	 HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, 1); // power mp3 player
	 HAL_Delay(2000); // wait for startup

	 DFP_Send_CMD(0x06, 0x00, 0x14); // set volume to 20

	 DFP_Send_CMD(0x12, 0x00, 0x01); // play track 1 in folder mp3
	 */

	// Test BL550072A
	// initialize
	LCD_INIT(&myLCD);

	LCD_Enable(&myLCD);

	LCD_Segment_AllOn(&myLCD);

	HAL_Delay(1000);

	LCD_Segment_normal(&myLCD);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		// Read RTC
		RTC_Get_Time(&myRTC);

		// State Machine:
		switch (nextState) {

		case STATE_INITIALISATION:
			// state newly entered; reset event timeout timer
			LastEvent = RTC_UNIX_TIME;

			// nothing to do; next state:
			nextState = STATE_STANDBY;

			break;

		case STATE_STANDBY: // ################################################
			// A: One time operations when a state is newly entered -----------
			if (nextState != currentState) {
				// state newly entered; reset event timeout timer
				LastEvent = RTC_UNIX_TIME;

				// One time setup finished
				currentState = nextState;
			}

			// B: Normal operations of the state ------------------------------

			// display current seconds
			LCD_Write_Number(&myLCD, 0, RTC_Second, 1);
			// display current state
			LCD_Write_Number(&myLCD, 1, currentState, 2);
			LCD_SendBuffer(&myLCD);


			// C: conditions for changing the state ---------------------------

			// check buttons
			uint16_t button = HMI_Read_INT_BTN_press(&myHMI);
			if ((button & HMI_BTN_TIME_DATE) != 0x0000) {
				// Time Date Button pressed
				nextState = STATE_STANDBY_ILUM;
			}

			// D: timeout conditions ------------------------------------------

			// none, this is the default state

			break;

		case STATE_STANDBY_ILUM: // ###########################################
			// A: One time operations when a state is newly entered -----------
			if (nextState != currentState) {
				// state newly entered; reset event timeout timer
				LastEvent = RTC_UNIX_TIME;

				// One time setup finished
				currentState = nextState;
			}

			// B: Normal operations of the state ------------------------------

			// display current seconds
			LCD_Write_Number(&myLCD, 0, RTC_Second, 1);
			// display current state
			LCD_Write_Number(&myLCD, 1, currentState, 2);
			LCD_SendBuffer(&myLCD);

			// enable LEDs
			HMI_set_PWM(&myHMI, PWM_CH_LCD, 5);

			// check buttons
			button = HMI_Read_INT_BTN_press(&myHMI);

			if ((button & HMI_BTN_TIME_DATE) != 0x0000) {
				// reset event timeout timer
				LastEvent = RTC_UNIX_TIME;
			}

			// C: conditions for changing the state ---------------------------

			// TBD

			// D: timeout conditions ------------------------------------------

			// check timeout
			if (RTC_UNIX_TIME > LastEvent + 5) {
				// timeout reached
				//deactivate LED
				HMI_set_PWM(&myHMI, PWM_CH_LCD, 0);
				//return to other state
				nextState = STATE_STANDBY;
			}

			break;

		}

		HAL_Delay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV8;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00000103;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 10 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 100 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 10 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : nI_O_INT_Pin */
	GPIO_InitStruct.Pin = nI_O_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(nI_O_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DFP_Audio_en_Pin */
	GPIO_InitStruct.Pin = DFP_Audio_en_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DFP_Audio_en_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : RTC_INT_Pin */
	GPIO_InitStruct.Pin = RTC_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RTC_INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

