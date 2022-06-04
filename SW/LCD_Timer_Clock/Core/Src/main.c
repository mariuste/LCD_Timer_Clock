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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


// Port Expander Test
static const uint8_t SX1503_ADDR = 0x20 << 1; // Use 8-bit address
static const uint8_t SX_1503_RegDataB = 0x00; // register address
static const uint8_t SX_1503_RegDataA = 0x01; // register address
static const uint8_t SX_1503_RegDirB = 0x02; // register address
static const uint8_t SX_1503_RegDirA = 0x03; // register address


// LCD BL55072A constants
static const uint8_t BL5502_ADDR = 0x7C; // Use 8-bit address


// Display Buffer
uint8_t BL5502_BUFF[23];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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

void allHMILEds_set() {
	uint8_t buf[12]; // transmission buffer

	// set pins of Bank A (I/O5, I/O6, I/O7) to high
	buf[0] = SX_1503_RegDataA;
	buf[1] = 0b11100000;
	HAL_I2C_Master_Transmit(&hi2c2, SX1503_ADDR, buf, 2, HAL_TIMEOUT);

	// set pins of Bank B (I/O8, I/O9) to high
	buf[0] = SX_1503_RegDataB;
	buf[1] = 0b00000011;
	HAL_I2C_Master_Transmit(&hi2c2, SX1503_ADDR, buf, 2, HAL_TIMEOUT);
}

void allHMILEds_reset() {
	uint8_t buf[12]; // transmission buffer

	// set pins of Bank A (I/O5, I/O6, I/O7) to low
	buf[0] = SX_1503_RegDataA;
	buf[1] = 0b00000000;
	HAL_I2C_Master_Transmit(&hi2c2, SX1503_ADDR, buf, 2, HAL_TIMEOUT);

	// set pins of Bank B (I/O8, I/O9) to low
	buf[0] = SX_1503_RegDataB;
	buf[1] = 0b00000000;
	HAL_I2C_Master_Transmit(&hi2c2, SX1503_ADDR, buf, 2, HAL_TIMEOUT);
}



HAL_StatusTypeDef DFP_Reset() {
	uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x0C, 0x00, 0x00, 0x00, 0xFE, 0xEF, 0xEF}; // Perform Reset
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_SetToSD() {
	uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE, 0xF0, 0xEF}; // Specify micro SD
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

HAL_StatusTypeDef DFP_setVolume() {
	// for now fixed to 25%
	uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE, 0xF0, 0xEF}; // Specify micro SD
	return HAL_UART_Transmit(&huart2, UART_buf, 10, 250);
}

// EA reset
// ICSET


HAL_StatusTypeDef LCD_INIT() {
	uint8_t buf[6]; // transmission buffer

	buf[0]= 0xFF; // Set all pixels off
	buf[1]= 0xC8; // Set display on
	buf[2]= 0xEA; // Software reset
	buf[3]= 0xB6; // Set power save mode
	buf[4]= 0xE8; // Set msb of ram address
	buf[5]= 0x00; // Set ram address

	for(int i=6;i<24;i++)
			{
			buf[i] = 0xFF;//
			}

	// send initialization
	return HAL_I2C_Master_Transmit(&hi2c2, BL5502_ADDR, (uint8_t *)buf, 24, 100);
}

HAL_StatusTypeDef LCD_Enable() {
	uint8_t buf[4]; // transmission buffer

	buf[0]= 0xB6; // Set power save mode
	buf[1]= 0xF0; // Set blink
	buf[2]= 0xFC; // Close all pixels on/off function
	buf[3]= 0xC8; // Set display on

	// send initialization
	return HAL_I2C_Master_Transmit(&hi2c2, BL5502_ADDR, (uint8_t *)buf, 4, 100);
}

HAL_StatusTypeDef LCD_Write() {
	uint8_t buf[4]; // transmission buffer

	buf[0]= 0xB6; // Set power save mode
	buf[1]= 0xF0; // Set blink
	buf[2]= 0xFC; // Close all pixels on/off function
	buf[3]= 0xC8; // Set display on
	buf[4]= 0xE8; // Set msb of ram address
	buf[5]= 0x00; // Set ram address

	for(int i=6;i<24;i++)
		{
		buf[i] = 0xFF;//
		}

	// send initialization
	return HAL_I2C_Master_Transmit(&hi2c2, BL5502_ADDR, (uint8_t *)buf, 24, 100);
}

HAL_StatusTypeDef SEG_WriteBuffer(uint8_t data)
{
	BL5502_BUFF[0]= 0xF0;
	BL5502_BUFF[1]= 0xA3;
	BL5502_BUFF[2]= 0xE8;
	BL5502_BUFF[3]= 0x00;

	for(int i=4;i<22;i++)
	{
		BL5502_BUFF[i] = data;//
	}
	HAL_StatusTypeDef return_value;

	return_value = HAL_I2C_Master_Transmit(&hi2c2, BL5502_ADDR, (uint8_t *)BL5502_BUFF, 22, 100);
	BL5502_BUFF[0]= 0xC8;
	HAL_I2C_Master_Transmit(&hi2c2, BL5502_ADDR, (uint8_t *)BL5502_BUFF, 1, 100);

	return return_value;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  /* USER CODE BEGIN 2 */


  // setup multiplexer
  setup_HMILEDs();
  allHMILEds_reset();

  int setup_speed = 500;

  // disable MP3 Player
  HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, 0);


  // Test Lights ###########################################
  // set PWM to 0
  TIM3->CCR1 = 0; // BG
  TIM3->CCR2 = 0; // LIGHT
  // start PWM Timers
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // BG
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // LIGHT

  // 10% Main Light
  TIM3->CCR2 = 10; // LIGHT
  HAL_Delay(setup_speed);

  // off
  TIM3->CCR2 = 0; // LIGHT
  TIM3->CCR1 = 0; // BG
  HAL_Delay(setup_speed);

  // 10% BG Light
  TIM3->CCR2 = 0; // LIGHT
  TIM3->CCR1 = 10; // BG
  HAL_Delay(setup_speed);

  // off
  TIM3->CCR2 = 0; // LIGHT
  TIM3->CCR1 = 0; // BG
  HAL_Delay(setup_speed);

  // Port Expander Test
  allHMILEds_set();
  HAL_Delay(setup_speed);

  allHMILEds_reset();
  HAL_Delay(setup_speed);



  // set default
  TIM3->CCR2 = 0; // LIGHT
  TIM3->CCR1 = 15; // BG
  allHMILEds_set();




/*

  // enable MP3 Player
  HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, 1);

  HAL_Delay(3000);

  if (1) {
	  uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x09, 0x00, 0x00, 0x02, 0xFE, 0xF0, 0xEF}; // specify Micro USB
	  HAL_UART_Transmit(&huart2, UART_buf, 10, HAL_TIMEOUT);
  }
  if (1) {
	  uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xFE, 0xF7, 0xEF}; // Select track 1
	  HAL_UART_Transmit(&huart2, UART_buf, 10, HAL_TIMEOUT);
  }
  if (1) {
	  uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x12, 0x00, 0x00, 0x01, 0xFE, 0xE8, 0xEF}; // Play track "0001" in the folder “MP3”
	  HAL_UART_Transmit(&huart2, UART_buf, 10, HAL_TIMEOUT);
  }*/
  //uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xFF, 0xE6, 0xEF}; // Select track 1
  //HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, UART_buf, 10, 250);

/*
  	  // test error bytes
  uint8_t UART_buf[10] = {0x7E, 0xFF, 0x06, 0x42, 0x00, 0x00, 0x00, 0xFE, 0xB9, 0xEF}; // Read current status
  HAL_StatusTypeDef ret1 = HAL_UART_Transmit(&huart2, UART_buf, 10, 250);

  uint8_t UART_rec_buf[10] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
  HAL_StatusTypeDef ret2 = HAL_UART_Receive(&huart2, &UART_rec_buf[0], 2, 1000);
*/
/*
  // init player
  uint8_t UART_buf1[10] = {0x7E, 0xFF, 0x06, 0x3F, 0x00, 0x00, 0x02, 0xFE, 0xBA, 0xEF}; // Initialize Player
  HAL_StatusTypeDef ret1 = HAL_UART_Transmit(&huart2, UART_buf1, 10, HAL_MAX_DELAY);

  HAL_Delay(200);

  // set volume
  uint8_t UART_buf2[10] = {0x7E, 0xFF, 0x06, 0x06, 0x00, 0x00, 0x14, 0xFE, 0xE1, 0xEF}; // Set volume to 20
  HAL_StatusTypeDef ret2 = HAL_UART_Transmit(&huart2, UART_buf2, 10, HAL_MAX_DELAY);

  HAL_Delay(200);

  // play sound
  uint8_t UART_buf3[10] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x01, 0xFE, 0xF7, 0xEF}; // play from start
  HAL_StatusTypeDef ret3 = HAL_UART_Transmit(&huart2, UART_buf3, 10, HAL_MAX_DELAY);

  HAL_Delay(200);
*/


  // Test BL550072A

  // initialize

  HAL_StatusTypeDef ret1 = LCD_INIT();


  HAL_StatusTypeDef ret2 = LCD_Enable();

  uint8_t data = 255;


  HAL_StatusTypeDef ret3 = LCD_Write();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C2_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DFP_Audio_en_GPIO_Port, DFP_Audio_en_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : nI_O_INT_Pin */
  GPIO_InitStruct.Pin = nI_O_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nI_O_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nUSB_PD_OK_Pin */
  GPIO_InitStruct.Pin = nUSB_PD_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nUSB_PD_OK_GPIO_Port, &GPIO_InitStruct);

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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

