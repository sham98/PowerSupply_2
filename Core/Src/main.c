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
//#define Enc_V	__HAL_TIM_GET_COUNTER(&htim1) / 4       //add in main.h
//#define Enc_I	__HAL_TIM_GET_COUNTER(&htim3) / 4       //add in main.h
#define LED_Num         96
#define ADC_V           AData [0]
#define ADC_I           AData [1]
#define ADC_I_USB       AData [2]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint16_t AData[3], iLED = 0, indx = 0;
uint8_t iBit = 0, LED_Data [LED_Num] = {1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0};
uint16_t DispEncV = 0, DispEncI = 0;
//uint16_t Enc_VV = 0, Enc_II = 0;
//uint8_t * Buf [8];
//typedef union
//{
//  struct
//  {
//    unsigned int A : 1;
//    unsigned int B : 1;
//    unsigned int C : 1;
//    unsigned int D : 1;
//    unsigned int E : 1;
//    unsigned int F : 1;
//    unsigned int G : 1;
//    unsigned int DP : 1;
//  }bit;
//  unsigned int ShiftByte;
//}ShiftLED;
//
//ShiftLED SSV1, SSV2, SSV3, SSV4, SSI1, SSI2, SSI3, SSI4, SSU1, SSU2, SSL1, SSL2;

typedef struct
{
	uint16_t Volt;
	float KRes;
}Monitor;
Monitor Volt, Amp, USBAmp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
uint16_t* SevSegm (uint8_t Num);
void Mon4Seg (uint16_t volt, uint8_t Loc);
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
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  Volt.KRes = 0.266;
  Amp.KRes = 0.65;
  USBAmp.KRes = 0.6;

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

//  HAL_TIM_Base_Start_IT(&htim16);  // clock for shift register used in PWM above too
  HAL_TIM_Base_Start_IT(&htim14);

  #if IF_ADC
  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)AData, 3);
  #endif
  Mon4Seg (2100,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_TIM_Base_Start_IT(&htim14);
    HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 16384;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 16384;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4096;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 4096;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, O_SER_Pin|O_RCK_Pin|O_CLK_Pin|O_S2_Pin
                          |O_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : O_SER_Pin O_CLK_Pin */
  GPIO_InitStruct.Pin = O_SER_Pin|O_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : O_RCK_Pin O_S2_Pin O_S1_Pin */
  GPIO_InitStruct.Pin = O_RCK_Pin|O_S2_Pin|O_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EXI_OUT_Pin EXI_S1_Pin */
  GPIO_InitStruct.Pin = EXI_OUT_Pin|EXI_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXI_SVI_Pin EXI_S2_Pin */
  GPIO_InitStruct.Pin = EXI_SVI_Pin|EXI_S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
////

//void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
//{
//  if (htim == &htim3)
//  {
//    Enc_VV = __HAL_TIM_GET_COUNTER(htim) / 4;
//  }
//  else if (htim == &htim1)
//  {
//    Enc_II = __HAL_TIM_GET_COUNTER(htim) / 4;
//  }
//}

uint16_t* SevSegm (uint8_t Num)
{
  static uint16_t Buf [8];
  switch (Num)
  {
  case 0:
          Buf [0] = 0;
          Buf [1] = 1;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 1:
          Buf [0] = 0;
          Buf [1] = 0;
          Buf [2] = 0;
          Buf [3] = 0;
          Buf [4] = 0;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 2:
          Buf [0] = 1;
          Buf [1] = 1;
          Buf [2] = 0;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 0;
          Buf [6] = 0;
          Buf [7] = 0;
          break;
  case 3:
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 0;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 4:
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 1;
          Buf [3] = 0;
          Buf [4] = 0;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 5:
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 0;
          Buf [7] = 0;
          break;
  case 6:
          Buf [0] = 1;
          Buf [1] = 1;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 0;
          Buf [7] = 0;
          break;
  case 7:
          Buf [0] = 0;
          Buf [1] = 0;
          Buf [2] = 0;
          Buf [3] = 0;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 8:
          Buf [0] = 1;
          Buf [1] = 1;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 9:
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  }
  return Buf;
}
////
////
void Mon4Seg (uint16_t volt, uint8_t Loc)
{
  uint8_t Integer = volt / 100;
  if (Integer == 0)
  {
    uint16_t *pBuf = SevSegm (0);
    for (uint8_t iInteger = 0; iInteger <= 8; iInteger ++)
    {
      LED_Data [Loc + iInteger] = 0;
      LED_Data [Loc + 8 + iInteger] = *(pBuf + iInteger);
    }
  }
  else
  {
    uint16_t * pBuf1 = SevSegm(Integer / 10);
    uint16_t * pBuf2 = SevSegm(Integer % 10);
    for (uint8_t iInteger = 0; iInteger <= 8; iInteger ++)
    {
      LED_Data [Loc + iInteger] = *(pBuf1 + iInteger);
      LED_Data [Loc + 8 + iInteger] = *(pBuf2 + iInteger);
    }
  }
  LED_Data [Loc + 15] = 1;
  uint8_t Frac = volt % 100;
  if (Frac == 0)
  {
    uint16_t *pBuf = SevSegm (0);
    for (uint8_t iFrac = 0; iFrac <= 8; iFrac ++)
    {
      LED_Data [Loc + 16 + iFrac] = *(pBuf + iFrac);
      LED_Data [Loc + 24 + iFrac] = 0;
    }
  }
  else
  {
    uint16_t * pBuf1 = SevSegm(Frac / 10);
    uint16_t * pBuf2 = SevSegm(Frac % 10);
    for (uint8_t iFrac = 0; iFrac <= 8; iFrac ++)
    {
      LED_Data [Loc + 16 + iFrac] = *(pBuf1 + iFrac);
      LED_Data [Loc + 24 + iFrac] = *(pBuf2 + iFrac);
    }
  }
//	for (uint8_t i = 0; i <= 3; i ++)
//	{
//		uint16_t Num = volt % 10;
//		volt = volt / 10;
//		SevSegm(Num,Buf);
////		LED_Data [Loc - i] = SevNum;
//	}
}
//
//
//void Mon2Seg (uint16_t volt, uint8_t Loc)
//{
//	for (uint8_t i = 0; i <= 1; i ++)
//	{
//		uint16_t Num = volt % 10;
//		volt = volt / 10;
//		uint16_t SevNum = SevSegm(Num);
//		LED_Data [Loc - i] = SevNum;
//	}
//}
//



/**
  * @brief  Period elapsed half complete callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim14)
	{
          
//		iDisp --;
//		if (iDisp == 0)
//		{
//			if (iLED == 0)
//			{
//				HAL_ADC_PollForConversion(&hadc, 100);
//			}
//			else
//			{
#if IF_ADC
          if (DispEncV > 0)
          {
            DispEncV --;
            Mon4Seg (Enc_V,0);
          }
          else
          {
            HAL_ADC_PollForConversion(&hadc, 100);
            Mon4Seg(ADC_V,0);
          }

          if (DispEncI > 0)
          {
            DispEncI --;
            Mon4Seg (Enc_I,32);
          }
          else
          {
            HAL_ADC_PollForConversion(&hadc, 100);
            Mon4Seg(ADC_I,32);
          }
#endif
#if IF_Disp
          if (iLED % 2 == 0)
          {
                  HAL_GPIO_WritePin(O_CLK_GPIO_Port, O_CLK_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(O_SER_GPIO_Port, O_SER_Pin, LED_Data[(LED_Num - 1 - iLED / 2)]);
//					HAL_GPIO_WritePin(O_SER_GPIO_Port, O_SER_Pin, LED_Data[(LED_Num - iLED) * (1 << iBit)]);
//					iBit ++;
          }
          else
          {
                  HAL_GPIO_WritePin(O_CLK_GPIO_Port, O_CLK_Pin, GPIO_PIN_SET);
          }

//				if (iBit >= 8)
//				{
//					iLED ++;
//					iBit = 0;
//				}
          iLED ++;
          if (iLED >= 2 * LED_Num)
          {
                  iLED = 0;
                  HAL_GPIO_WritePin(O_RCK_GPIO_Port, O_RCK_Pin, GPIO_PIN_SET);
                  while (HAL_GPIO_ReadPin(O_RCK_GPIO_Port, O_RCK_Pin) == 0);
                  HAL_GPIO_WritePin(O_RCK_GPIO_Port, O_RCK_Pin, GPIO_PIN_RESET);

                  HAL_TIM_Base_Stop_IT(&htim14);
          }
#endif
//			}
//		}
//		else if (EncoderSpeed < 10)
//		{
//			iDisp = Disp3s;
//			iLED = 0;
//		}
//		else if (EncoderSpeed < 20)
//		{
//			iDisp = Disp3s;
//			iLED = 0;
//		}
	}
}



//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	void (* SerialArray)(uint8_t , uint8_t) = & Mon4Seg;
//	Volt.Volt = AData [0] * 3.3 / 4096 * Volt.KRes * 100;
//	SerialArray (Volt.Volt, 3);
//
//	Amp.Volt = AData [1] * 3.3 / 4096 * Amp.KRes * 100;
//	SerialArray (Amp.Volt, 7);
//
//	void (* SerialArray)(uint8_t , uint8_t) = & Mon2Seg;
//	USBAmp.Volt = AData [2] * 3.3 / 4096 * USBAmp.KRes * 10;
//	SerialArray (USBAmp.Volt, 9);
//
//	HAL_TIM_Base_Start_IT(&htim14);
//}
//
//
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == GPIO_PIN_12)
//	{
//		if (O_S == )
//		{
//		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, Enc_I);
//		}
//		else if (O_S == )
//		{
//		__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, Enc_V);
//		}
//	}
//}
//



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
