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
#include <stdio.h>
#include <stdlib.h>

#include "PID.h"
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint32_t AData[3];
uint16_t iLED = 0;
uint8_t LED_Data [LED_Num] = {0};
uint8_t iSelEXI = 0;
uint8_t EXIS1PrePrs [9] = {0};
uint8_t EXIS2PrePrs [9] = {0};
uint8_t EXIS1PrvPrs [4] = {0};
uint8_t EXIS2PrvPrs [4] = {0};
uint16_t iEXIS1Prs [5] = {0,0,0,0};
uint16_t iPrsEXIS2 [5] = {0,0,0,0};
//uint16_t iKepEXI = 1000;
//uint16_t iClkEXI = 100;

uint16_t indx = 0;

uint16_t Disp3s = 30;

//uint16_t MaxSamEncTime = 30;

uint8_t LowByte = 0;
uint8_t HighByte = 0;
uint8_t dL = 0;
uint8_t dH = 0;
uint8_t cL = 0;
uint8_t cH = 0;


//uint16_t Knew = 0;
//uint16_t Kold = 0;
uint16_t VoltBstKnew = 30;
uint16_t VoltBstKold = 70;
uint16_t CurrBstKnew = 30;
uint16_t CurrBstKold = 70;


uint16_t MaxFIR = 200;
uint16_t iMaxFIR = 0;

uint8_t PIDEn = 0;
//float error = 0;
uint16_t MaxCountPID = 5000;
uint8_t InitFlag = 0;
uint8_t FirsRead = 0;
uint16_t InitCount = 0;
uint16_t MaxInitCount = 3000;
uint16_t TempVolt = 0;
//int32_t MAXSumError = 50000000;

uint16_t ErrStp1 = 5000;
float Kp1 = 0.1;
float Ki1 = 0.4;
float Kd1 = 0.01;
float Tau1 = 0.01;

float Kp2 = .1;
float Ki2 = .4;
float Kd2 = 6;
float Tau2 = 6;


uint16_t CurrErrStp1 = 500;
uint16_t CurrErrStp2 = 4000;
float CurrKp1 = 0.05;
float CurrKi1 = 0.3;
float CurrKd1 = 0.1;
float CurrTau1 = 0.01;

float CurrKp2 = .2;
float CurrKi2 = 1;
float CurrKd2 = 2;
float CurrTau2 = 3;


uint8_t VOLT2ENC = 12;

int16_t MaxEncSpeed = 20;
int16_t MaxEncInc = 1200;
//int16_t MinEncSpeed = 4;

uint32_t ArrNumFIFO = 0;
uint32_t MaxArrNumFIFO = 300;

uint16_t AFIFO [3][300] = {0};
//uint16_t ArrNumMeanFIFO = 1000;
//uint16_t MeanFIFO [3][100] = {0};
uint32_t SumFIFO [3] = {0,0,0};                       // Array sum of ADC values
uint32_t iFilADC = 0;
  
#if (IF_VOLTTest | IF_CURRTest)
uint16_t iTriInd = 0;
uint16_t ITriIndMax = 1;
int16_t iTriangle = 0;
uint16_t iTriMax = 6400;
uint16_t Ramp = 1;
uint16_t TriVolStep = 4;
#endif

#if IF_VOLTPIDTunning
uint16_t VOLTENCSTP1 = 8000;
uint16_t VOLTENCSTP2 = 40000;
uint16_t MaxiPIDSwitch = 2000;
#endif

#if IF_CURRPIDTunning
uint16_t CURRENCSTP1 = 4000;
uint16_t CURRENCSTP2 = 8000;
uint16_t MaxiPIDSwitch = 2000;
#endif


enum
{
  Nor,
  OC,
  NoMem,
  M1,
  M2,
  M3,
  M4
};


//PIDController pid;

Monitor Volt, Curr;
MonitorUSB USBCurr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
uint16_t* SevSegm (uint8_t Num);
void Mon4Seg (uint16_t volt, uint8_t Loc);
void Delay (void);
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
//  Knew = BstKnew;
//  Kold = BstKnew;
#if IF_CURRTest
uint16_t iTriMax = HTIM_PWM_CURR.Init.Period;
#endif

#if IF_VOLTTest
uint16_t iTriMax = HTIM_PWM_VOL.Init.Period;
#endif

  Volt.pid.Kp = 0.1;
  Volt.pid.Ki = 0.2;
  Volt.pid.Kd = 0.2;
  Volt.pid.tau = 0.01;
  Volt.pid.limMax = HTIM_PWM_VOL.Init.Period;
  Volt.pid.limMin = 0;
  Volt.pid.limMaxInt = 50000;
  Volt.pid.limMinInt = -50000;
  Volt.pid.T = 0.000225;

  Curr.pid.Kp = 0.1;
  Curr.pid.Ki = 0.2;
  Curr.pid.Kd = 0.2;
  Curr.pid.tau = 0.01;
  Curr.pid.limMax = HTIM_PWM_CURR.Init.Period;
  Curr.pid.limMin = 0;
  Curr.pid.limMaxInt = 50000;
  Curr.pid.limMinInt = -50000;
  Curr.pid.T = 0.000225;
  
  Volt.MaxVolt = VoltMAX;
  Curr.MaxVolt = CurrMAX;
  Volt.EncFactor = HTIM_ENC_VOL.Init.Period / HTIM_PWM_VOL.Init.Period;
  Curr.EncFactor = HTIM_ENC_CURR.Init.Period / HTIM_PWM_CURR.Init.Period;
  Volt.KDspnew = 30;
  Volt.KDspold = 70;
  Curr.KDspnew = 30;
  Curr.KDspold = 70;
  Volt.DispFactor0 = - 15.69;
  Volt.DispFactor1 = 0.8464;
  Curr.DispFactor0 = -331.9;
  Curr.DispFactor1 = 1.38;
  USBCurr.DispFactor1 = 1;
  Curr.Enc = 4000;
  __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc / Curr.EncFactor);

  HAL_TIM_Encoder_Start_IT(&HTIM_ENC_CURR, TIM_CHANNEL_ALL);    // Encoder Current PWM
  HAL_TIM_Encoder_Start_IT(&HTIM_ENC_VOL, TIM_CHANNEL_ALL);    // Encoder Voltage PWM

//  HAL_TIM_PWM_Start(&HTIM_ENC_VOL, TIM_CHANNEL_3);     // Fan PWM
  HAL_TIM_PWM_Start(&HTIM_PWM_CURR, TIM_CHANNEL_1);    // Current PWM
  HAL_TIM_PWM_Start(&HTIM_PWM_VOL, TIM_CHANNEL_1);    // Voltage PWM

  HAL_TIM_Base_Start_IT(&htim14);

  HAL_ADCEx_Calibration_Start(&hadc);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)AData, 3);

  PIDController_Init(&Volt.pid);

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
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
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
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 51200;
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
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 51200;
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
  htim14.Init.Period = 10000;
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
  htim16.Init.Period = 6400;
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
  htim17.Init.Period = 6400;
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EXI_S2_Pin */
  GPIO_InitStruct.Pin = EXI_S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXI_S2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief Delay
  * @param void
  * @retval void
  */
void Delay (void)
{
  for (uint16_t iDelay = 0; iDelay <= 10000; iDelay ++)
  {}
}




/**
  * @brief Get Number in input and convert it into 8 buffer to Display in 7segment.
  * @param Num  Number from 0 to 9.
  * @retval 8 buffer.
  */
uint16_t* SevSegm (uint8_t Num)
{
  static uint16_t Buf [8];
  switch (Num)
  {
  case 0:                       // Number '0'
          Buf [0] = 0;
          Buf [1] = 1;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 1:                       // Number '1'
          Buf [0] = 0;
          Buf [1] = 0;
          Buf [2] = 0;
          Buf [3] = 0;
          Buf [4] = 0;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 2:                       // Number '2'
          Buf [0] = 1;
          Buf [1] = 1;
          Buf [2] = 0;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 0;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 3:                       // Number '3'
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 0;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 4:                       // Number '4'
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 1;
          Buf [3] = 0;
          Buf [4] = 0;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 5:                       // Number '5'
          Buf [0] = 1;
          Buf [1] = 0;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 0;
          Buf [7] = 0;
          break;
  case 6:                       // Number '6'
          Buf [0] = 1;
          Buf [1] = 1;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 0;
          Buf [7] = 0;
          break;
  case 7:                       // Number '7'
          Buf [0] = 0;
          Buf [1] = 0;
          Buf [2] = 0;
          Buf [3] = 0;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 8:                       // Number '8'
          Buf [0] = 1;
          Buf [1] = 1;
          Buf [2] = 1;
          Buf [3] = 1;
          Buf [4] = 1;
          Buf [5] = 1;
          Buf [6] = 1;
          Buf [7] = 0;
          break;
  case 9:                       // Number '9'
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




/**
  * @brief Get Number in input and locate it into LED_Data array.
  * @param volt  Number from 0 to 9999 that in display devide in 100.
  * @param Loc  Location in LED_Data to put volt array
  * @retval None.
  */
void Mon4Seg (uint16_t volt, uint8_t Loc)
{
  uint8_t Integer = volt / 100;         // Quotient of divide in 100 
  if ((Integer <= 9) & (Loc == VolLoc))                     // Locate Quotient in LED_Data and remove zero before one digit numbers
  {
    uint16_t *pBuf = SevSegm (Integer);
    for (uint8_t iInteger = 0; iInteger < 8; iInteger ++)
    {
      LED_Data [Loc + iInteger] = 0;
      LED_Data [Loc + 8 + iInteger] = *(pBuf + iInteger);
    }
  }
  else                                  // Locate Quotient in LED_Data
  {
    uint16_t * pBuf1 = SevSegm(Integer / 10);
    for (uint8_t iInteger = 0; iInteger < 8; iInteger ++)
    {
      LED_Data [Loc + iInteger] = *(pBuf1 + iInteger);
    }
    uint16_t * pBuf2 = SevSegm(Integer % 10);
    for (uint8_t iInteger = 0; iInteger < 8; iInteger ++)
    {
      LED_Data [Loc + 8 + iInteger] = *(pBuf2 + iInteger);
    }
  }

//  LED_Data [Loc + 15] = 1;              // Point
  uint8_t Frac = volt % 100;            // Remainder of divide in 100

  uint16_t * pBuf1 = SevSegm(Frac / 10);
  for (uint8_t iFrac = 0; iFrac < 8; iFrac ++)
  {
    LED_Data [Loc + 16 + iFrac] = *(pBuf1 + iFrac);
  }
  uint16_t * pBuf2 = SevSegm(Frac % 10);
  for (uint8_t iFrac = 0; iFrac < 8; iFrac ++)
  {
    LED_Data [Loc + 24 + iFrac] = *(pBuf2 + iFrac);
  }
}


/**
  * @brief Get Number in input and locate it into LED_Data array.
  * @param volt  Number from 0 to 99 that in display devide in 10.
  * @retval None.
  */
void Mon2Seg (uint16_t volt)
{
  for (uint8_t i = 0; i <= 1; i ++)
  {
    uint16_t *pBuf = SevSegm (volt / 10);
    for (uint8_t i2disp = 0; i2disp < 8; i2disp ++)     // Locate Remainder in LED_Data
    {
      LED_Data [64 + i2disp] = *(pBuf + i2disp);
    }
    LED_Data [71] = 1;
    uint16_t *pBuf2 = SevSegm (volt % 10);                   // Locate Quotient in LED_Data
    for (uint8_t i2disp = 0; i2disp < 8; i2disp ++)
    {
      LED_Data [72 + i2disp] = *(pBuf2 + i2disp);
    }
  }
}




/**
  * @brief  Period elapsed half complete callback in non-blocking mode
  *         containing External Interrupt and display handling.
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim14)
	{
          HAL_TIM_Base_Stop(htim);                      // Stop timing 
          
          Curr.TempDisp = (Curr.DispFactor1 * Curr.Volt + Curr.DispFactor0);

#if (!IF_CURRTest & !IF_VOLTTest)
          if (InitFlag == 0)            // Zero offset
          {
            if (InitCount < MaxInitCount)
            {
              InitCount ++;
            }
            else
            {
              if (FirsRead == 0)
              {
                TempVolt = Volt.Volt;
                FirsRead = 1;
              }
              if (Volt.Volt < 1.005 * TempVolt)
              {
                Volt.PWM ++;
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL,TIM_CHANNEL_1, Volt.PWM);
              }
              else
                InitFlag = 1;
        //              Volt.MinVolt = 
            }
          }
          else
          {
//            if ((Curr.pid.error < CurrErrStp1) & (Curr.pid.error > -CurrErrStp1))
//            {
//              Curr.pid.Kp = CurrKp1;
//              Curr.pid.Ki = CurrKi1;
//              Curr.pid.Kd = CurrKd1;
//              Curr.pid.tau = CurrTau1;                
//            }
//            else if ((Curr.pid.error < -CurrErrStp2) | (Curr.pid.error > CurrErrStp2))
//            {
//              Curr.pid.Kp = CurrKp2;
//              Curr.pid.Ki = CurrKi2;
//              Curr.pid.Kd = CurrKd2;
//              Curr.pid.tau = CurrTau2;                
//            }
//            Curr.PWM = PIDController_Update(&Curr.pid, Curr.Enc , VOLT2ENC * Curr.Volt);
//            __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR,TIM_CHANNEL_1, Curr.PWM);
                        
            
            if ((Volt.pid.error < ErrStp1) & (Volt.pid.error > -ErrStp1))
            {
              Volt.pid.Kp = Kp1;
              Volt.pid.Ki = Ki1;
              Volt.pid.Kd = Kd1;
              Volt.pid.tau = Tau1;                
            }
            else
            {
              Volt.pid.Kp = Kp2;
              Volt.pid.Ki = Ki2;
              Volt.pid.Kd = Kd2;
              Volt.pid.tau = Tau2;                
            }
            Volt.PWM = PIDController_Update(&Volt.pid, Volt.Enc, VOLT2ENC * Volt.Volt);
            __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL,TIM_CHANNEL_1, Volt.PWM);
          }
#endif          
/*
 *
 *      Extrnal Interrupt
 *
 */


/*    S1  
 *    start
 */
          if(EXIS1PrePrs [iSelEXI + 4] == 1)                // when S1 pin pressed (in iSelEXI status )
          {
            if (HAL_GPIO_ReadPin(EXI_S1_GPIO_Port, EXI_S1_Pin) == 0)    // If time presses the button not as long as that is to be kept
            {
              iEXIS1Prs [iSelEXI + 1] ++;                   // add time presses the button 
            }
            else if (iEXIS1Prs [iSelEXI + 1] >= iKepEXI)         // If time presses the button as long as that is to be kept
            {
              EXIS1PrePrs [iSelEXI + 4] = 0;
              iEXIS1Prs [iSelEXI + 1] = 0;                   // to avoid run this if in next
              if (iSelEXI == 0)                         // if pin '0' -------->  M3
              {
                LowByte = Volt.Enc & 0xff;              // seperate Low Byte
                HighByte = (Volt.Enc & 0xff00) >> 8;            // seperate High Byte
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBVolM3Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );          // write Low Byte
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBVolM3Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );         // write High Byte
                Delay();

                LowByte = Curr.Enc & 0xff;              // seperate Low Byte
                HighByte = (Curr.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBCurM3Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBCurM3Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
              }
              else if (iSelEXI == 1)                    // if pin '1' -------->  M1
              {
                LowByte = Volt.Enc & 0xff;              // seperate Low Byte
                HighByte = (Volt.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBVolM1Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBVolM1Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();

                LowByte = Curr.Enc & 0xff;              // seperate Low Byte
                HighByte = (Curr.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBCurM1Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBCurM1Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
              }
              else if (iSelEXI == 2)                    // if pin '2' -------->  M1
              {
                LowByte = Volt.Enc & 0xff;              // seperate Low Byte
                HighByte = (Volt.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBVolM2Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBVolM2Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();

                LowByte = Curr.Enc & 0xff;              // seperate Low Byte
                HighByte = (Curr.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBCurM2Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBCurM2Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
              }
              else if (iSelEXI == 3)                    // if pin '3' -------->  M4
              {
                LowByte = Volt.Enc & 0xff;              // seperate Low Byte
                HighByte = (Volt.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBVolM4Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBVolM4Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();

                LowByte = Curr.Enc & 0xff;                      // seperate Low Byte
                HighByte = (Curr.Enc & 0xff00) >> 8;
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), LBCurM4Mem, I2C_MEMADD_SIZE_8BIT, &LowByte, I2C_MEMADD_SIZE_8BIT, 1000 );
                Delay();
                HAL_I2C_Mem_Write( &hi2c1, (0b1010000 << 1), HBCurM4Mem, I2C_MEMADD_SIZE_8BIT, &HighByte, I2C_MEMADD_SIZE_8BIT, 1000 );
              }              
            }
            else if ((iEXIS1Prs [iSelEXI + 1] >= iClkEXI) & (iEXIS1Prs [iSelEXI + 1] < iKepEXI))     // if time presses the button as long as that is to be clicked
            {
              EXIS1PrePrs [iSelEXI + 4] = 0;
              iEXIS1Prs [iSelEXI + 1] = 0;                   // to avoid run this if in next
              if (iSelEXI == 0)                                                         // if pin '0' -------->  M3
              {
                Volt.Mem = M3;
                LED_Data [LEDM1Num] = 0;
                LED_Data [LEDM2Num] = 0;
                LED_Data [LEDM3Num] = 1;
                LED_Data [LEDM4Num] = 0;
                
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBVolM3Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBVolM3Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Volt.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.Enc / Volt.EncFactor);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBCurM3Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBCurM3Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Curr.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_CURR, Curr.Enc);
              }
              else if (iSelEXI == 1)                                                    // if pin '1' -------->  M1
              {
                Volt.Mem = M1;
                LED_Data [LEDM1Num] = 1;
                LED_Data [LEDM2Num] = 0;
                LED_Data [LEDM3Num] = 0;
                LED_Data [LEDM4Num] = 0;

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBVolM1Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBVolM1Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                 Volt.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.Enc / Volt.EncFactor);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBCurM1Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBCurM1Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Curr.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_CURR, Curr.Enc);
              }
              else if (iSelEXI == 2)                                                    // if pin '2' -------->  M1
              {
                Volt.Mem = M2;
                LED_Data [LEDM1Num] = 0;
                LED_Data [LEDM2Num] = 1;
                LED_Data [LEDM3Num] = 0;
                LED_Data [LEDM4Num] = 0;

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBVolM2Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBVolM2Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Volt.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.Enc / Volt.EncFactor);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBCurM2Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBCurM2Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Curr.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_CURR, Curr.Enc);
              }
              else if (iSelEXI == 3)                                                    // if pin '3' -------->  M4
              {
                Volt.Mem = M4;
                LED_Data [LEDM1Num] = 0;
                LED_Data [LEDM2Num] = 0;
                LED_Data [LEDM3Num] = 0;
                LED_Data [LEDM4Num] = 1;

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBVolM4Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBVolM4Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Volt.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.Enc / Volt.EncFactor);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);

                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), LBCurM4Mem, I2C_MEMADD_SIZE_8BIT,&cL,I2C_MEMADD_SIZE_8BIT,1000);
                HAL_I2C_Mem_Read( &hi2c1, (0b10100000), HBCurM4Mem, I2C_MEMADD_SIZE_8BIT,&cH,I2C_MEMADD_SIZE_8BIT,1000);
                Curr.Enc = cL | (cH << 8);
                __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc);
                __HAL_TIM_SET_COUNTER(&HTIM_ENC_CURR, Curr.Enc);
              }
            }
            else
            {
              EXIS1PrePrs [iSelEXI + 4] = 0;
            }
          }
          else
          {
            iEXIS1Prs [iSelEXI + 1] = 0;
          }

/*    S1  
 *    end
 */




/*    S2  
 *    start
 */

          if(EXIS2PrePrs [iSelEXI] == 1)                // when S2 pin pressed (in iSelEXI status )
          {
            if (HAL_GPIO_ReadPin(EXI_S2_GPIO_Port, EXI_S2_Pin) == 0)
            {
              iPrsEXIS2 [iSelEXI + 1] ++;                   // add time presses the button 
            }
            else if (iPrsEXIS2 [iSelEXI + 1] >= iKepEXI)         // If time presses the button as long as that is to be kept
            {
              EXIS2PrePrs [iSelEXI] = 0;
              iPrsEXIS2 [iSelEXI + 1] = 0;                   // to avoid run this if in next
              if (iSelEXI == 0)                         // if pin '0' -------->  OVP
              {
                if (Volt.Status == OC)                  // if OVP enabled
                {
                  Volt.Status = Nor;                    // enable Normal 
                  LED_Data [LEDOVPNum] = 0;             // OCV LED OFF
                  Volt.MaxVolt = VoltMAX;               // set MAX voltage 
                }
                else if (Volt.Status == Nor)
                {
                  Volt.Status = OC;
                  LED_Data [LEDOVPNum] = 1;
                  Volt.MaxVolt = Volt.Enc;
                }
              }
              else if (iSelEXI == 1)                    // if pin '2' -------->  OCP
              {
                if (Curr.Status == OC)
                {
                  Curr.Status = Nor;
                  LED_Data [LEDOCPNum] = 0;
                  Curr.MaxVolt = VoltMAX;
                }
                else if (Curr.Status == Nor)
                {
                  Curr.Status = OC;
                  LED_Data [LEDOCPNum] = 1;
                  Curr.MaxVolt = Curr.Enc;
                }
              }
              else if (iSelEXI == 2)                    // if pin '1' -------->  SV
              {
                LED_Data [LEDLockNum] = ~LED_Data [LEDLockNum];
              }
              else if (iSelEXI == 3)                    // if pin '3' -------->  SI
              {
                
              }              
            }
            else if ((iPrsEXIS2 [iSelEXI + 1] >= iClkEXI) & (iPrsEXIS2 [iSelEXI + 1] < iKepEXI))     // if time presses the button as long as that is to be clicked
            {
              EXIS2PrePrs [iSelEXI] = 0;
              iPrsEXIS2 [iSelEXI + 1] = 0;                   // to avoid run this if in next
              if (iSelEXI == 0)                         // if pin '0' -------->  OVP
              {

              }
              else if (iSelEXI == 1)                    // if pin '1' -------->  SV
              {
                if ((Volt.CountDisp > 0) & (Volt.Status == OC))                       // if it's still in display encoder
                {
                  __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.Enc / Volt.EncFactor);
                  Volt.CountDisp = 0;
                }
              }
              else if (iSelEXI == 2)                    // if pin '2' -------->  OCP
              {

              }
              else if (iSelEXI == 3)                    // if pin '3' -------->  SI
              {
                if ((Curr.CountDisp > 0) & (Curr.Status == OC))                       // if it's still in display encoder
                {
                  __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc);
                  Curr.CountDisp = 0;
                }
              }
              else
              {
                EXIS2PrePrs [iSelEXI] = 0;
              }
            }
          }
          else
          {
            iPrsEXIS2 [iSelEXI + 1] = 0;                   // if time presses the button not as long as that is to be clicked or kept or not, reset it
          }
          
/*    S2  
 *    end
 */
          
          
          if (iSelEXI >= 3)                             // change S pins
          {
            iSelEXI = 0;
            O_S1_GPIO_Port -> ODR = O_S1_GPIO_Port -> ODR & 0xF3FF;
          }
          else
          {
            iSelEXI ++;
            O_S1_GPIO_Port -> ODR = O_S1_GPIO_Port -> ODR + 1024;
          }
  
/******************************************************************************/





/*
 *
 *      Shift Data Register
 *
 */
          
          if (Volt.KDspnew != VoltBstKnew)
          {
            if (iMaxFIR > MaxFIR)
            {
              iMaxFIR = 0;
              Volt.KDspnew --;
              Volt.KDspold ++;
            }
            else
            {
              iMaxFIR ++;
            }
          }            

          
          iLED ++;
          if (iLED >= 2 * LED_Num)
          {
            iLED = 0;
            HAL_GPIO_WritePin(O_RCK_GPIO_Port, O_RCK_Pin, GPIO_PIN_SET);
            while (HAL_GPIO_ReadPin(O_RCK_GPIO_Port, O_RCK_Pin) == 0);
            HAL_GPIO_WritePin(O_RCK_GPIO_Port, O_RCK_Pin, GPIO_PIN_RESET);


            
           /*******************************************************************
           *
           *      LED_Data buffer filling
           *
           *******************************************************************/
            
            
            if ((Volt.Status == Nor) | (Volt.CountDisp == 0))
            {
              int32_t TempDisp = (Volt.DispFactor1 * Volt.Volt + Volt.DispFactor0);
              if (TempDisp > 0)
                Volt.DispVolt = (Volt.KDspnew * TempDisp + Volt.KDspold * Volt.DispVolt) / 100;
              else
                Volt.DispVolt = 0;
              Mon4Seg(Volt.DispVolt, VolLoc);
              LED_Data [VolLoc + 15] = 1;              // Point
            }
            else if (Volt.CountDisp >= Disp3s)
            {
              Volt.DispEnc = Volt.Enc / 4;
              Mon4Seg(Volt.DispEnc, VolLoc);
              LED_Data [VolLoc + 15] = 1;              // Point
              Volt.CountDisp --;
            }
            else if (Volt.CountDisp > 0)
            {
              Volt.DispEnc = Volt.Enc / 4;
              Mon4Seg(Volt.DispEnc, VolLoc);
              LED_Data [VolLoc + 15] = 1;              // Point
              Volt.CountDisp --;
            }
            
            
//            if ((Curr.Status == Nor) | (Curr.CountDisp == 0))
            if (Curr.CountDisp == 0)
            {
//              Curr.TempDisp = (Curr.DispFactor1 * Curr.Volt + Curr.DispFactor0);
              if (Curr.TempDisp > 0)
                Curr.DispVolt = (Curr.KDspnew * Curr.TempDisp + Curr.KDspold * Curr.DispVolt) / 100;
              else
                Curr.DispVolt = 0;
              Mon4Seg(Curr.DispVolt, CurLoc);
              LED_Data [CurLoc + 7] = 1;              // Point
            }
            else if (Curr.CountDisp >= Disp3s)
            {
              Curr.DispEnc = Curr.Enc / 4;
//              Curr.MaxVolt = Curr.DispEnc;
              Mon4Seg (Curr.DispEnc, CurLoc);
              LED_Data [CurLoc + 7] = 1;              // Point
              Curr.CountDisp --;
            }
            else if (Curr.CountDisp > 0)
            {
              Curr.DispEnc = Curr.Enc / 4;
              Mon4Seg (Curr.DispEnc, CurLoc);
              LED_Data [CurLoc + 7] = 1;              // Point
              Curr.CountDisp --;
            }
            
            USBCurr.DispVolt = USBCurr.DispFactor1 * USBCurr.Volt + USBCurr.DispFactor0;
            Mon2Seg(USBCurr.DispVolt);

            /******************************************************************/
          }
          else
           {
            if (iLED % 2 == 0)
            {
              HAL_GPIO_WritePin(O_CLK_GPIO_Port, O_CLK_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(O_SER_GPIO_Port, O_SER_Pin, LED_Data[(LED_Num - 1 - iLED / 2)]);
            }
            else
            {
              HAL_GPIO_WritePin(O_CLK_GPIO_Port, O_CLK_Pin, GPIO_PIN_SET);
            }
          }

/******************************************************************************/

          HAL_TIM_Base_Start_IT(htim);
	}
}




/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim)
{
//  ArrNumFIFO = 10;
  Volt.KDspnew = MaxKnew;
  Volt.KDspold = MaxKold;

  Volt.Mem = NoMem;
  LED_Data [LEDM1Num] = 0;
  LED_Data [LEDM2Num] = 0;
  LED_Data [LEDM3Num] = 0;
  LED_Data [LEDM4Num] = 0;

  if (htim == &HTIM_ENC_VOL)
  {
    Volt.Enc = __HAL_TIM_GET_COUNTER(htim);
    Volt.CountDisp = Disp3s;
    int16_t TempVoltEnc = Volt.Enc - Volt.OldEnc;
    
    if (TempVoltEnc > 30000)
    {
      Volt.Enc = 0;
      __HAL_TIM_SET_COUNTER(htim, 0);
    }
    else if (Volt.Enc > Volt.MaxVolt)
    {
      Volt.Enc = Volt.MaxVolt;
      __HAL_TIM_SET_COUNTER(htim, Volt.Enc);
    }

    if (Volt.Status == Nor)                       // if in normal mode set PWM directly
    {
      Volt.SP = Volt.Volt;
//      Volt.PWM = Volt.Enc / Volt.EncFactor;
//      __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.PWM);
      Volt.CountPID = 0;
    }
  }
  else if (htim == &HTIM_ENC_CURR)
  {
    Curr.Enc = __HAL_TIM_GET_COUNTER(htim);
    Curr.CountDisp = Disp3s;

    if ((Curr.Enc - Curr.OldEnc) > 30000)
    {
      Curr.Enc = 0;
      __HAL_TIM_SET_COUNTER(htim, 0);
    }
    else if (Curr.Enc > Curr.MaxVolt)
    {
      Curr.Enc = Curr.MaxVolt;
      __HAL_TIM_SET_COUNTER(htim, Curr.Enc);
    }

    if (Curr.Status == Nor)                       // if it's still in display encoder
    {
      Curr.PWM = Curr.Enc / Curr.EncFactor * 2;
      __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.PWM);
    }
  }
}






/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint32_t SumFIFO [3] = {0,0,0};
  ArrNumFIFO = MaxArrNumFIFO;


  for (uint8_t iFilADC = 0; iFilADC < 3; iFilADC ++)
  {
    for (uint16_t iFIFO = 0; iFIFO < ArrNumFIFO - 1; iFIFO ++)
    {
      AFIFO [iFilADC] [iFIFO] = AFIFO [iFilADC] [iFIFO + 1];
      SumFIFO [iFilADC] += AFIFO [iFilADC] [iFIFO];
    }
    AFIFO [iFilADC][ArrNumFIFO - 1] = AData [iFilADC];
    SumFIFO [iFilADC] += AFIFO [iFilADC][ArrNumFIFO - 1];
  }
  
  Volt.Volt = (100 * (SumFIFO [0] / ArrNumFIFO) + 0 * Volt.Volt) / 100;
  Curr.Volt = (100 * (SumFIFO [1]/ ArrNumFIFO) + 0 * Curr.Volt) / 100;
  USBCurr.Volt = (100 * (SumFIFO [2]/ ArrNumFIFO) + 0 * USBCurr.Volt) / 100;

  
  

    if (Volt.Status == OC)
    {
      if (Volt.Volt > Volt.MaxVolt)
      {
        LED_Data [RelOutNum] = 0;
        Volt.Out = 0;
      }
    }
    

    if (Curr.Status == OC)
    {
      if (AData [1] > Curr.MaxVolt)
      {
        LED_Data [RelOutNum] = 0;
        Volt.Out = 0;
      }
    }
}

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t ireadRESET = 0;
  if (GPIO_Pin == EXI_S1_Pin)
  {
    for (uint8_t iread = 0; iread < MAXReadEXI; iread ++)                       // *************************************************
    {
      if (HAL_GPIO_ReadPin(EXI_S1_GPIO_Port, EXI_S1_Pin) == 0)                  // *
      {
        ireadRESET ++;                                                          // *
      }                                                                         // *                    Debouncing procedure
    }
    if (ireadRESET >= CorReadEXI)                                               // *
    {
      EXIS1PrePrs [iSelEXI + 4] = 1;                                            // *
    }                                                                           // *************************************************
  }
  else if (GPIO_Pin == EXI_S2_Pin)
  {
    for (uint8_t iread = 0; iread < MAXReadEXI; iread ++)
    {
      if (HAL_GPIO_ReadPin(EXI_S2_GPIO_Port, EXI_S2_Pin) == 0)
      {
        ireadRESET ++;
      }
    }
    if (ireadRESET >= CorReadEXI)
    {
      EXIS2PrePrs [iSelEXI] = 1;
    }
  }
  else if (GPIO_Pin == EXI_OUT_Pin)
  {
    for (uint8_t iread = 0; iread < MAXReadEXI; iread ++)
    {
      if (HAL_GPIO_ReadPin(EXI_OUT_GPIO_Port, EXI_OUT_Pin) == 0)
      {
        ireadRESET ++;
      }
    }
    if (ireadRESET >= CorReadEXI)
    {
      if (Volt.Out == 0)
      {
        LED_Data [RelOutNum] = 1;
        LED_Data [LEDOUTNum] = 1;
        Volt.Out = 1;
      }
      else
      {
        LED_Data [RelOutNum] = 0;
        LED_Data [LEDOUTNum] = 0;
        Volt.Out = 0;
      }
    }
  }
}



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
