/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern uint16_t indx;
int16_t EncoderSpeed_V = 0, EncoderSpeed_I = 0;
uint16_t oldpos_V = 0, oldpos_I = 0;
extern uint16_t DispEncV, DispEncI;
extern uint16_t Disp3s;
extern uint16_t Enc_V, Enc_I;
extern uint16_t MINEncoderSpeed;
//extern uint16_t EncoderSpeedInc;
extern uint16_t SampleTimeEncSpeed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	indx ++;

	if (indx >= SampleTimeEncSpeed)
	{
          indx = 0;
          EncoderSpeed_V = ((Enc_V - oldpos_V) * (1000 / SampleTimeEncSpeed));  // speed in clicks/sec
          if (EncoderSpeed_V > MINEncoderSpeed)
          {
            if(Enc_V + (1000 / SampleTimeEncSpeed) * EncoderSpeed_V > htim3.Init.Period)
            {
              __HAL_TIM_SET_COUNTER(&htim3, htim3.Init.Period);
              Enc_V = htim3.Init.Period;
              oldpos_V = htim3.Init.Period;
              htim17.Instance -> CCR1 = htim3.Init.Period;
            }
            else
            {
              __HAL_TIM_SET_COUNTER(&htim3,Enc_V + (1000 / SampleTimeEncSpeed) * EncoderSpeed_V);
              oldpos_V = Enc_V + (1000 / SampleTimeEncSpeed) * EncoderSpeed_V;
              Enc_V = oldpos_V;
              htim17.Instance -> CCR1 = Enc_V + (1000 / SampleTimeEncSpeed) * EncoderSpeed_V;
            }
            DispEncV = Disp3s;
          }
          else if (EncoderSpeed_V < -MINEncoderSpeed)
          {
            if(Enc_V < (1000 / SampleTimeEncSpeed) * EncoderSpeed_V)
            {
              __HAL_TIM_SET_COUNTER(&htim3, 0);
              htim17.Instance -> CCR1 = 0;
              Enc_V = 0;
              oldpos_V = 0;
            }
            else
            {
              __HAL_TIM_SET_COUNTER(&htim3,Enc_V - (1000 / SampleTimeEncSpeed) * EncoderSpeed_V);
              htim17.Instance -> CCR1 = Enc_V - (1000 / SampleTimeEncSpeed) * EncoderSpeed_V;
              oldpos_V = Enc_V - (1000 / SampleTimeEncSpeed) * EncoderSpeed_V;
              Enc_V = oldpos_V;
            }
            DispEncV = Disp3s;
          }
          else if ((EncoderSpeed_V > 0) | (EncoderSpeed_V < 0))
          {
            oldpos_V = Enc_V;
            htim17.Instance -> CCR1 = Enc_V;
            DispEncV = Disp3s;
          }



          EncoderSpeed_I = ((Enc_I - oldpos_I) * (1000 / SampleTimeEncSpeed));  // speed in clicks/sec
          if (EncoderSpeed_I > MINEncoderSpeed)
          {
            if(Enc_I + (1000 / SampleTimeEncSpeed) * EncoderSpeed_I > htim1.Init.Period)
            {
              __HAL_TIM_SET_COUNTER(&htim1, htim1.Init.Period);
              htim16.Instance -> CCR1 = htim1.Init.Period;
              oldpos_I = htim1.Init.Period;
            }
            else
            {
              __HAL_TIM_SET_COUNTER(&htim1,Enc_I + (1000 / SampleTimeEncSpeed) * EncoderSpeed_I);
              htim16.Instance -> CCR1 = Enc_I + (1000 / SampleTimeEncSpeed) * EncoderSpeed_I;
              oldpos_I = Enc_I + (1000 / SampleTimeEncSpeed) * EncoderSpeed_I;
            }
            DispEncI = Disp3s;
          }
          else if (EncoderSpeed_I < -MINEncoderSpeed)
          {
            if(Enc_I < (1000 / SampleTimeEncSpeed) * EncoderSpeed_I)
            {
              __HAL_TIM_SET_COUNTER(&htim1, 0);
              htim16.Instance -> CCR1 = 0;
              oldpos_I = 0;
            }
            else
            {
              __HAL_TIM_SET_COUNTER(&htim1,Enc_I - (1000 / SampleTimeEncSpeed) * EncoderSpeed_I);
              htim16.Instance -> CCR1 = Enc_I - (1000 / SampleTimeEncSpeed) * EncoderSpeed_I;
              oldpos_I = Enc_I - (1000 / SampleTimeEncSpeed) * EncoderSpeed_I;
            }
            DispEncV = Disp3s;
          }
          else if ((EncoderSpeed_I > 0) | (EncoderSpeed_I < 0))
          {
            htim16.Instance -> CCR1 = Enc_I;
            oldpos_I = Enc_I;
            DispEncI = Disp3s;
          }
        }

  
  
  
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXI_OUT_Pin);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXI_S1_Pin);
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(EXI_S2_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
