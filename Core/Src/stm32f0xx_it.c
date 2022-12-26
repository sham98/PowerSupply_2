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


//extern uint16_t MaxSamEncTime;
extern uint16_t MinSamEncTime;

extern Monitor Volt, Curr, USBCurr;
extern uint16_t Disp3s;

extern int16_t MaxEncSpeed;
extern int16_t MaxEncInc;
//extern int16_t MinEncSpeed;

//extern float Kp;
//extern float Ki;
//extern float Kd;
//extern uint8_t VOLT2ENC;
//
//extern uint8_t PIDEn;
//extern int32_t MAXSumError;
uint16_t iPIDSwitch = 0;
uint8_t PIDSwitch = 0;
#if (IF_VOLTTest | IF_CURRTest)
extern uint16_t iTriInd;
extern uint16_t ITriIndMax;
extern int16_t iTriangle;
extern uint16_t iTriMax;
extern uint16_t Ramp;
extern uint16_t TriVolStep;
#endif


#if PIDTunning
extern uint16_t ENCSTP1;
extern uint16_t ENCSTP2;
extern uint16_t MaxiPIDSwitch;
#endif
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

#if IF_VOLTTest
  if (iTriInd >= ITriIndMax)
  {
    iTriInd = 0;
    
    if (Ramp == 2)
    {
      iTriangle = iTriangle - TriVolStep;
      if (iTriangle < 0)
        iTriangle = 0;
    }
    else if (Ramp == 1)
    {
      iTriangle = iTriangle + TriVolStep;
    }


    if (iTriangle >= iTriMax)
    {
      Ramp = 2;
    }
    else if (iTriangle == 0)
    {
      Ramp = 1;
    }

    __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, iTriangle);
  }  
  else
  {
    iTriInd ++;
  }  
#endif

#if IF_CURRTest
  if (iTriInd >= ITriIndMax)
  {
    iTriInd = 0;
    
    if (Ramp == 2)
    {
      iTriangle = iTriangle - TriVolStep;
      if (iTriangle < 0)
        iTriangle = 0;
    }
    else if (Ramp == 1)
    {
      iTriangle = iTriangle + TriVolStep;
    }


    if (iTriangle >= iTriMax)
    {
      Ramp = 2;
    }
    else if (iTriangle == 0)
    {
      Ramp = 1;
    }

    __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, iTriangle);
  }  
  else
  {
    iTriInd ++;
  }  
#endif
  
  
#if PIDTunning
  if (iPIDSwitch > MaxiPIDSwitch)
  {
    iPIDSwitch = 0;
    if (PIDSwitch == 0)
    {
      PIDSwitch = 1;
//      Volt.PWM = 20000 / Volt.EncFactor;
//      __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.PWM);
      Volt.Enc = ENCSTP1;
      __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);
      Volt.CountPID = 0;
    }
    else
    {
      PIDSwitch = 0;      
//      Volt.PWM = 25000 / Volt.EncFactor;
//      __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.PWM);
      Volt.Enc = ENCSTP2;
      __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);
      Volt.CountPID = 0;
    }
  }
  else
    iPIDSwitch ++;
#endif  
  


  int8_t Kmin = 1;
  indx ++;
  if (indx >= MaxSamEncTime)
  {
    indx = 0;
    if (Volt.Enc != Volt.OldEnc)
    {
      Volt.SpdEnc = Volt.Enc - Volt.OldEnc;
      if (Volt.SpdEnc < 0)
      {
        Kmin = -1;
        Volt.SpdEnc = Kmin * Volt.SpdEnc;
      }
      else
      {
        Kmin = 1;
      }
      Volt.CountDisp = Disp3s;

      if (Volt.SpdEnc >= MaxEncSpeed)
      {
        int32_t TempVolt = Kmin * MaxEncInc + Volt.Enc;
        if (TempVolt > HTIM_ENC_VOL.Init.Period)
        {
          Volt.Enc = HTIM_ENC_VOL.Init.Period;
        }
        else if (TempVolt < 0)
        {
          Volt.Enc = 0;
        }
        else
        {
          Volt.Enc = TempVolt;
        }
      }
      else if (Volt.SpdEnc <= MinEncSpeed)
      {
      }
      else 
      {
        int32_t TempVolt = Volt.Enc + Kmin * ((MaxEncInc - 4)/(MaxEncSpeed - 4) * Volt.SpdEnc - (4 * MaxEncInc - 4 * MaxEncSpeed)/(MaxEncSpeed - 4));
        if (TempVolt > HTIM_ENC_VOL.Init.Period)
        {
          Volt.Enc = HTIM_ENC_VOL.Init.Period;
        }
        else if (TempVolt < 0)
        {
          Volt.Enc = 0;
        }
        else
        {
          Volt.Enc = TempVolt;
        }
      }
      
      if (Volt.Enc > Volt.MaxVolt)
      {
        Volt.Enc = Volt.MaxVolt;
      }
      else if (Volt.Enc < 0)
      {
        Volt.Enc = 0;
      }
      Volt.OldEnc = Volt.Enc;
      __HAL_TIM_SET_COUNTER(&HTIM_ENC_VOL, Volt.Enc);
      __HAL_TIM_SET_COMPARE(&HTIM_PWM_VOL, TIM_CHANNEL_1, Volt.Enc / Volt.EncFactor);
    }    



    if (Curr.Enc != Curr.OldEnc)
    {
      Curr.SpdEnc = Curr.Enc - Curr.OldEnc;
      if (Curr.SpdEnc < 0)
      {
        Kmin = -1;
        Curr.SpdEnc = Kmin * Curr.SpdEnc;
      }
      else
      {
        Kmin = 1;
      }
      Curr.CountDisp = Disp3s;

      if (Curr.SpdEnc >= MaxEncSpeed)
      {
        int32_t TempCurr = Kmin * MaxEncInc + Curr.Enc;
        if (TempCurr < 0)
        {
          Curr.Enc = 0;
        }
        else if (TempCurr > HTIM_ENC_CURR.Init.Period)
        {
          Curr.Enc = HTIM_ENC_CURR.Init.Period;
        }
        else
        {
          Curr.Enc = TempCurr;
        }
      }
      else if (Curr.SpdEnc <= MinEncSpeed)
      {
      }
      else 
      {
        int32_t TempCurr = Curr.Enc + Kmin * ((MaxEncInc - 4)/(MaxEncSpeed - 4) * Curr.SpdEnc - (4 * MaxEncInc - 4 * MaxEncSpeed)/(MaxEncSpeed - 4));
        if (TempCurr < 0)
        {
          Curr.Enc = 0;
        }
        else if (TempCurr > HTIM_ENC_CURR.Init.Period)
        {
          Curr.Enc = HTIM_ENC_CURR.Init.Period;
        }
        else
        {
          Curr.Enc = TempCurr;
        }
      }
      
      if (Curr.Enc > Curr.MaxVolt)
      {
        Curr.Enc = Curr.MaxVolt;
      }
      else if (Curr.Enc < 0)
      {
        Curr.Enc = 0;
      }
      Curr.OldEnc = Curr.Enc;
      __HAL_TIM_SET_COUNTER(&HTIM_ENC_CURR, Curr.Enc);
      __HAL_TIM_SET_COMPARE(&HTIM_PWM_CURR, TIM_CHANNEL_1, Curr.Enc / Curr.EncFactor);
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
