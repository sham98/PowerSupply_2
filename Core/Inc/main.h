/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AI_V_Pin GPIO_PIN_0
#define AI_V_GPIO_Port GPIOA
#define AI_I_Pin GPIO_PIN_1
#define AI_I_GPIO_Port GPIOA
#define AI_IUSB_Pin GPIO_PIN_2
#define AI_IUSB_GPIO_Port GPIOA
#define O_SER_Pin GPIO_PIN_3
#define O_SER_GPIO_Port GPIOA
#define O_RCK_Pin GPIO_PIN_4
#define O_RCK_GPIO_Port GPIOA
#define O_CLK_Pin GPIO_PIN_5
#define O_CLK_GPIO_Port GPIOA
#define PWM_I_Pin GPIO_PIN_6
#define PWM_I_GPIO_Port GPIOA
#define PWM_V_Pin GPIO_PIN_7
#define PWM_V_GPIO_Port GPIOA
#define PWM_FAN_Pin GPIO_PIN_0
#define PWM_FAN_GPIO_Port GPIOB
#define EXI_OUT_Pin GPIO_PIN_1
#define EXI_OUT_GPIO_Port GPIOB
#define EXI_OUT_EXTI_IRQn EXTI0_1_IRQn
#define ENC_IB_Pin GPIO_PIN_8
#define ENC_IB_GPIO_Port GPIOA
#define ENC_IA_Pin GPIO_PIN_9
#define ENC_IA_GPIO_Port GPIOA
#define O_S2_Pin GPIO_PIN_10
#define O_S2_GPIO_Port GPIOA
#define O_S1_Pin GPIO_PIN_11
#define O_S1_GPIO_Port GPIOA
#define EXI_S2_Pin GPIO_PIN_15
#define EXI_S2_GPIO_Port GPIOA
#define EXI_S2_EXTI_IRQn EXTI4_15_IRQn
#define EXI_S1_Pin GPIO_PIN_3
#define EXI_S1_GPIO_Port GPIOB
#define EXI_S1_EXTI_IRQn EXTI2_3_IRQn
#define ENC_VB_Pin GPIO_PIN_4
#define ENC_VB_GPIO_Port GPIOB
#define ENC_VA_Pin GPIO_PIN_5
#define ENC_VA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//#define Enc_V	__HAL_TIM_GET_COUNTER(&htim3)
//#define Enc_I	__HAL_TIM_GET_COUNTER(&htim1)
//#define Disp3s          3000
#define IF_Nor          0
#define IF_Test         1


typedef struct
{
	uint16_t Volt;
	uint8_t Status;
	uint8_t Mem;
        int32_t Enc;
        int16_t SpdEnc;
        int16_t OldEnc;
        uint16_t PWM;
        uint16_t DispEnc;
        uint16_t CountDisp;
        uint8_t  Out;
        uint16_t MaxVolt;
        uint16_t EncFactor;
        uint8_t LowOfset;
        uint8_t HighOfset;
}Monitor;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
