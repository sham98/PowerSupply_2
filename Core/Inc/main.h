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

#define IF_Nor          0
#define IF_Test         0
#define HTIM_ENC_CURR       htim1
#define HTIM_ENC_VOL        htim3
#define HTIM_PWM_CURR       htim16
#define HTIM_PWM_VOL        htim17
#define MAXSumError         50000

#define MaxEncSpeed             15
#define MinEncSpeed             4
#define MaxSamEncTime    30
#define iKepEXI         1000
#define iClkEXI         100


#define LED_Num         98
#define ADC_V           AData [0]
#define ADC_I           AData [1]
#define ADC_I_USB       AData [2]
#define LEDM1Num        81
#define LEDM2Num        82
#define LEDM3Num        83
#define LEDM4Num        84
#define LEDOVPNum       85
#define LEDOCPNum       86
#define LEDLockNum      87
#define LEDOUTNum       88
#define RelOutNum       89
#define MAXReadEXI      100
#define CorReadEXI      85

#define LBVolM1Mem          10
#define HBVolM1Mem          11
#define LBCurM1Mem          12
#define HBCurM1Mem          13

#define LBVolM2Mem          20
#define HBVolM2Mem          21
#define LBCurM2Mem          22
#define HBCurM2Mem          23

#define LBVolM3Mem          30
#define HBVolM3Mem          31
#define LBCurM3Mem          32
#define HBCurM3Mem          33

#define LBVolM4Mem          40
#define HBVolM4Mem          41
#define LBCurM4Mem          42
#define HBCurM4Mem          43
#define OfsV            0
#define OfsI            4
#define VolLoc            32
#define CurLoc            0
#define VoltMAX         51200

//#define PID_KP  2.0f
//#define PID_KI  0.5f
//#define PID_KD  0.25f

//#define PID_TAU 0.02f

#define PID_LIM_MIN  0
#define PID_LIM_MAX  51200

#define PID_LIM_MIN_INT  -500000f
#define PID_LIM_MAX_INT  500000f

#define SAMPLE_TIME_S 0.000225f

//#define VOLT2ENC        16
typedef struct
{
	uint16_t Volt;
	uint8_t Status;
	uint8_t Mem;
        uint16_t Enc;
        int16_t SpdEnc;
        uint16_t OldEnc;
        int32_t PWM;
        uint16_t DispEnc;
        uint16_t CountDisp;
        uint8_t  Out;
        uint16_t MaxVolt;
        uint16_t MinVolt;
        uint16_t EncFactor;
        float DispFactor1;
        float DispFactor0;
        uint16_t DispVolt;
        int32_t Err;
        int32_t DErr;
        int32_t OldErr;
        int32_t SumErr;
}Monitor;


typedef struct
{
	uint16_t Volt;
        uint16_t DispEnc;
        uint16_t CountDisp;
        float DispFactor1;
        float DispFactor0;
        uint16_t DispVolt;
}MonitorUSB;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
