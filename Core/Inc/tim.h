/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.h
 * @brief   This file contains all the function prototypes for
 *          the tim.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim5;

extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM6_Init(void);

/* USER CODE BEGIN Prototypes */
// Delay in microseconds with SysTick
void TIM_DelayUs(uint32_t us);

// Start a timer that counts microseconds
void TIM_StartUsTimer(TIM_HandleTypeDef *htim);
// Get a microsecond timestamp from the timer
int64_t TIM_GetUsTimer();
// Stop the timer
void TIM_StopUsTimer();

typedef void (*TIM_Callback)(void);
// Register a callback to be called when the timer triggers
void TIM_RegisterCallback(TIM_HandleTypeDef *htim, TIM_Callback callback);
// Unregister a callback
void TIM_UnregisterCallback(TIM_HandleTypeDef *htim);
// Start a timer that triggers at a given sample rate. This function assumes the
// prescaler is set to a proper value for the given sample rate and then
// calculates the auto reload value.
void TIM_StartPeriodic(TIM_HandleTypeDef *htim, double sampleRate);
// Stop the timer
void TIM_StopPeriodic(TIM_HandleTypeDef *htim);

double TIM_CountFrequencySync(TIM_HandleTypeDef *htim, int periodMs);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

