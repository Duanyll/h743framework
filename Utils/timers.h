#pragma once

#include "main.h"
#include "tim.h"

// TODO: Add enabled timers here
#define FOR_ALL_TIMERS(_)                                                      \
  _(TIM2)                                                                      \
  _(TIM3)                                                                      \
  _(TIM4)                                                                      \
  _(TIM5)                                                                      \
  _(TIM6)                                                                      \
  _(TIM7)                                                                      
  
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