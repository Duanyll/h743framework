#include "timers.h"

void TIM_DelayUs(uint32_t us) {
  // Delay in microseconds using SysTick
  __IO uint32_t currentTicks = SysTick->VAL;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  __IO uint32_t oldTicks = currentTicks;
  do {
    currentTicks = SysTick->VAL;
    elapsedTicks += (oldTicks < currentTicks)
                        ? tickPerMs + oldTicks - currentTicks
                        : oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
}

#define DEFINE_TIM_CALLBACK(tim) TIM_Callback tim##_Callback;
FOR_ALL_TIMERS(DEFINE_TIM_CALLBACK);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
#define CALL_IF_MATCH(tim)                                                     \
  if (htim->Instance == tim) {                                                 \
    if (tim##_Callback) {                                                      \
      tim##_Callback();                                                        \
    }                                                                          \
  }
  FOR_ALL_TIMERS(CALL_IF_MATCH);
}

volatile int64_t TIM_usTimer;
TIM_HandleTypeDef *TIM_usTimerHandle;

void TIM_StartUsTimer(TIM_HandleTypeDef *htim) {
  TIM_usTimer = 0;
  TIM_usTimerHandle = htim;
  int prescaler = (HAL_RCC_GetPCLK1Freq() * 2 / 1000000) - 1;
  __HAL_TIM_SET_PRESCALER(htim, prescaler);
  __HAL_TIM_SET_AUTORELOAD(htim, 4999);
  HAL_TIM_Base_Start_IT(htim);
}
int64_t TIM_GetUsTimer() {
  return TIM_usTimer + TIM_usTimerHandle->Instance->ARR + 1 -
         TIM_usTimerHandle->Instance->CNT;
}
void TIM_StopUsTimer() { HAL_TIM_Base_Stop_IT(TIM_usTimerHandle); }

void TIM_RegisterCallback(TIM_HandleTypeDef *htim, TIM_Callback callback) {
#define SET_CALLBACK_IF_MATCH(tim)                                             \
  if (htim->Instance == tim) {                                                 \
    tim##_Callback = callback;                                                 \
  }
  FOR_ALL_TIMERS(SET_CALLBACK_IF_MATCH);
}
void TIM_UnregisterCallback(TIM_HandleTypeDef *htim) {
#define CLEAR_CALLBACK_IF_MATCH(tim)                                           \
  if (htim->Instance == tim) {                                                 \
    tim##_Callback = NULL;                                                     \
  }
  FOR_ALL_TIMERS(CLEAR_CALLBACK_IF_MATCH);
}
void TIM_StartPeriodic(TIM_HandleTypeDef *htim, double sampleRate) {
  int period =
      HAL_RCC_GetPCLK1Freq() * 2 / (htim->Instance->PSC + 1) / sampleRate - 1;
  __HAL_TIM_SET_AUTORELOAD(htim, period);
  __HAL_TIM_SET_COUNTER(htim, 0);
  HAL_TIM_Base_Start_IT(htim);
}
void TIM_StopPeriodic(TIM_HandleTypeDef *htim) { HAL_TIM_Base_Stop_IT(htim); }

double TIM_CountFrequencySync(TIM_HandleTypeDef *htim, int periodMs) {
  __HAL_TIM_SetCounter(htim, 0);
  HAL_TIM_Base_Start(htim);
  HAL_Delay(periodMs);
  HAL_TIM_Base_Stop(htim);
  return (double)htim->Instance->CNT * (htim->Instance->PSC + 1) / periodMs *
         1000;
}