#pragma once

#ifdef DAC8830_ENABLE

#include "main.h"

/* ------------------------------- DAC8830 Pins ------------------------------ */
// For dual DAC8830 sharing the same SPI bus
// DA_CS1 DA_CS2
// DA_SCK DA_SDI
// Enable TIM3 and call DAC8830_TimerCallback() in TIM3_IRQHandler()

/* ------------------------------ DAC8830 Usage ------------------------------ */
// 1. Call DAC8830_Init()
// 2. Set output level by calling DAC8830_SetVoltage(CHIP, VOLTAGE) where CHIP
// is 2-bit flag.
// 3. To start a sine wave asynchronically, call DAC8830_StartSineWave(), and
// stop generating by calling DAC8830_StopSineWave().

void DAC8830_Init(void);
void DAC8830_SetVoltage(int chip, uint16_t voltage);

void DAC8830_StartSineWave(int chip, int frequency, int amplitude);
void DAC8830_StopSineWave();
void DAC8830_TimerCallback();

#endif