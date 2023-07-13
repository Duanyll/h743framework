#pragma once

#ifdef DAC8830_ENABLE

#include "main.h"

/* ------------------------------- DAC8830 Pins ------------------------------ */
// For dual DAC8830 sharing the same SPI bus
// DA_CS1 DA_CS2
// DA_SCK DA_SDI

void DAC8830_Init(void);
void DAC8830_SetVoltage(int chip, uint16_t voltage);

void DAC8830_StartSineWave(int chip, int frequency, int amplitude);
void DAC8830_StopSineWave();
void DAC8830_TimerCallback();

#endif