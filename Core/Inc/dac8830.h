#pragma once
#include "main.h"

void DAC8830_Init(void);
void DAC8830_SetVoltage(int chip, uint16_t voltage);

void DAC8830_StartSineWave(int chip, int frequency, int amplitude);
void DAC8830_StopSineWave();
void DAC8830_TimerCallback();