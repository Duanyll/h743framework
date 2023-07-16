#pragma once

#include <stdarg.h>
#include "main.h"

void SCREEN_EndLine();
void SCREEN_Init(UART_HandleTypeDef *huart);
void SCREEN_SetPage(const char* page);
void SCREEN_SetText(const char* controlId, const char* text);
void SCREEN_PrintText(const char* controlId, const char* format, ...);

void SCREEN_TransmitPlotData(const char* controlId, int channel, uint8_t *data, int len);

#define SCREEN_TOUCH_EVENT 0x65