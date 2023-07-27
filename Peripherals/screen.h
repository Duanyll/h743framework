#pragma once

#include "main.h"
#include <stdarg.h>

void SCREEN_EndLine();
void SCREEN_Init(UART_HandleTypeDef *huart);
void SCREEN_SetPage(const char *page);
void SCREEN_SetText(const char *controlId, const char *text);
void SCREEN_PrintText(const char *controlId, const char *format, ...);

void SCREEN_TransmitPlotData(const char *controlId, int channel, uint8_t *data,
                             int len);

#define SCREEN_OUTOUT_BUF_SIZE 32
#define SCREEN_DROP_AVERAGE_THRESHOLD 0.1

typedef struct {
  double buf[SCREEN_OUTOUT_BUF_SIZE]; // Circular queue
  int head;
  int tail;
  int size;
  double sum;
  int lastUpdateTick;
  int updateInterval;
  char controlId[8];
  char format[8];
} SCREEN_NumberBuffer;

void SCREEN_NumberBuffer_Init(SCREEN_NumberBuffer *buf, const char *controlId,
                              const char *format, int updateInterval);
void SCREEN_NumberBuffer_Update(SCREEN_NumberBuffer *buf, double value);

#define SCREEN_TOUCH_EVENT 0x65