#pragma once

#include "main.h"

#define KEYS_MAX_KEYS 8
#define KEYS_SAMPLE_RATE 100
#define KEYS_HOLD_COUNT 10
#define KEYS_LONG_HOLD_COUNT 100
#define KEYS_EVENT_QUEUE_SIZE 16

#define KEYS_EVENT_PRESS 1
#define KEYS_EVENT_RELEASE 2
#define KEYS_EVENT_HOLD 3

typedef void (*KEYS_EventCallback)(uint8_t event);

typedef struct {
  struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    KEYS_EventCallback callback;
  } pins[KEYS_MAX_KEYS];
  uint8_t keyCount;
  TIM_HandleTypeDef *htim;
} KEYS_Pins;


void KEYS_Init(KEYS_Pins *pins);
void KEYS_Start();
void KEYS_Stop();
void KEYS_Poll();