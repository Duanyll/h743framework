#pragma once
#include <stm32h7xx_hal.h>

#define KEYS_COUNT 4
#define KEYS_DEBOUNCE_TIME 50
#define KEYS_LONG_PRESS_TIME 1000
#define KEYS_STATE_RELEASE 0
#define KEYS_STATE_PRESS 1
#define KEYS_STATE_LONG_PRESS 2

void KEYS_Init();
void KEYS_Scan();
uint8_t KEYS_GetState(uint8_t key);
void KEYS_SetHandler(void (*handler)(uint8_t key, uint8_t state));