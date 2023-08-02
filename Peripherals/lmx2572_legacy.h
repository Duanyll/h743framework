#pragma once

#include "common.h"

// This driver is full of magic numbers.  I have no idea what they do, but they work.

typedef struct {
  GPIO_TypeDef *CS_Port;
  uint16_t CS_Pin;
  GPIO_TypeDef *SCK_Port;
  uint16_t SCK_Pin;
  GPIO_TypeDef *SDI_Port;
  uint16_t SDI_Pin;
  GPIO_TypeDef *ENABLE_Port;
  uint16_t ENABLE_Pin;
} LMX2572L_Pins;

void LMX2572L_SendData(uint32_t data);
void LMX2572L_SendDataArray(uint32_t *a, uint32_t num);

void LMX2572L_Init(LMX2572L_Pins *pins);

void LMX2572L_SetFrequency(uint32_t freq);