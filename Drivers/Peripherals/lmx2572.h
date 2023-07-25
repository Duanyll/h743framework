#pragma once

#ifdef LMX2572_ENABLE

#include "main.h"

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
} LMX2572_Pins;

void LMX2572_SendData(uint32_t data);
void LMX2572_SendDataArray(uint32_t *a, uint32_t num);

// I don't know the difference between SendData and Write.
void LMX2572_Init(LMX2572_Pins *pins);

void LMX2572_SetFrequency(uint32_t freq);

#endif