#pragma once

#ifdef PE43711_ENABLE

#include "main.h"

typedef struct PE43711_Pins {
  GPIO_TypeDef *LE_Port;
  uint16_t LE_Pin;
  GPIO_TypeDef *SI_Port;
  uint16_t SI_Pin;
  GPIO_TypeDef *CLK_Port;
  uint16_t CLK_Pin;
} PE43711_Pins;

// Initializes the PE43711
void PE43711_Init(PE43711_Pins *pins);
// Set the attenuation of the PE43711. Unit is dB. Range is 0 to 31.75 dB.
void PE43711_SetAttenuation(PE43711_Pins *pins, double attenuation);

#endif