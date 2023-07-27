#include <math.h>

#include "pe43711.h"

void PE43711_Init(PE43711_Pins *pins) {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  GPIO_InitStruct.Pin = pins->LE_Pin;
  HAL_GPIO_Init(pins->LE_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->SI_Pin;
  HAL_GPIO_Init(pins->SI_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->CLK_Pin;
  HAL_GPIO_Init(pins->CLK_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(pins->LE_Port, pins->LE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(pins->SI_Port, pins->SI_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(pins->CLK_Port, pins->CLK_Pin, GPIO_PIN_RESET);
}

void PE43711_Delay() {
  for (int i = 0; i < 20; i++) {
    __NOP();
  }
}

void PE43711_Write(PE43711_Pins *pins, uint8_t data) {
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(pins->SI_Port, pins->SI_Pin,
                      ((data >> i) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    PE43711_Delay();
    HAL_GPIO_WritePin(pins->CLK_Port, pins->CLK_Pin, GPIO_PIN_SET);
    PE43711_Delay();
    HAL_GPIO_WritePin(pins->CLK_Port, pins->CLK_Pin, GPIO_PIN_RESET);
  }
  PE43711_Delay();
  HAL_GPIO_WritePin(pins->LE_Port, pins->LE_Pin, GPIO_PIN_SET);
  PE43711_Delay();
  HAL_GPIO_WritePin(pins->LE_Port, pins->LE_Pin, GPIO_PIN_RESET);
}

void PE43711_SetAttenuation(PE43711_Pins *pins, double attenuation) {
  uint8_t data = (uint8_t)round(attenuation * 4);
  data = data > 0x7F ? 0x7F : data;
  PE43711_Write(pins, data);
}