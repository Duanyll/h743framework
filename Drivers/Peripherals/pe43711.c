#ifdef PE43711_ENABLE

#include <math.h>

#include "pe43711.h"

void PE43711_Init(PE43711_Config* config) {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    GPIO_InitStruct.Pin = config->LE_Pin;
    HAL_GPIO_Init(config->LE_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = config->SI_Pin;
    HAL_GPIO_Init(config->SI_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = config->CLK_Pin;
    HAL_GPIO_Init(config->CLK_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(config->LE_Port, config->LE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->SI_Port, config->SI_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(config->CLK_Port, config->CLK_Pin, GPIO_PIN_RESET);
}

void PE43711_Delay() {
    for (int i = 0; i < 20; i++) {
        __NOP();
    }
}

void PE43711_Write(PE43711_Config* config, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(config->SI_Port, config->SI_Pin, ((data >> i) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        PE43711_Delay();
        HAL_GPIO_WritePin(config->CLK_Port, config->CLK_Pin, GPIO_PIN_SET);
        PE43711_Delay();
        HAL_GPIO_WritePin(config->CLK_Port, config->CLK_Pin, GPIO_PIN_RESET);
    }
    PE43711_Delay();
    HAL_GPIO_WritePin(config->LE_Port, config->LE_Pin, GPIO_PIN_SET);
    PE43711_Delay();
    HAL_GPIO_WritePin(config->LE_Port, config->LE_Pin, GPIO_PIN_RESET);
}

void PE43711_SetAttenuation(PE43711_Config* config, double attenuation) {
    uint8_t data = (uint8_t)round(attenuation * 4);
    data = data > 0x7F ? 0x7F : data;
    PE43711_Write(config, data);
}

#endif