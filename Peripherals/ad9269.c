//
// Created by RisingEntropy on 2023/7/14.
//
#include "ad9269.h"

void AD9269_switch_GPIO_mode(AD9269 *device, uint8_t direction) {
  if (direction == AD9269_IO_DIRECTION_INPUT) {
    static GPIO_InitTypeDef input_mode;
    input_mode.Pin = device->SDA_Pin;
    input_mode.Mode = GPIO_MODE_INPUT;
    input_mode.Pull = GPIO_NOPULL;
    input_mode.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->SDA_Port, &input_mode);
  } else if (direction == AD9269_IO_DIRECTION_OUTPUT) {
    static GPIO_InitTypeDef output_mode;
    output_mode.Pin = device->SDA_Pin;
    output_mode.Mode = GPIO_MODE_OUTPUT_PP;
    output_mode.Pull = GPIO_NOPULL;
    output_mode.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->SDA_Port, &output_mode);
  }
}
void AD9269_delay() {
  volatile uint64_t dly = 50;
  while (dly--)
    __NOP();
}
void AD9269_Init(AD9269 *device) {
  //    device->Clock_divider.raw_clock_divider = 0x01;
  //    device->Output_mode.raw_output_mode = 0x01;
  HAL_GPIO_WritePin(device->SLEEP_Port, device->SLEEP_Pin,
                    GPIO_PIN_SET); // clear sleep state
  HAL_Delay(10);
  HAL_GPIO_WritePin(device->SLEEP_Port, device->SLEEP_Pin,
                    GPIO_PIN_RESET); // clear sleep state
  AD9269_delay();
  AD9269_WriteRegister(device, AD9269_REG_CLOCK_DIVIDER, 0x00); // no clock division
  AD9269_delay();
  AD9269_WriteRegister(device, AD9269_REG_OUTPUT_MODE, 0x81); // 2's complement
  AD9269_delay();
  AD9269_WriteRegister(device, 0xFF, 0x01); // update
  AD9269_delay();
}
void AD9269_WriteRegister(AD9269 *device, uint8_t address, uint8_t data) {
  AD9269_switch_GPIO_mode(device, AD9269_IO_DIRECTION_OUTPUT);
  HAL_GPIO_WritePin(device->EN_Port, device->EN_Pin,
                    GPIO_PIN_RESET); // enable the device
  uint16_t cmd = 0;
  cmd = (AD9269_WRITE_SIGN << 15) | address;
  //    cmd|=0x6000;
  for (int i = 0; i < 16; i++) {
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
    AD9269_delay();
    if (cmd & 0x8000) {
      HAL_GPIO_WritePin(device->SDA_Port, device->SDA_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(device->SDA_Port, device->SDA_Pin, GPIO_PIN_RESET);
    }
    AD9269_delay();
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_SET);
    AD9269_delay();
    cmd <<= 1;
  }
  cmd = data;
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
    if (cmd & 0x80) {
      HAL_GPIO_WritePin(device->SDA_Port, device->SDA_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(device->SDA_Port, device->SDA_Pin, GPIO_PIN_RESET);
    }
    AD9269_delay();
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_SET);
    AD9269_delay();
    cmd <<= 1;
  }
  HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(device->EN_Port, device->EN_Pin, GPIO_PIN_SET);
}
void AD9269_ReadRegister(AD9269 *device, uint8_t address, uint8_t *dst) {
  AD9269_switch_GPIO_mode(device, AD9269_IO_DIRECTION_OUTPUT);
  HAL_GPIO_WritePin(device->EN_Port, device->EN_Pin,
                    GPIO_PIN_RESET); // enable the device
  uint16_t cmd = 0;
  cmd = (AD9269_READ_SIGN << 15) | address;
  //    cmd|=0x6000;
  for (int i = 0; i < 16; i++) {
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
    AD9269_delay();
    if (cmd & 0x8000) {
      HAL_GPIO_WritePin(device->SDA_Port, device->SDA_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(device->SDA_Port, device->SDA_Pin, GPIO_PIN_RESET);
    }
    AD9269_delay();
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_SET);
    AD9269_delay();
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
    cmd <<= 1;
  }

  AD9269_delay();
  *dst = 0;
  AD9269_switch_GPIO_mode(device, AD9269_IO_DIRECTION_INPUT);
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
    AD9269_delay();
    HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_SET);
    AD9269_delay();
    *dst <<= 1;
    if (HAL_GPIO_ReadPin(device->SDA_Port, device->SDA_Pin) == GPIO_PIN_RESET) {
      *dst |= 0;
    } else {
      *dst |= 1;
    }
    AD9269_delay();
  }
  HAL_GPIO_WritePin(device->SCK_Port, device->SCK_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(device->EN_Port, device->EN_Pin, GPIO_PIN_RESET);
}
void AD9269_ReadConfigFromDevice(AD9269 *device) {
  AD9269_ReadRegister(device, AD9269_REG_SPI_CONF,
                  &device->SPI_Configuration.raw_conf);
  AD9269_ReadRegister(device, AD9269_REG_CHIP_ID, &device->Chip_ID.raw_chip_id);
  AD9269_ReadRegister(device, AD9269_REG_CHIP_GRADE,
                  &device->Chip_grade.raw_chip_grade);
  AD9269_ReadRegister(device, AD9269_REG_MODES, &device->Modes.raw_modes);
  AD9269_ReadRegister(device, AD9269_REG_CLOCK, &device->Clock.raw_clock);
  AD9269_ReadRegister(device, AD9269_REG_CLOCK_DIVIDER,
                  &device->Clock_divider.raw_clock_divider);
  AD9269_ReadRegister(device, AD9269_REG_OUTPUT_MODE,
                  &device->Output_mode.raw_output_mode);
}