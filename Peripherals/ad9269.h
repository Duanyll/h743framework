#pragma once

#include "common.h"

#define AD9269_WRITE_SIGN 0
#define AD9269_READ_SIGN 1
#define AD9269_REG_SPI_CONF 0x00
#define AD9269_REG_CHIP_ID 0x01
#define AD9269_REG_CHIP_GRADE 0x02
#define AD9269_REG_MODES 0x08
#define AD9269_REG_CLOCK 0x09
#define AD9269_REG_CLOCK_DIVIDER 0x0B
#define AD9269_REG_OUTPUT_MODE 0x14
#define AD9269_IO_DIRECTION_INPUT 0
#define AD9269_IO_DIRECTION_OUTPUT 1

typedef struct {
  GPIO_TypeDef *EN_Port;
  uint16_t EN_Pin;
  GPIO_TypeDef *SCK_Port;
  uint16_t SCK_Pin;
  GPIO_TypeDef *SDA_Port;
  uint16_t SDA_Pin;
  GPIO_TypeDef *SLEEP_Port;
  uint16_t SLEEP_Pin;

  union {
    struct {
      uint8_t CONST_0_R : 1;
      uint8_t LSB_FIRST_R : 1;
      uint8_t SOFT_RESET_R : 1;
      uint8_t CONST_1_R : 1;
      uint8_t CONST_1_L : 1;
      uint8_t SOFT_RESET_L : 1;
      uint8_t LSB_FIRST_L : 1; // this register data is symmetric, _L means the
                               // left one bit, *_L=*_R
      uint8_t CONST_0_L : 1;
    };
    uint8_t raw_conf;
  } SPI_Configuration;
  union {
    struct {
      uint8_t open_1 : 1;
      uint8_t chip_id : 7;
    };
    uint8_t raw_chip_id;
  } Chip_ID;
  union {
    struct {
      uint8_t open_4 : 4;
      uint8_t open_1 : 1;
      uint8_t speed_grade_id : 3;
    };
    uint8_t raw_chip_grade;
  } Chip_grade;
  union {
    struct {
      uint8_t power_mode : 2;
      uint8_t open_2 : 3;
      uint8_t external_pin_function : 2;
      uint8_t external_power_down_enable : 1;
    };
    uint8_t raw_modes;
  } Modes;
  union {
    struct {
      uint8_t duty_cycle_stabilize : 1;
      uint8_t open_7 : 7;
    };
    uint8_t raw_clock;
  } Clock;
  union {
    struct {
      uint8_t clock_divider : 3;
      uint8_t open_5 : 5;
    };
    uint8_t raw_clock_divider;
  } Clock_divider;
  union {
    struct {
      uint8_t output_format : 2;
      uint8_t output_invert : 1;
      uint8_t open1 : 1;
      uint8_t output_disable : 1;
      uint8_t output_mux_enable : 1;
      uint8_t output_voltage : 2;
    };
    uint8_t raw_output_mode;
  } Output_mode;

} AD9269;

void AD9269_Init(AD9269 *device);
void AD9269_WriteRegister(AD9269 *device, uint8_t address, uint8_t data);
void AD9269_ReadRegister(AD9269 *device, uint8_t address,
                     uint8_t *dst); // read one register
void AD9269_ReadConfigFromDevice(AD9269 *device);