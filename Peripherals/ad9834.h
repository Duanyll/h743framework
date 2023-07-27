#pragma once

#include "common.h"

/* ------------------------------- AD9834 Pins ------------------------------ */
// DDS_SDA
// DDS_SCK
// DDS_FSY

/* ------------------------------ AD9834 Usage ------------------------------ */
// 1. Call AD9834_Init() to init device
// 2. Create AD9834_Config structure and init it with AD9834_InitConfig()
// 3. Edit control registers in AD9834_Config structure. Then call
// AD9834_WriteControl() to apply changes. See datasheet for register details.
// 4. Set frequency with AD9834_SetFreq() and phase with AD9834_SetPhase()
// functions.

typedef struct {
  GPIO_TypeDef *SDA_Port;
  uint16_t SDA_Pin;
  GPIO_TypeDef *SCK_Port;
  uint16_t SCK_Pin;
  GPIO_TypeDef *FSY_Port;
  uint16_t FSY_Pin;
} AD9834_Pins;

typedef struct AD9834_Config {
  union {
    struct {
      uint16_t reserved_0 : 1;
      uint16_t mode : 1;
      uint16_t reserved_1 : 1;
      uint16_t div2 : 1;
      uint16_t sign_pib : 1;
      uint16_t opbiten : 1;
      uint16_t sleep12 : 1;
      uint16_t sleep1 : 1;
      uint16_t reset : 1;
      uint16_t pin_sw : 1; // Do not modify
      uint16_t psel : 1;
      uint16_t fsel : 1;
      uint16_t hlb : 1; // Do not modify
      uint16_t b28 : 1; // Do not modify
      uint16_t reserved_2 : 2;
    };
    uint16_t value;
  } control;
  uint32_t freq[2];
  uint16_t phase[2];
} AD9834_Config;

// Init AD9834 device
void AD9834_Init(AD9834_Pins *p);
void AD9834_WriteRaw(uint16_t data);
// Init AD9834_Config structure
void AD9834_InitConfig(AD9834_Config *config);
// Write control registers to AD9834
void AD9834_WriteControl(AD9834_Config *config);
// Write frequency registers to AD9834
void AD9834_WriteFreq(AD9834_Config *config, uint8_t reg);
// Write phase registers to AD9834
void AD9834_WritePhase(AD9834_Config *config, uint8_t reg);

// Set frequency in Hz. It calls AD9834_WriteFreq() to apply changes.
void AD9834_SetFreq(AD9834_Config *config, uint8_t reg, double freq,
                    double mclk);
// Set phase in degrees. It calls AD9834_WritePhase() to apply changes.
void AD9834_SetPhase(AD9834_Config *config, uint8_t reg, double phase);