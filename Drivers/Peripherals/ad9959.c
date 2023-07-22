#ifdef AD9959_ENABLE

#include "ad9959.h"

#include "tim.h"
#include <math.h>

#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET
#define WRITE(name, val)                                                       \
  HAL_GPIO_WritePin(AD9959_##name##_GPIO_Port, AD9959_##name##_Pin,            \
                    (val) ? HIGH : LOW)

// Delay at least 10ns
void AD9959_Delay() {
  for (int i = 0; i < 10; i++) {
    __NOP();
  }
}

void AD9959_DelayLong() {
  for (int i = 0; i < 100; i++) {
    __NOP();
  }
}

void AD9959_IOUpdate() {
  WRITE(IOUP, LOW);
  AD9959_DelayLong();
  WRITE(IOUP, HIGH);
  AD9959_DelayLong();
  WRITE(IOUP, LOW);
  AD9959_DelayLong();
}

void AD9959_Reset(AD9959_GlobalConfig *config) {
  WRITE(RST, LOW);
  AD9959_DelayLong();
  WRITE(RST, HIGH);
  AD9959_DelayLong();
  WRITE(RST, LOW);
  AD9959_DelayLong();

  AD9959_InitGlobalConfig(config);
  AD9959_Write(AD9959_FR1_ADDR, config->fr1.raw);
  AD9959_Write(AD9959_CFR_ADDR, 0x000300);
  AD9959_IOUpdate();
}

void AD9959_SetSDIO0Input() {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Pin = AD9959_SDIO0_Pin,
      .Mode = GPIO_MODE_INPUT,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(AD9959_SDIO0_GPIO_Port, &GPIO_InitStruct);
}

void AD9959_SetSDIO0Output() {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Pin = AD9959_SDIO0_Pin,
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(AD9959_SDIO0_GPIO_Port, &GPIO_InitStruct);
}

void AD9959_WriteRaw(uint8_t data) {
  AD9959_SetSDIO0Output();
  AD9959_Delay();
  for (int i = 7; i >= 0; i--) {
    WRITE(SCLK, LOW);
    WRITE(SDIO0, (data >> i) & 1);
    AD9959_Delay();
    WRITE(SCLK, HIGH);
    AD9959_Delay();
  }
  AD9959_Delay();
}

uint8_t AD9959_ReadRaw() {
  AD9959_SetSDIO0Input();
  AD9959_Delay();
  uint8_t data = 0;
  for (int i = 7; i >= 0; i--) {
    WRITE(SCLK, LOW);
    AD9959_Delay();
    data |= HAL_GPIO_ReadPin(AD9959_SDIO0_GPIO_Port, AD9959_SDIO0_Pin) << i;
    WRITE(SCLK, HIGH);
    AD9959_Delay();
  }
  AD9959_Delay();
  return data;
}

const int AD9959_REGISTER_SIZE[] = {1, 3, 2, 3, 4, 2, 3, 2, 4, 4, 4, 4, 4,
                                    4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};

void AD9959_Write(uint8_t addr, uint32_t data) {
  WRITE(CSB, LOW);
  AD9959_WriteRaw(addr);
  AD9959_Delay();
  for (int i = AD9959_REGISTER_SIZE[addr]; i > 0; i--) {
    AD9959_WriteRaw(data >> (8 * (i - 1)));
  }
  WRITE(CSB, HIGH);
}

uint32_t AD9959_Read(uint8_t addr) {
  WRITE(CSB, LOW);
  AD9959_WriteRaw(addr | 0x80);
  AD9959_Delay();
  uint32_t data = 0;
  for (int i = AD9959_REGISTER_SIZE[addr]; i > 0; i--) {
    data |= AD9959_ReadRaw() << (8 * (i - 1));
  }
  WRITE(CSB, HIGH);
  return data;
}

void AD9959_Init(AD9959_GlobalConfig *config) {
  WRITE(CSB, HIGH);
  WRITE(RST, LOW);
  WRITE(IOUP, LOW);
  WRITE(SCLK, HIGH);
  WRITE(SDIO0, LOW);

  AD9959_Reset(config);
}

void AD9959_InitGlobalConfig(AD9959_GlobalConfig *config) {
  config->csr.raw = 0xF0;
  config->fr1.raw = 0;
  config->fr1.vco_gain_control = 1;
  config->fr1.pll_divider_ratio = 20; // 25MHz oscillator * 20 = 500MHz VCO
  config->fr2.raw = 0;
}
void AD9959_InitChannelConfig(AD9959_ChannelConfig *config) {
  config->cfr.raw = 0;
  config->cfr.dac_full_scale_current = 3;
  config->cftw0 = 0;
  config->cpow0 = 0;
  config->acr.raw = 0;
  config->acr.amplitude_multiplier_enable = 1;
  config->lsrr.raw = 0;
  config->rdw = 0;
  config->fdw = 0;
  for (int i = 0; i < 15; i++) {
    config->tuning_words[i] = 0;
  }
}

uint32_t AD9959_CalculateFTW(double freq, double sysclk) {
  return (uint32_t)round(freq / sysclk * (1ll << 32));
}

uint16_t AD9959_CalculatePOW(double phase) {
  return (uint16_t)round(phase / 360 * (1ll << 14));
}

// Set frequency for AD9959 channel. Manual trigger IOUpdate to apply change.
// Unit: Hz
void AD9959_SetFrequency(AD9959_ChannelConfig *config, double freq,
                         double sysclk) {
  config->cftw0 = AD9959_CalculateFTW(freq, sysclk);
  AD9959_Write(AD9959_CFTW0_ADDR, config->cftw0);
}
// Set phase for AD9959 channel. Manual trigger IOUpdate to apply change. Unit:
// degree
void AD9959_SetPhase(AD9959_ChannelConfig *config, double phase) {
  config->cpow0 = AD9959_CalculatePOW(phase);
  AD9959_Write(AD9959_CPOW0_ADDR, config->cpow0);
}
// Set amplitude for AD9959 channel. Manual trigger IOUpdate to apply change. 10
// bits resolution.
void AD9959_SetAmplitude(AD9959_ChannelConfig *config, uint16_t amplitude) {
  config->acr.amplitude_scale_factor = amplitude;
  AD9959_Write(AD9959_ACR_ADDR, config->acr.raw);
}

void AD9959_SelectChannels(AD9959_GlobalConfig *config, uint8_t channels) {
  config->csr.raw = (config->csr.raw & 0x0F) | ((channels & 0xF) << 4);
  AD9959_Write(AD9959_CSR_ADDR, config->csr.raw);
}

#endif