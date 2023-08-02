#include "lmx2572.h"
#include "cmsis_gcc.h"
#include "stm32h7xx_hal.h"
#include "timers.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static LMX2572_Pins *pins;
#define WRITE(pin, val)                                                        \
  HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, val)

void LMX2572_Delay() {
  for (int i = 0; i < 30; i++) {
    __NOP();
  }
}

void LMX2572_WriteRegister(uint8_t addr, uint16_t data) {
  WRITE(CS, LOW);
  for (int i = 7; i > 0; i--) {
    WRITE(SCK, LOW);
    WRITE(SDI, (addr >> i) & 1);
    LMX2572_Delay();
    WRITE(SCK, HIGH);
    LMX2572_Delay();
  }
  for (int i = 15; i > 0; i--) {
    WRITE(SCK, LOW);
    WRITE(SDI, (data >> i) & 1);
    LMX2572_Delay();
    WRITE(SCK, HIGH);
    LMX2572_Delay();
  }
  WRITE(SCK, LOW);
  LMX2572_Delay();
  WRITE(CS, HIGH);
  LMX2572_Delay();
}

uint16_t LMX2572_ReadRegister(uint8_t addr) {
  if (pins->MUXOUT_Port == NULL) {
    return 0;
  }
  addr |= 0x80;
  WRITE(CS, LOW);
  for (int i = 7; i > 0; i--) {
    WRITE(SCK, LOW);
    WRITE(SDI, (addr >> i) & 1);
    LMX2572_Delay();
    WRITE(SCK, HIGH);
    LMX2572_Delay();
  }
  uint16_t data = 0;
  for (int i = 15; i > 0; i--) {
    WRITE(SCK, LOW);
    LMX2572_Delay();
    WRITE(SCK, HIGH);
    data |= HAL_GPIO_ReadPin(pins->MUXOUT_Port, pins->MUXOUT_Pin) << i;
    LMX2572_Delay();
  }
  WRITE(SCK, LOW);
  LMX2572_Delay();
  WRITE(CS, HIGH);
  LMX2572_Delay();
  return data;
}

void LMX2572_Reset() {
  if (pins->ENABLE_Port != NULL) {
    WRITE(ENABLE, LOW);
    TIM_DelayUs(1000);
    WRITE(ENABLE, HIGH);
  }
  TIM_DelayUs(1000);
  LMX2572_WriteRegister(0x00, 0x221E);
}

void LMX2572_Update(BOOL enable) {
  if (enable) {
    LMX2572_WriteRegister(0x00, 0x201C);
  } else {
    LMX2572_WriteRegister(0x00, 0x221C);
  }
}

void LMX2572_Init(LMX2572_Pins *p, uint32_t refin) {
  pins = p;
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  s.Pin = pins->CS_Pin;
  HAL_GPIO_Init(pins->CS_Port, &s);
  s.Pin = pins->SCK_Pin;
  HAL_GPIO_Init(pins->SCK_Port, &s);
  s.Pin = pins->SDI_Pin;
  HAL_GPIO_Init(pins->SDI_Port, &s);
  if (pins->ENABLE_Port != NULL) {
    s.Pin = pins->ENABLE_Pin;
    HAL_GPIO_Init(pins->ENABLE_Port, &s);
  }
  if (pins->MUXOUT_Port != NULL) {
    s.Pin = pins->MUXOUT_Pin;
    s.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(pins->MUXOUT_Port, &s);
  }

  WRITE(CS, LOW);
  WRITE(SCK, LOW);
  WRITE(SDI, LOW);

  LMX2572_Reset();
  if (refin == LMX2572_REFIN_40MHZ) {
    // Set PLL_R = 4
    LMX2572_WriteRegister(0x0B, 0xB048);
  } else if (refin == LMX2572_REFIN_50MHZ) {
    // Set PLL_R = 5
    LMX2572_WriteRegister(0x0B, 0xB058);
  }
  // The datasheet suggests to write these registers after reset
  LMX2572_WriteRegister(0x1D, 0x0000);
  LMX2572_WriteRegister(0x1E, 0x18A6);
  LMX2572_WriteRegister(0x34, 0x0421);
  LMX2572_WriteRegister(0x39, 0x0020);

  // Enable VCO fast calibration on R78
  // LMX2572_WriteRegister(0x4E, 0x0165);
  LMX2572_Update(FALSE);
}

#define LMX2572_CHDIV_NONE                                                     \
  -1 // Not a valid register value. Change the output mux to bypass the divider.
#define LMX2572_CHDIV_2 0
#define LMX2572_CHDIV_4 1
#define LMX2572_CHDIV_8 3
#define LMX2572_CHDIV_16 5
#define LMX2572_CHDIV_32 7
#define LMX2572_CHDIV_64 9
#define LMX2572_CHDIV_128 12
#define LMX2572_CHDIV_256 14

static int LMX2572_outAPower = 0x22;
static int LMX2572_outBPower = 0x22;

void LMX2572_SetOutAPower(int power) {
  LMX2572_outAPower = power;
  LMX2572_WriteRegister(0x2C, (power << 8) | 0x00A2);
}

void LMX2572_SelectChannelDivider(double freq, int *chdiv, double *ratio) {
  if (freq >= 3.2e9) {
    *chdiv = LMX2572_CHDIV_NONE;
    *ratio = 1;
  } else if (freq >= 1.6e9) {
    *chdiv = LMX2572_CHDIV_2;
    *ratio = 2;
  } else if (freq >= 800e6) {
    *chdiv = LMX2572_CHDIV_4;
    *ratio = 4;
  } else if (freq >= 400e6) {
    *chdiv = LMX2572_CHDIV_8;
    *ratio = 8;
  } else if (freq >= 200e6) {
    *chdiv = LMX2572_CHDIV_16;
    *ratio = 16;
  } else if (freq >= 100e6) {
    *chdiv = LMX2572_CHDIV_32;
    *ratio = 32;
  } else if (freq >= 50e6) {
    *chdiv = LMX2572_CHDIV_64;
    *ratio = 64;
  } else if (freq >= 25e6) {
    *chdiv = LMX2572_CHDIV_128;
    *ratio = 128;
  } else {
    *chdiv = LMX2572_CHDIV_256;
    *ratio = 256;
  }
}

int LMX2572_GetPFD_DLY_SEL(double fvco) {
  // Under default MASH order 2
  if (fvco < 4e9) {
    return 1;
  } else {
    return 2;
  }
}

void LMX2572_SetFrequency(double freq) {
  // We have to write these registers:
  // R36 - lower 16 bits for PLL_N
  // R37 - PFD_DLY_SEL
  // R38 - upper 16 bits for DEN
  // R39 - lower 16 bits for DEN
  // R42 - upper 16 bits for NUM
  // R43 - lower 16 bits for NUM
  // (R44 - output power and enable)
  // (R71 - SYSREF configuration)
  // R75 - channel divider CHDIV
  // (R78 - VCO calibration)
  if (freq < 12.5e6 || freq > 6.4e9) {
    printf("Invalid frequency: %lf\n", freq);
    return;
  }
  int chdiv;
  double ratio;
  LMX2572_SelectChannelDivider(freq, &chdiv, &ratio);
  uint32_t den = 10e6; // With 10Hz reference, achieve 1Hz resolution
  double f_vco = freq * ratio;
  uint32_t n = f_vco / den; // N is guaranteed to be more than 320 and less than
                            // 640, so it fits in 16 bits
  uint32_t num = (f_vco - n) * den;
  uint8_t pfd_dly_sel = LMX2572_GetPFD_DLY_SEL(f_vco);

  LMX2572_WriteRegister(0x24, n);
  LMX2572_WriteRegister(0x25, (pfd_dly_sel << 8) | 0x0005);
  LMX2572_WriteRegister(0x26, (den >> 16) & 0xFFFF);
  LMX2572_WriteRegister(0x27, den & 0xFFFF);
  LMX2572_WriteRegister(0x2A, (num >> 16) & 0xFFFF);
  LMX2572_WriteRegister(0x2B, num & 0xFFFF);
  if (chdiv != LMX2572_CHDIV_NONE) {
    LMX2572_WriteRegister(0x45, 0xC600 | LMX2572_outBPower);
    LMX2572_WriteRegister(0x4B, (chdiv << 6) | 0x0800);
  } else {
    LMX2572_WriteRegister(0x45, 0xCE00 | LMX2572_outBPower);
    LMX2572_WriteRegister(0x4B, 0x0800);
  }
  LMX2572_Update(TRUE);
  TIM_DelayUs(500);
}