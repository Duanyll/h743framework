#include "hmc833.h"

static HMC833_Pins *pins;

#define WRITE(pin, val)                                                        \
  HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, (val) ? HIGH : LOW)

void HMC833_Init(HMC833_Pins *p) {
  pins = p;
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  s.Pin = pins->CE_Pin;
  HAL_GPIO_Init(pins->CE_Port, &s);
  s.Pin = pins->SEN_Pin;
  HAL_GPIO_Init(pins->SEN_Port, &s);
  s.Pin = pins->SDI_Pin;
  HAL_GPIO_Init(pins->SDI_Port, &s);
  s.Pin = pins->SCK_Pin;
  HAL_GPIO_Init(pins->SCK_Port, &s);

  s.Mode = GPIO_MODE_INPUT;
  s.Pin = pins->LD_Pin;
  HAL_GPIO_Init(pins->LD_Port, &s);

  WRITE(CE, HIGH);
  WRITE(SEN, LOW);
  WRITE(SDI, LOW);
  WRITE(SCK, LOW);

  HAL_Delay(1);
}

void HMC833_Delay() {
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

void HMC833_WriteRegister(uint8_t addr, uint32_t data) {
  WRITE(SEN, HIGH);
  HMC833_Delay();

  // WR
  WRITE(SDI, LOW);
  HMC833_Delay();
  WRITE(SCK, HIGH);
  HMC833_Delay();
  WRITE(SCK, LOW);

  // Address
  for (int i = 5; i >= 0; i--) {
    WRITE(SDI, (addr >> i) & 1);
    HMC833_Delay();
    WRITE(SCK, HIGH);
    HMC833_Delay();
    WRITE(SCK, LOW);
  }

  // Data
  for (int i = 23; i >= 0; i--) {
    WRITE(SDI, (data >> i) & 1);
    HMC833_Delay();
    WRITE(SCK, HIGH);
    HMC833_Delay();
    WRITE(SCK, LOW);
  }

  // One more clock
  WRITE(SDI, LOW);
  HMC833_Delay();
  WRITE(SCK, HIGH);
  HMC833_Delay();
  WRITE(SEN, LOW);
  HMC833_Delay();
  WRITE(SCK, LOW);
}

uint32_t HMC833_ReadRegister(uint8_t addr) {
  WRITE(SEN, HIGH);
  HMC833_Delay();

  // RD
  WRITE(SDI, HIGH);
  HMC833_Delay();
  WRITE(SCK, HIGH);
  HMC833_Delay();
  WRITE(SCK, LOW);

  // Address
  for (int i = 5; i >= 0; i--) {
    WRITE(SDI, (addr >> i) & 1);
    HMC833_Delay();
    WRITE(SCK, HIGH);
    HMC833_Delay();
    WRITE(SCK, LOW);
  }

  HMC833_Delay();

  // Read data
  int32_t data = 0;
  for (int i = 23; i >= 0; i--) {
    WRITE(SCK, HIGH);
    HMC833_Delay();
    WRITE(SCK, LOW);
    data |= HAL_GPIO_ReadPin(pins->LD_Port, pins->LD_Pin) << i;
    HMC833_Delay();
  }

  // One more clock
  WRITE(SDI, LOW);
  HMC833_Delay();
  WRITE(SCK, HIGH);
  HMC833_Delay();
  WRITE(SEN, LOW);
  HMC833_Delay();

  WRITE(SCK, LOW);
  return data;
}

void HMC833_WriteVCORegister(uint8_t addr, uint16_t data) {
  HMC833_Reg_VCO vco;
  vco.raw = 0;
  vco.id = 0;
  vco.addr = addr;
  vco.data = data;
  HMC833_WriteRegister(HMC833_ADDR_VCO, vco.raw);
}

void HMC833_InitConfig(HMC833_Config *config) {
  // Magic numbers from datasheet
  config->refdiv = 1;
  config->sd_cfg.raw = 0x200B4A;
  config->charge_pump.raw = 0x403264;
  config->integer = 25;
  config->fractional = 0;
  config->vco.tuning.cal = 0;
  config->vco.tuning.caps = 16;
  config->vco.tuning.reserved_1 = 0;
  config->vco.enables.raw = 0x1F;
  config->vco.biases.rf_divide_ratio = 1;
  config->vco.biases.rf_output_buffer_gain_control = 3;
  config->vco.biases.divider_output_stage_gain_control = 0;
  config->vco.biases.reserved_1 = 0;
  config->vco.config.raw = 0;
  config->vco.config.fundamental_doubler_selection = 1;
  config->vco.config.rf_buffer_bias = 2;
}

void HMC833_SetFrequency(HMC833_Config *config, double fOUT) {
  // fVCO = fXTAL / R * (INT + FRAC / 2^24)
  // fOUT = fVCO / k
  // fXTAL = 50 MHz
  // R = [1, 2^14) - assume preconfigured
  // INT = [20, 524284]
  // FRAC = [0, 2^24)

  if (fOUT > 3e9) {
    config->vco.config.fundamental_doubler_selection = 0; // doubler enabled
  } else {
    config->vco.config.fundamental_doubler_selection = 1; // doubler disabled
  }

  config->vco.biases.rf_divide_ratio = 1;
  double k = (config->vco.config.fundamental_doubler_selection ? 1 : 0.5) *
             config->vco.biases.rf_divide_ratio;
  double N = fOUT * k * config->refdiv / HMC833_FXTAL;
  uint32_t integer = (uint32_t)N;
  uint32_t fractional = (uint32_t)((N - integer) * (1 << 24));
  if (integer < 20) {
    config->vco.biases.rf_divide_ratio = 0;
    while (integer < 20 && config->vco.biases.rf_divide_ratio < 62) {
      config->vco.biases.rf_divide_ratio += 2;
      k = (config->vco.config.fundamental_doubler_selection ? 1 : 0.5) *
          config->vco.biases.rf_divide_ratio;
      N = fOUT * k * config->refdiv / HMC833_FXTAL;
      integer = (uint32_t)N;
      fractional = (uint32_t)((N - integer) * (1 << 24));
    }
    if (integer < 20) {
      printf("Frequency too low.\n");
      return;
    }
  }
  if (integer > 524284) {
    printf("Frequency too high. Try adjust R.\n");
    return;
  }

  config->integer = integer;
  config->fractional = fractional;
  HMC833_WriteVCORegister(HMC833_VCO_ADDR_BIASES, config->vco.biases.raw);
  HMC833_WriteVCORegister(HMC833_VCO_ADDR_CONFIG, config->vco.config.raw);
  HMC833_WriteVCORegister(HMC833_VCO_ADDR_TUNING, 0);
  HMC833_WriteRegister(HMC833_ADDR_REFDIV, config->refdiv);
  HMC833_WriteRegister(HMC833_ADDR_INTEGER, config->integer);
  HMC833_WriteRegister(HMC833_ADDR_FRACTIONAL, config->fractional);

  // FIXME: Registers are written successfully, but the output frequency is wrong
}