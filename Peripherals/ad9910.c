#include "ad9910.h"

#include <math.h>
#include <stdio.h>

static AD9910_Pins *pins;
#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET
#define WRITE(name, state)                                                     \
  HAL_GPIO_WritePin(pins->name##_Port, pins->name##_Pin, (state) ? HIGH : LOW)

void AD9910_Delay() {
  for (int i = 0; i < 10; i++) {
    __NOP();
  }
}

void AD9910_DelayLong() {
  for (int i = 0; i < 100; i++) {
    __NOP();
  }
}

void AD9910_Reset() {
  WRITE(RST, 0);
  AD9910_Delay();
  WRITE(RST, 1);
  AD9910_DelayLong();
  WRITE(RST, 0);
  HAL_Delay(5);
}

void AD9910_Init(AD9910_Pins *p) {
  pins = p;
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
#define INIT(name)                                                             \
  s.Pin = pins->name##_Pin;                                                    \
  HAL_GPIO_Init(pins->name##_Port, &s);
  INIT(IOUP);
  INIT(PF0);
  INIT(PF1);
  INIT(PF2);
  INIT(RST);
  INIT(SCK);
  INIT(CSB);
#ifdef AD9910_USE_OPEN_DRAIN
  s.Mode = GPIO_MODE_OUTPUT_OD;
#endif
  INIT(SDIO);
#undef INIT

  WRITE(SDIO, 0);
  WRITE(IOUP, 0);
  WRITE(PF0, 0);
  WRITE(PF1, 0);
  WRITE(PF2, 0);
  WRITE(RST, 0);
  WRITE(SCK, 1);
  WRITE(CSB, 1);

  AD9910_Reset();
}

void AD9910_InitConfig(AD9910_Config *cfg) {
  cfg->cfr1.raw = 0x00000000;
  cfg->cfr2.raw = 0x00400820;
  cfg->cfr3.raw = 0x1f3f4000;
  cfg->aux_dac_control = 0;
  cfg->io_update_rate = 0x7fffffff;
  cfg->ftw = 0;
  cfg->pow = 0;
  cfg->asf.raw = 0;
  cfg->multichip_sync.raw = 0;
  cfg->digital_ramp_limit.raw = 0;
  cfg->digital_ramp_step.raw = 0;
  cfg->digital_ramp_rate.raw = 0;
  for (int i = 0; i < 8; i++) {
    cfg->single_tone_profile[i].raw = 0x08b5000000000000ull;
    cfg->ram_profile[i].raw = 0;
  }
}

void AD9910_SetupClock(AD9910_Config *cfg) {
  cfg->cfr1.inverse_sinc_filter_enable = 1;
  AD9910_WriteRegister(AD9910_ADDR_CFR1, cfg->cfr1.raw);
  // Manual delay before clock setup
  WRITE(IOUP, 1);
  HAL_Delay(1);
  WRITE(IOUP, 0);

  cfg->cfr2.sync_clk_enable = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR2, cfg->cfr2.raw);

  cfg->cfr3.pll_enable = 1;
  cfg->cfr3.n = 25;
  // Accroding to the datasheet, the following settings should be used to
  // disable input divider
  // cfg->cfr3.refclk_input_divider_bypass = 1;
  // cfg->cfr3.refclk_input_divider_reset_b = 0;
  // However, the above settings do not work, causing the register CFR3 fails
  // to write.
  cfg->cfr3.vco_sel = AD9910_vco_range_setting_vco5;
  cfg->cfr3.icp = AD9910_pump_current_237;
  // printf("CFR3 Expected: %d\n", (int)cfg->cfr3.raw);
  AD9910_WriteRegister(AD9910_ADDR_CFR3, cfg->cfr3.raw);
  WRITE(IOUP, 1);
  HAL_Delay(1);
  WRITE(IOUP, 0);
  HAL_Delay(1);
  // printf("CFR3 Actual: %d\n", (int)AD9910_ReadRegister(AD9910_ADDR_CFR3));
}

void AD9910_MakeSDIOOutput() {
#ifndef AD9910_USE_OPEN_DRAIN
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  s.Pin = pins->SDIO_Pin;
  HAL_GPIO_Init(pins->SDIO_Port, &s);
#endif
}

void AD9910_MakeSDIOInput() {
#ifndef AD9910_USE_OPEN_DRAIN
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_INPUT,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  s.Pin = pins->SDIO_Pin;
  HAL_GPIO_Init(pins->SDIO_Port, &s);
#else
  WRITE(SDIO, 1);
#endif
}

void AD9910_WriteByte(uint8_t data) {
  AD9910_MakeSDIOOutput();
  AD9910_Delay();
  for (int i = 7; i >= 0; i--) {
    WRITE(SDIO, ((data >> i) & 1));
    WRITE(SCK, 0);
    AD9910_Delay();
    WRITE(SCK, 1);
    AD9910_Delay();
  }
}

uint8_t AD9910_ReadByte() {
  uint8_t data = 0;
  AD9910_MakeSDIOInput();
  AD9910_Delay();
  for (int i = 7; i >= 0; i--) {
    WRITE(SCK, 0);
    AD9910_Delay();
    data |= (HAL_GPIO_ReadPin(pins->SDIO_Port, pins->SDIO_Pin) << i);
    WRITE(SCK, 1);
    AD9910_Delay();
  }
  return data;
}

const uint8_t AD9910_RegisterSize[] = {4, 4, 4, 4, 4, 0, 0, 4, 2, 4, 4,
                                       8, 8, 4, 8, 8, 8, 8, 8, 8, 8, 8};

void AD9910_WriteRegister(uint8_t reg, uint64_t data) {
  printf("Write %d: %llu\n", reg, data);
  WRITE(CSB, 0);
  AD9910_Delay();
  AD9910_WriteByte(reg);
  AD9910_Delay();
  for (int i = AD9910_RegisterSize[reg] - 1; i >= 0; i--) {
    AD9910_WriteByte((data >> (i * 8)) & 0xff);
    AD9910_Delay();
  }
  // AD9910_IOUpdate();
  WRITE(CSB, 1);
}

uint64_t AD9910_ReadRegister(uint8_t reg) {
  WRITE(CSB, 0);
  AD9910_Delay();
  AD9910_WriteByte(reg | 0x80);
  AD9910_Delay();
  uint64_t data = 0;
  for (int i = AD9910_RegisterSize[reg] - 1; i >= 0; i--) {
    data |= (uint64_t)AD9910_ReadByte() << (i * 8);
    AD9910_Delay();
  }
  // AD9910_IOUpdate();
  WRITE(CSB, 1);
  return data;
}

void AD9910_WriteRam(uint32_t *data, int count) {
  WRITE(CSB, 0);
  AD9910_Delay();
  AD9910_WriteByte(AD9910_ADDR_RAM);
  AD9910_Delay();
  for (int i = 0; i < count; i++) {
    AD9910_WriteByte((data[i] >> 24) & 0xff);
    AD9910_Delay();
    AD9910_WriteByte((data[i] >> 16) & 0xff);
    AD9910_Delay();
    AD9910_WriteByte((data[i] >> 8) & 0xff);
    AD9910_Delay();
    AD9910_WriteByte((data[i] >> 0) & 0xff);
    AD9910_Delay();
  }
  // AD9910_IOUpdate();
  WRITE(CSB, 1);
}

void AD9910_ReadRam(uint32_t *data, int count) {
  WRITE(CSB, 0);
  AD9910_Delay();
  AD9910_WriteByte(AD9910_ADDR_RAM | 0x80);
  AD9910_Delay();
  for (int i = 0; i < count; i++) {
    data[i] = 0;
    data[i] |= (uint32_t)AD9910_ReadByte() << 24;
    AD9910_Delay();
    data[i] |= (uint32_t)AD9910_ReadByte() << 16;
    AD9910_Delay();
    data[i] |= (uint32_t)AD9910_ReadByte() << 8;
    AD9910_Delay();
    data[i] |= (uint32_t)AD9910_ReadByte() << 0;
    AD9910_Delay();
  }
  // AD9910_IOUpdate();
  WRITE(CSB, 1);
}

void AD9910_IOUpdate() {
  WRITE(IOUP, 1);
  AD9910_DelayLong();
  WRITE(IOUP, 0);
  AD9910_Delay();
}

void AD9910_SelectProfile(uint8_t profile) {
  WRITE(PF0, profile & 1);
  WRITE(PF1, profile & 2);
  WRITE(PF2, profile & 4);
  AD9910_Delay();
}

uint32_t AD9910_CalculateFTW(double freq) {
  return (uint32_t)round(freq / AD9910_SYSCLK * (1ull << 32));
}

uint16_t AD9910_CalculatePOW(double phase) {
  return (uint16_t)round(phase / 360 * (1ull << 16));
}

void AD9910_SetupSingleTone(AD9910_Config *cfg, double freq, double phase,
                            uint16_t amp) {
  cfg->cfr1.ram_enable = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR1, cfg->cfr1.raw);
  cfg->cfr2.digital_ramp_enable = 0;
  cfg->cfr2.parallel_data_port_enable = 0;
  cfg->cfr2.enable_amp_scal_from_singal_tone_profile = 1;
  AD9910_WriteRegister(AD9910_ADDR_CFR2, cfg->cfr2.raw);
  AD9910_SelectProfile(0);
  cfg->single_tone_profile[0].ftw = AD9910_CalculateFTW(freq);
  cfg->single_tone_profile[0].pow = AD9910_CalculatePOW(phase);
  cfg->single_tone_profile[0].asf = amp;
  AD9910_WriteRegister(AD9910_ADDR_PROFILE, cfg->single_tone_profile[0].raw);
  AD9910_IOUpdate();
}

void AD9910_SetupParallel(AD9910_Config *cfg, double freq, double phase,
                          uint16_t amp) {
  cfg->cfr1.ram_enable = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR1, cfg->cfr1.raw);
  cfg->cfr2.digital_ramp_enable = 0;
  cfg->cfr2.parallel_data_port_enable = 1;
  AD9910_WriteRegister(AD9910_ADDR_CFR2, cfg->cfr2.raw);
  cfg->ftw = AD9910_CalculateFTW(freq);
  AD9910_WriteRegister(AD9910_ADDR_FTW, cfg->ftw);
  cfg->pow = AD9910_CalculatePOW(phase);
  AD9910_WriteRegister(AD9910_ADDR_POW, cfg->pow);
  cfg->asf.amp_scale_factor = amp;
  AD9910_WriteRegister(AD9910_ADDR_ASF, cfg->asf.raw);
  AD9910_IOUpdate();
}

uint32_t AD9910_ramBuffer[AD9910_RAM_SIZE];

static int AD9910_SetPlaybackRate(AD9910_Config *cfg, double modFreq) {
  // Calculate data length (L, 10-bit) and playback rate (M, 16-bit)
  // Playback sample rate R = fSYSCLK / (4 * M)
  // R / L = modFreq
  // => M * L = fSYSCLK / (4 * modFreq)
  double ML = AD9910_SYSCLK / (4 * modFreq);
  // We want to maximize L, and minimize error
  // Let L be at least 64, and search for the best M
  int L = 64;
  double error = 1e9;
  for (int l = 64; l < 1024; l++) {
    int m = round(ML / l);
    double e = fabs(ML / l - m);
    if (e <= error + 1e-6) {
      error = e;
      L = l;
    }
  }
  int M = round(ML / L);
  cfg->ram_profile[0].address_step_rate = M;
  return L;
}

void AD9910_SetupSingleToneAM(AD9910_Config *cfg, double baseFreq,
                              double modFreq, double modDepth,
                              uint16_t fullAmp) {
  cfg->cfr1.ram_enable = 1;
  cfg->cfr1.ram_playback_dest = AD9910_ram_dest_amplitude;
  cfg->cfr1.internal_profile_control = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR1, cfg->cfr1.raw);

  cfg->cfr2.digital_ramp_enable = 0;
  cfg->cfr2.parallel_data_port_enable = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR2, cfg->cfr2.raw);

  // Calculate the FTW and POW for the base frequency
  cfg->ftw = AD9910_CalculateFTW(baseFreq);
  AD9910_WriteRegister(AD9910_ADDR_FTW, cfg->ftw);
  cfg->pow = AD9910_CalculatePOW(0);
  AD9910_WriteRegister(AD9910_ADDR_POW, cfg->pow);
  cfg->asf.amp_scale_factor = fullAmp;
  AD9910_WriteRegister(AD9910_ADDR_ASF, cfg->asf.raw);

  AD9910_SelectProfile(0);
  cfg->ram_profile[0].mode_control = AD9910_ram_ctl_cont_recirculate;
  int L = AD9910_SetPlaybackRate(cfg, modFreq);
  cfg->ram_profile[0].start_address = 0;
  cfg->ram_profile[0].end_address = L;
  AD9910_WriteRegister(AD9910_ADDR_PROFILE, cfg->ram_profile[0].raw);
  AD9910_IOUpdate();

  // Generate the RAM data
  for (int i = 0; i < L; i++) {
    double phase = i * 2 * M_PI / L;
    double amp =
        fullAmp * (1 - modDepth / 2) + fullAmp * modDepth / 2 * cos(phase);
    AD9910_ramBuffer[i] = (uint32_t)CLAMP(round(amp), 0, AD9910_MAX_AMP) << 18;
  }
  AD9910_WriteRam(AD9910_ramBuffer, L);
  AD9910_IOUpdate();
}

void AD9910_SetupSingleToneFM(AD9910_Config *cfg, double baseFreq,
                              double modFreq, double freqDev, uint16_t amp) {
  cfg->cfr1.ram_enable = 1;
  cfg->cfr1.ram_playback_dest = AD9910_ram_dest_frequency;
  cfg->cfr1.internal_profile_control = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR1, cfg->cfr1.raw);

  cfg->cfr2.digital_ramp_enable = 0;
  cfg->cfr2.parallel_data_port_enable = 0;
  AD9910_WriteRegister(AD9910_ADDR_CFR2, cfg->cfr2.raw);

  // Calculate the FTW and POW for the base frequency
  cfg->ftw = AD9910_CalculateFTW(baseFreq);
  AD9910_WriteRegister(AD9910_ADDR_FTW, cfg->ftw);
  cfg->pow = AD9910_CalculatePOW(0);
  AD9910_WriteRegister(AD9910_ADDR_POW, cfg->pow);
  cfg->asf.amp_scale_factor = amp;
  AD9910_WriteRegister(AD9910_ADDR_ASF, cfg->asf.raw);

  AD9910_SelectProfile(0);
  cfg->ram_profile[0].mode_control = AD9910_ram_ctl_cont_recirculate;
  int L = AD9910_SetPlaybackRate(cfg, modFreq);
  cfg->ram_profile[0].start_address = 0;
  cfg->ram_profile[0].end_address = L;
  AD9910_WriteRegister(AD9910_ADDR_PROFILE, cfg->ram_profile[0].raw);
  AD9910_IOUpdate();

  // Generate the RAM data
  for (int i = 0; i < L; i++) {
    double phase = i * 2 * M_PI / L;
    double freq = baseFreq + freqDev * cos(phase);
    AD9910_ramBuffer[i] = AD9910_CalculateFTW(freq);
  }
  AD9910_WriteRam(AD9910_ramBuffer, L);
  AD9910_IOUpdate();
}