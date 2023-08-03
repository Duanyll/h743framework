#include "board.h"
#include "ad7606c.h"
#include "lmx2572.h"
#include "main.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_gpio.h"

/* -------------------------------------------------------------------------- */
/*                                   AD7606                                   */
/* -------------------------------------------------------------------------- */

AD7606B_Config ad7606b_config;
AD7606B_Pins ad7606b_pins;

#ifdef BOARD_V2

static uint8_t reverse_bits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}
static uint16_t ad7606b_convert(uint16_t x) {
  return (reverse_bits(x >> 8) << 8) | reverse_bits(x & 0xff);
}

void BOARD_InitAD7606() {
  ad7606b_pins.CS_Port = GPIOE;
  ad7606b_pins.CS_Pin = GPIO_PIN_12;
  ad7606b_pins.RD_Port = GPIOE;
  ad7606b_pins.RD_Pin = GPIO_PIN_11;
  ad7606b_pins.BUSY_Port = GPIOC;
  ad7606b_pins.BUSY_Pin = GPIO_PIN_6;
  ad7606b_pins.WR_Port = GPIOB;
  ad7606b_pins.WR_Pin = GPIO_PIN_6;
  ad7606b_pins.RESET_Port = GPIOE;
  ad7606b_pins.RESET_Pin = GPIO_PIN_14;
  ad7606b_pins.CONVST_Port = GPIOB;
  ad7606b_pins.CONVST_Pin = GPIO_PIN_5;

  ad7606b_pins.DB_Port = GPIOD;
  ad7606b_pins.DataToPins = ad7606b_convert;
  ad7606b_pins.PinsToData = ad7606b_convert;

  ad7606b_pins.OS0_Port = GPIOE;
  ad7606b_pins.OS0_Pin = GPIO_PIN_10;
  ad7606b_pins.OS1_Port = GPIOE;
  ad7606b_pins.OS1_Pin = GPIO_PIN_9;
  ad7606b_pins.OS2_Port = GPIOE;
  ad7606b_pins.OS2_Pin = GPIO_PIN_8;
  ad7606b_pins.PAR_SEL_Port = GPIOE;
  ad7606b_pins.PAR_SEL_Pin = GPIO_PIN_13;

  ad7606b_pins.TIM_Handle = &htim5;

  AD7606B_Init(&ad7606b_pins);
  AD7606B_InitConfig(&ad7606b_config);

  AD7606B_SetRange(&ad7606b_config, 0, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606C_RANGE_PM2V5);
  AD7606B_SetOverSample(&ad7606b_config, 0, AD7606B_OVERSAMPLE_4);
  AD7606B_SetOverSample(&ad7606b_config, 1, AD7606B_OVERSAMPLE_4);

  int version = AD7606B_ParallelRegisterRead(AD7606B_REG_ID);
  // printf("AD7606B version: %d\n", version);

  AD7606B_LeaveRegisterMode();
}

#endif

#ifdef BOARD_V3

static uint16_t identity_u16(uint16_t x) { return x; }

void BOARD_InitAD7606() {
  ad7606b_pins.CS_Port = AD_CS_GPIO_Port;
  ad7606b_pins.CS_Pin = AD_CS_Pin;
  ad7606b_pins.RD_Port = AD_RD_GPIO_Port;
  ad7606b_pins.RD_Pin = AD_RD_Pin;
  ad7606b_pins.BUSY_Port = AD_BUSY_GPIO_Port;
  ad7606b_pins.BUSY_Pin = AD_BUSY_Pin;
  ad7606b_pins.WR_Port = AD_WR_GPIO_Port;
  ad7606b_pins.WR_Pin = AD_WR_Pin;
  ad7606b_pins.RESET_Port = AD_RESET_GPIO_Port;
  ad7606b_pins.RESET_Pin = AD_RESET_Pin;
  ad7606b_pins.CONVST_Port = AD_CONVST_GPIO_Port;
  ad7606b_pins.CONVST_Pin = AD_CONVST_Pin;

  ad7606b_pins.DB_Port = AD_D0_GPIO_Port;
  ad7606b_pins.DataToPins = identity_u16;
  ad7606b_pins.PinsToData = identity_u16;

  ad7606b_pins.OS0_Port = AD_OS0_GPIO_Port;
  ad7606b_pins.OS0_Pin = AD_OS0_Pin;
  ad7606b_pins.OS1_Port = AD_OS1_GPIO_Port;
  ad7606b_pins.OS1_Pin = AD_OS1_Pin;
  ad7606b_pins.OS2_Port = AD_OS2_GPIO_Port;
  ad7606b_pins.OS2_Pin = AD_OS2_Pin;
  ad7606b_pins.PAR_SEL_Port = AD_PAR_SEL_GPIO_Port;
  ad7606b_pins.PAR_SEL_Pin = AD_PAR_SEL_Pin;

  ad7606b_pins.TIM_Handle = &htim5;

  AD7606B_Init(&ad7606b_pins);
  AD7606B_InitConfig(&ad7606b_config);
  ad7606b_config.raw[AD7606C_REG_BANDWIDTH] = 0xff;
  AD7606B_ParallelRegisterWrite(AD7606C_REG_BANDWIDTH,
                                ad7606b_config.raw[AD7606C_REG_BANDWIDTH]);
  AD7606B_SetRange(&ad7606b_config, 0, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 2, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 3, AD7606C_RANGE_5V);
  // AD7606B_SetOverSample(&ad7606b_config, 0, AD7606B_OVERSAMPLE_4);
  // AD7606B_SetOverSample(&ad7606b_config, 1, AD7606B_OVERSAMPLE_4);

  /*
    int version = AD7606B_ParallelRegisterRead(AD7606B_REG_ID);
    printf("AD7606B version: %d\n", version);
    int range = AD7606B_ParallelRegisterRead(AD7606B_REG_RANGE);
    printf("AD7606B range 12: %d\n", range);
    range = AD7606B_ParallelRegisterRead(AD7606B_REG_RANGE + 1);
    printf("AD7606B range 34: %d\n", range);
    range = AD7606B_ParallelRegisterRead(AD7606B_REG_RANGE + 2);
    printf("AD7606B range 56: %d\n", range);
    range = AD7606B_ParallelRegisterRead(AD7606B_REG_RANGE + 3);
    printf("AD7606B range 78: %d\n", range);
  */
  AD7606B_LeaveRegisterMode();
}

#endif

/* -------------------------------------------------------------------------- */
/*                                   AD9959                                   */
/* -------------------------------------------------------------------------- */

AD9959_Pins ad9959_pins;
AD9959_GlobalConfig ad9959_config;
AD9959_ChannelConfig ad9959_channel0, ad9959_channel1;

void BOARD_InitAD9959() {
  ad9959_pins.SDIO0_Port = AD9959_SDIO0_GPIO_Port;
  ad9959_pins.SDIO0_Pin = AD9959_SDIO0_Pin;
  ad9959_pins.SCLK_Port = AD9959_SCLK_GPIO_Port;
  ad9959_pins.SCLK_Pin = AD9959_SCLK_Pin;
  ad9959_pins.CSB_Port = AD9959_CSB_GPIO_Port;
  ad9959_pins.CSB_Pin = AD9959_CSB_Pin;
  ad9959_pins.RST_Port = AD9959_RST_GPIO_Port;
  ad9959_pins.RST_Pin = AD9959_RST_Pin;
  ad9959_pins.IOUP_Port = AD9959_IOUP_GPIO_Port;
  ad9959_pins.IOUP_Pin = AD9959_IOUP_Pin;
  AD9959_InitGlobalConfig(&ad9959_config);
  AD9959_Init(&ad9959_pins, &ad9959_config);
  AD9959_InitChannelConfig(&ad9959_channel0);
  AD9959_InitChannelConfig(&ad9959_channel1);

  AD9959_SelectChannels(&ad9959_config, AD9959_CHANNEL_0);
  AD9959_SetFrequency(&ad9959_channel0, 100e3);
  AD9959_SetPhase(&ad9959_channel0, 0);
  AD9959_SetAmplitude(&ad9959_channel0, 0x3fff);
  AD9959_IOUpdate();

  AD9959_SelectChannels(&ad9959_config,
                        AD9959_CHANNEL_1 | AD9959_CHANNEL_2 | AD9959_CHANNEL_3);
  ad9959_channel1.cfr.dac_power_down = 1;
  AD9959_Write(AD9959_CFR_ADDR, ad9959_channel1.cfr.raw);
  AD9959_IOUpdate();
}

/* -------------------------------------------------------------------------- */
/*                                   SI5351                                   */
/* -------------------------------------------------------------------------- */

SWIIC_Config si5351_pins;
void BOARD_InitSI5351() {
  si5351_pins.SDA_Port = GPIOB;
  si5351_pins.SDA_Pin = GPIO_PIN_6;
  si5351_pins.SCL_Port = GPIOB;
  si5351_pins.SCL_Pin = GPIO_PIN_5;

  // si5351_pins.SDA_Port = GPIOC;
  // si5351_pins.SDA_Pin = GPIO_PIN_3;
  // si5351_pins.SCL_Port = GPIOC;
  // si5351_pins.SCL_Pin = GPIO_PIN_2;
  si5351_pins.delay = 100;
  SWIIC_Init(&si5351_pins);
  SI5351_Init(&si5351_pins, 0);
  HAL_Delay(10);
  SI5351_SetupCLK0(50e6, SI5351_DRIVE_STRENGTH_8MA);
  SI5351_SetupCLK2(50e6, SI5351_DRIVE_STRENGTH_8MA);
  SI5351_EnableOutputs(SI5351_CHANNEL_FLAG_CLK0);

  uint8_t data = 0;
  SWIIC_ReadBytes8(&si5351_pins, SI5351_ADDR, 0x03, &data, 1);
  printf("SI5351 status: %02x\n", data);
}

/* -------------------------------------------------------------------------- */
/*                                   LMX2572                                  */
/* -------------------------------------------------------------------------- */

LMX2572_Pins lmx2572_pins;
void BOARD_InitLMX2572() {
  lmx2572_pins.CS_Port = GPIOC;
  lmx2572_pins.CS_Pin = GPIO_PIN_15;
  lmx2572_pins.ENABLE_Port = NULL;
  lmx2572_pins.SCK_Port = GPIOC;
  lmx2572_pins.SCK_Pin = GPIO_PIN_14;
  lmx2572_pins.SDI_Port = GPIOC;
  lmx2572_pins.SDI_Pin = GPIO_PIN_13;
  lmx2572_pins.MUXOUT_Port = GPIOC;
  lmx2572_pins.MUXOUT_Pin = GPIO_PIN_9;
  LMX2572_Init(&lmx2572_pins, LMX2572_REFIN_50MHZ);
  LMX2572_SetFrequency(100e6);
}