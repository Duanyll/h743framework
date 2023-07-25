#include <math.h>
#include <stdlib.h>

#include "app.h"

#include "ad7606b.h"
#include "hmc833.h"
#include "keys.h"
#include "nn.h"
#include "screen.h"
#include "signal.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "dac.h"
#include "retarget.h"

UART_HandleTypeDef *computer;

AD7606B_Config ad7606b_config;
AD7606B_Pins ad7606b_pins;

uint16_t identity_u16(uint16_t x) { return x; }

void APP_InitAD7606B() {
  ad7606b_pins.CS_Port = AD_CS_GPIO_Port;
  ad7606b_pins.CS_Pin = AD_CS_Pin;
  ad7606b_pins.RD_Port = AD_RD_GPIO_Port;
  ad7606b_pins.RD_Pin = AD_RD_Pin;
  ad7606b_pins.BUSY_Port = AD_BUSY_GPIO_Port;
  ad7606b_pins.BUSY_Pin = AD_BUSY_Pin;
  ad7606b_pins.WR_Port = AD_WR_GPIO_Port;
  ad7606b_pins.WR_Pin = AD_WR_Pin;

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

  ad7606b_pins.TIM_Handle = &htim2;

  AD7606B_Init(&ad7606b_pins);
  AD7606B_InitConfig(&ad7606b_config);
  ad7606b_config.raw[AD7606C_REG_BANDWIDTH] = 0xff;
  AD7606B_ParallelRegisterWrite(AD7606C_REG_BANDWIDTH,
                                ad7606b_config.raw[AD7606C_REG_BANDWIDTH]);
  AD7606B_SetRange(&ad7606b_config, 0, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606C_RANGE_PM2V5);
  AD7606B_SetOverSample(&ad7606b_config, 0, AD7606B_OVERSAMPLE_4);
  AD7606B_SetOverSample(&ad7606b_config, 1, AD7606B_OVERSAMPLE_4);

  AD7606B_LeaveRegisterMode();
}

HMC833_Pins hmc833_pins;
HMC833_Config hmc833_config;

void APP_InitHMC833() {
  hmc833_pins.LD_Port = GPIOB;
  hmc833_pins.LD_Pin = GPIO_PIN_5;
  hmc833_pins.SCK_Port = GPIOB;
  hmc833_pins.SCK_Pin = GPIO_PIN_6;
  hmc833_pins.SDI_Port = GPIOB;
  hmc833_pins.SDI_Pin = GPIO_PIN_7;
  hmc833_pins.SEN_Port = GPIOB;
  hmc833_pins.SEN_Pin = GPIO_PIN_8;
  hmc833_pins.CE_Port = GPIOB;
  hmc833_pins.CE_Pin = GPIO_PIN_9;

  HMC833_Init(&hmc833_pins);
  HMC833_InitConfig(&hmc833_config);
  HMC833_SetFrequency(&hmc833_config, 100e6);
}

#define AD_SAMPLE_COUNT 2048
int16_t ad_data[AD_SAMPLE_COUNT * 4];

int popcount(uint8_t x) {
  int count = 0;
  while (x) {
    count += x & 1;
    x >>= 1;
  }
  return count;
}

void APP_UploadADCData(uint8_t channels, uint32_t sample_count,
                       uint32_t sample_rate) {
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  AD7606B_CollectSamples(ad_data, channels, sample_count, sample_rate);
  HAL_UART_Transmit(computer, "\xff\xff\xff\xff", 4, 1000);
  HAL_UART_Transmit(computer, (uint8_t *)ad_data,
                    sample_count * 2 * popcount(channels), 10000);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void APP_HexCommandCallback(uint8_t *data, int len) {
  if (*data == '\x01') {
    // 1 byte channels
    // 4 byte sample count
    // 4 byte sample rate
    uint8_t channels = 0x0f;
    uint32_t sample_count = 128;
    uint32_t sample_rate = 100000;
    if (len >= 2)
      channels = data[1];
    if (len >= 6)
      sample_count = *(uint32_t *)(data + 2);
    if (len >= 10)
      sample_rate = *(uint32_t *)(data + 6);
    APP_UploadADCData(channels, sample_count, sample_rate);
  } else if (*data == '\x02') {
    double freq = TIM_CountFrequencySync(&htim2, 100);
    printf("Freq: %fMHz\n", freq / 1000000.0);
  }
}

void APP_Init() {
  computer = &huart6;
  RetargetInit(computer);
  UART_ResetHexRX(computer);
  APP_InitAD7606B();
  APP_InitHMC833();
}

void APP_Loop() { UART_PollHexData(APP_HexCommandCallback); }