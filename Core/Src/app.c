#include <math.h>
#include <stdlib.h>

#include "app.h"

#include "ad7606c.h"
#include "adf4351.h"
#include "ad9910.h"
#include "si5351.h"
#include "ad9959.h"
#include "keys.h"
#include "nn.h"
#include "pe43711.h"
#include "power.h"
#include "retarget.h"
#include "screen.h"
#include "serial.h"
#include "signal.h"
#include "spi.h"
#include "timers.h"

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

AD9959_Pins ad9959_pins;
AD9959_GlobalConfig ad9959_config;
AD9959_ChannelConfig ad9959_channel0, ad9959_channel1;

void APP_InitAD9959() {
  ad9959_pins.SDIO0_Port = GPIOB;
  ad9959_pins.SDIO0_Pin = GPIO_PIN_9;
  ad9959_pins.SCLK_Port = GPIOB;
  ad9959_pins.SCLK_Pin = GPIO_PIN_8;
  ad9959_pins.CSB_Port = GPIOB; 
  ad9959_pins.CSB_Pin = GPIO_PIN_7;
  ad9959_pins.RST_Port = GPIOA;
  ad9959_pins.RST_Pin = GPIO_PIN_8;
  ad9959_pins.IOUP_Port = GPIOA;
  ad9959_pins.IOUP_Pin = GPIO_PIN_9;
  AD9959_InitGlobalConfig(&ad9959_config);
  AD9959_Init(&ad9959_pins, &ad9959_config);
  AD9959_InitChannelConfig(&ad9959_channel0);
  AD9959_InitChannelConfig(&ad9959_channel1);

  AD9959_SelectChannels(&ad9959_config, AD9959_CHANNEL_0);
  AD9959_SetFrequency(&ad9959_channel0, 100e6);
  AD9959_SetPhase(&ad9959_channel0, 0);
  AD9959_SetAmplitude(&ad9959_channel0, 0x3fff);
  AD9959_IOUpdate(&ad9959_pins);

  AD9959_SelectChannels(&ad9959_config, AD9959_CHANNEL_1 | AD9959_CHANNEL_2 | AD9959_CHANNEL_3);
  ad9959_channel1.cfr.dac_power_down = 1;
  AD9959_Write(AD9959_CFR_ADDR, ad9959_channel1.cfr.raw);
  AD9959_IOUpdate(&ad9959_pins);
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
  UART_SendString(computer, "\xff\xff\xff\xff");
  HAL_UART_Transmit(computer, (uint8_t *)ad_data,
                    sample_count * 2 * popcount(channels), 10000);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void APP_HexCommandCallback(uint8_t *data, int len) {
  if (*data == 1) {
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
  } else if (*data == 2) {
    double freq = TIM_CountFrequencySync(&htim2, 100);
    printf("Freq: %fMHz\n", freq / 1000000.0);
  } else {
    // Echo back
    UART_SendHex(computer, data, len);
  }
}

KEYS_Pins keys_pins;
void APP_InitKeys() {
  keys_pins.keyCount = 4;
  keys_pins.pins[0].port = SWITCH1_GPIO_Port;
  keys_pins.pins[0].pin = SWITCH1_Pin;
  keys_pins.pins[0].callback = NULL;
  keys_pins.pins[1].port = SWITCH2_GPIO_Port;
  keys_pins.pins[1].pin = SWITCH2_Pin;
  keys_pins.pins[1].callback = NULL;
  keys_pins.pins[2].port = SWITCH3_GPIO_Port;
  keys_pins.pins[2].pin = SWITCH3_Pin;
  keys_pins.pins[2].callback = NULL;
  keys_pins.pins[3].port = SWITCH4_GPIO_Port;
  keys_pins.pins[3].pin = SWITCH4_Pin;
  keys_pins.pins[3].callback = NULL;
  keys_pins.htim = &htim7;

  KEYS_Init(&keys_pins);
}

void APP_Init() {
  POWER_Use400MHzClocks();
  computer = &huart6;
  RetargetInit(computer);
  // APP_InitAD7606B();

  APP_InitAD9959();
  // APP_InitSI5351();

  // KEYS_Start();
}

void APP_Loop() {
  // UART_PollCommands(1, APP_HexCommandCallback);
  AD7606B_CollectSamples(ad_data, 0x01, 1024, 20e3);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}