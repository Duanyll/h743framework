#include <math.h>
#include <stdlib.h>

#include "app.h"

#include "ad7606b.h"
#include "ad9959.h"
#include "dac8830.h"
#include "si5351.h"
#include "ad9910.h"
#include "keys.h"
#include "nn.h"
#include "screen.h"
#include "signal.h"
#include "spi.h"
#include "usart.h"
#include "tim.h"
#include "dac.h"

UART_HandleTypeDef *computer_uart;

AD9959_GlobalConfig ad9959_global_config;
AD9959_ChannelConfig ad9959_channel0_config;
AD9959_ChannelConfig ad9959_channel1_config;
AD7606B_Config ad7606b_config;

void APP_InitAD9959() {
  AD9959_Init(&ad9959_global_config);
  double sysclk = 500e6;
  AD9959_InitChannelConfig(&ad9959_channel0_config);
  AD9959_InitChannelConfig(&ad9959_channel1_config);

  AD9959_SelectChannels(&ad9959_global_config, AD9959_CHANNEL_0);
  AD9959_SetFrequency(&ad9959_channel0_config, 100e6, sysclk);
  AD9959_SetPhase(&ad9959_channel0_config, 0);
  AD9959_SetAmplitude(&ad9959_channel0_config, 1023);

  AD9959_IOUpdate();

  AD9959_SelectChannels(&ad9959_global_config,
                        AD9959_CHANNEL_1 | AD9959_CHANNEL_2 | AD9959_CHANNEL_3);
  ad9959_channel1_config.cfr.dac_power_down = 1;
  ad9959_channel1_config.cfr.digital_power_down = 1;
  AD9959_Write(AD9959_CFR_ADDR, ad9959_channel1_config.cfr.raw);

  AD9959_IOUpdate();
}

void APP_InitAD7606B() {
  AD7606B_Init();
  AD7606B_InitConfig(&ad7606b_config);
  ad7606b_config.raw[AD7606C_REG_BANDWIDTH] = 0xff;
  AD7606B_ParallelRegisterWrite(AD7606C_REG_BANDWIDTH,
                                ad7606b_config.raw[AD7606C_REG_BANDWIDTH]);
  AD7606B_SetRange(&ad7606b_config, 0, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606C_RANGE_PM2V5);
  AD7606B_SetOverSample(&ad7606b_config, 0, AD7606B_OVERSAMPLE_4);
  AD7606B_SetOverSample(&ad7606b_config, 1, AD7606B_OVERSAMPLE_4);

  // int ad_version = AD7606B_ParallelRegisterRead(AD7606B_REG_ID);
  // UART_Printf(computer_uart, "AD7606B version: %d\n", ad_version);

  AD7606B_LeaveRegisterMode();
}

SWIIC_Config si5351_swiic;
void APP_InitSI5351() {
  si5351_swiic.delay = 100;
  si5351_swiic.SCL_Port = GPIOC;
  si5351_swiic.SCL_Pin = GPIO_PIN_13;
  si5351_swiic.SDA_Port = GPIOC;
  si5351_swiic.SDA_Pin = GPIO_PIN_14;
  SWIIC_Init(&si5351_swiic);
  SI5351_Init(&si5351_swiic, 0);
  SI5351_SetupCLK0((76 + 48) * 1000000, SI5351_DRIVE_STRENGTH_2MA);
  SI5351_SetupCLK2(100000000, SI5351_DRIVE_STRENGTH_8MA);
  SI5351_EnableOutputs(0x05);
}

AD9910_Pins ad9910_pins;
AD9910_Config ad9910_config;
void APP_InitAD9910() {
  // SDIO -> B12
  // IOUP -> B11
  // PF0 -> B10
  // PF1 -> B9
  // PF2 -> B8
  // RST -> B7
  // SCK -> B6
  // CSB -> B5
  ad9910_pins.SDIO_Port = GPIOB;
  ad9910_pins.SDIO_Pin = GPIO_PIN_12;
  ad9910_pins.IOUP_Port = GPIOB;
  ad9910_pins.IOUP_Pin = GPIO_PIN_11;
  ad9910_pins.PF0_Port = GPIOB;
  ad9910_pins.PF0_Pin = GPIO_PIN_10;
  ad9910_pins.PF1_Port = GPIOB;
  ad9910_pins.PF1_Pin = GPIO_PIN_9;
  ad9910_pins.PF2_Port = GPIOB;
  ad9910_pins.PF2_Pin = GPIO_PIN_8;
  ad9910_pins.RST_Port = GPIOB;
  ad9910_pins.RST_Pin = GPIO_PIN_7;
  ad9910_pins.SCK_Port = GPIOB;
  ad9910_pins.SCK_Pin = GPIO_PIN_6;
  ad9910_pins.CSB_Port = GPIOB;
  ad9910_pins.CSB_Pin = GPIO_PIN_5;
  UART_Printf(computer_uart, "------------\n");
  AD9910_Init(&ad9910_pins);
  AD9910_InitConfig(&ad9910_config);
  AD9910_SetupClock(&ad9910_config);
  AD9910_SetupSingleTone(&ad9910_config, 50e6, 0, AD9910_MAX_AMP);
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
  HAL_UART_Transmit(computer_uart, "\xff\xff\xff\xff", 4, 1000);
  HAL_UART_Transmit(computer_uart, (uint8_t *)ad_data,
                    sample_count * 2 * popcount(channels), 10000);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void APP_SetFrequency(uint32_t frequency) {
  AD9959_SelectChannels(&ad9959_global_config, AD9959_CHANNEL_0);
  AD9959_SetFrequency(&ad9959_channel0_config, frequency, 500e6);
  AD9959_IOUpdate();
}

uint16_t spi_buffer[2048];
uint16_t spi_output[1024];

void APP_DACOutCallback() {
  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  int16_t level = SPI_GetOutputAtTime();
  // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, level);
  // HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  DAC8830_SetVoltage(0x01, level * 4);
}

void APP_SpiTest() {
  TIM_SetTIM6Callback(APP_DACOutCallback);
  int dac_sample_rate = 100000;
  int period = (HAL_RCC_GetPCLK1Freq() * 2) / (TIM6->PSC + 1) / dac_sample_rate - 1;
  TIM6->ARR = period;
  SPI_StartDMARecieve(0x24000000, 2048, spi_output);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim6);
  for (int i = 1; i <= 100; i++) {
    SPI_RequestOutput();
    UART_Printf(computer_uart, "-------------\n");
    for (int j = 0; j < 1024; j++) {
      UART_Printf(computer_uart, "%d,\n", spi_output[j]);
    }
  }
  HAL_TIM_Base_Stop_IT(&htim6);
  SPI_StopDMARecieve();
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
    // 4 byte frequency
    uint32_t frequency = *(uint32_t *)(data + 1);
    APP_SetFrequency(frequency);
  } else if (*data == '\x03') {
    APP_SpiTest();
  }
}

void APP_Init() {
  computer_uart = &huart6;
  UART_ResetHexRX(computer_uart);
  // APP_InitAD9959();
  // APP_InitAD7606B();
  // APP_InitSI5351();
  APP_InitAD9910();
  DAC8830_Init();
}

void APP_Loop() { UART_PollHexData(APP_HexCommandCallback); }