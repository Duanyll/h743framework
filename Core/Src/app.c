#include <math.h>
#include <stdlib.h>

#include "app.h"

#include "ad7606b.h"
#include "ad9959.h"
#include "dac8830.h"
#include "keys.h"
#include "screen.h"
#include "signal.h"
#include "usart.h"
#include "nn.h"

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

float nn_input[NN_Signet_Input_Samples * NN_Signet_Input_Channels];
uint8_t nn_buffer[4096];

void APP_NNBenchmark() {
  for (int i = 0; i < NN_Signet_Input_Samples * NN_Signet_Input_Channels; i++) {
    nn_input[i] = sinf(i * 2 * 3.14159265 / 1000);
  }
  NN_SignetInferenceInstance instance;
  NN_SignetInstance_init(&instance, nn_buffer);
  float nn_output[NN_Signet_Output_Classes];
  uint32_t start = HAL_GetTick();
  for (int i = 0; i < 1000; i++) {
    NN_Signet_Evaluate(&instance, nn_input, nn_output);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }
  uint32_t end = HAL_GetTick();
  UART_Printf(computer_uart, "NN benchmark: %d ms\n", end - start);
}

void APP_HexCommandCallback(uint8_t *data, int len) {
  if (*data == '\x01') {
    // 1 byte channels
    // 4 byte sample count
    // 4 byte sample rate
    uint8_t channels = 0x0f;
    uint32_t sample_count = 128;
    uint32_t sample_rate = 100000;
    if (len >= 2) channels = data[1];
    if (len >= 6) sample_count = *(uint32_t *)(data + 2);
    if (len >= 10) sample_rate = *(uint32_t *)(data + 6);
    APP_UploadADCData(channels, sample_count, sample_rate);
  } else if (*data == '\x02') {
    // 4 byte frequency
    uint32_t frequency = *(uint32_t *)(data + 1);
    APP_SetFrequency(frequency);
  } else if (*data == '\x03') {
    APP_NNBenchmark();
  }
}

void APP_Init() {
  computer_uart = &huart6;
  UART_ResetHexRX(computer_uart);
  APP_InitAD9959();
  APP_InitAD7606B();
  DAC8830_Init();
}

void APP_Loop() { UART_PollHexData(APP_HexCommandCallback); }