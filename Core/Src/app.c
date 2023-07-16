#include <math.h>

#include "app.h"

#include "ad7606b.h"
#include "ad9959.h"
#include "keys.h"
#include "screen.h"
#include "signal.h"
#include "usart.h"

#define APP_MODE_NO_SCAN 0
#define APP_MODE_AUTO_SCAN 1
#define APP_MODE_SCAN_UP 2
#define APP_MODE_SCAN_DOWN 3

double APP_curFreq = 50e6;
double APP_freqStep = 100e3;
int APP_mode = APP_MODE_NO_SCAN;
int APP_minFreq = 10e6;
int APP_maxFreq = 200e6;

BOOL APP_plotFlag = FALSE;

#define APP_LONGPRESS_TIME 500

AD9959_GlobalConfig ad9959_global_config;
AD9959_ChannelConfig ad9959_channel0_config;
AD9959_ChannelConfig ad9959_channel1_config;
double ad9959_sysclk = 500e6;

void APP_SetFreq(double freq) {
  AD9959_SelectChannels(&ad9959_global_config, AD9959_CHANNEL_0);
  AD9959_SetFrequency(&ad9959_channel0_config, freq, ad9959_sysclk);
  AD9959_IOUpdate();
}

void APP_IncreaseFreq() {
  APP_curFreq += APP_freqStep;
  if (APP_curFreq > APP_maxFreq) {
    APP_curFreq = APP_maxFreq;
    APP_mode = APP_MODE_NO_SCAN;
  }
  APP_SetFreq(APP_curFreq);
}

void APP_DecreaseFreq() {
  APP_curFreq -= APP_freqStep;
  if (APP_curFreq < APP_minFreq) {
    APP_curFreq = APP_minFreq;
    APP_mode = APP_MODE_NO_SCAN;
  }
  APP_SetFreq(APP_curFreq);
}

void APP_UpdateFreq() {
  switch (APP_mode) {
  case APP_MODE_NO_SCAN:
    break;
  case APP_MODE_AUTO_SCAN:
    if (APP_curFreq < APP_maxFreq) {
      APP_IncreaseFreq();
    } else {
      APP_curFreq = APP_minFreq;
      APP_SetFreq(APP_curFreq);
    }
    break;
  case APP_MODE_SCAN_UP:
    APP_IncreaseFreq();
    break;
  case APP_MODE_SCAN_DOWN:
    APP_DecreaseFreq();
    break;
  }
}

void APP_HandleKeys(uint8_t key, uint8_t state) {
  if (key == 0) {
    if (state == KEYS_STATE_PRESS) {
      APP_IncreaseFreq();
    } else if (state == KEYS_STATE_LONG_PRESS) {
      APP_mode = APP_MODE_SCAN_UP;
    } else if (state == KEYS_STATE_RELEASE) {
      APP_mode = APP_MODE_NO_SCAN;
    }
  }
  if (key == 1) {
    if (state == KEYS_STATE_PRESS) {
      APP_DecreaseFreq();
    } else if (state == KEYS_STATE_LONG_PRESS) {
      APP_mode = APP_MODE_SCAN_DOWN;
    } else if (state == KEYS_STATE_RELEASE) {
      APP_mode = APP_MODE_NO_SCAN;
    }
  }
  if (key == 2 && state == KEYS_STATE_PRESS) {
    if (APP_mode == APP_MODE_AUTO_SCAN) {
      APP_mode = APP_MODE_NO_SCAN;
    } else {
      APP_mode = APP_MODE_AUTO_SCAN;
      APP_curFreq = APP_minFreq;
    }
  }
  if (key == 3 && state == KEYS_STATE_PRESS) {
    APP_plotFlag = TRUE;
  }
  if (key == 3 && state == KEYS_STATE_LONG_PRESS) {
    APP_mode = APP_MODE_NO_SCAN;
    APP_curFreq = 50e6;
    APP_SetFreq(APP_curFreq);
  }
}

void APP_HandleUARTCommand(uint8_t* data, int len) {
  if (*data == '\x01') {
    APP_plotFlag = TRUE;
  }
}

AD7606B_Config ad7606b_config;

#define AD_SAMPLE_RATE 40e3
#define AD_SAMPLE_COUNT 1024

int16_t adc_buffer[AD_SAMPLE_COUNT * 2];
SIGNAL_TimeDataQ15 adc_time_data;
SIGNAL_SpectrumF32 adc_freq_data0, adc_freq_data1;

void APP_ADCSample() {
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  BOOL ok =
      AD7606B_CollectSamples(adc_buffer, 0x03, AD_SAMPLE_COUNT, AD_SAMPLE_RATE);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  if (!ok) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  }
}

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define APP_PLOT_HEIGHT 180
uint8_t APP_plotBuffer[AD_SAMPLE_COUNT];
void APP_Plot(float *data, float range, int count, int channel) {
  for (int i = 0; i < count; i++) {
    APP_plotBuffer[i] =
        CLAMP(roundf(data[i] / range * APP_PLOT_HEIGHT), 0, APP_PLOT_HEIGHT);
  }
  SCREEN_TransmitPlotData("s0", channel, APP_plotBuffer, count);
}

void APP_AnalysisADCData() {
  SCREEN_PrintText("fbase", "%.1lfMHz", APP_curFreq / 1e6);

  adc_time_data.points = AD_SAMPLE_COUNT;
  adc_time_data.timeDataOffset = 0;
  adc_time_data.timeDataStride = 2;
  adc_time_data.timeData = adc_buffer;
  adc_time_data.sampleRate = AD_SAMPLE_RATE;
  adc_time_data.range = 2.5;
  SIGNAL_TimeQ15ToSpectrumF32(&adc_time_data, &adc_freq_data0);

  SCREEN_PrintText("ch1f", "%.1lfkHz", adc_freq_data0.peakFreq / 1e3);
  SCREEN_PrintText("ch1a", "%.1lf", adc_freq_data0.peakAmp * 1e3);
  if (APP_plotFlag) {
    // APP_Plot(adc_freq_data0.ampData + 1, adc_freq_data0.peakAmp, 440, 0);
    UART_Printf(&huart1, "\xff\xff\xff\xff");
    HAL_UART_Transmit(&huart1, (uint8_t *)adc_freq_data0.ampData,
                      sizeof(float) * 512, 1000);
  }

  adc_time_data.timeDataOffset = 1;
  SIGNAL_TimeQ15ToSpectrumF32(&adc_time_data, &adc_freq_data1);

  SCREEN_PrintText("ch2f", "%.1lfkHz", adc_freq_data1.peakFreq / 1e3);
  SCREEN_PrintText("ch2a", "%.1lf", adc_freq_data1.peakAmp * 1e3);
  if (APP_plotFlag) {
    // APP_Plot(adc_freq_data1.ampData + 1, adc_freq_data1.peakAmp, 440, 1);
    UART_Printf(&huart1, "\xff\xff\xff\xff");
    HAL_UART_Transmit(&huart1, (uint8_t *)adc_freq_data1.ampData,
                      sizeof(float) * 512, 1000);
  }
  APP_plotFlag = FALSE;
}

void APP_InitAD9959() {
  AD9959_Init(&ad9959_global_config);
  double sysclk = 500e6;
  AD9959_InitChannelConfig(&ad9959_channel0_config);
  AD9959_InitChannelConfig(&ad9959_channel1_config);

  AD9959_SelectChannels(&ad9959_global_config, AD9959_CHANNEL_0);
  AD9959_SetFrequency(&ad9959_channel0_config, 50e6, sysclk);
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
  AD7606B_SetRange(&ad7606b_config, 0, AD7606B_RANGE_2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606B_RANGE_2V5);
  AD7606B_LeaveRegisterMode();
}

void APP_Init() {
  APP_InitAD9959();
  APP_InitAD7606B();
  SCREEN_Init(&huart2);
  APP_SetFreq(50e6);
  // KEYS_Init();
  // KEYS_SetHandler(APP_HandleKeys);
  UART_ResetHexRX(&huart1);
}

void APP_Loop() {
  // KEYS_Scan();
  UART_PollHexData(APP_HandleUARTCommand);
  APP_ADCSample();
  APP_UpdateFreq();
  APP_AnalysisADCData();
}