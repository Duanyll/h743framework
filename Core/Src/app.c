#include <math.h>
#include <stdlib.h>

#include "app.h"

#include "board.h"
#include "nn.h"
#include "power.h"
#include "retarget.h"
#include "serial.h"
#include "signal.h"
#include "timers.h"

UART_HandleTypeDef *computer;

#define AD_SAMPLE_COUNT 4096
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
  LED_On(2);
  AD7606B_CollectSamples(ad_data, channels, sample_count, sample_rate);
  UART_SendString(computer, "\xff\xff\xff\xff");
  HAL_UART_Transmit(computer, (uint8_t *)ad_data,
                    sample_count * 2 * popcount(channels), 10000);
  LED_Off(2);
}

float fft_buffer[AD_SAMPLE_COUNT * 2];
float fft_mag[AD_SAMPLE_COUNT];

void APP_LowPowerScanSystem() {
  int startTick = HAL_GetTick();
  int delay = 1000;
  LED_On(1);
  double rough_freq = 0;
  for (int freq = 200e6; freq >= 10e6; freq -= 0.25e6) {
    SI5351_SetupCLK0(freq, SI5351_DRIVE_STRENGTH_8MA);
    SI5351_SetupCLK2(freq, SI5351_DRIVE_STRENGTH_8MA);
    TIM_DelayUs(delay);
    AD7606B_CollectSamples(ad_data, AD7606B_CHANNEL_FLAG_CH3, 8, 100e3);
    double avg = 0;
    for (int i = 0; i < 8; i++) {
      avg += ad_data[i];
    }
    avg = avg * 2.5 / 32768 / 8;
    if (avg > 0.3) {
      printf("Detected Voltage %lfV\n", avg);
      rough_freq = freq;
      break;
    }
  }
  if (rough_freq == 0) {
    printf("No signal\n");
    LED_Off(1);
    return;
  }
  printf("Rough freq: %lfMHz\n", rough_freq / 1e6);
  LED_Off(1);

  LED_On(2);
  double center = 0;
  double sum = 0;
  for (int freq = rough_freq - 0.5e6; freq <= rough_freq + 0.5e6;
       freq += 10e3) {
    SI5351_SetupCLK0(freq, SI5351_DRIVE_STRENGTH_8MA);
    SI5351_SetupCLK2(freq, SI5351_DRIVE_STRENGTH_8MA);
    TIM_DelayUs(delay);
    AD7606B_CollectSamples(ad_data, AD7606B_CHANNEL_FLAG_CH3, 8, 100e3);
    double avg = 0;
    for (int i = 0; i < 8; i++) {
      avg += ad_data[i];
    }
    avg = avg * 2.5 / 32768 / 8;
    if (avg < 0.6)
      continue;
    sum += avg;
    center += avg * freq;
  }
  center /= sum;
  // center += 20e3;
  printf("Center freq: %lfMHz\n", center / 1e6);
  printf("Carrier Frequency: %lfMhz\n", center / 1e6 - 10.7);
  LED_Off(2);

  LED_On(3);
  SI5351_SetupCLK0(center, SI5351_DRIVE_STRENGTH_8MA);
  SI5351_SetupCLK2(center, SI5351_DRIVE_STRENGTH_8MA);
  TIM_DelayUs(delay);
  int points = 4096;
  int sampleRate = 40960;
  AD7606B_CollectSamples(ad_data,
                         AD7606B_CHANNEL_FLAG_CH1 | AD7606B_CHANNEL_FLAG_CH2,
                         points, sampleRate);
  SIGNAL_FFTBufferF32 buf = {.fftBuffer = fft_buffer, .magBuffer = fft_mag};
  SIGNAL_TimeDataQ15 amData = {.timeData = ad_data,
                               .offset = 0,
                               .stride = 2,
                               .points = points,
                               .range = 2.5,
                               .sampleRate = sampleRate};
  SIGNAL_SpectrumF32 amSpectrum;
  SIGNAL_TimeQ15ToSpectrumF32(&amData, &amSpectrum, &buf);
  double amSnr = SIGNAL_SimpleSNR(&amSpectrum);
  SIGNAL_TimeDataQ15 fmData = {.timeData = ad_data,
                               .offset = 1,
                               .stride = 2,
                               .points = points,
                               .range = 2.5,
                               .sampleRate = sampleRate};
  SIGNAL_SpectrumF32 fmSpectrum;
  SIGNAL_TimeQ15ToSpectrumF32(&fmData, &fmSpectrum, &buf);
  double fmSnr = SIGNAL_SimpleSNR(&fmSpectrum);
  printf("AM Amp: %lf\n", amSpectrum.peakAmp);
  printf("FM Amp: %lf\n", fmSpectrum.peakAmp);
  if (amSpectrum.peakAmp < 50 && fmSpectrum.peakAmp < 50) {
    printf("No signal\n");
  } else if (amSnr > fmSnr) {
    printf("AM: %lfkHz\n", amSpectrum.peakFreq / 1e3);
  } else {
    printf("FM: %lfkHz\n", fmSpectrum.peakFreq / 1e3);
  }
  LED_Off(3);
  int endTick = HAL_GetTick();
  printf("Time: %dms\n", endTick - startTick);
}

UART_RxBuffer com_buf;
UART_HandleTypeDef *computer;

void APP_PollUartCommands() {
  char data[16];
  int readCount = 0;
  readCount = UART_Read(&com_buf, data, 1, 1);
  if (readCount == 0)
    return;
  if (*data == 1) {
    uint8_t channels = 0x0f;
    uint32_t sample_count = 128;
    uint32_t sample_rate = 100000;
    readCount = UART_Read(&com_buf, data + 1, 9, 1000);
    if (readCount != 9)
      return;
    channels = data[1];
    sample_count = *(uint32_t *)(data + 2);
    sample_rate = *(uint32_t *)(data + 6);
    APP_UploadADCData(channels, sample_count, sample_rate);
  } else if (*data == 2) {
    double freq = TIM_CountFrequencySync(&htim2, 100);
    printf("Freq: %fMHz\n", freq / 1000000.0);
  } else if (*data == 3) {
    readCount = UART_Read(&com_buf, data + 1, 9, 1000);
    if (readCount != 9)
      return;
    int freq0 = *(int *)(data + 1);
    int freq2 = *(int *)(data + 5);
    uint8_t power = *(uint8_t *)(data + 9);
    SI5351_SetupCLK0(freq0, power);
    SI5351_SetupCLK2(freq2, power);
  } else if (*data == 4) {
    APP_LowPowerScanSystem();
  }
}

void APP_Key1Callback(uint8_t event) {
  if (event == KEYS_EVENT_PRESS) {
    APP_LowPowerScanSystem();
  }
}

KEYS_Pins keys_pins;
void APP_InitKeys() {
  keys_pins.keyCount = 4;
  keys_pins.pins[0].port = SWITCH2_GPIO_Port;
  keys_pins.pins[0].pin = SWITCH2_Pin;
  keys_pins.pins[0].callback = NULL;
  keys_pins.pins[1].port = SWITCH1_GPIO_Port;
  keys_pins.pins[1].pin = SWITCH1_Pin;
  keys_pins.pins[1].callback = NULL;
  keys_pins.pins[2].port = SWITCH3_GPIO_Port;
  keys_pins.pins[2].pin = SWITCH3_Pin;
  keys_pins.pins[2].callback = NULL;
  keys_pins.pins[3].port = SWITCH4_GPIO_Port;
  keys_pins.pins[3].pin = SWITCH4_Pin;
  keys_pins.pins[3].callback = NULL;
  keys_pins.htim = &htim7;

  KEYS_Init(&keys_pins);
  KEYS_Start(); 
}

void APP_Init() {
  POWER_Use400MHzClocks();
  computer = &huart1;
  RetargetInit(computer);
  BOARD_InitAD7606();
  BOARD_InitSI5351();
  UART_RxBuffer_Init(&com_buf, computer);
  UART_Open(&com_buf);
  APP_InitKeys();
}

void APP_Loop() { APP_PollUartCommands(); }