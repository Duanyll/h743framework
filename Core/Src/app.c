#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ad7606b.h"
#include "app.h"

#include "board.h"
#include "lmx2572.h"
#include "nn.h"
#include "power.h"
#include "retarget.h"
#include "serial.h"
#include "signal.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "timers.h"
#include "ui.h"

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

void APP_UploadADCDataFast(uint8_t channels, uint32_t sample_count) {
  LED_On(2);
  double sample_rate =
      AD7606B_FastCollectSamples(ad_data, channels, sample_count);
  UART_SendString(computer, "\xff\xff\xff\xff");
  UART_SendHex(computer, (uint8_t *)&sample_rate, 8);
  UART_SendHex(computer, (uint8_t *)ad_data,
               sample_count * 2 * popcount(channels));
  LED_Off(2);
}

void APP_InitRFSwitch() {
  // Init C0, C1 as output
  GPIO_InitTypeDef s = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  s.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOC, &s);
  s.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOC, &s);
}

#define RFSWITCH_DETECTOR 0    // 00
#define RFSWITCH_DEMODULATOR 1 // 11

void APP_SetRFSwitch(int mode) {
  if (mode == RFSWITCH_DETECTOR) {
    printf("RFSWITCH_DETECTOR\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
  } else if (mode == RFSWITCH_DEMODULATOR) {
    printf("RFSWITCH_DEMODULATOR\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
  }
  TIM_DelayUs(50);
}

double APP_PowerDetectorScan(double start, double end, double step,
                             double threshold) {
  APP_SetRFSwitch(RFSWITCH_DETECTOR);
  printf("Scanning from %lf MHz to %lf MHz with step %lf MHz\n", start / 1e6,
         end / 1e6, step / 1e6);
  for (int freq = start; ((step > 0) ? (freq <= end) : (freq >= end));
       freq += step) {
    LMX2572_SetFrequency(freq);
    // TIM_DelayUs(delay);
    AD7606B_CollectSamples(ad_data, AD7606B_CHANNEL_FLAG_CH4, 8, 100e3);
    double avg = 0;
    for (int i = 0; i < 8; i++) {
      avg += ad_data[i];
    }
    avg = avg * 2.5 / 32768 / 8 + 2.5;
    if (avg > threshold) {
      printf("Detected voltage %lfV at %lf MHz\n", avg, freq / 1e6);
      return freq;
    }
  }
  printf("Nothing detected.\n");
  return 0;
}

double APP_PowerDetectorFineScan(double center, double range, double step,
                                 int mode, double threshold) {
  APP_SetRFSwitch(mode);
  printf("Scanning around %lf MHz", center / 1e6);
  double result = 0;
  double sum = 0;
  int channel = (mode == RFSWITCH_DETECTOR) ? AD7606B_CHANNEL_FLAG_CH4
                                            : AD7606B_CHANNEL_FLAG_CH3;
  for (int freq = center - range; freq <= center + range; freq += step) {
    LMX2572_SetFrequency(freq);
    // TIM_DelayUs(delay);
    AD7606B_CollectSamples(ad_data, mode, 8, 100e3);
    double avg = 0;
    for (int i = 0; i < 8; i++) {
      avg += ad_data[i];
    }
    avg = avg * 2.5 / 32768 / 8 + (mode == RFSWITCH_DETECTOR ? 2.5 : 0);
    sum += avg;
    result += avg * freq;
  }
  result /= sum;
  printf("Detected center frequency %lf MHz\n", result / 1e6);
  return result;
}

float fft_buffer[AD_SAMPLE_COUNT * 2];
float fft_mag[AD_SAMPLE_COUNT];

void APP_DetectAMFM(double center) {
  APP_SetRFSwitch(RFSWITCH_DEMODULATOR);
  LMX2572_SetFrequency(center);

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
  printf("AM SNR: %lf\n", amSnr);
  printf("FM SNR: %lf\n", fmSnr);
  if (amSpectrum.peakAmp < 50 && fmSpectrum.peakAmp < 50) {
    printf(">> No Modulation\n");
  } else if (amSnr > fmSnr) {
    printf(">> AM: %lfkHz\n", amSpectrum.peakFreq / 1e3);
  } else {
    printf(">> FM: %lfkHz\n", fmSpectrum.peakFreq / 1e3);
  }
}

void APP_FullPowerScanSystem(double threshold1, double threshold2) {
  printf(">> Start full power scan\n");
  double freq_rough1 = APP_PowerDetectorScan(843e6, 423e6, -5e6, threshold1);
  if (freq_rough1 == 0) {
    printf(">> No signal detected.\n");
    return;
  }
  double freq_rough2 = APP_PowerDetectorFineScan(freq_rough2, 20e6, 100e3,
                                                 RFSWITCH_DETECTOR, threshold1);
  double carrier = freq_rough2 - 433e6;
  printf("Detected carrier frequency around %lf MHz\n", carrier / 1e6);
  double freq_fine = APP_PowerDetectorFineScan(
      freq_rough2 + 10.7e6, 1e6, 10e3, RFSWITCH_DEMODULATOR, threshold2);
  carrier = freq_fine - 10.7e6;
  printf(">> Carrier frequency %lf MHz\n", carrier / 1e6);
  APP_DetectAMFM(freq_fine);
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
    uint8_t channels = 0x0f;
    uint32_t sample_count = 128;
    readCount = UART_Read(&com_buf, data + 1, 5, 1000);
    if (readCount != 5)
      return;
    channels = data[1];
    sample_count = *(uint32_t *)(data + 2);
    APP_UploadADCDataFast(channels, sample_count);
  }
}

void APP_KeyCallback(uint8_t event) {
  if (event == KEYS_EVENT_PRESS) {
    LED_On(1);
    LED_On(2);
    LED_On(3);
  } else if (event == KEYS_EVENT_RELEASE) {
    LED_Off(1);
    LED_Off(2);
    LED_Off(3);
  }
}

KEYS_Pins keys_pins;
void APP_InitKeys() {
  keys_pins.keyCount = 4;
  keys_pins.pins[0].port = SWITCH2_GPIO_Port;
  keys_pins.pins[0].pin = SWITCH2_Pin;
  keys_pins.pins[0].callback = APP_KeyCallback;
  keys_pins.pins[1].port = SWITCH1_GPIO_Port;
  keys_pins.pins[1].pin = SWITCH1_Pin;
  keys_pins.pins[1].callback = APP_KeyCallback;
  keys_pins.pins[2].port = SWITCH3_GPIO_Port;
  keys_pins.pins[2].pin = SWITCH3_Pin;
  keys_pins.pins[2].callback = APP_KeyCallback;
  keys_pins.pins[3].port = SWITCH4_GPIO_Port;
  keys_pins.pins[3].pin = SWITCH4_Pin;
  keys_pins.pins[3].callback = APP_KeyCallback;
  keys_pins.htim = &htim7;

  KEYS_Init(&keys_pins);
  KEYS_Start();
}

void APP_Init() {
  POWER_Use400MHzClocks();
  computer = &huart1;
  RetargetInit(computer);
  BOARD_InitAD7606();
  BOARD_InitAD9959();
  // BOARD_InitLMX2572();
  APP_InitRFSwitch();
  UART_RxBuffer_Init(&com_buf, computer);
  UART_Open(&com_buf);
  APP_InitKeys();
  printf("CMake is working!\n");
}

void APP_Loop() {
  APP_PollUartCommands();
  KEYS_Poll();
}