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

double APP_PowerDetectorScan(double start, double end, double step,
                             double threshold, int delay) {
  printf("Scanning from %lf MHz to %lf MHz with step %lf MHz\n", start / 1e6,
         end / 1e6, step / 1e6);
  for (int freq = start; ((step > 0) ? (freq <= end) : (freq >= end));
       freq += step) {
    SI5351_SetupCLK0(freq, SI5351_DRIVE_STRENGTH_8MA);
    SI5351_SetupCLK2(freq, SI5351_DRIVE_STRENGTH_8MA);
    TIM_DelayUs(delay);
    AD7606B_CollectSamples(ad_data, AD7606B_CHANNEL_FLAG_CH3, 8, 100e3);
    double avg = 0;
    for (int i = 0; i < 8; i++) {
      avg += ad_data[i];
    }
    avg = avg * 2.5 / 32768 / 8;
    if (avg > threshold) {
      printf("Detected voltage %lfV at %lf MHz\n", avg, freq / 1e6);
      return freq;
    }
  }
  printf("Nothing detected.\n");
  return 0;
}

double APP_PowerDetectorFineScan(double center, int order, double threshold,
                                 int delay) {
  printf("Scanning around %lf MHz with order %d\n", center / 1e6, order);
  double result = 0;
  double sum = 0;
  for (int freq = center - 0.7e6 / order; freq <= center + 0.7e6 / order;
       freq += 10e3 / order) {
    SI5351_SetupCLK0(freq, SI5351_DRIVE_STRENGTH_8MA);
    SI5351_SetupCLK2(freq, SI5351_DRIVE_STRENGTH_8MA);
    TIM_DelayUs(delay);
    AD7606B_CollectSamples(ad_data, AD7606B_CHANNEL_FLAG_CH3, 8, 100e3);
    double avg = 0;
    for (int i = 0; i < 8; i++) {
      avg += ad_data[i];
    }
    avg = avg * 2.5 / 32768 / 8;
    sum += avg;
    result += avg * freq;
  }
  result /= sum;
  printf("Detected center frequency %lf MHz\n", result / 1e6);
  return result;
}

float fft_buffer[AD_SAMPLE_COUNT * 2];
float fft_mag[AD_SAMPLE_COUNT];

void APP_DetectAMFM(double center, int delay) {
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

void APP_FinalScan(double center, int order, int symbol, int delay) {
  center = APP_PowerDetectorFineScan(center, order, 0.3, delay);
  printf(">> Carrier frequency: %lf MHz\n",
         center * order / 1e6 - symbol * 10.7);
  APP_DetectAMFM(center, 10000);
}

void APP_InitFilters() {
  // Init GPIO C0-C3 as output
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

#define FILTER_NONE 0     // 0 0 0 0
#define FILTER_LPF_145M 1 // 0 1 1 1
#define FILTER_LPF_200M 2 // 1 1 0 1

void APP_SelectFilter(uint8_t filter) {
  if (filter == FILTER_NONE) {
    printf("No filter\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
  } else if (filter == FILTER_LPF_145M) {
    printf("145MHz LPF\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  } else if (filter == FILTER_LPF_200M) {
    printf("200MHz LPF\n");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
  }
  TIM_DelayUs(100);
}

void APP_LowPowerScanSystemOrder2(int threshold_int, int delay) {
  int startTick = HAL_GetTick();
  double threshold = threshold_int / 1000.0;

  APP_SelectFilter(FILTER_LPF_200M);
  double result_200_150 =
      APP_PowerDetectorScan(200e6, 150e6, -0.25e6, threshold, delay);
  if (result_200_150 == 0) {
    APP_SelectFilter(FILTER_NONE);
    double result_400_300 =
        APP_PowerDetectorScan(200e6, 150e6, -0.125e6, threshold, delay);
    if (result_400_300 == 0) {
      APP_SelectFilter(FILTER_LPF_145M);
      double result_150_100 =
          APP_PowerDetectorScan(150e6, 100e6, -0.25e6, threshold, delay);
      if (result_150_100 == 0) {
        APP_SelectFilter(FILTER_NONE);
        double result_300_200 =
            APP_PowerDetectorScan(150e6, 100e6, -0.125e6, threshold, delay);
        if (result_300_200 == 0) {
          APP_SelectFilter(FILTER_LPF_145M);
          double result_100_15 =
              APP_PowerDetectorScan(100e6, 15e6, -0.25e6, threshold, delay);
          if (result_100_15 == 0) {
            printf(">> No signal\n");
          } else {
            APP_FinalScan(result_100_15, 1, +1, delay);
          }
        } else {
          APP_FinalScan(result_300_200, 2, +1, delay);
        }
      } else {
        APP_FinalScan(result_150_100, 1, +1, delay);
      }
    } else {
      APP_FinalScan(result_400_300, 2, +1, delay);
    }
  } else if (result_200_150 <= 178e6) {
    APP_FinalScan(result_200_150, 1, +1, delay);
  } else {
    APP_SelectFilter(FILTER_NONE);
    double result_upper = APP_PowerDetectorScan(
        result_200_150 + 20e6, result_200_150 + 23e6, 0.25e6, threshold, delay);
    if (result_upper == 0) {
      APP_FinalScan(result_200_150, 1, +1, delay);
    } else {
      APP_FinalScan(result_200_150, 1, -1, delay);
    }
  }

  int endTick = HAL_GetTick();
  printf("Time: %dms\n", endTick - startTick);
}

void APP_LowPowerScanSystemOrder3(int threshold_int, int delay) {
  int startTick = HAL_GetTick();
  double threshold = threshold_int / 1000.0;

  APP_SelectFilter(FILTER_NONE);
  double result_200_133 =
      APP_PowerDetectorScan(200e6, 133e6, -0.25e6, threshold, delay);
  if (result_200_133 == 0) {
    APP_SelectFilter(FILTER_LPF_145M);
    double result_133_10 =
        APP_PowerDetectorScan(133e6, 20e6, -0.25e6, threshold, delay);
    if (result_133_10 == 0) {
      APP_SelectFilter(FILTER_NONE);
      double result_400_200 =
          APP_PowerDetectorScan(140e6, 66e6, -0.08e6, threshold, delay);
      if (result_400_200 == 0) {
        printf(">> No signal\n");
      } else {
        APP_FinalScan(result_400_200, 3, +1, delay);
      }
    } else {
      APP_FinalScan(result_133_10, 1, +1, delay);
    }
  } else if (result_200_133 <= 178e6) {
    APP_FinalScan(result_200_133, 1, +1, delay);
  } else {
    APP_SelectFilter(FILTER_NONE);
    double result_upper = APP_PowerDetectorScan(
        result_200_133 + 20e6, result_200_133 + 23e6, 0.25e6, threshold, delay);
    if (result_upper == 0) {
      APP_FinalScan(result_200_133, 1, +1, delay);
    } else {
      APP_FinalScan(result_200_133, 1, -1, delay);
    }
  }

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
    readCount = UART_Read(&com_buf, data + 1, 9, 1000);
    if (readCount != 9)
      return;
    uint8_t order = *(uint8_t *)(data + 1);
    int threshold = *(int *)(data + 2);
    int delay = *(int *)(data + 6);
    if (order == 2) {
      APP_LowPowerScanSystemOrder2(threshold, delay);
    } else if (order == 3) {
      APP_LowPowerScanSystemOrder3(threshold, delay);
    }
    printf("END\n");
  } else if (*data == 5) {
    readCount = UART_Read(&com_buf, data + 1, 1, 1000);
    if (readCount != 1)
      return;
    uint8_t filter = *(uint8_t *)(data + 1);
    APP_SelectFilter(filter);
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
  BOARD_InitSI5351();
  APP_InitFilters();
  UART_RxBuffer_Init(&com_buf, computer);
  UART_Open(&com_buf);
  // UI_InitEPD();
  // UI_TestEPD();
  APP_InitKeys();
}

void APP_Loop() {
  APP_PollUartCommands();
  KEYS_Poll();
}