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
  AD7606B_SetRange(&ad7606b_config, 0, AD7606B_RANGE_2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606B_RANGE_2V5);
  AD7606B_SetOverSample(&ad7606b_config, 0, AD7606B_OVERSAMPLE_4);
  AD7606B_SetOverSample(&ad7606b_config, 1, AD7606B_OVERSAMPLE_4);
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

static int compare_int16(const void *a, const void *b) {
  return *(int16_t *)a - *(int16_t *)b;
}

float i_buf[AD_SAMPLE_COUNT];
float q_buf[AD_SAMPLE_COUNT];

#define DELTA_TOL 20

int APP_IQRoughScan(uint32_t sample_count, uint32_t sample_rate) {
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  AD7606B_CollectSamples(ad_data, 0x0f, sample_count, sample_rate);
  int16_t *i_buf_s16 = (int16_t *)i_buf;
  int16_t *q_buf_s16 = (int16_t *)q_buf;
  for (int i = 0; i < sample_count; i++) {
    i_buf_s16[i] = ad_data[i * 4 + 0] - ad_data[i * 4 + 1];
    q_buf_s16[i] = ad_data[i * 4 + 2] - ad_data[i * 4 + 3];
  }
  qsort(i_buf_s16, sample_count, sizeof(int16_t), compare_int16);
  qsort(q_buf_s16, sample_count, sizeof(int16_t), compare_int16);
  int16_t i_delta = i_buf_s16[sample_count - DELTA_TOL] - i_buf_s16[DELTA_TOL];
  int16_t q_delta = q_buf_s16[sample_count - DELTA_TOL] - q_buf_s16[DELTA_TOL];

  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  return (i_delta > q_delta ? i_delta : q_delta);
}

typedef struct {
  BOOL amDetected;
  double frequency;
  float fmAmplitude;
} APP_IQPreciseScanResult;

APP_IQPreciseScanResult APP_IQPreciseScan(uint32_t sample_count,
                                          uint32_t sample_rate) {
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  AD7606B_CollectSamples(ad_data, 0x0f, sample_count, sample_rate);
  for (int i = 0; i < sample_count; i++) {
    i_buf[i] =
        ((float)ad_data[i * 4 + 0] - (float)ad_data[i * 4 + 1]) / 32768 * 2.5;
    q_buf[i] =
        ((float)ad_data[i * 4 + 2] - (float)ad_data[i * 4 + 3]) / 32768 * 2.5;
  }
  double corr = SIGNAL_GetCorrelationF32(i_buf, q_buf, sample_count);
  if (fabs(corr) > 0.7) {
    APP_IQPreciseScanResult result;
    result.amDetected = TRUE;
    SIGNAL_TimeDataF32 amTimeData = {
        .points = sample_count / 2,
        .timeData = i_buf,
        .offset = 0,
        .stride = 2,
        .sampleRate = sample_rate / 2,
    };
    SIGNAL_SpectrumF32 amSpectrum;
    SIGNAL_TimeF32ToSpectrumF32(&amTimeData, &amSpectrum);
    result.frequency = amSpectrum.peakFreq;
    result.fmAmplitude = 0;
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    return result;
  }
  double i_mean = 0;
  double q_mean = 0;
  for (int i = 0; i < sample_count; i++) {
    i_mean += i_buf[i];
    q_mean += q_buf[i];
  }
  i_mean /= sample_count;
  q_mean /= sample_count;
  for (int i = 0; i < sample_count; i++) {
    i_buf[i] = atan2f(q_buf[i] - q_mean, i_buf[i] - i_mean);
  }
  SIGNAL_UnwrapPhaseF32(i_buf, sample_count);
  SIGNAL_TimeDataF32 fmTimeData = {
      .points = sample_count / 2,
      .timeData = i_buf,
      .offset = 0,
      .stride = 2,
      .sampleRate = sample_rate / 2,
  };
  SIGNAL_SpectrumF32 fmSpectrum;
  SIGNAL_TimeF32ToSpectrumF32(&fmTimeData, &fmSpectrum);
  APP_IQPreciseScanResult result;
  result.amDetected = FALSE;
  if (fmSpectrum.peakFreq < 3100 && fmSpectrum.peakFreq > 200) {
    result.frequency = fmSpectrum.peakFreq;
    result.fmAmplitude = fmSpectrum.peakAmp;
  } else {
    result.frequency = 0;
    result.fmAmplitude = 0;
  }
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  return result;
}

typedef struct {
  double basebandFrequency;
  BOOL amDetected;
  BOOL fmDetected;
  double modulationFrequency;
} APP_IQDemodulationResult;

APP_IQDemodulationResult APP_IQFullScan() {
  uint32_t sample_count = 2048;
  uint32_t sample_rate = 200000;

  uint32_t roughFrequency = 0;
  int roughAmplitude = 0;
  for (int freq = 30e6; freq <= 100e6; freq += 0.5e6) {
    APP_SetFrequency(freq * 2);
    HAL_Delay(10);
    int amp = APP_IQRoughScan(sample_count, sample_rate);
    UART_Printf(computer_uart, "Rough %d %d\n", freq, amp);
    if (amp > roughAmplitude) {
      roughAmplitude = amp;
      roughFrequency = freq;
    }
  }

  if (roughAmplitude < 20) {
    APP_IQDemodulationResult result;
    result.basebandFrequency = 0;
    result.amDetected = FALSE;
    result.fmDetected = FALSE;
    result.modulationFrequency = 0;
    return result;
  }

  uint32_t preciseFrequency = 0;
  float preciseAmplitude = 0;
  double fmFrequency = 0;
  for (int freq = roughFrequency - 0.7e6; freq <= roughFrequency + 0.7e6;
       freq += 0.01e6) {
    APP_SetFrequency(freq * 2);
    HAL_Delay(10);
    APP_IQPreciseScanResult cur = APP_IQPreciseScan(sample_count, sample_rate);
    UART_Printf(computer_uart, "Precise %d %d %.2f %.2lf\n", freq,
                cur.amDetected, cur.fmAmplitude, cur.frequency);
    if (cur.amDetected) {
      APP_IQDemodulationResult result;
      result.basebandFrequency = freq;
      result.amDetected = TRUE;
      result.fmDetected = FALSE;
      result.modulationFrequency = cur.frequency;
    } else if (cur.fmAmplitude > preciseAmplitude) {
      preciseAmplitude = cur.fmAmplitude;
      preciseFrequency = freq;
      fmFrequency = cur.frequency;
    }
  }

  APP_IQDemodulationResult result;
  result.basebandFrequency = preciseFrequency;
  result.amDetected = FALSE;
  result.fmDetected = TRUE;
  result.modulationFrequency = fmFrequency;
  return result;
}

void APP_IQDemodulationTest(uint32_t sample_count, uint32_t sample_rate) {
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  AD7606B_CollectSamples(ad_data, 0x0f, sample_count, sample_rate);

  HAL_UART_Transmit(computer_uart, "\xff\xff\xff\xff", 4, 1000);
  /* ---------------------------- AM/FM Detection --------------------------- */
  for (int i = 0; i < sample_count; i++) {
    i_buf[i] =
        ((float)ad_data[i * 4 + 0] - (float)ad_data[i * 4 + 1]) / 32768 * 2.5;
    q_buf[i] =
        ((float)ad_data[i * 4 + 2] - (float)ad_data[i * 4 + 3]) / 32768 * 2.5;
  }
  double corr = SIGNAL_GetCorrelationF32(i_buf, q_buf, sample_count);
  HAL_UART_Transmit(computer_uart, (uint8_t *)&corr, sizeof(corr), 1000);

  /* ----------------------------- AM Frequency ----------------------------- */
  SIGNAL_TimeDataF32 amTimeData = {
      .points = sample_count / 2,
      .timeData = i_buf,
      .offset = 0,
      .stride = 2,
      .sampleRate = sample_rate / 2,
  };
  SIGNAL_SpectrumF32 amSpectrum;
  SIGNAL_TimeF32ToSpectrumF32(&amTimeData, &amSpectrum);
  HAL_UART_Transmit(computer_uart, (uint8_t *)&amSpectrum.peakFreq,
                    sizeof(amSpectrum.peakFreq), 1000);

  /* ----------------------------- FM Frequency ----------------------------- */
  double i_mean = 0;
  double q_mean = 0;
  for (int i = 0; i < sample_count; i++) {
    i_mean += i_buf[i];
    q_mean += q_buf[i];
  }
  i_mean /= sample_count;
  q_mean /= sample_count;
  for (int i = 0; i < sample_count; i++) {
    i_buf[i] = atan2f(q_buf[i] - q_mean, i_buf[i] - i_mean);
  }
  SIGNAL_UnwrapPhaseF32(i_buf, sample_count);
  SIGNAL_TimeDataF32 fmTimeData = {
      .points = sample_count / 2,
      .timeData = i_buf,
      .offset = 0,
      .stride = 2,
      .sampleRate = sample_rate / 2,
  };
  SIGNAL_SpectrumF32 fmSpectrum;
  SIGNAL_TimeF32ToSpectrumF32(&fmTimeData, &fmSpectrum);
  // SIGNAL_PeaksF32 fmPeaks;
  // SIGNAL_FindPeaksF32(&fmSpectrum, &fmPeaks, 0.3, 10);
  double peakFreq = fmSpectrum.peakFreq;
  // double peakAmp = 0;
  // // Find the peak with the highest amplitude within 200Hz - 3100Hz
  // for (int i = 0; i < fmPeaks.count; i++) {
  //   if (fmPeaks.peaks[i].freq >= 200 && fmPeaks.peaks[i].freq <= 3100 &&
  //       fmPeaks.peaks[i].amp > peakAmp) {
  //     peakFreq = fmPeaks.peaks[i].freq;
  //     peakAmp = fmPeaks.peaks[i].amp;
  //   }
  // }
  HAL_UART_Transmit(computer_uart, (uint8_t *)&peakFreq, sizeof(peakFreq),
                    1000);

  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void APP_HexCommandCallback(uint8_t *data, int len) {
  if (*data == '\x01') {
    // 1 byte channels
    // 4 byte sample count
    // 4 byte sample rate
    uint8_t channels = data[1];
    uint32_t sample_count = *(uint32_t *)(data + 2);
    uint32_t sample_rate = *(uint32_t *)(data + 6);
    APP_UploadADCData(channels, sample_count, sample_rate);
  } else if (*data == '\x02') {
    // 4 byte frequency
    uint32_t frequency = *(uint32_t *)(data + 1);
    APP_SetFrequency(frequency);
  } else if (*data == '\x03') {
    // 4 byte sample count
    // 4 byte sample rate
    uint32_t sample_count = *(uint32_t *)(data + 1);
    uint32_t sample_rate = *(uint32_t *)(data + 5);
    APP_IQDemodulationTest(sample_count, sample_rate);
  } else if (*data == '\x04') {
    APP_IQDemodulationResult result = APP_IQFullScan();
    UART_Printf(computer_uart, "Baseband: %.2lf\n", result.basebandFrequency);
    UART_Printf(computer_uart, "AM: %d, FM: %d\n", result.amDetected,
                result.fmDetected);
    UART_Printf(computer_uart, "Modulation: %.2lf\n",
                result.modulationFrequency);
  }
}

void APP_Init() {
  APP_InitAD9959();
  APP_InitAD7606B();
  DAC8830_Init();
  computer_uart = &huart2;
  UART_ResetHexRX(computer_uart);
  // AD7606B_StartContinuousConvert(100e3, 0x03, APP_ADCCallback);
}

void APP_Loop() { UART_PollHexData(APP_HexCommandCallback); }