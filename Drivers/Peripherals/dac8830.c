#include "dac8830.h"

#ifdef DAC8830_ENABLE

#include <stdio.h>

#include "tim.h"

void DAC8830_Init(void) {
  HAL_GPIO_WritePin(DA_CS1_GPIO_Port, DA_CS1_Pin | DA_CS2_Pin, GPIO_PIN_SET);
}

void DAC8830_Delay() {
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

void DAC8830_SetVoltage(int chip, uint16_t voltage) {
  uint16_t pin =
      ((chip & 0x1) ? DA_CS1_Pin : 0) | ((chip & 0x2) ? DA_CS2_Pin : 0);
  HAL_GPIO_WritePin(DA_CS1_GPIO_Port, pin, GPIO_PIN_RESET);
  for (int i = 15; i >= 0; i--) {
    if (voltage & (1 << i)) {
      HAL_GPIO_WritePin(DA_SDI_GPIO_Port, DA_SDI_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(DA_SDI_GPIO_Port, DA_SDI_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(DA_SCLK_GPIO_Port, DA_SCLK_Pin, GPIO_PIN_RESET);
    DAC8830_Delay();
    HAL_GPIO_WritePin(DA_SCLK_GPIO_Port, DA_SCLK_Pin, GPIO_PIN_SET);
    DAC8830_Delay();
  }
  HAL_GPIO_WritePin(DA_CS1_GPIO_Port, pin, GPIO_PIN_SET);
  DAC8830_Delay();
}

#define SAMPLES_PER_PERIOD 128
const uint16_t sine_wave[128] = {
    32767, 34374, 35978, 37574, 39159, 40728, 42278, 43805, 45306, 46776, 48213,
    49612, 50971, 52286, 53554, 54771, 55936, 57045, 58096, 59085, 60011, 60872,
    61664, 62388, 63039, 63618, 64123, 64552, 64904, 65179, 65376, 65494, 65534,
    65494, 65376, 65179, 64904, 64552, 64123, 63618, 63039, 62388, 61664, 60872,
    60011, 59085, 58096, 57045, 55936, 54771, 53554, 52286, 50971, 49612, 48213,
    46776, 45306, 43805, 42278, 40728, 39159, 37574, 35978, 34374, 32766, 31159,
    29555, 27959, 26374, 24805, 23255, 21728, 20227, 18757, 17320, 15921, 14562,
    13247, 11979, 10762, 9597,  8488,  7437,  6448,  5522,  4661,  3869,  3145,
    2494,  1915,  1410,  981,   629,   354,   157,   39,    0,     39,    157,
    354,   629,   981,   1410,  1915,  2494,  3145,  3869,  4661,  5522,  6448,
    7437,  8488,  9597,  10762, 11979, 13247, 14562, 15921, 17320, 18757, 20227,
    21728, 23255, 24805, 26374, 27959, 29555, 31159,
};

int sine_wave_chip; // 0x0 = disabled, 0x1 = enabled chip 1, 0x2 =
                    // enabled chip 2, 0x3 = enabled both chips
int sine_wave_index;
int sine_wave_step;
int sine_wave_amplitude;

void DAC8830_StartSineWave(int chip, int frequency, int amplitude) {
  sine_wave_chip = chip;
  sine_wave_index = 0;
  sine_wave_amplitude = amplitude;
  if (frequency > 10000) {
    sine_wave_step = 8;
  } else if (frequency > 5000) {
    sine_wave_step = 4;
  } else if (frequency > 2500) {
    sine_wave_step = 2;
  } else {
    sine_wave_step = 1;
  }
  int base_freq = HAL_RCC_GetPCLK1Freq() / (TIM3->PSC + 1);
  int period = base_freq / (frequency * SAMPLES_PER_PERIOD / sine_wave_step);
  printf("DAC8830_StartSineWave: base_freq=%d, period=%d\n", base_freq, period);
  __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);
  HAL_TIM_Base_Start_IT(&htim3);
}

void DAC8830_StopSineWave() {
  sine_wave_chip = 0;
  HAL_TIM_Base_Stop_IT(&htim3);
}

void DAC8830_TimerCallback() {
  uint32_t value =
      (uint32_t)sine_wave[sine_wave_index] * sine_wave_amplitude / 65535;
  DAC8830_SetVoltage(sine_wave_chip, value);
  sine_wave_index += sine_wave_step;
  if (sine_wave_index >= SAMPLES_PER_PERIOD) {
    sine_wave_index = 0;
  }
}

#endif