#include "dac8830.h"

#ifdef DAC8830_ENABLE

#include <stdio.h>

#include "tim.h"

#define WRITE(pin, value)                                                      \
  HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, value)

void DAC8830_Init(DAC8830_Pins *pins) {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  GPIO_InitStruct.Pin = pins->CS_Pin;
  HAL_GPIO_Init(pins->CS_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->SCK_Pin;
  HAL_GPIO_Init(pins->SCK_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->SDI_Pin;
  HAL_GPIO_Init(pins->SDI_Port, &GPIO_InitStruct);
  WRITE(CS, HIGH);
  WRITE(SCK, LOW);
  WRITE(SDI, LOW);
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

void DAC8830_SetVoltage(DAC8830_Pins *pins, uint16_t voltage) {
  WRITE(CS, LOW);
  for (int i = 15; i >= 0; i--) {
    if (voltage & (1 << i)) {
      WRITE(SDI, HIGH);
    } else {
      WRITE(SDI, LOW);
    }
    WRITE(SCK, LOW);
    DAC8830_Delay();
    WRITE(SCK, HIGH);
    DAC8830_Delay();
  }
  WRITE(CS, HIGH);
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

DAC8830_Pins *sine_wave_chip; // 0x0 = disabled, 0x1 = enabled chip 1, 0x2 =
                              // enabled chip 2, 0x3 = enabled both chips
int sine_wave_index;
int sine_wave_step;
int sine_wave_amplitude;

void DAC8830_TimerCallback();

void DAC8830_StartSineWave(DAC8830_Pins *pins, int frequency, int amplitude) {
  sine_wave_chip = pins;
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
  double sampleRate = frequency / sine_wave_step * SAMPLES_PER_PERIOD;
  TIM_RegisterCallback(pins->TIM_Handle, DAC8830_TimerCallback);
  TIM_StartPeriodic(pins->TIM_Handle, sampleRate);
}

void DAC8830_StopSineWave(DAC8830_Pins *pins) {
  sine_wave_chip = 0;
  TIM_StopPeriodic(pins->TIM_Handle);
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