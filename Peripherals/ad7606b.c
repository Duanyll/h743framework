#include "ad7606b.h"

#include <math.h>

#include "cmsis_gcc.h"
#include "led.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "timers.h"


static AD7606B_Pins *pins;

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET
#define WRITE(pin, value)                                                      \
  HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, (value) ? HIGH : LOW)

volatile BOOL AD7606B_isSampling;
volatile BOOL AD7606B_badSampleFlag;
uint16_t *AD7606B_output;
uint16_t AD7606B_lastSample[8];
volatile int AD7606B_sampleCount;
uint8_t AD7606B_channels;
int AD7606B_channelCount;
void (*AD7606B_SampleCallback)(int16_t *data);

void AD7606B_SetDBInput() {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Pin = 0xFFFF, // All pins
      .Mode = GPIO_MODE_INPUT,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(pins->DB_Port, &GPIO_InitStruct);
}

void AD7606B_SetDBOutput() {
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Pin = 0xFFFF, // All pins
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(pins->DB_Port, &GPIO_InitStruct);
}

// delay for at least 40ns
void AD7606B_Delay() {
  __NOP();
  __NOP();
  __NOP();
  // __NOP();
}

void AD7606B_FullReset(void) {
  WRITE(RESET, HIGH);
  TIM_DelayUs(5);
  WRITE(RESET, LOW);
  TIM_DelayUs(300);
}

void AD7606B_Init(AD7606B_Pins *p) {
  pins = p;

  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
#define INIT(pin)                                                              \
  GPIO_InitStruct.Pin = pins->pin##_Pin;                                       \
  HAL_GPIO_Init(pins->pin##_Port, &GPIO_InitStruct)

  INIT(CS);
  INIT(RD);
  INIT(RESET);
  INIT(CONVST);
  INIT(WR);
  INIT(OS0);
  INIT(OS1);
  INIT(OS2);
  INIT(PAR_SEL);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  INIT(BUSY);
#undef INIT

  WRITE(PAR_SEL, LOW);
  WRITE(OS0, HIGH);
  WRITE(OS1, HIGH);
  WRITE(OS2, HIGH);
  WRITE(CONVST, HIGH);
  WRITE(WR, HIGH);
  WRITE(CS, HIGH);
  WRITE(RD, HIGH);
  WRITE(RESET, LOW);

  AD7606B_SetDBInput();
  AD7606B_FullReset();
}

static int popcount(uint32_t x) {
  // Count the number of bits set in a 32-bit integer
  int count = 0;
  while (x) {
    count += x & 1;
    x >>= 1;
  }
  return count;
}

uint8_t AD7606B_ParallelRegisterRead(uint8_t addr) {
  AD7606B_SetDBOutput();
  WRITE(CS, LOW);
  pins->DB_Port->ODR = pins->DataToPins((1 << 15) | (addr << 8));
  WRITE(WR, LOW);
  AD7606B_Delay();
  AD7606B_Delay();
  AD7606B_Delay();
  AD7606B_Delay();
  WRITE(WR, HIGH);
  AD7606B_Delay();
  AD7606B_SetDBInput();
  WRITE(RD, LOW);
  AD7606B_Delay();
  uint16_t data = pins->PinsToData(pins->DB_Port->IDR);
  WRITE(RD, HIGH);
  WRITE(CS, HIGH);
  AD7606B_Delay();
  // printf("Read %04X\n", data);
  return data;
}

void AD7606B_ParallelRegisterWrite(uint8_t addr, uint8_t data) {
  AD7606B_SetDBOutput();
  WRITE(CS, LOW);
  pins->DB_Port->ODR = pins->DataToPins((0 << 15) | (addr << 8) | data);
  WRITE(WR, LOW);
  AD7606B_Delay();
  AD7606B_Delay();
  AD7606B_Delay();
  AD7606B_Delay();
  WRITE(WR, HIGH);
  WRITE(CS, HIGH);
  AD7606B_Delay();
  AD7606B_SetDBInput();
  TIM_DelayUs(2);
}

void AD7606B_LeaveRegisterMode() {
  AD7606B_SetDBOutput();
  WRITE(CS, LOW);
  pins->DB_Port->ODR = 0;
  WRITE(WR, LOW);
  AD7606B_Delay();
  AD7606B_Delay();
  AD7606B_Delay();
  AD7606B_Delay();
  WRITE(WR, HIGH);
  WRITE(CS, HIGH);
  AD7606B_Delay();
  AD7606B_SetDBInput();
}

void AD7606B_ADCConvert(uint16_t *data, uint8_t channels) {
  LED_On(3);
  AD7606B_isSampling = TRUE;
  WRITE(CS, LOW);
  WRITE(CONVST, LOW);
  AD7606B_Delay();
  WRITE(CONVST, HIGH);
  AD7606B_Delay();
  if (AD7606B_SampleCallback != NULL) {
    int16_t *lastSample = (int16_t *)AD7606B_lastSample;
    for (int i = 0; i < AD7606B_channelCount; i++) {
      lastSample[i] = pins->PinsToData(AD7606B_lastSample[i]);
    }
    AD7606B_SampleCallback(lastSample);
  }
  while (HAL_GPIO_ReadPin(pins->BUSY_Port, pins->BUSY_Pin) == GPIO_PIN_SET)
    ;
  AD7606B_Delay();
  for (int i = 0; i < 8; i++) {
    WRITE(RD, LOW);
    AD7606B_Delay();
    if (channels & (1 << i)) {
      *data = pins->DB_Port->IDR;
      data++;
    }
    WRITE(RD, HIGH);
    AD7606B_Delay();
  }
  WRITE(CS, HIGH);
  AD7606B_Delay();
  AD7606B_isSampling = FALSE;
  LED_Off(3);
}

void AD7606B_TimerCallback();

BOOL AD7606B_CollectSamples(int16_t *data, uint8_t channels, uint32_t count,
                            double sampleRate) {
  AD7606B_sampleCount = count;
  AD7606B_output = (uint16_t *)data;
  AD7606B_channels = channels;
  AD7606B_badSampleFlag = FALSE;
  AD7606B_channelCount = popcount(channels);
  AD7606B_SampleCallback = NULL;
  AD7606B_LeaveRegisterMode();
  TIM_RegisterCallback(pins->TIM_Handle, AD7606B_TimerCallback);
  TIM_StartPeriodic(pins->TIM_Handle, sampleRate);
  while (AD7606B_sampleCount > 0) {
    if (AD7606B_badSampleFlag) {
      TIM_StopPeriodic(pins->TIM_Handle);
      return FALSE;
    }
  }
  TIM_StopPeriodic(pins->TIM_Handle);
  int sampleCount = count * popcount(channels);
  for (int i = 0; i < sampleCount; i++) {
    data[i] = (int16_t)pins->PinsToData(data[i]);
  }
  return TRUE;
}

double AD7606B_FastCollectSamples(int16_t *data, uint8_t channels,
                                  uint32_t count) {
  __disable_irq();
  // Reset TIM2 to 0
  TIM2->CNT = 0;
  __HAL_TIM_ENABLE(&htim2);
  for (int cnt = 0; cnt < count; cnt++) {
    WRITE(CS, LOW);
    WRITE(CONVST, LOW);
    AD7606B_Delay();
    WRITE(CONVST, HIGH);
    AD7606B_Delay();

    while (HAL_GPIO_ReadPin(pins->BUSY_Port, pins->BUSY_Pin) == GPIO_PIN_SET)
      ;

    AD7606B_Delay();
    for (int i = 0; i < 8; i++) {
      WRITE(RD, LOW);
      AD7606B_Delay();
      if (channels & (1 << i)) {
        *data = pins->DB_Port->IDR;
        data++;
      }
      WRITE(RD, HIGH);
      AD7606B_Delay();
    }

    WRITE(CS, HIGH);
    AD7606B_Delay();
  }
  __HAL_TIM_DISABLE(&htim2);
  __enable_irq();
  double time = (double)TIM2->CNT / (HAL_RCC_GetPCLK1Freq() * 2);
  int points = count * popcount(channels);
  double sampleRate = points / time;
  for (int i = 0; i < points; i++) {
    data[i] = (int16_t)pins->PinsToData(data[i]);
  }
  return sampleRate;
}

void AD7606B_StartContinuousConvert(double sampleRate, uint8_t channels,
                                    void (*callback)(int16_t *data)) {
  AD7606B_sampleCount = -1;
  AD7606B_channels = channels;
  AD7606B_SampleCallback = callback;
  AD7606B_badSampleFlag = FALSE;
  AD7606B_channelCount = popcount(channels);
  AD7606B_LeaveRegisterMode();
  TIM_RegisterCallback(pins->TIM_Handle, AD7606B_TimerCallback);
  TIM_StartPeriodic(pins->TIM_Handle, sampleRate);
}
void AD7606B_StopContinuousConvert(void) {
  AD7606B_SampleCallback = NULL;
  AD7606B_badSampleFlag = FALSE;
  AD7606B_sampleCount = 0;
  TIM_StopPeriodic(pins->TIM_Handle);
}

void AD7606B_TimerCallback() {
  if (AD7606B_isSampling) {
    AD7606B_badSampleFlag = TRUE;
    LED_On(1);
    return;
  } else if (AD7606B_sampleCount > 0) {
    AD7606B_ADCConvert(AD7606B_output, AD7606B_channels);
    AD7606B_output += AD7606B_channelCount;
    AD7606B_sampleCount--;
  } else if (AD7606B_sampleCount == 0) {
    TIM_StopPeriodic(pins->TIM_Handle);
  } else if (AD7606B_sampleCount == -1) {
    AD7606B_ADCConvert(AD7606B_lastSample, AD7606B_channels);
  }
}

void AD7606B_InitConfig(AD7606B_Config *config) {
  for (int i = 0; i <= AD7606B_REG_ID; i++) {
    config->raw[i] = 0;
  }
  config->raw[AD7606B_REG_CONFIG] = 0x08;
  for (int i = 0; i < 4; i++) {
    config->raw[AD7606B_REG_RANGE + i] = 0x33;
  }
  for (int i = 0; i < 8; i++) {
    config->raw[AD7606B_REG_OFFSET + i] = 0x80;
  }
  config->raw[AD7606B_REG_DIGITAL_DIAG_ENABLE] = 0x01;
}

void AD7606B_SetRange(AD7606B_Config *config, uint8_t channel, uint8_t range) {
  uint8_t reg = AD7606B_REG_RANGE + (channel >> 1);
  uint8_t shift = (channel & 1) << 2;
  config->raw[reg] &= ~(0x0F << shift);
  config->raw[reg] |= (range << shift);
  AD7606B_ParallelRegisterWrite(reg, config->raw[reg]);
}
void AD7606B_SetOverSample(AD7606B_Config *config, uint8_t oversample_padding,
                           uint8_t oversample) {
  config->raw[AD7606B_REG_OVERSAMPLE] = (oversample_padding << 4) | oversample;
  AD7606B_ParallelRegisterWrite(AD7606B_REG_OVERSAMPLE,
                                config->raw[AD7606B_REG_OVERSAMPLE]);
}
void AD7606B_SetGain(AD7606B_Config *config, uint8_t channel, uint8_t gain) {
  uint8_t reg = AD7606B_REG_GAIN + channel;
  config->raw[reg] = gain;
  AD7606B_ParallelRegisterWrite(reg, config->raw[reg]);
}
void AD7606B_SetOffset(AD7606B_Config *config, uint8_t channel,
                       uint16_t offset) {
  uint8_t reg = AD7606B_REG_OFFSET + channel;
  config->raw[reg] = offset;
  AD7606B_ParallelRegisterWrite(reg, config->raw[reg]);
}
void AD7606B_SetPhase(AD7606B_Config *config, uint8_t channel, uint8_t phase) {
  uint8_t reg = AD7606B_REG_PHASE + channel;
  config->raw[reg] = phase;
  AD7606B_ParallelRegisterWrite(reg, config->raw[reg]);
}
void AD7606B_SetDiagMux(AD7606B_Config *config, uint8_t channel,
                        uint8_t diag_mux) {
  uint8_t reg = AD7606B_REG_DIAG_MUX + (channel >> 1);
  uint8_t shift = (channel & 1) ? 3 : 0;
  config->raw[reg] &= ~(0x07 << shift);
  config->raw[reg] |= (diag_mux << shift);
  AD7606B_ParallelRegisterWrite(reg, config->raw[reg]);
}