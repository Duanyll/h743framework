#ifdef AD7606_ENABLE

#include "ad7606.h"

#include <stdio.h>

#include "timers.h"

static AD7606_Pins *pins;

#define AD_WRITE(Pin, Value)                                                   \
  HAL_GPIO_WritePin(pins->Pin##_Port, pins->Pin##_Pin, (Value))
#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

AD7606_Config ad7606_config;
volatile BOOL ad7606_isSampling;
volatile int ad7606_sampleCount;
uint16_t *ad7606_output;
volatile BOOL ad7606_badSampleFlag;

void AD7606_Init(AD7606_Pins *p) {
  pins = p;

  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
#define INIT(pin)                                                              \
  GPIO_InitStruct.Pin = pins->pin##_Pin;                                       \
  HAL_GPIO_Init(pins->pin##_Port, &GPIO_InitStruct)

  INIT(PAR_SEL);
  INIT(STBY);
  INIT(RANGE);
  INIT(OS0);
  INIT(OS1);
  INIT(OS2);
  INIT(CONVSTA);
  INIT(CONVSTB);
  INIT(RD);
  INIT(CS);
  INIT(RESET);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  INIT(BUSY);
#undef INIT
  
  GPIO_InitStruct.Pins = 0xffff;
  HAL_GPIO_Init(pins->DB_Port, &GPIO_InitStruct);

  ad7606_config.range = AD_RANGE_5V;
  ad7606_config.oversample = AD_OVERSAMPLE_NONE;
  ad7606_config.channels = 0x01;
  ad7606_isSampling = FALSE;
  ad7606_sampleCount = 0;
  ad7606_output = NULL;
  ad7606_badSampleFlag = FALSE;
  AD7606_ApplyConfig();
  AD7606_Reset();
}

void AD7606_Delay(int delay) {
  for (int i = 0; i < delay / 2; i++) {
    __NOP();
  }
}

void AD7606_ApplyConfig(void) {
  // Set parallel mode
  AD_WRITE(PAR_SEL, LOW);

  // Leave standby mode
  AD_WRITE(STBY, HIGH);

  // Set range
  if (ad7606_config.range == AD_RANGE_10V) {
    AD_WRITE(RANGE, HIGH);
  } else {
    AD_WRITE(RANGE, LOW);
  }

  // Set oversample
  AD_WRITE(OS0, (ad7606_config.oversample >> 0) & 1);
  AD_WRITE(OS1, (ad7606_config.oversample >> 1) & 1);
  AD_WRITE(OS2, (ad7606_config.oversample >> 2) & 1);

  AD_WRITE(CONVSTA, HIGH);
  AD_WRITE(CONVSTB, HIGH);
  AD_WRITE(RD, HIGH);
  AD_WRITE(CS, HIGH);
}

void AD7606_Reset() {
  // Reset the AD7606
  AD_WRITE(RESET, HIGH);
  AD7606_Delay(100);
  AD_WRITE(RESET, LOW);
  AD7606_Delay(100);
}

int AD7606_SetConfig(AD7606_Config *config) {
  // Validate range
  if (config->range != AD_RANGE_10V && config->range != AD_RANGE_5V) {
    return -1;
  }
  // Validate oversample
  if (config->oversample > AD_OVERSAMPLE_64X) {
    return -1;
  }
  // Validate channels
  if (config->channels > 0xFF) {
    return -1;
  }
  ad7606_config = *config;
  return 0;
}

static uint32_t bitwise_reverse(uint8_t x) {
  // Reverse the bits in a byte
  return ((x & 0x01) << 7) | ((x & 0x02) << 5) | ((x & 0x04) << 3) |
         ((x & 0x08) << 1) | ((x & 0x10) >> 1) | ((x & 0x20) >> 3) |
         ((x & 0x40) >> 5) | ((x & 0x80) >> 7);
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

void AD7606_Sample(uint16_t *output) {
  ad7606_isSampling = TRUE;
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, HIGH);
  AD_WRITE(CONVSTA, LOW);
  AD7606_Delay(50);
  AD_WRITE(CONVSTA, HIGH);
  AD_WRITE(CONVSTB, LOW);
  AD7606_Delay(50);
  AD_WRITE(CONVSTB, HIGH);
  AD7606_Delay(100);
  if (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == LOW) {
    printf("Conversion not started\n");
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, HIGH);
  }
  // Wait for BUSY to go low
  while (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == HIGH)
    ;
  AD7606_Delay(50);
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(AD_CS_GPIO_Port, AD_CS_Pin | AD_RD_Pin, LOW);
    AD7606_Delay(50);
    if (ad7606_config.channels & (1 << i)) {
      *output = pins->DB_Port->IDR;
      output++;
    }
    HAL_GPIO_WritePin(AD_CS_GPIO_Port, AD_CS_Pin | AD_RD_Pin, HIGH);
    AD7606_Delay(50);
  }
  ad7606_isSampling = FALSE;
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, LOW);
}

BOOL AD7606_CollectSamples(int count, int sampleRate, int16_t *output) {
  TIM_RegisterCallback(pins->TIM_Handle, AD7606_TimerCallback);
  ad7606_sampleCount = count;
  ad7606_output = (uint16_t *)output;
  ad7606_badSampleFlag = FALSE;
  AD7606_Reset();
  TIM_StartPeriodic(pins->TIM_Handle, sampleRate);
  while (ad7606_sampleCount > 0) {
  }
  TIM_StopPeriodic(pins->TIM_Handle);
  int sampleCount = count * popcount(ad7606_config.channels);
  for (int i = 0; i < sampleCount; i++) {
    output[i] = pins->PinsToData(output[i]);
  }
  return !ad7606_badSampleFlag;
}

void AD7606_TimerCallback() {
  if (ad7606_isSampling) {
    ad7606_badSampleFlag = TRUE;
    return;
  } else if (ad7606_sampleCount <= 0) {
    return;
  }
  AD7606_Sample(ad7606_output);
  ad7606_output += popcount(ad7606_config.channels);
  ad7606_sampleCount--;
}

#endif