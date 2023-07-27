#pragma once

#include "common.h"

/* ------------------------------- AD7606 Pins ------------------------------ */
// Use parallel interface. The following pins are used:
// AD_PAR_SET (May be tied to LOW)
// AD_STBY (May be tied to HIGH)
// AD_RANGE
// AD_OS0 AD_OS1 AD_OS2
// AD_CONVSTA AD_CONVSTB
// AD_RD
// AD_CS
// AD_BUSY
// AD_RESET
// AD_DB0 - AD_DB15: These pins are connected to the same GPIO port for fast
// reading. If they are not connected in order, edit the code in
// AD7606_ConvertData() to rearrange the bits into the correct order.
//
// Also enable TIM2 and call AD7606_TimerCallback() in TIM2_IRQHandler().

/* ------------------------------ AD7606 Usage ------------------------------ */
// 1. Call AD7606_Init() to initialize the driver.
// 2. Create a AD7606_Config struct and set the desired configuration.
// 3. Call AD7606_SetConfig() to set the configuration.
// 4. Call AD7606_ApplyConfig() to apply the configuration.
// 5. Call AD7606_Sample() to sample one sample.
// 6. Call AD7606_CollectSamples() to collect samples synchronously at given
// sample rate.

typedef uint16_t(AD7606_DataConvertFunction)(uint16_t data);

typedef struct {
  GPIO_TypeDef *PAR_SEL_Port;
  uint16_t PAR_SEL_Pin;
  GPIO_TypeDef *STBY_Port;
  uint16_t STBY_Pin;
  GPIO_TypeDef *RANGE_Port;
  uint16_t RANGE_Pin;
  GPIO_TypeDef *OS0_Port;
  uint16_t OS0_Pin;
  GPIO_TypeDef *OS1_Port;
  uint16_t OS1_Pin;
  GPIO_TypeDef *OS2_Port;
  uint16_t OS2_Pin;
  GPIO_TypeDef *CONVSTA_Port;
  uint16_t CONVSTA_Pin;
  GPIO_TypeDef *CONVSTB_Port;
  uint16_t CONVSTB_Pin;
  GPIO_TypeDef *RD_Port;
  uint16_t RD_Pin;
  GPIO_TypeDef *CS_Port;
  uint16_t CS_Pin;
  GPIO_TypeDef *BUSY_Port;
  uint16_t BUSY_Pin;
  GPIO_TypeDef *RESET_Port;
  uint16_t RESET_Pin;

  GPIO_TypeDef *DB_Port;
  AD7606_DataConvertFunction *PinsToData;

  TIM_HandleTypeDef* TIM_Handle;
} AD7606_Pins;

#define AD_RANGE_10V 1
#define AD_RANGE_5V 0

#define AD_OVERSAMPLE_NONE 0
#define AD_OVERSAMPLE_2X 1
#define AD_OVERSAMPLE_4X 2
#define AD_OVERSAMPLE_8X 3
#define AD_OVERSAMPLE_16X 4
#define AD_OVERSAMPLE_32X 5
#define AD_OVERSAMPLE_64X 6

typedef struct AD7606_Config {
  uint32_t range;      // AD_RANGE_10V or AD_RANGE_5V
  uint32_t oversample; // See AD_OVERSAMPLE_*
  uint32_t channels;   // Bit mask of enabled channels. 0x1 = channel 1, 0x2 =
                       // channel 2, etc. There are 8 channels.
} AD7606_Config;

// Initialize the AD7606 driver.
void AD7606_Init(AD7606_Pins *pins);
// Apply the current configuration to the AD7606.
void AD7606_ApplyConfig(void);
// Reset the AD7606
void AD7606_Reset(void);
// Update the configuration of the AD7606. Call AD7606_ApplyConfig() after this
// to apply.
int AD7606_SetConfig(AD7606_Config *config);
// Take one sample from AD7606. Only save enabled channels.

// Sample one sample from the AD7606 synchronously. Use AD7606_ConvertData() to
// get real data.
void AD7606_Sample(uint16_t *output);
// Collect samples from the AD7606 synchronously. This function blocks until
// all samples are collected. Units for sampleRate is Hz. Return whether the
// operation is successful. Only save enabled channels into output in
// interleaved format.
BOOL AD7606_CollectSamples(int count, int sampleRate, int16_t *output);