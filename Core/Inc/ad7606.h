/**
 * AD7606 driver. Supports one AD7606 chip with parallel interface.
 */
#pragma once

#include "main.h"

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
    uint32_t range;       // AD_RANGE_10V or AD_RANGE_5V
    uint32_t oversample;  // See AD_OVERSAMPLE_*
    uint32_t channels;  // Bit mask of enabled channels. 0x1 = channel 1, 0x2 =
                        // channel 2, etc. There are 8 channels.
} AD7606_Config;

// Initialize the AD7606 driver.
void AD7606_Init(void);
// Apply the current configuration to the AD7606.
void AD7606_ApplyConfig(void);
// Reset the AD7606
void AD7606_Reset(void);
// Update the configuration of the AD7606. Call AD7606_ApplyConfig() after this
// to apply.
int AD7606_SetConfig(AD7606_Config *config);
// Take one sample from AD7606
void AD7606_Sample(uint16_t *output);
// Collect samples from the AD7606 synchronously. This function blocks until
// all samples are collected. Units for sampleRate is Hz. Return whether the
// operation is successful.
BOOL AD7606_CollectSamples(int count, int sampleRate, uint16_t *output);

void AD7606_TimerCallback(void);