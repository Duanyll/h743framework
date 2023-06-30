#include "ad7606.h"

#include <stdio.h>

#include "tim.h"

#define AD_WRITE(Pin, Value) \
    HAL_GPIO_WritePin(AD_##Pin##_GPIO_Port, AD_##Pin##_Pin, (Value))
#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET

AD7606_Config ad7606_config;
BOOL ad7606_isSampling;
int ad7606_sampleCount;
uint16_t* ad7606_output;
BOOL ad7606_badSampleFlag;

void AD7606_Init(void) {
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

void AD7606_ApplyConfig(void) {
    // Set parallel mode
    AD_WRITE(PAR_SET, LOW);

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
    delay_ns(100);
    AD_WRITE(RESET, LOW);
    delay_ns(100);
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

uint16_t AD7606_ConvertData(uint16_t data) {
    // DB0-DB7 are connected to D7-D0 on the STM32.
    // DB8-DB15 are connected to D8-D15 on the STM32.
    // Need to bitwise reverse low 8 bits
    return (bitwise_reverse(data >> 8) << 8) | (data & 0xFF);
}

void AD7606_Sample(uint16_t *output) {
    ad7606_isSampling = TRUE;
    AD_WRITE(CONVSTA, LOW);
    delay_ns(50);
    AD_WRITE(CONVSTA, HIGH);
    AD_WRITE(CONVSTB, LOW);
    delay_ns(50);
    AD_WRITE(CONVSTB, HIGH);
    delay_ns(100);
    if (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == LOW) {
        printf("Conversion not started\n");
    }
    // Wait for BUSY to go low
    while (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == HIGH)
        ;
    delay_ns(50);
    for (int i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(AD_CS_GPIO_Port, AD_CS_Pin | AD_RD_Pin, LOW);
        delay_ns(50);
        if (ad7606_config.channels & (1 << i)) {
            *output = AD_DB0_GPIO_Port->IDR;
            output++;
        }
        HAL_GPIO_WritePin(AD_CS_GPIO_Port, AD_CS_Pin | AD_RD_Pin, HIGH);
        delay_ns(50);
    }
    ad7606_isSampling = FALSE;
}

BOOL AD7606_CollectSamples(int count, int sampleRate, uint16_t *output) {
    // Config tim2 to trigger at sampleRate
    int period = HAL_RCC_GetPCLK1Freq() / (TIM2->PSC + 1) / sampleRate;
    __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    printf("period: %d\n", period);
    ad7606_sampleCount = count;
    ad7606_output = output;
    ad7606_badSampleFlag = FALSE;
    AD7606_Reset();
    HAL_TIM_Base_Start_IT(&htim2);
    while (ad7606_sampleCount > 0) {
    }
    HAL_TIM_Base_Stop_IT(&htim2);
    int sampleCount = count * popcount(ad7606_config.channels);
    for (int i = 0; i < sampleCount; i++) {
        output[i] = AD7606_ConvertData(output[i]);
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