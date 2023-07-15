#ifdef AD7606B_ENABLE

#include "ad7606b.h"

#include <math.h>

#include "tim.h"

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET
#define WRITE(pin, value)                                   \
    HAL_GPIO_WritePin(AD_##pin##_GPIO_Port, AD_##pin##_Pin, \
                      (value) ? HIGH : LOW)

BOOL AD7606B_isSampling;
BOOL AD7606B_badSampleFlag;
uint16_t *AD7606B_output;
int AD7606B_sampleCount;
uint8_t AD7606B_channels;

void AD7606B_SetDBInput() {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = 0xFFFF,  // All pins
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(AD_DB0_GPIO_Port, &GPIO_InitStruct);
}

void AD7606B_SetDBOutput() {
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = 0xFFFF,  // All pins
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(AD_DB0_GPIO_Port, &GPIO_InitStruct);
}

// delay for at least 40ns
void AD7606B_Delay() {
    for (int i = 0; i < 20; i++) {
        __NOP();
    }
}

void AD7606B_FullReset(void) {
    WRITE(RESET, HIGH);
    delay_us(5);
    WRITE(RESET, LOW);
    delay_us(300);
}

void AD7606B_Init(void) {
    WRITE(PAR_SET, LOW);
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

uint16_t AD7606B_DataToPins(uint16_t data) {
    return (bitwise_reverse(data >> 8) << 8) | bitwise_reverse(data & 0xFF);
}

uint16_t AD7606B_PinsToData(uint16_t pins) {
    return (bitwise_reverse(pins >> 8) << 8) | bitwise_reverse(pins & 0xFF);
}

uint8_t AD7606B_ParallelRegisterRead(uint8_t addr) {
    AD7606B_SetDBOutput();
    WRITE(CS, LOW);
    AD_DB0_GPIO_Port->ODR = AD7606B_DataToPins((1 << 15) | (addr << 8));
    WRITE(WR, LOW);
    AD7606B_Delay();
    WRITE(WR, HIGH);
    AD7606B_Delay();
    AD7606B_SetDBInput();
    WRITE(RD, LOW);
    AD7606B_Delay();
    uint8_t data = AD7606B_PinsToData(AD_DB0_GPIO_Port->IDR);
    WRITE(RD, HIGH);
    WRITE(CS, HIGH);
    AD7606B_Delay();
    return data;
}

void AD7606B_ParallelRegisterWrite(uint8_t addr, uint8_t data) {
    AD7606B_SetDBOutput();
    WRITE(CS, LOW);
    AD_DB0_GPIO_Port->ODR = AD7606B_DataToPins((0 << 15) | (addr << 8) | data);
    WRITE(WR, LOW);
    AD7606B_Delay();
    WRITE(WR, HIGH);
    WRITE(CS, HIGH);
    AD7606B_Delay();
    AD7606B_SetDBInput();
}

void AD7606B_LeaveRegisterMode() {
    AD7606B_SetDBOutput();
    WRITE(CS, LOW);
    AD_DB0_GPIO_Port->ODR = 0;
    WRITE(WR, LOW);
    AD7606B_Delay();
    WRITE(WR, HIGH);
    WRITE(CS, HIGH);
    AD7606B_Delay();
    AD7606B_SetDBInput();
}

void AD7606B_ADCConvert(uint16_t *data, uint8_t channels) {
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    AD7606B_isSampling = TRUE;
    WRITE(CS, LOW);
    WRITE(CONVST, LOW);
    AD7606B_Delay();
    WRITE(CONVST, HIGH);
    AD7606B_Delay();
    while (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == GPIO_PIN_SET)
        ;
    AD7606B_Delay();
    for (int i = 0; i < 8; i++) {
        WRITE(RD, LOW);
        AD7606B_Delay();
        if (channels & (1 << i)) {
            *data = AD_DB0_GPIO_Port->IDR;
            data++;
        }
        WRITE(RD, HIGH);
        AD7606B_Delay();
    }
    WRITE(CS, HIGH);
    AD7606B_Delay();
    AD7606B_isSampling = FALSE;
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

BOOL AD7606B_CollectSamples(int16_t *data, uint8_t channels, uint32_t count,
                            double sampleRate) {
    // Note that clock for timer is 2x PCLK1
    int period =
        round(HAL_RCC_GetPCLK1Freq() * 2 / (TIM2->PSC + 1) / sampleRate) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    AD7606B_sampleCount = count;
    AD7606B_output = (uint16_t *)data;
    AD7606B_channels = channels;
    AD7606B_badSampleFlag = FALSE;
    AD7606B_LeaveRegisterMode();
    HAL_TIM_Base_Start_IT(&htim2);
    while (AD7606B_sampleCount > 0) {
        if (AD7606B_badSampleFlag) {
            HAL_TIM_Base_Stop_IT(&htim2);
            return FALSE;
        }
    }
    HAL_TIM_Base_Stop_IT(&htim2);
    int sampleCount = count * popcount(channels);
    for (int i = 0; i < sampleCount; i++) {
        data[i] = (int16_t)AD7606B_PinsToData(data[i]);
    }
    return TRUE;
}

void AD7606B_TimerCallback() {
    if (AD7606B_isSampling) {
        AD7606B_badSampleFlag = TRUE;
        return;
    } else if (AD7606B_sampleCount > 0) {
        AD7606B_ADCConvert(AD7606B_output, AD7606B_channels);
        AD7606B_output += popcount(AD7606B_channels);
        AD7606B_sampleCount--;
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
    config->raw[AD7606B_REG_OVERSAMPLE] =
        (oversample_padding << 4) | oversample;
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

#endif