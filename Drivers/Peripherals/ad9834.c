#ifdef AD9834_ENABLE

#include "ad9834.h"

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET
#define WRITE(pin, state) \
    HAL_GPIO_WritePin(DDS_##pin##_GPIO_Port, DDS_##pin##_Pin, (state))

// At least 25ns
void AD9834_Delay() {
    for (int i = 0; i < 20; i++) {
        __NOP();
    }
}

void AD9834_Init() {
    WRITE(SCK, HIGH);
    WRITE(SDA, LOW);
    WRITE(FSY, HIGH);
}

void AD9834_WriteRaw(uint16_t data) {
    WRITE(FSY, LOW);
    AD9834_Delay();
    for (int i = 15; i >= 0; i--) {
        WRITE(SDA, ((data >> i) & 1) ? HIGH : LOW);
        AD9834_Delay();
        WRITE(SCK, LOW);
        AD9834_Delay();
        WRITE(SCK, HIGH);
    }
    AD9834_Delay();
    WRITE(FSY, HIGH);
    AD9834_Delay();
}

void AD9834_InitConfig(AD9834_Config* config) {
    config->control.value = 0;
    config->freq[0] = config->freq[1] = 1;
    config->phase[0] = config->phase[1] = 0;
}

void AD9834_WriteControl(AD9834_Config* config) {
    config->control.hlb = 0;
    config->control.b28 = 0;
    config->control.reserved_2 = 0;
    AD9834_WriteRaw(config->control.value);
}
void AD9834_WriteFreq(AD9834_Config* config, uint8_t reg) {
    config->control.hlb = 0;
    config->control.b28 = 1;
    config->control.reserved_2 = 0;
    config->control.fsel = reg;
    AD9834_WriteRaw(config->control.value);
    AD9834_WriteRaw((1 << (14 + reg)) | (config->freq[reg] & 0x3FFF));  // LSB
    AD9834_WriteRaw((1 << (14 + reg)) | (config->freq[reg] >> 14));     // MSB
    AD9834_WriteControl(config);
}
void AD9834_WritePhase(AD9834_Config* config, uint8_t reg) {
    AD9834_WriteControl(config);
    AD9834_WriteRaw((3 << 14) | (reg << 13) | (config->phase[reg] & 0xFFF));
}

void AD9834_SetFreq(AD9834_Config* config, uint8_t reg, double freq,
                    double mclk) {
    config->freq[reg] = (uint32_t)(freq * (1 << 28) / mclk);
    // Make sure freq is in 28 bits
    config->freq[reg] &= 0x0FFFFFFF;
    AD9834_WriteFreq(config, reg);
}
void AD9834_SetPhase(AD9834_Config* config, uint8_t reg, double phase) {
    config->phase[reg] = (uint16_t)(phase * (1 << 12) / 360);
    // Make sure phase is in 12 bits
    config->phase[reg] &= 0x0FFF;
    AD9834_WritePhase(config, reg);
}

#endif