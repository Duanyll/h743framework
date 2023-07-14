#pragma once

#ifdef AD9959_ENABLE

#include "main.h"

/* ------------------------------- AD9959 Pins ------------------------------ */
// DDS_SDIO0
// DDS_CLK
// DDS_IOUP
// DDS_CS
// DDS_RST

/* ------------------------------ AD9959 Usage ------------------------------ */
// 1. Create a AD9959_GlobalConfig struct
// 2. Call AD9959_Init(&config)
// 3. To edit global config, edit the struct and call AD9959_Write(ADDR, DATA)
// to write the updated register. ADDR is one of the AD9959_xxx_ADDR macros.
// DATA is the raw data to be written.
// 4. To edit channel config,
//    1. Create a AD9959_ChannelConfig struct
//    2. Call AD9959_InitChannelConfig(&config)
//    3. Select the channel to edit by calling AD9959_SelectChannel(CHANNEL).
//    CHANNEL is 4-bit flag combination of AD9959_CHANNEL_x
//    4. Edit the struct and call AD9959_Write(ADDR, DATA) to write.
//    5. To edit the channel frequency, phase or amplitude, call
//    AD9959_SetFrequency, AD9959_SetPhase or AD9959_SetAmplitude.
//    6. Finally call AD9959_IOUpdate() to apply the changes.
// 5. To read a register, call AD9959_Read(ADDR).

typedef union {
    struct {
        uint8_t lsb_first : 1;       // Do not modify
        uint8_t serial_io_mode : 2;  // Do not modify
        uint8_t reserved_0 : 1;
        uint8_t channel_0_enable : 1;
        uint8_t channel_1_enable : 1;
        uint8_t channel_2_enable : 1;
        uint8_t channel_3_enable : 1;
    };
    uint8_t raw;
} AD9959_CSR;

typedef union {
    struct {
        uint32_t manual_software_sync : 1;
        uint32_t manual_hardware_sync : 1;
        uint32_t reserved_2 : 2;
        uint32_t dac_reference_power_down : 1;
        uint32_t sync_clk_disable : 1;
        uint32_t external_power_down : 1;
        uint32_t ref_clock_input_power_down : 1;
        uint32_t modulation_level : 2;
        uint32_t ru_rd : 2;
        uint32_t ppc : 3;
        uint32_t reserved_1 : 1;
        uint32_t charge_pump_control : 2;
        uint32_t pll_divider_ratio : 5;
        uint32_t vco_gain_control : 1;
        uint32_t reserved_0 : 8;
    };
    uint32_t raw;
} AD9959_FR1;

typedef union {
    struct {
        uint16_t system_clock_offset : 2;
        uint16_t reserved_2 : 2;
        uint16_t multidevice_sync_mask : 1;
        uint16_t multidevice_sync_status : 1;
        uint16_t multidevice_sync_master_enable : 1;
        uint16_t auto_sync_enable : 1;
        uint16_t reserved_1 : 4;
        uint16_t all_channels_clear_phase_acc : 1;
        uint16_t all_channels_autoclear_phase_acc : 1;
        uint16_t all_channels_clear_sweep_acc : 1;
        uint16_t all_channels_autoclear_sweep_acc : 1;
    };
    uint16_t raw;
} AD9959_FR2;

typedef struct AD9959_GlobalConfig {
    AD9959_CSR csr;  // 0x00
    AD9959_FR1 fr1;  // 0x01
    AD9959_FR2 fr2;  // 0x02
} AD9959_GlobalConfig;

#define AD9959_CSR_ADDR 0x00
#define AD9959_FR1_ADDR 0x01
#define AD9959_FR2_ADDR 0x02

typedef union {
    struct {
        uint32_t sine_wave_out_enable : 1;  // 0 = Cosine, 1 = Sine
        uint32_t clear_phase_acc : 1;
        uint32_t autoclear_phase_acc : 1;
        uint32_t clear_sweep_acc : 1;
        uint32_t autoclear_sweep_acc : 1;
        uint32_t dac_power_down : 1;
        uint32_t digital_power_down : 1;
        uint32_t dac_full_scale_current : 2;
        uint32_t reserved_2 : 3;
        uint32_t load_srr_at_io_update : 1;
        uint32_t linear_sweep_enable : 1;
        uint32_t linear_sweep_no_dwell : 1;
        uint32_t reserved_1 : 6;
        uint32_t afp_select : 2;
        uint32_t reserved_0 : 8;
    };
    uint32_t raw;
} AD9959_CFR;

typedef union {
    struct {
        uint32_t amplitude_scale_factor : 10;
        uint32_t load_arr_at_io_update : 1;
        uint32_t ramp_enable : 1;
        uint32_t amplitude_multiplier_enable : 1;
        uint32_t reserved_1 : 1;
        uint32_t inc_dec_step_size : 2;
        uint32_t amplitude_ramp_rate : 8;
        uint32_t reserved_0 : 8;
    };
    uint32_t raw;
} AD9959_ACR;

typedef union {
    struct {
        uint16_t rsrr : 8;
        uint16_t fsrr : 8;
    };
    uint16_t raw;
} AD9959_LSRR;

typedef struct AD9959_ChannelConfig {
    AD9959_CFR cfr;             // 0x03
    uint32_t cftw0;             // 0x04
    uint16_t cpow0;             // 0x05, 14-bit
    AD9959_ACR acr;             // 0x06
    AD9959_LSRR lsrr;           // 0x07
    uint32_t rdw;               // 0x08
    uint32_t fdw;               // 0x09
    uint32_t tuning_words[15];  // 0x0A - 0x18, MSB aligned
} AD9959_ChannelConfig;

#define AD9959_CFR_ADDR 0x03
#define AD9959_CFTW0_ADDR 0x04
#define AD9959_CPOW0_ADDR 0x05
#define AD9959_ACR_ADDR 0x06
#define AD9959_LSRR_ADDR 0x07
#define AD9959_RDW_ADDR 0x08
#define AD9959_FDW_ADDR 0x09
#define AD9959_TUNING_WORDS_ADDR 0x0A

// Trigger AD9959 IOUpdate to apply changes
void AD9959_IOUpdate();
// Reset AD9959 and set all registers to default values
void AD9959_Reset(AD9959_GlobalConfig* config);
// Init AD9959
void AD9959_Init(AD9959_GlobalConfig* config);

void AD9959_InitGlobalConfig(AD9959_GlobalConfig* config);
void AD9959_InitChannelConfig(AD9959_ChannelConfig* config);

// Write to AD9959 register. May need to trigger IOUpdate to apply changes
void AD9959_Write(uint8_t addr, uint32_t data);
// Read from AD9959 register
uint32_t AD9959_Read(uint8_t addr);

// Calculate frequency tuning word for AD9959. Unit: Hz
uint32_t AD9959_CalculateFTW(double freq, double sysclk);
// Calculate phase offset word for AD9959. Unit: degree
uint16_t AD9959_CalculatePOW(double phase);

// Set frequency for AD9959 channel. Manual trigger IOUpdate to apply change.
// Unit: Hz
void AD9959_SetFrequency(AD9959_ChannelConfig* config, double freq,
                         double sysclk);
// Set phase for AD9959 channel. Manual trigger IOUpdate to apply change. Unit:
// degree
void AD9959_SetPhase(AD9959_ChannelConfig* config, double phase);
// Set amplitude for AD9959 channel. Manual trigger IOUpdate to apply change. 10
// bits resolution.
void AD9959_SetAmplitude(AD9959_ChannelConfig* config, uint16_t amplitude);

#define AD9959_CHANNEL_0 0x01
#define AD9959_CHANNEL_1 0x02
#define AD9959_CHANNEL_2 0x04
#define AD9959_CHANNEL_3 0x08
void AD9959_SelectChannels(AD9959_GlobalConfig* config, uint8_t channels);

#endif