#pragma once

#include "common.h"

typedef union {
  struct {
    uint32_t lsb_first : 1;       // do not change
    uint32_t sdio_input_only : 1; // do not change
    uint32_t reserved_1 : 1;
    uint32_t external_power_down : 1;
    uint32_t aux_dac_power_down : 1;
    uint32_t refclk_input_power_down : 1;
    uint32_t dac_power_down : 1;
    uint32_t digital_power_down : 1;
    uint32_t select_auto_osk : 1;
    uint32_t osk_enable : 1;
    uint32_t load_arr_at_io_update : 1;
    uint32_t clear_phase_acc : 1;
    uint32_t clear_digital_ramp_acc : 1;
    uint32_t auto_clear_phase_acc : 1;
    uint32_t auto_clear_digital_ramp_acc : 1;
    uint32_t load_lrr_at_io_update : 1;
    uint32_t select_dds_sine_output : 1;
    uint32_t internal_profile_control : 4;
    uint32_t reserved_2 : 1;
    uint32_t inverse_sinc_filter_enable : 1;
    uint32_t manual_osk_external_control : 1;
    uint32_t reserved_3 : 5;
    uint32_t ram_playback_dest : 2;
    uint32_t ram_enable : 1;
  };
  uint32_t raw;
} AD9910_Register_CFR1;

typedef union {
  struct {
    uint32_t fm_gain : 4;
    uint32_t parallel_data_port_enable : 1;
    uint32_t sync_timing_validation_disable : 1;
    uint32_t data_assembler_hold_last_value : 1;
    uint32_t marched_latency_enable : 1;
    uint32_t reserved_1 : 1;
    uint32_t tx_enable_invert : 1;
    uint32_t pdclk_invert : 1;
    uint32_t pdclk_enable : 1;
    uint32_t reserved_2 : 2;
    uint32_t io_update_rate : 2;
    uint32_t read_effective_ftw : 1;
    uint32_t digital_ramp_no_dwell_low : 1;
    uint32_t digital_ramp_no_dwell_high : 1;
    uint32_t digital_ramp_enable : 1;
    uint32_t digital_ramp_destination : 2;
    uint32_t sync_clk_enable : 1;
    uint32_t internal_io_update_active : 1;
    uint32_t enable_amp_scal_from_singal_tone_profile : 1;
    uint32_t reserved_3 : 7;
  };
  uint32_t raw;
} AD9910_Register_CFR2;

typedef union {
  struct {
    uint32_t reserved_1 : 1;
    uint32_t n : 7;
    uint32_t pll_enable : 1;
    uint32_t reserved_2 : 1;
    uint32_t pfd_reset : 1;
    uint32_t reserved_3 : 3;
    uint32_t refclk_input_divider_reset_b : 1;
    uint32_t refclk_input_divider_bypass : 1;
    uint32_t reserved_4 : 3;
    uint32_t icp : 3;
    uint32_t reserved_5 : 2;
    uint32_t vco_sel : 3;
    uint32_t reserved_6 : 1;
    uint32_t drv0 : 2;
    uint32_t reserved_7 : 2;
  };
  uint32_t raw;
} AD9910_Register_CFR3;

typedef union {
  struct {
    uint32_t amp_step_size : 2;
    uint32_t amp_scale_factor : 14;
    uint32_t amp_ramp_rate : 16;
  };
  uint32_t raw;
} AD9910_Register_ASF;

typedef union {
  struct {
    uint32_t reserved_1 : 3;
    uint32_t input_sync_receiver_delay : 5;
    uint32_t reserved_2 : 3;
    uint32_t output_sync_generator_delay : 5;
    uint32_t reserved_3 : 2;
    uint32_t sync_state_preset : 6;
    uint32_t reserved_4 : 1;
    uint32_t sync_generator_polarity : 1;
    uint32_t sync_generator_enable : 1;
    uint32_t sync_receiver_enable : 1;
    uint32_t sync_validation_delay : 4;
  };
  uint32_t raw;
} AD9910_Register_MultichipSync;

typedef union {
  struct {
    uint64_t lower : 32;
    uint64_t upper : 32;
  };
  uint64_t raw;
} AD9910_Register_DigitalRampLimit;

typedef union {
  struct {
    uint64_t increment : 32;
    uint64_t decrement : 32;
  };
  uint64_t raw;
} AD9910_Register_DigitalRampStep;

typedef union {
  struct {
    uint64_t positive : 32;
    uint64_t negative : 32;
  };
  uint64_t raw;
} AD9910_Register_DigitalRampRate;

typedef union {
  struct {
    uint64_t ftw : 32;
    uint64_t pow : 16;
    uint64_t asf : 14;
    uint64_t reserved_1 : 2;
  };
  uint64_t raw;
} AD9910_Register_SingleToneProfile;

typedef union {
  struct {
    uint64_t mode_control : 3;
    uint64_t zero_crossing : 1;
    uint64_t reserved_1 : 1;
    uint64_t no_dwell_high : 1;
    uint64_t reserved_2 : 8;
    uint64_t start_address : 10;
    uint64_t reserved_3 : 6;
    uint64_t end_address : 10;
    uint64_t address_step_rate : 16;
    uint64_t reserved_4 : 8;
  };
  uint64_t raw;
} AD9910_Register_RamProfile;

typedef struct {
  AD9910_Register_CFR1 cfr1;
  AD9910_Register_CFR2 cfr2;
  AD9910_Register_CFR3 cfr3;
  uint8_t aux_dac_control;
  uint32_t io_update_rate;
  uint32_t ftw;
  uint16_t pow;
  AD9910_Register_ASF asf;
  AD9910_Register_MultichipSync multichip_sync;
  AD9910_Register_DigitalRampLimit digital_ramp_limit;
  AD9910_Register_DigitalRampStep digital_ramp_step;
  AD9910_Register_DigitalRampRate digital_ramp_rate;
  AD9910_Register_SingleToneProfile single_tone_profile[8];
  AD9910_Register_RamProfile ram_profile[8];
} AD9910_Config;

#define AD9910_ADDR_CFR1 0x00
#define AD9910_ADDR_CFR2 0x01
#define AD9910_ADDR_CFR3 0x02
#define AD9910_ADDR_AUX_DAC_CONTROL 0x03
#define AD9910_ADDR_IO_UPDATE_RATE 0x04
#define AD9910_ADDR_FTW 0x07
#define AD9910_ADDR_POW 0x08
#define AD9910_ADDR_ASF 0x09
#define AD9910_ADDR_MULTICHIP_SYNC 0x0A
#define AD9910_ADDR_DIGITAL_RAMP_LIMIT 0x0B
#define AD9910_ADDR_DIGITAL_RAMP_STEP 0x0C
#define AD9910_ADDR_DIGITAL_RAMP_RATE 0x0D
#define AD9910_ADDR_PROFILE 0x0E
#define AD9910_ADDR_RAM 0x16

typedef enum {
  AD9910_drv0_output_disable = 0,
  AD9910_drv0_output_low = 1,
  AD9910_drv0_output_medium = 2,
  AD9910_drv0_output_high = 3
} AD9910_drv0_output;

typedef enum {
  AD9910_vco_range_setting_vco0 = 0,
  AD9910_vco_range_setting_vco1 = 1,
  AD9910_vco_range_setting_vco2 = 2,
  AD9910_vco_range_setting_vco3 = 3,
  AD9910_vco_range_setting_vco4 = 4,
  AD9910_vco_range_setting_vco5 = 5,
  AD9910_vco_range_setting_no_pll = 6
} AD9910_vco_range_setting;

typedef enum {
  AD9910_pump_current_212 = 0,
  AD9910_pump_current_237 = 1,
  AD9910_pump_current_262 = 2,
  AD9910_pump_current_287 = 3,
  AD9910_pump_current_312 = 4,
  AD9910_pump_current_337 = 5,
  AD9910_pump_current_363 = 6,
  AD9910_pump_current_387 = 7
} AD9910_pump_current;

typedef enum {
  AD9910_parallel_amplitude = 0x0,
  AD9910_parallel_phase = 0x1,
  AD9910_parallel_frequency = 0x2,
  AD9910_parallel_polar = 0x3
} AD9910_parallel_mode;

typedef enum {
  AD9910_ramp_dest_frequency = 0x0,
  AD9910_ramp_dest_phase = 0x1,
  AD9910_ramp_dest_amplitude = 0x2
} AD9910_ramp_destination;

typedef enum {
  AD9910_ram_dest_frequency = 0x0,
  AD9910_ram_dest_phase = 0x1,
  AD9910_ram_dest_amplitude = 0x2,
  AD9910_ram_dest_polar = 0x3
} AD9910_ram_destination;

typedef enum {
  AD9910_ram_ctl_direct_switch = 0x0,
  AD9910_ram_ctl_ramp_up = 0x1,
  AD9910_ram_ctl_bidirect_ramp = 0x2,
  AD9910_ram_ctl_cont_bidirect_ramp = 0x3,
  AD9910_ram_ctl_cont_recirculate = 0x4
} AD9910_ram_control;

#define PIN(name)                                                              \
  GPIO_TypeDef *name##_Port;                                                   \
  uint16_t name##_Pin
typedef struct {
  // PIN(PWR); // not used - grounded
  PIN(SDIO);
  // PIN(DPH); // not used - grounded
  // PIN(DRO); // not used - grounded
  PIN(IOUP);
  PIN(PF0);
  PIN(PF1);
  PIN(PF2);
  PIN(RST);
  PIN(SCK);
  // PIN(DRC); // not used - grounded
  // PIN(OSK);
  PIN(CSB);
} AD9910_Pins;
#undef PIN

#define AD9910_SYSCLK 1000000000     // 1 GHz
#define AD9910_MAX_AMP (1 << 14) - 1 // 14-bit amplitude
#define AD9910_RAM_SIZE 1024         // * 32 bits = 4 kB

void AD9910_Init(AD9910_Pins *pins);
void AD9910_InitConfig(AD9910_Config *cfg);
void AD9910_SetupClock(AD9910_Config *cfg);

void AD9910_WriteRegister(uint8_t reg, uint64_t data);
uint64_t AD9910_ReadRegister(uint8_t reg);
void AD9910_WriteRam(uint32_t *data, int count);
void AD9910_ReadRam(uint32_t *data, int count);
void AD9910_IOUpdate();
void AD9910_SelectProfile(uint8_t profile);

uint32_t AD9910_CalculateFTW(double freq);
uint16_t AD9910_CalculatePOW(double phase);

void AD9910_SetupSingleTone(AD9910_Config *cfg, double freq, double phase,
                            uint16_t amp);
void AD9910_SetupParallel(AD9910_Config *cfg, double freq, double phase,
                          uint16_t amp);
void AD9910_SetupSingleToneAM(AD9910_Config *cfg, double baseFreq,
                              double modFreq, double modDepth,
                              uint16_t fullAmp);
void AD9910_SetupSingleToneFM(AD9910_Config *config, double baseFreq,
                              double modFreq, double freqDev, uint16_t amp);