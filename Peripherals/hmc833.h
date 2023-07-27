#pragma once

#include "common.h"

typedef struct {
  GPIO_TypeDef *LD_Port;
  uint16_t LD_Pin;
  GPIO_TypeDef *SCK_Port;
  uint16_t SCK_Pin;
  GPIO_TypeDef *SDI_Port;
  uint16_t SDI_Pin;
  GPIO_TypeDef *SEN_Port;
  uint16_t SEN_Pin;
  GPIO_TypeDef *CE_Port;
  uint16_t CE_Pin;
} HMC833_Pins;

// Only common registers are defined, refer to datasheet for full register list

typedef union {
  struct {
    uint32_t seed : 2;
    uint32_t order : 2;
    uint32_t reserved_1 : 3; // Always 4d
    uint32_t frac_bypass : 1;
    uint32_t auto_seed : 1;
    uint32_t clkrq_refdiv_sel : 1;
    uint32_t sd_modulator_clk_select : 1; // Program 1
    uint32_t sd_enable : 1;
    uint32_t reserved_2 : 4;
    uint32_t reserved_3 : 2; // Program 3d
    uint32_t bist_enable : 1;
    uint32_t rdiv_bist_cycles : 2;
    uint32_t auto_clock_ocnfig : 1; // Program 0
    uint32_t reserved_4 : 10;
  };
  uint32_t raw;
} HMC833_Reg_SD_CFG;

typedef union {
  struct {
    uint32_t cp_dn_gain : 7;
    uint32_t cp_up_gain : 7;
    uint32_t offset_magnitude : 7;
    uint32_t offser_up_enable : 1;
    uint32_t hikcp : 1;
    uint32_t reserved_1 : 9;
  };
  uint32_t raw;
} HMC833_Reg_ChargePump;

typedef union {
  struct {
    uint32_t id : 3;
    uint32_t addr : 4;
    uint32_t data : 9;
    uint32_t reserved_1 : 16;
  };
  uint32_t raw;
} HMC833_Reg_VCO;

#define HMC833_ADDR_VERSION 0x00
#define HMC833_ADDR_REFDIV 0x02
#define HMC833_ADDR_INTEGER 0x03
#define HMC833_ADDR_FRACTIONAL 0x04
#define HMC833_ADDR_VCO 0x05
#define HMC833_ADDR_SD_CFG 0x06
#define HMC833_ADDR_CHARGE_PUMP 0x09

typedef union {
  struct {
    uint16_t cal : 1;
    uint16_t caps : 8;
    uint16_t reserved_1 : 7;
  };
  uint16_t raw;
} HMC833_VCOReg_Tuning;

typedef union {
  struct {
    uint16_t master_enable : 1;
    uint16_t pll_buffer_enable : 1;
    uint16_t rf_buffer_enable : 1;
    uint16_t divide_by_one_enable : 1;
    uint16_t rf_divider_enable : 1;
    uint16_t reserved_1 : 11;
  };
  uint16_t raw;
} HMC833_VCOReg_Enables;

typedef union {
  struct {
    uint16_t rf_divide_ratio : 6;
    uint16_t rf_output_buffer_gain_control : 2;
    uint16_t divider_output_stage_gain_control : 1;
    uint16_t reserved_1 : 7;
  };
  uint16_t raw;
} HMC833_VCOReg_Biases;

typedef union {
  struct {
    uint16_t fundamental_doubler_selection : 1;
    uint16_t reserved_1 : 1;
    uint16_t manual_rfo_mode : 1;
    uint16_t rf_buffer_bias : 2;
    uint16_t reserved_2 : 11;
  };
  uint16_t raw;
} HMC833_VCOReg_Config;

#define HMC833_VCO_ADDR_TUNING 0x00
#define HMC833_VCO_ADDR_ENABLES 0x01
#define HMC833_VCO_ADDR_BIASES 0x02
#define HMC833_VCO_ADDR_CONFIG 0x03

typedef struct {
  uint16_t refdiv;     // 14 bits
  uint32_t integer;    // 19 bits
  uint32_t fractional; // 28 bits
  HMC833_Reg_SD_CFG sd_cfg;
  HMC833_Reg_ChargePump charge_pump;
  struct {
    HMC833_VCOReg_Tuning tuning;
    HMC833_VCOReg_Enables enables;
    HMC833_VCOReg_Biases biases;
    HMC833_VCOReg_Config config;
  } vco;
} HMC833_Config;

void HMC833_Init(HMC833_Pins *pins);
void HMC833_InitConfig(HMC833_Config *config);
// Simply sets the frequency. AutoCal will configure the rest of the registers
// NOT WORKING, CANNOT SET CORRECT FREQUENCY
void HMC833_SetFrequency(HMC833_Config *config, double fOUT);

#define HMC833_FXTAL 50e6