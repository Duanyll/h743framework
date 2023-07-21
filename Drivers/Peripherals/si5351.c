/*
MIT License

Copyright (c) 2020 Aleksander Alekseev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifdef SI5351_ENABLE

#include "si5351.h"

// Private procedures.
void SI5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3,
                      uint8_t divBy4, SI5351_RDiv_t rdiv);
void SI5351_write(uint8_t reg, uint8_t value);

// See http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
enum {
  SI5351_REGISTER_0_DEVICE_STATUS = 0,
  SI5351_REGISTER_1_INTERRUPT_STATUS_STICKY = 1,
  SI5351_REGISTER_2_INTERRUPT_STATUS_MASK = 2,
  SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL = 3,
  SI5351_REGISTER_9_OEB_PIN_ENABLE_CONTROL = 9,
  SI5351_REGISTER_15_PLL_INPUT_SOURCE = 15,
  SI5351_REGISTER_16_CLK0_CONTROL = 16,
  SI5351_REGISTER_17_CLK1_CONTROL = 17,
  SI5351_REGISTER_18_CLK2_CONTROL = 18,
  SI5351_REGISTER_19_CLK3_CONTROL = 19,
  SI5351_REGISTER_20_CLK4_CONTROL = 20,
  SI5351_REGISTER_21_CLK5_CONTROL = 21,
  SI5351_REGISTER_22_CLK6_CONTROL = 22,
  SI5351_REGISTER_23_CLK7_CONTROL = 23,
  SI5351_REGISTER_24_CLK3_0_DISABLE_STATE = 24,
  SI5351_REGISTER_25_CLK7_4_DISABLE_STATE = 25,
  SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1 = 42,
  SI5351_REGISTER_43_MULTISYNTH0_PARAMETERS_2 = 43,
  SI5351_REGISTER_44_MULTISYNTH0_PARAMETERS_3 = 44,
  SI5351_REGISTER_45_MULTISYNTH0_PARAMETERS_4 = 45,
  SI5351_REGISTER_46_MULTISYNTH0_PARAMETERS_5 = 46,
  SI5351_REGISTER_47_MULTISYNTH0_PARAMETERS_6 = 47,
  SI5351_REGISTER_48_MULTISYNTH0_PARAMETERS_7 = 48,
  SI5351_REGISTER_49_MULTISYNTH0_PARAMETERS_8 = 49,
  SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1 = 50,
  SI5351_REGISTER_51_MULTISYNTH1_PARAMETERS_2 = 51,
  SI5351_REGISTER_52_MULTISYNTH1_PARAMETERS_3 = 52,
  SI5351_REGISTER_53_MULTISYNTH1_PARAMETERS_4 = 53,
  SI5351_REGISTER_54_MULTISYNTH1_PARAMETERS_5 = 54,
  SI5351_REGISTER_55_MULTISYNTH1_PARAMETERS_6 = 55,
  SI5351_REGISTER_56_MULTISYNTH1_PARAMETERS_7 = 56,
  SI5351_REGISTER_57_MULTISYNTH1_PARAMETERS_8 = 57,
  SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1 = 58,
  SI5351_REGISTER_59_MULTISYNTH2_PARAMETERS_2 = 59,
  SI5351_REGISTER_60_MULTISYNTH2_PARAMETERS_3 = 60,
  SI5351_REGISTER_61_MULTISYNTH2_PARAMETERS_4 = 61,
  SI5351_REGISTER_62_MULTISYNTH2_PARAMETERS_5 = 62,
  SI5351_REGISTER_63_MULTISYNTH2_PARAMETERS_6 = 63,
  SI5351_REGISTER_64_MULTISYNTH2_PARAMETERS_7 = 64,
  SI5351_REGISTER_65_MULTISYNTH2_PARAMETERS_8 = 65,
  SI5351_REGISTER_66_MULTISYNTH3_PARAMETERS_1 = 66,
  SI5351_REGISTER_67_MULTISYNTH3_PARAMETERS_2 = 67,
  SI5351_REGISTER_68_MULTISYNTH3_PARAMETERS_3 = 68,
  SI5351_REGISTER_69_MULTISYNTH3_PARAMETERS_4 = 69,
  SI5351_REGISTER_70_MULTISYNTH3_PARAMETERS_5 = 70,
  SI5351_REGISTER_71_MULTISYNTH3_PARAMETERS_6 = 71,
  SI5351_REGISTER_72_MULTISYNTH3_PARAMETERS_7 = 72,
  SI5351_REGISTER_73_MULTISYNTH3_PARAMETERS_8 = 73,
  SI5351_REGISTER_74_MULTISYNTH4_PARAMETERS_1 = 74,
  SI5351_REGISTER_75_MULTISYNTH4_PARAMETERS_2 = 75,
  SI5351_REGISTER_76_MULTISYNTH4_PARAMETERS_3 = 76,
  SI5351_REGISTER_77_MULTISYNTH4_PARAMETERS_4 = 77,
  SI5351_REGISTER_78_MULTISYNTH4_PARAMETERS_5 = 78,
  SI5351_REGISTER_79_MULTISYNTH4_PARAMETERS_6 = 79,
  SI5351_REGISTER_80_MULTISYNTH4_PARAMETERS_7 = 80,
  SI5351_REGISTER_81_MULTISYNTH4_PARAMETERS_8 = 81,
  SI5351_REGISTER_82_MULTISYNTH5_PARAMETERS_1 = 82,
  SI5351_REGISTER_83_MULTISYNTH5_PARAMETERS_2 = 83,
  SI5351_REGISTER_84_MULTISYNTH5_PARAMETERS_3 = 84,
  SI5351_REGISTER_85_MULTISYNTH5_PARAMETERS_4 = 85,
  SI5351_REGISTER_86_MULTISYNTH5_PARAMETERS_5 = 86,
  SI5351_REGISTER_87_MULTISYNTH5_PARAMETERS_6 = 87,
  SI5351_REGISTER_88_MULTISYNTH5_PARAMETERS_7 = 88,
  SI5351_REGISTER_89_MULTISYNTH5_PARAMETERS_8 = 89,
  SI5351_REGISTER_90_MULTISYNTH6_PARAMETERS = 90,
  SI5351_REGISTER_91_MULTISYNTH7_PARAMETERS = 91,
  SI5351_REGISTER_92_CLOCK_6_7_OUTPUT_DIVIDER = 92,
  SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET = 165,
  SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET = 166,
  SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET = 167,
  SI5351_REGISTER_168_CLK3_INITIAL_PHASE_OFFSET = 168,
  SI5351_REGISTER_169_CLK4_INITIAL_PHASE_OFFSET = 169,
  SI5351_REGISTER_170_CLK5_INITIAL_PHASE_OFFSET = 170,
  SI5351_REGISTER_177_PLL_RESET = 177,
  SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE = 183
};

typedef enum {
  SI5351_CRYSTAL_LOAD_6PF = (1 << 6),
  SI5351_CRYSTAL_LOAD_8PF = (2 << 6),
  SI5351_CRYSTAL_LOAD_10PF = (3 << 6)
} SI5351_CrystalLoad_t;

int32_t SI5351_Correction;
SWIIC_Config *SI5351_SWIIC;

/*
 * Initializes Si5351. Call this function before doing anything else.
 * `Correction` is the difference of actual frequency an desired frequency @ 100
 * MHz. It can be measured at lower frequencies and scaled linearly. E.g. if you
 * get 10_000_097 Hz instead of 10_000_000 Hz, `correction` is 97*10 = 970
 */
void SI5351_Init(SWIIC_Config *swiic, int32_t correction) {
  SI5351_SWIIC = swiic;
  SI5351_Correction = correction;

  // Disable all outputs by setting CLKx_DIS high
  SI5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, 0xFF);

  // Power down all output drivers
  SI5351_write(SI5351_REGISTER_16_CLK0_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_17_CLK1_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_18_CLK2_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_19_CLK3_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_20_CLK4_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_21_CLK5_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_22_CLK6_CONTROL, 0x80);
  SI5351_write(SI5351_REGISTER_23_CLK7_CONTROL, 0x80);

  // Set the load capacitance for the XTAL
  SI5351_CrystalLoad_t crystalLoad = SI5351_CRYSTAL_LOAD_10PF;
  SI5351_write(SI5351_REGISTER_183_CRYSTAL_INTERNAL_LOAD_CAPACITANCE,
               crystalLoad);
}

// Sets the multiplier for given PLL
void SI5351_SetupPLL(SI5351_PLL_t pll, SI5351_PLLConfig_t *conf) {
  int32_t P1, P2, P3;
  int32_t mult = conf->mult;
  int32_t num = conf->num;
  int32_t denom = conf->denom;

  P1 = 128 * mult + (128 * num) / denom - 512;
  // P2 = 128 * num - denom * ((128 * num)/denom);
  P2 = (128 * num) % denom;
  P3 = denom;

  // Get the appropriate base address for the PLL registers
  uint8_t baseaddr = (pll == SI5351_PLL_A ? 26 : 34);
  SI5351_writeBulk(baseaddr, P1, P2, P3, 0, 0);

  // Reset both PLLs
  SI5351_write(SI5351_REGISTER_177_PLL_RESET, (1 << 7) | (1 << 5));
}

// Configures PLL source, drive strength, multisynth divider, Rdivider and
// phaseOffset. Returns 0 on success, != 0 otherwise.
int SI5351_SetupOutput(uint8_t output, SI5351_PLL_t pllSource,
                       SI5351_DriveStrength_t driveStrength,
                       SI5351_OutputConfig_t *conf, uint8_t phaseOffset) {
  int32_t div = conf->div;
  int32_t num = conf->num;
  int32_t denom = conf->denom;
  uint8_t divBy4 = 0;
  int32_t P1, P2, P3;

  if (output > 2) {
    return 1;
  }

  if ((!conf->allowIntegerMode) && ((div < 8) || ((div == 8) && (num == 0)))) {
    // div in { 4, 6, 8 } is possible only in integer mode
    return 2;
  }

  if (div == 4) {
    // special DIVBY4 case, see AN619 4.1.3
    P1 = 0;
    P2 = 0;
    P3 = 1;
    divBy4 = 0x3;
  } else {
    P1 = 128 * div + ((128 * num) / denom) - 512;
    // P2 = 128 * num - denom * (128 * num)/denom;
    P2 = (128 * num) % denom;
    P3 = denom;
  }

  // Get the register addresses for given channel
  uint8_t baseaddr = 0;
  uint8_t phaseOffsetRegister = 0;
  uint8_t clkControlRegister = 0;
  switch (output) {
  case 0:
    baseaddr = SI5351_REGISTER_42_MULTISYNTH0_PARAMETERS_1;
    phaseOffsetRegister = SI5351_REGISTER_165_CLK0_INITIAL_PHASE_OFFSET;
    clkControlRegister = SI5351_REGISTER_16_CLK0_CONTROL;
    break;
  case 1:
    baseaddr = SI5351_REGISTER_50_MULTISYNTH1_PARAMETERS_1;
    phaseOffsetRegister = SI5351_REGISTER_166_CLK1_INITIAL_PHASE_OFFSET;
    clkControlRegister = SI5351_REGISTER_17_CLK1_CONTROL;
    break;
  case 2:
    baseaddr = SI5351_REGISTER_58_MULTISYNTH2_PARAMETERS_1;
    phaseOffsetRegister = SI5351_REGISTER_167_CLK2_INITIAL_PHASE_OFFSET;
    clkControlRegister = SI5351_REGISTER_18_CLK2_CONTROL;
    break;
  }

  uint8_t clkControl = 0x0C | driveStrength; // clock not inverted, powered up
  if (pllSource == SI5351_PLL_B) {
    clkControl |= (1 << 5); // Uses PLLB
  }

  if ((conf->allowIntegerMode) && ((num == 0) || (div == 4))) {
    // use integer mode
    clkControl |= (1 << 6);
  }

  SI5351_write(clkControlRegister, clkControl);
  SI5351_writeBulk(baseaddr, P1, P2, P3, divBy4, conf->rdiv);
  SI5351_write(phaseOffsetRegister, (phaseOffset & 0x7F));

  return 0;
}

// Calculates PLL, MS and RDiv settings for given Fclk in [8_000, 160_000_000]
// range. The actual frequency will differ less than 6 Hz from given Fclk,
// assuming `correction` is right.
void SI5351_Calc(int32_t Fclk, SI5351_PLLConfig_t *pll_conf,
                 SI5351_OutputConfig_t *out_conf) {
  if (Fclk < 8000)
    Fclk = 8000;
  else if (Fclk > 160000000)
    Fclk = 160000000;

  out_conf->allowIntegerMode = 1;

  if (Fclk < 1000000) {
    // For frequencies in [8_000, 500_000] range we can use SI5351_Calc(Fclk*64,
    // ...) and SI5351_R_DIV_64. In practice it's worth doing for any frequency
    // below 1 MHz, since it reduces the error.
    Fclk *= 64;
    out_conf->rdiv = SI5351_R_DIV_64;
  } else {
    out_conf->rdiv = SI5351_R_DIV_1;
  }

  // Apply correction, _after_ determining rdiv.
  Fclk = Fclk - ((Fclk / 1000000) * SI5351_Correction) / 100;

  // Here we are looking for integer values of a,b,c,x,y,z such as:
  // N = a + b / c    # pll settings
  // M = x + y / z    # ms  settings
  // Fclk = Fxtal * N / M
  // N in [24, 36]
  // M in [8, 1800] or M in {4,6}
  // b < c, y < z
  // b,c,y,z <= 2**20
  // c, z != 0
  // For any Fclk in [500K, 160MHz] this algorithm finds a solution
  // such as abs(Ffound - Fclk) <= 6 Hz

  const int32_t Fxtal = 25000000;
  int32_t a, b, c, x, y, z, t;

  if (Fclk < 81000000) {
    // Valid for Fclk in 0.5..112.5 MHz range
    // However an error is > 6 Hz above 81 MHz
    a = 36; // PLL runs @ 900 MHz
    b = 0;
    c = 1;
    int32_t Fpll = 900000000;
    x = Fpll / Fclk;
    t = (Fclk >> 20) + 1;
    y = (Fpll % Fclk) / t;
    z = Fclk / t;
  } else {
    // Valid for Fclk in 75..160 MHz range
    if (Fclk >= 150000000) {
      x = 4;
    } else if (Fclk >= 100000000) {
      x = 6;
    } else {
      x = 8;
    }
    y = 0;
    z = 1;

    int32_t numerator = x * Fclk;
    a = numerator / Fxtal;
    t = (Fxtal >> 20) + 1;
    b = (numerator % Fxtal) / t;
    c = Fxtal / t;
  }

  pll_conf->mult = a;
  pll_conf->num = b;
  pll_conf->denom = c;
  out_conf->div = x;
  out_conf->num = y;
  out_conf->denom = z;
}

// SI5351_CalcIQ() finds PLL and MS parameters that give phase shift 90° between
// two channels, if 0 and (uint8_t)out_conf.div are passed as phaseOffset for
// these channels. Channels should use the same PLL to make it work. Fclk can be
// from 1.4 MHz to 100 MHz. The actual frequency will differ less than 4 Hz from
// given Fclk, assuming `correction` is right.
void SI5351_CalcIQ(int32_t Fclk, SI5351_PLLConfig_t *pll_conf,
                   SI5351_OutputConfig_t *out_conf) {
  const int32_t Fxtal = 25000000;
  int32_t Fpll;

  if (Fclk < 1400000)
    Fclk = 1400000;
  else if (Fclk > 100000000)
    Fclk = 100000000;

  // apply correction
  Fclk = Fclk - ((Fclk / 1000000) * SI5351_Correction) / 100;

  // disable integer mode
  out_conf->allowIntegerMode = 0;

  // Using RDivider's changes the phase shift and AN619 doesn't give any
  // guarantees regarding this change.
  out_conf->rdiv = 0;

  if (Fclk < 4900000) {
    // Little hack, run PLL below 600 MHz to cover 1.4 MHz .. 4.725 MHz range.
    // AN619 doesn't literally say that PLL can't run below 600 MHz.
    // Experiments showed that PLL gets unstable when you run it below 177 MHz,
    // which limits Fclk to 177 / 127 = 1.4 MHz.
    out_conf->div = 127;
  } else if (Fclk < 8000000) {
    out_conf->div = 625000000 / Fclk;
  } else {
    out_conf->div = 900000000 / Fclk;
  }
  out_conf->num = 0;
  out_conf->denom = 1;

  Fpll = Fclk * out_conf->div;
  pll_conf->mult = Fpll / Fxtal;
  pll_conf->num = (Fpll % Fxtal) / 24;
  pll_conf->denom = Fxtal / 24; // denom can't exceed 0xFFFFF
}

// Setup CLK0 for given frequency and drive strength. Use PLLA.
void SI5351_SetupCLK0(int32_t Fclk, SI5351_DriveStrength_t driveStrength) {
  SI5351_PLLConfig_t pll_conf;
  SI5351_OutputConfig_t out_conf;

  SI5351_Calc(Fclk, &pll_conf, &out_conf);
  SI5351_SetupPLL(SI5351_PLL_A, &pll_conf);
  SI5351_SetupOutput(0, SI5351_PLL_A, driveStrength, &out_conf, 0);
}

// Setup CLK2 for given frequency and drive strength. Use PLLB.
void SI5351_SetupCLK2(int32_t Fclk, SI5351_DriveStrength_t driveStrength) {
  SI5351_PLLConfig_t pll_conf;
  SI5351_OutputConfig_t out_conf;

  SI5351_Calc(Fclk, &pll_conf, &out_conf);
  SI5351_SetupPLL(SI5351_PLL_B, &pll_conf);
  SI5351_SetupOutput(2, SI5351_PLL_B, driveStrength, &out_conf, 0);
}

// Enables or disables outputs depending on provided bitmask.
// Examples:
// SI5351_EnableOutputs(1 << 0) enables CLK0 and disables CLK1 and CLK2
// SI5351_EnableOutputs((1 << 2) | (1 << 0)) enables CLK0 and CLK2 and disables
// CLK1
void SI5351_EnableOutputs(uint8_t enabled) {
  SI5351_write(SI5351_REGISTER_3_OUTPUT_ENABLE_CONTROL, ~enabled);
}

// Writes an 8 bit value of a register over I2C.
void SI5351_write(uint8_t reg, uint8_t value) {
  SWIIC_WriteBytes8(SI5351_SWIIC, SI5351_ADDR, reg, &value, 1);
}

// Common code for _SetupPLL and _SetupOutput
void SI5351_writeBulk(uint8_t baseaddr, int32_t P1, int32_t P2, int32_t P3,
                      uint8_t divBy4, SI5351_RDiv_t rdiv) {
  SI5351_write(baseaddr, (P3 >> 8) & 0xFF);
  SI5351_write(baseaddr + 1, P3 & 0xFF);
  SI5351_write(baseaddr + 2, ((P1 >> 16) & 0x3) | ((divBy4 & 0x3) << 2) |
                                 ((rdiv & 0x7) << 4));
  SI5351_write(baseaddr + 3, (P1 >> 8) & 0xFF);
  SI5351_write(baseaddr + 4, P1 & 0xFF);
  SI5351_write(baseaddr + 5, ((P3 >> 12) & 0xF0) | ((P2 >> 16) & 0xF));
  SI5351_write(baseaddr + 6, (P2 >> 8) & 0xFF);
  SI5351_write(baseaddr + 7, P2 & 0xFF);
}

#endif