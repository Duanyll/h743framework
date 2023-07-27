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

#pragma once

#include "common.h"

#include "swiic.h"

typedef enum {
  SI5351_PLL_A = 0,
  SI5351_PLL_B,
} SI5351_PLL_t;

typedef enum {
  SI5351_R_DIV_1 = 0,
  SI5351_R_DIV_2 = 1,
  SI5351_R_DIV_4 = 2,
  SI5351_R_DIV_8 = 3,
  SI5351_R_DIV_16 = 4,
  SI5351_R_DIV_32 = 5,
  SI5351_R_DIV_64 = 6,
  SI5351_R_DIV_128 = 7,
} SI5351_RDiv_t;

typedef enum {
  SI5351_DRIVE_STRENGTH_2MA = 0x00, //  ~ 2.2 dBm
  SI5351_DRIVE_STRENGTH_4MA = 0x01, //  ~ 7.5 dBm
  SI5351_DRIVE_STRENGTH_6MA = 0x02, //  ~ 9.5 dBm
  SI5351_DRIVE_STRENGTH_8MA = 0x03, // ~ 10.7 dBm
} SI5351_DriveStrength_t;

typedef struct {
  int32_t mult;
  int32_t num;
  int32_t denom;
} SI5351_PLLConfig_t;

typedef struct {
  uint8_t allowIntegerMode;
  int32_t div;
  int32_t num;
  int32_t denom;
  SI5351_RDiv_t rdiv;
} SI5351_OutputConfig_t;

#define SI5351_ADDR 0x60

/*
 * Basic interface allows to use only CLK0 and CLK2.
 * This interface uses separate PLLs for both CLK0 and CLK2 thus the frequencies
 * can be changed independently. If you also need CLK1 one PLL should
 * be shared between two CLKx and things get a little more complicated.
 * CLK0 and CLK2 were chosen because they are distant from each other on a
 * common Si5351 module. This makes using them a little more convenient than
 * CLK0 and CLK1.
 */
void SI5351_Init(SWIIC_Config* swiic, int32_t correction);
void SI5351_SetupCLK0(int32_t Fclk, SI5351_DriveStrength_t driveStrength);
void SI5351_SetupCLK2(int32_t Fclk, SI5351_DriveStrength_t driveStrength);
void SI5351_EnableOutputs(uint8_t enabled);

/*
 * Advanced interface. Use it if you need:
 *
 * a. CLK0, CLK1 and CLK2 simultaneously;
 * b. A phase shift 90° between two channels;
 *
 * SI5351_Calc() always uses 900 MHz PLL for frequencies below 81 MHz.
 * This PLL can safely be shared between all CLKx that work @ <= 81 MHz.
 * You can also modify si5351.c to share one PLL for any frequencies <= 112.5
 * MHz, however this will increase the worse case calculation error to 13 Hz.
 */
void SI5351_Calc(int32_t Fclk, SI5351_PLLConfig_t *pll_conf,
                 SI5351_OutputConfig_t *out_conf);

/*
 * SI5351_CalcIQ() finds PLL and MS parameters that give phase shift 90° between
 * two channels, if 0 and (uint8_t)out_conf.div are passed as phaseOffset for
 * these channels. Channels should use the same PLL to make it work.
 */
void SI5351_CalcIQ(int32_t Fclk, SI5351_PLLConfig_t *pll_conf,
                   SI5351_OutputConfig_t *out_conf);

void SI5351_SetupPLL(SI5351_PLL_t pll, SI5351_PLLConfig_t *conf);
int SI5351_SetupOutput(uint8_t output, SI5351_PLL_t pllSource,
                       SI5351_DriveStrength_t driveStength,
                       SI5351_OutputConfig_t *conf, uint8_t phaseOffset);