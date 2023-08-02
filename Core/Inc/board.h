#pragma once

#include "ad7606c.h"
#include "ad9959.h"
#include "si5351.h"
#include "lmx2572_legacy.h"
#include "keys.h"
#include "led.h"
#include "timers.h"

extern AD7606B_Config ad7606b_config;
extern AD7606B_Pins ad7606b_pins;
void BOARD_InitAD7606();

extern AD9959_Pins ad9959_pins;
extern AD9959_GlobalConfig ad9959_config;
extern AD9959_ChannelConfig ad9959_channel0, ad9959_channel1;
void BOARD_InitAD9959();

extern SWIIC_Config si5351_pins;
void BOARD_InitSI5351();    

void BOARD_InitLMX2572();