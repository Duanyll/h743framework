#pragma once
#include "main.h"

#if defined(AD7606B_ENABLE) || defined(AD7606C_ENABLE)

/* ------------------------------ AD7606B Pins ------------------------------ */
// AD_7606B with parallel interface and software mode
// AD_PAR_SET (Always LOW)
// AD_CONVST (Same as AD_CONVSTA in AD7606)
// AD_WR (Same as AD_CONVSTB in AD7606)
// AD_CS
// AD_RD
// AD_BUSY
// AD_RESET
// AD_OS0 - AD_OS2 (Always HIGH)
// AD_DB0 - AD_DB15 These pins are connected to the same GPIO port for fast
// reading. If they are not connected in order, edit the code in
// AD7606B_PinsToData() and AD7606B_DataToPins() to rearrange the bits into the
// correct order.
//
// Also enable TIM2 and call AD7606B_TimerCallback() in TIM2_IRQHandler().

/* ------------------------------ AD7606B Usage ----------------------------- */
// 1. Call AD7606B_Init() to initialize the AD7606B
// 2. Create a AD7606B_Config struct and init it with AD7606B_InitConfig()
// 3. Set the config values according to the datasheet. Write a single resister
// with AD7606B_ParallelRegisterWrite(), and read a single register with
// AD7606B_ParallelRegisterRead()
// 4. To set oversample, range, gain, offset, and phase, use crossponding
// functions.
// 5. To take one sample, call AD7606B_ADCConvert(). Then convert the output
// with AD7606B_PinsToData() to get real data.
// 6. To take multiple samples at specified sample rate, call
// AD7606B_CollectSamples().

typedef uint16_t (AD7606B_DataConvertFunction)(uint16_t data);

typedef struct {
  GPIO_TypeDef *CS_Port;
  uint16_t CS_Pin;
  GPIO_TypeDef *RD_Port;
  uint16_t RD_Pin;
  GPIO_TypeDef *BUSY_Port;
  uint16_t BUSY_Pin;
  GPIO_TypeDef *RESET_Port;
  uint16_t RESET_Pin;
  GPIO_TypeDef *CONVST_Port;
  uint16_t CONVST_Pin;
  GPIO_TypeDef *WR_Port;
  uint16_t WR_Pin;

  GPIO_TypeDef *DB_Port; // Whole port
  AD7606B_DataConvertFunction *DataToPins;
  AD7606B_DataConvertFunction *PinsToData;

  // Optional
  GPIO_TypeDef *OS0_Port;
  uint16_t OS0_Pin;
  GPIO_TypeDef *OS1_Port;
  uint16_t OS1_Pin;
  GPIO_TypeDef *OS2_Port;
  uint16_t OS2_Pin;
  GPIO_TypeDef *PAR_SEL_Port;
  uint16_t PAR_SEL_Pin;
} AD7606B_Pins;

#define AD7606B_REG_STATUS 0x01
#define AD7606B_REG_CONFIG 0x02
#define AD7606B_REG_RANGE 0x03
#ifdef AD7606C_ENABLE
#define AD7606C_REG_BANDWIDTH 0x07
#endif
#define AD7606B_REG_OVERSAMPLE 0x08
#define AD7606B_REG_GAIN 0x09
#define AD7606B_REG_OFFSET 0x11
#define AD7606B_REG_PHASE 0x19
#define AD7606B_REG_DIGITAL_DIAG_ENABLE 0x21
#define AD7606B_REG_DIGITAL_ERR 0x22
#define AD7606B_REG_OPEN_DETECT_ENABLE 0x23
#define AD7606B_REG_OPEN_DETECTED 0x24
#define AD7606B_REG_AIN_OV_UV_DIAG_ENABLE 0x25
#define AD7606B_REG_AIN_OV_DIAG_ERROR 0x26
#define AD7606B_REG_AIN_UV_DIAG_ERROR 0x27
#define AD7606B_REG_DIAG_MUX 0x28
#define AD7606B_REG_OPEN_DETECT_QUEUE 0x2c
#define AD7606B_REG_FS_CLK_COUNTER 0x2d
#define AD7606B_REG_OS_CLK_COUNTER 0x2e
#define AD7606B_REG_ID 0x2f

#define AD7606B_OPMODE_NORMAL 0x00
#define AD7606B_OPMODE_STANDBY 0x01
#define AD7606B_OPMODE_AUTOSTANDBY 0x02
#define AD7606B_OPMODE_SHUTDOWN 0x03

#ifndef AD7606C_ENABLE
#define AD7606B_RANGE_2V5 0x00
#define AD7606B_RANGE_5V 0x01
#define AD7606B_RANGE_10V 0x02
#else
#define AD7606C_RANGE_PM2V5 0x00
#define AD7606C_RANGE_PM5V 0x01
#define AD7606C_RANGE_PM6V25 0x02
#define AD7606C_RANGE_PM10V 0x03
#define AD7606C_RANGE_PM12V5 0x04
#define AD7606C_RANGE_5V 0x05
#define AD7606C_RANGE_10V 0x06
#define AD7606C_RANGE_12V5 0x07
#define AD7606C_RANGE_PM5V_DIFF 0x08
#define AD7606C_RANGE_PM10V_DIFF 0x09
#define AD7606C_RANGE_PM12V5_DIFF 0x0A
#define AD7606C_RANGE_PM20V_DIFF 0x0B
#endif

#define AD7606B_OVERSAMPLE_NO 0x00
#define AD7606B_OVERSAMPLE_2 0x01
#define AD7606B_OVERSAMPLE_4 0x02
#define AD7606B_OVERSAMPLE_8 0x03
#define AD7606B_OVERSAMPLE_16 0x04
#define AD7606B_OVERSAMPLE_32 0x05
#define AD7606B_OVERSAMPLE_64 0x06
#define AD7606B_OVERSAMPLE_128 0x07
#define AD7606B_OVERSAMPLE_256 0x08

#define AD7606B_DIAG_MUX_AIN 0x00
#define AD7606B_DIAG_MUX_TEMP 0x01
#define AD7606B_DIAG_MUX_2V5REF 0x02
#define AD7606B_DIAG_MUX_ALDO_1 0x03
#define AD7606B_DIAG_MUX_ALDO_2 0x04
#define AD7606B_DIAG_MUX_VDRIVE 0x05
#define AD7606B_DIAG_MUX_AGND 0x06
#define AD7606B_DIAG_MUX_AVCC 0x07

typedef struct {
  uint8_t raw[0x2f + 1];
} AD7606B_Config;

// Init the AD7606B
void AD7606B_Init(AD7606B_Pins *p);
// Full reset the AD7606B
void AD7606B_FullReset(void);
// Read from a single register
uint8_t AD7606B_ParallelRegisterRead(uint8_t addr);
// Write to a single register
void AD7606B_ParallelRegisterWrite(uint8_t addr, uint8_t data);
// Leave register mode and enter ADC mode. This is required before taking
// samples
void AD7606B_LeaveRegisterMode(void);
// Take one sample from AD7606B, manually call AD7606B_PinsToData to get real
// data
void AD7606B_ADCConvert(uint16_t *data, uint8_t channels);
// Take multiple samples at given sample rate in blocking mode
BOOL AD7606B_CollectSamples(int16_t *data, uint8_t channels, uint32_t count,
                            double sampleRate);
// Start continuous convert mode, call AD7606B_StopContinuousConvert to stop.
// Recieve data in callback function asynchronously. LED1 indicates if the
// callback takes too long to process at given sample rate.
void AD7606B_StartContinuousConvert(double sampleRate, uint8_t channels,
                                    void (*callback)(int16_t *data));
// Stop continuous convert mode
void AD7606B_StopContinuousConvert(void);
void AD7606B_TimerCallback();

void AD7606B_InitConfig(AD7606B_Config *config);
void AD7606B_SetRange(AD7606B_Config *config, uint8_t channel, uint8_t range);
void AD7606B_SetOverSample(AD7606B_Config *config, uint8_t oversample_padding,
                           uint8_t oversample);
void AD7606B_SetGain(AD7606B_Config *config, uint8_t channel, uint8_t gain);
void AD7606B_SetOffset(AD7606B_Config *config, uint8_t channel,
                       uint16_t offset);
void AD7606B_SetPhase(AD7606B_Config *config, uint8_t channel, uint8_t phase);
void AD7606B_SetDiagMux(AD7606B_Config *config, uint8_t channel,
                        uint8_t diag_mux);

#endif