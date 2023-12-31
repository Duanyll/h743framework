#pragma once

#include "common.h"

/* ------------------------------- ADF4351 Pins ------------------------------
 */
// PLL_CLK
// PLL_DAT
// PLL_LE
// PLL_CE (May be tied to HIGH)

/* ------------------------------ ADF4351 Usage ------------------------------
 */
// 1. Call ADF4351_Init() to init device
// 2. Create ADF4351_Config structure and init it with ADF4351_InitConfig()
// 3. Edit control registers in ADF4351_Config structure.
// 4. Set frequency with ADF4351_SetFrequency() function.
// 5. Call ADF4351_WriteConfig() to apply changes.

typedef struct {
  GPIO_TypeDef *CLK_Port;
  uint16_t CLK_Pin;
  GPIO_TypeDef *DAT_Port;
  uint16_t DAT_Pin;
  GPIO_TypeDef *LE_Port;
  uint16_t LE_Pin;
  GPIO_TypeDef *CE_Port;
  uint16_t CE_Pin;
} ADF4351_Pins;

#define ADF5451_PFD_MAX 32.0e6
#define ADF4351_RFOUT_MAX 4400.0e6
#define ADF4351_RFOUTMIN 34.375e6
#define ADF4351_REFINMAX 250.0e6
#define ADF4351_REFIN 100.0e6

/** \brief  Union type for the structure of Register0 in ADF4351
 */
typedef union {
  struct {
    uint32_t ControlBits : 3; /*!< bit:  0.. 2 	CONTROL BITS */
    uint32_t FracVal : 12;    /*!< bit:  3..14 	12-BIT FRACTIONAL VALUE
                                 (FRAC) */
    uint32_t IntVal : 16;     /*!< bit: 15..30 	16- BIT INTEGER VALUE (INT) */
    uint32_t _reserved_0 : 1; /*!< bit: 31     	RESERVED */
  };
  uint32_t w;
} ADF4351_Reg0_t;

/** \brief  Union type for the structure of Register1 in ADF4351
 */
typedef union {
  struct {
    uint32_t ControlBits : 3; /*!< bit:  0.. 2 	CONTROL BITS */
    uint32_t ModVal : 12;     /*!< bit:  3..14 	12-BIT MODULUS VALUE (MOD) */
    uint32_t PhaseVal : 12;   /*!< bit: 15..26 	12-BIT PHASE VALUE
                                 (PHASE) */
    uint32_t Prescaler : 1;   /*!< bit:  27		 	PRESCALER */
    uint32_t PhaseAdjust : 1; /*!< bit:  28		 	PHASE ADJUST */
    uint32_t _reserved_0 : 3; /*!< bit: 29..31 	RESERVED */
  };
  uint32_t w;
} ADF4351_Reg1_t;

/** \brief  Union type for the structure of Register2 in ADF4351
 */
typedef union {
  struct {
    uint32_t ControlBits : 3;   /*!< bit:  0.. 2 	CONTROL BITS */
    uint32_t CounterReset : 1;  /*!< bit:  3 		 	Counter Reset */
    uint32_t CPTristate : 1;    /*!< bit:  4 		 	Charge Pump
                                 * Three-State
                                 */
    uint32_t PowerDown : 1;     /*!< bit:  5 		 	Power-Down */
    uint32_t PhasePolarity : 1; /*!< bit:  6 		 	Phase
                                   Detector Polarity */
    uint32_t LockPrecision : 1; /*!< bit:  7 		 	Lock
                                   Detect Precision */
    uint32_t
        LockFunction : 1;      /*!< bit:  8 		 	Lock Detect Function
                                */
    uint32_t CPCurrent : 4;    /*!< bit:  9..12 	Charge Pump Current Setting */
    uint32_t DoubleBuffer : 1; /*!< bit:  13		 	Double Buffer */
    uint32_t RCountVal : 10;   /*!< bit: 14..23 	10-Bit R Counter */
    uint32_t RDiv2 : 1;        /*!< bit: 24		 	Double Buffer */
    uint32_t RMul2 : 1;        /*!< bit: 25		 	Double Buffer */
    uint32_t MuxOut : 3;       /*!< bit: 26..28 	MUXOUT */
    uint32_t LowNoiseSpur : 2; /*!< bit: 29..30	Low Noise and Low Spur Modes  */
    uint32_t _reserved_0 : 1;  /*!< bit: 31			RESERVED  */
  };
  uint32_t w;
} ADF4351_Reg2_t;

/** \brief  Union type for the structure of Register3 in ADF4351
 */
typedef union {
  struct {
    uint32_t ControlBits : 3;   /*!< bit:  0.. 2 	CONTROL BITS */
    uint32_t ClkDivVal : 12;    /*!< bit:  3..14 	12-Bit Clock Divider Value */
    uint32_t ClkDivMod : 2;     /*!< bit:  15..16 Clock Divider Mode */
    uint32_t _reserved_0 : 1;   /*!< bit:  17		 	RESERVED */
    uint32_t CsrEn : 1;         /*!< bit:  18		 	CSR Enable */
    uint32_t _reserved_1 : 2;   /*!< bit:  19..20	RESERVED */
    uint32_t ChargeCh : 1;      /*!< bit:  21     Charge Cancelation */
    uint32_t AntibacklashW : 1; /*!< bit:  22     Antibacklash Pulse Width */
    uint32_t BandSelMode : 1;   /*!< bit:  23     Band Select Clock Mode */
    uint32_t _reserved_2 : 8;   /*!< bit:  24..31	RESERVED */
  };
  uint32_t w;
} ADF4351_Reg3_t;

/** \brief  Union type for the structure of Register4 in ADF4351
 */
typedef union {
  struct {
    uint32_t ControlBits : 3;  /*!< bit:  0.. 2 	CONTROL BITS */
    uint32_t OutPower : 2;     /*!< bit:  3.. 4 	Output Power */
    uint32_t OutEnable : 1;    /*!< bit:  5 	RF Output Enable */
    uint32_t AuxPower : 2;     /*!< bit:  6.. 7 	AUX Output Power */
    uint32_t AuxEnable : 1;    /*!< bit:  8 	AUX Output Enable */
    uint32_t AuxSel : 1;       /*!< bit:  9 	AUX Output Select */
    uint32_t Mtld : 1;         /*!< bit: 10 	Mute Till Lock Detect (MTLD) */
    uint32_t VcoPowerDown : 1; /*!< bit: 11 	VCO Power-Down */
    uint32_t BandClkDiv : 8;   /*!< bit: 12..19 	Band Select Clock Divider Value
                                */
    uint32_t RfDivSel : 3;     /*!< bit: 20..22 	RF Divider Select */
    uint32_t Feedback : 1;     /*!< bit: 23 	 Feedback Select */
    uint32_t _reserved_0 : 8;  /*!< bit: 24..31 RESERVED */
  };
  uint32_t w;
} ADF4351_Reg4_t;

/** \brief  Union type for the structure of Register5 in ADF4351
 */
typedef union {
  struct {
    uint32_t ControlBits : 3;  /*!< bit:  0.. 2 	CONTROL BITS */
    uint32_t _reserved_0 : 16; /*!< bit:  3..18 RESERVED */
    uint32_t _reserved_1 : 2;  /*!< bit: 19..20 RESERVED */
    uint32_t _reserved_2 : 1;  /*!< bit: 21     RESERVED */
    uint32_t LdPinMode : 2;    /*!< bit: 22..23 LD Pin Mode */
    uint32_t _reserved_3 : 8;  /*!< bit: 24..31 RESERVED */
  };
  uint32_t w;
} ADF4351_Reg5_t;

/** \brief  Phase adjust type
 *  	The phase adjust bit (Bit DB28) enables adjustment
 *  	of the output phase of a given output frequency.
 */
typedef enum {
  ADF4351_PHASE_ADJ_OFF,
  ADF4351_PHASE_ADJ_ON
} ADF4351_PHASE_ADJ_t;

/** \brief Prescaler Value  type
 *  	The dual-modulus prescaler (P/P + 1), along with the INT, FRAC, and
 *		MOD values, determines the overall division ratio from the VCO
 *		output to the PFD input.
 */
typedef enum {
  ADF4351_PRESCALER_4_5, /*!< Prescaler = 4/5: NMIN = 23 */
  ADF4351_PRESCALER_8_9  /*!< Prescaler = 8/9: NMIN = 75 */
} ADF4351_PRESCALER_t;

/** \brief Low Noise and Low Spur Modes  type
 *  	The noise mode allows the user to optimize a design either
 *    for improved spurious performance or for improved phase noise performance.
 */
typedef enum {
  ADF4351_LOW_NOISE_MODE,
  ADF4351_LOW_SPUR_MODE = 3
} ADF4351_SPURNOISE_t;

/** \brief MUXOUT  type
 *  	The on-chip multiplexer
 */
typedef enum {
  ADF4351_MUX_THREESTATE,
  ADF4351_MUX_DVDD,
  ADF4351_MUX_DGND,
  ADF4351_MUX_RCOUNTER,
  ADF4351_MUX_NDIVIDER,
  ADF4351_MUX_ANALOGLOCK,
  ADF4351_MUX_DIGITALLOCK
} ADF4351_MUX_t;

/** \brief Disable/Enable  type
 *  	various bits are Disable(0)/Enable(1) type
 */
typedef enum { ADF4351_OFF, ADF4351_ON } ADF4351_ED_t;

/** \brief Charge Pump Current Setting  type
 *  	This value should be set to the charge pump current
 *    that the loop filter is designed with
 */
typedef enum {
  ADF4351_CPCURRENT_0_31,
  ADF4351_CPCURRENT_0_63,
  ADF4351_CPCURRENT_0_94,
  ADF4351_CPCURRENT_1_25,
  ADF4351_CPCURRENT_1_56,
  ADF4351_CPCURRENT_1_88,
  ADF4351_CPCURRENT_2_19,
  ADF4351_CPCURRENT_2_50,
  ADF4351_CPCURRENT_2_81,
  ADF4351_CPCURRENT_3_13,
  ADF4351_CPCURRENT_3_44,
  ADF4351_CPCURRENT_3_75,
  ADF4351_CPCURRENT_4_06,
  ADF4351_CPCURRENT_4_38,
  ADF4351_CPCURRENT_4_69,
  ADF4351_CPCURRENT_5_00
} ADF4351_CPCURRENT_t;

/** \brief Lock Detect Function  type
 *  	The LDF controls the number of PFD cycles monitored by the lock detect
 *    circuit to ascertain whether lock has been achieved.
 */
typedef enum { ADF4351_LDF_FRAC, ADF4351_LDF_INT } ADF4351_LDF_t;

/** \brief  Lock Detect Precision type
 *  	The lock detect precision  sets the comparison window in the lock detect
 * circuit.
 */
typedef enum { ADF4351_LDP_10NS, ADF4351_LDP_6NS } ADF4351_LDP_t;

/** \brief Phase Detector Polarity  type
 *  	Phase Detector Polarity
 */
typedef enum {
  ADF4351_POLARITY_NEGATIVE, /*!< For active filter with an inverting
                                charac-teristic */
  ADF4351_POLARITY_POSITIVE  /*!< For passive loop filter or a noninverting
                                active loop filter */
} ADF4351_POLARITY_t;

/** \brief  Band Select Clock Mode type
 *
 */
typedef enum {
  ADF4351_BANDCLOCK_LOW,
  ADF4351_BANDCLOCK_HIGH
} ADF4351_BANDCLOCK_t;

/** \brief  Antibacklash Pulse Width type
 *
 */
typedef enum { ADF4351_ABP_6NS, ADF4351_ABP_3NS } ADF4351_ABP_t;

/** \brief Clock Divider Mode  type
 *
 */
typedef enum {
  ADF4351_CLKDIVMODE_OFF,
  ADF4351_CLKDIVMODE_FAST_LOCK,
  ADF4351_CLKDIVMODE_RESYNC
} ADF4351_CLKDIVMODE_t;

/** \brief Feedback Select   type
 *
 */
typedef enum {
  ADF4351_FEEDBACK_DIVIDED,    /*!< the signal is taken from the output of the
                                  output dividers */
  ADF4351_FEEDBACK_FUNDAMENTAL /*!<  he signal is taken directly from the VCO
                                */
} ADF4351_FEEDBACK_t;

/** \brief RF Divider Select type
 *
 */
typedef enum {
  ADF4351_RFDIV_1,
  ADF4351_RFDIV_2,
  ADF4351_RFDIV_4,
  ADF4351_RFDIV_8,
  ADF4351_RFDIV_16,
  ADF4351_RFDIV_32,
  ADF4351_RFDIV_64
} ADF4351_RFDIV_t;

/** \brief Reference Divider
 *
 */
typedef enum { ADF4351_REFDIV_1, ADF4351_REFDIV_2 } ADF4351_REFDIV_t;

/** \brief Reference Doubler
 *
 */
typedef enum { ADF4351_REFMUL_1, ADF4351_REFMUL_2 } ADF4351_REFMUL_t;

/** \brief VCO Power-Down  type
 *
 */
typedef enum { ADF4351_VCO_POWERUP, ADF4351_VCO_POWERDOWN } ADF4351_VCO_POWER_t;

/** \brief Output Power  type
 *
 */
typedef enum {
  ADF4351_POWER_MINUS4DB,
  ADF4351_POWER_MINUS1DB,
  ADF4351_POWER_PLUS2DB,
  ADF4351_POWER_PLUS5DB
} ADF4351_POWER_t;

/** \brief Lock Detect Pin Operation  type
 *
 */
typedef enum {
  ADF4351_LD_PIN_LOW,
  ADF4351_LD_PIN_DIGITAL_LOCK,
  ADF4351_LD_PIN_LOW_,
  ADF4351_LD_PIN_HIGH
} ADF4351_LD_PIN_t;

typedef struct ADF4351_Config {
  ADF4351_Reg0_t reg0;
  ADF4351_Reg1_t reg1;
  ADF4351_Reg2_t reg2;
  ADF4351_Reg3_t reg3;
  ADF4351_Reg4_t reg4;
  ADF4351_Reg5_t reg5;
} ADF4351_Config;

// Init the ADF4351
void ADF4351_Init(ADF4351_Pins *pins);
void ADF4351_WriteRaw(uint32_t data);

// Init the ADF4351 config structure
void ADF4351_InitConfig(ADF4351_Config *config);
// Write the ADF4351 config structure to the device
void ADF4351_WriteConfig(ADF4351_Config *config);

// Returns the RF divider value for a given frequency to guarantee that the
// VCO frequency is between 2.2 and 4.4 GHz
ADF4351_RFDIV_t ADF4351_SelectOutputDivider(double RFoutFrequency);

// Set the config structure to a given frequency. Returns the actual frequency.
// After this call ADF4351_WriteConfig() to apply the settings to the device.
// If the frequency cannot be set exactly, try adjusting R Counter value.
double ADF4351_SetFrequency(ADF4351_Config *config, double rfout,
                            double fresout);