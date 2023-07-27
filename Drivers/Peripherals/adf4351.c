#ifdef ADF4351_ENABLE

#include "adf4351.h"

#include <math.h>

static ADF4351_Pins *pins;

#define LOW GPIO_PIN_RESET
#define HIGH GPIO_PIN_SET
#define WRITE(pin, state)                                                      \
  HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, (state))

// Delay more than 25ns
void ADF4351_Delay() {
  for (int i = 0; i < 20; i++) {
    __NOP();
  }
}

void ADF4351_Init(ADF4351_Pins *p) {
  pins = p;
  GPIO_InitTypeDef GPIO_InitStruct = {
      .Mode = GPIO_MODE_OUTPUT_PP,
      .Pull = GPIO_NOPULL,
      .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  GPIO_InitStruct.Pin = pins->CE_Pin;
  HAL_GPIO_Init(pins->CE_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->CLK_Pin;
  HAL_GPIO_Init(pins->CLK_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->DAT_Pin;
  HAL_GPIO_Init(pins->DAT_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = pins->LE_Pin;
  HAL_GPIO_Init(pins->LE_Port, &GPIO_InitStruct);

  WRITE(CLK, LOW);
  WRITE(DAT, LOW);
  WRITE(LE, LOW);
  WRITE(CE, HIGH);
}

void ADF4351_WriteRaw(uint32_t data) {
  for (int i = 31; i >= 0; i--) {
    WRITE(DAT, ((data >> i) & 1) ? HIGH : LOW);
    ADF4351_Delay();
    WRITE(CLK, HIGH);
    ADF4351_Delay();
    WRITE(CLK, LOW);
  }
  ADF4351_Delay();
  WRITE(LE, HIGH);
  ADF4351_Delay();
  WRITE(LE, LOW);
  ADF4351_Delay();
}

void ADF4351_InitConfig(ADF4351_Config *config) {
  config->reg0.w = 0;
  config->reg1.w = 1;
  config->reg2.w = 2;
  config->reg3.w = 3;
  config->reg4.w = 4;
  config->reg5.w = 5;
  config->reg5._reserved_1 = 3;

  config->reg1.PhaseAdjust = ADF4351_OFF;
  config->reg1.PhaseVal = 1;
  config->reg1.Prescaler = ADF4351_PRESCALER_4_5;

  config->reg2.CounterReset = ADF4351_OFF;
  config->reg2.RDiv2 = ADF4351_REFDIV_2;
  config->reg2.RMul2 = ADF4351_REFMUL_1;
  config->reg2.CPCurrent = ADF4351_CPCURRENT_2_50;
  config->reg2.CPTristate = ADF4351_OFF;
  config->reg2.DoubleBuffer = ADF4351_OFF;
  config->reg2.LockFunction = ADF4351_LDF_FRAC;
  config->reg2.LockPrecision = ADF4351_LDP_10NS;
  config->reg2.LowNoiseSpur = ADF4351_LOW_NOISE_MODE;
  config->reg2.MuxOut = ADF4351_MUX_THREESTATE;
  config->reg2.PhasePolarity = ADF4351_POLARITY_POSITIVE;
  config->reg2.PowerDown = ADF4351_OFF;
  config->reg2.RCountVal = 5;

  config->reg3.AntibacklashW = ADF4351_ABP_6NS;
  config->reg3.BandSelMode = ADF4351_BANDCLOCK_LOW;
  config->reg3.ChargeCh = ADF4351_OFF;
  config->reg3.ClkDivMod = ADF4351_CLKDIVMODE_OFF;
  config->reg3.ClkDivVal = 150;
  config->reg3.CsrEn = ADF4351_OFF;

  config->reg4.AuxEnable = ADF4351_OFF;
  config->reg4.Mtld = ADF4351_OFF;
  config->reg4.OutEnable = ADF4351_ON;
  config->reg4.OutPower = ADF4351_POWER_PLUS5DB;
  config->reg4.VcoPowerDown = ADF4351_OFF;
  config->reg4.Feedback = ADF4351_FEEDBACK_FUNDAMENTAL;
  config->reg4.BandClkDiv = 200;

  config->reg5.LdPinMode = ADF4351_LD_PIN_DIGITAL_LOCK;
}

void ADF4351_WriteConfig(ADF4351_Config *c) {
  ADF4351_WriteRaw(c->reg5.w);
  ADF4351_WriteRaw(c->reg4.w);
  ADF4351_WriteRaw(c->reg3.w);
  ADF4351_WriteRaw(c->reg2.w);
  ADF4351_WriteRaw(c->reg1.w);
  ADF4351_WriteRaw(c->reg0.w);
}

ADF4351_RFDIV_t ADF4351_SelectOutputDivider(double RFoutFrequency) {
  // Select divider
  if (RFoutFrequency >= 2.2e9)
    return ADF4351_RFDIV_1;
  if (RFoutFrequency >= 1.1e9)
    return ADF4351_RFDIV_2;
  if (RFoutFrequency >= 550e6)
    return ADF4351_RFDIV_4;
  if (RFoutFrequency >= 275e6)
    return ADF4351_RFDIV_8;
  if (RFoutFrequency >= 137.5e6)
    return ADF4351_RFDIV_16;
  if (RFoutFrequency >= 68.75e6)
    return ADF4351_RFDIV_32;
  return ADF4351_RFDIV_64;
}

/*
The following equations are used to program the ADF4351
synthesizer:
RFOUT = [INT + (FRAC/MOD)] × (fPFD/RF Divider)
where:
RFOUT is the RF frequency output.
INT is the integer division factor.
FRAC is the numerator of the fractional division (0 to MOD − 1).
MOD is the preset fractional modulus (2 to 4095).
RF Divider is the output divider that divides down the
VCO frequency.
fPFD = REFIN × [(1 + D)/(R × (1 + T))]
where:
REFIN is the reference frequency input.
D is the RF REFIN doubler bit (0 or 1).
R is the RF reference division factor (1 to 1023).
T is the reference divide-by-2 bit (0 or 1).
*/

double ADF4351_SetFrequency(ADF4351_Config *config, double rfout, double refin,
                            double fresout) {
  double fpfd = refin * (1 + config->reg2.RMul2) /
                (config->reg2.RCountVal * (1 + config->reg2.RDiv2));
  ADF4351_REFDIV_t rdiv = ADF4351_SelectOutputDivider(rfout);
  double fvco = rfout * (1 << rdiv);
  double ratio = fvco / fpfd;

  // MOD satifies 1 / MOD * fpfd / rfdiv == fresout
  // MOD == fpfd / (rfdiv * fresout)
  uint32_t MOD = (uint32_t)round(refin / (rdiv * fresout));
  // MOD must be between 2 and 4095
  if (MOD < 2)
    MOD = 2;
  if (MOD > 4095)
    MOD = 4095;

  // Calculate INT and FRAC such that INT + FRAC / MOD == ratio
  uint32_t INT = (uint32_t)ratio;
  uint32_t FRAC = (uint32_t)round((ratio - INT) * MOD);

  if (FRAC >= MOD) {
    FRAC -= MOD;
    INT++;
  }

  // Make sure INT is between 23 and 65535
  if (config->reg1.Prescaler == ADF4351_PRESCALER_4_5) {
    if (INT < 23)
      INT = 23;
  } else {
    if (INT < 75)
      INT = 75;
  }
  if (INT > 65535)
    INT = 65535;

  // Calculate actual frequency
  double actual = (INT + (double)FRAC / MOD) * fpfd / (1 << rdiv);

  config->reg0.IntVal = INT;
  config->reg0.FracVal = FRAC;
  config->reg1.ModVal = MOD;
  config->reg4.RfDivSel = rdiv;

  return actual;
}

#endif