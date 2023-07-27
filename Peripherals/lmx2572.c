#include "lmx2572.h"
#include "timers.h"

static LMX2572_Pins *pins;
#define WRITE(pin, val)                                                        \
  HAL_GPIO_WritePin(pins->pin##_Port, pins->pin##_Pin, val)

void LMX2572_SendData(uint32_t data) {
  int k = data;
  WRITE(CS, LOW);

  WRITE(SCK, HIGH);
  TIM_DelayUs(1);
  for (int i = 0; i < 24; i++) {
    WRITE(SCK, LOW);
    if (k & 0x800000) {
      WRITE(SDI, HIGH);
    } else {
      WRITE(SDI, LOW);
    }
    WRITE(SCK, HIGH);
    TIM_DelayUs(1);
    k = k << 1;
  }
  WRITE(CS, HIGH);
}

void LMX2572_SendDataArray(uint32_t *a, uint32_t num) {
  uint32_t *p;
  for (p = a + num - 1; p >= a; p--) {
    LMX2572_SendData(*p);
  }
}

void LMX2572_Init(LMX2572_Pins *p) {
  pins = p;
  GPIO_InitTypeDef s = {
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  s.Pin = pins->CS_Pin;
  HAL_GPIO_Init(pins->CS_Port, &s);
  s.Pin = pins->SCK_Pin;
  HAL_GPIO_Init(pins->SCK_Port, &s);
  s.Pin = pins->SDI_Pin;
  HAL_GPIO_Init(pins->SDI_Port, &s);
  s.Pin = pins->ENABLE_Pin;
  HAL_GPIO_Init(pins->ENABLE_Port, &s);
}

// Suppress -Wunused-variable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

static uint32_t R[126];
static double Frequencyout = 0;
static uint32_t Ref = 0;
static double Chanspace = 0;
static uint8_t POW = 0;
static uint8_t Sweepmode = 0; 
static uint32_t Timedelay = 0; 
static uint32_t Startfreq = 0;

#pragma region Register Data Stuff

/********定义R[0]寄存器控制变量*********/
static uint16_t POWERDOWN = 0;
static uint16_t RESET_SOFT = 0;
static uint16_t MUXOUT_LD_SEL = 0;
static uint16_t FCAL_EN = 0;
static uint16_t FCAL_LPFD_ADJ = 0;
static uint16_t FCAL_HPFD_ADJ = 0;
static uint16_t OUT_MUTE = 0;
static uint16_t VCO_PHASE_SYNC = 0;
static uint16_t RAMP_EN = 0;

/********定义R[1]寄存器控制变量*********/

static uint16_t CAL_CLK_DIV = 0;

/********定义R[7]寄存器控制变量*********/

static uint16_t OUT_FORCE = 0;

/********定义R[8]寄存器控制变量*********/

static uint16_t VCO_CAPCTRL_FORCE = 0;
static uint16_t VCO_DACISET_FORCE = 0;

/********定义R[9]寄存器控制变量*********/

static uint16_t OSC_2X = 0;

/********定义R[10]寄存器控制变量*********/

static uint16_t MULT = 0;

/********定义R[11]寄存器控制变量*********/

static uint16_t PLL_R = 0;

/********定义R[12]寄存器控制变量*********/

static uint16_t Pll_R_PRE = 0;

/********定义R[14]寄存器控制变量*********/

static uint16_t CPG = 0;

/********定义R[16]寄存器控制变量*********/

static uint16_t VCO_DACISET = 0;

/********定义R[17]寄存器控制变量*********/

static uint16_t VCO_DACISET_STRT = 0;

/********定义R[19]寄存器控制变量*********/

static uint16_t VCO_CAPCTRL = 0;

/********定义R[20]寄存器控制变量*********/

static uint16_t VCO_SEL_STRT_EN = 0;
static uint16_t VCO_SEL = 0;
static uint16_t VCO_SEL_FORCE = 0;

/********定义R[31]寄存器控制变量*********/

static uint16_t CHDIV_DIV2 = 0;

/********定义R[34]寄存器控制变量*********/

static uint32_t PLL_N = 0;
static uint16_t PLL_N_H = 0;

/********定义R[36]寄存器控制变量*********/

static uint16_t PLL_N_L = 0;

/********定义R[37]寄存器控制变量*********/

static uint16_t MASH_SEED_EN = 0;
static uint16_t PFD_DLY_SEL = 0;

/********定义R[38]寄存器控制变量*********/

static uint32_t PLL_DEN = 0;
static uint16_t PLL_DEN_H = 0;

/********定义R[39]寄存器控制变量*********/

static uint16_t PLL_DEN_L = 0;

/********定义R[40]寄存器控制变量*********/

static uint32_t MASH_SEED = 0;
static uint16_t MASH_SEED_H = 0;

/********定义R[41]寄存器控制变量*********/

static uint16_t MASH_SEED_L = 0;

/********定义R[42]寄存器控制变量*********/

static uint32_t PLL_NUM = 0;
static uint16_t PLL_NUM_H = 0;

/********定义R[43]寄存器控制变量*********/

static uint16_t PLL_NUM_L = 0;

/********定义R[44]寄存器控制变量*********/

static uint16_t OUTA_PWR = 0;
static uint16_t OUTB_PD = 0;
static uint16_t OUTA_PD = 0;
static uint16_t MASH_RESET_N = 0;
static uint16_t MASH_ORDER = 0;

/********定义R[45]寄存器控制变量*********/

static uint16_t OUTA_MUX = 0;
static uint16_t OUT_ISET = 0;
static uint16_t OUTB_PWR = 0;

/********定义R[46]寄存器控制变量*********/

static uint16_t OUTB_MUX = 0;

/********定义R[58]寄存器控制变量*********/

static uint16_t INPIN_IGNORE = 0;
static uint16_t INPIN_HYST = 0;
static uint16_t INPIN_LVL = 0;
static uint16_t INPIN_FMT = 0;

/********定义R[59]寄存器控制变量*********/

static uint16_t LD_TYPE = 0;

/********定义R[60]寄存器控制变量*********/

static uint16_t LD_DLY = 0;

/********定义R[69]寄存器控制变量*********/

static uint32_t MASH_RST_COUNT = 0;
static uint16_t MASH_RST_COUNT_H = 0;

/********定义R[70]寄存器控制变量*********/

static uint16_t MASH_RST_COUNT_L = 0;

/********定义R[71]寄存器控制变量*********/

static uint16_t STSREF_DIV_PRE = 0;
static uint16_t SYSREF_PULSE = 0;
static uint16_t SYSREF_EN = 0;
static uint16_t SYSREF_REPEAT = 0;

/********定义R[72]寄存器控制变量*********/

static uint16_t SYSREF_DIV = 0;

/********定义R[73]寄存器控制变量*********/

static uint16_t JESD_DAC1_CTRL = 0;
static uint16_t JESD_DAC2_CTRL = 0;

/********定义R[74]寄存器控制变量*********/

static uint16_t JESD_DAC3_CTRL = 0;
static uint16_t JESD_DAC4_CTRL = 0;
static uint16_t SYSREF_PULSE_CNT = 0;

/********定义R[75]寄存器控制变量*********/

static uint16_t CHDIV = 0;

/********定义R[78]寄存器控制变量*********/

uint64_t RAMP_THRESH = 0;
static uint16_t RAMP_THRESH_HH = 0;
static uint16_t QUICK_RECAL_EN = 0;
static uint16_t VCO_CAPCTRL_STRT = 0;

/********定义R[79]寄存器控制变量*********/

static uint16_t RAMP_THRESH_H = 0;

/********定义R[80]寄存器控制变量*********/

static uint16_t RAMP_THRESH_L = 0;

/********定义R[81]寄存器控制变量*********/

uint64_t RAMP_LIMIT_HIGH = 0;
static uint16_t RAMP_LIMIT_HIGH_HH = 0;

/********定义R[82]寄存器控制变量*********/

static uint16_t RAMP_LIMIT_HIGH_H = 0;

/********定义R[83]寄存器控制变量*********/

static uint16_t RAMP_LIMIT_HIGH_L = 0;

/********定义R[84]寄存器控制变量*********/

uint64_t RAMP_LIMIT_LOW = 0;
static uint16_t RAMP_LIMIT_LOW_HH = 0;

/********定义R[85]寄存器控制变量*********/

static uint16_t RAMP_LIMIT_LOW_H = 0;

/********定义R[86]寄存器控制变量*********/

static uint16_t RAMP_LIMIT_LOW_L = 0;

/********定义R[96]寄存器控制变量*********/

static uint16_t RAMP_BURST_EN = 0;
static uint16_t RAMP_BURST_COUNT = 0;

/********定义R[97]寄存器控制变量*********/

static uint16_t RAMP0_RST = 0;
static uint16_t RAMP_TRIGA = 0;
static uint16_t RAMP_TRIGB = 0;
static uint16_t RAMP_BURST_TRIG = 0;

/********定义R[98]寄存器控制变量*********/

static uint32_t RAMP0_INC = 0;
static uint16_t RAMP0_INC_H = 0;
static uint16_t RAMP0_DLY = 0;

/********定义R[99]寄存器控制变量*********/

static uint16_t RAMP0_INC_L = 0;

/********定义R[100]寄存器控制变量*********/

static uint16_t RAMP0_LEN = 0;

/********定义R[101]寄存器控制变量*********/

static uint16_t RAMP1_DLY = 0;
static uint16_t RAMP1_RST = 0;
static uint16_t RAMP0_NEXT = 0;
static uint16_t RAMP0_NEXT_TRIG = 0;

/********定义R[102]寄存器控制变量*********/

static uint32_t RAMP1_INC = 0;
static uint16_t RAMP1_INC_H = 0;

/********定义R[103]寄存器控制变量*********/

static uint16_t RAMP1_INC_L = 0;

/********定义R[104]寄存器控制变量*********/

static uint16_t RAMP1_LEN = 0;

/********定义R[105]寄存器控制变量*********/

static uint16_t RAMP_DLY_CNT = 0;
static uint16_t RAMP_MANUAL = 0;
static uint16_t RAMP1_NEXT = 0;
static uint16_t RAMP_NEXT_TRIG = 0;

/********定义R[106]寄存器控制变量*********/

static uint16_t RAMP_TRIG_CAL = 0;
static uint16_t RAMP_SCALE_COUNT = 0;

/********定义R[110]寄存器控制变量*********/

static uint16_t rb_LD_VTUNE = 0;
static uint16_t rb_VCO_SEL = 0;

/********定义R[111]寄存器控制变量*********/

static uint16_t rb_VCO_CAPCTRL = 0;

/********定义R[112]寄存器控制变量*********/

static uint16_t rb_VCO_DACISET = 0;

/********定义R[114]寄存器控制变量*********/
static uint16_t FSK_EN = 0;
static uint16_t FSK_SPI_LEVEL = 0;
static uint16_t FSK_SPI_DEV_SEL = 0;
static uint16_t FSK_MODE_SEL = 0;

/********定义R[115]寄存器控制变量*********/
static uint16_t FSK_DEV_SCALE = 0;

/********定义R[116]寄存器控制变量*********/
static uint16_t FSK_DEV0 = 0;

/********定义R[117]寄存器控制变量*********/
static uint16_t FSK_DEV1 = 0;

/********定义R[118]寄存器控制变量*********/
static uint16_t FSK_DEV2 = 0;

/********定义R[119]寄存器控制变量*********/
static uint16_t FSK_DEV3 = 0;

/********定义R[120]寄存器控制变量*********/
static uint16_t FSK_DEV4 = 0;

/********定义R[121]寄存器控制变量*********/
static uint16_t FSK_DEV5 = 0;

/********定义R[122]寄存器控制变量*********/
static uint16_t FSK_DEV6 = 0;

/********定义R[123]寄存器控制变量*********/
static uint16_t FSK_DEV7 = 0;

/********定义R[124]寄存器控制变量*********/
static uint16_t FSK_SPI_FAST_DEV = 0;
/****************************************/

static double Outputfrequency = 0; // 定义输出信号频率变量
static uint32_t Frequencyint = 0;  // 定义频率整数部分变量
static uint32_t Frequencyfra = 0;  // 定义频率小数部分变量

static double Fpfd = 0;           // 定义鉴相频率变量
static uint64_t Int_frc = 0;      // 定义求解INT和FRAC的中间变量
static uint32_t Dividervalue = 0; // 定义RF分频器数值
static uint32_t Refin = 0;        // 输入参考频率变量
static double Resolution = 0;     // 频率分辨率变量
static double Resvco = 0;         // 定义VCO通道分辨率变量
static uint32_t Grecd =
    0; // 定义最大公约数变量（Modulus和Fractionalvalue的最大公约数）

/*********求解最大公约数函数********/
static uint32_t Gcd(uint32_t a, uint32_t b) {
  uint32_t c = 0;
  while (b != 0) {
    c = a % b;
    a = b;
    b = c;
  }
  return a;
}

/*********寄存器值计算函数********
功能：根据输入变量计算寄存器值
输入：Frequencyout（输出频率，单位KHz），Ref（参考频率，单位KHz），Chanspace（分辨率，单位KHz）
输出：无
*******************************/
void LMX2572_RegisterData(double Freqout, double Space, uint8_t power) {
  Outputfrequency = Freqout; /*输出信号频率，单位KHz*/
  Resolution = Space;        /*频率分辨率，单位KHz*/
  Refin = 40000;             /*输入参考频率，单位KHz*/

  if (Outputfrequency >= 1600000 && Outputfrequency < 3200000) {
    CHDIV = 0; // RF2分频输出
  } else if (Outputfrequency >= 800000 && Outputfrequency < 1600000) {
    CHDIV = 1; // RF4分频输出
  } else if (Outputfrequency >= 400000 && Outputfrequency < 800000) {
    CHDIV = 3; // RF8分频输出
  } else if (Outputfrequency >= 200000 && Outputfrequency < 400000) {
    CHDIV = 5; // RF16分频输出
  } else if (Outputfrequency >= 100000 && Outputfrequency < 200000) {
    CHDIV = 7; // RF32分频输出
  } else if (Outputfrequency >= 50000 && Outputfrequency < 100000) {
    CHDIV = 9; // RF64分频输出
  } else if (Outputfrequency >= 25000 && Outputfrequency < 50000) {
    CHDIV = 12; // RF128分频输出
  } else if (Outputfrequency >= 12500 && Outputfrequency < 25000) {
    CHDIV = 14; // RF256分频输出
  }

  switch (CHDIV) {
  case (0):
    Dividervalue = 2;
    break;
  case (1):
    Dividervalue = 4;
    break;
  case (3):
    Dividervalue = 8;
    break;
  case (5):
    Dividervalue = 16;
    break;
  case (7):
    Dividervalue = 32;
    break;
  case (9):
    Dividervalue = 64;
    break;
  case (12):
    Dividervalue = 128;
    break;
  case (14):
    Dividervalue = 256;
    break;
  }

  if (Outputfrequency >= 3200000 && Outputfrequency <= 6400000) {
    OUTA_MUX = 1;
    Dividervalue = 1;
  } else {
    OUTA_MUX = 0;
  }

  //			PLL_N = 150;
  //			PLL_DEN = 1;
  //			PLL_NUM = 0;

  MASH_SEED = 0;
  MASH_RST_COUNT = 50000;
  RAMP_THRESH = 0;
  RAMP_LIMIT_HIGH = 0;
  RAMP_LIMIT_LOW = 0;
  RAMP0_INC = 0;
  RAMP1_INC = 0;

  //			CHDIV = 0;

  /********R[9]寄存器相关参数初始化配置*********/

  OSC_2X = 0;

  /********R[10]寄存器相关参数初始化配置*********/

  MULT = 1;

  /********R[11]寄存器相关参数初始化配置*********/

  PLL_R = 1;

  /********R[12]寄存器相关参数初始化配置*********/

  Pll_R_PRE = 2;

  Fpfd = Refin * (1.0 + OSC_2X) * MULT / (PLL_R * Pll_R_PRE); // 求解鉴相器频率

  Resvco = Resolution * Dividervalue; // 计算VCO的通道分辨率

  PLL_DEN = Refin * Dividervalue / Resolution; // 求解小数部分的分母

  Int_frc = Outputfrequency * Dividervalue * PLL_R * Pll_R_PRE /
            (Refin * (1.0 + OSC_2X) * MULT) *
            100000; // 求解整数部分与小数部分之和，并保留

  PLL_N = (uint32_t)Outputfrequency * Dividervalue * PLL_R * Pll_R_PRE /
          (Refin * (1.0 + OSC_2X) * MULT); // 求解整数部分

  PLL_NUM = PLL_DEN * (Int_frc - 100000 * PLL_N) / 100000; // 求解小数部分的分子

  if (!PLL_NUM) // 如果是整数N分频，那么小数部分的分母PLL_DEN取1，小数值取0
  {
    PLL_DEN = 1;
  } else {
    Grecd = Gcd(PLL_DEN, PLL_NUM); // 取最大公约数
    PLL_DEN = PLL_DEN / Grecd;     //
    PLL_NUM = PLL_NUM / Grecd;     //
  }

  //			OUTA_PWR = 50;
  ////端口A输出功率，范围是0-63
  OUTA_PWR = power;
  PFD_DLY_SEL = 6;
  MASH_ORDER = 4;

  /********R[0]寄存器相关参数初始化配置*********/

  POWERDOWN = 0;
  RESET_SOFT = 0;
  MUXOUT_LD_SEL = 1;
  FCAL_EN = 1;
  FCAL_LPFD_ADJ = 0;
  FCAL_HPFD_ADJ = 0;
  OUT_MUTE = 1;
  VCO_PHASE_SYNC = 0;
  RAMP_EN = 0;

  /********R[1]寄存器相关参数初始化配置*********/

  CAL_CLK_DIV = 0;

  /********R[7]寄存器相关参数初始化配置*********/

  OUT_FORCE = !OUT_MUTE;

  /********R[8]寄存器相关参数初始化配置*********/

  VCO_CAPCTRL_FORCE = 0;
  VCO_DACISET_FORCE = 0;

  /********R[14]寄存器相关参数初始化配置*********/

  CPG = 7;

  /********R[16]寄存器相关参数初始化配置*********/

  VCO_DACISET = 128;

  /********R[17]寄存器相关参数初始化配置*********/

  VCO_DACISET_STRT = 250;

  /********R[19]寄存器相关参数初始化配置*********/

  VCO_CAPCTRL = 183;

  /********R[20]寄存器相关参数初始化配置*********/

  VCO_SEL_STRT_EN = 0;
  VCO_SEL = 7;
  VCO_SEL_FORCE = 0;

  /********R[31]寄存器相关参数初始化配置*********/

  if (CHDIV == 0) {
    CHDIV_DIV2 = 0;
  } else {
    CHDIV_DIV2 = 1;
  }
  /********R[34]寄存器相关参数初始化配置*********/

  PLL_N_H = (uint16_t)(PLL_N >> 16);

  /********R[36]寄存器相关参数初始化配置*********/

  PLL_N_L = (uint16_t)PLL_N;

  /********R[37]寄存器相关参数初始化配置*********/

  MASH_SEED_EN = 1;
  //		PFD_DLY_SEL = 2;

  /********R[38]寄存器相关参数初始化配置*********/

  PLL_DEN_H = (uint16_t)(PLL_DEN >> 16);

  /********R[39]寄存器相关参数初始化配置*********/

  PLL_DEN_L = (uint16_t)PLL_DEN;

  /********R[40]寄存器相关参数初始化配置*********/

  MASH_SEED_H = (uint16_t)(MASH_SEED >> 16);

  /********R[41]寄存器相关参数初始化配置*********/

  MASH_SEED_L = (uint16_t)MASH_SEED;

  /********R[42]寄存器相关参数初始化配置*********/

  PLL_NUM_H = (uint16_t)PLL_NUM >> 16;

  /********R[43]寄存器相关参数初始化配置*********/

  PLL_NUM_L = (uint16_t)PLL_NUM;

  /********R[44]寄存器相关参数初始化配置*********/

  //		OUTA_PWR = 50;
  OUTB_PD = 0; // 开启B通道，置0;相反	置1；默认关闭
  OUTA_PD = 0; // 开启A通道，置0
  MASH_RESET_N = 1;
  //		MASH_ORDER = 1;

  /********R[45]寄存器相关参数初始化配置*********/

  //		OUTA_MUX = 1;
  OUT_ISET = 0;
  OUTB_PWR = 10;

  /********R[46]寄存器相关参数初始化配置*********/

  OUTB_MUX = 0;

  /********R[58]寄存器相关参数初始化配置*********/

  INPIN_IGNORE = 1;
  INPIN_HYST = 0;
  INPIN_LVL = 0;
  INPIN_FMT = 0;

  /********R[59]寄存器相关参数初始化配置*********/

  LD_TYPE = 1;

  /********R[60]寄存器相关参数初始化配置*********/

  LD_DLY = 1000;

  /********R[69]寄存器相关参数初始化配置*********/

  MASH_RST_COUNT_H = (uint16_t)(MASH_RST_COUNT >> 16);

  /********R[70]寄存器相关参数初始化配置*********/

  MASH_RST_COUNT_L = (uint16_t)MASH_RST_COUNT;

  /********R[71]寄存器相关参数初始化配置*********/

  STSREF_DIV_PRE = 4;
  SYSREF_PULSE = 0;
  SYSREF_EN = 0;
  SYSREF_REPEAT = 0;

  /********R[72]寄存器相关参数初始化配置*********/

  SYSREF_DIV = 0;

  /********R[73]寄存器相关参数初始化配置*********/

  JESD_DAC1_CTRL = 63;
  JESD_DAC2_CTRL = 0;

  /********R[74]寄存器相关参数初始化配置*********/

  JESD_DAC3_CTRL = 0;
  JESD_DAC4_CTRL = 0;
  SYSREF_PULSE_CNT = 0;

  /********R[75]寄存器相关参数初始化配置*********/

  //		CHDIV = 0;

  /********R[78]寄存器相关参数初始化配置*********/

  RAMP_THRESH_HH = (uint16_t)(RAMP_THRESH >> 32);
  QUICK_RECAL_EN = 0;
  VCO_CAPCTRL_STRT = 0;

  /********R[79]寄存器相关参数初始化配置*********/

  RAMP_THRESH_H = (uint16_t)(RAMP_THRESH >> 16);

  /********R[80]寄存器相关参数初始化配置*********/

  RAMP_THRESH_L = (uint16_t)RAMP_THRESH;

  /********R[81]寄存器相关参数初始化配置*********/

  RAMP_LIMIT_HIGH_HH = (uint16_t)(RAMP_LIMIT_HIGH >> 32);

  /********R[82]寄存器相关参数初始化配置*********/

  RAMP_LIMIT_HIGH_H = (uint16_t)(RAMP_LIMIT_HIGH >> 16);

  /********R[83]寄存器相关参数初始化配置*********/

  RAMP_LIMIT_HIGH_L = (uint16_t)RAMP_LIMIT_HIGH;

  /********R[84]寄存器相关参数初始化配置*********/

  RAMP_LIMIT_LOW_HH = (uint16_t)(RAMP_LIMIT_LOW >> 32);

  /********R[85]寄存器相关参数初始化配置*********/

  RAMP_LIMIT_LOW_H = (uint16_t)(RAMP_LIMIT_LOW >> 16);

  /********R[86]寄存器相关参数初始化配置*********/

  RAMP_LIMIT_LOW_L = (uint16_t)RAMP_LIMIT_LOW;

  /********R[96]寄存器相关参数初始化配置*********/

  RAMP_BURST_EN = 0;
  RAMP_BURST_COUNT = 0;

  /********R[97]寄存器相关参数初始化配置*********/

  RAMP0_RST = 0;
  RAMP_TRIGA = 0;
  RAMP_TRIGB = 0;
  RAMP_BURST_TRIG = 0;

  /********R[98]寄存器相关参数初始化配置*********/

  RAMP0_INC_H = (uint16_t)(RAMP0_INC >> 16);
  RAMP0_DLY = 0;

  /********R[99]寄存器相关参数初始化配置*********/

  RAMP0_INC_L = (uint16_t)RAMP0_INC;

  /********R[100]寄存器相关参数初始化配置*********/

  RAMP0_LEN = 0;

  /********R[101]寄存器相关参数初始化配置*********/

  RAMP1_DLY = 0;
  RAMP1_RST = 0;
  RAMP0_NEXT = 0;
  RAMP0_NEXT_TRIG = 0;

  /********R[102]寄存器相关参数初始化配置*********/

  RAMP1_INC_H = (uint16_t)(RAMP1_INC >> 16);

  /********R[103]寄存器相关参数初始化配置*********/

  RAMP1_INC_L = (uint16_t)RAMP1_INC;

  /********R[104]寄存器相关参数初始化配置*********/

  RAMP1_LEN = 0;

  /********R[105]寄存器相关参数初始化配置*********/

  RAMP_DLY_CNT = 0;
  RAMP_MANUAL = 0;
  RAMP1_NEXT = 0;
  RAMP_NEXT_TRIG = 0;

  /********R[106]寄存器相关参数初始化配置*********/

  RAMP_TRIG_CAL = 0;
  RAMP_SCALE_COUNT = 7;

  /********R[110]寄存器相关参数初始化配置*********/

  rb_LD_VTUNE = 0;
  rb_VCO_SEL = 0;

  /********R[111]寄存器相关参数初始化配置*********/

  rb_VCO_CAPCTRL = 183;

  /********R[112]寄存器相关参数初始化配置*********/

  rb_VCO_DACISET = 170;

  /********R[114]寄存器相关参数初始化配置*********/

  FSK_EN = 0;
  FSK_SPI_LEVEL = 0;
  FSK_SPI_DEV_SEL = 0;
  FSK_MODE_SEL = 0;

  /********定义R[115]寄存器控制变量*********/
  FSK_DEV_SCALE = 0;

  /********定义R[116]寄存器控制变量*********/
  FSK_DEV0 = 0;

  /********定义R[117]寄存器控制变量*********/
  FSK_DEV1 = 0;

  /********定义R[118]寄存器控制变量*********/
  FSK_DEV2 = 0;

  /********定义R[119]寄存器控制变量*********/
  FSK_DEV3 = 0;

  /********定义R[120]寄存器控制变量*********/
  FSK_DEV4 = 0;

  /********定义R[121]寄存器控制变量*********/
  FSK_DEV5 = 0;

  /********定义R[122]寄存器控制变量*********/
  FSK_DEV6 = 0;

  /********定义R[123]寄存器控制变量*********/
  FSK_DEV7 = 0;

  /********定义R[124]寄存器控制变量*********/
  FSK_SPI_FAST_DEV = 0;
  /*********************************************/

  R[0] = 0x002410; // 0000 0000 0010 0100 0001 0000
  R[0] += POWERDOWN;
  R[0] += RESET_SOFT << 1;
  R[0] += MUXOUT_LD_SEL << 2;
  R[0] += FCAL_EN << 3;
  R[0] += FCAL_LPFD_ADJ << 5;
  R[0] += FCAL_HPFD_ADJ << 7;
  R[0] += OUT_MUTE << 9;
  R[0] += VCO_PHASE_SYNC << 14;
  R[0] += RAMP_EN << 15;

  R[1] = 0x010808; // 0000 0001 0000 1000 0000 1000
  R[1] += CAL_CLK_DIV;

  R[7] = 0x0700B2; // 0000 0111 0000 0000 1011 0010
  R[7] += OUT_FORCE << 14;

  R[8] = 0x082000; // 0000 1000 0010 0000 0000 0000
  R[8] += VCO_CAPCTRL_FORCE << 14;
  R[8] += VCO_DACISET_FORCE << 11;

  R[9] = 0x090604; // 0000 1001 0000 0110 0000 0100
  R[9] += OSC_2X << 12;

  R[10] = 0x0A1058; // 0000 1010 0001 0000 0101 1000
  R[10] += MULT << 7;

  R[11] = 0x0B0008; // 0000 1011 0000 0000 0000 1000
  R[11] += PLL_R << 4;

  R[12] = 0x0C5000; // 0000 1100 0101 0000 0000 0000
  R[12] += Pll_R_PRE;

  R[14] = 0x0E1E00; // 0000 1110 0001 1110 0000 0000
  R[14] += CPG << 4;

  R[16] = 0x100000; // 0001 0000 0000 0000 0000 0000
  R[16] += VCO_DACISET;

  R[17] = 0x110000; // 0001 0001 0000 0000 0000 0000
  R[17] += VCO_DACISET_STRT;

  R[19] = 0x132700; // 0001 0011 0010 0111 0000 0000
  R[19] += VCO_CAPCTRL;

  R[20] = 0x148048; // 0001 0100 1000 0000 0100 1000
  R[20] += VCO_SEL_STRT_EN << 14;
  R[20] += VCO_SEL << 11;
  R[20] += VCO_SEL_FORCE << 10; //

  R[31] = 0x1F03EC; // 0001 1111 0000 0011 1110 1100
  R[31] += CHDIV_DIV2 << 14;

  R[34] = 0x220000; // 0010 0010 0000 0000 0000 0000
  R[34] += PLL_N_H;

  R[36] = 0x240000; // 0010 0100 0000 0000 0000 0000
  R[36] += PLL_N_L;

  R[37] = 0x250004; // 0010 0101 0000 0000 0000 0100
  R[37] += MASH_SEED_EN << 15;
  R[37] += PFD_DLY_SEL << 8;

  R[38] = 0x260000; // 0010 0110 0000 0000 0000 0000
  R[38] += PLL_DEN_H;

  R[39] = 0x270000; // 0010 0111 0000 0000 0000 0000
  R[39] += PLL_DEN_L;

  R[40] = 0x280000; // 0010 1000 0000 0000 0000 0000
  R[40] += MASH_SEED_H;

  R[41] = 0x290000; // 0010 1001 0000 0000 0000 0000
  R[41] += MASH_SEED_L;

  R[42] = 0x2A0000; // 0010 1010 0000 0000 0000 0000
  R[42] += PLL_NUM_H;

  R[43] = 0x2B0000; // 0010 1011 0000 0000 0000 0000
  R[43] += PLL_NUM_L;

  R[44] = 0x2C0000; // 0010 1100 0000 0000 0000 0000
  R[44] += OUTA_PWR << 8;
  R[44] += OUTB_PD << 7;
  R[44] += OUTA_PD << 6;
  R[44] += MASH_RESET_N << 5;
  R[44] += MASH_ORDER;

  R[45] = 0x2D0000; // 0010 1101 1100 0000 1100 0000
  R[45] += OUTA_MUX << 11;
  R[45] += OUT_ISET << 9;
  R[45] += OUTB_PWR;

  R[46] = 0x2E07FC; // 0010 1101 0000 0111 1111 1100
  R[46] += OUTB_MUX;

  R[58] = 0x3A0001; // 0011 1010 0000 0000 0000 0001
  R[58] += INPIN_IGNORE << 15;
  R[58] += INPIN_HYST << 14;
  R[58] += INPIN_LVL << 12;
  R[58] += INPIN_FMT << 9;

  R[59] = 0x3B0000; // 0011 1011 0000 0000 0000 0000
  R[59] += LD_TYPE;

  R[60] = 0x3C0000; // 0011 1011 0000 0000 0000 0000
  R[60] += LD_DLY;

  R[69] = 0x450000; // 0100 0101 0000 0000 0000 0000
  R[69] += MASH_RST_COUNT_H;

  R[70] = 0x460000; // 0100 0110 0000 0000 0000 0000
  R[70] += MASH_RST_COUNT_L;

  R[71] = 0x470001; // 0100 0111 0000 0000 0000 0001
  R[71] += STSREF_DIV_PRE << 5;
  R[71] += SYSREF_PULSE << 4;
  R[71] += SYSREF_EN << 3;
  R[71] += SYSREF_REPEAT << 2;

  R[72] = 0x480000; // 0100 1000 0000 0000 0000 0000
  R[72] += SYSREF_DIV;

  R[73] = 0x490000; // 0100 1001 0000 0000 0000 0000
  R[73] += JESD_DAC1_CTRL;
  R[73] += JESD_DAC2_CTRL << 6;

  R[74] = 0x4A0000; // 0100 1010 0000 0000 0000 0000
  R[74] += JESD_DAC3_CTRL;
  R[74] += JESD_DAC4_CTRL << 6;
  R[74] += SYSREF_PULSE_CNT << 12;

  R[75] = 0x4B0800; // 0100 1011 0000 1000 0000 0000
  R[75] += CHDIV << 6;

  R[78] = 0x4E0001; // 0100 1110 0000 0000 0000 0001
  R[78] += RAMP_THRESH_HH << 11;
  R[78] += QUICK_RECAL_EN << 9;
  R[78] += VCO_CAPCTRL_STRT << 1;

  R[79] = 0x4F0000; // 0100 1111 0000 0000 0000 0000
  R[79] += RAMP_THRESH_H;

  R[80] = 0x500000; // 0101 0000 0000 0000 0000 0000
  R[80] += RAMP_THRESH_L;

  R[81] = 0x510000; // 0101 0001 0000 0000 0000 0000
  R[81] += RAMP_LIMIT_HIGH_HH;

  R[82] = 0x520000; // 0101 0010 0000 0000 0000 0000
  R[82] += RAMP_LIMIT_HIGH_H;

  R[83] = 0x530000; // 0101 0011 0000 0000 0000 0000
  R[83] += RAMP_LIMIT_HIGH_L;

  R[84] = 0x540000; // 0101 0100 0000 0000 0000 0000
  R[84] += RAMP_LIMIT_LOW_HH;

  R[85] = 0x550000; // 0101 0101 0000 0000 0000 0000
  R[85] += RAMP_LIMIT_LOW_H;

  R[86] = 0x560000; // 0101 0110 0000 0000 0000 0000
  R[86] += RAMP_LIMIT_LOW_L;

  R[96] = 0x600000; // 0110 0000 0000 0000 0000 0000
  R[96] += RAMP_BURST_EN << 15;
  R[96] += RAMP_BURST_COUNT << 2;

  R[97] = 0x610800; // 0110 0001 0000 1000 0000 0000
  R[97] += RAMP0_RST << 15;
  R[97] += RAMP_TRIGB << 7;
  R[97] += RAMP_TRIGA << 3;
  R[97] += RAMP_BURST_TRIG;

  R[98] = 0x620000; // 0110 0010 0000 0000 0000 0000
  R[98] += RAMP0_INC_H << 2;
  R[98] += RAMP0_DLY;

  R[99] = 0x630000; // 0110 0011 0000 0000 0000 0000
  R[99] += RAMP0_INC_L;

  R[100] = 0x640000; // 0110 0100 0000 0000 0000 0000
  R[100] += RAMP0_LEN;

  R[101] = 0x650000; // 0110 0101 0000 0000 0000 0000
  R[101] += RAMP1_DLY << 6;
  R[101] += RAMP1_RST << 5;
  R[101] += RAMP0_NEXT << 4;
  R[101] += RAMP0_NEXT_TRIG;

  R[102] = 0x660000; // 0110 0110 0000 0000 0000 0000
  R[102] += RAMP1_INC_H;

  R[103] = 0x670000; // 0110 0111 0000 0000 0000 0000
  R[103] += RAMP1_INC_L;

  R[104] = 0x680000; // 0110 1000 0000 0000 0000 0000
  R[104] += RAMP1_LEN;

  R[105] = 0x690000; // 0110 1001 0000 0000 0000 0000
  R[105] += RAMP_DLY_CNT << 6;
  R[105] += RAMP_MANUAL << 5;
  R[105] += RAMP1_NEXT << 4;
  R[105] += RAMP_NEXT_TRIG;

  R[106] = 0x6A0000; // 0110 1010 0000 0000 0000 0000
  R[106] += RAMP_TRIG_CAL << 4;
  R[106] += RAMP_SCALE_COUNT;

  R[110] = 0x6E0000; // 0110 1110 0000 0000 0000 0000
  R[110] += rb_LD_VTUNE << 9;
  R[110] += rb_VCO_SEL << 5;

  R[111] = 0x6F0000; // 0110 1111 0000 0000 0000 0000
  R[111] += rb_VCO_CAPCTRL;

  R[112] = 0x700000; // 0111 0000 0000 0000 0000 0000
  R[112] += rb_VCO_DACISET;

  R[125] = 0x7D2288;
  R[113] = 0x710000;
  R[109] = 0x6D0000;
  R[108] = 0x6C0000;
  R[107] = 0x6B0000;
  R[95] = 0x5F0000;
  R[94] = 0x5E0000;
  R[93] = 0x5D0000;
  R[92] = 0x5C0000;
  R[91] = 0x5B0000;
  R[90] = 0x5A0000;
  R[89] = 0x590000;
  R[88] = 0x580000;
  R[87] = 0x570000;
  R[77] = 0x4D0000;
  R[76] = 0x4C000C;
  R[68] = 0x4403E8;
  R[67] = 0x430000;
  R[66] = 0x4201F4;
  R[65] = 0x410000;
  R[64] = 0x401388;
  R[63] = 0x3F0000;
  R[62] = 0x3E0322;
  R[61] = 0x3D00A8;
  R[57] = 0x390020;
  R[56] = 0x380000;
  R[55] = 0x370000;
  R[54] = 0x360000;
  R[53] = 0x350000;
  R[52] = 0x340820;
  R[51] = 0x330080;
  R[50] = 0x320000;
  R[49] = 0x314180;
  R[48] = 0x300300;
  R[47] = 0x2F0300;
  R[35] = 0x230004;
  R[33] = 0x211E21;
  R[32] = 0x200393;
  R[30] = 0x1E318C;
  R[29] = 0x1D318C;
  R[28] = 0x1C0488;
  R[27] = 0x1B0002;
  R[26] = 0x1A0DB0;
  R[25] = 0x190624;
  R[24] = 0x18071A;
  R[23] = 0x17007C;
  R[22] = 0x160001;
  R[21] = 0x150401;
  R[18] = 0x120064;
  R[15] = 0x0F064F;
  R[13] = 0x0D4000;
  R[6] = 0x06C802;
  R[5] = 0x0500C8;
  R[4] = 0x040A43;
  R[3] = 0x030642;
  R[2] = 0x020500;
}

#pragma endregion

#pragma GCC diagnostic pop

void LMX2572_SetFrequency(uint32_t freq) {
  WRITE(ENABLE, HIGH);
  Frequencyout = freq; // KHz
  Chanspace = 1;       // KHz
  POW = 50;
  LMX2572_RegisterData(Frequencyout, Chanspace, POW);
  LMX2572_SendData(0x00241E); 
  LMX2572_SendData(0x00241C); 
  LMX2572_SendDataArray(R, 113); 
  LMX2572_SendData(R[0]);

  HAL_Delay(99);
}