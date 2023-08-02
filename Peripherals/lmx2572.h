#include "common.h"
#include "stm32h743xx.h"

typedef struct {
  GPIO_TypeDef *CS_Port;
  uint16_t CS_Pin;
  GPIO_TypeDef *SCK_Port;
  uint16_t SCK_Pin;
  GPIO_TypeDef *SDI_Port;
  uint16_t SDI_Pin;
  GPIO_TypeDef *ENABLE_Port;
  uint16_t ENABLE_Pin;
  GPIO_TypeDef *MUXOUT_Port;
  uint16_t MUXOUT_Pin;
} LMX2572_Pins;

#define LMX2572_REFIN_40MHZ 40000000
#define LMX2572_REFIN_50MHZ 50000000
#define LMX2572_FPD 10000000

void LMX2572_WriteRegister(uint8_t addr, uint16_t data);
uint16_t LMX2572_ReadRegister(uint8_t addr);
void LMX2572_Init(LMX2572_Pins *pins, uint32_t refin);
void LMX2572_Reset();
void LMX2572_Update(BOOL enable);
void LMX2572_SetOutAPower(int power);
void LMX2572_SetFrequency(double freq);