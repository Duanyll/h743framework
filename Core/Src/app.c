#include <math.h>
#include <stdlib.h>

#include "app.h"

#include "ad7606b.h"
#include "keys.h"
#include "nn.h"
#include "retarget.h"
#include "screen.h"
#include "serial.h"
#include "signal.h"
#include "spi.h"
#include "timers.h"

UART_HandleTypeDef *computer;

AD7606B_Config ad7606b_config;
AD7606B_Pins ad7606b_pins;

uint16_t identity_u16(uint16_t x) { return x; }

void APP_InitAD7606B() {
  ad7606b_pins.CS_Port = AD_CS_GPIO_Port;
  ad7606b_pins.CS_Pin = AD_CS_Pin;
  ad7606b_pins.RD_Port = AD_RD_GPIO_Port;
  ad7606b_pins.RD_Pin = AD_RD_Pin;
  ad7606b_pins.BUSY_Port = AD_BUSY_GPIO_Port;
  ad7606b_pins.BUSY_Pin = AD_BUSY_Pin;
  ad7606b_pins.WR_Port = AD_WR_GPIO_Port;
  ad7606b_pins.WR_Pin = AD_WR_Pin;

  ad7606b_pins.DB_Port = AD_D0_GPIO_Port;
  ad7606b_pins.DataToPins = identity_u16;
  ad7606b_pins.PinsToData = identity_u16;

  ad7606b_pins.OS0_Port = AD_OS0_GPIO_Port;
  ad7606b_pins.OS0_Pin = AD_OS0_Pin;
  ad7606b_pins.OS1_Port = AD_OS1_GPIO_Port;
  ad7606b_pins.OS1_Pin = AD_OS1_Pin;
  ad7606b_pins.OS2_Port = AD_OS2_GPIO_Port;
  ad7606b_pins.OS2_Pin = AD_OS2_Pin;
  ad7606b_pins.PAR_SEL_Port = AD_PAR_SEL_GPIO_Port;
  ad7606b_pins.PAR_SEL_Pin = AD_PAR_SEL_Pin;

  ad7606b_pins.TIM_Handle = &htim2;

  AD7606B_Init(&ad7606b_pins);
  AD7606B_InitConfig(&ad7606b_config);
  ad7606b_config.raw[AD7606C_REG_BANDWIDTH] = 0xff;
  AD7606B_ParallelRegisterWrite(AD7606C_REG_BANDWIDTH,
                                ad7606b_config.raw[AD7606C_REG_BANDWIDTH]);
  AD7606B_SetRange(&ad7606b_config, 0, AD7606C_RANGE_PM2V5);
  AD7606B_SetRange(&ad7606b_config, 1, AD7606C_RANGE_PM2V5);
  AD7606B_SetOverSample(&ad7606b_config, 0, AD7606B_OVERSAMPLE_4);
  AD7606B_SetOverSample(&ad7606b_config, 1, AD7606B_OVERSAMPLE_4);

  AD7606B_LeaveRegisterMode();
}

#define AD_SAMPLE_COUNT 2048
int16_t ad_data[AD_SAMPLE_COUNT * 4];

int popcount(uint8_t x) {
  int count = 0;
  while (x) {
    count += x & 1;
    x >>= 1;
  }
  return count;
}

void APP_UploadADCData(uint8_t channels, uint32_t sample_count,
                       uint32_t sample_rate) {
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  AD7606B_CollectSamples(ad_data, channels, sample_count, sample_rate);
  UART_SendString(computer, "\xff\xff\xff\xff");
  HAL_UART_Transmit(computer, (uint8_t *)ad_data,
                    sample_count * 2 * popcount(channels), 10000);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void APP_HexCommandCallback(uint8_t *data, int len) {
  if (*data == 1) {
    // 1 byte channels
    // 4 byte sample count
    // 4 byte sample rate
    uint8_t channels = 0x0f;
    uint32_t sample_count = 128;
    uint32_t sample_rate = 100000;
    if (len >= 2)
      channels = data[1];
    if (len >= 6)
      sample_count = *(uint32_t *)(data + 2);
    if (len >= 10)
      sample_rate = *(uint32_t *)(data + 6);
    APP_UploadADCData(channels, sample_count, sample_rate);
  } else if (*data == 2) {
    double freq = TIM_CountFrequencySync(&htim2, 100);
    printf("Freq: %fMHz\n", freq / 1000000.0);
  } else {
    // Echo back
    UART_SendHex(computer, data, len);
  }
}

UART_RxBuffer computer_rx_buf;
char computer_command[UART_RX_BUF_SIZE];

void APP_Init() {
  computer = &huart6;
  RetargetInit(computer);
  APP_InitAD7606B();

  UART_RxBuffer_Init(&computer_rx_buf, computer);
  UART_Open(&computer_rx_buf);
}

void APP_Loop() {
  int len = UART_ReadUntil(&computer_rx_buf, computer_command, UART_RX_BUF_SIZE,
                           "\n", -1);
  if (len > 0) {
    APP_HexCommandCallback((uint8_t *)computer_command, len - 1);
  } else if (computer_rx_buf.isOpen == FALSE) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    UART_Open(&computer_rx_buf);
  }
}