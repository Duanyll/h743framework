#include "screen.h"

#include "tim.h"
#include "usart.h"

static UART_HandleTypeDef *h;

void SCREEN_EndLine() { UART_SendString(h, "\xFF\xFF\xFF"); }

void SCREEN_Init(UART_HandleTypeDef *huart) {
  h = huart;
  SCREEN_EndLine();
  UART_SendString(h, "rest");
  SCREEN_EndLine();
  delay_ms(10);
}

void SCREEN_SetPage(const char *page) {
  UART_SendString(h, "page ");
  UART_SendString(h, page);
  SCREEN_EndLine();
}

void SCREEN_SetText(const char *controlId, const char *text) {
  UART_Printf(h, "%s.txt=\"%s\"", controlId, text);
  SCREEN_EndLine();
}

void SCREEN_PrintText(const char *controlId, const char *format, ...) {
  UART_SendString(h, controlId);
  UART_SendString(h, ".txt=\"");
  UART_Printf(h, format);
  UART_SendString(h, "\"");
  SCREEN_EndLine();
}

void SCREEN_TransmitPlotData(const char *controlId, int channel, uint8_t *data,
                             int len) {
  UART_Printf(h, "cle %s.id,%d", controlId, (int)channel);
  SCREEN_EndLine();
#ifdef SCREEN_USE_ADDT
  UART_Printf(h, "addt %s.id,%d,%d", controlId, (int)channel, len);
  SCREEN_EndLine();
  char buf[4];
  HAL_UART_Receive(h, (uint8_t *)buf, 4, 50);
  HAL_UART_Transmit(h, data, len, 100);
  HAL_UART_Receive(h, (uint8_t *)buf, 4, 100);
#else
  for (int i = 0; i < len; i++) {
    UART_Printf(h, "add %s.id,%d,%d", controlId, (int)channel, (int)data[i]);
    SCREEN_EndLine();
  }
#endif
}

// void (*SCREEN_userInputCallback)(uint8_t *data, int len);
// void SCREEN_UartCallback(uint8_t *data, int len) {}

// void SCREEN_HandleInput(void (*callback)(uint8_t *data, int len)) {
//   SCREEN_userInputCallback = callback;
//   UART_PollHexData(SCREEN_UartCallback);
// }