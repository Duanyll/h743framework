#include <string.h>
#include <math.h>

#include "screen.h"

#include "serial.h"

static UART_HandleTypeDef *h;

void SCREEN_EndLine() { UART_SendString(h, "\xFF\xFF\xFF"); }

void SCREEN_Init(UART_HandleTypeDef *huart) {
  h = huart;
  SCREEN_EndLine();
  UART_SendString(h, "rest");
  SCREEN_EndLine();
  HAL_Delay(10);
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

void SCREEN_NumberBuffer_Init(SCREEN_NumberBuffer *buf, const char *controlId,
                              const char *format, int updateInterval) {
  buf->head = 0;
  buf->tail = 0;
  buf->sum = 0;
  buf->size = 0;
  buf->lastUpdateTick = 0;
  buf->updateInterval = updateInterval;
  strcpy(buf->controlId, controlId);
  strcpy(buf->format, format);
}

void SCREEN_NumberBuffer_Update(SCREEN_NumberBuffer *buf, double value) {
  double avg = (buf->size == 0) ? 0 : buf->sum / buf->size;
  if (fabs(value - avg) > SCREEN_DROP_AVERAGE_THRESHOLD * avg) {
    // clear buffer
    buf->head = 0;
    buf->tail = 0;
    buf->sum = 0;
    buf->size = 0;
  }
  if (buf->size == SCREEN_OUTOUT_BUF_SIZE) {
    // buffer full, drop oldest value
    buf->sum -= buf->buf[buf->tail];
    buf->tail = (buf->tail + 1) % SCREEN_OUTOUT_BUF_SIZE;
    buf->size--;
  }
  buf->buf[buf->head] = value;
  buf->head = (buf->head + 1) % SCREEN_OUTOUT_BUF_SIZE;
  buf->sum += value;
  buf->size++;
  int curTick = HAL_GetTick();
  if (curTick - buf->lastUpdateTick > buf->updateInterval) {
    buf->lastUpdateTick = curTick;
    avg = (buf->size == 0) ? 0 : buf->sum / buf->size;
    SCREEN_PrintText(buf->controlId, buf->format, avg);
  }
}