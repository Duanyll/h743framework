#include <stdlib.h>
#include <string.h>

#include "serial.h"

#define UART_STATE_JSON_WAIT 0     // waiting for '{'
#define UART_STATE_JSON_RX 1       // receiving json, terminated by '\n'
#define UART_STATE_JSON_COMPLETE 2 // json received
#define UART_STATE_HEX_RX 3       // receiving hex, terminated by '\xff\xff\xff'
#define UART_STATE_HEX_COMPLETE 4 // hex received
typedef struct {
  uint8_t *rx_cur;
  uint8_t *rx_end;
  uint8_t rx[UART_RX_BUF_SIZE];
  int state;
  UART_HandleTypeDef *huart;
} UART_RxBuffer;
UART_RxBuffer UART_rxBuf;

void UART_ResetJsonRX(UART_HandleTypeDef *huart) {
  UART_rxBuf.rx_cur = UART_rxBuf.rx;
  UART_rxBuf.rx_end = UART_rxBuf.rx + UART_RX_BUF_SIZE;
  UART_rxBuf.state = UART_STATE_JSON_WAIT;
  UART_rxBuf.huart = huart;
  HAL_UART_Receive_IT(huart, UART_rxBuf.rx_cur, 1);
}

void UART_ResetHexRX(UART_HandleTypeDef *huart) {
  UART_rxBuf.rx_cur = UART_rxBuf.rx;
  UART_rxBuf.rx_end = UART_rxBuf.rx + UART_RX_BUF_SIZE;
  UART_rxBuf.state = UART_STATE_HEX_RX;
  UART_rxBuf.huart = huart;
  HAL_UART_Receive_IT(huart, UART_rxBuf.rx_cur, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == UART_rxBuf.huart) {
    char ch = *UART_rxBuf.rx_cur;
    // JSON mode
    if (UART_rxBuf.state == UART_STATE_JSON_WAIT) {
      if (ch == '{') {
        UART_rxBuf.state = UART_STATE_JSON_RX;
        UART_rxBuf.rx_cur++;
      }
      HAL_UART_Receive_IT(huart, UART_rxBuf.rx_cur, 1);
    } else if (UART_rxBuf.state == UART_STATE_JSON_RX) {
      if (ch == '\n') {
        UART_rxBuf.state = UART_STATE_JSON_COMPLETE;
        *UART_rxBuf.rx_cur = '\0';
      } else {
        UART_rxBuf.rx_cur++;
        if (UART_rxBuf.rx_cur >= UART_rxBuf.rx_end) {
          UART_rxBuf.rx_cur = UART_rxBuf.rx;
          UART_rxBuf.state = UART_STATE_JSON_WAIT;
          printf("UART Rx buffer overflow\n");
        }
        HAL_UART_Receive_IT(huart, UART_rxBuf.rx_cur, 1);
      }
      // HEX mode
    } else if (UART_rxBuf.state == UART_STATE_HEX_RX) {
      if (ch == '\xff' && UART_rxBuf.rx_cur - UART_rxBuf.rx >= 2 &&
          *(UART_rxBuf.rx_cur - 1) == '\xff' &&
          *(UART_rxBuf.rx_cur - 2) == '\xff') {
        UART_rxBuf.state = UART_STATE_HEX_COMPLETE;
        UART_rxBuf.rx_cur -= 2;
        *UART_rxBuf.rx_cur = '\0';
      } else {
        UART_rxBuf.rx_cur++;
        if (UART_rxBuf.rx_cur >= UART_rxBuf.rx_end) {
          UART_rxBuf.rx_cur = UART_rxBuf.rx;
          UART_rxBuf.state = UART_STATE_HEX_RX;
          printf("UART Rx buffer overflow\n");
        }
        HAL_UART_Receive_IT(huart, UART_rxBuf.rx_cur, 1);
      }
    }
  }
}

void UART_PollJsonData(void (*callback)(cJSON *json)) {
  if (UART_rxBuf.state == UART_STATE_JSON_COMPLETE) {
    cJSON *json = cJSON_Parse((char *)UART_rxBuf.rx);
    (*callback)(json);
    UART_ResetJsonRX(UART_rxBuf.huart);
    cJSON_Delete(json);
    free(json);
  }
}

void UART_PollHexData(void (*callback)(uint8_t *data, int len)) {
  if (UART_rxBuf.state == UART_STATE_HEX_COMPLETE) {
    (*callback)(UART_rxBuf.rx, UART_rxBuf.rx_cur - UART_rxBuf.rx);
    UART_ResetHexRX(UART_rxBuf.huart);
  }
}

typedef struct {
  uint8_t tx[UART_TX_BUF_SIZE];
  BOOL isBusy;
  UART_HandleTypeDef *huart;
} UART_TxBuffer;

UART_TxBuffer UART_txBuf;
void UART_SendJson(UART_HandleTypeDef *huart, cJSON *json) {
  char *str = cJSON_PrintUnformatted(json);
  UART_SendString(huart, str);
  free(str);
}

void UART_SendString(UART_HandleTypeDef *huart, const char *str) {
  while (UART_txBuf.isBusy) {
    // Wait for previous transmission to complete
  }
  UART_txBuf.huart = huart;
  UART_txBuf.isBusy = TRUE;
  int len = strlen(str);
  HAL_UART_Transmit(huart, (uint8_t *)str, len, 1000);
  UART_txBuf.isBusy = FALSE;
}

void UART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...) {
  while (UART_txBuf.isBusy) {
    // Wait for previous transmission to complete
  }
  UART_txBuf.huart = huart;
  UART_txBuf.isBusy = TRUE;
  va_list args;
  va_start(args, fmt);
  vsnprintf(UART_txBuf.tx, UART_TX_BUF_SIZE, fmt, args);
  va_end(args);
  HAL_UART_Transmit(huart, UART_txBuf.tx, strlen(UART_txBuf.tx), 1000);
  UART_txBuf.isBusy = FALSE;
}