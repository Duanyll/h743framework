#include <stdlib.h>
#include <string.h>

#include "serial.h"

// TODO: Add enabled UART ports here
#define FOREACH_UART_PORT(apply)                                               \
  apply(USART1) apply(USART2) apply(USART3) apply(USART6)

#define DEFINE_UART_BUFFER_POINTER(port) volatile UART_RxBuffer *port##RxBuffer;
FOREACH_UART_PORT(DEFINE_UART_BUFFER_POINTER)

volatile UART_RxBuffer *UART_GetRxBuffer(UART_HandleTypeDef *huart) {
#define GET_UART_BUFFER_POINTER(port)                                          \
  if (huart->Instance == port) {                                               \
    return port##RxBuffer;                                                     \
  }
  FOREACH_UART_PORT(GET_UART_BUFFER_POINTER)
  return NULL;
}

// Init UART_RxBuffer struct
void UART_RxBuffer_Init(UART_RxBuffer *rxBuf, UART_HandleTypeDef *huart) {
  rxBuf->huart = huart;
  rxBuf->head = 0;
  rxBuf->tail = 0;
  rxBuf->isOpen = FALSE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  volatile UART_RxBuffer *rxBuf = UART_GetRxBuffer(huart);
  if (rxBuf == NULL) {
    return;
  }
  if (rxBuf->isOpen == FALSE) {
    return;
  }
  int newHead = (rxBuf->head + 1) % UART_RX_BUF_SIZE;
  if (newHead == rxBuf->tail) {
    // Buffer full
    UART_Close((UART_RxBuffer *)rxBuf);
    return;
  }
  rxBuf->buf[rxBuf->head] = rxBuf->recv;
  rxBuf->head = newHead;
  HAL_UART_Receive_IT(huart, (uint8_t *)&rxBuf->recv, 1);
}

// Open UART port and start receiving data via interrupt
BOOL UART_Open(UART_RxBuffer *rxBuf) {
  if (rxBuf->isOpen == TRUE) {
    return FALSE;
  }
#define SET_UART_BUFFER_POINTER(port)                                          \
  if (rxBuf->huart->Instance == port) {                                        \
    if (port##RxBuffer != NULL) {                                              \
      return FALSE;                                                            \
    }                                                                          \
    port##RxBuffer = rxBuf;                                                    \
  }
  FOREACH_UART_PORT(SET_UART_BUFFER_POINTER)
  rxBuf->head = 0;
  rxBuf->tail = 0;
  rxBuf->isOpen = TRUE;
  HAL_UART_Receive_IT(rxBuf->huart, (uint8_t *)&rxBuf->recv, 1);
  return 1;
}

/// @brief Try to read data with given length from UART_RxBuffer in blocking
/// mode. Only call this on main thread.
/// @param rxBuf Pointer to UART_RxBuffer struct
/// @param out Output buffer
/// @param len Max length to read
/// @param timeout Timeout in ms. Set to 0 will cause the function to return
/// immediately, even if no data is read. Set to -1 will cause the function to
/// block forever until len bytes are read.
/// @return Number of bytes read into out.
int UART_Read(UART_RxBuffer *rxBuf, char *out, int len, int timeout) {
  if (rxBuf->isOpen == FALSE) {
    return 0;
  }
  volatile UART_RxBuffer *buf = UART_GetRxBuffer(rxBuf->huart);
  if (buf == NULL) {
    return 0;
  }
  int received = 0;
  while (buf->tail != buf->head && buf->isOpen) {
    *out = buf->buf[buf->tail];
    buf->tail = (buf->tail + 1) % UART_RX_BUF_SIZE;
    out++;
    received++;
    if (received == len) {
      return received;
    }
  }
  if (timeout == 0)
    return received;
  if (timeout < 0)
    timeout = 0x3f3f3f3f;
  int start = HAL_GetTick();
  while (received < len && buf->isOpen && HAL_GetTick() - start < timeout) {
    if (buf->tail != buf->head) {
      *out = buf->buf[buf->tail];
      buf->tail = (buf->tail + 1) % UART_RX_BUF_SIZE;
      out++;
      received++;
    }
  }
  return received;
}

static BOOL check_delim_present(char *out, int received, const char *delim,
                                int delimLen) {
  if (received < delimLen) {
    return FALSE;
  }
  for (int i = 0; i < delimLen; i++) {
    if (*(out - i) != *(delim + delimLen - i - 1)) {
      return FALSE;
    }
  }
  return TRUE;
}

/// @brief Try to read data until delim is found or timeout in blocking mode.
/// Only call this on main thread.
/// @param rxBuf Pointer to UART_RxBuffer struct
/// @param out Output buffer
/// @param len Max length to read. 
/// @param delim Delimiter
/// @param timeout Timeout in ms. Set to 0 will cause the function to return
/// immediately, even if no data is read. Set to -1 will cause the function to
/// block forever until delim is found or len bytes are read.
/// @param readLen Number of bytes read into out.
/// @return Whether delim is found.
BOOL UART_ReadUntil(UART_RxBuffer *rxBuf, char *out, int len, const char *delim,
                   int timeout, int* readLen) {
  if (rxBuf->isOpen == FALSE) {
    return 0;
  }
  volatile UART_RxBuffer *buf = UART_GetRxBuffer(rxBuf->huart);
  if (buf == NULL) {
    return 0;
  }
  *readLen = 0;
  int delimLen = strlen(delim);
  while (buf->tail != buf->head && buf->isOpen) {
    *out = buf->buf[buf->tail];
    buf->tail = (buf->tail + 1) % UART_RX_BUF_SIZE;
    *readLen += 1;
    if (check_delim_present(out, *readLen, delim, delimLen)) {
      return TRUE;
    }
    out++;
    if (*readLen == len) {
      return FALSE;
    }
  }
  if (timeout == 0)
    return FALSE;
  if (timeout < 0)
    timeout = 0x3f3f3f3f;
  int start = HAL_GetTick();
  while (buf->isOpen && HAL_GetTick() - start < timeout) {
    if (buf->tail != buf->head) {
      *out = buf->buf[buf->tail];
      buf->tail = (buf->tail + 1) % UART_RX_BUF_SIZE;
      *readLen += 1;
      if (check_delim_present(out, *readLen, delim, delimLen)) {
        return TRUE;
      }
      out++;
    }
    if (*readLen == len) {
      return FALSE;
    }
  }
  return FALSE;
}
/// @brief Get number of unread bytes in UART_RxBuffer.
int UART_GetUnreadSize(UART_RxBuffer *rxBuf) {
  if (rxBuf->isOpen == FALSE) {
    return 0;
  }
  volatile UART_RxBuffer *buf = UART_GetRxBuffer(rxBuf->huart);
  if (buf == NULL) {
    return 0;
  }
  int size = buf->head - buf->tail;
  if (size < 0) {
    size += UART_RX_BUF_SIZE;
  }
  return size;
}
/// @brief Close UART port and stop receiving data
void UART_Close(UART_RxBuffer *rxBuf) {
  if (rxBuf->isOpen == FALSE) {
    return;
  }
  rxBuf->isOpen = FALSE;
#define CLEAR_UART_BUFFER_POINTER(port)                                        \
  if (rxBuf->huart->Instance == port) {                                        \
    port##RxBuffer = NULL;                                                     \
  }
  FOREACH_UART_PORT(CLEAR_UART_BUFFER_POINTER)
}

void UART_SendHex(UART_HandleTypeDef *huart, uint8_t *buf, int len) {
  HAL_UART_Transmit(huart, (uint8_t *)buf, len, 1000);
}

void UART_SendJson(UART_HandleTypeDef *huart, cJSON *json) {
  char *str = cJSON_PrintUnformatted(json);
  UART_SendString(huart, str);
  free(str);
}

void UART_SendString(UART_HandleTypeDef *huart, const char *str) {
  int len = strlen(str);
  HAL_UART_Transmit(huart, (uint8_t *)str, len, 1000);
}

char UART_txBuf[UART_TX_BUF_SIZE];
void UART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(UART_txBuf, UART_TX_BUF_SIZE, fmt, args);
  va_end(args);
  HAL_UART_Transmit(huart, (uint8_t *)UART_txBuf, strlen(UART_txBuf), 1000);
}