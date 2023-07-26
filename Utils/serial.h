#pragma once

#include <stdarg.h>

#include "cJSON.h"
#include "main.h"
#include "usart.h"

#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024

typedef struct {
  UART_HandleTypeDef *huart;
  char buf[UART_RX_BUF_SIZE]; // circular buffer
  char recv;                  // last received char
  int head; // buf[head - 1] is valid, buf[head] is next to write. head == tail
            // means buffer is empty. Head should never overwrite tail.
  int tail; // buf[tail] is next to read. head == tail means buffer is empty.
  BOOL isOpen; // Whether the port is open. The port will be closed if buffer is
               // full.
} UART_RxBuffer;

// Init UART_RxBuffer struct
void UART_RxBuffer_Init(UART_RxBuffer *rxBuf, UART_HandleTypeDef *huart);
// Open UART port and start receiving data via interrupt. Return if the
// operation is successful.
BOOL UART_Open(UART_RxBuffer *rxBuf);
/// @brief Try to read data with given length from UART_RxBuffer in blocking
/// mode. Only call this on main thread.
/// @param rxBuf Pointer to UART_RxBuffer struct
/// @param out Output buffer
/// @param len Max length to read
/// @param timeout Timeout in ms. Set to 0 will cause the function to return
/// immediately, even if no data is read. Set to -1 will cause the function to
/// block forever until len bytes are read.
/// @return Number of bytes read into out.
int UART_Read(UART_RxBuffer *rxBuf, char *out, int len, int timeout);
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
                    int timeout, int *readLen);
/// @brief Get number of unread bytes in UART_RxBuffer.
int UART_GetUnreadSize(UART_RxBuffer *rxBuf);
/// @brief Close UART port and stop receiving data
void UART_Close(UART_RxBuffer *rxBuf);

void UART_SendHex(UART_HandleTypeDef *huart, uint8_t *buf, int len);
void UART_SendJson(UART_HandleTypeDef *huart, cJSON *json);
void UART_SendString(UART_HandleTypeDef *huart, const char *str);
void UART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);