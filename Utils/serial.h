#pragma once

#include <stdarg.h>

#include "main.h"
#include "usart.h"
#include "cJSON.h"

#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024

void UART_ResetJsonRX(UART_HandleTypeDef *huart);
void UART_PollJsonData(void (*callback)(cJSON *json));
void UART_ResetHexRX(UART_HandleTypeDef *huart);
void UART_PollHexData(void (*callback)(uint8_t *data, int len));

void UART_SendJson(UART_HandleTypeDef *huart, cJSON *json);
void UART_SendString(UART_HandleTypeDef *huart, const char *str);
void UART_Printf(UART_HandleTypeDef *huart, const char *fmt, ...);