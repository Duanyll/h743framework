/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
#include "timers.h"
#include "led.h"
/* USER CODE END 0 */

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI3 clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SPI3 DMA Init */
    /* SPI3_RX Init */
    hdma_spi3_rx.Instance = DMA1_Stream0;
    hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi3_rx);

    /* SPI3 interrupt Init */
    HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    PC12     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

    /* SPI3 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);

    /* SPI3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI3_IRQn);
  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
volatile uint16_t *SPI_rxBuffer;
volatile uint16_t *SPI_outputBuffer;
volatile uint16_t SPI_halfBufferLength;
volatile uint16_t *SPI_validBuffer;
volatile int64_t SPI_validBufferStartTime;
volatile int64_t SPI_validBufferEndTime;

volatile BOOL SPI_requestOutputFlag;

void SPI_StartDMARecieve(uint16_t *buffer, uint16_t totalSize, uint16_t *output,
                         TIM_HandleTypeDef *htim) {
  TIM_StartUsTimer(htim);
  SPI_rxBuffer = buffer;
  SPI_outputBuffer = output;
  SPI_halfBufferLength = totalSize / 2;
  SPI_validBuffer = NULL;
  SPI_validBufferStartTime = 0;
  SPI_validBufferEndTime = TIM_GetUsTimer();
  SPI_requestOutputFlag = FALSE;

  HAL_StatusTypeDef status =
      HAL_SPI_Receive_DMA(&hspi3, (uint8_t *)buffer, totalSize);
  if (status != HAL_OK) {
    
  }
}

void SPI_StopDMARecieve() {
  TIM_StopUsTimer();
  HAL_SPI_DMAStop(&hspi3);
}

void SPI_RequestOutput() {
  SPI_requestOutputFlag = TRUE;
}

void SPI_WaitOutput() {
  while (SPI_requestOutputFlag) {
    __NOP();
  }
}

uint16_t SPI_GetResampledOutput() {
  int64_t currentTime = TIM_GetUsTimer();
  int64_t startTime = SPI_validBufferStartTime;
  int64_t endTime = SPI_validBufferEndTime;
  int64_t duration = endTime - startTime;
  int64_t time = currentTime - duration;
  if (time < startTime) {
    return SPI_validBuffer[0];
  } else if (time > endTime) {
    return SPI_validBuffer[SPI_halfBufferLength - 1];
  }
  int64_t offset = time - startTime;
  int64_t index = offset * SPI_halfBufferLength / duration;
  return SPI_validBuffer[index];
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
  LED_On(2);
  SPI_validBuffer = SPI_rxBuffer;
  SPI_validBufferStartTime = SPI_validBufferEndTime;
  SPI_validBufferEndTime = TIM_GetUsTimer();
  if (SPI_requestOutputFlag) {
    for (int i = 0; i < SPI_halfBufferLength; i++) {
      SPI_outputBuffer[i] = SPI_validBuffer[i];
    }
    SPI_requestOutputFlag = FALSE;
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  LED_Off(2);
  SPI_validBuffer = SPI_rxBuffer + SPI_halfBufferLength;
  SPI_validBufferStartTime = SPI_validBufferEndTime;
  SPI_validBufferEndTime = TIM_GetUsTimer();
  if (SPI_requestOutputFlag) {
    for (int i = 0; i < SPI_halfBufferLength; i++) {
      SPI_outputBuffer[i] = SPI_validBuffer[i];
    }
    SPI_requestOutputFlag = FALSE;
  }
}
/* USER CODE END 1 */
