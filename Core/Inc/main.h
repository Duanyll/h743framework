/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BOOL uint8_t
#define TRUE 1
#define FALSE 0
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AD_OS2_Pin GPIO_PIN_2
#define AD_OS2_GPIO_Port GPIOE
#define AD_CONVST_Pin GPIO_PIN_3
#define AD_CONVST_GPIO_Port GPIOE
#define AD_RESET_Pin GPIO_PIN_4
#define AD_RESET_GPIO_Port GPIOE
#define AD_RANGE_Pin GPIO_PIN_5
#define AD_RANGE_GPIO_Port GPIOE
#define AD_BUSY_Pin GPIO_PIN_6
#define AD_BUSY_GPIO_Port GPIOE
#define SWITCH1_Pin GPIO_PIN_0
#define SWITCH1_GPIO_Port GPIOA
#define SWITCH2_Pin GPIO_PIN_1
#define SWITCH2_GPIO_Port GPIOA
#define AD9834_FS_Pin GPIO_PIN_0
#define AD9834_FS_GPIO_Port GPIOB
#define AD9834_PS_Pin GPIO_PIN_1
#define AD9834_PS_GPIO_Port GPIOB
#define SWITCH4_Pin GPIO_PIN_2
#define SWITCH4_GPIO_Port GPIOB
#define AD_FIRSTDATA_Pin GPIO_PIN_7
#define AD_FIRSTDATA_GPIO_Port GPIOE
#define AD_RD_Pin GPIO_PIN_8
#define AD_RD_GPIO_Port GPIOE
#define AD_CS_Pin GPIO_PIN_9
#define AD_CS_GPIO_Port GPIOE
#define AD_WR_Pin GPIO_PIN_10
#define AD_WR_GPIO_Port GPIOE
#define AD_PAR_SEL_Pin GPIO_PIN_11
#define AD_PAR_SEL_GPIO_Port GPIOE
#define AD_STBY_Pin GPIO_PIN_12
#define AD_STBY_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOE
#define AD9834_FSY_Pin GPIO_PIN_10
#define AD9834_FSY_GPIO_Port GPIOB
#define AD9834_SCK_Pin GPIO_PIN_11
#define AD9834_SCK_GPIO_Port GPIOB
#define AD9834_SDA_Pin GPIO_PIN_12
#define AD9834_SDA_GPIO_Port GPIOB
#define AD9834_RST_Pin GPIO_PIN_13
#define AD9834_RST_GPIO_Port GPIOB
#define AD_D8_Pin GPIO_PIN_8
#define AD_D8_GPIO_Port GPIOD
#define AD_D9_Pin GPIO_PIN_9
#define AD_D9_GPIO_Port GPIOD
#define AD_D10_Pin GPIO_PIN_10
#define AD_D10_GPIO_Port GPIOD
#define AD_D11_Pin GPIO_PIN_11
#define AD_D11_GPIO_Port GPIOD
#define AD_D12_Pin GPIO_PIN_12
#define AD_D12_GPIO_Port GPIOD
#define AD_D13_Pin GPIO_PIN_13
#define AD_D13_GPIO_Port GPIOD
#define AD_D14_Pin GPIO_PIN_14
#define AD_D14_GPIO_Port GPIOD
#define AD_D15_Pin GPIO_PIN_15
#define AD_D15_GPIO_Port GPIOD
#define DA_SCLK_Pin GPIO_PIN_8
#define DA_SCLK_GPIO_Port GPIOA
#define DA_SDI_Pin GPIO_PIN_9
#define DA_SDI_GPIO_Port GPIOA
#define DA_CS1_Pin GPIO_PIN_10
#define DA_CS1_GPIO_Port GPIOA
#define DA_CS2_Pin GPIO_PIN_11
#define DA_CS2_GPIO_Port GPIOA
#define SWITCH3_Pin GPIO_PIN_12
#define SWITCH3_GPIO_Port GPIOA
#define AD_D0_Pin GPIO_PIN_0
#define AD_D0_GPIO_Port GPIOD
#define AD_D1_Pin GPIO_PIN_1
#define AD_D1_GPIO_Port GPIOD
#define AD_D2_Pin GPIO_PIN_2
#define AD_D2_GPIO_Port GPIOD
#define AD_D3_Pin GPIO_PIN_3
#define AD_D3_GPIO_Port GPIOD
#define AD_D4_Pin GPIO_PIN_4
#define AD_D4_GPIO_Port GPIOD
#define AD_D5_Pin GPIO_PIN_5
#define AD_D5_GPIO_Port GPIOD
#define AD_D6_Pin GPIO_PIN_6
#define AD_D6_GPIO_Port GPIOD
#define AD_D7_Pin GPIO_PIN_7
#define AD_D7_GPIO_Port GPIOD
#define AD9959_RST_Pin GPIO_PIN_5
#define AD9959_RST_GPIO_Port GPIOB
#define AD9959_IOUP_Pin GPIO_PIN_6
#define AD9959_IOUP_GPIO_Port GPIOB
#define AD9959_SDIO0_Pin GPIO_PIN_7
#define AD9959_SDIO0_GPIO_Port GPIOB
#define AD9959_SCLK_Pin GPIO_PIN_8
#define AD9959_SCLK_GPIO_Port GPIOB
#define AD9959_CSB_Pin GPIO_PIN_9
#define AD9959_CSB_GPIO_Port GPIOB
#define AD_OS0_Pin GPIO_PIN_0
#define AD_OS0_GPIO_Port GPIOE
#define AD_OS1_Pin GPIO_PIN_1
#define AD_OS1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
