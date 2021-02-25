/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUT1_OP9_Pin GPIO_PIN_13
#define OUT1_OP9_GPIO_Port GPIOC
#define OUT1_OP10_Pin GPIO_PIN_14
#define OUT1_OP10_GPIO_Port GPIOC
#define ON_LED1_Pin GPIO_PIN_1
#define ON_LED1_GPIO_Port GPIOD
#define OUT1_OP4_Pin GPIO_PIN_0
#define OUT1_OP4_GPIO_Port GPIOA
#define OUT1_OP3_Pin GPIO_PIN_1
#define OUT1_OP3_GPIO_Port GPIOA
#define OUT1_OP2_Pin GPIO_PIN_2
#define OUT1_OP2_GPIO_Port GPIOA
#define OUT1_OP1_Pin GPIO_PIN_3
#define OUT1_OP1_GPIO_Port GPIOA
#define OUT1_OP8_Pin GPIO_PIN_4
#define OUT1_OP8_GPIO_Port GPIOA
#define OUT1_OP7_Pin GPIO_PIN_5
#define OUT1_OP7_GPIO_Port GPIOA
#define OUT1_OP6_Pin GPIO_PIN_6
#define OUT1_OP6_GPIO_Port GPIOA
#define OUT1_OP5_Pin GPIO_PIN_7
#define OUT1_OP5_GPIO_Port GPIOA
#define ON_BUZZ_Pin GPIO_PIN_0
#define ON_BUZZ_GPIO_Port GPIOB
#define OUT1_OP12_Pin GPIO_PIN_1
#define OUT1_OP12_GPIO_Port GPIOB
#define OUT1_OP11_Pin GPIO_PIN_2
#define OUT1_OP11_GPIO_Port GPIOB
#define GPIO_W5500_CS_Pin GPIO_PIN_12
#define GPIO_W5500_CS_GPIO_Port GPIOB
#define SPI1_SCK_Pin GPIO_PIN_13
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_15
#define SPI1_MOSI_GPIO_Port GPIOB
#define Rst_Pin GPIO_PIN_8
#define Rst_GPIO_Port GPIOA
#define OUT1_OP15_Pin GPIO_PIN_3
#define OUT1_OP15_GPIO_Port GPIOB
#define OUT1_OP16_Pin GPIO_PIN_4
#define OUT1_OP16_GPIO_Port GPIOB
#define OUT1_OP14_Pin GPIO_PIN_5
#define OUT1_OP14_GPIO_Port GPIOB
#define OUT1_OP13_Pin GPIO_PIN_6
#define OUT1_OP13_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
