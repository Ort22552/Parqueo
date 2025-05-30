/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define P3R_Pin GPIO_PIN_13
#define P3R_GPIO_Port GPIOC
#define P3G_Pin GPIO_PIN_14
#define P3G_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOC
#define P4R_Pin GPIO_PIN_2
#define P4R_GPIO_Port GPIOC
#define P4G_Pin GPIO_PIN_3
#define P4G_GPIO_Port GPIOC
#define LCD_RD_Pin GPIO_PIN_0
#define LCD_RD_GPIO_Port GPIOA
#define LCD_WR_Pin GPIO_PIN_1
#define LCD_WR_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_4
#define LCD_RS_GPIO_Port GPIOA
#define Parqueo4_Pin GPIO_PIN_5
#define Parqueo4_GPIO_Port GPIOC
#define Parqueo4_EXTI_IRQn EXTI9_5_IRQn
#define LCD_CS_Pin GPIO_PIN_0
#define LCD_CS_GPIO_Port GPIOB
#define SegE_Pin GPIO_PIN_1
#define SegE_GPIO_Port GPIOB
#define SegD_Pin GPIO_PIN_2
#define SegD_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_10
#define LCD_D6_GPIO_Port GPIOB
#define SegC_Pin GPIO_PIN_12
#define SegC_GPIO_Port GPIOB
#define SegG_Pin GPIO_PIN_14
#define SegG_GPIO_Port GPIOB
#define SegF_Pin GPIO_PIN_15
#define SegF_GPIO_Port GPIOB
#define Parqueo1_Pin GPIO_PIN_6
#define Parqueo1_GPIO_Port GPIOC
#define Parqueo1_EXTI_IRQn EXTI9_5_IRQn
#define LCD_D1_Pin GPIO_PIN_7
#define LCD_D1_GPIO_Port GPIOC
#define Parqueo2_Pin GPIO_PIN_8
#define Parqueo2_GPIO_Port GPIOC
#define Parqueo2_EXTI_IRQn EXTI9_5_IRQn
#define Parqueo3_Pin GPIO_PIN_9
#define Parqueo3_GPIO_Port GPIOC
#define Parqueo3_EXTI_IRQn EXTI9_5_IRQn
#define LCD_D7_Pin GPIO_PIN_8
#define LCD_D7_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_9
#define LCD_D0_GPIO_Port GPIOA
#define LCD_D2_Pin GPIO_PIN_10
#define LCD_D2_GPIO_Port GPIOA
#define SegB_Pin GPIO_PIN_11
#define SegB_GPIO_Port GPIOA
#define SegA_Pin GPIO_PIN_12
#define SegA_GPIO_Port GPIOA
#define P1R_Pin GPIO_PIN_10
#define P1R_GPIO_Port GPIOC
#define P1G_Pin GPIO_PIN_11
#define P1G_GPIO_Port GPIOC
#define P2R_Pin GPIO_PIN_12
#define P2R_GPIO_Port GPIOC
#define P2G_Pin GPIO_PIN_2
#define P2G_GPIO_Port GPIOD
#define LCD_D3_Pin GPIO_PIN_3
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_4
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_5
#define LCD_D4_GPIO_Port GPIOB
#define SD_SS_Pin GPIO_PIN_6
#define SD_SS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
