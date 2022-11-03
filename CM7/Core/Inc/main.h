/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOK
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOK
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOK
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOK
#define RMII_TXD1_Pin GPIO_PIN_12
#define RMII_TXD1_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define MCO1_Pin GPIO_PIN_8
#define MCO1_GPIO_Port GPIOA
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define RMII_TX_EN RMII_TX_EN_GPIO_Port, RMII_TX_EN_Pin
#define RMII_TXD0 RMII_TXD0_GPIO_Port, RMII_TXD0_Pin
#define RMII_TXD1 RMII_TXD1_GPIO_Port, RMII_TXD1_Pin

#define RMII_MDC RMII_MDC_GPIO_Port, RMII_MDC_Pin
#define RMII_MDIO RMII_MDIO_GPIO_Port, RMII_MDIO_Pin

#define RMII_REF RMII_REF_CLK_GPIO_Port, RMII_REF_CLK_Pin

#define RMII_CRS_DV RMII_CRS_DV_GPIO_Port, RMII_CRS_DV_Pin

#define RMII_RXD0 RMII_RXD0_GPIO_Port, RMII_RXD0_Pin
#define RMII_RXD1 RMII_RXD1_GPIO_Port, RMII_RXD1_Pin

#define LED1 LED1_GPIO_Port, LED1_Pin
#define LED2 LED2_GPIO_Port, LED2_Pin
#define LED3 LED3_GPIO_Port, LED3_Pin
#define LED4 LED4_GPIO_Port, LED4_Pin

#define LED_GREEN LED1
#define LED_ORANGE LED2
#define LED_RED LED3
#define LED_BLUE LED4

#define PIN_SET(PIN) HAL_GPIO_WritePin(PIN, GPIO_PIN_SET)
#define PIN_RESET(PIN) HAL_GPIO_WritePin(PIN, GPIO_PIN_RESET)
#define PIN_TOGGLE(PIN) HAL_GPIO_TogglePin(PIN)

#define LED_ON(LED) HAL_GPIO_WritePin(LED, GPIO_PIN_RESET)
#define LED_OFF(LED) HAL_GPIO_WritePin(LED, GPIO_PIN_SET)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
