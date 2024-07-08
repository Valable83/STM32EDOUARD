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
#include "stm32l1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void TIM7_IRQ(void);
void StartSequence(uint32_t MyDelay);
void StopAff(void);
void OnAff(void);
void BP1(void);
void motor(int MotorState);
void adcFunction(void);
void buzzer (int BuzzerState);
void K2000L(void);
void K2000R(void);
void BP3_IRQ(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define BTN4_Pin GPIO_PIN_5
#define BTN4_GPIO_Port GPIOC
#define BTN4_EXTI_IRQn EXTI9_5_IRQn
#define L0_Pin GPIO_PIN_1
#define L0_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_2
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_10
#define L2_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_11
#define L3_GPIO_Port GPIOB
#define L4_Pin GPIO_PIN_12
#define L4_GPIO_Port GPIOB
#define L5_Pin GPIO_PIN_13
#define L5_GPIO_Port GPIOB
#define L6_Pin GPIO_PIN_14
#define L6_GPIO_Port GPIOB
#define L7_Pin GPIO_PIN_15
#define L7_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_6
#define BTN3_GPIO_Port GPIOC
#define BTN3_EXTI_IRQn EXTI9_5_IRQn
#define Buzz_Pin GPIO_PIN_7
#define Buzz_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_8
#define SPI_CS_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_11
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Mot_Pin GPIO_PIN_4
#define Mot_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
