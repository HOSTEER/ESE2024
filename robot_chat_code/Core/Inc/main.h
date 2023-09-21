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
#include "stm32g0xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BT_RX_Pin GPIO_PIN_11
#define BT_RX_GPIO_Port GPIOC
#define R_FORWARD_Pin GPIO_PIN_12
#define R_FORWARD_GPIO_Port GPIOC
#define SW_GAME_STATUS_Pin GPIO_PIN_13
#define SW_GAME_STATUS_GPIO_Port GPIOC
#define BUMP_EXTI_Pin GPIO_PIN_0
#define BUMP_EXTI_GPIO_Port GPIOC
#define FBD_EXTI_Pin GPIO_PIN_1
#define FBD_EXTI_GPIO_Port GPIOC
#define BBD_EXTI_Pin GPIO_PIN_2
#define BBD_EXTI_GPIO_Port GPIOC
#define BAT_ADC_Pin GPIO_PIN_0
#define BAT_ADC_GPIO_Port GPIOA
#define ST_LINK_TX_Pin GPIO_PIN_2
#define ST_LINK_TX_GPIO_Port GPIOA
#define ST_LINK_RX_Pin GPIO_PIN_3
#define ST_LINK_RX_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_5
#define BT_TX_GPIO_Port GPIOA
#define R_ENCODE_B_Pin GPIO_PIN_6
#define R_ENCODE_B_GPIO_Port GPIOA
#define R_ENCODE_A_Pin GPIO_PIN_7
#define R_ENCODE_A_GPIO_Port GPIOA
#define LIDAR_TX_Pin GPIO_PIN_4
#define LIDAR_TX_GPIO_Port GPIOC
#define LIDAR_RX_Pin GPIO_PIN_5
#define LIDAR_RX_GPIO_Port GPIOC
#define I_READ_L_Pin GPIO_PIN_0
#define I_READ_L_GPIO_Port GPIOB
#define I_READ_R_Pin GPIO_PIN_1
#define I_READ_R_GPIO_Port GPIOB
#define L_FORWARD_Pin GPIO_PIN_14
#define L_FORWARD_GPIO_Port GPIOB
#define LIDAR_SPEED_Pin GPIO_PIN_15
#define LIDAR_SPEED_GPIO_Port GPIOB
#define L_ENCODE_B_Pin GPIO_PIN_8
#define L_ENCODE_B_GPIO_Port GPIOA
#define L_ENCODE_A_Pin GPIO_PIN_9
#define L_ENCODE_A_GPIO_Port GPIOA
#define LIDAR_EN_Pin GPIO_PIN_6
#define LIDAR_EN_GPIO_Port GPIOC
#define LIDAR_RANGING_EN_Pin GPIO_PIN_7
#define LIDAR_RANGING_EN_GPIO_Port GPIOC
#define L_REVERSE_Pin GPIO_PIN_0
#define L_REVERSE_GPIO_Port GPIOD
#define R_REVERSE_Pin GPIO_PIN_1
#define R_REVERSE_GPIO_Port GPIOD
#define USER_LED0_Pin GPIO_PIN_4
#define USER_LED0_GPIO_Port GPIOB
#define USER_LED1_Pin GPIO_PIN_5
#define USER_LED1_GPIO_Port GPIOB
#define USER_LED2_Pin GPIO_PIN_8
#define USER_LED2_GPIO_Port GPIOB
#define USER_LED3_Pin GPIO_PIN_9
#define USER_LED3_GPIO_Port GPIOB
#define USER_LED4_Pin GPIO_PIN_10
#define USER_LED4_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
