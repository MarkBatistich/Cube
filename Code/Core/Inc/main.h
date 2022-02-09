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
#define BRAKE_Pin GPIO_PIN_8
#define BRAKE_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOB
#define BAT_MEAS_Pin GPIO_PIN_0
#define BAT_MEAS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define BTN_Pin GPIO_PIN_5
#define BTN_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_12
#define CS_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define ICM_ADDR 0x68 << 1
#define DEVICE_CONFIG 0x11
#define DRIVE_CONFIG 0x13
#define GYRO_DATA_X1 0x25
#define GYRO_DATA_X0 0x26
#define ACCEL_DATA_X1 0x1F
#define ACCEL_CONFIG0 0x50
#define PWR_MGMT0 0x4E
#define WHO_AM_I 0x75
#define GYRO_CONFIG0 0x4F
#define R (0x1) << 7
#define W (0x0) << 7
#define SPI_WAIT 1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
