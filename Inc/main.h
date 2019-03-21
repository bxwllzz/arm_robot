/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define configUSE_NEWLIB_REENTRANT 1
#define LWIP_POSIX_SOCKETS_IO_NAMES 0
#define RS485_1_WE_Pin GPIO_PIN_2
#define RS485_1_WE_GPIO_Port GPIOE
#define LED_STATUS2_Pin GPIO_PIN_4
#define LED_STATUS2_GPIO_Port GPIOE
#define LED_STATUS1_Pin GPIO_PIN_5
#define LED_STATUS1_GPIO_Port GPIOE
#define BEEP_Pin GPIO_PIN_6
#define BEEP_GPIO_Port GPIOE
#define SENSE_TOP_Pin GPIO_PIN_13
#define SENSE_TOP_GPIO_Port GPIOC
#define SENSE_MIDDLE_Pin GPIO_PIN_14
#define SENSE_MIDDLE_GPIO_Port GPIOC
#define SENSE_BOTTOM_Pin GPIO_PIN_15
#define SENSE_BOTTOM_GPIO_Port GPIOC
#define RS485_1_RX_Pin GPIO_PIN_3
#define RS485_1_RX_GPIO_Port GPIOA
#define PWR_DOWN_INT_Pin GPIO_PIN_7
#define PWR_DOWN_INT_GPIO_Port GPIOE
#define LIFT_ALM_Pin GPIO_PIN_8
#define LIFT_ALM_GPIO_Port GPIOE
#define LIFT_DIR_DOWN_Pin GPIO_PIN_9
#define LIFT_DIR_DOWN_GPIO_Port GPIOE
#define LIFT_DISABLE_Pin GPIO_PIN_10
#define LIFT_DISABLE_GPIO_Port GPIOE
#define LIFT_UNLOCK_Pin GPIO_PIN_11
#define LIFT_UNLOCK_GPIO_Port GPIOE
#define SERVO1_Pin GPIO_PIN_13
#define SERVO1_GPIO_Port GPIOE
#define SERVO2_Pin GPIO_PIN_14
#define SERVO2_GPIO_Port GPIOE
#define KEY_DOWN_Pin GPIO_PIN_15
#define KEY_DOWN_GPIO_Port GPIOE
#define KEY_UP_Pin GPIO_PIN_10
#define KEY_UP_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define I2C3_INT2_Pin GPIO_PIN_10
#define I2C3_INT2_GPIO_Port GPIOD
#define I2C3_INT1_Pin GPIO_PIN_11
#define I2C3_INT1_GPIO_Port GPIOD
#define LIFT_PULSE_Pin GPIO_PIN_13
#define LIFT_PULSE_GPIO_Port GPIOD
#define RS485_2_TX_Pin GPIO_PIN_9
#define RS485_2_TX_GPIO_Port GPIOA
#define RS485_2_RX_Pin GPIO_PIN_10
#define RS485_2_RX_GPIO_Port GPIOA
#define RS485_3_WE_Pin GPIO_PIN_15
#define RS485_3_WE_GPIO_Port GPIOA
#define RS485_3_TX_Pin GPIO_PIN_10
#define RS485_3_TX_GPIO_Port GPIOC
#define RS485_3_RX_Pin GPIO_PIN_11
#define RS485_3_RX_GPIO_Port GPIOC
#define RS485_2_WE_Pin GPIO_PIN_3
#define RS485_2_WE_GPIO_Port GPIOD
#define LED_START_Pin GPIO_PIN_4
#define LED_START_GPIO_Port GPIOD
#define RS485_1_TX_Pin GPIO_PIN_5
#define RS485_1_TX_GPIO_Port GPIOD
#define BUT_START_Pin GPIO_PIN_6
#define BUT_START_GPIO_Port GPIOD
#define LED_STOP_Pin GPIO_PIN_7
#define LED_STOP_GPIO_Port GPIOD
#define BUT_STOP_Pin GPIO_PIN_3
#define BUT_STOP_GPIO_Port GPIOB
#define LED_RESET_Pin GPIO_PIN_4
#define LED_RESET_GPIO_Port GPIOB
#define BUT_RESET_Pin GPIO_PIN_5
#define BUT_RESET_GPIO_Port GPIOB
#define I2C1_INT1_Pin GPIO_PIN_8
#define I2C1_INT1_GPIO_Port GPIOB
#define I2C1_INT2_Pin GPIO_PIN_9
#define I2C1_INT2_GPIO_Port GPIOB
#define BUT_EMG1_Pin GPIO_PIN_0
#define BUT_EMG1_GPIO_Port GPIOE
#define BUT_EMG2_Pin GPIO_PIN_1
#define BUT_EMG2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
