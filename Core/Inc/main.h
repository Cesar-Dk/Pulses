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
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "linked_list.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern DMA_QListTypeDef Queue_i2c_sensor_rx;
extern DMA_QListTypeDef Queue_i2c_sensor_tx;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UC_SENSOR_REED_Pin GPIO_PIN_13
#define UC_SENSOR_REED_GPIO_Port GPIOC
#define UC_SENSOR_REED_EXTI_IRQn EXTI13_IRQn
#define UC_SMPS_ENABLE_Pin GPIO_PIN_0
#define UC_SMPS_ENABLE_GPIO_Port GPIOH
#define UC_MODBUS_ENABLE_Pin GPIO_PIN_1
#define UC_MODBUS_ENABLE_GPIO_Port GPIOH
#define UC_TAMPER_Pin GPIO_PIN_0
#define UC_TAMPER_GPIO_Port GPIOA
#define UC_TAMPER_EXTI_IRQn EXTI0_IRQn
#define UC_MODBUS_DRV_Pin GPIO_PIN_1
#define UC_MODBUS_DRV_GPIO_Port GPIOA
#define UC_MODBUS_TRX_TX_Pin GPIO_PIN_2
#define UC_MODBUS_TRX_TX_GPIO_Port GPIOA
#define UC_MODBUS_TRX_RX_Pin GPIO_PIN_3
#define UC_MODBUS_TRX_RX_GPIO_Port GPIOA
#define UC_ADC1_Pin GPIO_PIN_4
#define UC_ADC1_GPIO_Port GPIOA
#define UC_MBUS_RX_Pin GPIO_PIN_5
#define UC_MBUS_RX_GPIO_Port GPIOA
#define UC_PULSES_CH1P_Pin GPIO_PIN_6
#define UC_PULSES_CH1P_GPIO_Port GPIOA
#define UC_PULSES_CH1P_EXTI_IRQn EXTI6_IRQn
#define UC_PULSES_CH1D_Pin GPIO_PIN_7
#define UC_PULSES_CH1D_GPIO_Port GPIOA
#define UC_PULSES_CH1D_EXTI_IRQn EXTI7_IRQn
#define UC_LED_BLUE_Pin GPIO_PIN_0
#define UC_LED_BLUE_GPIO_Port GPIOB
#define UC_PULSES_TAMP_Pin GPIO_PIN_1
#define UC_PULSES_TAMP_GPIO_Port GPIOB
#define UC_PULSES_TAMP_EXTI_IRQn EXTI1_IRQn
#define UC_SECURE_ENABLER_Pin GPIO_PIN_2
#define UC_SECURE_ENABLER_GPIO_Port GPIOB
#define UC_MBUS_TX_Pin GPIO_PIN_10
#define UC_MBUS_TX_GPIO_Port GPIOB
#define UC_SPI_SS_Pin GPIO_PIN_12
#define UC_SPI_SS_GPIO_Port GPIOB
#define UC_SPI_SCK_Pin GPIO_PIN_13
#define UC_SPI_SCK_GPIO_Port GPIOB
#define UC_SPI_MISO_Pin GPIO_PIN_14
#define UC_SPI_MISO_GPIO_Port GPIOB
#define UC_SPI_MOSI_Pin GPIO_PIN_15
#define UC_SPI_MOSI_GPIO_Port GPIOB
#define UC_NBIOT_PWRMON_Pin GPIO_PIN_8
#define UC_NBIOT_PWRMON_GPIO_Port GPIOA
#define UC_NBIOT_PWRMON_EXTI_IRQn EXTI8_IRQn
#define UC_NBIOT_UART_TX_Pin GPIO_PIN_9
#define UC_NBIOT_UART_TX_GPIO_Port GPIOA
#define UC_NBIOT_UART_RX_Pin GPIO_PIN_10
#define UC_NBIOT_UART_RX_GPIO_Port GPIOA
#define UC_NBIOT_UART_CTS_Pin GPIO_PIN_11
#define UC_NBIOT_UART_CTS_GPIO_Port GPIOA
#define UC_NBIOT_UART_RTS_Pin GPIO_PIN_12
#define UC_NBIOT_UART_RTS_GPIO_Port GPIOA
#define UC_NBIOT_ON_OFF_Pin GPIO_PIN_15
#define UC_NBIOT_ON_OFF_GPIO_Port GPIOA
#define UC_NBIOT_ENABLE_Pin GPIO_PIN_4
#define UC_NBIOT_ENABLE_GPIO_Port GPIOB
#define UC_SENSOR_ENABLE_Pin GPIO_PIN_5
#define UC_SENSOR_ENABLE_GPIO_Port GPIOB
#define UC_SPI_MEMORY_ENABLE_Pin GPIO_PIN_3
#define UC_SPI_MEMORY_ENABLE_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */
#define UC_AC_DETECTION_Pin GPIO_PIN_3
#define UC_AC_DETECTION_GPIO_Port GPIOB
#define UC_I2C1_SCL_EXT_Pin GPIO_PIN_8
#define UC_I2C1_SCL_EXT_GPIO_Port GPIOB
#define UC_I2C1_SDA_EXT_Pin GPIO_PIN_9
#define UC_I2C1_SDA_EXT_GPIO_Port GPIOB
#define UC_UART1_I2C_RX_Pin GPIO_PIN_7
#define UC_UART1_I2C_RX_GPIO_Port GPIOB
#define UC_UART1_I2C_TX_Pin GPIO_PIN_6
#define UC_UART1_I2C_TX_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
