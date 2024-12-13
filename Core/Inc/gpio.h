/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define	ENABLE_SMPS()				HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_SET)
#define	DISABLE_SMPS()				HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_RESET)
#define ENABLE_EURIDIS()			HAL_GPIO_WritePin(UC_MBUS_EURIDIS_GPIO_Port, UC_MBUS_EURIDIS_Pin, GPIO_PIN_SET)
#define ENABLE_MBUS()				HAL_GPIO_WritePin(UC_MBUS_EURIDIS_GPIO_Port, UC_MBUS_EURIDIS_Pin, GPIO_PIN_RESET)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void MX_GPIO_Custom_Init(void);
void MX_GPIO_Enable_Power_Mon( uint8_t enable );
void MX_GPIO_Enable_Reed_Sensor( uint8_t enable );
void MX_GPIO_Enable_Tamper( uint8_t enable );
void MX_GPIO_Enable_VBackup_Detection( uint8_t enable );
void MX_GPIO_Enable_Alarm_Detection( uint8_t enable );
void MX_GPIO_Enable_Pulses_CH1D( uint8_t enable );
void MX_GPIO_Enable_Pulses_CH1P( uint8_t enable );
//void MX_GPIO_Enable_Pulses_CH2P( uint8_t enable );
void MX_GPIO_Enable_Pulses_CH2C( uint8_t enable );
void MX_GPIO_Enable_Sensor( void );
void MX_GPIO_Disable_Sensor( void );
void MX_GPIO_Relay_Activation( uint32_t _act );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

