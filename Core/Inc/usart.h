/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "comm_serial.h"
#include "serial_modbus.h"
#include "serial_une82326.h"
#include "serial_mbus.h"
#include "modbus_sensors.h"
#include "stm32l4xx_hal_uart_emul.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
/*  Enable the clock for port UART Emulation */
#define UART_EMUL_TX_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE();
#define UART_EMUL_RX_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE();
/* Initialize GPIO and pin number for UART Emulation */
#define UART_EMUL_TX_PIN                      GPIO_PIN_9
#define UART_EMUL_TX_PORT                     GPIOA
#define UART_EMUL_RX_PIN                      GPIO_PIN_10
#define UART_EMUL_RX_PORT                     GPIOA

/* Definition for UART EMUL NVIC */
#define UART_EMUL_EXTI_IRQHandler             EXTI10_IRQHandler
#define UART_EMUL_EXTI_IRQ                    EXTI10_IRQn

extern UART_Emul_HandleTypeDef UartEmulHandle;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void 	MX_USART3_UART_Custom_Init(void);
void    MX_USART3_UART_Custom_Severn_Trent_Init(void);
void 	MX_USART2_UART_Custom_Init(void);
void 	MX_USART2_UART_DLMS_Init(void);

void    MX_USART1_UART_Euridis_Init(void);
void    MX_USART1_UART_Euridis_Init_Tx(void);
void 	MX_USART1_UART_Euridis_WakeUp_Init(void);
void	MX_USART1_UART_Custom_Init(void);

ITStatus	HAL_UART_GetUartReady( void );
void		HAL_UART_SetUartReady( ITStatus _ready );

void    MX_USART1_USART_DeInit( void );
void    MX_USART2_USART_DeInit( void );
void    MX_USART3_USART_DeInit( void );

void    MX_UART_EMUL_INIT( void );
void    MX_UART_EMUL_DEINIT( void );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

