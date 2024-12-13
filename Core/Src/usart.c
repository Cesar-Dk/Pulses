/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
//#include "stm32l4xx_hal_uart_emul.h"
#include "main.h"
#include "dlms_client.h"

/* UART Emulation handler declaration */
__IO ITStatus UartReady = RESET;
__IO uint32_t msp_severn_trent = 0;
__IO uint32_t msp_euridis = 0;
UART_Emul_HandleTypeDef UartEmulHandle;
extern TIM_HandleTypeDef htim3;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef handle_GPDMA1_Channel7;
DMA_HandleTypeDef handle_GPDMA1_Channel6;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  msp_euridis = 0;
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
	  /* USER CODE BEGIN USART1_MspInit 0 */
	  /* USER CODE END USART1_MspInit 0 */

	  /** Initializes the peripherals clock
	   */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
		  Error_Handler();
	  }

	  /* USART1 clock enable */
	  __HAL_RCC_USART1_CLK_ENABLE();

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  /**USART1 GPIO Configuration
    	  PA9     ------> USART1_TX
    	  PA10     ------> USART1_RX
	   */
	  GPIO_InitStruct.Pin = UC_NBIOT_UART_TX_Pin|UC_NBIOT_UART_RX_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* USART1 interrupt Init */
	  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);
	  /* USER CODE BEGIN USART1_MspInit 1 */
	  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = UC_MODBUS_TRX_TX_Pin|UC_MODBUS_TRX_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* GPDMA1_REQUEST_USART2_TX Init */
    handle_GPDMA1_Channel7.Instance = GPDMA1_Channel7;
    handle_GPDMA1_Channel7.Init.Request = GPDMA1_REQUEST_USART2_TX;
    handle_GPDMA1_Channel7.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel7.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel7.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel7.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA1_Channel7.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel7.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel7.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
    handle_GPDMA1_Channel7.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel7.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel7.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel7.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel7.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel7) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmatx, handle_GPDMA1_Channel7);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel7, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA1_REQUEST_USART2_RX Init */
    handle_GPDMA1_Channel6.Instance = GPDMA1_Channel6;
    handle_GPDMA1_Channel6.Init.Request = GPDMA1_REQUEST_USART2_RX;
    handle_GPDMA1_Channel6.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel6.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel6.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA1_Channel6.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel6.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel6.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel6.Init.Priority = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel6.Init.SrcBurstLength = 1;
    handle_GPDMA1_Channel6.Init.DestBurstLength = 1;
    handle_GPDMA1_Channel6.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_GPDMA1_Channel6.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel6.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel6) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmarx, handle_GPDMA1_Channel6);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel6, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
	  /* USER CODE BEGIN USART3_MspInit 0 */
	  if ( 0 == msp_severn_trent )
	  {
		  /* USER CODE END USART3_MspInit 0 */

		  /** Initializes the peripherals clock
		   */
		  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
		  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
		  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		  {
			  Error_Handler();
		  }

		  /* USART3 clock enable */
		  __HAL_RCC_USART3_CLK_ENABLE();

		  __HAL_RCC_GPIOA_CLK_ENABLE();
		  __HAL_RCC_GPIOB_CLK_ENABLE();
		  /**USART3 GPIO Configuration
    			PA5     ------> USART3_RX
    			PB10     ------> USART3_TX
		   */
		  GPIO_InitStruct.Pin = UC_MBUS_RX_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		  HAL_GPIO_Init(UC_MBUS_RX_GPIO_Port, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = UC_MBUS_TX_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		  HAL_GPIO_Init(UC_MBUS_TX_GPIO_Port, &GPIO_InitStruct);

		  /* USART3 interrupt Init */
		  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
		  HAL_NVIC_EnableIRQ(USART3_IRQn);
		  /* USER CODE BEGIN USART3_MspInit 1 */
#if defined(UNE82326)
		  serial_une82326_pwr_enable();
#elif defined(MBUS)
		  serial_mbus_pwr_enable();
#endif
	  }
	  else if ( 1 == msp_severn_trent )
	  {
		  /* USER CODE END USART3_MspInit 0 */
		  /* USART3 clock enable */
		  __HAL_RCC_USART3_CLK_ENABLE();

		  __HAL_RCC_GPIOA_CLK_ENABLE();
		  __HAL_RCC_GPIOB_CLK_ENABLE();
#if defined(MBUS)
		  /**USART3 GPIO Configuration
    			PB10     ------> USART3_TX
    			PB11     ------> USART3_RX
		   */
		  GPIO_InitStruct.Pin    = UC_MBUS_TX_Pin;
		  GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
		  GPIO_InitStruct.Pull   = GPIO_PULLUP;
		  GPIO_InitStruct.Speed  = GPIO_SPEED_HIGH;
		  HAL_GPIO_WritePin(PORT_MBUS_TX, PIN_MBUS_TX, GPIO_PIN_RESET);
		  HAL_GPIO_Init(PORT_MBUS_TX, &GPIO_InitStruct);

		  GPIO_InitStruct.Pin = UC_MBUS_RX_Pin;
		  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		  HAL_GPIO_Init(UC_MBUS_RX_GPIO_Port, &GPIO_InitStruct);
#endif
	  }
	  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, UC_NBIOT_UART_TX_Pin|UC_NBIOT_UART_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, UC_MODBUS_TRX_TX_Pin|UC_MODBUS_TRX_RX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PA5     ------> USART3_RX
    PB10     ------> USART3_TX
    */
    HAL_GPIO_DeInit(UC_MBUS_RX_GPIO_Port, UC_MBUS_RX_Pin);

    HAL_GPIO_DeInit(UC_MBUS_TX_GPIO_Port, UC_MBUS_TX_Pin);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void MX_USART1_USART_DeInit( void )
{
	HAL_UART_DeInit(&huart1);
}

void MX_USART2_USART_DeInit( void )
{
	HAL_UART_DeInit(&huart2);
}

void MX_USART3_USART_DeInit( void )
{
	if (huart3.Instance == USART3)
	{
		HAL_UART_DeInit(&huart3);
	}
}

void MX_UART_EMUL_INIT( void )
{
	uint32_t baudrate = 1180;
	if ( ( params_get_mbus_baudrate() != 2400 ) && ( params_get_mbus_baudrate() != 0 ) && ( params_get_mbus_baudrate() != 9600 ) )
	{
		baudrate = params_get_mbus_baudrate();
	}
	params_set_mbus_baudrate(baudrate);
	/*## Configure the UART Emulation for UNE82326 Protocol  ######################################*/
	/* UART Emulation configured as follow:
	      - Word Length = 7 Bits
	      - Stop Bit = Two Stop bit
	      - BaudRate = 1200 baud
	      - Parity = None
	 */

	UartEmulHandle.Init.Mode        = UART_EMUL_MODE_RX;//UART_EMUL_MODE_TX_TX;
	UartEmulHandle.Init.BaudRate    = baudrate;//1180;//1170;
	UartEmulHandle.Init.StopBits    = UART_EMUL_STOPBITS_2;
	UartEmulHandle.Init.Parity      = UART_EMUL_PARITY_NONE;
	UartEmulHandle.Init.WordLength  = UART_EMUL_WORDLENGTH_7B;

	if (HAL_UART_Emul_Init(&UartEmulHandle) != HAL_OK)
	{
		Error_Handler();
	}
#if defined(UNE82326)
	serial_une82326_rx_vars_init();
#elif defined(MBUS)
	serial_mbus_rx_vars_init();
#endif
#ifdef UART_EMUL
	serial_une82326_receive_buffer(&UartEmulHandle);//serial_une82326_receive_one_byte(&UartEmulHandle);
#endif
}

void MX_UART_EMUL_DEINIT( void )
{
	HAL_UART_Emul_DeInit(&UartEmulHandle);
}

/**
  * @brief UART Emulation MSP Initialization
  *        This function configures the UART Emulation resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
 *           - Port for UART Emulation
 *           - Pin Tx and Rx
  * @param  htim: UART Emulation handle pointer
  * @retval None
  */
void HAL_UART_Emul_MspInit(UART_Emul_HandleTypeDef *huart)
{
  GPIO_InitTypeDef   GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #######################*/
  /* Enable clock for UART Emul */
  __UART_EMUL_CLK_ENABLE();

  /* Enable GPIO TX/RX clock */
  UART_EMUL_TX_GPIO_CLK_ENABLE();
  UART_EMUL_RX_GPIO_CLK_ENABLE();

  /* Initialize UART Emulation port name */
//  UartEmulHandle.TxPortName = UART_EMUL_TX_PORT;
  UartEmulHandle.RxPortName = UART_EMUL_RX_PORT;

  /*Initialize UART Emulation pin number for Tx */
  UartEmulHandle.Init.RxPinNumber = UART_EMUL_RX_PIN;
//  UartEmulHandle.Init.TxPinNumber = UART_EMUL_TX_PIN;

//  /* Configure GPIOA 9 for UART Emulation Tx */
//  GPIO_InitStruct.Pin    = UART_EMUL_TX_PIN;
//  GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull   = GPIO_NOPULL;
//  GPIO_InitStruct.Speed  = GPIO_SPEED_HIGH;
//
//  HAL_GPIO_Init(UART_EMUL_TX_PORT, &GPIO_InitStruct);

  /* Configure GPIOA 10 for UART Emulation Rx */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pin  = UART_EMUL_RX_PIN;

  HAL_GPIO_Init(UART_EMUL_RX_PORT, &GPIO_InitStruct);

  /*##-2- Enable NVIC for line Rx  #################################*/
  /* Enable and set EXTI Line Interrupt to the highest priority */
  HAL_NVIC_SetPriority(UART_EMUL_EXTI_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(UART_EMUL_EXTI_IRQ);
}

/**
 * @brief  UART Emulation MSP DeInit.
 * @param  huart: UART Emulation handle
 * @retval None
 */
void HAL_UART_Emul_MspDeInit(UART_Emul_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	UART_Emul_Disable();

//	/* Configure GPIOA 15 for UART Emulation Tx */
//	GPIO_InitStruct.Pin    = UART_EMUL_TX_PIN;
//	GPIO_InitStruct.Mode   = GPIO_MODE_ANALOG;
//	GPIO_InitStruct.Pull   = GPIO_NOPULL;
//	HAL_GPIO_Init(UART_EMUL_TX_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin    = UART_EMUL_RX_PIN;
	GPIO_InitStruct.Mode   = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull   = GPIO_NOPULL;
	HAL_GPIO_Init(UART_EMUL_RX_PORT, &GPIO_InitStruct);
}

ITStatus HAL_UART_GetUartReady( void )
{
	return UartReady;
}

void HAL_UART_SetUartReady( ITStatus _ready)
{
	UartReady = _ready;
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if ( UartHandle->Instance == USART3 )
	{
#ifdef MBUS
		if ( 1 == serial_mbus_int_tx(UartHandle) )
		{
			HAL_UART_SetUartReady(SET);
		}
#endif
	}
	else if (UartHandle->Instance == USART2)
	{
		serial_modbus_UART_txCplt_Callback();
	}
	else if (UartHandle->Instance == USART1)
	{
		if ( 1 == comm_serial_int_tx(UartHandle) )
		{
			HAL_UART_SetUartReady(SET);
		}
	}
//  /* Set transmission flag: transfer complete */
//  UartReady = SET;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if ( UartHandle->Instance == USART3 ) {
#if defined (UNE82326)
#ifndef UART_EMUL
		serial_une82326_int_rx(UartHandle);
		if ( serial_une82326_rcx_bytes_n() == 1 ) {
			if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
				/* Starting Error */
				asm("nop");
			}
		}

#endif
#elif defined (MBUS)
		serial_mbus_int_rx(UartHandle);
#endif
	} else if ( UartHandle->Instance == USART2 ) {

	} else if ( UartHandle->Instance == USART1 ) {
		comm_serial_int_rx(UartHandle);
	}
}

void HAL_UART_Emul_RxCpltCallback(UART_Emul_HandleTypeDef *UartHandle)
{
#ifdef UART_EMUL
	serial_une82326_int_rx(UartHandle);
	serial_une82326_set_start_bit(0);
#endif
}

void HAL_UART_Emul_TxCpltCallback(UART_Emul_HandleTypeDef *UartHandle)
{
#ifdef UART_EMUL
	serial_une82326_int_tx(UartHandle);
#endif
}
/**
  * @brief  UART Emulation error callbacks.
  * @param  huart: UART Emulation handle
  * @retval None
  */
void HAL_UART_Emul_ErrorCallback(UART_Emul_HandleTypeDef *huart)
{
#ifdef UNE82326
	serial_une82326_unsel();
#endif
}


/**
  * @brief  USART3 Custom Initialization.
  * @param  None
  * @retval None
  */
void MX_USART3_UART_Custom_Init(void)
{
  uint32_t baudrate = 9600;
  if ( ( 1200 == params_get_mbus_baudrate() ) || ( 9600 == params_get_mbus_baudrate() ) || ( 2400 == params_get_mbus_baudrate() ) ) {
	  baudrate = params_get_mbus_baudrate();
  } else {
	  baudrate = 2400;
  }
  msp_severn_trent = 0;

  huart3.Instance                    = USART3;
  huart3.Init.BaudRate               = baudrate;
  huart3.Init.WordLength             = UART_WORDLENGTH_9B;
  huart3.Init.StopBits               = UART_STOPBITS_1;
  huart3.Init.Parity                 = UART_PARITY_EVEN;
  huart3.Init.Mode                   = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_DeInit(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
#if defined(UNE82326)
  serial_une82326_rx_vars_init();
#elif defined (MBUS)
  serial_mbus_rx_vars_init();
//  serial_mbus_receive_buffer(&huart1);
  serial_mbus_receive_one_byte(&huart3);
#endif
}

/**
  * @brief  USART3 Custom Initialization.
  * @param  None
  * @retval None
  */
void MX_USART3_UART_Custom_Severn_Trent_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */
	msp_severn_trent = 1;
	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 1200;
	huart3.Init.WordLength = UART_WORDLENGTH_7B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_EVEN;
	huart3.Init.Mode = UART_MODE_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_DeInit(&huart3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	serial_mbus_rx_vars_init();
	serial_mbus_receive_one_byte(&huart3);
	/* USER CODE END USART1_Init 2 */

}

/**
  * @brief  USART2 Custom Initialization.
  * @param  None
  * @retval None
  */
void MX_USART2_UART_Custom_Init(void)
{
  uint32_t baud_rate = modbus_sensors_get_serial_config_baud_rate();
  huart2.Instance = USART2;
  if ( 0 == baud_rate ) {
	  huart2.Init.BaudRate = 115200;
  } else {
	  huart2.Init.BaudRate = baud_rate;
  }
  huart2.Init.WordLength = UART_WORDLENGTH_8B;

  huart2.Init.Parity = UART_PARITY_NONE;
  if ( serial_8N2 == modbus_sensors_get_serial_config_stop_bits() ) {
	  huart2.Init.StopBits = UART_STOPBITS_2;
	  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  } else {
	  huart2.Init.StopBits = UART_STOPBITS_1;
  }
  if ( no_parity == modbus_sensors_get_serial_config_parity()) {
	  huart2.Init.Parity = UART_PARITY_NONE;
  } else if ( parity_even == modbus_sensors_get_serial_config_parity()) {
	  huart2.Init.WordLength = UART_WORDLENGTH_9B;
	  huart2.Init.Parity = UART_PARITY_EVEN;
  } else if ( parity_odd == modbus_sensors_get_serial_config_parity()) {
	  huart2.Init.WordLength = UART_WORDLENGTH_9B;
	  huart2.Init.Parity = UART_PARITY_ODD;
  }
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_DeInit(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  serialRS485_init();
  /*##- Program the Reception process #####################################*/
  if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)serial_modbus_get_rx_data_pointer(), serial_modbus_get_buffer_size()) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
 * @fn void MX_USART2_UART_DLMS_Init(void)
 * @brief
 *
 * @pre
 * @post
 */
void MX_USART2_UART_DLMS_Init(void)
{
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = dlms_client_get_baudrate();//19200//9600;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


/**
  * @brief  USART3 Custom Initialization.
  */
void MX_USART1_UART_Custom_Init(void)
{

	msp_euridis = 0;
	/**
	 * Deinit UART and GPIO
	 */
	if (huart1.Instance == USART1)
	{
		HAL_UART_DeInit(&huart1);
	}

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if(HAL_UART_DeInit(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Once the COMM UART is initialized, start an asynchronous recursive
   listening. the HAL_UART_Receive_IT() call below will wait until one char is
   received to trigger the HAL_UART_RxCpltCallback(). The latter will recursively
   call the former to read another char.  */
	comm_serial_rx_vars_init();
	comm_serial_receive_one_byte(&huart1);//comm_serial_receive_buffer(&huart3);//
}

void HAL_UART_Custom_MspInit(UART_HandleTypeDef* uartHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(uartHandle->Instance==USART3)
	{
		/* USER CODE BEGIN USART3_MspInit 0 */
		if ( 0 == msp_severn_trent )
		{
			/* USER CODE END USART3_MspInit 0 */
			/* USART3 clock enable */
			__HAL_RCC_USART3_CLK_ENABLE();

			__HAL_RCC_GPIOB_CLK_ENABLE();
			/**USART3 GPIO Configuration
    			PB10     ------> USART3_TX
    			PB11     ------> USART3_RX
			 */
			GPIO_InitStruct.Pin = UC_MBUS_TX_Pin|UC_MBUS_RX_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
			HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			/* USER CODE BEGIN USART3_MspInit 1 */
#if defined(UNE82326)
			serial_une82326_pwr_enable();
#elif defined(MBUS)
			serial_mbus_pwr_enable();
#endif
		}
		else if ( 1 == msp_severn_trent )
		{
			/* USER CODE END USART3_MspInit 0 */
			/* USART3 clock enable */
			__HAL_RCC_USART3_CLK_ENABLE();

			__HAL_RCC_GPIOB_CLK_ENABLE();
#if defined(MBUS)
			/**USART3 GPIO Configuration
    			PB10     ------> USART3_TX
    			PB11     ------> USART3_RX
			 */
			GPIO_InitStruct.Pin    = UC_MBUS_TX_Pin;
			GPIO_InitStruct.Mode   = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull   = GPIO_PULLUP;
			GPIO_InitStruct.Speed  = GPIO_SPEED_HIGH;
			HAL_GPIO_WritePin(PORT_MBUS_TX, PIN_MBUS_TX, GPIO_PIN_RESET);
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

			GPIO_InitStruct.Pin = UC_MBUS_RX_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif
		}
		/* USER CODE END USART1_MspInit 1 */
	}
	else if(uartHandle->Instance==USART2)
	{
		/* USER CODE BEGIN USART2_MspInit 0 */

		/* USER CODE END USART2_MspInit 0 */
		/* USART2 clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
	    /**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA3     ------> USART2_RX
	    */
	    GPIO_InitStruct.Pin = UC_MODBUS_TRX_TX_Pin|UC_MODBUS_TRX_RX_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    /* USART2 DMA Init */
	    /* GPDMA1_REQUEST_USART2_TX Init */
	    handle_GPDMA1_Channel7.Instance = GPDMA1_Channel7;
	    handle_GPDMA1_Channel7.Init.Request = GPDMA1_REQUEST_USART2_TX;
	    handle_GPDMA1_Channel7.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
	    handle_GPDMA1_Channel7.Init.Direction = DMA_MEMORY_TO_PERIPH;
	    handle_GPDMA1_Channel7.Init.SrcInc = DMA_SINC_INCREMENTED;
	    handle_GPDMA1_Channel7.Init.DestInc = DMA_DINC_FIXED;
	    handle_GPDMA1_Channel7.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
	    handle_GPDMA1_Channel7.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
	    handle_GPDMA1_Channel7.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
	    handle_GPDMA1_Channel7.Init.SrcBurstLength = 1;
	    handle_GPDMA1_Channel7.Init.DestBurstLength = 1;
	    handle_GPDMA1_Channel7.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1|DMA_DEST_ALLOCATED_PORT0;
	    handle_GPDMA1_Channel7.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
	    handle_GPDMA1_Channel7.Init.Mode = DMA_NORMAL;
	    if (HAL_DMA_Init(&handle_GPDMA1_Channel7) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    __HAL_LINKDMA(uartHandle, hdmatx, handle_GPDMA1_Channel7);

	    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel7, DMA_CHANNEL_NPRIV) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    /* GPDMA1_REQUEST_USART2_RX Init */
	    handle_GPDMA1_Channel6.Instance = GPDMA1_Channel6;
	    handle_GPDMA1_Channel6.Init.Request = GPDMA1_REQUEST_USART2_RX;
	    handle_GPDMA1_Channel6.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
	    handle_GPDMA1_Channel6.Init.Direction = DMA_PERIPH_TO_MEMORY;
	    handle_GPDMA1_Channel6.Init.SrcInc = DMA_SINC_FIXED;
	    handle_GPDMA1_Channel6.Init.DestInc = DMA_DINC_INCREMENTED;
	    handle_GPDMA1_Channel6.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
	    handle_GPDMA1_Channel6.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
	    handle_GPDMA1_Channel6.Init.Priority = DMA_HIGH_PRIORITY;
	    handle_GPDMA1_Channel6.Init.SrcBurstLength = 1;
	    handle_GPDMA1_Channel6.Init.DestBurstLength = 1;
	    handle_GPDMA1_Channel6.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
	    handle_GPDMA1_Channel6.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
	    handle_GPDMA1_Channel6.Init.Mode = DMA_NORMAL;
	    if (HAL_DMA_Init(&handle_GPDMA1_Channel6) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    __HAL_LINKDMA(uartHandle, hdmarx, handle_GPDMA1_Channel6);

	    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel6, DMA_CHANNEL_NPRIV) != HAL_OK)
	    {
	      Error_Handler();
	    }

		/* USER CODE BEGIN USART2_MspInit 1 */
		/* DMA1_Channel6_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(GPDMA1_Channel6_IRQn, 3, 3);
		HAL_NVIC_EnableIRQ(GPDMA1_Channel6_IRQn);
		/* DMA1_Channel7_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(GPDMA1_Channel7_IRQn, 3, 3);
		HAL_NVIC_EnableIRQ(GPDMA1_Channel7_IRQn);
		/* USART2_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(USART2_IRQn, 3, 3);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		/* USER CODE END USART2_MspInit 1 */
	}
	else if(uartHandle->Instance==USART1)
	{
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */
		/* USART1 clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART1 GPIO Configuration
    		PA9     ------> USART3_TX
    		PA10    ------> USART3_RX
		 */
		GPIO_InitStruct.Pin = UC_NBIOT_UART_TX_Pin|UC_NBIOT_UART_RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN USART1_MspInit 1 */

		/* USER CODE END USART1_MspInit 1 */
	}
}

/* USER CODE END 1 */
