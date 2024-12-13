/**
  ******************************************************************************
  * @file           serial_modbus.c
  * @author 		Datakorum Development Team
  * @brief          Driver to handle low level modbus communication.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * serial_modbus.c
 *
 *  Created on: 30 may. 2019
 *      Author: smill
 */
#include "serial_modbus.h"
#include "modbus.h"
#include "usart.h"
#include "gpdma.h"
#include "params.h"
#include "leds.h"

static struct
{
    uint8_t  tx_data[ MODBUS_FRAME_SIZE ];
    uint32_t n;
} modbus_tx;

static struct
{
    uint8_t  rx_data[ MODBUS_FRAME_SIZE ];
    uint32_t n;
    uint32_t start;
    uint32_t end;
} modbus_rx;

uint8_t enable_rx_rs485 		  = 0;
uint8_t serial_modbus_initialized = 0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel6;
extern DMA_HandleTypeDef handle_GPDMA1_Channel7;


void serial_enable_rx_rs485( uint8_t enable_rx)
{
    enable_rx_rs485 = enable_rx;
}

uint8_t serial_get_enable_rx_rs485( void )
{
    return enable_rx_rs485;
}

uint8_t serial_modbus_is_initialized( void )
{
	return serial_modbus_initialized;
}

void serial_modbus_set_initialized( uint8_t initialized )
{
	serial_modbus_initialized = initialized;
}

uint8_t *serial_modbus_get_tx_data_pointer( void )
{
	return modbus_tx.tx_data;
}

uint8_t *serial_modbus_get_rx_data_pointer( void )
{
	return modbus_rx.rx_data;
}

uint32_t serial_modbus_get_num_tx_data( void )
{
	return modbus_tx.n;
}

uint32_t serial_modbus_get_num_rx_data( void )
{
	return modbus_rx.n;
}

uint32_t serial_modbus_get_buffer_size( void )
{
	return sizeof(modbus_rx.rx_data);
}
int8_t serial_modbus_write_modbus( uint8_t *data, uint32_t len )
{
    uint32_t i;

    if( 0 == len ) {
        return 1;
    }

    DE_TX_RS485();
    serial_enable_rx_rs485( 0 );

    /* Disable the peripheral */
    __HAL_DMA_DISABLE(&handle_GPDMA1_Channel7);

    if( modbus_tx.n || ( len > MODBUS_FRAME_SIZE )) {
        init_modbusTxBufferParams();
        return -1;
    }

    for( i = 0; i < len; i++ ) {
        modbus_tx.tx_data[ i ] = data[ i ];
    }
    modbus_tx.n = len;

#ifndef COMPILER
    modbus_set_send_lock(1);
#endif

//    DMA_SetCurrDataCounter( DMA1_Channel2, modbus_tx.n );
//    /* Clear the TC bit in the SR register by writing 0 to it */
//    USART_ClearFlag( USART5, USART_FLAG_TC );
//    /* Enable the DMA TX Stream */
//    DMA_Cmd( DMA1_Channel2, ENABLE );
//
//    USART_ITConfig( USART5, USART_IT_TC, ENABLE );

    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t*)modbus_tx.tx_data, modbus_tx.n)!= HAL_OK)
    {
      Error_Handler();
    }

    return 0;
}

__STATIC_INLINE uint8_t __serialModbusRxTask(void)
{
    uint32_t buffer_left = 0, buffer_end = 0;

    buffer_left = __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel6);
    buffer_end  = MODBUS_FRAME_SIZE - buffer_left;

    if ( ( buffer_end == modbus_rx.start ) || ( buffer_end >= MODBUS_FRAME_SIZE ) ) {
        return 1;
    }
    if ( buffer_end < modbus_rx.start ) {
        modbus_rx.n = MODBUS_FRAME_SIZE - modbus_rx.start + buffer_end;
    }
    else {
        modbus_rx.n = buffer_end - modbus_rx.end;
    }

    modbus_rx.end = buffer_end;

    return 0;
}

uint32_t serial_modbus_read_modbus( uint8_t *data )
{
    uint32_t i, n;

//    HAL_Delay(1000);
    n = __serialModbusRxTask();

    if( n == 0 ) {
        for( i = 0; i < modbus_rx.n; i++ ) {
            data[ i ] = modbus_rx.rx_data[ i ];
        }
    }

    n = modbus_rx.n;

    __HAL_DMA_DISABLE(&handle_GPDMA1_Channel6);
    __HAL_DMA_CLEAR_FLAG(&handle_GPDMA1_Channel6,  DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE );
    GPDMA1_Channel6->CDAR  = (uint32_t) modbus_rx.rx_data;
//    GPDMA1_Channel6->CBR1 = MODBUS_FRAME_SIZE;
    /* Configure the DMA channel data size */
    MODIFY_REG(GPDMA1_Channel6->CBR1, DMA_CBR1_BNDT, (MODBUS_FRAME_SIZE & DMA_CBR1_BNDT));
    __HAL_DMA_ENABLE(&handle_GPDMA1_Channel6);

    init_modbusRxBufferParams();

    return n;
}

void serial_modbus_DMA1_Channel7_TransferCmplt( DMA_HandleTypeDef * _hdma )
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if ( __HAL_DMA_GET_FLAG(_hdma, DMA_FLAG_TC) ) {
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		__HAL_DMA_CLEAR_FLAG(_hdma, DMA_FLAG_TC | DMA_FLAG_HT);
		modbus_tx.n = __HAL_DMA_GET_COUNTER(_hdma);
		serial_enable_rx_rs485(0);
	}
}

void serial_modbus_UART_txCplt_Callback( void )
{
	if (serial_get_enable_rx_rs485() == 0) {
//		__HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
		modbus_tx.n = __HAL_DMA_GET_COUNTER(&handle_GPDMA1_Channel7);
		DE_RX_RS485();
		serial_enable_rx_rs485(1);
		modbus_set_send_lock(0);
	}
}

void init_modbusTxBufferParams(void)
{
    modbus_tx.n = 0;
}

void init_modbusRxBufferParams(void)
{
    modbus_rx.n     = 0;
    modbus_rx.start = 0;
    modbus_rx.end   = 0;
}

/**
 * @fn void serialRS485_init(void)
 * @brief Enables RS485 driver.
 *
 */
void serialRS485_init( void )
{
	/** Clears buffers */
	init_modbusTxBufferParams();
	init_modbusRxBufferParams();

	/** Enables RS485 physical driver */
	ENABLE_RS485();
	HAL_Delay(1);
	DE_RX_RS485();

	/** Enables modbus initialized flag*/
	serial_modbus_initialized = 1;
}

/**
  * @brief  USART2 Custom Initialization.
  * @param  None
  * @retval None
  */
void serialRS485_uart_Init(uint32_t baudrate)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = baudrate;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.StopBits = UART_STOPBITS_1;

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
 * @fn void serialRS485_module_init(void)
 * @brief Low level RS485 serial interface initialization
 *
 */
void serialRS485_module_init(uint32_t baudrate)
{
	/** low level UART2 initialization */
//	MX_USART2_UART_Custom_Init();
	serialRS485_uart_Init(baudrate);
	/** Enables driver */
	serialRS485_init();
}

/**
 * @fn void serialRS485_deInit(void)
 * @brief Low level RS485 serial interface de-initialization
 *
 */
void serialRS485_deInit( void )
{
	/** Disables UART2 */
    __HAL_UART_DISABLE(&huart2);
    /** Disables uart2 DMA recepcion channel */
    __HAL_DMA_DISABLE(&handle_GPDMA1_Channel6);
    /** Disables physical driver power rail */
    DISABLE_RS485();

	serial_modbus_initialized = 0;
}

int serial_write_loglive(char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */

	for (int i=0 ; i<len ; i++)
	{
#ifdef LOGLIVE_MODE
		/* To use printf with huart3 for LOGLIVE */
		HAL_UART_Transmit(&huart2, (uint8_t *)ptr++, 1, 0xFF);
#endif
	}
  return len;
}
