/*
 * comm_serial.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Sergio Mill�n L�pez
 */

#include <comm_serial.h>
#include <main.h>
#include <stm32u575xx.h>
#include <stm32u5xx_hal_def.h>
#include <stm32u5xx_hal_gpio.h>
#include <stm32u5xx_hal_rcc.h>
#include <stm32u5xx_hal_uart.h>
#include <sys/_stdint.h>
#include <math.h>
#include <usart.h>
#include "udp_protocol.h"
#include "tick.h"
#include "params.h"

typedef struct
{
    uint8_t data[ TXSIZE ];
    uint32_t n;
} comm_serial_tx_st;

typedef struct
{
	uint8_t data[ RXSIZE ];
	uint32_t n, start, end;
} comm_serial_rx_st;

comm_serial_rx_st comm_serial_rx;
comm_serial_tx_st comm_serial_tx;

uint32_t rts_nb_disabled;
uint32_t times_2nd_block = 0;
uint32_t uart_total_bytes = 0;
uint32_t rx_block = 0;

void comm_serial_disable_rts(void)
{
    if ( rts_nb_disabled == 0 ) {
        DISABLE_NB_RTS();
        rts_nb_disabled = 1;
    }
}

void comm_serial_enable_rts(void)
{
    if ( rts_nb_disabled == 1 ) {
        ENABLE_NB_RTS();
        rts_nb_disabled = 0;
    }
}

uint32_t comm_serial_enable( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	MX_USART1_UART_Custom_Init();
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, UC_NBIOT_UART_CTS_Pin | UC_NBIOT_UART_RTS_Pin,
			GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = UC_NBIOT_UART_CTS_Pin | UC_NBIOT_UART_RTS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	comm_serial_enable_rts();

	memset( comm_serial_rx.data, 0, sizeof( comm_serial_rx_st ));
	memset( comm_serial_tx.data, 0, sizeof( comm_serial_tx_st ));

	return 0;
}

uint32_t comm_serial_disable( void )
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	MX_USART1_USART_DeInit();

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = UC_NBIOT_UART_CTS_Pin | UC_NBIOT_UART_RTS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	comm_serial_disable_rts();

	return 0;
}

uint32_t comm_serial_rcx( uint8_t *data )
{
	uint32_t i, j, n;

	j = comm_serial_rx.start;
	n = comm_serial_rx.n;

	for( i = 0; i < n; i++ ) {
		data[ i ] = comm_serial_rx.data[ j ];
		if( j < ( RXSIZE - 1 ) ) {
			j++;
		}
		else {
			j = 0;
		}
	}

    return n;
}

uint32_t comm_serial_rcx_n( uint8_t *data, uint32_t n )
{
    uint32_t i, j;

    j = comm_serial_rx.start;
    n = min( n, comm_serial_rx.n );

    for( i = 0; i < n; i++ ) {
    	data[ i ] = comm_serial_rx.data[ j ];
		if( j < ( RXSIZE - 1 ) ) {
			j++;
		}
		else {
			j = 0;
		}
    }

    return n;
}

uint32_t comm_serial_rcx_bytes_n( void )
{
    return comm_serial_rx.n;
}

void comm_serial_delete( uint32_t n )
{
	uint32_t x;

	n = min( n, comm_serial_rx.n );
	x = comm_serial_rx.start + n;

	if( x > ( RXSIZE - 1 )) {
		comm_serial_rx.start = x - RXSIZE;
	}
    else {
    	comm_serial_rx.start = x;
    }

	comm_serial_rx.n -= n;

    if ((RXSIZE - comm_serial_rx.n) >= kFLOW_CONTROL_THRESHOLD_BYTES) {
    	comm_serial_enable_rts();
    }
}

void comm_serial_rx_vars_init( void )
{
	comm_serial_rx.end   = 0;
	comm_serial_rx.start = 0;
	comm_serial_rx.n     = 0;
}

uint32_t comm_serial_receive_one_byte(UART_HandleTypeDef *uart)
{
	HAL_UART_Receive_IT(uart, (uint8_t *) &comm_serial_rx.data[comm_serial_rx.end], 1);

	return 0;
}

uint32_t comm_serial_receive_buffer(UART_HandleTypeDef *uart)
{
	HAL_UART_Receive_IT(uart, (uint8_t *) &comm_serial_rx.data[comm_serial_rx.end], RXSIZE);

	return 0;
}

uint32_t comm_serial_int_rx(UART_HandleTypeDef *uart)
 {
	if (comm_serial_rx.end < ( RXSIZE - 1)) {
		comm_serial_rx.end++;
	} else {
		comm_serial_rx.end = 0;
	}
	if (comm_serial_rx.end == comm_serial_rx.start) {
		if (comm_serial_rx.start < ( RXSIZE - 1)) {
			comm_serial_rx.start++;
		} else {
			comm_serial_rx.start = 0;
		}
	} else {
		comm_serial_rx.n++;
	}
	if ((RXSIZE - comm_serial_rx.n) <= kFLOW_CONTROL_THRESHOLD_BYTES) {
		comm_serial_disable_rts();
	}

	HAL_StatusTypeDef error = HAL_OK;
	error = HAL_UART_Receive_IT(uart, (uint8_t *) &comm_serial_rx.data[comm_serial_rx.end], 1);
	if ( error != HAL_OK) {
		Error_Handler();
	}

	return 0;
}

uint32_t comm_serial_is_put_ready( void )
{
	if ( comm_serial_tx.n == 0 ) {
		return TXSIZE;
	}
	return 0;
}

uint32_t comm_serial_trx( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len )
{
	uint32_t i, j;

	comm_serial_tx.n = min( data_len, TXSIZE );

	for( i = 0, j = comm_serial_tx.n; j; i++, j-- ) {
		comm_serial_tx.data[ j - 1 ] = data[ i ];
	}

	while( HAL_GPIO_ReadPin( PORT_CTS, PIN_CTS ) ); 	// Espero CTS

	comm_serial_tx.n--;
	HAL_UART_SetUartReady(RESET);
	if (HAL_UART_Transmit_IT(uart, &comm_serial_tx.data[comm_serial_tx.n], 1)!= HAL_OK) {
		Error_Handler();
		return HAL_ERROR;
	}

	while( HAL_UART_GetUartReady() != SET){}
	HAL_UART_SetUartReady(RESET);
	return HAL_OK;
}

void comm_serial_trx_header_msg( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len )
{
	uint32_t i, j;

	comm_serial_tx.n = min( data_len + hdr_len, TXSIZE );

	for( i = 0, j = data_len; j; i++, j-- ) {
		comm_serial_tx.data[ j - 1 ] = data[ i ];
	}

	for( i = 0, j = hdr_len; j; i++, j-- ) {
		comm_serial_tx.data[ j - 1 + data_len ] = hdr[ i ];
	}

	if (HAL_UART_Transmit_IT(uart, &comm_serial_tx.data[comm_serial_tx.n], comm_serial_tx.n)!= HAL_OK) {
		Error_Handler();
	}
}

uint32_t comm_serial_trx_bytes_n( void )
{
    return comm_serial_tx.n;
}

uint32_t comm_serial_int_tx(UART_HandleTypeDef *uart)
{
	if( comm_serial_tx.n ) {
		comm_serial_tx.n--;
		if (HAL_UART_Transmit_IT(uart, &comm_serial_tx.data[comm_serial_tx.n], 1)!= HAL_OK) {
			Error_Handler();
		}
	} else {
		return 1;
	}

	return 0;
}

uint32_t comm_serial_read_while_data(uint8_t *data)
{
    uint32_t n = 0, pos  = 0;
    int32_t  num_bytes   = 0;
    uint32_t last_byte   = comm_serial_rx.end;
//    float_t  time_delay  = (float_t)( (float_t)( 10 * ( UDP_LENGTH - 1 * kFLOW_CONTROL_THRESHOLD_BYTES ) * 1000 ) / (float_t)(115200) );
    static uint32_t loop = 0;

    volatile uint32_t i = Tick_Get(MILLISECONDS);

    n = comm_serial_rcx_bytes_n();
//    if ( n < UDP_LENGTH/4 ) {
//    	HAL_Delay(100);
//    } else {
//    	HAL_Delay(20);
//    }
//    comm_serial_disable_rts();

    do {
    	loop++;
        comm_serial_enable_rts();
        HAL_Delay( 125 );//HAL_Delay( 2000 );//HAL_Delay( (uint32_t)( time_delay * (float_t)( (float_t)( UDP_LENGTH - num_bytes ) / (float_t)UDP_LENGTH )) );//HAL_Delay((uint32_t)time_delay);
        last_byte = comm_serial_rx.end;
        n = comm_serial_rcx_bytes_n();

        if (n > 0) {
            if ((n + pos - 1) < UDP_LENGTH) {
            	comm_serial_rcx_n(data + pos, n);
                comm_serial_delete(n);
                num_bytes += n;
            } else {
                times_2nd_block++;
                n = UDP_LENGTH - num_bytes;
                comm_serial_rcx_n(data + pos, n);
                comm_serial_delete(n);
                num_bytes += n;
            }
        }
        pos += n;
        LOGLIVE(LEVEL_1,"n:%d num:%d 2nd:%d rx_end:%d last:%d\r\n", (int)n, (int)num_bytes, (int)times_2nd_block, (int)comm_serial_rx.end,(int)last_byte);
//        HAL_Delay(2000);//HAL_Delay( (uint32_t)( time_delay * (float_t)( (float_t)( UDP_LENGTH - num_bytes ) / (float_t)UDP_LENGTH )) );//HAL_Delay((uint32_t)time_delay);
    }while( ( pos                 < UDP_LENGTH  )
         && ( comm_serial_rx.end != last_byte   )
          );

    volatile uint32_t diff = Tick_Get(MILLISECONDS) - i;
    (void)diff;

    uart_total_bytes += num_bytes;
    rx_block++;

    if ( uart_total_bytes >= 50000 ) {
    	asm("nop");
    }
    loop = 0;

//    comm_serial_enable_rts();
    return num_bytes;
}

////////////////////////////////////////////////////////////////////////////////

uint8_t comm_serial_get_NO_CARRIER( void )
{
    uint32_t i, j;
    uint8_t k;
    static int8_t str[ 11 ] = "NO CARRIER";

    i = comm_serial_rx.start;
    while( i != comm_serial_rx.end )
    {
        if( comm_serial_rx.data[ i ] == 'N' ) {
            j = i;
            for( k = 0; k < 10; k++ ) {
                if(( char ) comm_serial_rx.data[ j ] != str[ k ]) {
                	break;
                }
                if( j < ( RXSIZE - 1 )) {
                	j++;
                }
                else {
                	j = 0;
                }
            }
            if( k == 10 ) {
            	return 1;
            }
        }
        if( i < ( RXSIZE - 1 )) {
        	i++;
        }
        else {
        	i = 0;
        }
    }
    return 0;
}
