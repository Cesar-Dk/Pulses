/*
 * serial_une82326.c
 *
 *  Created on: 18 jul. 2019
 *      Author: smill
 */

#include <main.h>
#include <serial_une82326.h>
#include <stm32u5xx_hal_def.h>
#include "tick.h"

#ifdef UNE82326
#define	min( a, b )	(((a) < (b)) ? (a) : (b))
#define	max( a, b )	(((a) > (b)) ? (a) : (b))

static struct
{
    uint8_t data[ UNE_TXSIZE ];
    uint32_t n;
} serial_une82326_tx;

static struct
{
	uint8_t data[ UNE_RXSIZE ];
	uint32_t n, start, end;
} serial_une82326_rx;

static uint32_t start_bit;

extern uint32_t unselected;

#ifndef UART_EMUL
static uint32_t une82326_RC_FLAG;
uint32_t serial_une82326_get_RC_FLAG( void )
{
	return une82326_RC_FLAG;
}

void serial_une82326_set_RC_FLAG( uint32_t  _une82326_RC_FLAG )
{
	une82326_RC_FLAG = _une82326_RC_FLAG;
}
#endif

void serial_une82326_pwr_enable( void )
{
//	UNE_TX_DISABLE();
	UNE_PWR_ENABLE();
}

void serial_une82326_pwr_disable( void )
{
	UNE_PWR_DISABLE();
}

void serial_une82326_sel( void )
{
	unselected = 0;
	UNE_SEL();
}

void serial_une82326_unsel( void )
{
	unselected = 1;
	UNE_UNSEL();
}

void serial_une82326_set_start_bit( uint32_t _start_bit )
{
	start_bit = _start_bit;
}

uint32_t serial_une82326_get_start_bit( void )
{
	return start_bit;
}

uint8_t *serial_une82326_rx_data( void )
{
	return serial_une82326_rx.data;
}

uint8_t *serial_une82326_rx_data_pointer( void )
{
	return serial_une82326_rx.data + serial_une82326_rx.start;
}

uint8_t *serial_une82326_tx_data_pointer( void )
{
	return serial_une82326_tx.data;
}

uint32_t serial_une82326_rx_num( void )
{
	return serial_une82326_rx.n;
}

uint32_t serial_une82326_tx_num( void )
{
	return serial_une82326_tx.n;
}

uint32_t serial_une82326_rcx( uint8_t *data )
{
	uint32_t i, j, n;

	j = serial_une82326_rx.start;
	n = serial_une82326_rx.n;

	for( i = 0; i < n; i++ ) {
		data[ i ] = serial_une82326_rx.data[ j ];
		if( j < ( UNE_RXSIZE - 1 ) ) {
			j++;
		}
		else {
			j = 0;
		}
	}

    return n;
}

uint32_t serial_une82326_rcx_n( uint8_t *data, uint8_t n )
{
    uint32_t i, j;

    j = serial_une82326_rx.start;
    n = min( n, serial_une82326_rx.n );

    for( i = 0; i < n; i++ ) {
    	data[ i ] = serial_une82326_rx.data[ j ];
		if( j < ( UNE_RXSIZE - 1 ) ) {
			j++;
		}
		else {
			j = 0;
		}
    }

    return n;
}

uint32_t serial_une82326_rcx_bytes_n( void )
{
    return serial_une82326_rx.n;
}

void serial_une82326_set_rcx_bytes_n( uint32_t n )
{
    serial_une82326_rx.n = n;
}

void serial_une82326_delete( uint32_t n )
{
	uint32_t x;

	n = min( n, serial_une82326_rx.n );
	x = serial_une82326_rx.start + n;

	if( x > ( UNE_RXSIZE - 1 )) {
		serial_une82326_rx.start = x - UNE_RXSIZE;
	}
    else {
    	serial_une82326_rx.start = x;
    }

	serial_une82326_rx.n -= n;

}

void serial_une82326_rx_vars_init( void )
{
	serial_une82326_rx.end = 0;
	serial_une82326_rx.start = 0;
	serial_une82326_rx.n = 0;
	memset( serial_une82326_rx.data, 0 , sizeof(serial_une82326_rx.data) );
}

#ifndef UART_EMUL
uint32_t serial_une82326_receive_one_byte(UART_HandleTypeDef *uart)
#else
uint32_t serial_une82326_receive_one_byte(UART_Emul_HandleTypeDef *uart)
#endif
{
#ifndef UART_EMUL
	HAL_UART_Receive_IT(uart, (uint8_t *) &serial_une82326_rx.data[serial_une82326_rx.end], 1);
#else
	/*## Put UART Emulation in reception process ###########################*/
	if ( HAL_UART_Emul_Receive_DMA( uart, (uint8_t *)&serial_une82326_rx.data[serial_une82326_rx.end], 1 ) != HAL_OK ) {
		Error_Handler();
	}
#endif

	return 0;
}

#ifndef UART_EMUL
uint32_t serial_une82326_receive_buffer(UART_HandleTypeDef *uart)
#else
uint32_t serial_une82326_receive_buffer(UART_Emul_HandleTypeDef *uart)
#endif
{
#ifndef UART_EMUL
	/*## Put UART Emulation in reception process ###########################*/
	if ( HAL_UART_Receive_IT( uart, (uint8_t *)&serial_une82326_rx.data[serial_une82326_rx.end], UNE_RXSIZE ) != HAL_OK ) {
		Error_Handler();
	}
#else
	/*## Put UART Emulation in reception process ###########################*/
	if ( HAL_UART_Emul_Receive_DMA( uart, (uint8_t *)&serial_une82326_rx.data[serial_une82326_rx.end], UNE_RXSIZE ) != HAL_OK ) {
		Error_Handler();
	}
#endif
	return 0;
}

#ifndef UART_EMUL
void serial_une2326_wait_rx(UART_HandleTypeDef *uart)
#else
uint8_t serial_une2326_wait_rx( UART_Emul_HandleTypeDef *uart )
#endif
{
#ifdef UART_EMUL
    uint32_t elapsed = 0;

	do {
		HAL_Delay(1000);
		elapsed++;
	} while( ( elapsed < 10 ) && (__HAL_UART_EMUL_GET_FLAG(uart, UART_EMUL_FLAG_RC) != SET) );
//	  while (__HAL_UART_EMUL_GET_FLAG(uart, UART_EMUL_FLAG_RC) != SET)
//	  {}
	  /* Clear Reception Complete Flag. */
	  __HAL_UART_EMUL_CLEAR_FLAG(uart, UART_EMUL_FLAG_RC);

	  if ( elapsed >= 10) {
		  return 1;
	  } else {
		  return 0;
	  }
#else
	  while ( serial_une82326_get_RC_FLAG() != 1 )
	  {}
#endif
}

#ifndef UART_EMUL
uint32_t serial_une82326_int_rx(UART_HandleTypeDef *uart)
#else
uint32_t serial_une82326_int_rx(UART_Emul_HandleTypeDef *uart)
#endif
{
	if (serial_une82326_rx.end < ( UNE_RXSIZE - 1)) {
		serial_une82326_rx.end++;
	} else {
		serial_une82326_rx.end = 0;
	}
	if (serial_une82326_rx.end == serial_une82326_rx.start) {
		if (serial_une82326_rx.start < ( UNE_RXSIZE - 1)) {
			serial_une82326_rx.start++;
		} else {
			serial_une82326_rx.start = 0;
		}
	} else {
		serial_une82326_rx.n++;
	}

#ifndef UART_EMUL
	/*## Put UART in reception process ###########################*/
	serial_une82326_set_RC_FLAG(0);
	if (HAL_UART_Receive_IT(uart, (uint8_t *) &serial_une82326_rx.data[serial_une82326_rx.end], UNE_RXSIZE) != HAL_OK) {
		Error_Handler();
	}
#else
//	/*## Put UART Emulation in reception process ###########################*/
//	if ( HAL_UART_Emul_Receive_DMA( uart, (uint8_t *)&serial_une82326_rx.data[serial_une82326_rx.end], 1 ) != HAL_OK ) {
//		Error_Handler();
//	}
#endif
	return 0;
}

#ifndef UART_EMUL
void serial_une2326_wait_rx(UART_HandleTypeDef *uart)
#else
void serial_une2326_wait_tx( UART_Emul_HandleTypeDef *uart )
#endif
{
#ifdef UART_EMUL
	  while (__HAL_UART_EMUL_GET_FLAG(uart, UART_EMUL_FLAG_TC) != SET)
	  {}
	  /* Clear Reception Complete Flag. */
	  __HAL_UART_EMUL_CLEAR_FLAG(uart, UART_EMUL_FLAG_TC);
#else
	  while ( serial_une82326_get_RC_FLAG() != 1 )
	  {}
#endif
}

#ifndef UART_EMUL
void serial_une82326_trx( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len )
#else
void serial_une82326_trx( UART_Emul_HandleTypeDef *uart, uint8_t *data, uint32_t data_len )
#endif
{
	uint32_t i;
//	uint32_t j;

	serial_une82326_tx.n = min( data_len, UNE_TXSIZE );

//	for( i = 0, j = serial_une82326_tx.n; j; i++, j-- ) {
//		serial_une82326_tx.data[ j - 1 ] = data[ i ];
//	}
	for( i = 0; i<serial_une82326_tx.n; i++ ) {
		serial_une82326_tx.data[ i ] = data[ i ];
	}

#ifndef UART_EMUL
	if (HAL_UART_Transmit_IT(uart, serial_une82326_tx.data, serial_une82326_tx.n)!= HAL_OK) {
		Error_Handler();
	}
#else
	/*## Start the transmission process #####################################*/
	if (HAL_UART_Emul_Transmit_DMA(uart, serial_une82326_tx.data, serial_une82326_tx.n) != HAL_OK) {
		Error_Handler();
	}
#endif
}

#ifndef UART_EMUL
void serial_une82326_trx_header_msg( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len )
#else
void serial_une82326_trx_header_msg( UART_Emul_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len )
#endif
{
	uint32_t i, j;

	serial_une82326_tx.n = min( data_len + hdr_len, UNE_TXSIZE );

	for( i = 0, j = data_len; j; i++, j-- ) {
		serial_une82326_tx.data[ j - 1 ] = data[ i ];
	}

	for( i = 0, j = hdr_len; j; i++, j-- ) {
		serial_une82326_tx.data[ j - 1 + data_len ] = hdr[ i ];
	}

#ifndef UART_EMUL
	if (HAL_UART_Transmit_IT(uart, &serial_une82326_tx.data[serial_une82326_tx.n], serial_une82326_tx.n)!= HAL_OK) {
		Error_Handler();
	}
#else
	/*## Start the transmission process #####################################*/
	if (HAL_UART_Emul_Transmit_DMA(uart, &serial_une82326_tx.data[serial_une82326_tx.n], serial_une82326_tx.n) != HAL_OK) {
		Error_Handler();
	}
#endif
}

uint32_t serial_une82326_trx_bytes_n( void )
{
    return serial_une82326_tx.n;
}

#ifndef UART_EMUL
uint32_t serial_une82326_int_tx(UART_HandleTypeDef *uart)
#else
uint32_t serial_une82326_int_tx(UART_Emul_HandleTypeDef *uart)
#endif
{
	if( serial_une82326_tx.n ) {
		serial_une82326_tx.n--;
#ifndef UART_EMUL
		if (HAL_UART_Transmit_IT(uart, &serial_une82326_tx.data[serial_une82326_tx.n], 1)!= HAL_OK) {
			Error_Handler();
		}
#else
//	/*## Start the transmission process #####################################*/
//	if (HAL_UART_Emul_Transmit_DMA(uart, &serial_une82326_tx.data[serial_une82326_tx.n], serial_une82326_tx.n) != HAL_OK) {
//		Error_Handler();
//	}
#endif
	}

	return 0;
}
#endif
