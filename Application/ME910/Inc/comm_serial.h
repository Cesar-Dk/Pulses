/*
 * comm_serial.h
 *
 *  Created on: 15 jul. 2019
 *      Author: Sergio Mill�n L�pez
 */

#ifndef APPLICATION_ME910_INC_COMM_SERIAL_H_
#define APPLICATION_ME910_INC_COMM_SERIAL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"


#define kFLOW_CONTROL_THRESHOLD_BYTES   ( 512 )//( 350 )

#define PORT_RTS				( GPIOA )
#define PIN_RTS        			( GPIO_PIN_12 )
#define PORT_CTS				( GPIOA )
#define PIN_CTS        			( GPIO_PIN_11 )

#define DISABLE_NB_RTS()  		HAL_GPIO_WritePin( PORT_RTS, PIN_RTS, GPIO_PIN_SET )
#define ENABLE_NB_RTS()   		HAL_GPIO_WritePin( PORT_RTS, PIN_RTS, GPIO_PIN_RESET ) //Telit can send.

#define TXSIZE 					( 4096 )
#define RXSIZE 					(3*4096 + 2*512)//(2*4096)//(6*4096)//( 4096 )


#define	min( a, b )	(((a) < (b)) ? (a) : (b))
#define	max( a, b )	(((a) > (b)) ? (a) : (b))

void     comm_serial_disable_rts( void );
void     comm_serial_enable_rts( void );
uint32_t comm_serial_enable( void );
uint32_t comm_serial_disable( void );

uint32_t comm_serial_rcx( uint8_t *data );
uint32_t comm_serial_rcx_n( uint8_t *data, uint32_t n );
uint32_t comm_serial_rcx_bytes_n( void );
void     comm_serial_delete( uint32_t n );
void     comm_serial_rx_vars_init( void );
uint32_t comm_serial_receive_one_byte( UART_HandleTypeDef *uart );
uint32_t comm_serial_receive_buffer(UART_HandleTypeDef *uart);
uint32_t comm_serial_int_rx( UART_HandleTypeDef *uart );

uint32_t comm_serial_is_put_ready( void );
uint32_t comm_serial_trx( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len );
void 	 comm_serial_trx_header_msg( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len );
uint32_t comm_serial_trx_bytes_n( void );
uint32_t comm_serial_int_tx(UART_HandleTypeDef *uart);
uint32_t comm_serial_read_while_data(uint8_t *data);

uint8_t  comm_serial_get_NO_CARRIER( void );

#endif /* APPLICATION_ME910_INC_COMM_SERIAL_H_ */
