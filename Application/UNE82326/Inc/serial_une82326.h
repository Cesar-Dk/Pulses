/*
 * serial_une82326.h
 *
 *  Created on: 18 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_UNE82326_INC_SERIAL_UNE82326_H_
#define APPLICATION_UNE82326_INC_SERIAL_UNE82326_H_

/* Includes ------------------------------------------------------------------*/

#include <stm32u5xx_hal_uart.h>
#include <sys/_stdint.h>

#include "params.h"

#ifdef UNE82326

#define UART_EMUL

#define PORT_UNE_SEL			( GPIOA )
#define PIN_UNE_SEL        		( GPIO_PIN_9 )
//#define PORT_UNE_TX			    ( GPIOA )
//#define PIN_UNE_TX        		( GPIO_PIN_9 )
#define PORT_UNE_PWR_ENABLE		( GPIOA )//( GPIOB )
#define PIN_UNE_PWR_ENABLE      ( GPIO_PIN_15 )//( GPIO_PIN_3 )

#define UNE_SEL()  				HAL_GPIO_WritePin( PORT_UNE_SEL, PIN_UNE_SEL, GPIO_PIN_SET )
#define UNE_UNSEL()   			HAL_GPIO_WritePin( PORT_UNE_SEL, PIN_UNE_SEL, GPIO_PIN_RESET )
#define UNE_PWR_ENABLE()  		HAL_GPIO_WritePin( PORT_UNE_PWR_ENABLE, PIN_UNE_PWR_ENABLE, GPIO_PIN_SET )
#define UNE_PWR_DISABLE()   	HAL_GPIO_WritePin( PORT_UNE_PWR_ENABLE, PIN_UNE_PWR_ENABLE, GPIO_PIN_RESET )
//#define UNE_TX_DISABLE()   	    HAL_GPIO_WritePin( PORT_UNE_TX, PIN_UNE_TX, GPIO_PIN_RESET )

#define UNE_TXSIZE 				( (128)*(50) )
#define UNE_RXSIZE 				( (128)*(50) )

#ifndef UART_EMUL
uint32_t serial_une82326_get_RC_FLAG( void );
void     serial_une82326_set_RC_FLAG( uint32_t  _une82326_RC_FLAG );
#endif
void 	 serial_une82326_pwr_enable( void );
void 	 serial_une82326_pwr_disable( void );
void 	 serial_une82326_sel( void );
void 	 serial_une82326_unsel( void );
void     serial_une82326_set_start_bit( uint32_t _start_bit );
uint32_t serial_une82326_get_start_bit( void );
uint8_t *serial_une82326_rx_data( void );
uint8_t *serial_une82326_rx_data_pointer( void );
uint8_t *serial_une82326_tx_data_pointer( void );
uint32_t serial_une82326_tx_num( void );
uint32_t serial_une82326_rx_num( void );

uint32_t serial_une82326_rcx( uint8_t *data );
uint32_t serial_une82326_rcx_n( uint8_t *data, uint8_t n );
uint32_t serial_une82326_rcx_bytes_n( void );
void     serial_une82326_set_rcx_bytes_n( uint32_t n );
void     serial_une82326_delete( uint32_t n );
void     serial_une82326_rx_vars_init( void );
#ifndef UART_EMUL
uint32_t serial_une82326_receive_one_byte( UART_HandleTypeDef *uart );
uint32_t serial_une82326_int_rx( UART_HandleTypeDef *uart );

void     serial_une82326_trx( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len );
void 	 serial_une82326_trx_header_msg( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len );
uint32_t serial_une82326_trx_bytes_n( void );
uint32_t serial_une82326_int_tx(UART_HandleTypeDef *uart);
uint32_t serial_une82326_receive_buffer(UART_HandleTypeDef *uart);
void 	 serial_une2326_wait_rx(UART_HandleTypeDef *uart);
#else
uint32_t serial_une82326_receive_one_byte( UART_Emul_HandleTypeDef *uart );
uint32_t serial_une82326_receive_buffer(UART_Emul_HandleTypeDef *uart);
uint8_t  serial_une2326_wait_rx( UART_Emul_HandleTypeDef *uart );
uint32_t serial_une82326_int_rx( UART_Emul_HandleTypeDef *uart );

void     serial_une2326_wait_tx( UART_Emul_HandleTypeDef *uart );
void     serial_une82326_trx( UART_Emul_HandleTypeDef *uart, uint8_t *data, uint32_t data_len );
void 	 serial_une82326_trx_header_msg( UART_Emul_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len );
uint32_t serial_une82326_trx_bytes_n( void );
uint32_t serial_une82326_int_tx(UART_Emul_HandleTypeDef *uart);
#endif

#endif
#endif /* APPLICATION_UNE82326_INC_SERIAL_UNE82326_H_ */
