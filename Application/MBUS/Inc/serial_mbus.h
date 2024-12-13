/*
 * serial_mbus.h
 *
 *  Created on: 15 oct. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_MBUS_INC_SERIAL_MBUS_H_
#define APPLICATION_MBUS_INC_SERIAL_MBUS_H_


/* Includes ------------------------------------------------------------------*/

#include <stm32u5xx_hal_uart.h>
#include <sys/_stdint.h>

#include "params.h"

#include "mbus_protocol.h"
#include "mbus_protocol_aux.h"

#ifdef MBUS

#define PORT_MBUS_TX		    (UC_MBUS_TX_GPIO_Port)
#define PIN_MBUS_TX        		(UC_MBUS_TX_Pin)
//#define PORT_MBUS_PWR_ENABLE	(GPIOA)
//#define PIN_MBUS_PWR_ENABLE     (GPIO_PIN_15)

#define MBUS_PWR_ENABLE()  		HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_SET)
#define MBUS_PWR_DISABLE()   	HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_RESET)
#define MBUS_TX_DISABLE()   	HAL_GPIO_WritePin(PORT_MBUS_TX, PIN_MBUS_TX, GPIO_PIN_RESET)

#define SEVERN_TRENT_DATAREQ_ON()  		HAL_GPIO_WritePin(PORT_MBUS_TX, PIN_MBUS_TX, GPIO_PIN_SET)
#define SEVERN_TRENT_DATAREQ_OFF()   	HAL_GPIO_WritePin(PORT_MBUS_TX, PIN_MBUS_TX, GPIO_PIN_RESET)

#define MBUS_TXSIZE 			(512)
#define MBUS_RXSIZE 			(512)

#define PACKET_BUFF_SIZE MBUS_TXSIZE

uint32_t serial_mbus_get_RC_FLAG(void);
void     serial_mbus_set_RC_FLAG( uint32_t  _une82326_RC_FLAG );
#endif
void 	 serial_mbus_pwr_enable( void );
void 	 serial_mbus_pwr_disable( void );
void     serial_mbus_set_start_bit( uint32_t _start_bit );
uint32_t serial_mbus_get_start_bit( void );
uint8_t *serial_mbus_rx_data_pointer( void );
uint8_t *serial_mbus_tx_data_pointer( void );
uint32_t serial_mbus_tx_num( void );
uint32_t serial_mbus_rx_num( void );

uint32_t serial_mbus_rcx( uint8_t *data );
uint32_t serial_mbus_rcx_n( uint8_t *data, uint8_t n );
uint32_t serial_mbus_rcx_bytes_n( void );
void     serial_mbus_set_rcx_bytes_n( uint32_t n );
void     serial_mbus_delete( uint32_t n );
void     serial_mbus_rx_vars_init( void );

uint32_t serial_mbus_receive_one_byte( UART_HandleTypeDef *uart );
uint32_t serial_mbus_int_rx( UART_HandleTypeDef *uart );

uint32_t serial_mbus_trx( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len );
void 	 serial_mbus_trx_header_msg( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len );
uint32_t serial_mbus_trx_bytes_n( void );
uint32_t serial_mbus_int_tx(UART_HandleTypeDef *uart);
uint32_t serial_mbus_receive_buffer(UART_HandleTypeDef *uart);
void 	 serial_mbus_wait_rx(UART_HandleTypeDef *uart);

int32_t  serial_mbus_send_frame(mbus_frame *frame);
int32_t  serial_mbus_recv_frame(mbus_frame *frame);

#endif /* APPLICATION_MBUS_INC_SERIAL_MBUS_H_ */
