/*
 * serial_modbus.h
 *
 *  Created on: 30 may. 2019
 *      Author: smill
 */

#ifndef SERIAL_MODBUS_H_
#define SERIAL_MODBUS_H_

#include "stm32u5xx_hal.h"

#define MODBUS_FRAME_SIZE 			 (1024)

#define DE_RX_RS485()                HAL_GPIO_WritePin(UC_MODBUS_DRV_GPIO_Port, UC_MODBUS_DRV_Pin, GPIO_PIN_RESET)
#define DE_TX_RS485()                HAL_GPIO_WritePin(UC_MODBUS_DRV_GPIO_Port, UC_MODBUS_DRV_Pin, GPIO_PIN_SET)

#define DISABLE_RS485()              HAL_GPIO_WritePin(UC_MODBUS_ENABLE_GPIO_Port, UC_MODBUS_ENABLE_Pin, GPIO_PIN_RESET)
#define ENABLE_RS485()               HAL_GPIO_WritePin(UC_MODBUS_ENABLE_GPIO_Port, UC_MODBUS_ENABLE_Pin, GPIO_PIN_SET)

void     serial_enable_rx_rs485( uint8_t enable_tx);
uint8_t  serial_get_enable_rx_rs485( void );
uint8_t  serial_modbus_is_initialized( void );
void     serial_modbus_set_initialized( uint8_t initialized );
uint8_t *serial_modbus_get_tx_data_pointer( void );
uint8_t *serial_modbus_get_rx_data_pointer( void );
uint32_t serial_modbus_get_num_tx_data( void );
uint32_t serial_modbus_get_num_rx_data( void );
uint32_t serial_modbus_get_buffer_size( void );
int8_t   serial_modbus_write_modbus( uint8_t *data, uint32_t len );
uint32_t serial_modbus_read_modbus( uint8_t *data );
void 	 serial_modbus_DMA1_Channel7_TransferCmplt( DMA_HandleTypeDef * _hdma );
void 	 serial_modbus_UART_txCplt_Callback( void );
void     init_modbusTxBufferParams( void );
void     init_modbusRxBufferParams( void );
void     serialRS485_init( void );
void     serialRS485_module_init( uint32_t baudrate );
void     serialRS485_deInit( void );
void     serial_rs485_disable( void );
void     serial_rs485_enable( void );
int      serial_write_loglive(char *ptr, int len);

#endif /* SERIAL_MODBUS_H_ */
