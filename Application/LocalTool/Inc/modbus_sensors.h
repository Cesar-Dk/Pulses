/*
 * modbus_sensors.h
 *
 *  Created on: 23 ene. 2020
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_LOCALTOOL_INC_MODBUS_SENSORS_H_
#define APPLICATION_LOCALTOOL_INC_MODBUS_SENSORS_H_

#include "modbus.h"

/** Modbus UART number of bits and stop bit configuration*/
typedef enum {
	serial_8N1 = 0,
	serial_8N2 = 1
} serial_config_stop_bits;

/** Modbus UART parity configuration */
typedef enum {
	parity_even = 0,
	parity_odd  = 1,
	no_parity   = 2
} serial_config_parity;

/** Modbus communication functional states */
typedef enum {
	MODBUS_COMM_DISABLED = 0,
	MODBUS_COMM_STARTED  = 1,
	MODBUS_COMM_END      = 2
} modbus_comm_state;

void                    modbus_sensors_init_task( void );
serial_config_stop_bits modbus_sensors_get_serial_config_stop_bits( void );
uint32_t 				modbus_sensors_get_serial_config_baud_rate( void );
serial_config_parity    modbus_sensors_get_serial_config_parity( void );
void    				modbus_sensors_set_serial_config_stop_bits( serial_config_stop_bits _stop_bits );
void    				modbus_sensors_set_serial_config_baud_rate( uint32_t _baud_rate );
void                    modbus_sensors_set_serial_config_parity( serial_config_parity _parity );
uint8_t  				modbus_sensors_get_comm_state( void );
void 					modbus_sensors_set_comm_state( uint8_t _comm_state );
void 					modbus_sensors_set_get_data( uint8_t _get_data );
uint8_t  				modbus_sensors_get_get_data( void );
uint32_t 				modbus_sensors_serial_config_usart( void );
uint32_t                modbus_sensors_get_param_index( void );
void     				modbus_sensors_task( void );
void    				modbus_sensors_tx( void );
void   					modbus_sensors_rx( void );

#endif /* APPLICATION_LOCALTOOL_INC_MODBUS_SENSORS_H_ */
