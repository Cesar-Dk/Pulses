/*
 * modbus_sensors_log.h
 *
 *  Created on: 27 ene. 2020
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_LOCALTOOL_INC_MODBUS_SENSORS_LOG_H_
#define APPLICATION_LOCALTOOL_INC_MODBUS_SENSORS_LOG_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32u5xx_hal.h"
#include "params.h"

#define MAX_MODBUS_TELEGRAM_VALUES (2048)//(512)

uint32_t  modbus_sensors_log_init( uint32_t _num_params );
uint32_t  modbus_sensors_get_num_params( void );
uint8_t   modbus_sensors_log_CheckDataInMem( void );
uint32_t  modbus_sensors_log_get_sensor_id( uint32_t index );
uint32_t  modbus_sensors_log_get_num_chars_param( uint32_t index );
char    * modbus_sensors_log_GetParamAvg( uint32_t i  );
char    * modbus_sensors_log_GetParamMin( uint32_t i  );
char    * modbus_sensors_log_GetParamMax( uint32_t i );
char    * modbus_sensors_log_get_param( uint32_t param_index, uint32_t index );
void      modbus_sensors_log_Log( void );
void      modbus_sensors_log_raw_modbus( void );
char    * modbus_sensors_log_message_value_params_sensor( uint32_t i );
void      modbus_sensors_log_params_store_telegram_value( void );
void      modbus_sensors_log_params_store_raw_modbus( void );
char    * modbus_sensors_log_params_telegram( void );
void      modbus_sensors_log_recover_rd_address_backup( void );
char    * modbus_sensors_log_GetDatalogger_Info( void );
char    * modbus_sensors_log_params_sensor_param_telegram( uint32_t param_index, uint32_t value_length, uint32_t *measures_count );
char    * modbus_sensors_log_sensor_raw_telegram( uint32_t *measure_count );
void      modbus_sensors_log_measures_reset( void );
void      modbus_sensors_log_params_reset_telegram_values_pointers( void );
void      modbus_sensors_log_write_lock( uint8_t _write_lock );
uint32_t  modbus_sensors_raw_frame_register( void );
uint32_t  modbus_sensors_log_params_register( void );
#endif /* APPLICATION_LOCALTOOL_INC_MODBUS_SENSORS_LOG_H_ */
