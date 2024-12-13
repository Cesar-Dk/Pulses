/*
 * dlms_log.h
 *
 *  Created on: 20 may. 2021
 *      Author: Sergio Millán López
 */

#ifndef DLMS_LOG_INC_DLMS_LOG_H_
#define DLMS_LOG_INC_DLMS_LOG_H_


#include <stdint.h>
#include <stdbool.h>

#include "stm32u5xx_hal.h"
#include "params.h"

#define MAX_DLMS_TELEGRAM_VALUES (2048 + 767)

uint8_t  dlms_log_init(void);
uint8_t  dlms_log_CheckDataInMem( void );
uint8_t  dlms_log_CheckProfileDataInMem( void );
uint8_t  dlms_log_CheckEventsProfileDataInMem( void );
uint32_t dlms_log_get_telegram_value_frame_type( void );
uint32_t dlms_log_get_telegram_value_num_meter( void );
uint32_t dlms_log_get_telegram_value_capture_period( void );
void     dlms_log_set_telegram_value_capture_period( uint32_t _capture_period );
uint32_t dlms_log_set_event_ff_frame( uint32_t __ff_frame );
uint32_t dlms_log_get_frame_tunneling(void);
void     dlms_log_set_frame_tunneling( uint32_t _tunn );
uint32_t dlms_log_get_profile_max_demand_extra( void );
void     dlms_log_set_profile_max_demand_extra( uint32_t __xtra );
char   * dlms_log_get_dataCmdOnDemand( void );
uint32_t dlms_log_set_dataCmdOnDemand( char * __dataCmdOnDemand);

int  dlms_log_store_raw_loadprofile(const unsigned char* bytes, uint16_t count, uint16_t size);
int  dlms_log_store_raw_eventsprofile(const unsigned char* bytes, uint16_t count, uint16_t size);
void dlms_log_raw_modbus(void);
uint32_t dlms_log_params_store_raw_modbus( uint32_t frame_type );
uint32_t dlms_log_params_store_raw_modbus_tunneling( uint32_t frame_type );
void dlms_log_recover_rd_address_backup( void );
void dlms_log_profile_recover_rd_address_backup( void );
char * dlms_log_GetDatalogger_Info( void );
char * dlms_log_sensor_raw_telegram( uint32_t *measure_count );
char * dlms_log_sensor_raw_profile_telegram( uint32_t *frame_size );
char * dlms_log_sensor_raw_eventsprofile_telegram( uint32_t *frame_size );
void dlms_log_measures_reset( void );
void dlms_log_params_reset_telegram_values_pointers( void );
void dlms_log_write_lock( uint8_t _write_lock );
uint32_t dlms_raw_frame_register(void);
uint32_t dlms_raw_frame_tunneling_register(void);

#endif /* DLMS_LOG_INC_DLMS_LOG_H_ */
