/*
 * mbus.h
 *
 *  Created on: 15 oct. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_MBUS_INC_MBUS_H_
#define APPLICATION_MBUS_INC_MBUS_H_

#include "stm32u5xx_hal.h"
#include "params.h"

#	ifdef MBUS
#		define F_FRAME (0)
#		define M_FRAME (1)


/**
 * @struct _mbus_st
 * @brief MBUS structure
 *
 */
typedef struct _mbus_st
{
	uint8_t    start_comm;		/*!< @arg 1 - MBUS Communications have started.
									 @arg 0 - MBUS Communications have not started */
	uint8_t    end_comm;		/*!< 1 -> MBUS communication have finished. 0 -> MBUS Communications have not finished */
	uint8_t    last_device;		/*!< 1 -> MBUS last slave selected. 0 -> MBUS last slave not selected */
	uint8_t    write_record;	/*!< */
	uint8_t    frame_type;		/*!< */
}M_bus;

void    mbus_manager(void);
void    mbus_init( void );
void    mbus_reset( void );
uint8_t mbus_get_start_comm( void );
void    mbus_set_start_comm( uint8_t _start_comm );
uint8_t mbus_get_end_comm( void );
uint8_t mbus_task_end( void );

void    mbus_set_end_comm( uint8_t _end_comm );
uint8_t mbus_get_last_device( void );
void    mbus_set_last_device( uint8_t _last_device );

uint8_t mbus_get_write_record( void );

void    mbus_set_write_record( uint8_t _write_record );
uint8_t mbus_get_frame_type( void );
void    mbus_set_frame_type( uint8_t _frame_type );

uint8_t * mbus_get_raw_telegram( void );
uint32_t  mbus_get_raw_telegram_length( void );
void      mbus_set_raw_telegram_length( uint32_t _raw_telegram_length );
uint32_t  mbus_check_mbus_frame_ok( void );
uint32_t  mbus_check_severn_trent_ok(void);

char *   mbus_get_serial_num( void );
char *   mbus_get_reads_num( void );
char *   mbus_get_counter( void );
uint32_t mbus_get_val_counter_diff( void );
char *   mbus_get_counter_inst( void );
char *   mbus_get_inst_flow( void );

char *   mbus_get_meter_manufacturer( void );
char *   mbus_get_meter_version( void );
uint32_t mbus_get_manufacturer( void );

char * mbus_get_dewa_error_hours( void );
char * mbus_get_dewa_flow_rate( void );
char * mbus_get_dewa_last_overload_duration( void );
char * mbus_get_watermeter_manufacturer_serial_num( void );
void   mbus_set_watermeter_manufacturer_serial_num( char *manufacturer_serial_num );
char * mbus_get_watermeter_pr6_manufacturer_serial_num( void );
char * mbus_get_watermeter_hydrus_manufacturer_serial_num( void );
char * mbus_get_watermeter_badger_manufacturer_serial_num( void );
void   mbus_set_watermeter_pr6_manufacturer_serial_num( char *manufacturer_serial_num );
char * mbus_get_dewa_device_identifier( void );
char * mbus_get_dewa_serial_num( void );
void   mbus_set_dewa_serial_num( char *dewa_serial_num );
char * mbus_get_pr6_serial_num( void );
void   mbus_set_pr6_serial_num( char *pr6_serial_num );
char * mbus_get_pr6_crc( void );
void   mbus_set_pr6_crc( char * pr6_crc);
char * mbus_get_hydrus_serial_num( void );
void   mbus_set_hydrus_serial_num( char *hydrus_serial_num );
char * mbus_get_hydrus_crc( void );
void   mbus_set_hydrus_crc( char * hydrus_crc);
char * mbus_get_badger_serial_num( void );
void   mbus_set_badger_serial_num( char *hydrus_serial_num );
char * mbus_get_badger_crc( void );
void   mbus_set_badger_crc( char * hydrus_crc);
char * mbus_get_dewa_reads_num( void );
char * mbus_get_dewa_meter_battery_remaining( void );
char * mbus_get_dewa_minimum_flow_rate_date( void );
char * mbus_get_dewa_minimum_flow_rate_value( void );
char * mbus_get_dewa_module_temperature( void );
char * mbus_get_dewa_module_operating_hours( void );
char * mbus_get_dewa_overload_time_start( void );
char * mbus_get_dewa_overload_time_stop( void );
char * mbus_get_dewa_peak_flow_rate_date( void );
char * mbus_get_dewa_peak_flow_rate_value( void );
char * mbus_get_dewa_peak_temperature_value( void );
char * mbus_get_dewa_reverse_volume( void );
char * mbus_get_dewa_special_read_date( void );
char * mbus_get_dewa_special_read_value( void );
char * mbus_get_dewa_status( void );
uint32_t mbus_get_dewa_status_int( void );
char * mbus_get_dewa_timestamp( void );
char * mbus_get_dewa_total_overload_duration( void );
char * mbus_get_dewa_total_volume( void );
uint32_t mbus_get_dewa_val_counter_diff( void );

void mbus_task_set_time_data( time_t _time_data);
void mbus_task_set_billing_frame( time_t _billing_frame);

void    mbus_task_init( void );
uint8_t mbus_end( void );
void    mbus_set_end( uint8_t _end );
void    mbus_set_acquire_telegram( uint8_t _acquire_telegram );
uint8_t mbus_acquire_telegram( void );
uint8_t mbus_power_enabled( void );

void   mbus_Task( void );
void   mbus_slaves_Task( void );
void   severn_trent_Task(void);

void   mbus_enable( void );
void   mbus_disable( void );

char * mbus_dewa_device_get_battery_var( void );
char * mbus_dewa_pressure_sensor( void );
void   mbus_get_date_filename( char *date_in, char *date_out );
char * mbus_parse_exp_data( char * data, uint8_t exp );
void   mbus_build_dewa_telegram( void );
void   mbus_build_falconpr6_telegram( void );
void   mbus_build_hydrus_telegram( void );
void   mbus_build_badger_telegram( void );
void   mbus_build_typeM_telegram( char *telegram_1, uint32_t *num_chars );
char * mbus_get_data( void );
char * mbus_get_telegram_one( void );
#endif

#endif /* APPLICATION_MBUS_INC_MBUS_H_ */
