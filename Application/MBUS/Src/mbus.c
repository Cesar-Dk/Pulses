/**
  ******************************************************************************
  * @file           mbus.c
  * @author 		Datakorum Development Team
  * @brief          .
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution SL.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* mbus.c
 *
 *  Created on: 15 oct. 2019
 *      Author: Sergio Millán López
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "usart.h"
#include "udp_protocol.h"
#include "mbus.h"
#include "ad.h"
#include "serial_mbus.h"
#include "mbus_protocol.h"
#include "mbus_slave_table_manager.h"
#include "tick.h"
#include "rtc_system.h"
#include "datalogger_buffer.h"
#include "sensor_log.h"
#include "rtc_system.h"

#include "dlms_client.h"

#include "ME910.h"

#include "leds.h"

#ifdef MBUS

//#define F_FRAME 0
//#define M_FRAME 1

/**
 * @defgroup system_mbus MBUS
 * @brief
 * @{
 */

/**
 * @enum mbus_status
 * @brief mbus status for the FSM
 *
 */
static enum
{
	MBUS_IDLE, MBUS_ENABLE,
	MBUS_PING_REQ, MBUS_PING_RESP,
	MBUS_CHANGE_BAUDRATE, MBUS_CHANGE_BAUDRATE_RESP,
	MBUS_CHANGE_LONG_TELEGRAM, MBUS_CHANGE_LONG_TELEGRAM_RESP,
	MBUS_DATA_REQ, MBUS_DATA_RESP, MBUS_DATA_REQ_1, MBUS_DATA_RESP_1,
	MBUS_DISABLE, MBUS_OFF
} mbus_status = MBUS_IDLE;

typedef enum _eMBusState
{
	MBUS_SLAVES_IDLE, MBUS_SLAVES_ENABLE,
	MBUS_SLAVES_NETWORK_ALL_SLAVES, MBUS_SLAVES_NETWORK_SCAN_NEW_SLAVES,
	MBUS_SLAVES_PING_REQ, MBUS_SLAVES_PING_RESP,
	MBUS_SLAVES_CHANGE_LONG_TELEGRAM, MBUS_SLAVES_CHANGE_LONG_TELEGRAM_RESP,
	MBUS_SLAVES_CHANGE_POINTER_TELEGRAM, MBUS_SLAVES_CHANGE_POINTER_TELEGRAM_RESP,
	MBUS_SLAVES_GET_BILLING_TELEGRAM, MBUS_SLAVES_GET_BILLING_TELEGRAM_RESP,
	MBUS_SLAVES_CHANGE_DATA_TIME, MBUS_SLAVES_CHANGE_DATA_TIME_RESP,
	MBUS_SLAVES_DATA_REQ_1, MBUS_SLAVES_DATA_RESP_1,
	MBUS_SLAVES_DISABLE, MBUS_SLAVES_OFF
} eMBusState;

typedef struct _MBus{
	eMBusState mbus_status;
	uint32_t   mbus_time;
	uint32_t   attempt_n;
	uint32_t   error;
	uint32_t   slave_item;
	uint32_t   write_record;
	uint32_t   end;
}sMBus;

sMBus MBus;

mbus_slaves_address_table * slaves_table;

#define SEVERN_TRENT_FRAME_LENGTH (58)
#define SEVERN_TRENT_TDRW_MS      (6)
#define SEVERN_TRENT_TDRD_MS      (10)
#define SEVERN_TRENT_TDRR_MS      (50)
#define SEVERN_TRENT_TDRAW_MS     (20)
/**
 * @enum severntrent_status
 * @brief mbus status for the FSM
 *
 */
static enum
{
	SEVERNTRENT_IDLE, SEVERNTRENT_DATAREQ, SEVERNTRENT_WAIT_DATAREQ, SEVERNTRENT_DATAOUT,
	SEVERNTRENT_DISABLE, SEVERNTRENT_SEND_ABORT_AND_RETRY, SEVERNTRENT_ABORT, SEVERNTRENTS_RETRY, SEVERNTRENT_OFF
} severntrent_status = SEVERNTRENT_IDLE;

/**
 * @struct values
 * @brief
 *
 */
typedef struct {
	uint32_t counter;
	uint32_t counter_diff;
	uint32_t counter_inst;
	uint32_t inst_flow;
}values;

/**
 * @struct mbus_water_data
 * @brief
 *
 */
typedef struct {
	char   serial_num[20];
	char   reads_num[4];
	char   counter[12];
	char   counter_inst[12];
	char   inst_flow[12];
	values val;
	values previous_val;
}mbus_water_data;


static mbus_water_data mbus_Water_Data;

/**
 * @struct mbus_meter_identifier
 * @brief Specifies the manufacturer and the firmware version of the meter
 *
 */
typedef struct{
	char   manufacturer[4];		/*!< Meter manufacturer name 4 characters */
	char   version[4];			/*!< Meter firmware version */
}mbus_meter_identifier;

/**
 * @struct mbus_dewa_long_telegram
 * @brief structure that holds all the meaningful parameters of the MBUS long telegram.
 *
 * @note This structure is not used anymore as the data is transmitted in raw.
 *
 */
typedef struct {
	char   manufacturer_serial_num[11];
	char   device_identifier[11];
	char   manufacturer[4];
	char   version[4];
	char   device_type[7];
	char   reads_num[4];
	char   dewa_serial_num[20];
	char   total_volume[20];
	char   reverse_volume[20];
	char   flow_rate[20];
	char   special_read_date[26];
	char   special_read_value[20];
	char   module_temperature[10];
	char   peak_temperature_date[26];
	char   peak_temperature_value[10];
	char   meter_battery_remaining[10];
	char   operating_hours[10];
	char   error_hours[10];
	char   overload_time_start[26];
	char   overload_time_stop[26];
	char   peak_flow_rate_date[26];
	char   peak_flow_rate_value[20];
	char   minimum_flow_rate_date[26];
	char   minimum_flow_rate_value[20];
	char   last_overload_duration[10];
	char   total_overload_duration[10];
	char   crc[5];
	char   status[4];
	char   timestamp[26];
	char   device_battery[24];
	values val;
	values previous_val;
}mbus_dewa_long_telegram;

/**
 * @struct
 * @brief
 *
 */
typedef struct {
	char   manufacturer_serial_num[11];
	char   device_identifier[11];
	char   manufacturer[4];
	char   version[4];
	char   device_type[7];
	char   reads_num[4];
	char   pr6_serial_num[20];
	char   total_volume[20];
	char   reverse_volume[20];
	char   current_date[26];
	char   last_annual_rep_date[26];
	char   vol_last_annual_rep[20];
	char   next_annual_rep_date[26];
	char   max_throughput[20];
	char   max_throughput_date[26];
	char   curr_throughput[20];
//	char   month1_date[26];
//	char   month1_state[20];
//	char   month2_date[26];
//	char   month2_state[20];
//	char   month3_date[26];
//	char   month3_state[20];
//	char   month4_date[26];
//	char   month4_state[20];
//	char   month5_date[26];
//	char   month5_state[20];
//	char   month6_date[26];
//	char   month6_state[20];
//	char   month7_date[26];
//	char   month7_state[20];
//	char   month8_date[26];
//	char   month8_state[20];
//	char   month9_date[26];
//	char   month9_state[20];
//	char   month10_date[26];
//	char   month10_state[20];
//	char   month11_date[26];
//	char   month11_state[20];
//	char   month12_date[26];
//	char   month12_state[20];
	char   crc[5];
	char   status[4];
	char   timestamp[26];
	char   device_battery[5];
	values val;
	values previous_val;
} mbus_pr6_long_telegram;

typedef struct {
	char   manufacturer_serial_num[11];
	char   device_identifier[11];
	char   manufacturer[4];
	char   version[4];
	char   device_type[7];
	char   reads_num[4];
	char   hydrus_serial_num[20];
	char   dewa_serial_num[20];
	char   current_volume[20];
	char   reverse_volume[20];
	char   flow_rate[20];
	char   medium_temperature[10];
	char   ambient_temperature[10];
	char   battery_empty_date[26];
	char   operating_hours[10];
	char   error_hours[10];
	char   overflow_seconds_counter[10];
	char   overflow_event_counter[10];
	char   current_date[26];
	char   crc[5];
	char   status[4];
	char   device_battery[5];
	values val;
	values previous_val;
} mbus_hydrus_telegram;

typedef struct {
	char   manufacturer_serial_num[11];
	char   device_identifier[11];
	char   manufacturer[4];
	char   version[4];
	char   device_type[7];
	char   reads_num[4];
	char   badger_serial_num[20];
	char   dewa_serial_num[20];
	char   actuality_duration[10];
	char   current_volume[20];
	char   record_date_volume[20];
	char   record_date_date[26];
	char   remaining_battery_lifetime[10];
	char   current_date[26];
	char   reverse_volume[20];
	char   flow_rate[20];
	char   medium_temperature[10];
	char   operating_hours[10];
	char   error_hours[10];
	char   overload_time[26];
	char   overload_hours_counter[26];
	char   ambient_temperature[10];
	char   error_flag[10];
	char   crc[5];
	char   status[4];
	char   device_battery[5];
	values val;
	values previous_val;
} mbus_badger_telegram;

static mbus_meter_identifier   mbus_Watermeter_Identifier;
static mbus_dewa_long_telegram mbus_Dewa_Long_Telegram;
static mbus_pr6_long_telegram  mbus_PR6_Long_Telegram;
static mbus_hydrus_telegram    mbus_Hydrus_Telegram;
static mbus_badger_telegram    mbus_Badger_Telegram;

static char mbus_dewa_pressure[7];
static char mbus_device_battery[24];

static char *json     = NULL;
static char json_array[512];

static uint32_t        t, raw_telegram_len;
static uint8_t         data[max(MBUS_TXSIZE, MBUS_RXSIZE)];
static uint8_t		   attempt_n;
static mbus_frame      frame;
static mbus_frame_data data_frame;

static M_bus  mbus;

uint8_t end              = 0;
uint8_t acquire_telegram = 0;
uint8_t mbus_enabled     = 0;
uint32_t mbus_test_ok    = 0;
time_t  time_tick_data   = 0;
uint32_t mbus_billing_frame = 0;


extern struct params param;
extern uint32_t reed_sending;
extern uint32_t powermon_sending;

static void mbus_check_watermeter( mbus_frame_data *data_frame );
//static void mbus_dewa_parse_telegram( void );
static void mbus_falconpr6_parse_telegram( void );
#ifdef HYDRUS
static void mbus_hydrus_parse_telegram( void );
#endif
#ifdef BADGER
static void mbus_badger_parse_telegram( void );
#endif
static void mbus_parse_data( char * json );
//static void mbus_build_dewa_telegram( void );

/** reverses a string 'str' of length 'len'
 * @private
 */
static void __reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

/**
 * Converts a given integer x to string str[]. d is the number of digits required in output.
 * If d is bigger than the number of digits in x, then 0s are added at the beginning.
 * @private
 */
static int __intToStr(int x, char str[], int d)
{
	int i = 0;
	if (x == 0) {
		str[i++] = '0';
	}
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	/* If number of digits required is more, then add 0s at the beginning */
	while (i < d)
		str[i++] = '0';

	__reverse(str, i);
	str[i] = '\0';
	return i;
}

/**
 * Converts a floating point number to string.
 * @private
 */
static void __ftoa(float n, char *res, int afterpoint)
{
	int ipart = (int)n;
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = __intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part up to a given number .
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		__intToStr((int)fpart, res + i + 1, afterpoint);
	}
}

void mbus_manager(void)
{
	if ( MBUS_METER_TYPE == params_get_uart_meter_type())
	{
			mbus_Task();
	}
	else if ( MBUS_SLAVES_METER_TYPE == params_get_uart_meter_type())
	{
			mbus_slaves_Task();
	}
	else if ( SEVERN_TRENT_METER_TYPE == params_get_uart_meter_type())
	{
		severn_trent_Task();
	}
	else
	{
		params_set_uart_meter_type(MBUS_METER_TYPE);
	}
}

void mbus_init( void )
{
	mbus.end_comm   = 0;
	mbus.start_comm = 0;
}

void mbus_reset( void )
{
	mbus.end_comm     = 0;
	mbus.start_comm   = 0;
	mbus.last_device  = 0;
	mbus.write_record = 0;
	end               = 0;
	acquire_telegram  = 0;
	mbus_enabled      = 0;

	mbus_status       = MBUS_IDLE;


	MBus.attempt_n    = 0;
	MBus.mbus_status  = MBUS_SLAVES_IDLE;
	MBus.mbus_time    = 0;
	MBus.end          = 0;
	MBus.slave_item   = 0;
	MBus.write_record = 0;
}

/**
 * @fn uint8_t mbus_get_start_comm(void)
 * @brief Reads the mbus measurement communication flag
 *
 * @return _start_com mbus communication
 */
uint8_t mbus_get_start_comm(void)
{
	return mbus.start_comm;
}

/**
 * @fn void mbus_set_start_comm(uint8_t)
 * @brief Enables Disables the flag to start an mbus measurement
 *
 * @param _start_comm
 * 			@arg 0 - mbus communication not started
 * 			@arg 1 - mbus communication started
 */
void mbus_set_start_comm( uint8_t _start_comm )
{
	mbus.start_comm = _start_comm;
}


uint8_t mbus_get_end_comm( void )
{
	return mbus.end_comm;
}

void mbus_set_end_comm( uint8_t _end_comm )
{
	mbus.end_comm = _end_comm;
}

uint8_t mbus_get_last_device( void )
{
	return mbus.last_device;
}

void mbus_set_last_device( uint8_t _last_device )
{
	mbus.last_device = _last_device;
}

uint8_t mbus_get_write_record( void )
{
	return mbus.write_record;
}

void mbus_set_write_record( uint8_t _write_record )
{
	mbus.write_record = _write_record;
}

uint8_t mbus_get_frame_type( void )
{
	return mbus.frame_type;
}

void mbus_set_frame_type( uint8_t _frame_type )
{
	mbus.frame_type = _frame_type;
}

uint8_t * mbus_get_raw_telegram( void )
{
	return data;
}

uint32_t mbus_get_raw_telegram_length( void )
{
	return raw_telegram_len;
}

void mbus_set_raw_telegram_length( uint32_t _raw_telegram_length )
{
	raw_telegram_len = _raw_telegram_length;
}

uint32_t mbus_check_mbus_frame_ok( void )
{
	if ( (( 0x68 == frame.start1 ) && ( 0x68 == frame.start2 ) && ( 0x16 == frame.stop )) || (1 == mbus_test_ok)) {
		return 1;
	} else {
		return 0;
	}
}

uint32_t st_ok = 0;
uint32_t mbus_check_severn_trent_ok(void)
{
	return st_ok;
}

char * mbus_get_serial_num( void )
{
	return mbus_Water_Data.serial_num;
}

char * mbus_get_reads_num( void )
{
	return mbus_Water_Data.reads_num;
}

char * mbus_get_counter( void )
{
	return mbus_Water_Data.counter;
}

uint32_t mbus_get_val_counter_diff( void )
{
	return mbus_Water_Data.val.counter_diff;
}

char * mbus_get_counter_inst( void )
{
	return mbus_Water_Data.counter_inst;
}

char * mbus_get_inst_flow( void )
{
	return mbus_Water_Data.inst_flow;
}

char * mbus_get_meter_manufacturer( void )
{
	return mbus_Watermeter_Identifier.manufacturer;
}

char * mbus_get_meter_version( void )
{
	return mbus_Watermeter_Identifier.version;
}

uint32_t mbus_get_manufacturer( void )
{
	uint32_t ret = 0;
	if ( memcmp(mbus_get_meter_manufacturer(), "BMI", 3 ) == 0 ) {
		ret = 1;
	} else if ( memcmp(mbus_get_meter_manufacturer(), "DME", 3 ) == 0 ) {
		ret = 2;
	}
	return ret;
}

char * mbus_get_dewa_error_hours( void )
{
	return mbus_Dewa_Long_Telegram.error_hours;
}

char * mbus_get_dewa_flow_rate( void )
{
	return mbus_Dewa_Long_Telegram.flow_rate;
}

char * mbus_get_dewa_last_overload_duration( void )
{
	return mbus_Dewa_Long_Telegram.last_overload_duration;
}

char * mbus_get_watermeter_manufacturer_serial_num( void )
{
	if ( param.manufacturer_serial_num[0] == '\0' ) {
		return mbus_Dewa_Long_Telegram.manufacturer_serial_num;
	}
	else {
		return (char *)param.manufacturer_serial_num;
	}
}

void mbus_set_watermeter_manufacturer_serial_num( char *manufacturer_serial_num )
{
	strncpy(mbus_Dewa_Long_Telegram.manufacturer_serial_num, manufacturer_serial_num, 11);
	strncpy((char *)param.manufacturer_serial_num, manufacturer_serial_num, 12);
}

char * mbus_get_watermeter_pr6_manufacturer_serial_num( void )
{
	if ( param.manufacturer_serial_num[0] == '\0' ) {
		return mbus_PR6_Long_Telegram.manufacturer_serial_num;
	}
	else {
		return (char *)param.manufacturer_serial_num;
	}
}

char * mbus_get_watermeter_hydrus_manufacturer_serial_num( void )
{
	if ( param.manufacturer_serial_num[0] == '\0' ) {
		return mbus_Hydrus_Telegram.manufacturer_serial_num;
	}
	else {
		return (char *)param.manufacturer_serial_num;
	}
}

char * mbus_get_watermeter_badger_manufacturer_serial_num( void )
{
//	if ( param.manufacturer_serial_num[0] == '\0' ) {
//		return mbus_Badger_Telegram.manufacturer_serial_num;
//	}
//	else {
//		return (char *)param.manufacturer_serial_num;
//	}
	return mbus_Badger_Telegram.manufacturer_serial_num;
}

void mbus_set_watermeter_pr6_manufacturer_serial_num( char *manufacturer_serial_num )
{
	strncpy(mbus_PR6_Long_Telegram.manufacturer_serial_num, manufacturer_serial_num, 11);
	strncpy((char *)param.manufacturer_serial_num, manufacturer_serial_num, 12);
}

char * mbus_get_dewa_device_identifier( void )
{
	return mbus_Dewa_Long_Telegram.device_identifier;
}

char * mbus_get_dewa_serial_num( void )
{
	if ( param.dewa_serial_num[0] == '\0' ) {
		return mbus_Dewa_Long_Telegram.dewa_serial_num;
	}
	else {
		return (char *)param.dewa_serial_num;
	}
}

void mbus_set_dewa_serial_num( char *dewa_serial_num )
{
	strncpy(mbus_Dewa_Long_Telegram.dewa_serial_num, dewa_serial_num, 20);
	strncpy((char *)param.dewa_serial_num, dewa_serial_num, 20);
}

char * mbus_get_pr6_serial_num( void )
{
	if ( param.dewa_serial_num[0] == '\0' ) {
		return mbus_PR6_Long_Telegram.pr6_serial_num;
	}
	else {
		return (char *)param.dewa_serial_num;
	}
}

void mbus_set_pr6_serial_num( char *pr6_serial_num )
{
	strncpy(mbus_PR6_Long_Telegram.pr6_serial_num, pr6_serial_num, 20);
	strncpy((char *)param.dewa_serial_num, pr6_serial_num, 20);
}

char * mbus_get_pr6_crc( void )
{
	return (char *)mbus_PR6_Long_Telegram.crc;
}

void mbus_set_pr6_crc( char * pr6_crc)
{
	strncpy( mbus_PR6_Long_Telegram.crc, pr6_crc, 5 );
}

char * mbus_get_hydrus_serial_num( void )
{
//	if ( param.dewa_serial_num[0] == '\0' ) {
//		return mbus_Hydrus_Telegram.hydrus_serial_num;
//	}
//	else {
//		return (char *)param.dewa_serial_num;
//	}
	return mbus_Hydrus_Telegram.dewa_serial_num;
}

void mbus_set_hydrus_serial_num( char *hydrus_serial_num )
{
	strncpy(mbus_Hydrus_Telegram.hydrus_serial_num, hydrus_serial_num, 20);
	strncpy((char *)param.dewa_serial_num, hydrus_serial_num, 20);
}

char * mbus_get_hydrus_crc( void )
{
	return (char *)mbus_Hydrus_Telegram.crc;
}

void mbus_set_hydrus_crc( char * hydrus_crc)
{
	strncpy( mbus_Hydrus_Telegram.crc, hydrus_crc, 5 );
}

char * mbus_get_badger_serial_num( void )
{
//	if ( param.dewa_serial_num[0] == '\0' ) {
//		return mbus_Badger_Telegram.badger_serial_num;
//	}
//	else {
//		return (char *)param.dewa_serial_num;
//	}
	return mbus_Badger_Telegram.dewa_serial_num;
}

void mbus_set_badger_serial_num( char *badger_serial_num )
{
	strncpy(mbus_Badger_Telegram.badger_serial_num, badger_serial_num, 20);
	strncpy((char *)param.dewa_serial_num, badger_serial_num, 20);
}

char * mbus_get_badger_crc( void )
{
	return (char *)mbus_Badger_Telegram.crc;
}

void mbus_set_badger_crc( char * badger_crc)
{
	strncpy( mbus_Badger_Telegram.crc, badger_crc, 5 );
}

char * mbus_get_dewa_reads_num( void )
{
	return mbus_Dewa_Long_Telegram.reads_num;
}

char * mbus_get_dewa_meter_battery_remaining( void )
{
	return mbus_Dewa_Long_Telegram.meter_battery_remaining;
}

char * mbus_get_dewa_minimum_flow_rate_date( void )
{
	return mbus_Dewa_Long_Telegram.minimum_flow_rate_date;
}

char * mbus_get_dewa_minimum_flow_rate_value( void )
{
	return mbus_Dewa_Long_Telegram.minimum_flow_rate_value;
}

char * mbus_get_dewa_module_temperature( void )
{
	return mbus_Dewa_Long_Telegram.module_temperature;
}

char * mbus_get_dewa_module_operating_hours( void )
{
	return mbus_Dewa_Long_Telegram.operating_hours;
}

char * mbus_get_dewa_overload_time_start( void )
{
	return mbus_Dewa_Long_Telegram.overload_time_start;
}

char * mbus_get_dewa_overload_time_stop( void )
{
	return mbus_Dewa_Long_Telegram.overload_time_stop;
}

char * mbus_get_dewa_peak_flow_rate_date( void )
{
	return mbus_Dewa_Long_Telegram.peak_flow_rate_date;
}

char * mbus_get_dewa_peak_flow_rate_value( void )
{
	return mbus_Dewa_Long_Telegram.peak_flow_rate_value;
}

char * mbus_get_dewa_peak_temperature_value( void )
{
	return mbus_Dewa_Long_Telegram.peak_temperature_value;
}

char * mbus_get_dewa_reverse_volume( void )
{
	return mbus_Dewa_Long_Telegram.reverse_volume;
}

char * mbus_get_dewa_special_read_date( void )
{
	return mbus_Dewa_Long_Telegram.special_read_date;
}

char * mbus_get_dewa_special_read_value( void )
{
	return mbus_Dewa_Long_Telegram.special_read_value;
}

char * mbus_get_dewa_status( void )
{
	return mbus_Dewa_Long_Telegram.status;
}

uint32_t mbus_get_dewa_status_int( void )
{
	return atoi(mbus_Dewa_Long_Telegram.status);
}

char * mbus_get_dewa_timestamp( void )
{
	return mbus_Dewa_Long_Telegram.timestamp;
}

char * mbus_get_dewa_total_overload_duration( void )
{
	return mbus_Dewa_Long_Telegram.total_overload_duration;
}

char * mbus_get_dewa_total_volume( void )
{
	return mbus_Dewa_Long_Telegram.total_volume;
}

uint32_t mbus_get_dewa_val_counter_diff( void )
{
	return mbus_Dewa_Long_Telegram.val.counter_diff;
}

uint8_t mbus_end( void )
{
	return end;
}

void mbus_set_end( uint8_t _end )
{
	end = _end;
}

void mbus_set_acquire_telegram( uint8_t _acquire_telegram )
{
	acquire_telegram = _acquire_telegram;
}

uint8_t mbus_acquire_telegram( void )
{
	return acquire_telegram;
}

uint8_t mbus_power_enabled( void )
{
	return mbus_enabled;
}

uint32_t mbus_get_status( void )
{
	return mbus_status;
}

void mbus_set_status( uint32_t _status )
{
	mbus_status = _status;
}

void mbus_task_init( void )
{
	MBus.attempt_n = 0;
	MBus.mbus_status = MBUS_SLAVES_IDLE;
	MBus.mbus_time = 0;

	mbus_slave_table_manager_init(1);
}

uint8_t mbus_task_end( void )
{
	return MBus.end;
}

uint8_t mbus_task_write_record( void )
{
	return MBus.write_record;
}

void mbus_task_set_write_record( uint8_t _write_record )
{
	MBus.write_record = _write_record;
}

uint32_t mbus_task_get_mbus_delay( void )
{
	return MBus.mbus_time;
}

void mbus_task_set_time_data( time_t _time_data)
{
	time_tick_data = _time_data;
}

void mbus_task_set_billing_frame( time_t _billing_frame)
{
	mbus_billing_frame = _billing_frame;
}

static void __updateState( uint32_t _t, eMBusState next_status )
{
	Tick_update_tick( TICK_MBUS );
	MBus.mbus_time   = _t;
	MBus.mbus_status = next_status;
}

static void __configureSlaveBaudrate( mbus_slaves_address_table * _slaves_table, uint8_t slave_num )
{
	if (_slaves_table->slave[slave_num].baudrate == MBUS_BAUDRATE_2400 )
	{
		MX_USART3_USART_DeInit();
		params_set_mbus_baudrate(2400);
		MX_USART3_UART_Custom_Init();
	}
	else
	{
		MX_USART3_USART_DeInit();
		params_set_mbus_baudrate(300);
		MX_USART3_UART_Custom_Init();
	}
}

/**
 * @fn void mbus_Task(void)
 * @brief MBUS system thread.
 *
\msc

a [label="Pipe20"],b [label="Meter"];
a-xb [label="PING_REQ 1 x5"];
 ---  [label = "END PROCESS MBUS ERROR"];
a=>b [label="PING_REQ"];
a<=b [label="PING_RESPONSE 0xE5"];
a=>b [label="SHORT TELEGRAM"];

\endmsc

 *
 *
 * */
uint8_t scan_baudrate = MBUS_BAUDRATE_2400;

void mbus_slaves_Task( void )
{
	char *addr_mask;
	static uint8_t slave_num;
	static uint32_t subcode = 0xF0;
	static uint32_t installation_sent = 0;
	static uint32_t billing_count = 0;
	static uint32_t subcode_billing = 2, subcode2_billing = 0xC3;

	switch( MBus.mbus_status )
	{
	case MBUS_SLAVES_IDLE:
		if ((1 == mbus_get_start_comm()) && (0 == params_pulses_on()) && (MBUS_SLAVES_METER_TYPE == params_get_uart_meter_type()))
		{
			MBus.end = 0;
			__updateState( 1, MBUS_SLAVES_ENABLE );
		}
		break;
	case MBUS_SLAVES_ENABLE:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			MX_USART3_USART_DeInit();
			params_set_mbus_baudrate(2400);
			MX_USART3_UART_Custom_Init();
			MBus.attempt_n = 10;
			memset(data,  0, sizeof(data));
			__updateState( 1, MBUS_SLAVES_NETWORK_ALL_SLAVES );
		}
		break;
	case MBUS_SLAVES_NETWORK_ALL_SLAVES:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			slaves_table = mbus_slave_table_manager_read_table();
			LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Slave Table:%d.\r\n", (int)Tick_Get( SECONDS ),(int)slaves_table->slaves_num);
			if ( ( 1 == slaves_table->isfull ) || (slaves_table->slaves_num != 0) /*|| ( 0 == leds_device_switch_on() )*/ )
			{
				slave_num = 0;
				MBus.attempt_n = 10;
				if ( 1 == mbus_billing_frame )
				{
					__updateState( 5, MBUS_SLAVES_GET_BILLING_TELEGRAM );
					billing_count = 0;
				}
				else
				{
					__updateState( 5, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );
				}
			}
			else
			{
				if ( 1 == leds_device_switch_on() )
				{
					leds_set_device_switch_on(0);
					scan_baudrate = MBUS_BAUDRATE_2400;
				}
				mbus_slave_table_manager_init(1);
				__updateState( 1, MBUS_SLAVES_NETWORK_SCAN_NEW_SLAVES );
			}
		}
		break;
	case MBUS_SLAVES_NETWORK_SCAN_NEW_SLAVES:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			if (MBus.attempt_n)
			{
				MBus.attempt_n--;
				addr_mask = strdup("FFFFFFFFFFFFFFFF");
				if ( 0 == mbus_scan_2nd_address_range(0, addr_mask) )
				{
					mbus_slave_table_manager_save_table();
					slaves_table = mbus_slave_table_manager_read_table();
					free(addr_mask);
					slave_num = 0;
					if ( ( MBUS_BAUDRATE_2400 == scan_baudrate ) && ( slaves_table->slaves_num == 0 ) )
					{  // Maybe more slaves with different baudrate are installed.
						MX_USART3_USART_DeInit();
						params_set_mbus_baudrate(300);
						MX_USART3_UART_Custom_Init();
						scan_baudrate          = MBUS_BAUDRATE_300;
						slaves_table->baudrate = MBUS_BAUDRATE_300;
						__updateState( 1, MBUS_SLAVES_NETWORK_SCAN_NEW_SLAVES );
					}
					else
					{
						if (0 == slaves_table->slaves_num)
						{
							MBus.error = 1;
							LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS>SLAVES NUM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
							__updateState( 1, MBUS_SLAVES_DISABLE );
						}
						else
						{
							MBus.attempt_n = 10;
							__updateState( 1, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );
						}
					}
				}
			}
			else
			{
				MBus.error = 1;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS>SLAVES RETRIES NUM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				__updateState( 1, MBUS_SLAVES_DISABLE );
			}
		}
		break;
	case MBUS_SLAVES_CHANGE_LONG_TELEGRAM:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			static uint32_t disable_high_sec_address = 0;
//			LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Change Long Telegram Slave num:%d.Slave add:%s.Subcode:%d.\r\n",
//					(int)Tick_Get( SECONDS ), (int)slave_num, slaves_table->slave[slave_num].secondary, (int)subcode);
			if (MBus.attempt_n)
			{
				if (subcode != 0x80)
				{
					subcode = params_get_mbus_frame();
				}
				MBus.attempt_n--;
				__configureSlaveBaudrate( slaves_table, slave_num );
				if ( 1 == disable_high_sec_address )
				{
					slaves_table->slave[slave_num].secondary[8]  = 'F';
					slaves_table->slave[slave_num].secondary[9]  = 'F';
					slaves_table->slave[slave_num].secondary[10] = 'F';
					slaves_table->slave[slave_num].secondary[11] = 'F';
					slaves_table->slave[slave_num].secondary[12] = 'F';
					slaves_table->slave[slave_num].secondary[13] = 'F';
					slaves_table->slave[slave_num].secondary[14] = 'F';
					slaves_table->slave[slave_num].secondary[15] = 'F';

					disable_high_sec_address = 0;
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Change Long Telegram Slave num:%d.Slave add:%s.Subcode:%d.\r\n",
						(int)Tick_Get( SECONDS ), (int)slave_num, slaves_table->slave[slave_num].secondary, (int)subcode);
				if ( 0 == mbus_send_change_long_telegram( slaves_table->slave[slave_num].secondary, subcode/*params_get_mbus_frame()*/ ) )
				{
//					memset((void *) mbus_protocol_get_mbus_json_array(), 0, mbus_protocol_size_of_mbus_json_array());
					memset(json_array,  0, sizeof(json_array));
					MBus.attempt_n = 10;
					if (time_tick_data != 0)
					{
						__updateState(1, MBUS_SLAVES_CHANGE_DATA_TIME);
					}
					else
					{
						__updateState(1, MBUS_SLAVES_DATA_REQ_1/*MBUS_SLAVES_CHANGE_POINTER_TELEGRAM*/);
					}
				}
				else
				{
//					MBus.error = 1;
//					__updateState( 1, MBUS_SLAVES_DISABLE );
					disable_high_sec_address = 1;
					LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> CHANGE LONG TELEGRAM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				}
			}
			else
			{
//				MBus.error = 1;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> RETRIES CHANGE LONG TELEGRAM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				MBus.attempt_n = 10;
				slave_num++;
				mbus_set_write_record(1);
				if ( slave_num < slaves_table->slaves_num )
				{
					sprintf( (char *)data,
							"*#*#*#*#*#*#*#MBUS-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
					raw_telegram_len = strlen((char *)data);
					__updateState( 1, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );
				}
				else
				{
					mbus_set_last_device(1);
					__updateState( 1, MBUS_SLAVES_DISABLE );
				}
			}
		}
		break;
	case MBUS_SLAVES_CHANGE_DATA_TIME:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Change Data Time:%d.\r\n", (int)Tick_Get( SECONDS ), (int)slave_num);
			if (MBus.attempt_n)
			{
				MBus.attempt_n--;
				__configureSlaveBaudrate( slaves_table, slave_num );
				uint8_t data_time[4];
//				time_tick_data = Tick_Get( SECONDS );
				mbus_data_tm_encode( &time_tick_data , data_time, 4);
				if ( 0 == mbus_send_change_parametrise(slaves_table->slave[slave_num].secondary, 0x04, -1, 0x6d, data_time, 4))
				{
					char mbus_on_demand[200];
					memset(json_array,  0, sizeof(json_array));
					MBus.attempt_n = 10;
					__updateState(1, MBUS_SLAVES_DISABLE/*MBUS_SLAVES_DATA_REQ_1*/);
					sprintf(mbus_on_demand, "%s;%d#", slaves_table->slave[slave_num].secondary, (int)time_tick_data );
					dlms_client_set_dlms_data_cmd_on_demand(mbus_on_demand);
				}
				else
				{
					LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> CHANGE DATA TIME ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				}
				time_tick_data = 0;
			}
			else
			{
//				MBus.error = 1;
//				__updateState( 1, MBUS_SLAVES_DISABLE );
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> RETRIES CHANGE LONG TELEGRAM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				MBus.attempt_n = 10;
				slave_num++;
				mbus_set_write_record(1);
				if ( slave_num < slaves_table->slaves_num )
				{
					sprintf( (char *)data,
							"*#*#*#*#*#*#*#MBUS-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
					raw_telegram_len = strlen((char *)data);
					__updateState( 1, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );
				}
				else
				{
					mbus_set_last_device(1);
					__updateState( 1, MBUS_SLAVES_DISABLE );
				}
			}
		}
		break;
	case MBUS_SLAVES_CHANGE_POINTER_TELEGRAM:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Change Data Time:%d.\r\n", (int)Tick_Get( SECONDS ), (int)slave_num);
			if (MBus.attempt_n)
			{
				MBus.attempt_n--;
				__configureSlaveBaudrate( slaves_table, slave_num );
				uint8_t data_pointer[3];
				data_pointer[0] = 0x00;
				data_pointer[1] = 0x21;
				data_pointer[2] = 0x80;
				if ( 0 == mbus_send_change_parametrise(slaves_table->slave[slave_num].secondary, 0x03, 0xfd, 0x1f, data_pointer, 3))
				{
//					char mbus_on_demand[200];
					memset(json_array,  0, sizeof(json_array));
					MBus.attempt_n = 10;
					__updateState(1, MBUS_SLAVES_DATA_REQ_1);
//					sprintf(mbus_on_demand, "%s;%d#", slaves_table->slave[slave_num].secondary, (int)time_tick_data );
//					dlms_client_set_dlms_data_cmd_on_demand(mbus_on_demand);
				}
				else
				{
					LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> CHANGE POINTER TELEGRAM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				}
			}
			else
			{
				MBus.error = 1;
				__updateState( 1, MBUS_SLAVES_DISABLE );
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> RETRIES CHANGE DATA TIME ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
			}
		}
		break;
	case MBUS_SLAVES_GET_BILLING_TELEGRAM:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Get Billing:%d.\r\n", (int)Tick_Get( SECONDS ), (int)slave_num);
			if (MBus.attempt_n)
			{
				MBus.attempt_n--;
				__configureSlaveBaudrate( slaves_table, slave_num );
				if ( 0 == billing_count )
				{
					subcode_billing  = (Tick_Get(SECONDS) - dlms_client_get_dlms_load_profile_end_time())/3600 - 1;
					subcode_billing  = (subcode_billing + 1) / (24 * 30) + 1;
					subcode2_billing = ((subcode_billing + 1) | 0xC0);
					billing_count = (dlms_client_get_dlms_load_profile_end_time() - dlms_client_get_dlms_load_profile_start_time())/3600 + 1;
					billing_count = billing_count / (24 * 30);
				}
				else
				{
					subcode_billing  += 1;
					subcode2_billing += 1;
					billing_count--;
				}
				if ( 0 == mbus_send_read_billing(slaves_table->slave[slave_num].secondary, subcode_billing, subcode2_billing))
				{
//					char mbus_on_demand[200];
					memset(json_array,  0, sizeof(json_array));
					MBus.attempt_n = 10;
					__updateState(1, MBUS_SLAVES_DATA_REQ_1);
//					sprintf(mbus_on_demand, "%s;%d#", slaves_table->slave[slave_num].secondary, (int)time_tick_data );
//					dlms_client_set_dlms_data_cmd_on_demand(mbus_on_demand);
				}
				else
				{
					LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> CHANGE BILLING TELEGRAM ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
				}
			}
			else
			{
				MBus.error = 1;
				__updateState( 1, MBUS_SLAVES_DISABLE );
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> RETRIES CHANGE DATA TIME ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
			}
		}
			break;
	case MBUS_SLAVES_DATA_REQ_1:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			if (1 == Tick_cloud_time_init())
			{
				if (MBus.attempt_n)
				{
					MBus.attempt_n--;
					if ( 0 == mbus_send_data_request_secondary_address() )
					{
//						memcpy(mbus_get_telegram_one(), (char *)mbus_protocol_get_mbus_json_array(), mbus_protocol_size_of_mbus_json_array() );
//						mbus_dewa_parse_telegram( slave_num );
//						memcpy(json_array, (char *) mbus_get_mbus_json_array(), sizeof(json_array));
						LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Datagram Slave:%d.\r\n", (int)Tick_Get( SECONDS ),(int)slave_num);
						slave_num++;
						mbus_set_write_record(1);
						if ( slave_num < slaves_table->slaves_num )
						{
							if ( 1 == mbus_billing_frame )
							{
								if (billing_count)
								{
									__updateState( 1, MBUS_SLAVES_GET_BILLING_TELEGRAM );
								}
								else
								{
									mbus_set_last_device(1);
									__updateState( 1, MBUS_SLAVES_DISABLE );
								}
							}
							else
							{
								__updateState( 1, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );
							}
						}
						else
						{
							if ( 1 == mbus_billing_frame )
							{
								if (billing_count)
								{
									__updateState( 1, MBUS_SLAVES_GET_BILLING_TELEGRAM );
								}
								else
								{
									mbus_set_last_device(1);
									__updateState( 1, MBUS_SLAVES_DISABLE );
								}
							}
							else
							{
								mbus_set_last_device(1);
								__updateState( 1, MBUS_SLAVES_DISABLE );
							}
						}
					}
					else
					{
//						MBus.error = 1;
						if ( 1 == mbus_billing_frame )
						{
							LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> BILLING DATA_REQ ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
							__updateState( 1, MBUS_SLAVES_GET_BILLING_TELEGRAM );
						}
						else
						{
							LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> DATA_REQ ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
							__updateState( 1, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );///__updateState( 1, MBUS_SLAVES_DISABLE );
						}
					}
				}
				else
				{
//					MBus.error = 1;
//					__updateState( 1, MBUS_SLAVES_DISABLE );
					LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> RETRIES DATA_REQ ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
					MBus.attempt_n = 10;
					slave_num++;
					mbus_set_write_record(1);
					if ( slave_num < slaves_table->slaves_num )
					{
						sprintf( (char *)data,
								"*#*#*#*#*#*#*#MBUS-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
						raw_telegram_len = strlen((char *)data);
						__updateState( 1, MBUS_SLAVES_CHANGE_LONG_TELEGRAM );
					}
					else
					{
						mbus_set_last_device(1);
						__updateState( 1, MBUS_SLAVES_DISABLE );
					}
				}
			}
		}
		break;
	case MBUS_SLAVES_DISABLE:
		if( CHECK_ELAPSED_TIME( mbus_task_get_mbus_delay(), TICK_MBUS ) )
		{
			if ( 1 == MBus.error )
			{
				MBus.error = 0;
//				sprintf( mbus_protocol_get_mbus_json_array(), "*#*#*#*#*#*#*#MBUS-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
//				memcpy(mbus_get_telegram_one(), (char *)mbus_protocol_get_mbus_json_array(), mbus_protocol_size_of_mbus_json_array() );
//				mbus_dewa_parse_telegram( 0 );
				mbus_task_set_write_record(1);
				mbus_set_write_record(1);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> ERROR!!!.\r\n", (int)Tick_Get( SECONDS ));
			}
			mbus_billing_frame = 0;
			billing_count      = 0;
#if 0
			mbus_set_acquire_telegram(0);
			mbus_set_end_comm(1);
//			mbus_set_write_record(1);
			mbus_set_last_device(1);
			mbus_set_start_comm(0);
			serial_mbus_pwr_disable();
			MX_USART1_USART_DeInit();
			leds_LED_Off(LED_GREEN);
//			AD_pressure_store_telegram_value();
#endif
			if (((0xF0 == subcode) && (1 == installation_sent)) || (subcode!=0xF0))
			{
				installation_sent = 0;
				if (0x80 == subcode)
				{
					subcode = 0xF0;
				}
#if 1
				mbus_set_acquire_telegram(0);
				mbus_set_end_comm(1);
//				mbus_set_write_record(1);
				mbus_set_last_device(1);
				mbus_set_start_comm(0);
				serial_mbus_pwr_disable();
				MX_USART3_USART_DeInit();
				leds_LED_Off(LED_GREEN);
//				AD_pressure_store_telegram_value();
#endif
				if ((1 == reed_sending) || (1 == powermon_sending))
				{
					powermon_sending = 0;
					udp_protocol_set_send_pending(1);
				}
				MBus.end = 1;
				MBus.mbus_status++;
			}
			else if ((0xF0 == subcode) && (0 == installation_sent))
			{
				subcode = 0x80;
				installation_sent = 1;
				__updateState( 1, MBUS_SLAVES_ENABLE );
			}
#if 0
			MBus.end = 1;
			MBus.mbus_status++;
#endif
		}
		break;
	case MBUS_SLAVES_OFF:
		if (0 == MBus.end)
		{
			MBus.mbus_status = MBUS_SLAVES_IDLE;
		}
		break;
	default:
		break;
	}
}

uint32_t time_tick;
void mbus_Task(void)
{
	uint32_t data_len;
	static uint8_t mbus_error, telegram_type;

	switch (mbus_status)
	{
		case MBUS_IDLE:
			/* If enabled initializes buffer */
			if ((1 == mbus_get_start_comm()) && (0 == params_pulses_on()) && (MBUS_METER_TYPE == params_get_uart_meter_type()))
			{
				end = 0;
				memset(json_array, 0, sizeof(json_array));
				memset(data,0, sizeof(data));
				memset(&frame, 0, sizeof(frame));
				memset(&data_frame, 0, sizeof(data_frame));
				Tick_update_tick(TICK_MBUS);
				t = 1;
				mbus_status++;
			}
		break;

		/* Initializes local MBUS USART and structure variables */
		case MBUS_ENABLE:
			attempt_n = 5;//10;
			Tick_update_tick(TICK_MBUS);
			t = 2;//1;
			/* MBUS Initialization */
			LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> Started.\r\n", (int)Tick_Get( SECONDS ));
			mbus_set_last_device(0);
			mbus_set_end_comm(0);
			serial_mbus_pwr_enable();
			HAL_Delay(10);
			MX_USART3_UART_Custom_Init();
//			leds_LED_On(LED_GREEN);
			mbus_status++;
#if 0
//				// TODO: Debugging.
//				uint32_t alarm = 15;
			param.meter.type = param.meter.type + 1;
			if (param.meter.type >= 1000) {
//					alarm            = 15;
				param.meter.type = 0;
			}
//				if (param.meter.type > 800)  {
//					alarm = 35;
//				}
//				else if (param.meter.type > 700) {
//					alarm = 0;
//				}
//				else if (param.meter.type > 500) {
//					alarm = 55;
//				}
//				else if (param.meter.type > 20) {
//					alarm = 89;
//				}
//				sprintf( json_array,
//						"2929#ELR#1#Water#39#438671770000e-3#123456789012e-4#M15QS0123%d#572222342321e-3#03.09.2018T23:08:35#00.00.2000T00:00:00#232641771234e-3"
//						"#0027e0#02.10.2001T21:30:00#0030e0#2970#33323333#15579222#00.00.2000T00:00:00#00.00.2000T00:00:00#00.00.2000T00:00:00#738321773242e-4#"
//						"02.10.2001T19:56:00#543320004356e-4#23234000#12345119#%d#",
//						param.meter.type,
//						param.meter.type);
			sprintf( json_array,
					"2929292929#ELR#1#water#39#%de-3#123456789012e-4#M15QS01%d  #572222342321e-3#2018-09-03 23:08:35#2000-00-00 00:00:00#232641771234e-3"
					"#0027e0#2001-10-02 21:30:35#0030e0#2970#33323333#15579222#2000-00-00 00:00:00#2000-00-00 00:00:00#2000-00-00 00:00:00#738321773242e-4#"
					"2001-10-02 19:56:00#543320004356e-4#-23234e-4#12345119#%d#",
					param.meter.type,
					param.meter.type,
					param.meter.type);
//						(int)alarm);
			mbus_dewa_parse_telegram( );
			mbus_status = MBUS_DISABLE;
#endif
		break;

		/* Sends the PING_REQ */
		case MBUS_PING_REQ:
			if (CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
			if(attempt_n)
			{
				frame.type    = MBUS_FRAME_TYPE_SHORT;
				frame.control = MBUS_CONTROL_MASK_SND_NKE | MBUS_CONTROL_MASK_DIR_M2S;
				frame.address = 254;//1;
				frame.start1  = MBUS_FRAME_SHORT_START;
				frame.stop    = MBUS_FRAME_STOP;
				data_len      = mbus_frame_pack(&frame, data, sizeof(data));

				if(data_len > 0)
				{
					serial_mbus_trx(&huart3, data, data_len);
				}

				attempt_n--;
				Tick_update_tick(TICK_MBUS);
				t = 1;
				mbus_status++;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> SEND NKE.\r\n", (int)Tick_Get( SECONDS ));
			}
			else
			{
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> ERROR! NO NKE ACK.\r\n", (int)Tick_Get( SECONDS ));
				mbus_error  = 1;
				mbus_status = MBUS_DISABLE;
			}
			}
		break;

		/* Processes MBUS slave PING RESPONSE */
		case MBUS_PING_RESP:
			if(serial_mbus_rcx_bytes_n())
			{
				serial_mbus_rcx_n(data, 1);
				serial_mbus_delete(serial_mbus_rcx_bytes_n());
				if(data[0] == 0xE5)
				{
					attempt_n = 3;
					if (1 == Tick_cloud_time_init())
					{
						mbus_status = MBUS_CHANGE_LONG_TELEGRAM;//MBUS_DATA_REQ_1;//
					}
					else
					{
						mbus_status = MBUS_DATA_REQ_1;
					}
					mbus_test_ok = 1;
					LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> NKE ACK.\r\n", (int)Tick_Get( SECONDS ));
				}
				else
				{
					mbus_status--;
					LOGLIVE(LEVEL_1, "LOGLIVE> %d MBUS> ERROR! WRONG NKE ACK.\r\n", (int)Tick_Get( SECONDS ));

				}
			}
			else if(CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				Tick_update_tick(TICK_MBUS);
				t = 1;
				mbus_status--;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d MBUS> ERROR! NKE ACK TIMEOUT.\r\n", (int)Tick_Get( SECONDS ));
			}
		break;

		/* */
		case MBUS_CHANGE_BAUDRATE:
			if (CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				if (attempt_n)
				{
					frame.type    = MBUS_FRAME_TYPE_LONG;
					frame.control = MBUS_CONTROL_MASK_SND_UD | MBUS_CONTROL_MASK_DIR_M2S;
					frame.address = 254;//1;
					frame.start1  = MBUS_FRAME_LONG_START;
					frame.start2  = MBUS_FRAME_LONG_START;
					frame.control_information = MBUS_CONTROL_INFO_SET_BAUDRATE_9600;
					frame.stop    = MBUS_FRAME_STOP;
					data_len      = mbus_frame_pack( &frame, data, sizeof( data ));

					if (data_len > 0)
					{
						serial_mbus_trx(&huart3, data, data_len);
					}

					attempt_n--;
					Tick_update_tick(TICK_MBUS);
					t = 1;
					mbus_status++;
				}
				else
				{
					mbus_error  = 1;
					mbus_status = MBUS_DISABLE;
				}
			}
		break;

		/**/
		case MBUS_CHANGE_BAUDRATE_RESP:
			if (serial_mbus_rcx_bytes_n())
			{
				serial_mbus_rcx_n(data, 1);
				serial_mbus_delete(serial_mbus_rcx_bytes_n());
				Tick_update_tick(TICK_MBUS);
				t = 1;

				if (data[0] == 0xE5)
				{
					attempt_n   = 3;
					mbus_status = MBUS_DATA_REQ_1;
					MX_USART3_USART_DeInit();
//					serial_mbus_enable_baudrate(9600);
				}
				else
				{
					mbus_status--;
				}
			}
			else if (CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				Tick_update_tick(TICK_MBUS);
				t = 1;
				mbus_status--;
			}
		break;

		case MBUS_CHANGE_LONG_TELEGRAM:
			if (CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				if (attempt_n)
				{
					frame.type    = MBUS_FRAME_TYPE_LONG;
					frame.control = MBUS_CONTROL_MASK_SND_UD | MBUS_CONTROL_MASK_DIR_M2S;
					frame.address = 254;//1;
					frame.start1  = MBUS_FRAME_CONTROL_START;
					frame.start2  = MBUS_FRAME_CONTROL_START;
					frame.control_information = MBUS_CONTROL_INFO_APPLICATION_RESET;
					frame.data_size = 1;
					frame.data[0]   = telegram_type = params_get_mbus_frame();//0x00;//0x02;//0x03;
					frame.stop      = MBUS_FRAME_STOP;
					data_len 	    = mbus_frame_pack( &frame, data, sizeof( data ));
//					uint8_t data_time[4];
//					time_tick = Tick_Get( SECONDS );
//					mbus_data_tm_encode( (time_t *)(&time_tick) , data_time, 2);
//					frame.type    = MBUS_FRAME_TYPE_LONG;
//					frame.control = MBUS_CONTROL_MASK_SND_UD;
//					frame.address = 0xFE;
//					frame.start1  = MBUS_FRAME_LONG_START;
//					frame.start2  = MBUS_FRAME_LONG_START;
//					frame.control_information = MBUS_CONTROL_INFO_DATA_SEND;
//					frame.data_size = 5;
//					frame.data[0]   = 0x42;
//					frame.data[1]   = 0xEC;
//					frame.data[2]   = 0x7E;
//					frame.data[3]   = data_time[0];
//					frame.data[4]   = data_time[1];
//					frame.stop      = MBUS_FRAME_STOP;
//					data_len        = mbus_frame_pack( &frame, data, sizeof( data ));

					if(data_len > 0)
					{
						serial_mbus_trx(&huart3, data, data_len);
					}

					attempt_n--;
					Tick_update_tick(TICK_MBUS);
					t = 1;
					mbus_status++;
				}
				else
				{
					mbus_error  = 1;
					mbus_status = MBUS_DISABLE;
				}
			}
		break;

		case MBUS_CHANGE_LONG_TELEGRAM_RESP:
			if (serial_mbus_rcx_bytes_n())
			{
				serial_mbus_rcx_n(data, 1);
				serial_mbus_delete(serial_mbus_rcx_bytes_n());
				Tick_update_tick(TICK_MBUS);
				t = 1;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> TELEGRAM TYPE 0x%X ACK.\r\n", (int)Tick_Get( SECONDS ), (int)telegram_type);
				if (data[0] == 0xE5)
				{
					attempt_n = 3;
					mbus_status = MBUS_DATA_REQ_1;
				}
				else
				{
					mbus_status--;
				}
			}
			else if(CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				Tick_update_tick(TICK_MBUS);
				t = 1;
				mbus_status--;
			}
		break;

		case MBUS_DATA_REQ:
			if (attempt_n)
			{
				frame.type    = MBUS_FRAME_TYPE_SHORT;
				frame.control = MBUS_CONTROL_MASK_REQ_UD2 | MBUS_CONTROL_MASK_DIR_M2S | MBUS_CONTROL_MASK_FCB;
				frame.address = 254;//1;//
				frame.start1  = MBUS_FRAME_SHORT_START;
				frame.stop    = MBUS_FRAME_STOP;
				data_len 	  = mbus_frame_pack(&frame, data, sizeof(data));

				if(data_len > 0)
				{
					serial_mbus_trx(&huart3, data, data_len);
				}

				attempt_n--;
				Tick_update_tick(TICK_MBUS);
				t = 1;//5;
				mbus_status++;
			}
			else
			{
				mbus_status = MBUS_DISABLE;
			}
		break;

		case MBUS_DATA_RESP:
			if (CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				if (serial_mbus_rcx_bytes_n())
				{
					data_len = serial_mbus_rcx(data);
					memset(&frame, 0, sizeof(frame));

					if(0 == mbus_parse(&frame, data, data_len))
					{
						if (0 == mbus_frame_data_parse(&frame, &data_frame))
						{
							json = mbus_frame_data_json( &data_frame );
							data_len = strlen( json );
							memcpy(json_array, (char *)mbus_frame_data_json( &data_frame ), data_len);
							mbus_parse_data( json_array );
							if (mbus_Water_Data.serial_num[0] != '\0') {
								mbus_status = MBUS_DISABLE;
							}
							else {
								mbus_status = MBUS_IDLE;
							}
						}
					}
					serial_mbus_delete( serial_mbus_rcx_bytes_n() );
				}
				else {
					Tick_update_tick( TICK_MBUS );
					t = 1;
					mbus_status--;
				}
			}
		break;

		case MBUS_DATA_REQ_1:
			if (attempt_n)
			{
				memset(json_array,  0, sizeof(json_array));
				memset(data,        0, sizeof(data));
				memset(&frame,      0, sizeof(frame));
				memset(&data_frame, 0, sizeof(data_frame));
				frame.type    = MBUS_FRAME_TYPE_SHORT;
				frame.control = MBUS_CONTROL_MASK_REQ_UD2 | MBUS_CONTROL_MASK_DIR_M2S | MBUS_CONTROL_MASK_FCB;
				frame.address = 254;//1;//
				frame.start1  = MBUS_FRAME_SHORT_START;
				frame.stop    = MBUS_FRAME_STOP;
				data_len 	  = mbus_frame_pack(&frame, data, sizeof(data));

				if(data_len > 0)
				{
					serial_mbus_trx(&huart3, data, data_len);
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> TX: %s.\r\n", (int)Tick_Get( SECONDS ), data);

				attempt_n--;
				Tick_update_tick(TICK_MBUS);
				t = 3;//2;
				mbus_status++;
			}
			else
			{
				mbus_error  = 1;
				mbus_status = MBUS_DISABLE;
			}
		break;

		case MBUS_DATA_RESP_1:
			if (serial_mbus_rcx_bytes_n())
			{
				memset(&frame, 0, sizeof(frame));
				uint32_t elapsed = 0;
				uint32_t initial_tick = Tick_Get(SECONDS);

				do {
					elapsed  = Tick_Get(SECONDS) - initial_tick;
					data_len = serial_mbus_rcx(data);
				} while ((mbus_parse(&frame, data, data_len) != 0) && (elapsed < 3));

				if (0 == mbus_frame_data_parse(&frame, &data_frame))
				{
					raw_telegram_len = data_len;
					memcpy(json_array, (char *) mbus_get_mbus_json_array(), sizeof(json_array));
					/* Only raw data is processed */
//					mbus_dewa_parse_telegram( );

					mbus_check_watermeter(&data_frame);
					/* Only raw data is processed */
#if 0
#	ifdef HYDRUS
					if (2 == mbus_get_manufacturer())
					{
						mbus_hydrus_parse_telegram();
					}
#	elif defined(BADGER)
					else if (1 == mbus_get_manufacturer())
					{
						mbus_badger_parse_telegram();
					}
#	else
					else
					{
						mbus_falconpr6_parse_telegram();
					}
#	endif
#endif
					mbus_status = MBUS_DISABLE;
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS> RX: %s\r\n", (int)Tick_Get( SECONDS ),(char *) data);
				serial_mbus_delete(serial_mbus_rcx_bytes_n());
			}
			else if (CHECK_ELAPSED_TIME(t,TICK_MBUS))
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d MBUS> ERROR! RX TIMEOUT. \r\n", (int)Tick_Get( SECONDS ));
				Tick_update_tick(TICK_MBUS);
				t = 1;
				mbus_status--;
			}
		break;

		case MBUS_DISABLE:
			if ( 1 == mbus_error ) {
				mbus_error = 0;
				sprintf( json_array,
						"*#*#*#*#*#*#*#MBUS-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
//				mbus_dewa_parse_telegram( );
				mbus_falconpr6_parse_telegram();
				sprintf( (char *)data,
						"*#*#*#*#*#*#*#MBUS-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
				data_len = strlen((char *)data);
				raw_telegram_len = data_len;
			}
			end = 1;
			mbus_set_acquire_telegram(0);
			mbus_set_end_comm(1);
			mbus_set_write_record(1);
			mbus_set_last_device(1);
			mbus_set_start_comm(0);
			serial_mbus_pwr_disable();
			MX_USART3_USART_DeInit();
//			leds_LED_Off(LED_GREEN);
//			AD_pressure_store_telegram_value();
			if ((1 == reed_sending) || (1 == powermon_sending))
			{
				powermon_sending = 0;
				udp_protocol_set_send_pending(1);
			}
			mbus_status++;
		break;

		case MBUS_OFF:
			if (0 == end)
			{
				mbus_status = MBUS_IDLE;
			}
		break;
	}
}

//static char* __replace_char(char* str, char find, char replace)
//{
//    char *current_pos = strchr(str,find);
//    while (current_pos)
//    {
//        *current_pos = replace;
//        current_pos = strchr(current_pos,find);
//    }
//    return str;
//}

static uint32_t __replacechar(char *str, char orig, char rep)
{
    char *ix = str;
    uint32_t n = 0;
    while((ix = strchr(ix, orig)) != NULL) {
        *ix++ = rep;
        n++;
    }
    return n;
}

/**
 * @fn void severn_trends_Task(void)
 * @brief Severn Trends system thread.
 *
\msc

a [label="Pipe20"],b [label="Meter"];
a-xb [label="PING_REQ 1 x5"];
 ---  [label = "END PROCESS MBUS ERROR"];
a=>b [label="PING_REQ"];
a<=b [label="PING_RESPONSE 0xE5"];
a=>b [label="SHORT TELEGRAM"];

\endmsc

 *
 *
 * */
void severn_trent_Task(void)
{
	uint32_t data_len;
	static uint8_t mbus_error;

	switch (severntrent_status)
	{
		case SEVERNTRENT_IDLE:
			if ((1 == mbus_get_start_comm()) && (0 == params_pulses_on()) && (SEVERN_TRENT_METER_TYPE == params_get_uart_meter_type()))
			{
//				memset(json_array,  0, sizeof(json_array));
				memset(data,        0, sizeof(data));
//				memset(&frame,      0, sizeof(frame));
//				memset(&data_frame, 0, sizeof(data_frame));
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> Started.\r\n", (int)Tick_Get( SECONDS ));
				mbus_set_last_device(0);
				mbus_set_end_comm(0);
				serial_mbus_pwr_disable();
//				leds_LED_On(LED_GREEN);
				end         = 0;
				attempt_n   = 5;
				Tick_update_tick(TICK_MBUS);
				t           = 100;
				severntrent_status = SEVERNTRENT_DATAREQ;
				MX_USART3_UART_Custom_Severn_Trent_Init();
			}
			break;

		case SEVERNTRENT_DATAREQ:
			if (CHECK_ELAPSED_MILISEC(t, TICK_MBUS))
			{
				severntrent_status = SEVERNTRENT_WAIT_DATAREQ;
				Tick_update_tick(TICK_MBUS);
				t           = SEVERN_TRENT_TDRW_MS - 1;
				SEVERN_TRENT_DATAREQ_ON();
//				data[0]  = '@';//0x40;
//				data_len = 1;
//				serial_mbus_trx(&huart1, data, data_len);
			}
			break;

		case SEVERNTRENT_WAIT_DATAREQ:
			if ( attempt_n != 0 )
			{
				if (CHECK_ELAPSED_MILISEC(t, TICK_MBUS))
				{
					SEVERN_TRENT_DATAREQ_OFF();
					severntrent_status = SEVERNTRENT_DATAOUT;
					Tick_update_tick(TICK_MBUS);
					t           = 3;//SEVERN_TRENT_TDRD_MS - SEVERN_TRENT_TDRW_MS - 1;
				}
			}
			else
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> ERROR! NO DATA OUT.\r\n", (int)Tick_Get( SECONDS ));
				mbus_error  = 1;
				severntrent_status = SEVERNTRENT_DISABLE;
			}
			break;

		case SEVERNTRENT_DATAOUT:
			if (serial_mbus_rcx_bytes_n() != 0)
			{
				uint32_t elapsed = 0;
				uint32_t initial_tick = Tick_Get(SECONDS);
				uint32_t rx_ok = 0;

				do {
					elapsed  = Tick_Get(SECONDS) - initial_tick;
					data_len = serial_mbus_rcx(data);
					if (( 0x2A == data[0] ) && (0x0D == data[SEVERN_TRENT_FRAME_LENGTH - 1]))
					{
						rx_ok = 1;
					}

				} while (((data_len < SEVERN_TRENT_FRAME_LENGTH) && (0 == rx_ok))
					  && (elapsed < 3));
				raw_telegram_len = data_len;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d SEVERN TRENT> RX: %s\r\n", (int)Tick_Get( SECONDS ),(char *) data);
				serial_mbus_delete(serial_mbus_rcx_bytes_n());
				if ( 1 == rx_ok )
				{
					severntrent_status = SEVERNTRENT_DISABLE;
				}
				else
				{
					attempt_n--;
					Tick_update_tick(TICK_MBUS);
					t = SEVERN_TRENT_TDRR_MS;
					severntrent_status = SEVERNTRENT_SEND_ABORT_AND_RETRY;
				}
			}
			else if (CHECK_ELAPSED_TIME(t, TICK_MBUS))
			{
				attempt_n--;
				Tick_update_tick(TICK_MBUS);
				t = SEVERN_TRENT_TDRR_MS;
				severntrent_status = SEVERNTRENT_SEND_ABORT_AND_RETRY;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> ERROR! RX TIMEOUT. \r\n", (int)Tick_Get( SECONDS ));
			}
			break;

		case SEVERNTRENT_DISABLE:
			if ( 1 == mbus_error )
			{
				mbus_error = 0;
				st_ok      = 2;
				sprintf( (char *)data,
						"*#*#*#*#*#*#*#SEVERN TRENT-ERROR#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#");
				data_len         = strlen((char *)data);
				raw_telegram_len = data_len;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> OFF.FRAME ERROR!\r\n", (int)Tick_Get( SECONDS ));
			}
			else
			{
				uint32_t n = __replacechar((char *)data, 0x0D, 0x23);//Replaces CR by #.
				if ( n == 2 )
				{
					st_ok = 1;
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> FRAME OK!\r\n", (int)Tick_Get( SECONDS ));
				}
				else
				{
					st_ok = 2;
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> FRAME CORRUPTED!\r\n", (int)Tick_Get( SECONDS ));
				}
			}
			end = 1;
			mbus_set_acquire_telegram(0);
			mbus_set_end_comm(1);
			mbus_set_write_record(1);
			mbus_set_last_device(1);
			mbus_set_start_comm(0);
//			MX_USART1_USART_DeInit();
//			leds_LED_Off(LED_GREEN);
			Tick_update_tick(TICK_MBUS);
			t   = 1;//SEVERN_TRENT_TDRR_MS;
			if ((1 == reed_sending) || (1 == powermon_sending))
			{
				powermon_sending = 0;
				udp_protocol_set_send_pending(1);
			}
			severntrent_status = SEVERNTRENT_OFF;
			break;

		case SEVERNTRENT_SEND_ABORT_AND_RETRY:
			if (CHECK_ELAPSED_MILISEC(t, TICK_MBUS))
			{
				SEVERN_TRENT_DATAREQ_ON();
				severntrent_status = SEVERNTRENTS_RETRY;
				t           = SEVERN_TRENT_TDRAW_MS;
				Tick_update_tick(TICK_MBUS);
			}
			break;

		case SEVERNTRENT_ABORT:
			if (0 == end)
			{
				if (CHECK_ELAPSED_MILISEC(t, TICK_MBUS))
				{
					SEVERN_TRENT_DATAREQ_ON();
					severntrent_status = SEVERNTRENT_OFF;
					t           = SEVERN_TRENT_TDRAW_MS;
					Tick_update_tick(TICK_MBUS);
				}
			}
			break;

		case SEVERNTRENTS_RETRY:
			if (CHECK_ELAPSED_MILISEC(t, TICK_MBUS))
			{
				SEVERN_TRENT_DATAREQ_OFF();
				t           = 1;
				Tick_update_tick(TICK_MBUS);
				severntrent_status = SEVERNTRENT_DATAREQ;
			}
			break;

		case SEVERNTRENT_OFF:
			if (CHECK_ELAPSED_MILISEC(t, TICK_MBUS))
			{
				SEVERN_TRENT_DATAREQ_OFF();
				MX_USART3_USART_DeInit();
				severntrent_status = SEVERNTRENT_IDLE;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SEVERN TRENT> OFF \r\n", (int)Tick_Get( SECONDS ));
			}
			break;

		default:
			break;
	}
}

#define DECIMAL(a)	((int)((fabsf((a-(float)((int)a))))*10))
static void mbus_dewa_device_battery( void )
{
	float num_sends = params_maintenance_number_ok_sendings() + params_maintenance_number_nok_sendings();//datalog_get_num_sends();
	float rem       = ( num_sends * 100 )/87600;
	float battery   = 100 - rem;

	sprintf( mbus_Dewa_Long_Telegram.device_battery, "%2d,%d", (int)battery, DECIMAL(battery));
}

char *mbus_dewa_device_get_battery_var( void )
{
	float num_sends = params_maintenance_number_ok_sendings() + params_maintenance_number_nok_sendings();//datalog_get_num_sends();
	float rem       = ( num_sends * 100 )/87600;
	float battery   = 100 - rem;

	sprintf( mbus_device_battery, "%2d.%d", (int)battery, DECIMAL(battery));

	return mbus_device_battery;
}

char *mbus_dewa_pressure_sensor( void )
{
	float_t pressure;

//	pressure = (AD_GetAverage()/1000 + 0.2176)/0.2943;
	pressure = AD_GetAverage()/1000;

	__ftoa( pressure, mbus_dewa_pressure, 3 );

	if ( 0 == pressure ) {
		sensor_log_SetOperatingAlarm(1);
	}

	return mbus_dewa_pressure;
}

static int __cz2_append_crc_frame( char read_frame[] )
{
	static unsigned short i, data;
	static unsigned int pos;
	static unsigned short accum, genpoly = 0x1021;

	accum = 0;
	pos   = 1;

	uint32_t len_frame = strlen(read_frame);

	if ( len_frame > 0 ) {
		while( pos != len_frame ) {
			data = read_frame[pos++] << 8;
			for ( i = 8; i > 0; i-- ) {
				if ( ( data ^ accum ) & 0x8000 ) {
					accum = ( accum << 1 ) ^ genpoly;
				} else {
					accum <<= 1;
				}
				data <<= 1;
			}
		}

	} else {
		accum = 1;
	}

	return accum;
}

void mbus_meter_frame_append_checksum( char *data, char *crc_field )
{
	unsigned short accum;
	uint32_t val;

	accum = __cz2_append_crc_frame( data );
	val   = ( accum & 0xF000 ) >> 12 | 0x30;
	crc_field[0] = val;
	val   = ( accum & 0x0F00 ) >> 8  | 0x30;
	crc_field[1] = val;
	val   = ( accum & 0x00F0 ) >> 4  | 0x30;
	crc_field[2] = val;
	val   = ( accum & 0x000F ) >> 0  | 0x30;
	crc_field[3] = val;
	crc_field[4] = '\0';
}

char crc_calc[512];
static char manufacturer_serial_num_backup[12]="00000000000";
static void mbus_falconpr6_parse_telegram( void )
{
	char *manufacturer_serial_num, *manufacturer, *version, *device_type;
	char *total_volume;
	char *reverse_volume;
	char *curr_date;
	char *last_annual_rep_date;
	char *vol_last_annual_rep;
	char *next_annual_rep_date;
	char *max_throughput;
	char *max_throughput_date;
	char *curr_throughput;
	char *status;
	char *read_num;
	char crc[5];

	memset(crc_calc, 0, 512);
    manufacturer_serial_num        = strtok( json_array, "#" );
    strncpy( mbus_PR6_Long_Telegram.manufacturer_serial_num, manufacturer_serial_num, 11 );
    strcat(crc_calc, manufacturer_serial_num);

    manufacturer 			= strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.manufacturer, manufacturer, 4);
    strcat(crc_calc, manufacturer);

    version 				= strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.version, version, 4);
    strcat(crc_calc, version);

    device_type = strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.device_type, device_type, 7);
    strcat(crc_calc, device_type);

    read_num    		    = strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.reads_num, read_num, 4);

    status                  = strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.status, status, 4);
    strcat(crc_calc, status);

    total_volume            = strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.total_volume, total_volume, 20);
    strcat(crc_calc, total_volume);

    reverse_volume          = strtok( NULL, "#" );
    snprintf(mbus_PR6_Long_Telegram.reverse_volume, sizeof(mbus_PR6_Long_Telegram.reverse_volume), "%s", reverse_volume);
    strcat(crc_calc, reverse_volume);

    curr_date               = strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.current_date, curr_date, 26);

    last_annual_rep_date    = strtok( NULL, "#" );
    strncpy(mbus_PR6_Long_Telegram.last_annual_rep_date, last_annual_rep_date, 26);
    strcat(crc_calc, last_annual_rep_date);

    vol_last_annual_rep     = strtok( NULL, "#" );
    snprintf(mbus_PR6_Long_Telegram.vol_last_annual_rep, sizeof(mbus_PR6_Long_Telegram.vol_last_annual_rep), "%s", vol_last_annual_rep);
    strcat(crc_calc, vol_last_annual_rep);

    next_annual_rep_date     = strtok( NULL, "#" );
    snprintf(mbus_PR6_Long_Telegram.next_annual_rep_date, sizeof(mbus_PR6_Long_Telegram.next_annual_rep_date), "%s", next_annual_rep_date);
    strcat(crc_calc, next_annual_rep_date);

    max_throughput_date     = strtok( NULL, "#" );
    snprintf(mbus_PR6_Long_Telegram.max_throughput_date, sizeof(mbus_PR6_Long_Telegram.max_throughput_date), "%s", max_throughput_date);
    strcat(crc_calc, max_throughput_date);

    curr_throughput         = strtok( NULL, "#" );
    snprintf(mbus_PR6_Long_Telegram.curr_throughput, sizeof(mbus_PR6_Long_Telegram.curr_throughput), "%s", curr_throughput);
    strcat(crc_calc, curr_throughput);

    max_throughput          = strtok( NULL, "#" );
    snprintf(mbus_PR6_Long_Telegram.max_throughput, sizeof(mbus_PR6_Long_Telegram.max_throughput), "%s", max_throughput);
    strcat(crc_calc, max_throughput);

//    month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );
//    month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );
//    month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );
//    month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );month1_date = strtok( NULL, "#" );

    snprintf(mbus_Dewa_Long_Telegram.device_identifier, sizeof(mbus_Dewa_Long_Telegram.device_identifier), "%s", Telit_dev_identifier());

    mbus_dewa_device_battery();

	mbus_meter_frame_append_checksum( crc_calc, crc );

    if ( memcmp( mbus_PR6_Long_Telegram.manufacturer_serial_num,
    	         manufacturer_serial_num_backup,
			     sizeof(mbus_PR6_Long_Telegram.manufacturer_serial_num) ) != 0 ) {
    	strncpy( manufacturer_serial_num_backup, mbus_PR6_Long_Telegram.manufacturer_serial_num, 11 );
    }

    if (memcmp( mbus_get_pr6_crc(),
	         crc,
		     sizeof(mbus_PR6_Long_Telegram.crc) ) != 0 ) {
    	mbus_set_frame_type(F_FRAME);
    } else {
    	mbus_set_frame_type(M_FRAME);
    }
    mbus_set_pr6_crc( crc );
}

static void mbus_check_watermeter( mbus_frame_data *data_frame )
{
	snprintf( mbus_Watermeter_Identifier.manufacturer, sizeof(mbus_Watermeter_Identifier.manufacturer), "%s", mbus_decode_manufacturer( data_frame->data_var.header.manufacturer[ 0 ], data_frame->data_var.header.manufacturer[ 1 ]));
	snprintf( mbus_Watermeter_Identifier.version, sizeof(mbus_Watermeter_Identifier.version), "%d", data_frame->data_var.header.version);
}

#ifdef HYDRUS
/**
 * @fn void mbus_hydrus_parse_telegram(void)
 * @brief
 *
 * @pre
 * @post
 */
static void mbus_hydrus_parse_telegram( void )
{
	char *manufacturer_serial_num, *manufacturer, *version, *device_type;
	char *dewa_serial_num;
	char *current_volume;
	char *reverse_volume;
	char *flow_rate;
	char *medium_temp;
	char *ambient_temp;
	char *battery_empty_date;
	char *operating_hours;
	char *error_hours;
	char *overflow_seconds_counter;
	char *overflow_event_counter;
	char *curr_date;
	char *status;
	char *read_num;
	char crc[5];

	memset(crc_calc, 0, 512);

// Header.
    manufacturer_serial_num = strtok( json_array, "#" );
    strncpy( mbus_Hydrus_Telegram.manufacturer_serial_num, manufacturer_serial_num, 11 );
    strcat(crc_calc, manufacturer_serial_num);

    manufacturer 			= strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.manufacturer, manufacturer, 4);
    strcat(crc_calc, manufacturer);

    version 				= strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.version, version, 4);
    strcat(crc_calc, version);

    device_type = strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.device_type, device_type, 7);
    strcat(crc_calc, device_type);

    read_num    		    = strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.reads_num, read_num, 4);

    status                  = strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.status, status, 4);
    strcat(crc_calc, status);
//End Header.

    read_num    		    = strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.reads_num, read_num, 4);

    manufacturer_serial_num = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.manufacturer_serial_num, sizeof(mbus_Hydrus_Telegram.manufacturer_serial_num), "%s", manufacturer_serial_num);

    dewa_serial_num    		= strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.dewa_serial_num, sizeof(mbus_Hydrus_Telegram.dewa_serial_num), "%s", dewa_serial_num);

    current_volume          = strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.current_volume, current_volume, 20);
    strcat(crc_calc, current_volume);

    reverse_volume          = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.reverse_volume, sizeof(mbus_Hydrus_Telegram.reverse_volume), "%s", reverse_volume);
    strcat(crc_calc, reverse_volume);

    flow_rate               = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.flow_rate, sizeof(mbus_Hydrus_Telegram.flow_rate), "%s", flow_rate);
    strcat(crc_calc, flow_rate);

    medium_temp             = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.medium_temperature, sizeof(mbus_Hydrus_Telegram.medium_temperature), "%s", medium_temp);
    strcat(crc_calc, medium_temp);

    ambient_temp            = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.ambient_temperature, sizeof(mbus_Hydrus_Telegram.ambient_temperature), "%s", ambient_temp);
    strcat(crc_calc, ambient_temp);

    battery_empty_date      = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.battery_empty_date, sizeof(mbus_Hydrus_Telegram.battery_empty_date), "%s", battery_empty_date);
    strcat(crc_calc, battery_empty_date);

    operating_hours         = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.operating_hours, sizeof(mbus_Hydrus_Telegram.operating_hours), "%s", operating_hours);
    strcat(crc_calc, operating_hours);

    error_hours             = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.error_hours, sizeof(mbus_Hydrus_Telegram.error_hours), "%s", error_hours);
    strcat(crc_calc, error_hours);

    overflow_seconds_counter = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.overflow_seconds_counter, sizeof(mbus_Hydrus_Telegram.overflow_seconds_counter), "%s", overflow_seconds_counter);
    strcat(crc_calc, overflow_seconds_counter);

    overflow_event_counter   = strtok( NULL, "#" );
    snprintf(mbus_Hydrus_Telegram.overflow_event_counter, sizeof(mbus_Hydrus_Telegram.overflow_event_counter), "%s", overflow_event_counter);
    strcat(crc_calc, overflow_event_counter);

    curr_date                = strtok( NULL, "#" );
    strncpy(mbus_Hydrus_Telegram.current_date, curr_date, 26);

    snprintf(mbus_Hydrus_Telegram.device_identifier, sizeof(mbus_Hydrus_Telegram.device_identifier), "%s", Telit_dev_identifier());

    mbus_dewa_device_battery();

	mbus_meter_frame_append_checksum( crc_calc, crc );

    if ( memcmp( mbus_Hydrus_Telegram.manufacturer_serial_num,
    	         manufacturer_serial_num_backup,
			     sizeof(mbus_Hydrus_Telegram.manufacturer_serial_num) ) != 0 ) {
    	strncpy( manufacturer_serial_num_backup, mbus_Hydrus_Telegram.manufacturer_serial_num, 11 );
    }

    if (memcmp( mbus_get_hydrus_crc(),
	         crc,
		     sizeof(mbus_Hydrus_Telegram.crc) ) != 0 ) {
    	mbus_set_frame_type(F_FRAME);
    } else {
    	mbus_set_frame_type(M_FRAME);
    }
    mbus_set_hydrus_crc( crc );
}
#endif
#ifdef BADGER
static void mbus_badger_parse_telegram( void )
{
	char *manufacturer_serial_num, *manufacturer, *version, *device_type;
	char *actuality_duration;
	char *dewa_serial_num;
	char *current_volume;
	char *record_date_volume;
	char *record_date_date;
	char *remaining_battery_life;
	char *reverse_volume;
	char *flow_rate;
	char *medium_temp;
	char *operating_hours;
	char *error_hours;
	char *overload_time;
	char *overload_hours_counter;
	char *ambient_temp;
	char *curr_date;
	char *status;
	char *read_num;
	char  crc[5];

	memset(crc_calc, 0, 512);

// Header.
    manufacturer_serial_num = strtok( json_array, "#" );
    strncpy( mbus_Badger_Telegram.manufacturer_serial_num, manufacturer_serial_num, 11 );
    strcat(crc_calc, manufacturer_serial_num);

    manufacturer 			= strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.manufacturer, manufacturer, 4);
    strcat(crc_calc, manufacturer);

    version 				= strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.version, version, 4);
    strcat(crc_calc, version);

    device_type = strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.device_type, device_type, 7);
    strcat(crc_calc, device_type);

    read_num    		    = strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.reads_num, read_num, 4);

    status                  = strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.status, status, 4);
    strcat(crc_calc, status);
//End Header.

    actuality_duration      = strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.actuality_duration, actuality_duration, sizeof(mbus_Badger_Telegram.actuality_duration));

    current_volume = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.current_volume, sizeof(mbus_Badger_Telegram.current_volume), "%s", current_volume);

    record_date_volume      = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.record_date_volume, sizeof(mbus_Badger_Telegram.record_date_volume), "%s", record_date_volume);

    record_date_date        = strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.record_date_date, record_date_date, sizeof(mbus_Badger_Telegram.record_date_date));
    strcat(crc_calc, record_date_date);

    remaining_battery_life  = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.remaining_battery_lifetime, sizeof(mbus_Badger_Telegram.remaining_battery_lifetime), "%s", remaining_battery_life);
    strcat(crc_calc, remaining_battery_life);

    curr_date               = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.current_date, sizeof(mbus_Badger_Telegram.current_date), "%s", curr_date);
//    strcat(crc_calc, curr_date);

    reverse_volume          = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.reverse_volume, sizeof(mbus_Badger_Telegram.reverse_volume), "%s", reverse_volume);
    strcat(crc_calc, reverse_volume);

    flow_rate               = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.flow_rate, sizeof(mbus_Badger_Telegram.flow_rate), "%s", flow_rate);
    strcat(crc_calc, flow_rate);

    medium_temp             = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.medium_temperature, sizeof(mbus_Badger_Telegram.medium_temperature), "%s", medium_temp);
    strcat(crc_calc, medium_temp);

    dewa_serial_num    		= strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.dewa_serial_num, sizeof(mbus_Badger_Telegram.dewa_serial_num), "%s", dewa_serial_num);

    operating_hours         = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.operating_hours, sizeof(mbus_Badger_Telegram.operating_hours), "%s", operating_hours);
    strcat(crc_calc, operating_hours);

    error_hours             = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.error_hours, sizeof(mbus_Badger_Telegram.error_hours), "%s", error_hours);
    strcat(crc_calc, error_hours);

    overload_time           = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.overload_time, sizeof(mbus_Badger_Telegram.overload_time), "%s", overload_time);
    strcat(crc_calc, overload_time);

    overload_hours_counter  = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.overload_hours_counter, sizeof(mbus_Badger_Telegram.overload_hours_counter), "%s", overload_hours_counter);
    strcat(crc_calc, overload_hours_counter);

    ambient_temp            = strtok( NULL, "#" );
    snprintf(mbus_Badger_Telegram.ambient_temperature, sizeof(mbus_Badger_Telegram.ambient_temperature), "%s", ambient_temp);
    strcat(crc_calc, ambient_temp);

    status                  = strtok( NULL, "#" );
    strncpy(mbus_Badger_Telegram.error_flag, status, sizeof(mbus_Badger_Telegram.error_flag));
    strcat(crc_calc, status);

    snprintf(mbus_Badger_Telegram.device_identifier, sizeof(mbus_Badger_Telegram.device_identifier), "%s", Telit_dev_identifier());

    mbus_dewa_device_battery();

	mbus_meter_frame_append_checksum( crc_calc, crc );

    if ( memcmp( mbus_Badger_Telegram.manufacturer_serial_num,
    	         manufacturer_serial_num_backup,
			     sizeof(mbus_Badger_Telegram.manufacturer_serial_num) ) != 0 ) {
    	strncpy( manufacturer_serial_num_backup, mbus_Badger_Telegram.manufacturer_serial_num, 11 );
    }

    if (memcmp( mbus_get_badger_crc(),
	         crc,
		     sizeof(mbus_Badger_Telegram.crc) ) != 0 ) {
    	mbus_set_frame_type(F_FRAME);
    } else {
    	mbus_set_frame_type(M_FRAME);
    }
    mbus_set_badger_crc( crc );
}
#endif
#if 0
static void mbus_dewa_parse_telegram( void )
{
	char *manufacturer_serial_num, *manufacturer, *version, *device_type;
	char *dewa_serial_num;
	char *total_volume;
	char *reverse_volume;
	char *flow_rate;
	char *special_read_date;
	char *special_read_value;
	char *module_temperature;
	char *meter_battery_remaining;
	char *operating_hours;
	char *error_hours;
	char *overload_time_start;
	char *overload_time_stop;
	char *last_overload_duration;
	char *total_overload_duration;
	char *status;
	char *timestamp;
	char *read_num, *peak_temperature_date, *peak_temperature_value, *peak_flow_rate_date, *peak_flow_rate_value, *minimum_flow_rate_date, *minimum_flow_rate_value;

    manufacturer_serial_num = strtok( json_array, "#" );
    strncpy(mbus_Dewa_Long_Telegram.manufacturer_serial_num, manufacturer_serial_num, 11);

    manufacturer 			= strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.manufacturer, manufacturer, 4);

    version 				= strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.version, version, 4);

    device_type = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.device_type, device_type, 7);

    read_num    		    = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.reads_num, read_num, 4);

    total_volume            = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.total_volume, total_volume, 20);

    flow_rate               = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.flow_rate, flow_rate, 20);

    dewa_serial_num    		= strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.dewa_serial_num, sizeof(mbus_Dewa_Long_Telegram.dewa_serial_num), "%s", dewa_serial_num);

    reverse_volume          = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.reverse_volume, reverse_volume, 20);

    timestamp               = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.timestamp, sizeof(mbus_Dewa_Long_Telegram.timestamp), "%s", timestamp);

    special_read_date       = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.special_read_date, sizeof(mbus_Dewa_Long_Telegram.special_read_date), "%s", special_read_date);

    special_read_value      = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.special_read_value, special_read_value, 20);

    module_temperature      = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.module_temperature, module_temperature, 10);

    peak_temperature_date   = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.peak_temperature_date, sizeof(mbus_Dewa_Long_Telegram.peak_temperature_date), "%s", peak_temperature_date);

    peak_temperature_value  = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.peak_temperature_value, peak_temperature_value, 10);

    meter_battery_remaining = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.meter_battery_remaining, meter_battery_remaining, 10);

    operating_hours         = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.operating_hours, operating_hours, 10);

    error_hours    		    = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.error_hours, error_hours, 10);

    overload_time_start     = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.overload_time_start, sizeof(mbus_Dewa_Long_Telegram.overload_time_start), "%s", overload_time_start);

    overload_time_stop      = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.overload_time_stop, sizeof(mbus_Dewa_Long_Telegram.overload_time_stop), "%s", overload_time_stop);

    peak_flow_rate_date     = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.peak_flow_rate_date, sizeof(mbus_Dewa_Long_Telegram.peak_flow_rate_date), "%s", peak_flow_rate_date);

    peak_flow_rate_value    = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.peak_flow_rate_value, peak_flow_rate_value, 20);

    minimum_flow_rate_date  = strtok( NULL, "#" );
    snprintf(mbus_Dewa_Long_Telegram.minimum_flow_rate_date, sizeof(mbus_Dewa_Long_Telegram.minimum_flow_rate_date), "%s", minimum_flow_rate_date);

    minimum_flow_rate_value = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.minimum_flow_rate_value, minimum_flow_rate_value, 20);

    last_overload_duration  = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.last_overload_duration, last_overload_duration, 10);

    total_overload_duration = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.total_overload_duration, total_overload_duration, 10);

    status                  = strtok( NULL, "#" );
    strncpy(mbus_Dewa_Long_Telegram.status, status, 4);

    snprintf(mbus_Dewa_Long_Telegram.device_identifier, sizeof(mbus_Dewa_Long_Telegram.device_identifier), "%s", Telit_dev_identifier());

    mbus_Dewa_Long_Telegram.val.counter          = atoi(mbus_Dewa_Long_Telegram.total_volume);

    mbus_dewa_device_battery();

//    mbus_build_dewa_telegram();
}
#endif

void mbus_get_date_filename( char *date_in, char *date_out )
{
    char *str;
    // www.epochconverter.com

    memset(date_out, 0, 20);

    str = strtok( &date_in[1], "- :" );
    strcpy(&date_out[0], str);
    str = strtok( NULL, "- :" );
    strncpy(&date_out[4], str, 2);
    str = strtok( NULL, "- :" );
    strncpy(&date_out[6], str, 2);
    str = strtok( NULL, "- :" );
    strncpy(&date_out[8], str, 2);
    str = strtok( NULL, "- :" );
    strncpy(&date_out[10], str, 2);
    str = strtok( NULL, "- :" );
    strncpy(&date_out[12], str, 2);
    str = strtok( NULL, "- :" );
    strncpy(&date_out[14], str, 2);
}

static void mbus_parse_data( char * json )
{
	char MBUS_str[300];
	char *serial_num;
	char *reads_num;
	char *counter;
	char *counter_inst;
	char *inst_flow;

    strncpy( MBUS_str, json, sizeof(MBUS_str) );
    serial_num   = strtok( MBUS_str, "#" );
    reads_num    = strtok( NULL, "#" );
    counter      = strtok( NULL, "#" );
    counter_inst = strtok( NULL, "#" );
    inst_flow    = strtok( NULL, "#" );

    strncpy(mbus_Water_Data.serial_num, serial_num, 20);

    strncpy(mbus_Water_Data.reads_num, reads_num, 4);

    strncpy(mbus_Water_Data.inst_flow, inst_flow, 12);
    mbus_Water_Data.val.inst_flow          = atoi(mbus_Water_Data.inst_flow);

    strncpy(mbus_Water_Data.counter, counter, 12);
    mbus_Water_Data.val.counter          = atoi(mbus_Water_Data.counter);
    mbus_Water_Data.previous_val.counter = mbus_Water_Data.val.counter ;

    strncpy(mbus_Water_Data.counter_inst, counter_inst, 12);
    mbus_Water_Data.val.counter_inst     = atoi(mbus_Water_Data.counter_inst);
}

char * mbus_get_data( void )
{
	static char no_data[] = "_;";
	if( json == NULL )
		json = no_data;

	return json;
}

char * mbus_get_telegram_one( void )
{
	static char no_data[] = "_;";
	if( json_array == NULL )
		memcpy(json_array, no_data, 2);

	return json_array;
}

char * mbus_parse_exp_data( char * data, uint8_t exp )
{
	char    *counter,  *counter2;
	char     count[10], count2[10];
	double    mantisa, mantisa_int, mantisa_diff;
	uint32_t exponent;

    counter  = strtok( data, "e-" );
    counter2 = strtok( NULL, "e-" );
    strcpy( count,  counter );
    strcpy( count2, counter2 );

    mantisa  = atof( count );
    exponent = atoi( count2 );

    if (exponent != exp) {
    	exp = exponent;
    }

    if (data[0] == '-') {
    	mantisa = -mantisa;
    }

    if ( exp == 3 ) {
    	mantisa = mantisa/1000;
    	mantisa_int = (double)((int)mantisa);
    	mantisa_diff = mantisa - mantisa_int;
    	if ( ( mantisa_int == 0 ) && ( mantisa_diff < 0 ) ) {
    		sprintf(data, "-%d.%03d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*1000)));
    	} else {
    		sprintf(data, "%d.%03d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*1000)));
    	}
    } else if ( exp == 4 ) {
    	mantisa = mantisa*0.0001;
    	mantisa_int = (double)((int)mantisa);
    	mantisa_diff = mantisa - mantisa_int;
    	if ( ( mantisa_int == 0 ) && ( mantisa_diff < 0 ) ) {
    		sprintf(data, "-%d.%04d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*10000)));
    	} else {
    		sprintf(data, "%d.%04d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*10000)));
    	}
    }  else if ( exp == 2 ) {
    	mantisa = mantisa/100;
    	mantisa_int = (double)((int)mantisa);
    	mantisa_diff = mantisa - mantisa_int;
    	if ( ( mantisa_int == 0 ) && ( mantisa_diff < 0 ) ) {
    		sprintf(data, "-%d.%02d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*100)));
    	} else {
    		sprintf(data, "%d.%02d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*100)));
    	}
    } else if ( exp == 1 ) {
    	mantisa = mantisa/10;
    	mantisa_int = (double)((int)mantisa);
    	mantisa_diff = mantisa - mantisa_int;
    	if ( ( mantisa_int == 0 ) && ( mantisa_diff < 0 ) ) {
    		sprintf(data, "-%d.%01d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*10)));
    	} else {
    		sprintf(data, "%d.%01d", (int)mantisa, ((int)((fabsl((mantisa_diff)))*10)));
    	}
    } else if ( exp == 0 ) {
    	sprintf(data, "%d", (int)mantisa);
    }

//    data[4]='\0';

    return data;
}

#if 0
char dst[512];
void mbus_build_dewa_telegram( void )
{
	char sep[2];
	char signal[10], serial_number[10];

	memset(signal,     0, sizeof(signal));
	memset(dst,        0, sizeof(dst));
	memset(json_array, 0, sizeof(json_array));

	sep[0] = ';';
	sep[1] = '\0';

	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

	if (mbus_Dewa_Long_Telegram.dewa_serial_num[0] == ' ') {
		uint8_t i1 = strlen(mbus_Dewa_Long_Telegram.manufacturer_serial_num);
		uint8_t i2 = i1 + strlen(mbus_Dewa_Long_Telegram.manufacturer);
		strncpy(mbus_Dewa_Long_Telegram.dewa_serial_num,      mbus_Dewa_Long_Telegram.manufacturer_serial_num, 5);
		strncpy(&mbus_Dewa_Long_Telegram.dewa_serial_num[i1], mbus_Dewa_Long_Telegram.manufacturer, 5);
		strncpy(&mbus_Dewa_Long_Telegram.dewa_serial_num[i2], mbus_Dewa_Long_Telegram.device_type, 7);
	}

	sprintf(signal,		   "%s", Telit_getSignalStrengthCondition());
	sprintf(serial_number, "%s", Telit_dev_identifier());

	//TODO: #serial-number y device-identification estaban cambiados el uno por el otro inicialmente.
	//#serial-number (primer campo) se corresponderá con el IMEI y device-identification con el manufacturer-serial-num del caudalímetro.
	//el campo 13 (customer-location) es el dewa-serial-num.
	strcat(dst, serial_number);										//#serial-number
	strcat(dst,sep);
	strcat(dst, mbus_get_watermeter_manufacturer_serial_num());	    //device-identification
//	strcat(dst, mbus_Dewa_Long_Telegram.manufacturer_serial_num);	//device-identification
//	strcat(dst, "16600015");										//device-identification//field test
	strcat(dst,sep);
	rtc_system_GetCreatedTime( Tick_Get( SECONDS ));
//	strcat(dst, mbus_Dewa_Long_Telegram.timestamp);					//created
	strcat(dst, rtc_system_getCreatedMessageTime());			    //created
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.version);					//value-data-count
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.manufacturer);				//manufacturer
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.version);					//version
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.device_type);				//device-type
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.reads_num);					//access-number
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.status);					//status
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.status);					//signature
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.total_volume, 3);
	strcat(dst, mbus_Dewa_Long_Telegram.total_volume);				//total volume 12 BCD exp -3
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.flow_rate, 4);
	strcat(dst, mbus_Dewa_Long_Telegram.flow_rate);					//flow-rate 12 bcd exp -4
	strcat(dst,sep);
	strcat(dst, mbus_get_dewa_serial_num());	                    //customer-location
//	strcat(dst, mbus_Dewa_Long_Telegram.dewa_serial_num);			//customer-location
//	strcat(dst, "16E0600015");										//customer-location//field test
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.reverse_volume, 3);
	strcat(dst, mbus_Dewa_Long_Telegram.reverse_volume);			//volume acc-of-abs-value-only-if-neg-contr  reverse volume 12 bcd exp -3
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.timestamp);					//datetime
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.special_read_date);			//datetime
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.special_read_value, 3);
	strcat(dst, mbus_Dewa_Long_Telegram.special_read_value);		//volume special read value 12bcd exp-3
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.module_temperature, 0);
	strcat(dst, mbus_Dewa_Long_Telegram.module_temperature);		//ext-temp 4 bcd exp 0
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.peak_temperature_date);		//datetime
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.peak_temperature_value, 0);
	strcat(dst, mbus_Dewa_Long_Telegram.peak_temperature_value);	//ext-temp peak temperature value 4 bcd exp0
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.meter_battery_remaining);	//remaining-battery-lifetime
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.operating_hours);			//op-time
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.error_hours);				//error hours
	strcat(dst,sep);
	strcat(dst, signal);											//signal strength
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.device_battery);			//device battery remaining
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.peak_flow_rate_date);		//datetime
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.peak_flow_rate_value, 4);
	strcat(dst, mbus_Dewa_Long_Telegram.peak_flow_rate_value);		//volume-flow 12 bcd exp-4
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.minimum_flow_rate_date);	//min-value-datetime
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.minimum_flow_rate_value, 4);
	strcat(dst, mbus_Dewa_Long_Telegram.minimum_flow_rate_value);	//volume-flow 12 bcd exp-4
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Dewa_Long_Telegram.last_overload_duration, 4);
	strcat(dst, mbus_Dewa_Long_Telegram.last_overload_duration);	//volume-flow duration-of-last-upper-limit-exceeded 8 bcd exp-4
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.total_overload_duration);	//op-time value-during-upper-limit-exceeded
	strcat(dst,sep);
	strcat(dst, mbus_Dewa_Long_Telegram.status);					//error-flags-dev-spec
//	strcat(dst,sep);

	memcpy(json_array, dst, sizeof(dst));
}

void mbus_build_falconpr6_telegram( void )
{
	char sep[2];
	char signal[10], serial_number[10];

	memset(signal,     0, sizeof(signal));
	memset(dst,        0, sizeof(dst));
	memset(json_array, 0, sizeof(json_array));

	sep[0] = ';';
	sep[1] = '\0';

	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

	if ( (mbus_PR6_Long_Telegram.pr6_serial_num[0] == ' ') || (mbus_PR6_Long_Telegram.pr6_serial_num[0] == '\0') ) {
		uint8_t i1 = strlen(mbus_Dewa_Long_Telegram.manufacturer_serial_num);
		uint8_t i2 = i1 + strlen(mbus_Dewa_Long_Telegram.manufacturer);
		strncpy(mbus_PR6_Long_Telegram.pr6_serial_num,      mbus_PR6_Long_Telegram.manufacturer_serial_num, 5);
		strncpy(&mbus_PR6_Long_Telegram.pr6_serial_num[i1], mbus_PR6_Long_Telegram.manufacturer, 5);
		strncpy(&mbus_PR6_Long_Telegram.pr6_serial_num[i2], mbus_PR6_Long_Telegram.device_type, 7);
	}

	sprintf(signal,		   "%s", Telit_getSignalStrengthCondition());
	sprintf(serial_number, "%s", Telit_dev_identifier());

	//TODO: #serial-number y device-identification estaban cambiados el uno por el otro inicialmente.
	//#serial-number (primer campo) se corresponderá con el IMEI y device-identification con el manufacturer-serial-num del caudalímetro.
	//el campo 13 (customer-location) es el dewa-serial-num.
	strcat(dst, serial_number);										//#serial-number
	strcat(dst,sep);
	strcat(dst, mbus_get_watermeter_pr6_manufacturer_serial_num()); //device-identification
	strcat(dst,sep);
	rtc_system_GetCreatedTime( Tick_Get( SECONDS ));
	strcat(dst, rtc_system_getCreatedMessageTime());			    //created
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.version);					//value-data-count
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.manufacturer);				//manufacturer
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.version);					//version
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.device_type);				//device-type
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.reads_num);					//access-number
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.status);					    //status
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.status);					    //signature
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_PR6_Long_Telegram.total_volume, 3);
	strcat(dst, mbus_PR6_Long_Telegram.total_volume);				//total volume 12 BCD exp -3
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_PR6_Long_Telegram.reverse_volume, 4);
	strcat(dst, mbus_PR6_Long_Telegram.reverse_volume);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_PR6_Long_Telegram.vol_last_annual_rep, 3);
	strcat(dst, mbus_PR6_Long_Telegram.vol_last_annual_rep);
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.last_annual_rep_date);
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.next_annual_rep_date);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_PR6_Long_Telegram.curr_throughput, 4);
	strcat(dst, mbus_PR6_Long_Telegram.curr_throughput);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_PR6_Long_Telegram.max_throughput, 4);
	strcat(dst, mbus_PR6_Long_Telegram.max_throughput);
	strcat(dst,sep);
	strcat(dst, mbus_PR6_Long_Telegram.max_throughput_date);
	strcat(dst,sep);

	strcat(dst, mbus_get_pr6_serial_num());	                    	//customer-location
	strcat(dst,sep);

	sprintf(mbus_PR6_Long_Telegram.device_battery, "%s", mbus_dewa_device_get_battery_var());
	strcat(dst, mbus_PR6_Long_Telegram.device_battery);			   //device battery remaining
	strcat(dst,sep);

	strcat(dst, mbus_Dewa_Long_Telegram.status);				   //error-flags-dev-spec

	memcpy(json_array, dst, sizeof(dst));
}

void mbus_build_hydrus_telegram( void )
{
	char sep[2];
	char signal[10], serial_number[10];

	memset(signal,     0, sizeof(signal));
	memset(dst,        0, sizeof(dst));
	memset(json_array, 0, sizeof(json_array));

	sep[0] = ';';
	sep[1] = '\0';

	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

	if ( (mbus_Hydrus_Telegram.dewa_serial_num[0] == ' ') || (mbus_Hydrus_Telegram.dewa_serial_num[0] == '\0') ) {
		uint8_t i1 = strlen(mbus_Hydrus_Telegram.manufacturer_serial_num);
		uint8_t i2 = i1 + strlen(mbus_Hydrus_Telegram.manufacturer);
		strncpy(mbus_Hydrus_Telegram.dewa_serial_num,      mbus_Hydrus_Telegram.manufacturer_serial_num, 5);
		strncpy(&mbus_Hydrus_Telegram.dewa_serial_num[i1], mbus_Hydrus_Telegram.manufacturer, 5);
		strncpy(&mbus_Hydrus_Telegram.dewa_serial_num[i2], mbus_Hydrus_Telegram.device_type, 7);
	}

	sprintf(signal,		   "%s", Telit_getSignalStrengthCondition());
	sprintf(serial_number, "%s", Telit_dev_identifier());

	strcat(dst, serial_number);										   //#serial-number
	strcat(dst,sep);
	strcat(dst, mbus_get_watermeter_hydrus_manufacturer_serial_num()); //device-identification
	strcat(dst,sep);
	rtc_system_GetCreatedTime( Tick_Get( SECONDS ));
	strcat(dst, rtc_system_getCreatedMessageTime());			//created
	strcat(dst,sep);

	strcat(dst, mbus_Hydrus_Telegram.version);					//value-data-count
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.manufacturer);				//manufacturer
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.version);					//version
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.device_type);				//device-type
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.reads_num);			    //access-number
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.status);					//status
	strcat(dst,sep);

	mbus_parse_exp_data(mbus_Hydrus_Telegram.current_volume, 3);
	strcat(dst, mbus_Hydrus_Telegram.current_volume);		    //total volume 12 BCD exp -3
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Hydrus_Telegram.reverse_volume, 3);
	strcat(dst, mbus_Hydrus_Telegram.reverse_volume);
	strcat(dst,sep);
	if (memcmp( mbus_Hydrus_Telegram.flow_rate, "ebbddd", 6 ) != 0 ) {
		mbus_parse_exp_data(mbus_Hydrus_Telegram.flow_rate, 3);
		strcat(dst, mbus_Hydrus_Telegram.flow_rate);
	} else {
    	strcat(dst, "ebbddd");
    }
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Hydrus_Telegram.medium_temperature, 4);
	strcat(dst, mbus_Hydrus_Telegram.medium_temperature);	  //medium temperature
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Hydrus_Telegram.ambient_temperature, 1);
	strcat(dst, mbus_Hydrus_Telegram.ambient_temperature);	  //ambient temperature
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.battery_empty_date);	  //battery empty date
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.operating_hours);	      //operating hours
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.error_hours);	          //error hours
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.overflow_seconds_counter);	//overflow seconds counter
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.overflow_event_counter);	//overflow events counter
	strcat(dst,sep);
	strcat(dst, mbus_Hydrus_Telegram.current_date);				//current date
	strcat(dst,sep);

	strcat(dst,mbus_Hydrus_Telegram.dewa_serial_num);	        //customer-location
	strcat(dst,sep);

	sprintf(mbus_Hydrus_Telegram.device_battery, "%s", mbus_dewa_device_get_battery_var());
	strcat(dst, mbus_Hydrus_Telegram.device_battery);		   //device battery remaining
	strcat(dst,sep);

	memcpy(json_array, dst, sizeof(dst));
}

void mbus_build_badger_telegram( void )
{
	char sep[2];
	char signal[10], serial_number[10];

	memset(signal,     0, sizeof(signal));
	memset(dst,        0, sizeof(dst));
	memset(json_array, 0, sizeof(json_array));

	sep[0] = ';';
	sep[1] = '\0';

	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

	if ( (mbus_Badger_Telegram.dewa_serial_num[0] == ' ') || (mbus_Badger_Telegram.dewa_serial_num[0] == '\0') ) {
		uint8_t i1 = strlen(mbus_Badger_Telegram.manufacturer_serial_num);
		uint8_t i2 = i1 + strlen(mbus_Badger_Telegram.manufacturer);
		strncpy(mbus_Badger_Telegram.dewa_serial_num,      mbus_Badger_Telegram.manufacturer_serial_num, i1);
		strncpy(&mbus_Badger_Telegram.dewa_serial_num[i1], mbus_Badger_Telegram.manufacturer, 5);
		strncpy(&mbus_Badger_Telegram.dewa_serial_num[i2], mbus_Badger_Telegram.device_type, 7);
	}

	sprintf(signal,		   "%s", Telit_getSignalStrengthCondition());
	sprintf(serial_number, "%s", Telit_dev_identifier());

	strcat(dst, serial_number);										   //#serial-number
	strcat(dst,sep);
	strcat(dst, mbus_get_watermeter_badger_manufacturer_serial_num()); //device-identification
	strcat(dst,sep);
	rtc_system_GetCreatedTime( Tick_Get( SECONDS ));
	strcat(dst, rtc_system_getCreatedMessageTime());			//created
	strcat(dst,sep);

	strcat(dst, mbus_Badger_Telegram.version);					//value-data-count
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.manufacturer);				//manufacturer
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.version);					//version
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.device_type);				//device-type
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.reads_num);			    //access-number
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.status);					//status
	strcat(dst,sep);

	strcat(dst, mbus_Badger_Telegram.actuality_duration);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Badger_Telegram.current_volume, 4);
	strcat(dst, mbus_Badger_Telegram.current_volume);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Badger_Telegram.record_date_volume, 4);
	strcat(dst, mbus_Badger_Telegram.record_date_volume);
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.record_date_date);
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.remaining_battery_lifetime);
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.current_date);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Badger_Telegram.reverse_volume, 2);
	strcat(dst, mbus_Badger_Telegram.reverse_volume);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Badger_Telegram.flow_rate, 4);
	strcat(dst, mbus_Badger_Telegram.flow_rate);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Badger_Telegram.medium_temperature, 3);
	strcat(dst, mbus_Badger_Telegram.medium_temperature);
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.operating_hours);
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.error_hours);
	strcat(dst,sep);
	strcat(dst, mbus_Badger_Telegram.overload_time);
	strcat(dst,sep);
	mbus_parse_exp_data(mbus_Badger_Telegram.ambient_temperature, 0);
	strcat(dst, mbus_Badger_Telegram.ambient_temperature);
	strcat(dst,sep);

	strcat(dst,mbus_Badger_Telegram.dewa_serial_num);	                //customer-location
	strcat(dst,sep);

	sprintf(mbus_Badger_Telegram.device_battery, "%s", mbus_dewa_device_get_battery_var());
	strcat(dst, mbus_Badger_Telegram.device_battery);		   //device battery remaining
	strcat(dst,sep);

	memcpy(json_array, dst, sizeof(dst));
}

void mbus_build_typeM_telegram( char *telegram_1, uint32_t *num_chars )
{
	uint32_t len = 0;
	if ( 2 == mbus_get_manufacturer() ) {
		len = strlen(mbus_Hydrus_Telegram.current_volume);
	}
	else if ( 1 == mbus_get_manufacturer() ) {
		len = strlen(mbus_Badger_Telegram.current_volume);
	}
	else {
		len = strlen(mbus_PR6_Long_Telegram.total_volume);
	}

	*num_chars = len;
	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

	memcpy(&telegram_1[0], rtc_system_getCreatedMeterValueTime(), 5);

	if ( 2 == mbus_get_manufacturer() ) {
		memcpy(&telegram_1[4], mbus_Hydrus_Telegram.current_volume, len);
	}
	else if ( 1 == mbus_get_manufacturer() ) {
		memcpy(&telegram_1[4], mbus_Badger_Telegram.current_volume, len);
	}
	else {
		memcpy(&telegram_1[4], mbus_PR6_Long_Telegram.total_volume, len);
	}
}
#endif

#endif
/**
 * @}
 */ /* End addtogroup system mbus*/
