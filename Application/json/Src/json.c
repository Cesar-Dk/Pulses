/**
  ******************************************************************************
  * @file           json.c
  * @author 		Datakorum Development Team
  * @brief          Java Script Object Notation firmware for JSON data object
  *  parser.
  *
  * @note JSON is an open standard file format, and data interchange format, that
  * uses human-readable text to store and transmit data objects consisting of
  * attribute–value pairs and array data types (or any other serializable value).
  *
  * @see https://www.json.org/json-en.html
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution SL.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * json.c
 *
 *  Created on: 15 may. 2018
 *      Author: Sergio Mill�n L�pez
 */
#define _GNU_SOURCE
#include <string.h>

#include "json.h"
#include "gpio.h"
#include "ad.h"
#include "shutdown.h"
#include "modbus.h"
#include "modbus_sensors.h"
#include "generic_modbus.h"
#include "generic_modbus_table.h"
#include "i2c_sensor.h"
#include "common_lib.h"
#include "pulses.h"
#include "udp_protocol.h"
#include "iwdg.h"

#ifdef MBUS
#include "mbus.h"
#include "mbus_slave_table_manager.h"
#endif
#ifdef UNE82326
#   include "une82326_device_table.h"
#endif
#ifdef DLMS
#include "dlms_client.h"
#include "dlms_client_table.h"
#include "dlms_client_on_demand_comm.h"
#endif
#ifdef MQTT
#include "mqtt_task.h"
#include "message_queue.h"
#include "mqtt_frames.h"
#include "mqtt_buffer.h"
#endif
/** *
 * @defgroup System_JSON JSON
 * @brief
 * @{
 */

#define MAX_TOKENS      (1000)//(950)//(850)//(750)//(550)//(400)//(300)//(200)//(180)//(140)

#define max(a,b)        (((a) > (b)) ? (a) : (b))
#define min(a,b)        (((a) < (b)) ? (a) : (b))

static jsmn_parser parser;
static jsmntok_t   token[MAX_TOKENS];
static int32_t     r;

static char read_hours[257];
static uint32_t srv_add[30][2];
static uint32_t dlms_profiles_configs[30];
static uint32_t params_change = 0;
static uint32_t get_params = 0;

extern connection con;

void __reset_strlen_vars(void);

/**
 * @fn char json_read_hours*(void)
 * @brief
 *
 * @return
 */
char * json_read_hours( void )
{
	return read_hours;
}

/**
 * @fn uint32_t json_decode(char*, uint32_t)
 * @brief
 *
 * @param str
 * @param len
 * @return ret
 * 			@arg 0 - Parameter not found
 * 			@arg 1 - JSON not received
 * 			@arg 2 -
 */
uint32_t json_decode(char *str, uint32_t len)
{
	int32_t  i = 0, k = 0, t = 0, z = 0;//, j = 0, k_back = 0,
	int32_t  data[ 26 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	char     string_aux[ 1024 ], string_aux_2[ 256 ];
	uint32_t ret = 1, read_hours_len = 0;
	uint32_t profile_num = 0;
	uint32_t count_profile = 0;

	memset( string_aux, 0, 256 );

	jsmn_init(&parser);
	r = jsmn_parse(&parser, str, len, token, MAX_TOKENS);
	if (r < 1) {
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON>ERROR:%d !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n", (int)Tick_Get( SECONDS ), (int)r);
		return 1;// The server is not sending json, the device must work anyway.
	}

	if ((i = json_search_section(0, "Gateway-ID", str))) {
		json_get_str(i, str, string_aux);
#ifdef MBUS
//		mbus_set_watermeter_pr6_manufacturer_serial_num(string_aux);
#endif
	}
	else {
		return 1;// The server is not sending json, the device must work anyway.
	}

	if (((udp_get_server_resp_type() == 5) || (udp_get_server_resp_type() == 1) || (udp_get_server_resp_type() == 2)) && (0==get_params))
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON>NOT PARAMS FRAME !!\r\n", (int)Tick_Get( SECONDS ));
		return 1;
	}

	get_params = 0;

	if (strncmp(string_aux, Telit_dev_identifier(), 9)) {
		return 2;
	}

	if ((i = json_search_section(0, "Middleware-ID", str))) {
		json_get_str(i, str, string_aux);
		params_set_middleware_id(string_aux);
#if defined (MBUS)
		mbus_set_dewa_serial_num(string_aux);
#elif defined(UNE82326)
			une82326_device_table_manager_get_device_serial_num(0);
#endif
	}


	if(( i = json_search_section( 0, "apn", str ))) {
		json_get_str( i, str, param.APN[0].name );
	}

	if(( i = json_search_section( 0, "server", str ))) {
		json_get_str( i, str, param.server[0].name );
	}

	if ((i = json_search_section(0, "eDRX", str))) {
		data[0] = json_get_int(i, str);
		params_psm_edrx_set_edrx(data[0]);
	}

	if ((i = json_search_section(0, "PSM", str))) {
		json_get_str( i, str, string_aux );
		json_get_psm( string_aux );
	}

	if ((i = json_search_section(0, "rt1", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 0, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt2", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 1, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt3", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 2, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt4", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 3, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt5", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 4, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt6", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 5, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt7", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 6, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt8", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 7, string_aux );
		sprintf( read_hours + read_hours_len, "%s", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "st", str))) {
		json_get_str( i, str, string_aux );
		json_get_send_hours( string_aux );
	}

	if ((i = json_search_section(0, "srt", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_sensor_read_hours(0, string_aux);
	}

	if ((i = json_search_section(0, "srt2", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_sensor_read_hours(1, string_aux);
	}

	if ((i = json_search_section(0, "sst", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_sensor_send_hours( string_aux );
	}

	if ((i = json_search_section(0, "mbrt", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_modbus_read_hours(0, string_aux_2);
	}
	else
	{
		param.modbus_read_time[0].init_time = -1;
		param.modbus_read_time[0].end_time  = -1;
		param.modbus_read_time[0].cycle     = -1;
	}

	if ((i = json_search_section(0, "mbrt2", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_modbus_read_hours(1, string_aux_2);
	}
	else
	{
		param.modbus_read_time[1].init_time = -1;
		param.modbus_read_time[1].end_time  = -1;
		param.modbus_read_time[1].cycle     = -1;
	}

	if ((i = json_search_section(0, "mbrt3", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_modbus_read_hours(2, string_aux_2);
	}
	else
	{
		param.modbus_read_time[2].init_time = -1;
		param.modbus_read_time[2].end_time  = -1;
		param.modbus_read_time[2].cycle     = -1;
	}

	if ((i = json_search_section(0, "mbst", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_modbus_send_hours( string_aux_2 );
	}
	else
	{
		param.modbus_send_time[1].send_time = -1;
	}

	if ((i = json_search_section(0, "pst", str))) {
		json_get_str( i, str, string_aux );
		json_get_send_network_params_hours( string_aux );
	}

	if ((i = json_search_section(0, "oversensor_alarm", str))) {
		json_get_str( i, str, string_aux );
		json_get_sensor_over_sensor_alarms( string_aux );
	}

	if ((i = json_search_section(0, "lowsensor_alarm", str))) {
		json_get_str( i, str, string_aux );
		json_get_sensor_low_sensor_alarms( string_aux );
	}

	if ((i = json_search_section(0, "oversensor_condition", str))) {
		data[0] = json_get_int(i, str);
		params_set_over_sensor_condition(data[0]);
	}

	if ((i = json_search_section(0, "lowsensor_condition", str))) {
		data[0] = json_get_int(i, str);
		params_set_low_sensor_condition(data[0]);
	}

	if ((i = json_search_section(0, "release_assistance", str))) {
		data[6] = json_get_int(i, str);
		params_manteinance_set_release_assistance(data[6]);
	}

	if ((i = json_search_section(0, "fwupdate", str))) {
		data[7] = json_get_int(i, str);
		if ( 1 == data[7] ) {
			ret = 3;
		}
	}

	if ((i = json_search_section(0, "firmware_version", str))) {
		json_get_str(i, str, string_aux);
		rest_get_fw_version(string_aux);
	}

	if ((i = json_search_section(0, "local_port", str))) {
		data[8] = json_get_int(i, str);
		params_modbus_log_set_enabled( data[8] );
	}

	if ((i = json_search_section(0, "reed_activation", str))) {
		data[9] = json_get_int(i, str);
		params_manteinance_set_reed_activation(data[9]);
	}

	if ((i = json_search_section(0, "delete_memory", str))) {
		data[10] = json_get_int(i, str);
		params_manteinance_set_delete_memory(data[10]);
		Datalogger_Set_Send_Disable(DATALOGGER_SEND_ENABLED);
		if ( CLEAR_MEMORY_FORCE_HARD_RESET == data[10] ) {
			ret = 4;
		} else if ( CLEAR_MEMORY_DISABLE_DATALOGGER == data[10] ) {
			Datalogger_Set_Send_Disable(DATALOGGER_SEND_DISABLED);
		}
	}

	if ((i = json_search_section(0, "reset_meters", str))) {
		data[11] = json_get_int(i, str);
		params_manteinance_set_reset_meters(data[11]);
	}

	if ((i = json_search_section(0, "reset_pulse_totalizer", str))) {
		data[11] = json_get_int(i, str);
		if ( 1 == data[11] )
		{
			params_reset_pulses_totalizer();
		}
	}

	if ((i = json_search_section(0, "private_key", str))) {
		json_get_str(i, str, string_aux);
		modbus_set_tool_cert(string_aux);
	}

	if ((i = json_search_section(0, "pulse_on", str))) {
		data[12] = json_get_int(i, str);
		if (data[12] > 3)
		{
			data[12] = 3;
		}
		pulses_set_input_num(data[12]);
		if (data[12] != 0)
		{
			params_pulses_set_on(1);
			pulses_init();
		}
		else {
			params_pulses_set_on(0);
		}
	}

	if ((i = json_search_section(0, "Meter-ID", str))) {
		json_get_str(i, str, string_aux);
		params_set_meter_id(string_aux);
#if defined(MBUS)
		mbus_set_watermeter_manufacturer_serial_num(string_aux);
#elif defined(UNE82326)
			une82326_device_table_manager_get_device_serial_num(0);
#endif
	}

	if ((i = json_search_section(0, "counter_factor", str))) {
		data[13] = json_get_int(i, str);
		params_pulses_set_pulse_factor(data[13]);
	}

	if ((i = json_search_section(0, "k1_factor", str))) {
		data[14] = json_get_int(i, str);
		params_pulses_set_pulse_unit_k_factor_out_1(data[14]);
	}

	if ((i = json_search_section(0, "k2_factor", str))) {
		data[15] = json_get_int(i, str);
		params_pulses_set_pulse_unit_k_factor_out_2(data[15]);
	}

	if ((i = json_search_section(0, "wq", str))) {
		data[16] = json_get_int(i, str);
		params_wq_set_on(data[16]);
	}

	if ((i = json_search_section(0, "sformat", str))) {
		data[6] = json_get_int(i, str);
		params_set_telegram_mode(data[6]+1);
	}

	if ((i = json_search_section(0, "sprotocol", str))) {
		json_get_str(i, str, string_aux);
		data[6] = 1;
		if ( 0 == memcmp( string_aux, "UDP", 3 ) ) {
			data[6] = 2;
		} else {
			data[6] = 1;
		}
		params_set_protocol_mode(data[6]);
	}

	if ((i = json_search_section(0, "sport", str))) {
		data[6] = json_get_int(i, str);
		param.server[0].port = data[6];
	}

	if ((i = json_search_section(0, "snbpower", str))) {
		data[6] = json_get_int(i, str);
		params_set_nbiotpwr_mode(data[6]+1);
	}

	if ( ( i = json_search_section(0, "rt_period", str) ) ) {
		data[4] = json_get_int(i, str);
		if ( (data[4] == 0) || (data[4] == 15) || (data[4] == 30) || (data[4] == 60)  || (data[4] == 120) || (data[5] == 180) || (data[4] == 240) || (data[4] == 360) || (data[4] == 1380) || (data[4] == 1440)) {
			if ( 0 == data[4] ) {
				params_config_set_disable_meter(1);
				data[4] = param.config.st/60;//15;
			} else {
				params_config_set_disable_meter(0);
			}
			data[4] = data[4] * 60;
			if (data[4] != param.config.rt) {
				param.config.send_count = 0;
				shutdown_reset_meter_count();
			}
			param.config.rt = data[4];
		}
	}

	if ( ( i = json_search_section(0, "st_period", str) ) ) {
		data[5] = json_get_int(i, str);
		if ( (data[5] == 15) || (data[5] == 30) || (data[5] == 60) || (data[5] == 120) || (data[5] == 180) || (data[5] == 240) || (data[5] == 360) || (data[5] == 1080) || (data[5] == 1380) || (data[5] == 1440) ) {
			data[5] = data[5] * 60;
			if (data[5] != param.config.st) {
				param.config.send_count = 0;
				shutdown_reset_meter_count();
			}
			param.config.st = data[5];
			if ( 1 == params_config_get_disable_meter())
			{
				param.config.rt = param.config.st;
			}
		}
	}

	if ((i = json_search_section(0, "period", str))) {
		data[16] = json_get_int(i, str);
		params_config_set_period(data[16]);
	}

	if ((i = json_search_section(0, "sensor", str))) {
		data[16] = json_get_int(i, str);
		AD_SetADOn(data[16]);
	}

	if ((i = json_search_section(0, "i2c_sensor", str))) {
		data[16] = json_get_int(i, str);
		params_set_i2c_sensor(data[16]);
	}

	if ((i = json_search_section(0, "sensor_log", str))) {
		data[16] = json_get_int(i, str);
		params_config_set_sensor_log(data[16]);
		if ( 1 == params_pulses_on() ) {
			params_config_set_sensor_log(1);
			AD_SetADOn(1);
		}
	} else {
		if ( 0 == params_pulses_on() ) {
			params_config_set_sensor_log(0);
		} else {
			params_config_set_sensor_log(1);
			AD_SetADOn(1);
		}
	}

	if ((i = json_search_section(0, "MODBUS_dt", str))) {
		data[17] = json_get_int(i, str);
		generic_485_set_device(data[17]);
		if ( 1 == data[17] ) {
//			generic_485_set_enable(0);
//			generic_485_set_type(MODBUS_RAW);
			generic_485_set_type_and_disable(MODBUS_RAW);
			params_config_set_sensor_log(1);
		} else if ( 2 == data[17] ) {
//			generic_485_set_enable(0);
//			generic_485_set_type(MODBUS_RAW);
			generic_485_set_type_and_disable(MODBUS_RAW);
			generic_485_set_dtl645(1);
			params_config_set_sensor_log(1);
		}
		else {
//			generic_485_set_enable(0);
//			generic_485_set_type(LAST_GENERIC_485_TYPE);
			generic_485_set_type_and_disable(LAST_GENERIC_485_TYPE);
		}
	}

	if ((i = json_search_section(0, "modbus_speed", str))) {
		data[18] = json_get_int(i, str);
	    modbus_sensors_set_serial_config_baud_rate( data[18] );
	}

	if ((i = json_search_section(0, "modbus_mode", str))) {
//		json_get_str(i, str, string_aux);
		data[19] = json_get_int(i, str);//1;
		modbus_sensors_set_serial_config_stop_bits( data[19] );
//		if ( 0 == memcmp( string_aux, "8N1", 3 ) ) {
//			 modbus_sensors_set_serial_config_stop_bits( serial_8N1 );
//		} else if ( 0 == memcmp( string_aux, "8N2", 3 ) ) {
//			 modbus_sensors_set_serial_config_stop_bits( serial_8N2 );
//		}
	}

	if ((i = json_search_section(0, "modbus_parity", str))) {
//		json_get_str(i, str, string_aux);
		data[19] = json_get_int(i, str);//1;
		modbus_sensors_set_serial_config_parity( data[19] );
//		if ( 0 == memcmp( string_aux, "even", 4 ) ) {
//			modbus_sensors_set_serial_config_parity( parity_even );
//		} else if ( 0 == memcmp( string_aux, "odd", 3 ) ) {
//			modbus_sensors_set_serial_config_parity( parity_odd );
//		} else {
//			modbus_sensors_set_serial_config_parity( no_parity );
//		}
	}

	if ((i = json_search_section(0, "sync_read", str))) {
		data[20] = json_get_int(i, str);
		params_config_set_sync_read(data[20]);
	}

	uint32_t modbus_set = 1;
	if ((i = json_search_section(0, "modbus_set", str))) {
		data[20] = json_get_int(i, str);
		modbus_set = data[20];
	}

	if ( 1 == modbus_set )
	{
		__reset_strlen_vars();
		if ((i = json_search_section(0, "modbus_num", str))) {
			data[20] = json_get_int(i, str);
			generic_modbus_table_set_num_devices(data[20]);
		}

		if ((i = json_search_section(0, "modbus_warm_time", str))) {
			data[20] = json_get_int(i, str);
			params_set_modbus_warm_time(data[20]);
		}

		if ((i = json_search_section(0, "gensensor_warm_time", str))) {
			data[20] = json_get_int(i, str);
			params_set_generic_sensor_warm_time(data[20]);
		}

		if ((i = json_search_section(0, "input_alarm_sensor", str))) {
			data[20] = json_get_int(i, str);
			params_set_input_alarm_sensor(data[20]);
		}

		uint32_t slave;

		for (slave = 0; slave < generic_modbus_table_get_num_devices(); slave++)
		{
			if ((i = json_search_section(0, "slave_id", str))) {
				json_get_str( i, str, string_aux );
				if  ( 0 == generic_485_get_dtl645() )
				{
					json_get_modbus_slave_id( string_aux, slave );
				}
				else if  ( 1 == generic_485_get_dtl645() )
				{
					json_get_modbus_address_645( string_aux, slave );
				}
			}

			if ((i = json_search_section(0, "function", str))) {
				json_get_str( i, str, string_aux );
				json_get_modbus_function(string_aux, slave);
			}

			if ((i = json_search_section(0, "addr", str))) {
				json_get_str( i, str, string_aux );
				json_get_modbus_address( string_aux, slave );
			}

			if ((i = json_search_section(0, "quantity", str))) {
				json_get_str( i, str, string_aux );
				json_get_modbus_quantity( string_aux, slave );
				generic_modbus_table_manager_write_slave(slave);
			}
		}
	}
	if ((i = json_search_section(0, "modbus_read", str))) {
		data[24] = json_get_int(i, str);
		if ( data[24] != generic_485_get_read_time() ) {
			shutdown_reset_modbus_count();
			dlms_client_set_param_change(1);
		}
		generic_485_set_read_time(data[24]);
		if ( 1 == params_config_get_disable_meter())
		{
			if (data[24] != 0)
			{
				param.config.rt = data[24];
			}
		}
	}

	if ((i = json_search_section(0, "modbus_send", str))) {
		data[25] = json_get_int(i, str);
		if ( data[25] != generic_485_get_send_time() ) {
			shutdown_reset_modbus_count();
			dlms_client_set_param_change(1);
		}
		generic_485_set_send_time(data[25]);
		if ( 0 != generic_485_get_send_time() ) {
			params_config_set_sensor_log(1);
		}
	}

	if ((i = json_search_section(0, "overpressure_alarm", str))) {
		float_t value = 0.0;
		value = json_get_float(i, str);
		params_config_set_overpressure_alarm( value );
	}

	if ((i = json_search_section(0, "low_pressure_alarm", str))) {
		float_t value = 0.0;
		value = json_get_float(i, str);
		params_config_set_lowpressure_alarm( value );
	}

	if ((i = json_search_section(0, "reset_time", str))) {
		data[25] = json_get_hour( &str[ token[i+1].start ]);
		params_set_time_reset(data[25]);
		rtc_system_SetResetAlarm();
	}

	if ((i = json_search_section(0, "timeout_connection", str))) {
		data[18] = json_get_int(i, str);
		params_set_timeout_connection( data[18] );
	}

	if ((i = json_search_section(0, "retries_socket", str))) {
		data[19] = json_get_int(i, str);
		params_set_retries_socket( data[19] );
	}

	if ((i = json_search_section(0, "timeout_server", str))) {
		data[20] = json_get_int(i, str);
		params_set_timeout_server( data[20] );
	}

	if ((i = json_search_section(0, "mbus_baudrate", str))) {
		data[21] = json_get_int(i, str);
		params_set_mbus_baudrate( data[21] );
	}

	if ((i = json_search_section(0, "retries_server", str))) {
		data[22] = json_get_int(i, str);
		params_set_retries_server( data[22] );
	}

	if ((i = json_search_section(0, "max_dtlog_size", str))) {
		data[23] = json_get_int(i, str);
		params_set_max_datalogger_size( data[23] );
	}

	if ((i = json_search_section(0, "max_dtlog_msgs", str))) {
		data[24] = json_get_int(i, str);
		params_set_max_datalogger_msgs( data[24] );
	}

	if ((i = json_search_section(0, "no_ack", str))) {
		data[23] = json_get_int(i, str);
		udp_protocol_set_no_ack(data[23]);
	}
//	else
//	{
//		udp_protocol_set_no_ack(0);
//	}

	if ((i = json_search_section(0, "pulse_acc", str))) {
		data[23] = json_get_int(i, str);
		if ( 1 == params_pulses_on() )
		{
			if ( params_get_pulse_acc() != data[23])
			{
				params_set_pulse_acc_backup(data[23]);
			}
			else
			{
				params_set_pulse_acc(data[23]);
				params_set_pulse_acc_backup(data[23]);
			}
			if ( params_get_pulse_acc_backup() == params_get_pulse_acc() )
			{
				if ( 1 == data[23] )
				{
					params_sensor_read_time_set_cycle(0, param.config.rt);
				}
				else
				{
					params_sensor_read_time_set_cycle(0, 30);
				}
			}
		}
		else
		{
			params_set_pulse_acc(data[23]);
			params_set_pulse_acc_backup(data[23]);
		}
	}
//	else
//	{
//		params_set_pulse_acc(0);
//		params_sensor_read_time_set_cycle(0, 30);
//	}

	if ((i = json_search_section(0, "meter_type", str))) {
		data[23] = json_get_int(i, str);
		params_set_uart_meter_type(data[23]);
	}
	
	if ((i = json_search_section(0, "loglive_level", str))) {
		data[23] = json_get_int(i, str);
		params_set_loglive_level(data[23]);
	}

	if ((i = json_search_section(0, "mqtt_dc_on", str))) {
		data[24] = json_get_int(i, str);
		params_set_mqtt_dc_on(data[24]);
	}

	if ((i = json_search_section(0, "type_comm", str))) {
		data[23] = json_get_int(i, str);
		params_set_type_comm(data[23]);
	}

	if ((i = json_search_section(0, "dlms_enable", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_dlms_enable())
		{
			dlms_client_set_param_change(1);
		}
		else
		{
			dlms_client_set_param_change(0);
		}
		memset(dlms_client_get_client(), 0, dlms_client_get_size());
		dlms_client_set_dlms_enable(data[24]);
	}

	if ((i = json_search_section(0, "dlms_read", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_read_time())
		{
//			dlms_client_set_param_change(1);
		}
		dlms_client_set_read_time(data[24]);
	}

	if ((i = json_search_section(0, "dlms_send", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_send_time())
		{
//			dlms_client_set_param_change(1);
			shutdown_reset_modbus_count();
		}
		dlms_client_set_send_time(data[24]);
		if ( 0 != dlms_client_get_send_time() ) {
			params_config_set_sensor_log(1);
		}
	}

	if ((i = json_search_section(0, "mbus_num", str))) {
		data[24] = json_get_int(i, str);
//		mbus_slave_table_manager_read_table();
		if (data[24] != mbus_slave_table_get_slaves_num())
		{
			mbus_slave_table_manager_init(1);
			dlms_client_set_param_change(1);
		}
		else
		{
			dlms_client_set_param_change(0);
		}
//		mbus_slave_table_set_slaves_num(data[24]);
	}

	if ((k = json_search_section(z, "mbus_add", str))) {
		json_get_str( k, str, string_aux );
		if ( 1 == dlms_client_get_param_change() )
		{
			json_get_mbus_add(string_aux, data[24]);
		}
	}

#if 1
	if(( i = json_search_section( 0, "dlms_clients", str )))
	{
		uint32_t clients_num =  0;
		if ( token[i+1].type == JSMN_OBJECT )
		{
			k = i + 1;//k = i + 2;

			if (( i = json_search_section( k, "number", str )))
			{
				data[24]    = json_get_int(i, str);
				clients_num = data[24];
				if (data[24] != dlms_client_table_get_num_devices())
				{
					dlms_client_set_param_change(1);
					params_change = 1;
				}
				else
				{
					dlms_client_set_param_change(0);
					params_change = 0;
				}
				dlms_client_table_set_num_devices(clients_num);
			}
			if (( i = json_search_section( k, "dlms_srv_address", str )))
			{
				json_get_str( i, str, string_aux );
				json_get_dlms_address(string_aux, clients_num);
			}
			if (( i = json_search_section( k, "connection_configs_items", str )))
			{
				json_get_str( i, str, string_aux );
				json_get_dlms_connection_config(string_aux, clients_num);
			}
			if (( i = json_search_section( k, "dlms_profiles_configs_items", str )))
			{
				json_get_str( i, str, string_aux );
				json_get_dlms_profiles_config( string_aux, clients_num );
			}
			if (( i = json_search_section( k, "checksum", str )))
			{
				char checksum[30];
				memset(checksum, 0, sizeof(checksum));
				dlms_client_get_checksum_param(checksum);
				json_get_str( i, str, string_aux );
				if (strncmp(string_aux, checksum, 32))
				{
					dlms_client_set_param_change(1);
					params_change = 1;
					shutdown_set_start_count(0);
				}
				else
				{
					dlms_client_set_param_change(0);
					params_change = 0;
				}
				dlms_client_set_checksum_param(string_aux);
			}
		}
		if(( i = json_search_section( i, "connection_configs", str )))
		{
			if ( token[i + 1].type == JSMN_ARRAY )
			{
				k = i + 2;
				uint32_t num_objects_max = token[i+1].size;
				uint32_t num_objects     = 0;
				for (num_objects = 0; num_objects < num_objects_max; num_objects++ )
				{
					if ( token[k].type == JSMN_OBJECT )
					{
		    			if ((k = json_search_section(z, "dlms_logical_address", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_logical_address())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_logical_address(data[24]);
		    			}

		    			if ((k = json_search_section(z, "dlms_cl_address", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_cl_address())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_cl_address(data[24]);
		    			}

		    			if ((k = json_search_section(z, "dlms_baudrate", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_baudrate())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_baudrate(data[24]);
		    			}
		    			else
		    			{
		    				dlms_client_set_baudrate(19200);
		    			}

		    			if ((k = json_search_section(z, "dlms_short_name_ref", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_short_name_ref())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_short_name_ref(data[24]);
		    			}

		    			if ((k = json_search_section(z, "dlms_pass_level", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_pass_level())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_pass_level(data[24]);
		    			}

		    			if ((k = json_search_section(z, "dlms_authen_pass", str)))
		    			{
		    				char authen[30];
		    				memset(authen, 0, sizeof(authen));
		    				dlms_client_get_authen(authen);
		    				json_get_str( k, str, string_aux );
		    				if (strncmp(string_aux, authen, 30))
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_authen(string_aux);
		    			}

		    			if ((k = json_search_section(z, "dlms_pass_is_hex", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_pass_is_hex())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_pass_is_hex(data[24]);
		    			}

		    			if ((k = json_search_section(z, "dlms_billingday", str)))
		    			{
		    				data[24] = json_get_int(k, str);
		    				if (data[24] != dlms_client_get_billingday())
		    				{
//		    					dlms_client_set_param_change(1);
		    				}
		    				dlms_client_set_billingday(data[24]);
		    			}
		    			k += 2;
		    			z  = k;
		    			uint32_t dev = 0;
		    			if (params_change)
		    			{
		    				for (dev = 0; dev < dlms_client_table_get_num_devices(); dev++)
		    				{
		    					if (num_objects == srv_add[dev][1])
		    					{
		    						dlms_client_set_srv_address(srv_add[dev][0]);
		    						dlms_client_reset_indexes();
		    						dlms_client_table_write_client(dev, dlms_client_get_client());
		    						dlms_client_table_read_client_id( dlms_client_get_client(), dev );
		    					}
		    				}
		    			}
					}
				}
			}
		}
		if(( k = json_search_section( k - 1, "dlms_profiles_configs", str )))
		{
			if ( token[k + 1].type == JSMN_ARRAY )
			{
				uint32_t num_profiles_max = token[k+1].size;
				uint32_t num_profiles     = 0;
				k = k + 2;
				for (num_profiles = 0; num_profiles < num_profiles_max; num_profiles++ )
				{
					if ( token[k].type == JSMN_OBJECT )
					{
//		    			uint32_t count = 0;
		    			uint32_t obis_profile_num = 0, generic_profile_num = 0;
		    			if (params_change)
		    			{
		    				memset(dlms_client_get_client(), 0, dlms_client_get_size());
		    			}
		    			for (profile_num = 0; profile_num < DLMS_PROFILE_NUM; profile_num++)
		    			{
		    				HAL_IWDG_Refresh(&hiwdg);
		    				if ((k = json_search_section(z, "profile_type", str)))
		    				{
		    					data[24]    = json_get_int(k, str);
		    					profile_num = data[24];
		    				}
		    				else
		    				{
		    					break;
		    				}

		    				if (data[24] < DLMS_OBIS_PROFILE_NUM)
		    				{
		    					uint32_t count_obis = 0;
		    					obis_profile_num++;
		    					dlms_client_table_flush_client_obis_profile(dlms_client_get_client_obis_profile());
		    					dlms_client_set_obis_profile_frame_type(data[24]);
		    					if ((k = json_search_section(k, "prof_read", str)))
		    					{
		    						data[24] = json_get_int(k, str);
		    						if (data[24] != dlms_client_get_read_time_obis_n(dlms_client_get_obis_profile_frame_type()))
		    						{
//		    							dlms_client_set_param_change(1);
		    						}
		    						dlms_client_set_read_time_obis_n(dlms_client_get_obis_profile_frame_type(), data[24]);
		    						dlms_client_set_obis_profile_read_time(data[24]);
		    					}

		    					if ((k = json_search_section(k, "prof_send", str)))
		    					{
		    						data[24] = json_get_int(k, str);
		    						dlms_client_set_send_time_obis_n(dlms_client_get_obis_profile_frame_type(), data[24]);
		    						dlms_client_set_obis_profile_send_time(data[24]);
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_1", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(0), 32))
		    						{
		    						}
		    						if (string_aux != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,0);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_2", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(1), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,1);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_3", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(2), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,2);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_4", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(3), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,3);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_5", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(4), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,4);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_6", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(5), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,5);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_7", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(6), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,6);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_8", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(7), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,7);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_9", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(8), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,8);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_10", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(9), 32))
		    						{
		    						}
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,9);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_11", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						count_obis++;
		    						dlms_client_set_obis_n(string_aux,10);
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_12", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,11);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_13", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,12);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_14", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,13);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_15", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,14);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_16", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,15);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_17", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,16);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_18", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,17);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_19", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,18);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}

		    					if ((k = json_search_next_section(k, "dlms_obis_20", str, &t)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (string_aux[0] != '\0')
		    						{
		    							count_obis++;
		    							dlms_client_set_obis_n(string_aux,19);
		    						}
		    					}
		    					else
		    					{
		    						k = t;
		    					}
				    			uint32_t dev_profile = 0;
				    			if (params_change)
				    			{
				    			for (dev_profile = 0; dev_profile < dlms_client_table_get_num_devices(); dev_profile++)
				    			{
				    				if (num_profiles == dlms_profiles_configs[dev_profile])
				    				{
				    					dlms_client_set_obis_profile_num_obis(count_obis);
				    					dlms_client_set_dlms_num_obis_profiles(count_obis);
				    					dlms_client_reset_indexes();
				    					dlms_client_table_write_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
				    					dlms_client_table_read_client_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				    				}
				    			}
				    			}
		    					z = k;
		    				}
		    				else
		    				{
		    					if ((data[24] >= DLMS_OBIS_PROFILE_NUM) && (data[24] < DLMS_PROFILE_NUM) )
		    					{
//		    						dlms_client_table_read_client_generic_profile(0, dlms_client_get_client(),  dlms_client_get_client_generic_profile(), data[24]);
		    						dlms_client_set_generic_profile_frame_type(data[24]);
		    						generic_profile_num++;
		    					}

		    					if ((k = json_search_section(z, "dlms_obis_generic_read", str)))
		    					{
		    						data[24] = json_get_int(k, str);
		    						if (data[24] != dlms_client_get_read_time())
		    						{
		    						}
		    						dlms_client_set_read_time_obis_n(dlms_client_get_generic_profile_frame_type()/*DLMS_OBIS_PROFILE_NUM + count_profile*/, data[24]);
		    						dlms_client_set_dlms_load_profile_read_time(data[24]);
		    					}

		    					if ((k = json_search_section(z, "dlms_obis_generic_send", str)))
		    					{
		    						data[24] = json_get_int(k, str);
		    						dlms_client_set_send_time_obis_n(dlms_client_get_generic_profile_frame_type()/*DLMS_OBIS_PROFILE_NUM + count_profile*/, data[24]);
		    						dlms_client_set_dlms_load_profile_send_time(data[24]);
		    					}

		    					if ((k = json_search_section(z, "dlms_obis_generic", str)))
		    					{
		    						json_get_str( k, str, string_aux );
		    						if (strncmp(string_aux, dlms_client_get_obis_n(0), 32))
		    						{
		    						}
		    						dlms_client_set_generic_obis_n(string_aux,0);
		    						count_profile++;
		    					}
				    			uint32_t dev_generic = 0;
				    			if (params_change)
				    			{
				    			for (dev_generic = 0; dev_generic < dlms_client_table_get_num_devices(); dev_generic++)
				    			{
				    				if (num_profiles == dlms_profiles_configs[dev_generic])
				    				{
				    					dlms_client_set_dlms_num_obis_profiles(obis_profile_num);
				    					dlms_client_set_dlms_num_generic_profiles(generic_profile_num);
				    					dlms_client_table_write_generic_profile(dev_generic, dlms_client_get_client(),
				    							dlms_client_get_client_generic_profile(), dlms_client_get_generic_profile_frame_type());
				    					dlms_client_table_read_client_generic_profile(dev_generic, dlms_client_get_client(),
				    							dlms_client_get_client_generic_profile(),  dlms_client_get_generic_profile_frame_type());
				    	    			dlms_client_table_read_client_id( dlms_client_get_client_backup(), dev_generic );
				    	    			dlms_client_copy_read_send_times(dlms_client_get_client_backup(),dlms_client_get_client());
				    					dlms_client_set_dlms_num_obis_profiles(obis_profile_num);
				    					dlms_client_set_dlms_num_generic_profiles(generic_profile_num);
				    	    			dlms_client_table_write_client(dev_generic, dlms_client_get_client_backup());
				    	    			dlms_client_table_read_client_id( dlms_client_get_client_backup(), dev_generic );
				    				}
				    			}
				    			}
		    					z = k;
		    				}
		    			}
		    			__NOP();
					}
					k += 2;
				}
			}
		}
	}
#endif
	if ((i = json_search_section(0, "keepalive", str))) {
		data[24] = json_get_int(i, str);
		params_set_server_keepalive(0, data[24] );
		params_set_server_keepalive(1, data[24] );
	}

	if ((i = json_search_section(0, "synch_meters", str))) {
		data[24] = json_get_int(i, str);
		params_set_synch_meters(data[24]);
	}

	if ((i = json_search_section(0, "slow_meter", str))) {
		data[24] = json_get_int(i, str);
		params_set_slow_meter(data[24]);
	}

	if ((i = json_search_section(0, "mbus_frame", str))) {
		data[24] = json_get_int(i, str);
		params_set_mbus_frame(data[24]);
	}

	if ((i = json_search_section(0, "input_pulse_as_sensor_num", str))) {
		data[24] = json_get_int(i, str);
		if ( 0 == data[24] )
		{
			if ((0 == params_pulses_on()) && (params_get_i2c_sensor() != 0))
			{
				params_set_i2c_sensor(0);
			}
		}
		params_input_pulse_as_sensor_set_num(data[24]);
	}

	if ((i = json_search_section(0, "i2c_sensor_addr", str))) {
		i2c_sensor_set_num_sensors(params_input_pulse_as_sensor_get_num());
		json_get_str( i, str, string_aux );
		json_get_i2c_address( string_aux );
	}
	return ret;
}

#ifdef MQTT
void json_get_devices_cmd( char *str, uint32_t devices_num )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset(srv_add, -1, sizeof(srv_add));

	if ( *str == '\0' ) {
		return;
	}

	token        = strtok(str,s);
	strncpy(con_dlms_get_device_from_cmd(0), token, strlen(token) );

	i = 1;
	while ( (token != NULL) && ( i < devices_num ) )
	{
		token = strtok( NULL, s );
		if (token != NULL)
		{
			strncpy(con_dlms_get_device_from_cmd(i), token, strlen(token) );
			i++;
		}
	}
}

void json_mqtt_decode( uint8_t type, char *str, uint32_t len )
{
    int32_t i = 0, k = 0, z = 0;//, t = 0;//, j = 0, m = 0;
    int32_t data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    char    string_aux[ 256 ];

    if ( type != PT_FRAME_P_NETWORK_PARAMS ) {
    	jsmn_init( &parser );
    	r = jsmn_parse( &parser, str, len, token, MAX_TOKENS );
    	if ( r < 1 ) {
    		return;
    	}
    }

    memset( data, 0, sizeof( data ));
//    memset(dlms_client_get_dlms_id_request(), 0 , 40*sizeof(char));
//    memset(datalog_get_idRequest(), 0 , 40*sizeof(char));

    if ( ( i = json_search_section( 0, "idRequest", str ) ) ) {
//    	static uint32_t commands = 0;
//    	if (++commands>=2)
//    	{
//    		__NOP();
//    		commands = 0;
//    	}
    	json_get_str(i, str, string_aux);
    	dlms_client_set_dlms_id_request(string_aux);
    	datalog_set_idRequest(string_aux);
        if ( ( i = json_search_section( 0, "devices", str ) ) ) {
        	char devices[32];
        	i++;
        	uint32_t num_meters     = 0;
        	uint32_t num_meters_max = token[i].size;
        	con_dlms_set_curr_device(0xFF);
        	dlms_client_table_set_num_meters_from_cmd(0);
        	for (num_meters = 0; num_meters < num_meters_max; num_meters++)
        	{
        		json_get_str(i++, str, devices);
        		dlms_client_table_check_meter_id( dlms_client_get_client(), (uint8_t *)devices);
        		dlms_client_table_add_meter_id_from_cmd( dlms_client_get_client(), (uint8_t *)devices);
        	}
        	if (dlms_client_table_get_num_meters_from_cmd()!=0)
        	{
        		con_dlms_get_next_device_for_cmd(0);
        	}
//        	if (strstr(params_get_meter_id(), (char *)devices))
//        	{
//        		con_dlms_set_curr_device(0);
//        	}
        }
    }

    switch( type )
    {
        case PT_STATUS:
#if 1
        	if ( ( i = json_search_section( 0, "idstatus", str ) ) ) {
        		data[ 1 ] = json_get_int( i, str );
        		MX_GPIO_Relay_Activation( data[1] );
        		params_set_relay_status( data[1] );
//        		message_queue_write( SEND_PT_STATUS );
        		mqtt_buffer_write(PT_STATUS);
        		mqtt_set_reply_get_topic(1);
        	}
        	if ( ( i = json_search_section( 0, "manual_power", str ) ) ) {
        		data[ 2 ] = json_get_int( i, str );
        	}
        	if (( i = json_search_section( 0, "read_delay", str )))
        	{
//        		vacon_device_type.read_delay = json_get_int( i, str );
        	}
        	if (( i = json_search_section( 0, "send_delay", str )))
        	{
//        		vacon_device_type.send_delay = json_get_int( i, str );
        		mqtt_set_relay(1);
        	}
#endif
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_STATUS Status Decode. Relay:%d.\r\n", (int)Tick_Get( SECONDS ), (int)data[1]);
//    		shutdown_mqttCommandprocess();
//        	shutdown_mqttCommandReconnectionprocess(data[ 1 ]);
//    		mqtt_stop();
            break;
    	case PT_CONFIG_TIME:
    		if ( ( i = json_search_section( 0, "time", str ) ) ) {
    			str[ token[i+1].end - 3 ] = '\0';
    			Tick_system_time_init( json_get_int( i, str ) );
        		rtc_system_SetServerTime( Tick_Get(SECONDS) );
        		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_CONFIG_TIME Time Decode.\r\n", (int)Tick_Get( SECONDS ));
    		}
            break;
    	case PT_FRAME_P_NETWORK_PARAMS:
//    		json_decode(str, len);
//    		shutdown_mqttCommandprocess();
    		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_FRAME_P_NETWORK_PARAMS params_frame_decode.\r\n", (int)Tick_Get( SECONDS ));
    		break;
    	case PT_DEV_REGISTERS:
    	{
    		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_DEV_REGISTERS Registers Decode.\r\n", (int)Tick_Get( SECONDS ));
    		if ((i = json_search_section(0, "modbus_num", str)))
    		{
    			data[20] = json_get_int(i, str);
    			generic_modbus_table_set_num_devices(data[20]);
    		}

    		uint32_t slave;

    		for (slave = 0; slave < generic_modbus_table_get_num_devices(); slave++)
    		{
    			if ((i = json_search_section(0, "slave_id", str)))
    			{
    				json_get_str( i, str, string_aux );
    				if  ( 0 == generic_485_get_dtl645() )
    				{
    					json_get_modbus_slave_id( string_aux, slave );
    				}
    				else if  ( 1 == generic_485_get_dtl645() )
    				{
    					json_get_modbus_address_645( string_aux, slave );
    				}
    			}

    			if ((i = json_search_section(0, "function", str)))
    			{
    				json_get_str( i, str, string_aux );
    				json_get_modbus_function(string_aux, slave);
    			}

    			if ((i = json_search_section(0, "addr", str)))
    			{
    				json_get_str( i, str, string_aux );
    				json_get_modbus_address( string_aux, slave );
    			}

    			if ((i = json_search_section(0, "quantity", str)))
    			{
    				json_get_str( i, str, string_aux );
    				json_get_modbus_quantity( string_aux, slave );
    			}

    			if ((i = json_search_section(0, "value", str)))
    			{
    				json_get_str( i, str, string_aux );
    				json_get_modbus_value( string_aux, slave );
    				generic_modbus_table_manager_write_slave(slave);
    				get_params = 1;
    			}
    		}
    		shutdown_mqttCommandprocess();
    		mqtt_stop();
    	}
    		break;
    	case PT_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_INSTVALUES instvalues_decode.\r\n", (int)Tick_Get( SECONDS ));
        	dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES);
        	if ( ( i = json_search_section( 0, "groups", str ) ) ) {
        		char groups[32];
        		i++;
        		json_get_str( i++, str,  groups      );
        		json_get_str( i++, str,  groups + 6  );
        		json_get_str( i++, str,  groups + 12 );
        		json_get_str( i,   str,  groups + 18 );
        		if ((memmem(groups, 18, "Group1", 6))||(memmem(groups, 18, "group1", 6))) {
        			dlms_client_set_inst_profile_1(1);
        		}
        		if ((memmem(groups, 18, "Group2", 6))||(memmem(groups, 18, "group2", 6))) {
        			dlms_client_set_inst_profile_2(1);
        		}
        		if ((memmem(groups, 18, "Group3", 6))||(memmem(groups, 18, "group3", 6))) {
        			dlms_client_set_inst_profile_3(1);
        		}
        		if ((memmem(groups, 24, "Group4", 6))||(memmem(groups, 24, "group4", 6))) {
        			dlms_client_set_inst_profile_4(1);
        		}
        	}
        	shutdown_mqttCommandprocess();
    		break;
        case PT_COMMAND_ON_DEMAND_ENERGY_REGISTERS:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_ENERGYPROFILE energyprofile_decode.\r\n", (int)Tick_Get( SECONDS ));
        	dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_ENERGY_REGISTERS);
        	if ( ( i = json_search_section( 0, "groups", str ) ) ) {
        		char groups[18];
        		i++;
        		json_get_str( i++, str, groups );
        		json_get_str( i++, str, groups + 6 );
        		json_get_str( i,   str, groups + 12);
        		if ((memmem(groups, 18, "Group1", 6))||(memmem(groups, 18, "group1", 6))) {
        			dlms_client_set_energy_profile_1(1);
        		}
        		if ((memmem(groups, 18, "Group2", 6))||(memmem(groups, 18, "group2", 6))) {
        			dlms_client_set_energy_profile_2(1);
        		}
        		if ((memmem(groups, 18, "Group3", 6))||(memmem(groups, 18, "group3", 6))) {
        			dlms_client_set_energy_profile_3(1);
        		}
        	}
        	shutdown_mqttCommandprocess();
        	break;
        case PT_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS maxdemand_decode.\r\n", (int)Tick_Get( SECONDS ));
        	dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS);
        	shutdown_mqttCommandprocess();
        	break;
        case PT_COMMAND_ON_DEMAND_BILLING_PROFILE:
        	udp_protocol_set_on_billing_profile(1);
        case PT_COMMAND_ON_DEMAND_LOAD_PROFILE_1:
        case PT_COMMAND_ON_DEMAND_LOAD_PROFILE_2:
        case PT_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE:
        case PT_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_ON_DEMAND_GENERIC_PROFILE genericprofile_decode:%d.\r\n", (int)Tick_Get( SECONDS ), (int)(type - 11));
        	if ( ( i = json_search_section( 0, "startDate", str ) ) ) {
        		dlms_client_set_dlms_load_profile_start_time( json_get_int( i, str ) );
        	}
        	if ( ( i = json_search_section( 0, "endDate", str ) ) ) {
        		dlms_client_set_dlms_load_profile_end_time( json_get_int( i, str ) );
        	}
        	dlms_client_set_command(type - 11);
    		shutdown_mqttOnDemandCommandprocess();
    		if ( PT_COMMAND_ON_DEMAND_BILLING_PROFILE == type )
    		{
    			shutdown_set_tamper_param_cmd(TAMPER_MQTT_ON_DEMAND_BILLING_PROFILE);
    			if ( 0 == params_config_get_disable_meter() )
    			{
    				mbus_task_set_billing_frame(1);
    				mbus_set_start_comm(1);
    				shutdown_set_meter_send(1);
    			}
    		}
        	break;
        case PT_COMMAND_GET_SWITCH_STATUS:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SWITCH_STATUS switch_status.\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_SWITCH_STATUS);
    		shutdown_mqttCommandReconnectionprocess(0);
        	break;
        case PT_COMMAND_RECONNECTION:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_RECONNECTION reconnection_decode.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "type", str ) ) )
        	{
        		json_get_str(i, str, string_aux);
        		if ( 0 == strncmp(string_aux, "DISCONNECTION", 13))
        		{
        			dlms_client_set_command(DLMS_HES_COMMAND_CUTTOFF_RECONNECTION);
        			shutdown_mqttCommandReconnectionprocess(1);
        			shutdown_set_tamper_param_cmd(TAMPER_MQTT_SWITCH_STATUS);
        		}
        		else if ( 0 == strncmp(string_aux, "CONNECTION", 10))
        		{
        			dlms_client_set_command(DLMS_HES_COMMAND_CUTTOFF_RECONNECTION);
        			shutdown_mqttCommandReconnectionprocess(2);
        			shutdown_set_tamper_param_cmd(TAMPER_MQTT_SWITCH_STATUS);
        		}
        		else
        		{
        			shutdown_mqttCommandReconnectionprocess(0);
        		}
        	}
        	if ( ( i = json_search_section( 0, "mode", str ) ) ) {
        		json_get_str(i, str, string_aux);
        		if ( 0 == strncmp(string_aux, "Mode0", 5)) {
        			dlms_client_on_demand_set_control_mode(0);
        		}
        		else if ( 0 == strncmp(string_aux, "Mode1", 5)) {
        			dlms_client_on_demand_set_control_mode(1);
        		}
        		else if ( 0 == strncmp(string_aux, "Mode2", 5)) {
        			dlms_client_on_demand_set_control_mode(2);
        		}
        		else if ( 0 == strncmp(string_aux, "Mode3", 5)) {
        			dlms_client_on_demand_set_control_mode(3);
        		}
        		else if ( 0 == strncmp(string_aux, "Mode4", 5)) {
        			dlms_client_on_demand_set_control_mode(4);
        		}
        		else if ( 0 == strncmp(string_aux, "Mode5", 5)) {
        			dlms_client_on_demand_set_control_mode(5);
        		}
        		else if ( 0 == strncmp(string_aux, "Mode6", 5)) {
        			dlms_client_on_demand_set_control_mode(6);
        		}
        	}
        	break;
        case PT_COMMAND_METER_SYNCHRONIZE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_METER_SYNCHRONIZATION meter_synchronization.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "date", str ) ) )
        	{
//        		dlms_client_set_synch_time(json_get_int( i, str));
//        		dlms_client_set_command(DLMS_HES_COMMAND_METER_SYNCHRONIZE);
//        		shutdown_mqttCommandprocess();
//        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_METER_SYNCH);

        		time_t time_date    = json_get_int(i, str);// + 4*3600;
//        		uint64_t date_param = dlms_client_on_demand_synch(time_date);
        		dlms_client_set_synch_time(json_get_int( i, str));
        		dlms_client_set_command(DLMS_HES_COMMAND_METER_SYNCHRONIZE);
        		mbus_task_set_time_data(time_date);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_METER_SYNCH);
    			if ( 0 == params_config_get_disable_meter() )
    			{
    				mbus_set_start_comm(1);
    			}
        	}
        	break;
        case PT_COMMAND_READ_TIME_CLOCK:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_READ_TIME_CLOCK. clocktime\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_READ_TIME_CLOCK);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_READ_REAL_TIME_CLOCK);
        	break;
        case PT_COMMAND_MAXIMUM_DEMAND_RESET:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_MDI. mbdieobreset\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_MAXIMUM_DEMAND_RESET);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_MDIEOB_RESET);
        	break;
        case PT_COMMAND_SET_LOAD_LIMITATION:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_LOAD_LIMITATION. loadlimit\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "limitThreshold", str ) ) )
        	{
        		dlms_client_on_demand_set_threshold(json_get_int( i, str));
        	}
        	if ( ( i = json_search_section( 0, "threshDuration", str ) ) )
        	{
        		dlms_client_on_demand_set_threshDuration(json_get_int( i, str));
        	}
        	dlms_client_set_command(DLMS_HES_COMMAND_SET_LOAD_LIMITATION);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_LOAD_LIMITATION);
        	break;
        case PT_COMMAND_GET_LOAD_LIMIT_THRESHOLD:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_LOAD_LIMITATION. getloadlimit\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_LOAD_LIMIT_THRESHOLD);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_LOAD_LIMIT_THRESHOLD);
        	break;
        case PT_COMMAND_SET_BILLING_DATE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_BILLING_DATE. setbillingdate\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "billingPeriod", str ) ) )
        	{
        		json_get_str( i, str, string_aux);
        		dlms_client_on_demand_set_billing_date(string_aux);
        	}
        	dlms_client_set_command(DLMS_HES_COMMAND_SET_BILLING_DATE);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_BILLING_DATE);
		break;
        case PT_COMMAND_GET_BILLING_DATE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_BILLING_DATE. getbillingdate\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_BILLING_DATE);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_BILLING_DATE);
        	break;
        case PT_COMMAND_CLEAR_ALARMS:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_CLEAR_ALARMS. clearalarms\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_CLEAR_ALARMS);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_CLEAR_ALARMS);
        	break;
        case PT_COMMAND_GET_VOLT_RANGE_LOW:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_VOLTAGE_LOW. getvoltagelow\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_VOLT_RANGE_LOW);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_VOLTAGE_LOW);
        	break;
        case PT_COMMAND_GET_VOLT_RANGE_UP:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_VOLTAGE_LOW. getvoltageup\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_VOLT_RANGE_UP);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_VOLTAGE_UP);
        	break;
        case PT_COMMAND_SET_VOLT_RANGE_LOW:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_VOLT_RANGE_LOW setvoltrangeLow.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "Threshold", str ) ) )
        	{
        		dlms_client_on_demand_set_voltage_range(json_get_int( i, str));
        		dlms_client_set_command(DLMS_HES_COMMAND_SET_VOLT_RANGE_LOW);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_VOLTAGE_LOW);
        	}
        	break;
        case PT_COMMAND_SET_VOLT_RANGE_UP:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_VOLT_RANGE_UP setvoltrangeUp.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "Threshold", str ) ) )
        	{
        		dlms_client_on_demand_set_voltage_range(json_get_int( i, str));
        		dlms_client_set_command(DLMS_HES_COMMAND_SET_VOLT_RANGE_UP);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_VOLTAGE_UP);
        	}
        	break;
        case PT_COMMAND_GET_CURRENT_RANGE_LOW:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_CURRENT_LOW. getcurrtagelow\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_CURRENT_RANGE_LOW);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_CURRENT_LOW);
        	break;
        case PT_COMMAND_GET_CURRENT_RANGE_UP:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_CURR_LOW. getcurrtageup\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_CURRENT_RANGE_UP);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_CURRENT_UP);
        	break;
        case PT_COMMAND_SET_CURRENT_RANGE_LOW:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_CURR_RANGE_LOW setcurrtrangeLow.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "Threshold", str ) ) )
        	{
        		dlms_client_on_demand_set_current_range(json_get_int( i, str));
        		dlms_client_set_command(DLMS_HES_COMMAND_SET_CURRENT_RANGE_LOW);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_CURRENT_LOW);
        	}
        	break;
        case PT_COMMAND_SET_CURRENT_RANGE_UP:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_CURR_RANGE_UP setcurrtrangeUp.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "Threshold", str ) ) )
        	{
        		dlms_client_on_demand_set_current_range(json_get_int( i, str));
        		dlms_client_set_command(DLMS_HES_COMMAND_SET_CURRENT_RANGE_UP);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_CURRENT_UP);
        	}
        	break;
        case PT_COMMAND_SET_DEMAND_INTEGRATION_PERIOD:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_DEMAND_INTEGRATION_PERIOD setdemandintegrationperiod.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "demandIntPeriod", str ) ) )
        	{
        		dlms_client_on_demand_set_demand_integration_period(json_get_int( i, str));
        		dlms_client_set_command(DLMS_HES_COMMAND_SET_DEMAND_INTEGRATION_PERIOD);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_DEMAND_INTEGRATION_PERIOD);
        	}
        	break;
        case PT_COMMAND_GET_DEMAND_INTEGRATION_PERIOD:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_DEMAND_INTEGRATION_PERIOD. getdemandInt\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_DEMAND_INTEGRATION_PERIOD);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_DEMAND_INTEGRATION_PERIOD);
        	break;
        case PT_COMMAND_SET_PAYMENT_MODE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_PAYMENT_MODE setPaymentMode.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "paymentMode", str ) ) )
        	{
        		json_get_str(i, str, string_aux);
        		if ( 0 == strncmp(string_aux, "Prepaid", 7)) {
        			dlms_client_on_demand_set_payment_mode(1);
        		}
        		else if ( 0 == strncmp(string_aux, "Postpaid", 8)) {
        			dlms_client_on_demand_set_payment_mode(0);
        		}

        		dlms_client_set_command(DLMS_HES_COMMAND_SET_PAYMENT_MODE);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_PAYMENT_MODE);
        	}
        	break;
        case PT_COMMAND_GET_PAYMENT_MODE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_PAYMENT_MODE. paymentMode\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_PAYMENT_MODE);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_PAYMENT_MODE);
        	break;
        case PT_COMMAND_SET_METERING_MODE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_METERING_MODE setMeteringmode.\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "meteringMode", str ) ) )
        	{
        		json_get_str(i, str, string_aux);
        		if ( 0 == strncmp(string_aux, "Bidirectional", 13)) {
        			dlms_client_on_demand_set_metering_mode(0);
        		}
        		else if ( 0 == strncmp(string_aux, "Unidirectional", 14)) {
        			dlms_client_on_demand_set_metering_mode(1);
        		}

        		dlms_client_set_command(DLMS_HES_COMMAND_SET_METERING_MODE);
        		shutdown_mqttCommandprocess();
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_METERING_MODE);
        	}
        	break;
        case PT_COMMAND_GET_METERING_MODE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_METERING_MODE. getmeteringmode\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_METERING_MODE);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_METERING_MODE);
        	break;
        case PT_COMMAND_GET_METER_STATUS:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_METER_STATUS. getmeterstatus\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_METER_STATUS);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_METER_STATUS);
        	break;
        case PT_COMMAND_READ_METER_PLATE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_READ_METR_PLATE. nameplate\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_READ_METER_NAMEPLATE);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_READ_METER_NAMEPLATE);
        	break;
        case PT_COMMAND_GET_CURRENT_ACTIVE_TARIFF:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GET_CURRENT_ACTIVE_TARIFF. activetariff\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_ACTIVE_TARIFF);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_CURR_ACTIVE_TARIFF);
        	break;
        case PT_COMMAND_METER_PING:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_METER_PING. meterping\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_METER_PING);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_METER_PING);
        	break;
        case PT_COMMAND_WATERPROFILE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_WATERPROFILE. waterprofile\r\n", (int)Tick_Get( SECONDS ));
//    		dlms_client_set_command(DLMS_HES_COMMAND_READ_METER_NAMEPLATE);
//    		shutdown_mqttCommandprocess();
        	shutdown_mqttWaterCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_WATERPROFILE);
        	break;
        case PT_COMMAND_SET_TARIFF_AGREEMENT:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_TARIFF_AGREEMENT. toutariff\r\n", (int)Tick_Get( SECONDS ));
        	if(( k = json_search_section( 0, "calendarName", str )))
        	{
				json_get_str(k, str,string_aux);
				if (*string_aux != '\0')
				{
					dlms_client_on_demand_set_calendar_name(string_aux);
				}
        	}
	        if(( k = json_search_section( 0, "activateCalendarTime", str )))
	        {
	        	json_get_str(k, str,string_aux);
	        	if (*string_aux != '\0')
	        	{
	        		dlms_client_on_demand_set_activate_calendar_time(string_aux);
	        	}
	        }
        	if(( k = json_search_section( 0, "seasonProfile", str )))
    		{
    			if ( token[k + 1].type == JSMN_ARRAY )
    			{
    				uint32_t num_season_profiles_max = token[k+1].size;
    				uint32_t num_season_profiles     = 0;
    				k = k + 2;
    				for (num_season_profiles = 0; num_season_profiles < num_season_profiles_max; num_season_profiles++ )
    				{
    					if ( token[k].type == JSMN_OBJECT )
    					{
	    					if ((k = json_search_section(k, "seasonProfileId", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_season_profile_id(num_season_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "seasonStart", str)))
	    					{
	    						json_get_str(k, str,string_aux);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_season_start(num_season_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "weekProfileId", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_season_profile_week_profile_id(num_season_profiles, string_aux);
	    						}
	    					}
	    					k = k + 2;
    					}
    				}
    			}
    		}
        	if(( k = json_search_section( 0, "weekProfile", str )))
    		{
    			if ( token[k + 1].type == JSMN_ARRAY )
    			{
    				uint32_t num_week_profiles_max = token[k+1].size;
    				uint32_t num_week_profiles     = 0;
    				k = k + 2;
    				for (num_week_profiles = 0; num_week_profiles < num_week_profiles_max; num_week_profiles++ )
    				{
    					if ( token[k].type == JSMN_OBJECT )
    					{
	    					if ((k = json_search_section(k, "weekProfileId", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_id(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "monday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_monday(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "tuesday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_tuesday(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "wednesday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_wednesday(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "thursday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_thursday(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "friday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_friday(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "saturday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_saturday(num_week_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "sunday", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_week_profile_sunday(num_week_profiles, string_aux);
	    						}
	    					}
    						k = k + 2;
    					}
    				}
    			}
    		}
        	if(( k = json_search_section( 0, "dayProfile", str )))
    		{
    			if ( token[k + 1].type == JSMN_ARRAY )
    			{
    				uint32_t num_day_profiles_max = token[k+1].size;
    				uint32_t num_day_profiles     = 0;
    				k = k + 2;
    				for (num_day_profiles = 0; num_day_profiles < num_day_profiles_max; num_day_profiles++ )
    				{
    					if ( token[k].type == JSMN_OBJECT )
    					{
	    					if ((k = json_search_section(k, "dayId", str)))
	    					{
	    						data[0] = json_get_int(k, str);
	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_day_profile_dayId(num_day_profiles, string_aux);
	    						}
	    					}
	    					if ((k = json_search_section(k, "qty", str)))
	    					{
	    						data[0] = json_get_int(k, str);
//	    						itoa(data[0],string_aux,10);
	    						if (*string_aux != '\0')
	    						{
	    							dlms_client_on_demand_set_day_profile_qty(num_day_profiles, data[0]);
	    						}
	    					}
	    		        	if(( k = json_search_section( k, "dayItem", str )))
	    		    		{
	    		    			if ( token[k + 1].type == JSMN_ARRAY )
	    		    			{
	    		    				uint32_t num_day_item_max = token[k+1].size;
	    		    				uint32_t num_day_items    = 0;
	    		    				k = k + 2;
	    		    				for (num_day_items = 0; num_day_items < num_day_item_max; num_day_items++ )
	    		    				{
	    		    					if ( token[k].type == JSMN_OBJECT )
	    		    					{
	    			    					if ((k = json_search_section(k, "startTime", str)))
	    			    					{
	    			    						json_get_str(k, str,string_aux);
	    			    						if (*string_aux != '\0')
	    			    						{
	    			    							dlms_client_on_demand_set_day_profile_dayItem_startTime(num_day_profiles, num_day_items, string_aux);
	    			    						}
	    			    					}
	    			    					if ((k = json_search_section(k, "scriptLogicalName", str)))
	    			    					{
	    			    						json_get_str(k, str,string_aux);
	    			    						if (*string_aux != '\0')
	    			    						{
	    			    							dlms_client_on_demand_set_day_profile_dayItem_scriptLogicalName(num_day_profiles, num_day_items, string_aux);
	    			    						}
	    			    					}
	    			    					if ((k = json_search_section(k, "scriptSelector", str)))
	    			    					{
//	    			    						json_get_str(k, str,string_aux);
	    			    						data[0] = json_get_int(k, str);
	    			    						if (*string_aux != '\0')
	    			    						{
	    			    							dlms_client_on_demand_set_day_profile_dayItem_scriptSelector(num_day_profiles, num_day_items, data[0]);
	    			    						}
	    			    					}
	    			    					k = k + 2;
	    		    					}
	    		    				}
	    		    			}
	    		    		}
//	    					k = k + 2;
    					}
    				}
    			}
    		}
    		dlms_client_set_command(DLMS_HES_COMMAND_METER_SET_TARIFF_AGREEMENT);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_CURR_ACTIVE_TARIFF);
        	break;
        case PT_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_READ_LOAD_PROFILE_CAPTURE. getprofileinterval\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD);
        	if ( ( i = json_search_section( 0, "profile", str ) ) ) {
        		char profiles[32];
        		i++;
        		json_get_str( i++, str,  profiles );
//        		json_get_str( i++, str,  profiles + 12 );
//        		json_get_str( i++, str,  profiles + 19 );
//        		json_get_str( i,   str,  profiles + 22 );
        		dlms_client_on_demand_set_profile_capture_load1(0);
        		dlms_client_on_demand_set_profile_capture_load2(0);
        		dlms_client_on_demand_set_profile_capture_powerquality(0);
        		dlms_client_on_demand_set_profile_capture_instrumentation(0);
        		if ((memmem(profiles, 18, "LoadProfile1", 12))/*||(memmem(groups, 18, "group1", 6))*/) {
        			dlms_client_on_demand_set_profile_capture_load1(1);
        		}
        		if ((memmem(profiles, 12, "LoadProfile2", 12))/*||(memmem(groups, 18, "group2", 6))*/) {
        			dlms_client_on_demand_set_profile_capture_load2(1);
        		}
        		if ((memmem(profiles, 19, "PowerQualityProfile", 19))/*||(memmem(groups, 18, "group3", 6))*/) {
        			dlms_client_on_demand_set_profile_capture_powerquality(1);
        		}
        		if ((memmem(profiles, 22, "InstrumentationProfile", 22))/*||(memmem(groups, 18, "group4", 6))*/) {
        			dlms_client_on_demand_set_profile_capture_instrumentation(1);
        		}
        	}
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_READ_LOAD_PROFILE_CAPTURE_PERIOD);
        	break;
        case PT_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_LOAD_PROFILE_CAPTURE. setprofileinterval\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD);
    		if ( (i = json_search_section( 0, "schedulerConfig", str )))
    		{
    			if ( token[i + 1].type == JSMN_ARRAY )
    			{
    				k = i + 2;
    				uint32_t num_objects_max = token[i+1].size;
    				uint32_t num_objects     = 0;
    				for (num_objects = 0; num_objects < num_objects_max; num_objects++ )
    				{
    					if ( token[k].type == JSMN_OBJECT )
    					{
    						if ((k = json_search_section(z, "loadprofile1_Period", str)))
    						{
    							data[0] = json_get_int(k, str);
    							dlms_client_on_demand_set_profile_capture_load1(data[0]);
    						}
    						else
							{
    							dlms_client_on_demand_set_profile_capture_load1(0);
							}
    						if ((k = json_search_section(z, "loadprofile2_Period", str)))
    						{
    							data[1] = json_get_int(k, str);
    							dlms_client_on_demand_set_profile_capture_load2(data[1]);
    						}
    						else
							{
    							dlms_client_on_demand_set_profile_capture_load2(0);
							}
    						if ((k = json_search_section(z, "powerqualityprofile_Period", str)))
    						{
    							data[2] = json_get_int(k, str);
    							dlms_client_on_demand_set_profile_capture_powerquality(data[2]);
    						}
    						else
							{
    							dlms_client_on_demand_set_profile_capture_powerquality(0);
							}
    						if ((k = json_search_section(z, "instrumentationprofile_Period", str)))
    						{
    							data[3] = json_get_int(k, str);
    							dlms_client_on_demand_set_profile_capture_instrumentation(data[3]);
    						}
    						else
							{
    							dlms_client_on_demand_set_profile_capture_instrumentation(0);
							}
    					}
    				}
    			}
    		}
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_SET_LOAD_PROFILE_CAPTURE_PERIOD);
        	break;
        case PT_COMMAND_GET_GW_INTERVAL:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GW_INTERVAL. getgwinterval\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_GW_INTERVAL);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_GW_GET_INTERVAL);
        	break;
        case PT_COMMAND_GW_PING:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GW_PING. ping\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GW_PING);
//    		shutdown_mqttCommandprocess();
    		shutdown_mqtt_restoreContext();
    		shutdown_set_start_count(1);
    		shutdown_setInitTelitModule(1);
    		mqtt_stop();
    		message_queue_write( SEND_NETWORK_PARAMS_CMD );
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_GW_PING);
        	break;
        case PT_COMMAND_GET_GW_NAMEPLATE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GW_NAMEPLATE. gwnameplate\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_GW_NAMEPLATE);
//    		shutdown_mqttCommandprocess();
    		shutdown_mqtt_restoreContext();
    		shutdown_set_start_count(1);
    		shutdown_setInitTelitModule(1);
    		mqtt_stop();
    		message_queue_write( SEND_NETWORK_PARAMS_CMD );
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_CMD_GET_GW_NAMEPLATE);
        	break;
        case PT_COMMAND_GET_GW_TIME:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GW_TIME. getgwclocktime\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_GET_GW_TIME);
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_GW_GET_TIME);
        	break;
        case PT_COMMAND_SET_GW_INTERVAL:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_SET_GW_INTERVAL. setgwinterval\r\n", (int)Tick_Get( SECONDS ));
    		dlms_client_set_command(DLMS_HES_COMMAND_SET_GW_INTERVAL);
    		if ( (i = json_search_section( 0, "schedulerConfig", str )))
    		{
    			if ( token[i + 1].type == JSMN_ARRAY )
    			{
    				k = i + 2;
    				uint32_t num_objects_max = token[i+1].size;
    				uint32_t num_objects     = 0;
    				uint32_t dev_generic     = 0;
       				uint32_t dev_profile     = 0;
        			uint32_t num_groups      = 0;
    				for (num_objects = 0; num_objects < num_objects_max; num_objects++ )
    				{
    					if ( token[k].type == JSMN_OBJECT )
    					{
							if ((k = json_search_section(z, "InstantaneousValues", str)))
							{
								for (dev_profile = 0; dev_profile < dlms_client_table_get_num_devices(); dev_profile++)
								{
									data[0] = json_get_int(k, str);
									for (num_groups = 0; num_groups < 4; num_groups++)
									{
										dlms_client_table_read_client_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(), num_groups);
										if (num_groups == dlms_client_get_obis_profile_frame_type())
										{
											dlms_client_set_read_time_obis_n(num_groups, data[0]);
											dlms_client_set_obis_profile_read_time(data[0]);
											dlms_client_set_send_time_obis_n(num_groups, data[0]);
											dlms_client_set_obis_profile_send_time(data[0]);
											dlms_client_table_write_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(),  num_groups);
											dlms_client_table_read_client_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(), num_groups);
										}
									}
								}
							}
							if ((k = json_search_section(z, "EnergyProfile", str)))
							{
								for (dev_profile = 0; dev_profile < dlms_client_table_get_num_devices(); dev_profile++)
								{
									data[0] = json_get_int(k, str);
									for (num_groups = 4; num_groups < 7; num_groups++)
									{
										dlms_client_table_read_client_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(), num_groups);
										if (num_groups == dlms_client_get_obis_profile_frame_type())
										{
											dlms_client_set_read_time_obis_n(num_groups, data[0]);
											dlms_client_set_obis_profile_read_time(data[0]);
											dlms_client_set_send_time_obis_n(num_groups, data[0]);
											dlms_client_set_obis_profile_send_time(data[0]);
											dlms_client_table_write_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(),  num_groups);
											dlms_client_table_read_client_obis_profile(dev_profile, dlms_client_get_client(), dlms_client_get_client_obis_profile(), num_groups);
										}
									}
								}
							}
    						if ((k = json_search_section(z, "loadProfile1", str)))
    						{
    							for (dev_generic = 0; dev_generic < dlms_client_table_get_num_devices(); dev_generic++)
    							{
    								dlms_client_table_read_client_generic_profile(dev_generic, dlms_client_get_client(), dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM);
    								if (DLMS_OBIS_PROFILE_NUM == dlms_client_get_generic_profile_frame_type())
    								{
    									data[0] = json_get_int(k, str);
    									dlms_client_set_dlms_load_profile_read_time(data[0]);
    									dlms_client_set_dlms_load_profile_send_time(data[0]);
    									dlms_client_table_write_generic_profile(dev_generic, dlms_client_get_client(),
    											dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM);
    								}
    							}
    						}
    						if ((k = json_search_section(z, "loadProfile2", str)))
    						{
    							for (dev_generic = 0; dev_generic < dlms_client_table_get_num_devices(); dev_generic++)
    							{
    								dlms_client_table_read_client_generic_profile(dev_generic, dlms_client_get_client(), dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + 1);
    								if ((DLMS_OBIS_PROFILE_NUM + 1) == dlms_client_get_generic_profile_frame_type())
    								{
    									data[1] = json_get_int(k, str);
    									dlms_client_set_dlms_load_profile_read_time(data[1]);
    									dlms_client_set_dlms_load_profile_send_time(data[1]);
    									dlms_client_table_write_generic_profile(dev_generic, dlms_client_get_client(),
    										dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + 1);
    								}
    							}
    						}
    						if ((k = json_search_section(z, "PowerQualityProfile", str)))
    						{
    							for (dev_generic = 0; dev_generic < dlms_client_table_get_num_devices(); dev_generic++)
    							{
    								dlms_client_table_read_client_generic_profile(dev_generic, dlms_client_get_client(), dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + 2);
    								if ((DLMS_OBIS_PROFILE_NUM + 2) == dlms_client_get_generic_profile_frame_type())
    								{
    									data[2] = json_get_int(k, str);
    									dlms_client_set_dlms_load_profile_read_time(data[2]);
    									dlms_client_set_dlms_load_profile_send_time(data[2]);
    									dlms_client_table_write_generic_profile(dev_generic, dlms_client_get_client(),
    										dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + 2);
    								}
    							}
    						}

    						if ((k = json_search_section(z, "InstrumentationProfile", str)))
    						{
    							for (dev_generic = 0; dev_generic < dlms_client_table_get_num_devices(); dev_generic++)
    							{
    								dlms_client_table_read_client_generic_profile(dev_generic, dlms_client_get_client(),
    										dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + 3);
    								if ((DLMS_OBIS_PROFILE_NUM + 3) == dlms_client_get_generic_profile_frame_type())
    								{
    									data[3] = json_get_int(k, str);
    									dlms_client_set_dlms_load_profile_read_time(data[3]);
    									dlms_client_set_dlms_load_profile_send_time(data[3]);
    									dlms_client_table_write_generic_profile(dev_generic, dlms_client_get_client(),
    										dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + 3);
    								}
    							}
    						}
    						if ((k = json_search_section(z, "Heartbeat", str)))
    						{
    							data[3] = json_get_int(k, str);
    							if (data[3] != param.config.st)
    							{
    								param.config.send_count = 0;
    								shutdown_reset_meter_count();
    							}
    							param.config.st = data[3];
    							if ( data[3] < param.config.rt)
    							{
    								param.config.rt = param.config.st;
    							}
    						}
    						if ((k = json_search_section(z, "eMonthlyBillingProfile", str)))
    						{
    							data[3] = json_get_int(k, str);
    							if (data[3] != dlms_client_get_billingday())
    							{
//		    					dlms_client_set_param_change(1);
    							}
    							dlms_client_set_billingday(data[3]);
    						}
    					}
    				}
    			}
    		}
    		shutdown_mqttCommandprocess();
    		shutdown_set_tamper_param_cmd(TAMPER_MQTT_GW_SET_INTERVAL);
        	break;
        case PT_COMMAND_SET_GW_SYNCH:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GW_SYNCH. gwsynchronization\r\n", (int)Tick_Get( SECONDS ));
        	if ( ( i = json_search_section( 0, "date", str ) ) )
        	{
        		dlms_client_set_synch_time(json_get_int( i, str));
				Tick_system_time_init(json_get_int( i, str));
				time_t secs  = Tick_Get(SECONDS);
				rtc_system_SetServerTime(secs);
				uint32_t i;
				for ( i = 0; i < dlms_client_table_get_num_devices(); i++ )
				{
					dlms_client_table_add_meter_id_from_cmd(dlms_client_get_client(), (uint8_t *)dlms_client_get_meter_id(i));
				}
				con_dlms_set_curr_device(0);
				dlms_client_set_command(DLMS_HES_COMMAND_SET_GW_SYNCH);
				shutdown_mqttCommandprocess();
        		mbus_task_set_time_data(secs + 4*3600);
    			if ( 0 == params_config_get_disable_meter() )
    			{
    				mbus_set_start_comm(1);
    			}
				shutdown_set_tamper_param_cmd(TAMPER_MQTT_GW_SET_TIME);
        	}
        	break;
        case PT_COMMAND_GW_FIRMWARE_UPDATE:
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_COMMAND_GW_FIRMWARE_UPDATE. gwfirmwareupdate\r\n", (int)Tick_Get( SECONDS ));
        	if ((i = json_search_section(0, "firmware_version", str)))
        	{
        		json_get_str(i, str, string_aux);
        		rest_get_fw_version(string_aux);
        		dlms_client_set_command(DLMS_HES_COMMAND_GW_FIRMWARE_UPDATE);
//        		udp_protocol_set_waiting_answer(0);
//        		__processFWUpgradeResponse();
        		shutdown_mqtt_restoreContext();
        		shutdown_set_start_count(1);
        		shutdown_setInitTelitModule(1);
        		mqtt_stop();
        		message_queue_write( SEND_NETWORK_PARAMS_CMD );
        		shutdown_set_tamper_param_cmd(TAMPER_MQTT_GW_SET_FW_UPDATE);
        	}
        	break;
        default:
            break;
    }
}

uint32_t json_mqtt_encode( uint8_t type, char *str )
{
    uint32_t msg_size = 0;
	switch( type )
    {
	case PT_CONNECTION:
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_CONNECTION encode.\r\n", (int)Tick_Get( SECONDS ));
		sprintf( str, "{\"status\":\"online\",\"APN\":\"%s\",\"ICCID\":\"%s\",\"fw\":\"%02d.%02d\"}", param.APN[ ME910_APN_get_index() ].name, ME910_ICCID(), (int)param.version.major, (int)param.version.minor );
		break;
	case PT_STATUS:
	{
		char date[32];
		json_set_date2str(Tick_Get(SECONDS), date);
		sprintf( str, "{\"date\":%s,\"idmode\":\"0\",\"idstatus\":\"%d\",\"manual_power\":\"100\",\"firmware_version\":\"%02d.%02d\",\"read_delay\":\"%d\",\"send_delay\":\"%d\"}", date, (int)params_get_relay_status(), (int)param.version.major, (int)param.version.minor, (int)0, (int)0);
	}
		break;
	case PT_REGISTER:
		break;
	case PT_CONFIG_TIME:
		break;
	case PT_FRAME_P_NETWORK_PARAMS:
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_FRAME_P_NETWORK_PARAMS encode.\r\n", (int)Tick_Get( SECONDS ));
		mqtt_frames_network_params_pubmsg(str);
		mqtt_frames_set_wait_server_resp(NETPARAMS_SERVER_RESP);
		break;
	case PT_FRAME_F:
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_FRAME_F encode.\r\n", (int)Tick_Get( SECONDS ));
		msg_size =  sizeof( str );
		memset( str, 0, msg_size );
		msg_size = datalog_buffer_read( str, 0 );
		if ( 0 == msg_size ) {
			DataLogger_ResetPointers();
		} else {
			mqtt_frames_set_wait_server_resp(MESSAGE_SERVER_RESP);
		}
		break;
	case PT_FRAME_MF:
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_FRAME_MF encode.\r\n", (int)Tick_Get( SECONDS ));
		msg_size = mqtt_frames_modbus_sensor_raw_pubmsg(str);
		mqtt_frames_set_wait_server_resp(MODBUS_SENSOR_SERVER_RESP);
		break;
	case PT_FRAME_S:
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_FRAME_S encode.\r\n", (int)Tick_Get( SECONDS ));
		msg_size = mqtt_frames_sensor_pubmsg(str);
		mqtt_frames_set_wait_server_resp(SENSOR_SERVER_RESP);
		break;
	case PT_FRAME_S_PLUS:
	{
		uint32_t sensors = 0x16;
		LOGLIVE(LEVEL_1, "LOGLIVE> %d JSON> PT_FRAME_S_PLUS encode,\r\n", (int)Tick_Get( SECONDS ));
		if ( 0 == AD_GetADOn() ) {
			sensors = 0x06;
		}
		msg_size = mqtt_frames_sensor_plus_pubmsg(str, 2, sensors );
		mqtt_frames_set_wait_server_resp(MODBUS_SENSOR_SERVER_RESP);
	}
		break;
	case PT_TEST_CONNECTION:
		sprintf( str, "online" );
		break;
	case PT_TEST_RESULT:
		sprintf( str, "{\"result\":\"1\"}" );
		break;
	default:
		str[0] = '\0';
		break;
	}

    return strlen( str );
}
#endif
////////////////////////////////////////////////////////////////////////////////

int32_t json_search_section( int32_t tk_i, char *name, char *str )// uint32_t end ) // search for an array
{
	int32_t i;

	for( i = tk_i + 1; i < r; i++ )
	{
		if( ( token[i].type == JSMN_STRING ) &&
			( (int) strlen(name) == token[i].end - token[i].start ) &&
			! strncmp( name, str + token[i].start, token[i].end - token[i].start ))
		{
			return i;
		}
	}

	return 0;
}

int32_t json_search_next_section( int32_t tk_i, char *name, char *str, int32_t *t )// uint32_t end ) // search for an array
{
	if( ( token[tk_i + 2].type == JSMN_STRING ) &&
			( (int) strlen(name) == token[tk_i + 2].end - token[tk_i + 2].start ) &&
			! strncmp( name, str + token[tk_i + 2].start, token[tk_i + 2].end - token[tk_i + 2].start ))
	{
		*t = tk_i + 2;
		return tk_i + 2;
	}
	*t = tk_i;
	return 0;
}

int32_t json_get_int( int32_t tk_i, char *str )
{
    int32_t value = 0;

    tk_i++;
    if( token[tk_i].type == JSMN_PRIMITIVE ) {
        if( isdigit( (unsigned char)str[ token[tk_i].start ]) ||
            ( str[ token[tk_i].start ] == '-' && isdigit( (unsigned char)str[ token[tk_i].start + 1 ] ))) {
            value = atoi( &str[ token[tk_i].start ]);
        }
        else if( ! strncmp( "null", &str[ token[tk_i].start ], 4 )) {
            value = 0;
        }
        else if( ! strncmp( "true", &str[ token[tk_i].start ], 4 )) {
            value = 1;
        }
        else if( ! strncmp( "false", &str[ token[tk_i].start ], 5 )) {
            value = 0;
        }
    }
    else if( token[tk_i].type == JSMN_STRING ) {
        if(( isdigit( (unsigned char)str[ token[tk_i].start ])) ||
            ( str[ token[tk_i].start ] == '-' && isdigit( (unsigned char)str[ token[tk_i].start + 1 ] ))) {
            value = atoi( &str[ token[tk_i].start ]);
        }
    }
    return value;
}

float json_get_float( int32_t tk_i, char *str )
{
	float value = 0.0;

	tk_i++;
	if( token[tk_i].type == JSMN_PRIMITIVE )
	{
		if( isdigit( (unsigned char)str[ token[tk_i].start ]) || ( str[ token[tk_i].start ] == '-' ))
			value = (float) atof( &str[ token[tk_i].start ]);
		else if( ! strncmp( "null", &str[ token[tk_i].start ], 4 )) value = 0.0;
		else if( ! strncmp( "true", &str[ token[tk_i].start ], 4 )) value = 1.0;
		else if( ! strncmp( "false", &str[ token[tk_i].start ], 5 )) value = 0.0;
	}
	else if( token[tk_i].type == JSMN_STRING )
	{
		if( isdigit( (unsigned char)str[ token[tk_i].start ]) ||
			( str[ token[tk_i].start ] == '-' && isdigit( (unsigned char)str[ token[tk_i].start + 1 ] )))
				value = (float) atof( &str[ token[tk_i].start ]);
	}

	return value;
}

void json_get_str( int32_t tk_i, char *src, char *dst )
{
	tk_i++;
	if( token[tk_i].type == JSMN_STRING )
	{
		strncpy( dst, &src[ token[tk_i].start ], token[tk_i].end - token[tk_i].start );
		dst[ token[tk_i].end - token[tk_i].start ] = '\0';
	}
}

void json_get_mbus_add( char *str, uint32_t clients_num )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	if ( *str == '\0' ) {
		return;
	}

	token = strtok(str,s);
	mbus_slave_table_manager_set_new_slave_secondary_address(token);
	i     = 1;
	while ( (token != NULL) && ( i < clients_num ) )
	{
		token = strtok( NULL, s );
		if (token != NULL)
		{
			mbus_slave_table_manager_set_new_slave_secondary_address(token);
			i++;
		}
	}
	mbus_slave_table_manager_save_table();
}

time_t json_get_date( char *date )
{
    struct tm fecha = { 0 };
    char *str;
    // www.epochconverter.com

    str = strtok( date, "- :" );
    fecha.tm_year = atoi( str ) - 1900;
    str = strtok( NULL, "- :" );
    fecha.tm_mon = atoi( str ) - 1;
    str = strtok( NULL, "- :" );
    fecha.tm_mday = atoi( str );

    str = strtok( NULL, "- :" );
    fecha.tm_hour = atoi( str );
    str = strtok( NULL, "- :" );
    fecha.tm_min = atoi( str );
    //str = strtok( NULL, "/ :" );
    //fecha->tm_sec = atoi( str );

    return mktime( &fecha );
}

time_t json_get_hour( char *date )
{
    struct tm fecha = { 0 };
    char *str;
    // www.epochconverter.com

    str = strtok( date, ":" );
    if ( ( *str < 0x30 ) || ( *str > 0x39 ) ) {
    	return 0;
    }
    fecha.tm_hour = atoi( str );
    str = strtok( NULL, ":" );
    fecha.tm_min = atoi( str );
    str = strtok( NULL, ":" );
    fecha.tm_sec = atoi( str );

    fecha.tm_year = 2020 - 1900;
    fecha.tm_mon  = 10 - 1;
    fecha.tm_mday = 5;

    return mktime( &fecha );
}



void json_get_send_network_params_hours( char *str )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset((unsigned int *) param.send_params_time, -1, sizeof(param.send_params_time));

	if ( *str == '\0' ) {
		rtc_system_setSendAlarmNum(0);
		return;
	}

	token                                      = strtok(str,s);
	param.send_params_time[0].send_params_time = atoi(token);
	if (param.send_params_time[0].send_params_time == 0 ) {
		param.send_params_time[0].send_params_time = 24;
	}
	i = 1;
	while ( (token != NULL) && ( i < READ_WINDOWS ) ) {
		token                        = strtok( NULL, s );
		if (token != NULL) {
			param.send_params_time[i].send_params_time = atoi(token);
			if (param.send_params_time[i].send_params_time == 0 ) {
				param.send_params_time[i].send_params_time = 24;
			}
			i++;
		}
	}

	// Sort array.
	for ( uint8_t ii = 0; ii < i; ii++ ) {
		for ( uint8_t j = 0; j < i; j++ ) {
			if ( param.send_params_time[j].send_params_time != 0xFF ) {
				if ( param.send_params_time[j].send_params_time > param.send_params_time[ii].send_params_time ) {
					int tmp                                     = param.send_params_time[ii].send_params_time;

					param.send_params_time[ii].send_params_time = param.send_params_time[j].send_params_time;
					param.send_params_time[j].send_params_time  = tmp;

					rtc_system_setSendParamsAlarmSendTime(ii, param.send_time[ii].send_time);
					rtc_system_setSendParamsAlarmSendTime(j, param.send_time[j].send_time);
				}
			}
		}
	}
	rtc_system_setSendParamsAlarmNum( i );
}

void json_get_psm( char *str )
{
	const char s[2] = ",";
	char *token;
	uint32_t t3324 = 0, t3412 = 0;

	if ( *str == '\0' ) {
		return;
	}

	token = strtok(str,s);
	t3324 = atoi(token);
	token = strtok( NULL, s );
	t3412 = atoi(token);

	params_psm_edrx_set_tt3324(t3324);
	params_psm_edrx_set_tt3412(t3412);
}

void json_decode_edrx( char *str )
{
	uint8_t edrx = 0;

    if (0 == strcmp(str,"5.12")) {
    	edrx = 0;
    }
    else if (0 == strcmp(str,"10.24")) {
    	edrx = 1;
    }
    else if (0 == strcmp(str,"20.48")) {
    	edrx = 2;
    }
    else if (0 == strcmp(str,"40.96")) {
    	edrx = 3;
    }
    else if (0 == strcmp(str,"61.44")) {
    	edrx = 4;
    }
    else if (0 == strcmp(str,"81.92")) {
    	edrx = 5;
    }
    else if (0 == strcmp(str,"102.4")) {
    	edrx = 6;
    }
    else if (0 == strcmp(str,"122.88")) {
    	edrx = 7;
    }
    else if (0 == strcmp(str,"144.36")) {
    	edrx = 8;
    }
    else if (0 == strcmp(str,"163.84")) {
    	edrx = 9;
    }
    else if (0 == strcmp(str,"327.68")) {
    	edrx = 10;
    }
    else if (0 == strcmp(str,"655.36")) {
    	edrx = 11;
    }
    else if (0 == strcmp(str,"1310.72")) {
    	edrx = 12;
    }
    else if (0 == strcmp(str,"2621.44")) {
    	edrx = 13;
    }
    else if (0 == strcmp(str,"542.88")) {
    	edrx = 14;
    }
    else if (0 == strcmp(str,"10485.76")) {
    	edrx = 15;
    }
    params_psm_edrx_set_edrx(edrx);
}

void json_get_read_hours( uint8_t num, char *str )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0, add_alarm = 0;

	if ( 0 == num ) {// When receiving first read window, reset all values
		for ( i = 0; i < READ_WINDOWS; i++ ) {
			param.read_time[i].init_time = -1;
			param.read_time[i].end_time  = -1;
			param.read_time[i].cycle     = -1;
		}
	}

	if ( *str == '\0' ) {
		rtc_system_setReadAlarmNum(0);
		return;
	}

	token                          = strtok(str,s);
	param.read_time[num].init_time = atoi(token);
	token                          = strtok( NULL, s );
	param.read_time[num].end_time  = atoi(token);
    token                          = strtok( NULL, s );
    param.read_time[num].cycle     = atoi(token);
    token                          = strtok( NULL, s );

//    param.read_time[num].cycle     = 1; //DEBUG

	if (param.read_time[num].init_time == 0 ) {
		param.read_time[num].init_time = 24;
	}
	if (param.read_time[num].end_time == 0 ) {
		param.read_time[num].end_time = 24;
	}

    for ( i = 0; i < READ_WINDOWS; i++ ) {
    	if ( ( param.read_time[i].init_time >= 1 ) && ( param.read_time[i].init_time <= 24 ) ) {
    		add_alarm++;
    	}
    }
	rtc_system_setReadAlarmNum(add_alarm);
	i = rtc_system_getReadAlarmNum();

    if ( ( param.read_time[num].init_time >= 1 ) && ( param.read_time[num].init_time <= 24 ) ) {
    	if ( ( param.read_time[num].end_time >= 1 ) && ( param.read_time[num].end_time <= 24 ) ) {
    		if ( ( param.read_time[num].cycle >= 1 ) && ( param.read_time[num].init_time <= 24*60 ) ) {// In minutes.
    			// Sort array init time.
    			for ( uint8_t ii = 0; ii < i; ii++ ) {
    				for ( uint8_t j = 0; j < i; j++ ) {
    					if ( param.read_time[j].init_time != 0xFF ){
    						if ( param.read_time[j].init_time > param.read_time[ii].init_time ) {
    							int tmp                       = param.read_time[ii].init_time;
    							int tmp1                      = param.read_time[ii].end_time;
    							int tmp2                      = param.read_time[ii].cycle;

    							param.read_time[ii].init_time = param.read_time[j].init_time;
    							param.read_time[ii].end_time  = param.read_time[j].end_time;
    							param.read_time[ii].cycle     = param.read_time[j].cycle;

    							param.read_time[j].init_time  = tmp;
    							param.read_time[j].end_time   = tmp1;
    							param.read_time[j].cycle      = tmp2;

    							rtc_system_setReadAlarmInitTime(ii, param.read_time[ii].init_time);
    							rtc_system_setReadAlarmEndTime(ii,  param.read_time[ii].end_time);
    							rtc_system_setReadAlarmCycle(ii,    param.read_time[ii].cycle);
    							rtc_system_setReadAlarmInitTime(j,  param.read_time[j].init_time);
    							rtc_system_setReadAlarmEndTime(j,   param.read_time[j].end_time);
    							rtc_system_setReadAlarmCycle(j,     param.read_time[j].cycle);
    						}
    					}
    				}
    			}
    		}
    	}
    }
}

/**
 * @fn void json_get_send_hours(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */
void json_get_send_hours( char *str )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset((unsigned int *) param.send_time, -1, sizeof(param.send_time));

	if ( *str == '\0' ) {
		rtc_system_setSendAlarmNum(0);
		return;
	}

	token                        = strtok(str,s);
	param.send_time[0].send_time = atoi(token);
	if (param.send_time[0].send_time == 0 ) {
		param.send_time[0].send_time = 24;
	}
	i = 1;
	while ( (token != NULL) && ( i < READ_WINDOWS ) ) {
		token                        = strtok( NULL, s );
		if (token != NULL) {
			param.send_time[i].send_time = atoi(token);
			if (param.send_time[i].send_time == 0 ) {
				param.send_time[i].send_time = 24;
			}
			i++;
		}
	}

	// Sort array.
	for ( uint8_t ii = 0; ii < i; ii++ ) {
		for ( uint8_t j = 0; j < i; j++ ) {
			if ( param.send_time[j].send_time != 0xFF ) {
				if ( param.send_time[j].send_time > param.send_time[ii].send_time ) {
					int tmp             = param.send_time[ii].send_time;
					param.send_time[ii].send_time = param.send_time[j].send_time;
					param.send_time[j].send_time  = tmp;
					rtc_system_setSendAlarmSendTime(ii, param.send_time[ii].send_time);
					rtc_system_setSendAlarmSendTime(j, param.send_time[j].send_time);
				}
			}
		}
	}
	rtc_system_setSendAlarmNum( i );
}

/**
 * @fn void json_get_sensor_read_hours(uint8_t, char*)
 * @brief
 *
 * @pre
 * @post
 * @param num
 * @param str
 */
void json_get_sensor_read_hours( uint8_t num, char *str )
{
	const char s[2]  = ",";
	const char s1[2] = ";";
	const char s2[2] = "-";
	char *token;
	uint8_t i = 0, add_alarm = 0;

	if ( 0 == num ) {// When receiving first read window, reset all values
		for ( i = 0; i < READ_WINDOWS; i++ ) {
			param.sensor_read_time[i].init_time = -1;
			param.sensor_read_time[i].end_time  = -1;
			param.sensor_read_time[i].cycle     = -1;
			param.sensor_read_date_day[i]       = -1;
			param.sensor_read_date_month[i]     = -1;
		}
	}

	if ( *str == '\0' ) {
		rtc_system_setReadSensorAlarmNum(0);
		return;
	}

	token                                       = strtok(str,s);
	if (token != NULL)
	param.sensor_read_time[num].init_time       = atoi(token);
	token                                       = strtok( NULL, s );
	if (token != NULL)
	param.sensor_read_time[num].end_time        = atoi(token);
    token                                       = strtok( NULL, s1 );
    if (token != NULL)
    param.sensor_read_time[num].cycle           = atoi(token);
    token                                       = strtok( NULL, s2 );
    if (token != NULL)
    param.sensor_read_date_day[num]      	    = atoi(token);
    token                               	    = strtok( NULL, s1 );
    if (token != NULL)
    param.sensor_read_date_month[num]     		= atoi(token);
    token                               	    = strtok( NULL, s1 );
    if (token != NULL)
    param.sensor_periodic_send_date_month[num]  = atoi(token);

//    param.read_time[num].cycle     = 1; //DEBUG

	if (param.sensor_read_time[num].init_time == 0 ) {
		param.sensor_read_time[num].init_time = 24;
	}
	if (param.sensor_read_time[num].end_time == 0 ) {
		param.sensor_read_time[num].end_time = 24;
	}

    for ( i = 0; i < READ_WINDOWS; i++ ) {
    	if ( ( param.sensor_read_time[i].init_time >= 1 ) && ( param.sensor_read_time[i].init_time <= 24 ) ) {
    		add_alarm++;
    	}
    }
	rtc_system_setReadSensorAlarmNum(add_alarm);
	i = rtc_system_getReadSensorAlarmNum();

    if ( ( param.sensor_read_time[num].init_time >= 1 ) && ( param.sensor_read_time[num].init_time <= 24 ) ) {
    	if ( ( param.sensor_read_time[num].end_time >= 1 ) && ( param.sensor_read_time[num].end_time <= 24 ) ) {
    		if ( ( param.sensor_read_time[num].cycle >= 1 ) && ( param.sensor_read_time[num].init_time <= 24*60 ) ) {// In minutes.
    			// Sort array init time.
    			for ( uint8_t ii = 0; ii < i; ii++ ) {
    				for ( uint8_t j = 0; j < i; j++ ) {
    					if ( param.sensor_read_time[j].init_time != 0xFF ){
    						if ( param.sensor_read_time[j].init_time > param.sensor_read_time[ii].init_time ) {
    							int tmp                              = param.sensor_read_time[ii].init_time;
    							int tmp1                             = param.sensor_read_time[ii].end_time;
    							int tmp2                             = param.sensor_read_time[ii].cycle;
    							int tmp3                             = param.sensor_read_date_day[ii];
    							int tmp4                             = param.sensor_read_date_month[ii];

    							param.sensor_read_time[ii].init_time = param.sensor_read_time[j].init_time;
    							param.sensor_read_time[ii].end_time  = param.sensor_read_time[j].end_time;
    							param.sensor_read_time[ii].cycle     = param.sensor_read_time[j].cycle;
    							param.sensor_read_date_day[ii]       = param.sensor_read_date_day[j];
    							param.sensor_read_date_month[ii]     = param.sensor_read_date_month[j];

    							param.sensor_read_time[j].init_time  = tmp;
    							param.sensor_read_time[j].end_time   = tmp1;
    							param.sensor_read_time[j].cycle      = tmp2;
    							param.sensor_read_date_day[j]        = tmp3;
    							param.sensor_read_date_month[j]      = tmp4;

    							rtc_system_setReadSensorAlarmInitTime(ii, param.sensor_read_time[ii].init_time);
    							rtc_system_setReadSensorAlarmEndTime(ii,  param.sensor_read_time[ii].end_time);
    							rtc_system_setReadSensorAlarmCycle(ii,    param.sensor_read_time[ii].cycle);
    							rtc_system_setReadSensorAlarmInitTime(j,  param.sensor_read_time[j].init_time);
    							rtc_system_setReadSensorAlarmEndTime(j,   param.sensor_read_time[j].end_time);
    							rtc_system_setReadSensorAlarmCycle(j,     param.sensor_read_time[j].cycle);
    						}
    					}
    				}
    			}
    		}
    	}
    }
    if ( 1 == i ) {
			rtc_system_setReadSensorAlarmInitTime(0, param.sensor_read_time[0].init_time);
			rtc_system_setReadSensorAlarmEndTime(0,  param.sensor_read_time[0].end_time);
			rtc_system_setReadSensorAlarmCycle(0,    param.sensor_read_time[0].cycle);
    }
}

/**
 * @fn void json_get_sensor_send_hours(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */
void json_get_sensor_send_hours( char *str )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset((unsigned int *) param.sensor_send_time, -1, sizeof(param.sensor_send_time));

	if ( *str == '\0' ) {
		rtc_system_setSendSensorAlarmNum(0);
		return;
	}

	token                               = strtok(str,s);
	param.sensor_send_time[0].send_time = atoi(token);
	if (param.sensor_send_time[0].send_time == 0 ) {
		param.sensor_send_time[0].send_time = 24;
	}
	i = 1;
	while ( (token != NULL) && ( i < READ_WINDOWS ) ) {
		token                        = strtok( NULL, s );
		if (token != NULL) {
			param.sensor_send_time[i].send_time = atoi(token);
			if (param.sensor_send_time[i].send_time == 0 ) {
				param.sensor_send_time[i].send_time = 24;
			}
			i++;
		}
	}

	// Sort array.
	for ( uint8_t ii = 0; ii < i; ii++ ) {
		for ( uint8_t j = 0; j < i; j++ ) {
			if ( param.sensor_send_time[j].send_time != 0xFF ) {
				if ( param.sensor_send_time[j].send_time > param.sensor_send_time[ii].send_time ) {
					int tmp                              = param.sensor_send_time[ii].send_time;

					param.sensor_send_time[ii].send_time = param.sensor_send_time[j].send_time;
					param.sensor_send_time[j].send_time  = tmp;

					rtc_system_setSendSensorAlarmSendTime(ii, param.sensor_send_time[ii].send_time);
					rtc_system_setSendSensorAlarmSendTime(j, param.sensor_send_time[j].send_time);
				}
			}
		}
	}
	rtc_system_setSendSensorAlarmNum( i );
}

/**
 * @fn void json_get_sensor_over_sensor_alarms(char*)
 * @brief
 *
 * @pre
 * @post
 * @param num
 * @param str
 */
void json_get_sensor_over_sensor_alarms( char *str )
{
	const char s[2]  = ",";
	const char s1[2] = ";";
	char *token;
	uint8_t i = 0, num_sensor = 0;

	// When receiving first read window, reset all values
	for ( i = 0; i < NUM_INPUTS_SENSOR; i++ )
	{
		param.over_sensor_alarm[i] = 0xFFFFFFFF;
	}
	token = strtok(str,s);
	num_sensor = atoi( token );
	i = 1;
	while ( (token != NULL) && ( i < NUM_INPUTS_SENSOR ) ) {
		token = strtok( NULL, s1 );
		if (token != NULL) {
			param.over_sensor_alarm[num_sensor-1] = atof( token );
			token      = strtok( NULL, s );
			if ( token != NULL )
			num_sensor = atoi( token );
			i++;
		}
	}
}

/**
 * @fn void json_get_sensor_over_sensor_alarms(char*)
 * @brief
 *
 * @pre
 * @post
 * @param num
 * @param str
 */
void json_get_sensor_low_sensor_alarms( char *str )
{
	const char s[2]  = ",";
	const char s1[2] = ";";
	char *token;
	uint8_t i = 0, num_sensor = 0;

	// When receiving first read window, reset all values
	for ( i = 0; i < NUM_INPUTS_SENSOR; i++ )
	{
		param.low_sensor_alarm[i] = 0xFFFFFFFF;
	}

	token = strtok(str,s);
	num_sensor = atoi( token );
	i = 1;
	while ( (token != NULL) && ( i < NUM_INPUTS_SENSOR ) ) {
		token = strtok( NULL, s1 );
		if (token != NULL) {
			param.low_sensor_alarm[num_sensor-1] = atof( token );
			token      = strtok( NULL, s );
			if ( token != NULL )
			num_sensor = atoi( token );
			i++;
		}
	}
}

/**
 * @fn void json_get_dlms_address(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */
void json_get_i2c_address( char *str )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset(srv_add, -1, sizeof(srv_add));

	if ( *str == '\0' ) {
		return;
	}

	token         = strtok(str,s);
//	srv_add[0][0] = atoi(token);
	i2c_sensor_set_address(0, atoi(token));

	i = 1;
	while ( (token != NULL) && ( i < i2c_sensor_get_num_sensors() ) )
	{
		token = strtok( NULL, s );
		if (token != NULL)
		{
//			srv_add[i][0] = atoi(token);
			i2c_sensor_set_address(i, atoi(token));
			i++;
		}
	}
}

/**
 * @fn void json_get_dlms_address(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */
void json_get_dlms_address( char *str, uint32_t clients_num )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset(srv_add, -1, sizeof(srv_add));

	if ( *str == '\0' ) {
		return;
	}

	token        = strtok(str,s);
	srv_add[0][0] = atoi(token);

	i = 1;
	while ( (token != NULL) && ( i < clients_num ) )
	{
		token = strtok( NULL, s );
		if (token != NULL)
		{
			srv_add[i][0] = atoi(token);
			i++;
		}
	}
}

/**
 * @fn void json_get_connection_config(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */
void json_get_dlms_connection_config( char *str, uint32_t clients_num )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	if ( *str == '\0' ) {
		return;
	}

	token         = strtok(str,s);
	srv_add[0][1] = atoi(token);

	i = 1;
	while ( (token != NULL) && ( i < clients_num ) )
	{
		token = strtok( NULL, s );
		if (token != NULL)
		{
			srv_add[i][1] = atoi(token);
			i++;
		}
	}
}

/**
 * @fn void json_get_connection_config(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */
void json_get_dlms_profiles_config( char *str, uint32_t clients_num )
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	if ( *str == '\0' ) {
		return;
	}

	token        			 = strtok(str,s);
	dlms_profiles_configs[0] = atoi(token);

	i = 1;
	while ( (token != NULL) && ( i < clients_num ) )
	{
		token = strtok( NULL, s );
		if (token != NULL)
		{
			dlms_profiles_configs[i] = atoi(token);
			i++;
		}
	}
}

/**
 * @defgroup System_JSON_Modbus System JSON Modbus
 * @brief
 * @{
 * */

/**
 * @fn void json_get_dlms_address(char*)
 * @brief
 *
 * @pre
 * @post
 * @param str
 */

/**
 * @fn uint32_t json_modbus_decode(char*, uint32_t)
 * @brief Decodes the json file received by the local tool.
 *
 * @param str
 * @param len
 * @return json response.
 * 				@arg 0 - No JSON file
 * 				@arg
 */
uint32_t json_modbus_decode(char *str, uint32_t len)
{
	int32_t  i;
	int32_t  data[ 26 ] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	char     string_aux[ 256 ];//, string_aux_2[ 256 ];
	uint32_t ret = 1;//, read_hours_len = 0;

	memset(string_aux, 0, 256);

	if ( 0 == strncmp(str, "AT+TEST", 7) )
	{
		return 13;
	}

	jsmn_init(&parser);
	r = jsmn_parse(&parser, str, len, token, MAX_TOKENS);

	/* If there is no JSON file then */
	if (r < 1)
	{
		return 0;
	}

	/* Configuration*/
	if ((i = json_search_section( 0, "Get-Config", str)))
	{
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8))
		{
			return 2;
		}
		else
		{
			return 4;
		}
	}

	/* Network parameters */
	if ((i = json_search_section( 0, "Get-Network", str))) {
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8)) {
			return 2;
		} else {
			return 7;
		}
	}

	/* Datakorum */
	if ((i = json_search_section( 0, "Get-Datakorum", str))) {
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8)) {
			return 2;
		} else {
			return 11;
		}
	}

	/* System parameters */
	if ((i = json_search_section( 0, "Get-Parameters", str))) {
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8)) {
			return 2;
		} else {
			return 8;
		}
	}

	/* Pressure */
	if ((i = json_search_section( 0, "Get-Pressure", str))) {
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8)) {
			return 2;
		} else {
			return 9;
		}
	}

	/* Meter */
	if ((i = json_search_section( 0, "Get-Meter", str))) {
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8)) {
			return 2;
		} else {
			return 10;
		}
	}

	/* Send data */
	if ((i = json_search_section( 0, "Send-Data", str))) {
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8)) {
			return 2;
		} else {
			return 6;
		}
	}

	/* Init-Log-Level*/
	if ((i = json_search_section( 0, "Init-Log-Level", str)))
	{
		data[4] = json_get_int(i, str);
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8))
		{
			return 2;
		}
		else
		{
			params_set_loglive_level(data[4]);
			return 4;
		}
	}

	/* Set Configuration*/
	if ((i = json_search_section( 0, "Set-Config", str)))
	{
		json_get_str(i, str, string_aux);
		if (strncmp(string_aux, modbus_get_tool_cert(), 8))
		{
			return 2;
		}
		else
		{
			r = 12;
		}
	}
#if 0
	if ((i = json_search_section(0, "Gateway-ID", str))) {
		json_get_str(i, str, string_aux);
#ifdef MBUS
//		mbus_set_watermeter_pr6_manufacturer_serial_num(string_aux);
#endif
	}
	else {
		return 1;// The server is not sending json, the device must work anyway.
	}

	if (strncmp(string_aux, Telit_dev_identifier(), 9)) {
		return 2;
	}

	if ((i = json_search_section(0, "Middleware-ID", str))) {
		json_get_str(i, str, string_aux);
		params_set_middleware_id(string_aux);
#if defined (MBUS)
		mbus_set_dewa_serial_num(string_aux);
#elif defined(UNE82326)
			une82326_device_table_manager_get_device_serial_num(0);
#endif
	}


	if(( i = json_search_section( 0, "apn", str ))) {
		json_get_str( i, str, param.APN[0].name );
	}

	if(( i = json_search_section( 0, "server", str ))) {
		json_get_str( i, str, param.server[0].name );
	}

	if ((i = json_search_section(0, "eDRX", str))) {
		data[0] = json_get_int(i, str);
		params_psm_edrx_set_edrx(data[0]);
	}

	if ((i = json_search_section(0, "PSM", str))) {
		json_get_str( i, str, string_aux );
		json_get_psm( string_aux );
	}

	if ((i = json_search_section(0, "rt1", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 0, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt2", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 1, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "st", str))) {
		json_get_str( i, str, string_aux );
		json_get_send_hours( string_aux );
	}

	if ((i = json_search_section(0, "srt", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_sensor_read_hours(0, string_aux);
		json_get_modbus_read_hours(0, string_aux_2);
	}

	if ((i = json_search_section(0, "sst", str))) {
		json_get_str( i, str, string_aux );
		json_get_str( i, str, string_aux_2 );
		json_get_sensor_send_hours( string_aux );
		json_get_modbus_send_hours( string_aux_2 );
	}

	if ((i = json_search_section(0, "pst", str))) {
		json_get_str( i, str, string_aux );
		json_get_send_network_params_hours( string_aux );
	}

	if ((i = json_search_section(0, "release_assistance", str))) {
		data[6] = json_get_int(i, str);
		params_manteinance_set_release_assistance(data[6]);
	}

	if ((i = json_search_section(0, "fwupdate", str))) {
		data[7] = json_get_int(i, str);
		if ( 1 == data[7] ) {
			ret = 3;
		}
	}

	if ((i = json_search_section(0, "firmware_version", str))) {
		json_get_str(i, str, string_aux);
		rest_get_fw_version(string_aux);
	}

	if ((i = json_search_section(0, "local_port", str))) {
		data[8] = json_get_int(i, str);
		params_modbus_log_set_enabled( data[8] );
	}

	if ((i = json_search_section(0, "reed_activation", str))) {
		data[9] = json_get_int(i, str);
		params_manteinance_set_reed_activation(data[9]);
	}

	if ((i = json_search_section(0, "delete_memory", str))) {
		data[10] = json_get_int(i, str);
		params_manteinance_set_delete_memory(data[10]);
		Datalogger_Set_Send_Disable(DATALOGGER_SEND_ENABLED);
		if ( CLEAR_MEMORY_FORCE_HARD_RESET == data[10] ) {
			ret = 4;
		} else if ( CLEAR_MEMORY_DISABLE_DATALOGGER == data[10] ) {
			Datalogger_Set_Send_Disable(DATALOGGER_SEND_DISABLED);
		}
	}

	if ((i = json_search_section(0, "reset_meters", str))) {
		data[11] = json_get_int(i, str);
		params_manteinance_set_reset_meters(data[11]);
	}

	if ((i = json_search_section(0, "private_key", str))) {
		json_get_str(i, str, string_aux);
		modbus_set_tool_cert(string_aux);
	}

	if ((i = json_search_section(0, "pulse_on", str))) {
		data[12] = json_get_int(i, str);
		if (data[12] > 3)
		{
			data[12] = 3;
		}
		pulses_set_input_num(data[12]);
		if (data[12] != 0)
		{
			params_pulses_set_on(1);
			pulses_init();
		}
		else {
			params_pulses_set_on(0);
		}
	}

	if ((i = json_search_section(0, "Meter-ID", str))) {
		json_get_str(i, str, string_aux);
		params_set_meter_id(string_aux);
#if defined(MBUS)
		mbus_set_watermeter_manufacturer_serial_num(string_aux);
#elif defined(UNE82326)
			une82326_device_table_manager_get_device_serial_num(0);
#endif
	}

	if ((i = json_search_section(0, "counter_factor", str))) {
		data[13] = json_get_int(i, str);
		params_pulses_set_pulse_factor(data[13]);
	}

	if ((i = json_search_section(0, "k1_factor", str))) {
		data[14] = json_get_int(i, str);
		params_pulses_set_pulse_unit_k_factor_out_1(data[14]);
	}

	if ((i = json_search_section(0, "k2_factor", str))) {
		data[15] = json_get_int(i, str);
		params_pulses_set_pulse_unit_k_factor_out_2(data[15]);
	}

	if ((i = json_search_section(0, "wq", str))) {
		data[16] = json_get_int(i, str);
		params_wq_set_on(data[16]);
	}

	if ((i = json_search_section(0, "sformat", str))) {
		data[6] = json_get_int(i, str);
		params_set_telegram_mode(data[6]+1);
	}

	if ((i = json_search_section(0, "sprotocol", str))) {
		json_get_str(i, str, string_aux);
		data[6] = 1;
		if ( 0 == memcmp( string_aux, "UDP", 3 ) ) {
			data[6] = 2;
		} else {
			data[6] = 1;
		}
		params_set_protocol_mode(data[6]);
	}

	if ((i = json_search_section(0, "sport", str))) {
		data[6] = json_get_int(i, str);
		param.server[0].port = data[6];
	}

	if ((i = json_search_section(0, "snbpower", str))) {
		data[6] = json_get_int(i, str);
		params_set_nbiotpwr_mode(data[6]+1);
	}

	if ( ( i = json_search_section(0, "rt_period", str) ) ) {
		data[4] = json_get_int(i, str);
		if ( (data[4] == 0) || (data[4] == 15) || (data[4] == 60) ) {
			if ( 0 == data[4] ) {
				params_config_set_disable_meter(1);
				data[4] = 15;
			} else {
				params_config_set_disable_meter(0);
			}
			data[4] = data[4] * 60;
			if (data[4] != param.config.rt) {
				param.config.send_count = 0;
				shutdown_reset_meter_count();
			}
			param.config.rt = data[4];
		}
	}

	if ( ( i = json_search_section(0, "st_period", str) ) ) {
		data[5] = json_get_int(i, str);
		if ( (data[5] == 15) || (data[5] == 60) || (data[5] == 120) || (data[5] == 240) || (data[5] == 360) || (data[5] == 1440) ) {
			data[5] = data[5] * 60;
			if (data[5] != param.config.st) {
				param.config.send_count = 0;
				shutdown_reset_meter_count();
			}
			param.config.st = data[5];
		}
	}

	if ((i = json_search_section(0, "period", str))) {
		data[16] = json_get_int(i, str);
		params_config_set_period(data[16]);
	}

	if ((i = json_search_section(0, "sensor", str))) {
		data[16] = json_get_int(i, str);
		AD_SetADOn(data[16]);
	}

	if ((i = json_search_section(0, "sensor_log", str))) {
		data[16] = json_get_int(i, str);
		params_config_set_sensor_log(data[16]);
		if ( 1 == params_pulses_on() ) {
			params_config_set_sensor_log(1);
			AD_SetADOn(1);
		}
	} else {
		if ( 0 == params_pulses_on() ) {
			params_config_set_sensor_log(0);
		} else {
			params_config_set_sensor_log(1);
			AD_SetADOn(1);
		}
	}

	if ((i = json_search_section(0, "rt1", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 0, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt2", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 1, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt3", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 2, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt4", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 3, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt5", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 4, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt6", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 5, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt7", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 6, string_aux );
		sprintf( read_hours + read_hours_len, "%s,", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "rt8", str))) {
		json_get_str( i, str, string_aux );
		json_get_read_hours( 7, string_aux );
		sprintf( read_hours + read_hours_len, "%s", string_aux );
		read_hours_len = strlen( read_hours );
	}

	if ((i = json_search_section(0, "MODBUS_dt", str))) {
		data[17] = json_get_int(i, str);
		generic_485_set_device(data[17]);
		if ( 1 == data[17] ) {
			generic_485_set_type(MODBUS_RAW);
			params_config_set_sensor_log(1);
		} else if ( 2 == data[17] ) {
			generic_485_set_type(MODBUS_RAW);
			generic_485_set_dtl645(1);
			params_config_set_sensor_log(1);
		}
		else {
			generic_485_set_type(LAST_GENERIC_485_TYPE);
		}
	}

	if ((i = json_search_section(0, "modbus_speed", str))) {
		data[18] = json_get_int(i, str);
	    modbus_sensors_set_serial_config_baud_rate( data[18] );
	}

	if ((i = json_search_section(0, "modbus_mode", str))) {
//		json_get_str(i, str, string_aux);
		data[19] = json_get_int(i, str);//1;
		modbus_sensors_set_serial_config_stop_bits( data[19] );
//		if ( 0 == memcmp( string_aux, "8N1", 3 ) ) {
//			 modbus_sensors_set_serial_config_stop_bits( serial_8N1 );
//		} else if ( 0 == memcmp( string_aux, "8N2", 3 ) ) {
//			 modbus_sensors_set_serial_config_stop_bits( serial_8N2 );
//		}
	}

	if ((i = json_search_section(0, "modbus_parity", str))) {
//		json_get_str(i, str, string_aux);
		data[19] = json_get_int(i, str);//1;
		modbus_sensors_set_serial_config_parity( data[19] );
//		if ( 0 == memcmp( string_aux, "even", 4 ) ) {
//			modbus_sensors_set_serial_config_parity( parity_even );
//		} else if ( 0 == memcmp( string_aux, "odd", 3 ) ) {
//			modbus_sensors_set_serial_config_parity( parity_odd );
//		} else {
//			modbus_sensors_set_serial_config_parity( no_parity );
//		}
	}

	if ((i = json_search_section(0, "sync_read", str))) {
		data[20] = json_get_int(i, str);
		params_config_set_sync_read(data[20]);
	}

	if ((i = json_search_section(0, "slave_id", str))) {
//		data[20] = json_get_int(i, str);
//		generic_485_set_slave_id(data[20]);
		json_get_str( i, str, string_aux );
		if  ( 0 == generic_485_get_dtl645() )
		{
			json_get_modbus_slave_id( string_aux );
		}
		else if  ( 1 == generic_485_get_dtl645() )
		{
			json_get_modbus_address_645( string_aux );
		}
	}

	if ((i = json_search_section(0, "function", str))) {
//		data[21] = json_get_int(i, str);
//		generic_485_set_function(data[21]);
		json_get_str( i, str, string_aux );
		json_get_modbus_function(string_aux);
	}

	if ((i = json_search_section(0, "addr", str))) {
//		data[22] = json_get_int(i, str);
//		generic_485_set_addr(data[22]);
		json_get_str( i, str, string_aux );
		json_get_modbus_address( string_aux );
	}

	if ((i = json_search_section(0, "quantity", str))) {
//		data[23] = json_get_int(i, str);
//		generic_485_set_quantity(data[23]);
		json_get_str( i, str, string_aux );
		json_get_modbus_quantity( string_aux );
		generic_485_update_slaves();
	}

	if ((i = json_search_section(0, "modbus_read", str))) {
		data[24] = json_get_int(i, str);
		if ( data[24] != generic_485_get_read_time() ) {
			shutdown_reset_modbus_count();
		}
		generic_485_set_read_time(data[24]);
//		params_config_set_generic_modbus_read_time(data[24]);
	}

	if ((i = json_search_section(0, "modbus_send", str))) {
		data[25] = json_get_int(i, str);
		if ( data[25] != generic_485_get_send_time() ) {
			shutdown_reset_modbus_count();
		}
		generic_485_set_send_time(data[25]);
//		dlms_client_upload_read_send_time_with_load_profile();
		if ( 0 != generic_485_get_send_time() ) {
			params_config_set_sensor_log(1);
		}
	}

	if ((i = json_search_section(0, "overpressure_alarm", str))) {
		float_t value = 0.0;
		value = json_get_float(i, str);
		params_config_set_overpressure_alarm( value );
	}

	if ((i = json_search_section(0, "low_pressure_alarm", str))) {
		float_t value = 0.0;
		value = json_get_float(i, str);
		params_config_set_lowpressure_alarm( value );
	}

	if ((i = json_search_section(0, "reset_time", str))) {
		data[25] = json_get_hour( &str[ token[i+1].start ]);
		params_set_time_reset(data[25]);
		rtc_system_SetResetAlarm();
	}

	if ((i = json_search_section(0, "timeout_connection", str))) {
		data[18] = json_get_int(i, str);
		params_set_timeout_connection( data[18] );
	}

	if ((i = json_search_section(0, "retries_socket", str))) {
		data[19] = json_get_int(i, str);
		params_set_retries_socket( data[19] );
	}

	if ((i = json_search_section(0, "timeout_server", str))) {
		data[20] = json_get_int(i, str);
		params_set_timeout_server( data[20] );
	}

	if ((i = json_search_section(0, "mbus_baudrate", str))) {
		data[21] = json_get_int(i, str);
		params_set_mbus_baudrate( data[21] );
	}

	if ((i = json_search_section(0, "retries_server", str))) {
		data[22] = json_get_int(i, str);
		params_set_retries_server( data[22] );
	}

	if ((i = json_search_section(0, "max_dtlog_size", str))) {
		data[23] = json_get_int(i, str);
		params_set_max_datalogger_size( data[23] );
	}

	if ((i = json_search_section(0, "max_dtlog_msgs", str))) {
		data[24] = json_get_int(i, str);
		params_set_max_datalogger_msgs( data[24] );
	}

	if ((i = json_search_section(0, "no_ack", str))) {
		data[23] = json_get_int(i, str);
		udp_protocol_set_no_ack(data[23]);
	}
//	else
//	{
//		udp_protocol_set_no_ack(0);
//	}

	if ((i = json_search_section(0, "pulse_acc", str))) {
		data[23] = json_get_int(i, str);
		if ( 1 == params_pulses_on() )
		{
			if ( params_get_pulse_acc() != data[23])
			{
				params_set_pulse_acc_backup(data[23]);
			}
			else
			{
				params_set_pulse_acc(data[23]);
				params_set_pulse_acc_backup(data[23]);
			}
			if ( params_get_pulse_acc_backup() == params_get_pulse_acc() )
			{
				if ( 1 == data[23] )
				{
					params_sensor_read_time_set_cycle(0, param.config.rt);
				}
				else
				{
					params_sensor_read_time_set_cycle(0, 30);
				}
			}
		}
		else
		{
			params_set_pulse_acc(data[23]);
			params_set_pulse_acc_backup(data[23]);
		}
	}
//	else
//	{
//		params_set_pulse_acc(0);
//		params_sensor_read_time_set_cycle(0, 30);
//	}

	if ((i = json_search_section(0, "meter_type", str))) {
		data[23] = json_get_int(i, str);
		params_set_uart_meter_type(data[23]);
	}

	if ((i = json_search_section(0, "mqtt_dc_on", str))) {
		data[24] = json_get_int(i, str);
		params_set_mqtt_dc_on(data[24]);
	}

	if ((i = json_search_section(0, "dlms_enable", str))) {
		data[24] = json_get_int(i, str);
//		dlms_client_set_param_change(0);
		if (data[24] != dlms_client_get_dlms_enable())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_dlms_enable(data[24]);
	}

	if ((i = json_search_section(0, "dlms_read", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_read_time())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_read_time(data[24]);
	}

	if ((i = json_search_section(0, "dlms_send", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_send_time())
		{
			dlms_client_set_param_change(1);
			shutdown_reset_modbus_count();
		}
//		if (dlms_client_get_dlms_load_profile_read_time() == 0)
//		{
			dlms_client_set_send_time(data[24]);
//		}
		if ( 0 != dlms_client_get_send_time() ) {
			params_config_set_sensor_log(1);
		}
	}

	if ((i = json_search_section(0, "dlms_cl_address", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_cl_address())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_cl_address(data[24]);
	}

	if ((i = json_search_section(0, "dlms_logical_address", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_logical_address())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_logical_address(data[24]);
	}

	if ((i = json_search_section(0, "dlms_srv_address", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_srv_address())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_srv_address(data[24]);
	}

	if ((i = json_search_section(0, "dlms_baudrate", str))) {
		data[24] = json_get_int(i, str);
		if (data[24] != dlms_client_get_baudrate())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_baudrate(data[24]);
	}
	else
	{
		dlms_client_set_baudrate(19200);
	}

	if ((i = json_search_section(0, "dlms_authen_pass", str))) {
		char authen[30];
		memset(authen, 0, sizeof(authen));
		dlms_client_get_authen(authen);
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, authen, 30))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_authen(string_aux);
	}

	if ((i = json_search_section(0, "dlms_pass_is_hex", str))) {
		data[24] = json_get_int(i, str);
//		dlms_client_set_param_change(0);
		if (data[24] != dlms_client_get_pass_is_hex())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_pass_is_hex(data[24]);
	}

	if ((i = json_search_section(0, "dlms_pass_level", str))) {
		data[24] = json_get_int(i, str);
//		dlms_client_set_param_change(0);
		if (data[24] != dlms_client_get_pass_level())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_pass_level(data[24]);
	}

	if ((i = json_search_section(0, "dlms_short_name_ref", str))) {
		data[24] = json_get_int(i, str);
//		dlms_client_set_param_change(0);
		if (data[24] != dlms_client_get_short_name_ref())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_short_name_ref(data[24]);
	}

	if ((i = json_search_section(0, "dlms_billingday", str))) {
		data[24] = json_get_int(i, str);
//		dlms_client_set_param_change(0);
		if (data[24] != dlms_client_get_billingday())
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_billingday(data[24]);
	}

	if ((i = json_search_section(0, "dlms_obis_1", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(0), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,0);
	}

	if ((i = json_search_section(0, "dlms_obis_2", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(1), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,1);
	}

	if ((i = json_search_section(0, "dlms_obis_3", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(2), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,2);
	}

	if ((i = json_search_section(0, "dlms_obis_4", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(3), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,3);
	}

	if ((i = json_search_section(0, "dlms_obis_5", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(4), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,4);
	}

	if ((i = json_search_section(0, "dlms_obis_6", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(5), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,5);
	}

	if ((i = json_search_section(0, "dlms_obis_7", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(6), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,6);
	}

	if ((i = json_search_section(0, "dlms_obis_8", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(7), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,7);
	}

	if ((i = json_search_section(0, "dlms_obis_9", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(8), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,8);
	}

	if ((i = json_search_section(0, "dlms_obis_10", str))) {
		json_get_str( i, str, string_aux );
		if (strncmp(string_aux, dlms_client_get_obis_n(9), 32))
		{
			dlms_client_set_param_change(1);
		}
		dlms_client_set_obis_n(string_aux,9);
	}

	if ((i = json_search_section(0, "dlms_obis_11", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,10);
	}

	if ((i = json_search_section(0, "dlms_obis_12", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,11);
	}

	if ((i = json_search_section(0, "dlms_obis_13", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,12);
	}

	if ((i = json_search_section(0, "dlms_obis_14", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,13);
	}

	if ((i = json_search_section(0, "dlms_obis_15", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,14);
	}

	if ((i = json_search_section(0, "dlms_obis_16", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,15);
	}

	if ((i = json_search_section(0, "dlms_obis_17", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,16);
	}

	if ((i = json_search_section(0, "dlms_obis_18", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,17);
	}

	if ((i = json_search_section(0, "dlms_obis_19", str))) {
		json_get_str( i, str, string_aux );
		dlms_client_set_obis_n(string_aux,18);
	}

	if ((i = json_search_section(0, "keepalive", str))) {
		data[24] = json_get_int(i, str);
		params_set_server_keepalive(0, data[24] );
		params_set_server_keepalive(1, data[24] );
	}
#endif
	return ret;
}


/**
 * @fn void json_get_modbus_read_hours(uint8_t, char*)
 * @brief Creates the corresponidng number of alarms based on
 *
 * @param num
 * @param str
 */
void json_get_modbus_read_hours(uint8_t num, char *str)
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0, add_alarm = 0;

	if (*str == '\0')
	{
		rtc_system_setReadModbusAlarmNum(0);
		return;
	}

	/* When receiving first read window, reset all values */
	if (0 == num) {
		for (i = 0; i < READ_WINDOWS; i++)
		{
			param.modbus_read_time[i].init_time = -1;
			param.modbus_read_time[i].end_time  = -1;
			param.modbus_read_time[i].cycle     = -1;
		}
	}

	token = strtok(str,s);
	param.modbus_read_time[num].init_time = atoi(token);
	token = strtok( NULL, s );
	param.modbus_read_time[num].end_time = atoi(token);
    token = strtok( NULL, s );
    param.modbus_read_time[num].cycle = atoi(token);
    token = strtok( NULL, s );

    /* Adjust time */
	if (param.modbus_read_time[num].init_time == 0)
	{
		param.modbus_read_time[num].init_time = 24;
	}

	if (param.modbus_read_time[num].end_time == 0)
	{
		param.modbus_read_time[num].end_time = 24;
	}

	/* Counts number of alarms to create */
    for (i = 0; i < READ_WINDOWS; i++)
    {
    	if ((param.modbus_read_time[i].init_time >= 1) && ( param.modbus_read_time[i].init_time <= 24))
    	{
    		add_alarm++;
    	}
    }

	rtc_system_setReadModbusAlarmNum(add_alarm);
	i = rtc_system_getReadModbusAlarmNum();

	/* */
    if ((param.modbus_read_time[num].init_time >= 1) && (param.modbus_read_time[num].init_time <= 24))
    {
    	if ((param.modbus_read_time[num].end_time >= 1) && (param.modbus_read_time[num].end_time <= 24))
    	{
    		/* cycle is in minutes.*/
    		if ((param.modbus_read_time[num].cycle >= 1) && (param.modbus_read_time[num].init_time <= 24*60))
    		{
    			/* Sort array init time. */
    			for (uint8_t k = 0; k < i; k++)
    			{
    				for (uint8_t j = 0; j < i; j++)
    				{
    					if (param.modbus_read_time[j].init_time != 0xFF)
    					{
    						if (param.modbus_read_time[j].init_time > param.modbus_read_time[k].init_time)
    						{
    							int tmp = param.modbus_read_time[k].init_time;
    							int tmp1 = param.modbus_read_time[k].end_time;
    							int tmp2 = param.modbus_read_time[k].cycle;

    							param.modbus_read_time[k].init_time = param.modbus_read_time[j].init_time;
    							param.modbus_read_time[k].end_time  = param.modbus_read_time[j].end_time;
    							param.modbus_read_time[k].cycle     = param.modbus_read_time[j].cycle;

    							param.modbus_read_time[j].init_time  = tmp;
    							param.modbus_read_time[j].end_time   = tmp1;
    							param.modbus_read_time[j].cycle      = tmp2;

    							rtc_system_setReadModbusAlarmInitTime(k, param.modbus_read_time[k].init_time);
    							rtc_system_setReadModbusAlarmEndTime(k,  param.modbus_read_time[k].end_time);
    							rtc_system_setReadModbusAlarmCycle(k,    param.modbus_read_time[k].cycle);
    							rtc_system_setReadModbusAlarmInitTime(j,  param.modbus_read_time[j].init_time);
    							rtc_system_setReadModbusAlarmEndTime(j,   param.modbus_read_time[j].end_time);
    							rtc_system_setReadModbusAlarmCycle(j,     param.modbus_read_time[j].cycle);
    						}
    					}
    				}
    			}
    		}
    	}
    }

    if (1 == i)
    {
			rtc_system_setReadModbusAlarmInitTime(0, param.modbus_read_time[0].init_time);
			rtc_system_setReadModbusAlarmEndTime(0, param.modbus_read_time[0].end_time);
			rtc_system_setReadModbusAlarmCycle(0, param.modbus_read_time[0].cycle);
    }
}

/**
 * @fn void json_get_modbus_send_hours(char*)
 * @brief Configures the RTC Alarms to set the transmission windows.
 *
 * @param str JSON string to parse
 */
void json_get_modbus_send_hours(char *str)
{
	const char s[2] = ",";
	char *token;
	uint8_t i = 0;

	memset((unsigned int *) param.modbus_send_time, -1, sizeof(param.modbus_send_time));

	/* Empty string is passed then exit */
	if ( *str == '\0' )
	{
		rtc_system_setSendModbusAlarmNum(0);
		return;
	}

	token = strtok(str,s);
	param.modbus_send_time[0].send_time = atoi(token);

	if (param.modbus_send_time[0].send_time == 0)
	{
		param.modbus_send_time[0].send_time = 24;
	}

	i = 1;
	/** Parses the JSOM array containing the transmission windows */
	while ((token != NULL) && (i < READ_WINDOWS))
	{
		token = strtok(NULL, s);

		if (token != NULL)
		{
			param.modbus_send_time[i].send_time = atoi(token);

			if (param.modbus_send_time[i].send_time == 0)
			{
				param.modbus_send_time[i].send_time = 24;
			}

			i++;
		}
	}

	/** Sort array and sets the corresponding alarms */
	for (uint8_t k = 0; k < i; k++ )
	{
		for (uint8_t j = 0; j < i; j++)
		{
			if (param.modbus_send_time[j].send_time != 0xFF)
			{
				if (param.modbus_send_time[j].send_time > param.modbus_send_time[k].send_time)
				{
					int tmp = param.modbus_send_time[k].send_time;
					param.modbus_send_time[k].send_time = param.modbus_send_time[j].send_time;
					param.modbus_send_time[j].send_time  = tmp;
					rtc_system_setSendModbusAlarmSendTime(k, param.modbus_send_time[k].send_time);
					rtc_system_setSendModbusAlarmSendTime(j, param.modbus_send_time[j].send_time);
				}
			}
		}
	}
	rtc_system_setSendModbusAlarmNum(i);
}

void json_get_modbus_slave_id( char *str, uint32_t j )
{
	const char s[2] = ",";
	char *token;
	uint32_t i = 0;

	if ( *str == '\0' )
	{
		return;
	}

	generic_485_set_slave_id_reg(0, 0);
	i = 0;
	token = strtok(str,s);
	if ( 0 == j )
	{
		generic_485_set_slave_id_type(0, generic_485_get_type());
		generic_485_set_slave_id_reg(atoi(token), 0);
	}
	else
	{
		while ( ( token != 0 ) && (i <= j) )
		{
			generic_485_set_slave_id_type(i, generic_485_get_type());
			generic_485_set_slave_id_reg(atoi(token), i++);
			token = strtok( NULL, s );
		}
	}

	generic_485_set_function_reg(j, 0xFF, i);
}


void json_get_modbus_address_645( char *str, uint32_t j )
{
	const char s[2] = ",";
	char *token;
	char bcd[14];
	uint32_t i = 0, k = 0;
	uint64_t num;

	if ( *str == '\0' )
	{
		return;
	}

	token = strtok(str,s);
	if ( 0 == j )
	{
		num = atoll(token);
		common_lib_u64tostrn_a(num, bcd, 12);
		for (k = 0; k < 12; k++)
		{
			generic_485_set_address_645(bcd[k], k, i);
		}
		generic_485_set_slave_id_reg(255, i);
	}
	else
	{
		while ( ( token != 0 ) && (i<=j) )
		{
			num = atoll(token);
			common_lib_u64tostrn_a(num, bcd, 12);
			for (k = 0; k < 12; k++)
			{
				generic_485_set_address_645(bcd[k], k, i);
			}
			generic_485_set_slave_id_reg(255, i);
			i++;
			token = strtok( NULL, s );
		}
	}
}

uint32_t modbus_num_commands = 0;
char     function_str[200];
uint32_t function_str_len = 0;
void json_get_modbus_function( char *str, uint32_t j )
{
	const char s[2] = ",";
	const char t[2] = ";";
	char *token_1;
	uint32_t i = 0;

	if ( *str == '\0' )
	{
		return;
	}

	if (0 == j)
	{
		token_1 = strtok(str,t);
		token_1 = strtok(token_1,s);
		while ( token_1 != 0 )
		{
			generic_485_set_function_reg(j, atoi(token_1), i++);
			modbus_num_commands++;
			if ( function_str_len < 200 )
			{
				sprintf(function_str + function_str_len,"%d,", atoi(token_1));
				function_str_len = strlen(function_str);
			}
			token_1 = strtok( NULL, s );
//			if (token_1[1] != ';')
//			{
//				token_1 = strtok( NULL, s );
//			}
//			else
//			{
//				break;
//			}
		}
	}
	else
	{
		token_1 = strtok(str,t);
		while ( ( token_1 != 0 ) && ( i < j ) )
		{
			if (i < j)
			{
				token_1 = strtok(NULL,t);
				i++;
			}
		}
		token_1 = strtok(token_1,s);
		i = 0;
		while ( token_1 != 0 )
		{
			generic_485_set_function_reg(j, atoi(token_1), i++);
			if ( function_str_len < 200 )
			{
				sprintf(function_str + function_str_len,"%d,", atoi(token_1));
				function_str_len = strlen(function_str);
			}
			token_1 = strtok( NULL, s );
		}
	}
	generic_485_set_function_reg(j, 0xFF, i);
}

char     address_str[200];
uint32_t address_str_len = 0;
void json_get_modbus_address( char *str, uint32_t j )
{
	const char s[2] = ",";
	const char t[2] = ";";
	char *token;
	uint32_t i = 0;

	if ( *str == '\0' )
	{
		return;
	}

	if (0 == j)
	{
		token = strtok(str,t);
		token = strtok(token,s);
		while ( token != 0 )
		{
			generic_485_set_data_id_645(j, atoi(token), i);
			generic_485_set_addr_reg(j, atoi(token), i++);
			if ( address_str_len < 200 )
			{
				sprintf(address_str + address_str_len,"%d,", atoi(token));
				address_str_len = strlen(address_str);
			}
			token = strtok( NULL, s );
//			if ( i > modbus_num_commands )
//			{
//				token = 0;
//			}
//			else
//			{
//				token = strtok( NULL, s );
//			}
		}
	}
	else
	{
		token = strtok(str,t);
		while ( ( token != 0 ) && ( i < j ) )
		{
			if (i < j)
			{
				token = strtok(NULL,t);
				i++;
			}
		}
		token = strtok(token,s);
		i = 0;
		while ( token != 0 )
		{
			generic_485_set_data_id_645(j, atoi(token), i);
			generic_485_set_addr_reg(j, atoi(token), i++);
			if ( address_str_len < 200 )
			{
				sprintf(address_str + address_str_len,"%d,", atoi(token));
				address_str_len = strlen(address_str);
			}
			token = strtok( NULL, s );
//			if ( i > modbus_num_commands )
//			{
//				token = 0;
//			}
//			else
//			{
//				token = strtok( NULL, s );
//			}
		}
	}
}

char     quantity_str[200];
uint32_t quantity_str_len = 0;
void json_get_modbus_quantity( char *str, uint32_t j )
{
	const char s[2] = ",";
	const char t[2] = ";";
	char *token;
	uint32_t i = 0;

	if ( *str == '\0' )
	{
		return;
	}

	if (0 == j)
	{
		token = strtok(str,t);
		token = strtok(token,s);
		while ( token != 0 )
		{
			generic_485_set_quantity_reg(j, atoi(token), i++);
			if ( quantity_str_len < 200 )
			{
				sprintf(quantity_str + quantity_str_len,"%d,", atoi(token));
				quantity_str_len = strlen(quantity_str);
			}
			token = strtok( NULL, s );
//			if ( i > modbus_num_commands )
//			{
//				token = 0;
//			}
//			else
//			{
//				token = strtok( NULL, s );
//			}
		}
	}
	else
	{
		token = strtok(str,t);
		while ( ( token != 0 ) && ( i < j ) )
		{
			if (i < j)
			{
				token = strtok(NULL,t);
				i++;
			}
		}
		token = strtok(token,s);
		i = 0;
		while ( token != 0 )
		{
			generic_485_set_quantity_reg(j, atoi(token), i++);
			if ( quantity_str_len < 200 )
			{
				sprintf(quantity_str + quantity_str_len,"%d,", atoi(token));
				quantity_str_len = strlen(quantity_str);
			}
			token = strtok( NULL, s );
//			if ( i > modbus_num_commands )
//			{
//				token = 0;
//			}
//			else
//			{
//				token = strtok( NULL, s );
//			}
		}
	}
}

char * json_get_function_str(void)
{
	if (function_str[0]=='\0')
	{
		return (char *)"0";
	}
	else
	{
		return function_str;
	}
}

char * json_get_address_str(void)
{
	if (address_str[0]=='\0')
	{
		return (char *)"0";
	}
	else
	{
		return address_str;
	}
}

char * json_get_quantity_str(void)
{
	if (quantity_str[0]=='\0')
	{
		return (char *)"0";
	}
	else
	{
		return quantity_str;
	}
}

void __reset_strlen_vars(void)
{
	quantity_str_len = function_str_len = address_str_len = 0;
}

void json_get_modbus_value( char *str, uint32_t j )
{
	const char s[2] = ",";
	const char t[2] = ";";
	char *token;
	uint32_t i = 0, k = 0;

	if ( *str == '\0' )
	{
		return;
	}

	if (0 == j)
	{
		token = strtok(str,t);
		token = strtok(token,s);
		while ( token != 0 )
		{
			generic_485_set_value_reg(j, atoi(token), i, k++);
			generic_485_set_is_on_demand(i,1);
			token = strtok( NULL, s );
//			if ( i > modbus_num_commands )
//			{
//				token = 0;
//			}
//			else
//			{
//				token = strtok( NULL, s );
//			}
			if ( k == generic_485_get_quantity_slave_reg(j, i) )
			{
				k=0;
				i++;
			}
		}
	}
	else
	{
		token = strtok(str,t);
		while ( ( token != 0 ) && ( i < j ) )
		{
			if (i < j)
			{
				token = strtok(NULL,t);
				i++;
			}
		}
		token = strtok(token,s);
		i = 0;
		while ( token != 0 )
		{
			generic_485_set_value_reg(j, atoi(token), i, k++);
			generic_485_set_is_on_demand(i,1);
			token = strtok( NULL, s );
//			if ( i > modbus_num_commands )
//			{
//				token = 0;
//			}
//			else
//			{
//				token = strtok( NULL, s );
//			}
			if ( k == generic_485_get_quantity_slave_reg(j,i) )
			{
				k=0;
				i++;
			}
		}
	}
}

void json_set_date2str( time_t hora_UTCs, char *str )
{
    struct tm *fecha;

    fecha = gmtime( &hora_UTCs );
    sprintf( str, "\"%04u-%02u-%02u %02u:%02u:%02u\"",
        fecha->tm_year+1900, fecha->tm_mon+1, fecha->tm_mday, fecha->tm_hour, fecha->tm_min, fecha->tm_sec );
}
/**
 * @}
 *  */ // End defgroup System_JSON_Modbus

/**
 * @}
 * */ //End defgroup System_JSON
