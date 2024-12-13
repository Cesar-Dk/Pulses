/**
  ******************************************************************************
  * @file           udp_protocol.c
  * @author 		Datakorum Development Team
  * @brief          Driver to handle LTE transmission and reception data.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * udp_protocol.c
 *
 *  Created on: 31 mar. 2019
 *      Author: smill
 */
#define _GNU_SOURCE
#include <string.h>
#include <stdlib.h>

#include "params.h"
#include "udp_protocol.h"
#include "message_queue.h"
#include "tick.h"
#include "leds.h"
#include "shutdown.h"
#include "datalogger_buffer.h"
#include "log.h"
#include "sensor_log.h"
#include "modbus_sensors_log.h"
#include "generic_modbus.h"
#include "json.h"
#include "modbus_sensors.h"
#include "dlms_client.h"
#include "dlms_log.h"
#include "fw_update.h"
#include "pulses.h"
#include "mbus.h"
#include "ad.h"
#include "une82326_protocol.h"
#include "une82326_device_table.h"
#include "battery.h"
#include "generic_sensor.h"
/* For LOGLIVE */
#include "serial_modbus.h"
#include "usart.h"

#ifdef MQTT
#include "mqtt_timer.h"
#endif

#ifdef UDP

#define JSON_ERROR 			      (255)

#define SERVER_RESP               (1)
#define SERVER_FW_RESP     	  	  (3)
#define SERVER_RESET_DEV          (4)

#define MESSAGE_SERVER_RESP       (1)
#define SENSOR_SERVER_RESP        (2)
#define NETPARAMS_SERVER_RESP     (3)
#define NETWORK_SERVER_TIME_RESP  (4)
#define MODBUS_SENSOR_SERVER_RESP (5)

#define TIME_UDP_PROTOCOL         (60)//(8)//(5)//(20)//(60)

char 			str_tx[ UDP_LENGTH/3 ];
char 			str_rx[ UDP_LENGTH ];
char            param_tx[1500];
char            sensor_tx[MAX_PRESSURE_TELEGRAM_VALUES];
char            modbus_sensor_tx[MAX_MODBUS_TELEGRAM_VALUES];
char            fw_version[ 40 ];
udp_protocol_st udp_protocol_status 	         = UDP_PROT_IDLE;
uint32_t 		udp_protocol_send_pending        = 0;
uint32_t 		udp_protocol_send_network_params = 0;
uint32_t        udp_server_resp_type;
uint32_t 		time_udp_protocol;
uint32_t        upgrade_rx_data, upgrading_firmware;
uint32_t	    rtt_ini, rtt_end, send_ok;
uint32_t        timeout_tries = 0, start_datalogger = 0, retry_sensor_msg = 0, start_modbus_datalogger = 0, retry_modbus_msg = 0, start_dlms_datalogger = 0, retry_dlms_msg = 0;
uint32_t        udp_protocol_no_ack = 0, udp_protocol_waiting_answer = 0;
uint32_t        udp_protocol_on_demand_command = 0, udp_protocol_billing_profile = 0, udp_protocol_on_periodic_profile = 0, udp_protocol_event_profile = 0;

uint32_t        params_from_cmd   = 0;

extern uint32_t 	rtc_refresh_time;
extern connection   con;

extern uint32_t     on_demand_backup;

static void __getNextDLMSObisMessage( void );
static void __getNextDLMSProfileMessage( void );
static void __getNextDLMSEventsProfileMessage( void );
/**
  * @brief .
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_reset_vars(void)
{
	if (UDP_PROT_WAITING_FOR_RESPONSE == udp_protocol_get_status())
	{
		__processTimeoutWaitingResponse();
	}
	else
	{
	udp_comm_reset_buffers();
	udp_reset_buffers();
	udp_protocol_set_send_pending(0);
	if (udp_protocol_get_status() != UDP_PROT_GET_FW_UPGRADE)
	{
		udp_protocol_set_status( UDP_PROT_IDLE );
	}
	udp_set_server_resp_type(0);
	udp_protocol_reset_datalogger_max_msgs_params();
	}
}

/**
  * @brief Updates the state of the udp_protocol FSM.
  * @param @arg @ref udp_protocol_st possible values .
  * @retval None.
  */
void udp_protocol_set_status( udp_protocol_st  _status )
{
	udp_protocol_status = _status;
}

/**
  * @brief Returns the current state of the udp_protocol FSM.
  * @retval Returns the current state of the udp_protocol FSM.
  */
udp_protocol_st udp_protocol_get_status(void)
{
	return udp_protocol_status;
}

/**
  * @brief
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_set_no_ack( uint32_t  _no_ack )
{
	udp_protocol_no_ack = _no_ack;
}

/**
  * @brief
  * @retval
  */
uint32_t udp_protocol_get_no_ack(void)
{
	return udp_protocol_no_ack;
}

/**
  * @brief
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_set_waiting_answer( uint32_t  _waiting_answer )
{
	udp_protocol_waiting_answer = _waiting_answer;
}

/**
  * @brief
  * @retval
  */
uint32_t udp_protocol_get_waiting_answer(void)
{
	return udp_protocol_waiting_answer;
}

/**
  * @brief
  * @param @arg @ref
  * @retval None.
  */
uint32_t udp_get_server_resp_type( void )
{
	return udp_server_resp_type;
}

/**
  * @brief
  * @param @arg @ref
  * @retval None.
  */
void udp_set_server_resp_type( uint32_t  _udp_server_resp_type )
{
	udp_server_resp_type = _udp_server_resp_type;
}

uint32_t udp_protocol_get_send_ok( void )
{
	send_ok = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR20);
	return send_ok;
}

void udp_protocol_set_send_ok( uint32_t ok )
{
	send_ok = ok;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR20, send_ok);
}

void udp_protocol_inc_send_ok( void )
{
	send_ok++;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR20, send_ok);
}

uint32_t udp_protocol_get_send_pending( void )
{
	return udp_protocol_send_pending;
}

void udp_protocol_set_send_pending( uint32_t  pend )
{
	udp_protocol_send_pending = pend;
}

uint32_t udp_protocol_get_send_network_params( void )
{
	return udp_protocol_send_network_params;
}

void udp_protocol_set_send_network_params( uint32_t  _network_params )
{
	udp_protocol_send_network_params = _network_params;
}

void rest_get_upgrading_data( void )
{
	if (comm_serial_rcx_bytes_n()) {
		memset( str_rx, 0, strlen(str_rx) );
		upgrade_rx_data = comm_serial_read_while_data((uint8_t *)str_rx);
	} else {
		comm_serial_enable_rts();
	}
}

uint32_t rest_check_upgrading_data(void)
{
    return upgrade_rx_data;
}

uint32_t rest_upgrading_firmware(void)
{
    return upgrading_firmware;
}

void rest_set_upgrading_firmware(uint32_t _upgrading_firmware)
{
	upgrading_firmware = _upgrading_firmware;
}

void rest_reset_upgrading_data(void)
{
    upgrade_rx_data = 0;
}

char * rest_get_str_rx( void )
{
	return str_rx;
}

char * rest_get_str_tx( void )
{
	return str_tx;
}

char * rest_get_modbus_sensor_tx( void )
{
	return modbus_sensor_tx;
}

void udp_reset_buffers( void )
{
	memset( str_tx, 0, sizeof( str_tx ) );
	memset( str_rx, 0, sizeof( str_rx ) );
}

uint32_t end_sensor_messages = 0;
uint32_t udp_protocol_get_end_sensor_messages( void )
{
	return end_sensor_messages;
}

void udp_protocol_set_end_sensor_messages( uint32_t _end_sensor_messages )
{
	end_sensor_messages = _end_sensor_messages;
}

/**
  * @brief .
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_set_on_demand_command( udp_protocol_st  _on_demand_command )
{
	udp_protocol_on_demand_command = _on_demand_command;
}

/**
  * @brief .
  * @retval .
  */
uint32_t udp_protocol_get_on_demand_command(void)
{
	return udp_protocol_on_demand_command;
}

/**
  * @brief .
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_set_on_periodic_profile( udp_protocol_st  _on_periodic_profile )
{
	udp_protocol_on_periodic_profile = _on_periodic_profile;
}

/**
  * @brief .
  * @retval .
  */
uint32_t udp_protocol_get_on_periodic_profile(void)
{
	return udp_protocol_on_periodic_profile;
}

/**
  * @brief .
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_set_on_billing_profile( udp_protocol_st  _on_billing_profile )
{
	udp_protocol_billing_profile = _on_billing_profile;
}

/**
  * @brief .
  * @retval .
  */
uint32_t udp_protocol_get_on_billing_profile(void)
{
	return udp_protocol_billing_profile;
}

/**
  * @brief .
  * @param @arg @ref  .
  * @retval None.
  */
void udp_protocol_set_on_event_profile( udp_protocol_st  _on_event_profile )
{
	udp_protocol_event_profile = _on_event_profile;
}

/**
  * @brief .
  * @retval .
  */
uint32_t udp_protocol_get_on_event_profile(void)
{
	return udp_protocol_event_profile;
}

static void __send_more_packets( uint8_t sending )
{
#if defined (UNE82326)
	DataLogger_SetSend(sending);
	udp_protocol_set_send_pending(1);
#elif defined (MBUS)
	DataLogger_SendEnd();
	udp_protocol_set_send_pending(1);//udp_protocol_set_send_pending(0);//TEST!!!
#endif
//	shutdown_reset_watchdog();
	udp_protocol_status = UDP_PROT_IDLE;
}

void udp_protocol_reconnect(  uint8_t sending  )
{
#if defined (UNE82326)
	DataLogger_SetSend(sending);
	udp_protocol_set_send_pending(1);
#elif defined (MBUS)
	DataLogger_SendEnd();
	udp_protocol_set_send_pending(1);//udp_protocol_set_send_pending(0);//TEST!!!
#endif
	shutdown_reset_watchdog();
	Telit_socket_set_available( NOT_CONNECTED );
	Telit_socket_reconnect();
	udp_protocol_status = UDP_PROT_IDLE;// Reconnect, session is closed after receiving HTTP OK.
}

/* GatewayIMEI|S+|idSensors|measuresCount|sensorsCount|valuelength|sensorsBits(xsensorsCount)|decimals(xsensorsCount)|timestep|datetime|Value1Value2...ValueN*/
/* 35308109022824|S+|0106|4|2|2|2|1006|20|15|20190910123524|1654se3f|*/
uint32_t msg_sensor_plus_num_total = 0;
uint32_t udp_protocol_send_sensor_plus_frame( char *ptr_header, uint32_t num_sensors, uint32_t sensors )
{
	size_t   str_len = 0, str_len_header = 0;
	uint32_t measure_count = 0, i, id_sensor[8], send_pressure = 0, send_pulses = 0;
	uint32_t datalogger_threshold = 0;
	char     header[ 256 ];
	char     str_sensors_id[9];
	char    *str_log;

	for ( i = 0; i < num_sensors; i++ ) {
		id_sensor[i] = ( ( sensors >> (i*4) ) & 0x0F );
		if ( 1 == id_sensor[i] ) {
			send_pressure = 1;
		} else if ( 6 == id_sensor[i] ) {
			send_pulses++;
		}
	}

	/*str_log:datetime|Value1Value2...ValueN|*/
	str_log = sensor_log_pressure_instant_pulses_telegram( &measure_count, send_pulses, send_pressure );
	if ( NULL == str_log ) {//TODO:Test
		return 0;
	}
	if ( 0 == retry_sensor_msg ) {
		msg_sensor_plus_num_total += measure_count;
	}
//	if ( params_config_send_time() >= 3600 ) {
		datalogger_threshold = (uint32_t)((msg_sensor_plus_num_total/2)/(params_config_send_time()/60));
		if ( datalogger_threshold > 1 ) {
			start_datalogger = 1;
			msg_sensor_plus_num_total    = 0;
		}
//	}
	retry_sensor_msg = 0;
	memset( str_sensors_id, 0, sizeof(str_sensors_id) );
	if ( 1 == send_pulses ) {
		char *bit_num = "1006";
		if (1 == params_get_pulse_acc())
		{
			bit_num = "1616";
		}
		snprintf(str_sensors_id, sizeof(str_sensors_id), "0106");
		sprintf(ptr_header,
				"%s|S+|%s|%d|2|2|%s|20|%d|"
				"%s|%s",
				ME910_IMEI(),
				str_sensors_id,
				(int)measure_count,
				bit_num,
				(int)params_sensor_read_time_cycle(0),
				str_log,
				datalog_get_idRequest()
		);
	}
	else if ( 2 == send_pulses ) {
		char *bit_num_2 = "10060606";//"10060600";
		if (1 == params_get_pulse_acc())
		{
			bit_num_2 = "16161606";//"16161600";
		}
		snprintf(str_sensors_id, sizeof(str_sensors_id), "01060600");
		sprintf(ptr_header,
				"%s|S+|%s|%d|4|2|%s|2000|%d|"
				"%s|%s",
				ME910_IMEI(),
				str_sensors_id,
				(int)measure_count,
				bit_num_2,
				(int)params_sensor_read_time_cycle(0),
				str_log,
				datalog_get_idRequest()
		);
	}
	else if ( 3 == send_pulses ) {
		char *bit_num_3 = "10060606";
		if (1 == params_get_pulse_acc())
		{
			bit_num_3 = "16161616";
		}
		snprintf(str_sensors_id, sizeof(str_sensors_id), "01060606");
		sprintf(ptr_header,
				"%s|S+|%s|%d|4|2|%s|2000|%d|"
				"%s|%s",
				ME910_IMEI(),
				str_sensors_id,
				(int)measure_count,
				bit_num_3,
				(int)params_sensor_read_time_cycle(0),
				str_log,
				datalog_get_idRequest()
		);
	}
	else {
		snprintf(str_sensors_id, sizeof(str_sensors_id), "01");
		sprintf(ptr_header,
				"%s|S+|%s|%d|2|2|1006|20|%d|"
				"%s|%s",
				ME910_IMEI(),
				str_sensors_id,
				(int)measure_count,
				(int)params_sensor_read_time_cycle(0),
				str_log,
				datalog_get_idRequest()
		);
	}

	if ( 1 == start_datalogger )
	{
		strcat(ptr_header,"|DL|");
	}

	str_len = strlen(ptr_header);

	if ( PROTOCOL_TCP == params_protocol_mode() ) {
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		memmove(ptr_header + str_len_header, ptr_header, str_len );
		memcpy(ptr_header, header, str_len_header);
		str_len += str_len_header;
	}

	return str_len;
}

uint32_t msg_sensor_num_total = 0;
uint32_t current_sensor = 0;
uint32_t udp_protocol_send_sensor_frame( char *ptr_header )
{
	size_t   str_len = 0, str_len_header = 0;
	uint32_t measure_count = 0;
	uint32_t datalogger_threshold = 0;
	char     header[ 256 ];
	char    *str_log;
	uint32_t sensor_id = 1;
	uint32_t num_dig = 5;

	if (params_input_pulse_as_sensor_get_num() != 0)
	{
//		current_sensor = generic_sensor_get_next_sensor_to_send();
		str_log        = generic_sensor_log_sensor_num_telegram( &measure_count, &sensor_id, current_sensor);
//		if (sensor_id == 3)
//		{
//			num_dig = 6;
//		}
//		else
//		{
//			num_dig = 4;
//		}
		num_dig = 6;
	}
	else
	{
		str_log = sensor_log_pressure_telegram( &measure_count, &sensor_id );
	}
	if ( NULL == str_log ) {//TODO:Test
		return 0;
	}
	if ( 0 == retry_sensor_msg ) {
		msg_sensor_num_total += measure_count;
	}
//	if ( params_config_send_time() >= 3600 ) {
		datalogger_threshold = (uint32_t)((msg_sensor_num_total/2)/(params_config_send_time()/60));
		if ( datalogger_threshold > 1 ) {
			start_datalogger = 1;
			msg_sensor_num_total    = 0;
		}
//	}
	retry_sensor_msg = 0;
	sprintf(ptr_header,
			"%s|S|%d|%d|6|%d|"
			"%s",
			ME910_IMEI(),
			(int)sensor_id,
			(int)measure_count,
			(int)num_dig,
			str_log
	);

	if ( 1 == start_datalogger )
	{
		strcat(ptr_header,"|DL|");
	}

	str_len = strlen(ptr_header);

	if ( PROTOCOL_TCP == params_protocol_mode() ) {
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		memmove(ptr_header + str_len_header, ptr_header, str_len );
		memcpy(ptr_header, header, str_len_header);
		str_len += str_len_header;
	}

	return str_len;
}

uint32_t udp_protocol_send_modbus_sensor_param_frame( char *ptr_header, uint32_t modbus_sensor_param )
{
	size_t   str_len = 0, str_len_header = 0;
	uint32_t measures_count;
	char     header[256];
	char    *str_log;
	uint32_t _modbus_sensor_param     = modbus_sensors_log_get_sensor_id(modbus_sensor_param);
	uint32_t _modbus_sensor_num_chars = modbus_sensors_log_get_num_chars_param(modbus_sensor_param);

	str_log = modbus_sensors_log_params_sensor_param_telegram(modbus_sensor_param, _modbus_sensor_num_chars, &measures_count);
	if ( NULL == str_log ) {//TODO:Test
		return 0;
	}
	sprintf(ptr_header,
			"%s|S|%d|%d|6|%d|"
			"%s",
			ME910_IMEI(),
			(int)_modbus_sensor_param,
			(int) measures_count,
			(int)_modbus_sensor_num_chars,
			str_log
	);

	str_len = strlen(ptr_header);

	if ( PROTOCOL_TCP == params_protocol_mode() ) {
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		memmove(ptr_header + str_len_header, ptr_header, str_len );
		memcpy(ptr_header, header, str_len_header);
		str_len += str_len_header;
	}

	return str_len;
}

uint32_t msg_modbus_num_total = 0;
uint32_t udp_protocol_send_modbus_sensor_raw_frame( char *ptr_header )
{
	size_t   str_len = 0, str_len_header = 0;
	uint32_t measure_count = 0;
	uint32_t datalogger_threshold = 0;
	char     header[256];
	char    *str_log;

	str_log = modbus_sensors_log_sensor_raw_telegram(&measure_count);
	if ( NULL == str_log ) {//TODO:Test
		return 0;
	}

	snprintf(ptr_header,
			strlen(str_log) + 30,
			"%s",
			str_log
	);

	if ( 1 == start_modbus_datalogger )
	{
		strcat(ptr_header,"DL|");
	}

	if ( 0 == retry_modbus_msg )
	{
		msg_modbus_num_total += measure_count;
	}
	datalogger_threshold = (uint32_t)(((msg_modbus_num_total/params_get_modbus_slave_num()) * generic_485_get_read_time())/(generic_485_get_send_time()));
	if ( datalogger_threshold > 1 )
	{
		start_modbus_datalogger = 1;
		msg_modbus_num_total    = 0;
	}
	retry_modbus_msg = 0;

	str_len = strlen(ptr_header);

	if ( PROTOCOL_TCP == params_protocol_mode() )
	{
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		memmove(ptr_header + str_len_header, ptr_header, str_len );
		memcpy(ptr_header, header, str_len_header);
		str_len += str_len_header;
	}

	return str_len;
}

uint32_t msg_dlms_num_total = 0;
uint32_t udp_protocol_send_dlms_raw_frame( char *ptr_header )
{
	size_t   str_len = 0, str_len_header = 0;
	uint32_t measure_count        = 0;
	uint32_t datalogger_threshold = 0;
	char     header[256];
	char    *str_log;

	str_log = dlms_log_sensor_raw_telegram(&measure_count);
	if ( '\0' == str_log[0] )
	{//TODO:Test
		return 0;
	}

	snprintf(ptr_header,
			strlen(str_log) + 30,
			"%s",
			str_log
	);

	if ( 1 == start_dlms_datalogger )
	{
		strcat(ptr_header,"DL|");
	}

	if ( 0 == retry_dlms_msg )
	{
		msg_dlms_num_total += measure_count;
	}
	datalogger_threshold = (uint32_t)((msg_dlms_num_total * dlms_client_get_read_time())/(dlms_client_get_send_time()));
	if ( datalogger_threshold > 1 ) {
		start_dlms_datalogger = 1;
		msg_dlms_num_total    = 0;
	}
	retry_dlms_msg = 0;

	str_len = strlen(ptr_header);

	if ( PROTOCOL_TCP == params_protocol_mode() )
	{
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		memmove(ptr_header + str_len_header, ptr_header, str_len);
		memcpy(ptr_header, header, str_len_header);
		str_len += str_len_header;
	}

	return str_len;
}

uint32_t udp_protocol_send_dlms_on_demand_raw_frame( char *ptr_header )
{
	size_t   str_len = 0, str_len_header = 0;
	uint32_t measure_count        = 0;
	uint32_t datalogger_threshold = 0;
	char     header[256];
	char    *str_log;

	str_log = modbus_sensor_tx;
	if ( '\0' == str_log[0] )
	{//TODO:Test
		return 0;
	}

	snprintf(ptr_header,
			strlen(str_log) + 30,
			"%s",
			str_log
	);

	if ( 1 == start_dlms_datalogger )
	{
		strcat(ptr_header,"DL|");
	}

	if ( 0 == retry_dlms_msg )
	{
		msg_dlms_num_total += measure_count;
	}
	datalogger_threshold = (uint32_t)((msg_dlms_num_total * dlms_client_get_read_time())/(dlms_client_get_send_time()));
	if ( datalogger_threshold > 1 ) {
		start_dlms_datalogger = 1;
		msg_dlms_num_total    = 0;
	}
	retry_dlms_msg = 0;

	str_len = strlen(ptr_header);

	if ( PROTOCOL_TCP == params_protocol_mode() )
	{
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		memmove(ptr_header + str_len_header, ptr_header, str_len);
		memcpy(ptr_header, header, str_len_header);
		str_len += str_len_header;
	}

	return str_len;
}

static char * __modbusDataloggerInfo( void )
{
	if  (1 == dlms_client_get_dlms_enable() )
	{
		return dlms_log_GetDatalogger_Info();
	}
	else
	{
		return modbus_sensors_log_GetDatalogger_Info();
	}
}

char * __getcmdIdRequest(uint32_t cmd)
{
	if (1 == cmd)
	{
		return dlms_client_get_dlms_id_request();
	}
	else
	{
		return (char *)"\0\0";
	}
}

char * __getcmdDataOnDemand(uint32_t cmd)
{
	if (1 == cmd)
	{
		return dlms_client_get_dlms_data_cmd_on_demand();
	}
	else
	{
		return (char *)"\0\0";
	}
}

uint32_t __gettamper(uint32_t cmd)
{
	if ( 0 == cmd )
	{
		return shutdown_get_tamper_param();
	}
	else
	{
		return shutdown_get_tamper_param_cmd();
	}
}

char *__get_sensor_log_info(void)
{
	if (params_input_pulse_as_sensor_get_num() != 0)
	{
		return generic_sensor_log_GetDatalogger_Info();
	}
	else
	{
		return sensor_log_GetDatalogger_Info();
	}
}

size_t u64tostrn(unsigned long long x, char *s, size_t buff_size)
{
	unsigned long long	t = x;
	size_t	i, r=1;

	while ( t >= 10 ) {
		t /= 10;
		r++;
	}

	if (s == 0)
		return r;

	if (r > buff_size)
		r = buff_size - 1;

	for (i = r; i != 0; i--) {
		s[i-1] = (char)(x % 10) + '0';
		x /= 10;
	}

	s[r] = (char)0;

	return r;
}

char pulses_totalizer[12];
char *__get_pulses_totalizer(void)
{
	memset(pulses_totalizer, 0, sizeof(pulses_totalizer));
	if ( 0 == params_pulses_on() )
	{
		pulses_totalizer[0] = '0';
	}
	else
	{
		u64tostrn(params_get_pulses_totalizer(), pulses_totalizer,sizeof(pulses_totalizer));
	}

	return pulses_totalizer;
}

uint32_t udp_protocol_send_network_params_msg( char *ptr_header, uint8_t queue )
{
	char     header[ 256 ];
	char	 message[ 2500 ];
	char	 date[ 20 ];
	char     read_intervals[120];
	char     send_intervals[120];
	char     read_sensor_intervals[120];
	char     send_sensor_intervals[120];
	char     send_params_intervals[120];
	char     read_value[7], send_value[7];
	char     pulse_params[20];
	char     modbus_params[1024];
	char     param_pressure_val[7];
	char     param_pressure_avg[7];
	char     param_pressure_max[7];
	char     param_pressure_min[7];
	uint32_t read_hours_len = 0, send_hours_len = 0, i;
	uint32_t measure_count = 0;
//	uint32_t dlms_checksum[32];

	memcpy( date, (char *)( rtc_system_getCreatedTimeFileName() ) , sizeof( date  ) );
	memset( read_intervals, '0', 120 );
	memset( send_intervals, '0', 120 );
	read_intervals[2] = '\0';
	send_intervals[2] = '\0';
	memset( read_sensor_intervals, '0', 120 );
	memset( send_sensor_intervals, '0', 120 );
	read_sensor_intervals[2] = '\0';
	send_sensor_intervals[2] = '\0';
	memset( send_params_intervals, '0', 120 );
	send_params_intervals[2] = '\0';

	if ( 0 == params_config_get_period() ) {
		for ( i = 0; i < READ_WINDOWS; i++ ) {
			if ( params_read_time_init_time(i) != 0xFF ) {
				if ( params_read_time_init_time(i) < 10 ) {
					sprintf( read_value, "0%d", params_read_time_init_time(i) );
				} else {
					sprintf( read_value, "%d", params_read_time_init_time(i) );
				}
				sprintf( read_intervals + read_hours_len, "%s", read_value );
				read_hours_len = strlen( read_intervals );
				if ( params_read_time_end_time(i) < 10 ) {
					sprintf( read_value, "0%d", params_read_time_end_time(i) );
				} else {
					sprintf( read_value, "%d", params_read_time_end_time(i) );
				}
				sprintf( read_intervals + read_hours_len, "%s", read_value );
				read_hours_len = strlen( read_intervals );
				if ( params_read_time_cycle(i) < 10 ) {
					sprintf( read_value, "0%d", params_read_time_cycle(i) );
				} else {
					sprintf( read_value, "%d", params_read_time_cycle(i) );
				}
				sprintf( read_intervals + read_hours_len, "%s", read_value );
				read_hours_len = strlen( read_intervals );
			}
		}
		read_hours_len = 0;
		for ( i = 0; i < SEND_WINDOWS; i++ ) {
			if ( params_send_time_send_time(i) != 0xFF ) {
				if ( params_send_time_send_time(i) < 10 ) {
					sprintf( send_value, "0%d", params_send_time_send_time(i) );
				} else {
					sprintf( send_value, "%d", params_send_time_send_time(i) );
				}
				sprintf(send_intervals + send_hours_len,"%s", send_value);
				send_hours_len = strlen( send_intervals );
			}
		}
		send_hours_len = 0;
		for ( i = 0; i < READ_WINDOWS; i++ ) {
			if ( params_sensor_read_time_init_time(i) != 0xFF ) {
				if ( params_sensor_read_time_init_time(i) < 10 ) {
					sprintf( read_value, "0%d", params_sensor_read_time_init_time(i) );
				} else {
					sprintf( read_value, "%d", params_sensor_read_time_init_time(i) );
				}
				sprintf( read_sensor_intervals + read_hours_len, "%s", read_value );
				read_hours_len = strlen( read_sensor_intervals );
				if ( params_sensor_read_time_end_time(i) < 10 ) {
					sprintf( read_value, "0%d", params_sensor_read_time_end_time(i) );
				} else {
					sprintf( read_value, "%d", params_sensor_read_time_end_time(i) );
				}
				sprintf( read_sensor_intervals + read_hours_len, "%s", read_value );
				read_hours_len = strlen( read_sensor_intervals );
				if ( params_sensor_read_time_cycle(i) < 10 ) {
					sprintf( read_value, "0%d", params_sensor_read_time_cycle(i) );
				} else {
					sprintf( read_value, "%d", params_sensor_read_time_cycle(i) );
				}
				sprintf( read_sensor_intervals + read_hours_len, "%s", read_value );
				read_hours_len = strlen( read_sensor_intervals );
			}
		}
		read_hours_len = 0;
		for ( i = 0; i < SEND_WINDOWS; i++ ) {
			if ( params_sensor_send_time_send_time(i) != 0xFF ) {
				if ( params_sensor_send_time_send_time(i) < 10 ) {
					sprintf( send_value, "0%d", params_sensor_send_time_send_time(i) );
				} else {
					sprintf( send_value, "%d", params_sensor_send_time_send_time(i) );
				}
				sprintf(send_sensor_intervals + send_hours_len,"%s", send_value);
				send_hours_len = strlen( send_sensor_intervals );
			}
		}
		send_hours_len = 0;
		for ( i = 0; i < SEND_WINDOWS; i++ ) {
			if ( params_send_network_params_time_send_time(i) != 0xFF ) {
				if ( params_send_network_params_time_send_time(i) < 10 ) {
					sprintf( send_value, "0%d", params_send_network_params_time_send_time(i) );
				} else {
					sprintf( send_value, "%d", params_send_network_params_time_send_time(i) );
				}
				sprintf(send_params_intervals + send_hours_len,"%s", send_value);
				send_hours_len = strlen( send_params_intervals );
			}
		}
	} else {
		read_intervals[0]        = '\0';
		send_intervals[0]        = '\0';
		read_sensor_intervals[0] = '\0';
		send_sensor_intervals[0] = '\0';
		send_params_intervals[0] = '\0';
	}

	if ( 1 == params_pulses_on() ) {
		sprintf(pulse_params, "%d;%d;%d;%d", (int)pulses_get_input_num(), (int)params_pulses_pulse_factor(), (int)params_pulses_pulse_unit_k_factor_out_1(), (int)params_pulses_pulse_unit_k_factor_out_2() );
	} else {
		sprintf(pulse_params, ";;;");
	}

	if ( params_get_modbus_type() != LAST_GENERIC_485_TYPE ) {
		sprintf(modbus_params, "%d;%d;%d;%d;%d;%s;%s;%s;%d;%d||",
				(int)params_get_modbus_type(),
				(int)modbus_sensors_get_serial_config_baud_rate(),
				(int)modbus_sensors_get_serial_config_stop_bits(),
				(int)modbus_sensors_get_serial_config_parity(),
				(int)generic_485_get_slave_id(),
				json_get_function_str(),//(int)generic_485_get_function(),
				json_get_address_str(),//(int)generic_485_get_addr(),
				json_get_quantity_str(),//(int)generic_485_get_quantity(),
				(int)generic_485_get_read_time(),
				(int)generic_485_get_send_time()
		);
	}
	else if ( (1 == dlms_client_get_dlms_enable()) /*&& (1 == dlms_client_get_param_change())*/ )
	{
//		char authen[30];
//		dlms_client_get_authen(authen);
//		dlms_client_set_param_change(0);
//		sprintf(modbus_params, "||%d;%d;%d;%d;%d;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;",
//				(int)dlms_client_get_dlms_enable(),
//				(int)dlms_client_get_read_time(),
//				(int)dlms_client_get_send_time(),
//				(int)dlms_client_get_cl_address(),
//				(int)dlms_client_get_srv_address(),
//				authen,
//				dlms_client_get_obis_n(0),
//				dlms_client_get_obis_n(1),
//				dlms_client_get_obis_n(2),
//				dlms_client_get_obis_n(3),
//				dlms_client_get_obis_n(4),
//				dlms_client_get_obis_n(5),
//				dlms_client_get_obis_n(6),
//				dlms_client_get_obis_n(7),
//				dlms_client_get_obis_n(8),
//				dlms_client_get_obis_n(9),
//				dlms_client_get_obis_n(10),
//				dlms_client_get_obis_n(11),
//				dlms_client_get_obis_n(12),
//				dlms_client_get_obis_n(13),
//				dlms_client_get_obis_n(14)
//				);
//				memcpy(modbus_params,dlms_client_get_params(),sizeof(char)*500);//modbus_params = dlms_client_get_params();
//		dlms_client_get_params(modbus_params);
		dlms_client_get_checksum_param(modbus_params);
	}
	else
	{
		sprintf(modbus_params, "||;;;;;;;;;");
	}

	memset( message, 0, 512 );
	memset( header,  0, 256 );

	memcpy(param_pressure_val, sensor_log_get_pressure(),   7);
	memcpy(param_pressure_avg, sensor_log_GetPressureAvg(), 7);
	memcpy(param_pressure_min, sensor_log_GetPressureMin(), 7);
	memcpy(param_pressure_max, sensor_log_GetPressureMax(), 7);

	if (1 == params_get_mqtt_dc_on())
	{
		if (params_manteinance_reed_activation() != 2)
		{
			if ( CHECK_VBACKUP == 0 )
			{
				shutdown_set_tamper_param(TAMPER_BATTERY_VBACKUP);
			}
		}
	}

	if ( 1 == params_get_input_alarm_sensor() )
	{
		if ( shutdown_get_alarm_sending() == 2 )
		{
			shutdown_set_tamper_param(TAMPER_INPUT_ALARM_ON);
			shutdown_set_alarm_sending(3);
		}
		else if ( shutdown_get_alarm_sending() == 4 )
		{
			shutdown_set_tamper_param(TAMPER_INPUT_ALARM_OFF);
			shutdown_set_alarm_sending(0);
		}
	}

	sprintf(message,
			"%s|P|"
			"%s|%s;%s;"												      //Communication.3.
			"%d;%d;%d;%d;%d;%d;%s;%d;"								      //Network Params.8.
			"%s;%s;%s;%s;%d;%d;%02d.%02d;%d;%d;"						  //Gateway Params.10.
			"%d;%d;%d;ERR:%d;%d;%d;%d;%d;%dEND_ERR;"					  //Send Params.9.
			"%d;%d;%d;%d;%d,%d;%d;%d;%d;"	 					          //Gateway Params.9.
			"%d;"														  //Server Port.1.
			"%d;%d;%d;%d;"												  //Period Params.4.
			"%d;"														  //File Size.1.
			"%d;"														  //Tamper.1.
			"%s;"														  //IMSI.1.
			"%d;%s;%s;%s;"								                  //Datalogger Info.4.
			"%s;"														  //Meter-ID.
			"%s;"														  //Pulse params.
			"%d;"														  //WQ.
			"%d;"														  //Synch Read.
			"%d;"														  //Overpressure.
			"%d;"														  //Lowpressure.
			"%d;"														  //.
			"%d;"														  //.
			"%d;"														  //.
			"%d;"														  //.
			"%d;"														  //.
			"%d;"														  //Max datalog size.
			"%d;"														  //Max datalog msgs.
			"%d;"														  //Sensor on.
			"%d;"														  //Sensor log.
			"%d;"														  //Format message.
			"%d;"														  //Protocol.
			"%d;"														  //NB-IoT Power.
			"%s;"										                  //Tool cert.
			"%s;"										                  //Read sensor intervals.
			"%s;"										                  //Send sensor intervals.
			"%s;"										                  //Send params intervals.
			"%s;"										                  //Pressure data.
			"%s;"										                  //IP Dev.
			"%s|"														  //Pulse Totalizer
			"||%s|"														  //DLMS Checksum.
			"%s#"														  //idRequest.
			"%s|;"														  //dataCmdOnDemand.
			"\r\n",
			ME910_IMEI(),
			//Communication.
			date, params_get_server(), params_get_APN(),//3
			// Network Params.
			(int)ME910_network_params_rssi(),    //1.
			(int)ME910_network_params_rsrq(),    //2.
			(int)ME910_network_params_rsrp(),    //3.
			(int)ME910_network_params_pci(),     //4.
			(int)ME910_network_params_sinr(),    //5.
			(int)ME910_network_params_tx_power(),//6.
			ME910_network_params_cell_id(),      //7.
			(int)ME910_network_params_abnd(),    //8.
			//Gateway Params.
			Telit_dev_identifier(),           //1.
			params_get_middleware_id(),       //2.
			read_intervals,                   //3.
			send_intervals,                   //4.
//			params_device_get_battery_var(),  //5.%s
			(int)battery_get_Vdda(),          //5.%d Battery Voltage
			(int)params_modbus_log_enabled(), //6.
			(int)params_version_major(),      //7.
			(int)params_version_minor(),      //8.
			(int)0,//fw_update                //9.
			(int)params_get_time_reset(),     //10.
			//Send Params.
			(int)params_maintenance_number_readings(),    //1.
			(int)params_maintenance_number_ok_sendings(), //2.
			(int)params_maintenance_number_nok_sendings(),//3.
			(int)params_attach_error_get_SIM_error(),     //4.
			(int)params_attach_error_get_CSQ_error(),     //5.
			(int)params_attach_error_get_CEREG_error(),   //6.
			(int)params_attach_error_get_TSO_error(),     //7.
			(int)params_attach_error_get_TSE_error(),     //8.
			(int)params_attach_error_get_IP_error(),      //9.
			//Gateway Params.
			(int)params_manteinance_release_assistance(),//1.
			(int)params_manteinance_reed_activation(),   //2.
			(int)params_manteinance_delete_memory(),     //3.
			(int)params_manteinance_reset_meters(),      //4.
			(int)params_psm_edrx_tt3324(),               //5.
			(int)params_psm_edrx_tt3412(),               //6.
			(int)params_psm_edrx_edrx(),                 //7.
			(int)ME910_network_params_rtt_mean(),        //8.
			(int)ME910_get_connection_time(),            //9.
			//Server port.
			(int)param.server[0].port,             //1.
			//Period Params.
			(int)params_config_get_period(),       //1.
			(int)params_config_read_time(),        //2.
			(int)params_config_send_time(),        //3.
			(int)params_config_get_disable_meter(),//4
			//File Size.
			(int)fw_update_get_file_size(),        //1
			//Tamper.
			(int)__gettamper(params_from_cmd),//(int)shutdown_get_tamper_param(),      //1.
			//IMSI.
			ME910_IMSI(),//1.
			//Datalogger Info.4.
#if defined(MBUS)
			(int)mbus_get_raw_telegram_length(),    //1
#elif defined(UNE82326)
			(int)Datalogger_get_frame_size(),
#endif
			Datalogger_Get_Info(),                  //2.
			__get_sensor_log_info(),//sensor_log_GetDatalogger_Info(),        //3.
			__modbusDataloggerInfo(),				//4.
			//Params.
#if defined(MBUS)
			mbus_get_watermeter_manufacturer_serial_num(),
#elif defined(UNE82326)
			une82326_device_table_manager_get_device_serial_num(0),
#endif
			pulse_params,
			(int)params_wq_on(),
			(int)params_config_get_sync_read(),
			(int)params_config_get_overpressure_alarm(),
			(int)params_config_get_lowpressure_alarm(),
			(int)params_get_timeout_connection(),
			(int)params_get_retries_socket(),
			(int)params_get_timeout_server(),
			(int)params_get_mbus_baudrate(),
			(int)params_get_retries_server(),
			(int)params_get_max_datalogger_size(),
			(int)params_get_max_datalogger_msgs(),
			(int)AD_GetADOn(),
			(int)params_config_get_sensor_log(),
			(int)params_telegram_mode(),
			(int)params_protocol_mode(),
			(int)params_nbiotpwr_mode(),
			params_modbus_get_tool_cert(),
			read_sensor_intervals,
			send_sensor_intervals,
			send_params_intervals,
			//Pressure.
			sensor_log_pressure_params(&measure_count),
			ME910_getIP(),
			__get_pulses_totalizer(),
			modbus_params,
//			ME910_getIP(),
			__getcmdIdRequest(params_from_cmd),//dlms_client_get_dlms_id_request(),
			__getcmdDataOnDemand(params_from_cmd)//dlms_client_get_dlms_data_cmd_on_demand()
	);
//	static uint32_t delete_req = 0;
//	if ((0 == params_from_cmd) && (1 == delete_req)) //if (0 == shutdown_is_tamper_send_on_demand())
//	{
//		delete_req = 0;
//		memset(dlms_client_get_dlms_id_request(),         0 , 40*sizeof(char));
//		memset(dlms_client_get_dlms_data_cmd_on_demand(), 0 , 40*sizeof(char));
//		memset(datalog_get_idRequest(),                   0 , 40*sizeof(char));
//		memset(dlms_log_get_dataCmdOnDemand(),            0 , 40*sizeof(char));
//	}
//	if ( 1 == params_from_cmd )
//	{
//		delete_req = 1;
//	}
	shutdown_set_tamper_param(TAMPER_TSEND);
	battery_set_Vdda(MAX_VOLTAGE);
	if ( ( PROTOCOL_TCP == params_protocol_mode() ) && ( 0 == queue ) ) {
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				strlen(message)
		);
		sprintf(ptr_header,
				"%s%s",
				header, message);
	} else if ( ( PROTOCOL_UDP == params_protocol_mode() ) || ( queue > 0 ) ) {
		sprintf(ptr_header,
				"%s",
				message);
	}

	return strlen(ptr_header);
}

void rest_get_fw_version( char * _fw_version )
{
	strncpy(fw_version, _fw_version, strlen(_fw_version));
}

uint32_t rest_send_get_fw_update(char *ptr_header )
{
	sprintf(ptr_header,
			"GET /file3_restws/files/firmwares/%s.e3t HTTP/1.1\r\n"
			"User-Agent: Model/CMe3100 Hardware/R1D Serial/%s Application/1.7.1 MAC/00:D0:93:37:BD:A4\r\n"
//			"Host:etisalat.e3tmqtt.com\r\n"
//			"Host:dewa.e3tmqtt.com\r\n"
			"Host:%s\r\n"
			"Connection: close\r\n"
			"Authorization: Basic Y3VjdWx1czpjdWN1bHVz\r\n"
			"\r\n",
			fw_version,
			Telit_dev_identifier(),
			param.server[ME910_server_get_index()].name);
	return strlen(ptr_header);
}

/**
 * @fn uint32_t rest_send_head(char*)
 * @brief
 *
 * @pre
 * @post
 * @param ptr_header
 * @return
 */
uint32_t rest_send_head(char *ptr_header )
{
	sprintf(ptr_header,
			"HEAD /file3_restws/file_upload/storeonly HTTP/1.1\r\n"
			"User-Agent: Model/CMe3100 Hardware/R1D Serial/%s Application/1.7.1 MAC/00:D0:93:37:BD:A4\r\n"
//			"Host:etisalat.e3tmqtt.com\r\n"
//			"Host:10.170.4.13\r\n"
			"Host:%s\r\n"
//			"Connection: close\r\n"
//			"Authorization: Basic Y3VjdWx1czpjdWN1bHVz\r\n"
			"\r\n",
			Telit_dev_identifier(),
			param.server[ME910_server_get_index()].name);
	return strlen(ptr_header);
}

/**
 * @fn void __askForServerTime(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __askForServerTime( void )
{
//	message_queue_delete( 1 );
//	Telit_socket_HTTP_connection();
	udp_protocol_status  = UDP_PROT_GET_TIME;
}

/**
 * @brief
 */
uint32_t modbus_params_num = 0;
/**
 * @fn void __checkQueueMessage(send_type)
 * @brief Checsk the type of message existing on the message queue and transmits the appropriate.
 * @post Changes the UDP protocl FSM state to UDP_IDLE or UDP_WAITING_FOR_RESPONSE depending on the transmission status.
 * @param type type of meesage @ref send_type
 * 			@arg SEND_MESSAGE
 * 			@arg SEND_NETWORK_PARAMS
 * 			@arg SEND_SENSOR
 * 			@arg SEND_MODBUS_SENSOR
 *
 */
static void __checkQueueMessage(send_type type)
{
	uint32_t msg_size = 0;

	/* Telit is initialized  */
	if (GPIO_PIN_SET == ME910_CHECK_PSM)
	{
		asm("nop");
	}
	rtt_ini = Tick_Get( MILLISECONDS );
//	LOGLIVE(LEVEL_1, "LOGLIVE> %u ME910-TCP> rtt ini: %d.\r\n", (uint16_t)Tick_Get( SECONDS ), (int)rtt_ini);
	battery_voltage_meas(1);
	if (SEND_NETWORK_PARAMS_CMD == type)
	{
		params_from_cmd = 1;
	}
	else if (SEND_NETWORK_PARAMS == type)
	{
		params_from_cmd = 0;
	}
	switch(type)
	{
		/* MBUS/UNE meter */
		case SEND_MESSAGE:
			leds_set_NET_status( SSL_Y );
//		    rtt_ini    = Tick_Get( MILLISECONDS );
#if defined(UNE82326)
//			if ( 1 == params_pulses_on() ) {
//				msg_size = datalog_buffer_read_pulses( str_tx );
//				if ( 0 == msg_size ) {
//					Telit_socket_quick_close_and_shutdown();
//					udp_protocol_status = UDP_PROT_IDLE;
//				} else {
//					leds_LED_On(LED_WHITE);
//					Telit_write_data( str_tx );
//					udp_server_resp_type = MESSAGE_SERVER_RESP;
//					udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
//					Tick_update_tick( TICK_UDPPROT );
//					time_udp_protocol    = 20;
//				}
//			} else {
				datalog_buffer_next_record();
//		        udp_put_array( (uint8_t *)datalog_frame_tx_get_send_addr(), datalog_get_frame_n() );
//				leds_LED_On(LED_WHITE);
				Telit_write_data( datalog_get_frame_tx_buffer() );//datalog_frame_tx_get_send_addr()
				udp_server_resp_type = MESSAGE_SERVER_RESP;
				udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
				Tick_update_tick( TICK_UDPPROT );
				time_udp_protocol    = params_get_timeout_server();//20;
//			}
#elif defined(MBUS)
			memset(str_tx, 0, sizeof(str_tx));
			memset(str_rx, 0, sizeof(str_rx));
			msg_size = datalog_buffer_read(str_tx, 0);

			if (0 == msg_size)
			{
				message_queue_delete(1);//delete wrong msg from queue.
				DataLogger_ResetPointers();
				Telit_socket_quick_close_and_shutdown();
				udp_protocol_status = UDP_PROT_IDLE;
			}
			else
			{
				if (TELEGRAM_MODE_RAW == params_telegram_mode())
				{
					Telit_write_data_length(str_tx, msg_size);
				}
				else
				{
					Telit_write_data(str_tx);
				}

				udp_protocol_set_waiting_answer(1);
				udp_server_resp_type = MESSAGE_SERVER_RESP;
				udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;//UDP_PROT_SEND;
				Tick_update_tick(TICK_UDPPROT);
				time_udp_protocol    = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
			}
#endif
		break;

		case SEND_NETWORK_PARAMS_CMD:
		case SEND_NETWORK_PARAMS:
			memset(str_rx,    0, sizeof(str_rx));
			if ((1 == Tick_cloud_time_init()) && (0 == rtc_refresh_time))
			{
				if (0 == params_config_get_sensor_log())
				{
					uint8_t queue;
					message_queue_delete(1);
					datalog_buffer_poll();
					memset(str_tx, 0, sizeof(str_tx));
					queue = message_queue_get_elements();

					if (message_queue_get_elements())
					{
						if (SEND_MESSAGE == message_queue_read())
						{
							message_queue_delete(1);
						}
					}

					msg_size = udp_protocol_send_network_params_msg(param_tx, queue);
					sensor_log_pressure_reset_acc_values();
					memcpy(str_tx, param_tx, msg_size);
					msg_size += datalog_buffer_read(str_tx + msg_size, msg_size);

					if (0 == msg_size)
					{
						DataLogger_ResetPointers();
						Telit_socket_quick_close_and_shutdown();
						udp_protocol_status = UDP_PROT_IDLE;
					}
					else
					{
						Telit_write_data(str_tx);

						if (0 == queue)
						{
							udp_server_resp_type = NETPARAMS_SERVER_RESP;
						}
						else
						{
							udp_server_resp_type = MESSAGE_SERVER_RESP;
						}

						udp_protocol_set_waiting_answer(1);
						udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;//UDP_PROT_SEND;
						Tick_update_tick( TICK_UDPPROT );
						time_udp_protocol    = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
					}
				}
				else
				{
					msg_size = udp_protocol_send_network_params_msg(param_tx, 0);
					if (0 == msg_size)
					{
						Telit_socket_quick_close_and_shutdown();
						udp_protocol_status = UDP_PROT_IDLE;
					}
					else
					{
						Telit_write_data(param_tx);
						udp_protocol_set_waiting_answer(1);
						udp_server_resp_type = NETPARAMS_SERVER_RESP;
						udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;//UDP_PROT_SEND;
						Tick_update_tick(TICK_UDPPROT);
						time_udp_protocol    = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
					}
				}
			}
			else
			{
				__askForServerTime();
			}
		break;

		case SEND_SENSOR:
			memset(sensor_tx, 0, sizeof(sensor_tx));
			memset(str_rx,    0, sizeof(str_rx));
			if (0 == params_pulses_on())
			{
				msg_size = udp_protocol_send_sensor_frame(sensor_tx);
			}
			else if (1 == params_pulses_on())
			{
				uint32_t sensors = 0;
				if ( 1 == pulses_get_input_num() )
				{
					sensors = 0x16;
				}
				else if ( 2 == pulses_get_input_num() )
				{
					sensors = 0x166;
				}
				else
				{
					sensors = 0x1666;
				}
				if (0 == AD_GetADOn())
				{
					sensors = 0x06;
				}
				if ( 1 == pulses_get_input_num() )
				{
					msg_size = udp_protocol_send_sensor_plus_frame(sensor_tx, 2, sensors);
				}
				else if ( 2 == pulses_get_input_num() )
				{
					msg_size = udp_protocol_send_sensor_plus_frame(sensor_tx, 3, sensors);
				}
				else
				{
					msg_size = udp_protocol_send_sensor_plus_frame(sensor_tx, 4, sensors);
				}
			}
			if (0 == msg_size)
			{
				message_queue_delete(1);//delete wrong msg from queue.
				sensor_log_pressure_reset_telegram_values();
				Telit_socket_quick_close_and_shutdown();
				udp_protocol_status = UDP_PROT_IDLE;
			}
			else
			{
				Telit_write_data_length(sensor_tx, msg_size);
				udp_protocol_set_waiting_answer(1);
				udp_server_resp_type = SENSOR_SERVER_RESP;
				udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
				Tick_update_tick(TICK_UDPPROT);
				time_udp_protocol    = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
			}
		break;

		case SEND_MODBUS_SENSOR:
			if ((MODBUS_RAW == generic_485_get_type()) || (1 == params_wq_on()))
			{
				memset(modbus_sensor_tx, 0, sizeof(modbus_sensor_tx));
				memset(str_rx,    0, sizeof(str_rx));
				msg_size = udp_protocol_send_modbus_sensor_raw_frame(modbus_sensor_tx);
				if (0 == msg_size)
				{
					message_queue_delete(1);// TODO:REVIEW
					modbus_sensors_log_params_reset_telegram_values_pointers();
					datalog_buffer_poll();
					if ( message_queue_get_elements() ) {
						udp_protocol_status = UDP_PROT_CONNECTED;
					} else {// TODO:REVIEW END
//						message_queue_delete(1);//delete wrong msg from queue.
//						modbus_sensors_log_params_reset_telegram_values_pointers();
						Telit_socket_quick_close_and_shutdown();
						udp_protocol_status = UDP_PROT_IDLE;
					}
				}
				else
				{
					Telit_write_data_length(modbus_sensor_tx, msg_size);
					udp_protocol_set_waiting_answer(1);
					udp_server_resp_type = MODBUS_SENSOR_SERVER_RESP;
					udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
					Tick_update_tick( TICK_UDPPROT );
					time_udp_protocol    = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
				}
			}
			else
			{
				if (dlms_client_get_dlms_enable())
				{
					if ((( CircularBuffer_Read(dlms_client_get_send_msg_queue()) > DLMS_SEND_MAX_DEMAND_PROF_1)
					 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= DLMS_SEND_EVENT_LOG_PROF))
					   )
					{
						uint32_t frame_size ;
						if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == DLMS_SEND_EVENT_LOG_PROF)
						{
							if ( NULL == dlms_log_sensor_raw_eventsprofile_telegram(&frame_size) )
							{
								LOGLIVE(LEVEL_1,"LOGLIVE> %d UDP PROTOCOL> EMPTY PROFILE MSG. DELETE MESSAGE QUEUE ALL!!!!!!!!!!!\r\n", (int)Tick_Get( SECONDS ));
								message_queue_delete(message_queue_get_elements());//message_queue_delete(1);//delete wrong msg from queue.
								dlms_log_params_reset_telegram_values_pointers();
								Telit_socket_quick_close_and_shutdown();
								udp_protocol_status = UDP_PROT_IDLE;
							}
							else
							{
								udp_protocol_set_waiting_answer(1);
								udp_server_resp_type = MODBUS_SENSOR_SERVER_RESP;
								udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
								Tick_update_tick( TICK_UDPPROT );
								time_udp_protocol    = params_get_timeout_server();
							}
						}
						else
						{
							if ( NULL == dlms_log_sensor_raw_profile_telegram(&frame_size) )
							{
								LOGLIVE(LEVEL_1,"LOGLIVE> %d UDP PROTOCOL> EMPTY PROFILE MSG. DELETE MESSAGE QUEUE ALL!!!!!!!!!!!\r\n", (int)Tick_Get( SECONDS ));
								message_queue_delete(message_queue_get_elements());//message_queue_delete(1);//delete wrong msg from queue.
								dlms_log_params_reset_telegram_values_pointers();
								Telit_socket_quick_close_and_shutdown();
								udp_protocol_status = UDP_PROT_IDLE;
							}
							else
							{
								udp_protocol_set_waiting_answer(1);
								udp_server_resp_type = MODBUS_SENSOR_SERVER_RESP;
								udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
								Tick_update_tick( TICK_UDPPROT );
								time_udp_protocol    = params_get_timeout_server();
							}
						}
					}
					else
					{
						memset(modbus_sensor_tx, 0, sizeof(modbus_sensor_tx));
						memset(str_rx,    0, sizeof(str_rx));
						msg_size = udp_protocol_send_dlms_raw_frame(modbus_sensor_tx);
						if (0 == msg_size)
						{
							LOGLIVE(LEVEL_1,"LOGLIVE> %d UDP PROTOCOL> EMPTY OBIS MSG. DELETE MESSAGE QUEUE ALL!!!!!!!!!!!\r\n", (int)Tick_Get( SECONDS ));
							message_queue_delete(1);//delete wrong msg from queue.
							dlms_log_params_reset_telegram_values_pointers();
							Telit_socket_quick_close_and_shutdown();
							udp_protocol_status = UDP_PROT_IDLE;
//							__getNextDLMSObisMessage();
						}
						else
						{
							Telit_write_data_length(modbus_sensor_tx, msg_size);
							udp_protocol_set_waiting_answer(1);
							udp_server_resp_type = MODBUS_SENSOR_SERVER_RESP;
							udp_protocol_status  = UDP_PROT_WAITING_FOR_RESPONSE;
							Tick_update_tick( TICK_UDPPROT );
							time_udp_protocol    = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
						}
					}
				}
			}
		break;

		default:
		break;
	}
}

/**
 * @@brief
 */
uint32_t num_meter_frames = 0;

/**
 * @fn void __processMessageServerResponse(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __processMessageServerResponse( void )
{
	if ( 1 == datalog_get_start_meter_datalogger() ) {
		num_meter_frames++;
	}
	rtt_end = Tick_Get(MILLISECONDS);
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> RTT Message: %d.\r\n", (int)Tick_Get( SECONDS ), (int)( rtt_end - rtt_ini ));
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	if ( 0 == params_config_get_sensor_log() ) {
		sensor_log_pressure_reset_telegram_values();
	}
	if ( ( 1 == datalog_check_all_mesgs_sent( ) ) /*|| ( 1 == params_pulses_on() )*/ ) {
#if defined(UNE82326)
		if ( ( ( 0 == Datalogger_CheckLIFOPointers() ) && ( 0 == datalog_buffer_n() ) ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_meter_frames > params_get_max_datalogger_msgs() ) ) || ( num_meter_frames > params_get_max_datalogger_size() )
			|| ( 1 == Datalogger_CheckLogMemIsEmpty() ) ) {
#elif defined(MBUS)
		if ( ( ( ( 0 == Datalogger_CheckLIFOPointers() ) && ( 0 == datalog_buffer_n() ) ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_meter_frames > params_get_max_datalogger_msgs() ) ) || ( num_meter_frames > params_get_max_datalogger_size() ) )
		  || ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() ) && ( 1 == datalog_get_start_meter_datalogger() ) ) ) {
#endif
			if ((message_queue_get_elements() != 0) || (dlms_client_get_send_meter() != 0))
			{
				udp_protocol_status = UDP_PROT_CONNECTED;
			}
			else
			{
			params_attach_insert_log(0);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of MBus Messages.\r\n", (int)Tick_Get( SECONDS ));
			if ( Telit_in_PSM() ) {
				Telit_socket_enters_PSM();
			} else {
				Telit_socket_close_and_shutdown();
			}
			udp_protocol_status = UDP_PROT_CLOSE;
			num_meter_frames = 0;
			uint32_t i;
			for (i = 0; i < datalog_buffer_n(); i++)
			{
				datalog_buffer_delete();
			}
//			message_queue_delete(message_queue_get_elements());
			}
			datalog_set_start_meter_datalogger(0);
			udp_protocol_set_send_pending(0);
//			shutdown_reset_meter_count();
//			if ( ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) ) {
				DataLogger_ResetPointers();
//			}
		} else {
//			__restart_session(READING_FRAMES);
			__send_more_packets(READING_FRAMES);
#if defined(UNE82326)
			if ( 0 == params_pulses_on() ) {
				datalog_frame_tx_buffer_init();
				if ( 0 == Datalogger_CheckLogMemIsEmpty() ) {
					Datalogger_NextFramesMemoryBlock();
				}
			}
#elif defined(MBUS)
#endif
//			Telit_socket_reconnect();
		}
	} else {
//		__restart_session(PARSING_NEW_FRAME);
//		Telit_socket_reconnect();
		__send_more_packets(PARSING_NEW_FRAME);
	}
}

uint32_t num_frames = 0;
static void __processSensorServerResponse( void )
{
	if ( 1 == start_datalogger ) {
		num_frames++;
	}
	end_sensor_messages = 0;
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> RTT: %d. EOM: %d.\r\n", (int)Tick_Get( SECONDS ), (int)( rtt_end - rtt_ini ), (int)end_sensor_messages);
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	if ( ( ( ( 0 == sensor_log_CheckDataInMem() ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_frames > params_get_max_datalogger_msgs() ) ) || ( num_frames > params_get_max_datalogger_size() ) ) )
	  || ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() ) && ( 1 == start_datalogger ) ) ) {
		datalog_buffer_poll();
		if ((message_queue_get_elements() != 0) || (dlms_client_get_send_meter() != 0)) {
			udp_protocol_status = UDP_PROT_CONNECTED;
		} else {
			message_queue_delete(message_queue_get_elements());
			params_attach_insert_log(0);
			if ( Telit_in_PSM() ) {
				Telit_socket_enters_PSM();
			} else {
				Telit_socket_close_and_shutdown();
			}
		}
		if ( 1 == params_pulses_on() )
		{
			if ( params_get_pulse_acc_backup() != params_get_pulse_acc() )
			{
				params_set_pulse_acc(params_get_pulse_acc_backup());
				params_set_pulse_acc_backup(params_get_pulse_acc_backup());
				if ( 1 == params_get_pulse_acc() )
				{
					params_sensor_read_time_set_cycle(0, param.config.rt);
				}
				else
				{
					params_sensor_read_time_set_cycle(0, 30);
				}
			}
		}
		num_frames       = 0;
		start_datalogger = 0;
		end_sensor_messages = 1;
//		message_queue_delete(message_queue_get_elements());
		LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of Messages. EOM: %d. \r\n", (int)Tick_Get( SECONDS ), (int)end_sensor_messages);
//		if ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) {
			sensor_log_pressure_reset_telegram_values();
//		}
	} else {
		message_queue_write( SEND_SENSOR );
//		shutdown_reset_watchdog();
//		Telit_socket_set_available( NOT_CONNECTED );
		udp_protocol_status = UDP_PROT_CONNECTED;//UDP_PROT_IDLE;// Reconnect, session is closed after receiving HTTP OK.
//		Telit_socket_reconnect();
	}

//	udp_protocol_status = UDP_PROT_CLOSE;
}

//uint32_t num_frames = 0;
static void __processGenericSensorServerResponse( void )
{
//	if ( 1 == start_datalogger ) {
//		num_frames++;
//	}
//	end_sensor_messages = 0;
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> RTT: %d. EOM: %d.\r\n", (int)Tick_Get( SECONDS ), (int)( rtt_end - rtt_ini ), (int)end_sensor_messages);
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	if ( ( ( ( 0 == generic_sensor_log_CheckDataInMem(current_sensor) ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_frames > params_get_max_datalogger_msgs() ) ) || ( num_frames > params_get_max_datalogger_size() ) ) )
	  || ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() ) && ( 1 == start_datalogger ) ) ) {
		datalog_buffer_poll();
		if (0 == generic_sensor_log_CheckDataInMem(current_sensor))
		{
			generic_sensor_log_pressure_reset_telegram_values(current_sensor);
			if ( current_sensor < ( params_input_pulse_as_sensor_get_num() - 1 ) )
			{
				current_sensor = generic_sensor_get_next_sensor_to_send();
				if (current_sensor!=0xFF)
				{
					message_queue_write( SEND_SENSOR );
					udp_protocol_status = UDP_PROT_CONNECTED;
				}
				else
				{
					current_sensor = 0;
				}
				return;
			}
			else if ( current_sensor == ( params_input_pulse_as_sensor_get_num() - 1 ) )
			{
				current_sensor = 0;
			}
		}
		if ((message_queue_get_elements() != 0) || (dlms_client_get_send_meter() != 0)) {
			udp_protocol_status = UDP_PROT_CONNECTED;
		} else {
			message_queue_delete(message_queue_get_elements());
			params_attach_insert_log(0);
			if ( Telit_in_PSM() ) {
				Telit_socket_enters_PSM();
			} else {
				Telit_socket_close_and_shutdown();
			}
		}
		num_frames       = 0;
		start_datalogger = 0;
		end_sensor_messages = 1;
		LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of Messages. EOM: %d. \r\n", (int)Tick_Get( SECONDS ), (int)end_sensor_messages);
//		generic_sensor_log_pressure_reset_telegram_values(current_sensor);
//		current_sensor = generic_sensor_get_next_sensor_to_send();
	} else {
		message_queue_write( SEND_SENSOR );
		udp_protocol_status = UDP_PROT_CONNECTED;//UDP_PROT_IDLE;// Reconnect, session is closed after receiving HTTP OK.
	}

}
static void __throwNewMessage( void )
{
	message_queue_write( SEND_MODBUS_SENSOR );
	udp_protocol_status = UDP_PROT_CONNECTED;
}

uint32_t num_modbus_frames = 0;
static void __checkNextModbusMessage( void )
{
	if ( 1 == start_modbus_datalogger )
	{
		num_modbus_frames++;
	}
	if ( ( ( 1 == on_demand_backup )
	    || ( 0 == modbus_sensors_log_CheckDataInMem() )
	    || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_modbus_frames > params_get_max_datalogger_msgs() ) ) )
	    || ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() )  && ( 1 == start_modbus_datalogger ) ) )
	{
		datalog_buffer_poll();
		if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
		{
			udp_protocol_status = UDP_PROT_CONNECTED;
		}
		else
		{
			message_queue_delete(message_queue_get_elements());
			params_attach_insert_log(0);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of Modbus Messages.\r\n", (int)Tick_Get( SECONDS ));
			if ( Telit_in_PSM() )
			{
				Telit_socket_enters_PSM();
			}
			else
			{
				Telit_socket_close_and_shutdown();
			}
		}
		num_modbus_frames       = 0;
		start_modbus_datalogger = 0;
//		message_queue_delete(message_queue_get_elements());
		modbus_sensors_log_params_reset_telegram_values_pointers();
	}
	else
	{
		__throwNewMessage();
	}
}

uint32_t num_dlms_frames = 0;

#define ENER_PROF (0)
#define LOAD_PROF (1)
#define BILL_PROF (2)
#define END_PROF  (3)

uint32_t profile_sending = ENER_PROF;

void udp_protocol_reset_profile_sending(void)
{
	profile_sending = ENER_PROF;
}

static void __getNextDLMSObisMessage( void )
{
//	static uint32_t datalogger_on = 0;
	if ( 1 == start_dlms_datalogger )
	{
		num_dlms_frames++;
	}
	shutdown_set_obis_num_msg(shutdown_get_obis_num_msg()-1);
	shutdown_set_obis_read_num_msg(shutdown_get_obis_read_num_msg()-1);

//	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Num Obis Msg:%d.\r\n", (int)Tick_Get( SECONDS ), (int)shutdown_get_obis_num_msg());
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Num Obis Profile Msg:%d. Frame type:%d. Last profile:%d. Num meter:%d\r\n", (int)Tick_Get( SECONDS ),
			(int)shutdown_get_obis_num_msg(),(int)dlms_log_get_telegram_value_frame_type(),
			(int)shutdown_get_last_obis_profile_to_send(dlms_log_get_telegram_value_num_meter()),(int)dlms_log_get_telegram_value_num_meter());

	CircularBuffer_Get(dlms_client_get_send_msg_queue());
	if ( ( (*dlms_client_get_dlms_id_request() != '\0') || ( 0 == dlms_log_CheckDataInMem() ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_modbus_frames > params_get_max_datalogger_msgs() ) ) )
			|| ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() )  && ( 1 == start_dlms_datalogger ) ) )
	{
		if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
		{
			udp_protocol_status = UDP_PROT_CONNECTED;
		}
		else
		{
			datalog_buffer_poll();
			if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
			{
				udp_protocol_status = UDP_PROT_CONNECTED;
			}
			else
			{
				if ( Telit_in_PSM() )
				{
					Telit_socket_enters_PSM();
				} else {
					Telit_socket_close_and_shutdown();
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of DLMS Messages.\r\n", (int)Tick_Get( SECONDS ));
			}
			message_queue_delete(message_queue_get_elements());
//			datalogger_on = 0;
			shutdown_set_check_datalogger(0);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Get(dlms_client_get_send_msg_queue());//todo:test
			if (( 0 == dlms_log_CheckProfileDataInMem() ) && ( 0 == dlms_log_CheckEventsProfileDataInMem() ))
			{
				if (*dlms_client_get_dlms_id_request() != '\0')
				{
					__NOP();
				}
				else
				{
					dlms_log_params_reset_telegram_values_pointers();
				}
			}
		}
	}
	else
	{
//		if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= DLMS_SEND_INST_PROF_1 )
//		 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= DLMS_SEND_MAX_DEMAND_PROF_1 ))
		if ( 1 == CircularBuffer_Find(dlms_client_get_send_msg_queue(), DLMS_SEND_INST_PROF_1, DLMS_SEND_MAX_DEMAND_PROF_1 ) )
		{
			shutdown_reset_watchdog();
			udp_protocol_status = UDP_PROT_CONNECTED;//__throwNewMessage();
		}
		else //if ( 1 == shutdown_get_profile_to_send(dlms_log_get_telegram_value_num_meter(), dlms_log_get_telegram_value_frame_type()) )//if ( dlms_log_get_telegram_value_frame_type() == shutdown_get_last_obis_profile_to_send(dlms_log_get_telegram_value_num_meter()) || (1 == datalogger_on))//if ( 1 == shutdown_get_check_datalogger())//
		{
			__NOP();
//			datalogger_on = 1;
			shutdown_reset_watchdog();
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.Mem not Empty.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_log_get_telegram_value_frame_type());
			shutdown_set_obis_num_msg(shutdown_get_obis_num_msg()+1);
			__throwNewMessage();
		}
	}
}

static void __getNextDLMSProfileMessage( void )
{
//	static int32_t datalogger_on = 0;
	if ( 1 == start_dlms_datalogger )
	{
		num_dlms_frames++;
	}
	shutdown_set_generic_profile_num_msg(shutdown_get_generic_profile_num_msg() - 1);
	shutdown_set_generic_profile_read_num_msg(shutdown_get_generic_profile_read_num_msg() - 1);

	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Num Generic Profile Msg:%d. Frame type:%d. Last profile:%d. Num meter:%d\r\n", (int)Tick_Get( SECONDS ),
			(int)shutdown_get_generic_profile_num_msg(),(int)dlms_log_get_telegram_value_frame_type(),
			(int)shutdown_get_last_profile_to_send(dlms_log_get_telegram_value_num_meter()),(int)dlms_log_get_telegram_value_num_meter());

	CircularBuffer_Get(dlms_client_get_send_msg_queue());
	if ( ( (*dlms_client_get_dlms_id_request() != '\0') || ( 0 == dlms_log_CheckProfileDataInMem() ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_modbus_frames > params_get_max_datalogger_msgs() ) ) )
			|| ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() )  && ( 1 == start_dlms_datalogger ) ) )
	{
		if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
		{
			udp_protocol_status = UDP_PROT_CONNECTED;
		}
		else
		{
			datalog_buffer_poll();
			if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
			{
				udp_protocol_status = UDP_PROT_CONNECTED;
			}
			else
			{
				if ( Telit_in_PSM() )
				{
					Telit_socket_enters_PSM();
				}
				else
				{
					Telit_socket_close_and_shutdown();
				}
//				datalogger_on = 0;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of DLMS Messages.\r\n", (int)Tick_Get( SECONDS ));
			}
			message_queue_delete(message_queue_get_elements());
			shutdown_set_check_datalogger(0);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Get(dlms_client_get_send_msg_queue());
			if (( 0 == dlms_log_CheckDataInMem() ) && ( 0 == dlms_log_CheckEventsProfileDataInMem() ))
			{
				if (*dlms_client_get_dlms_id_request() != '\0')
				{
					__NOP();
				}
				else
				{
					dlms_log_params_reset_telegram_values_pointers();
				}
			}
		}
	}
	else
	{
//		if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= DLMS_SEND_LOAD_PROF_1 )
//		 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= DLMS_SEND_EVENT_LOG_PROF ))
		if ( 1 == CircularBuffer_Find(dlms_client_get_send_msg_queue(), DLMS_SEND_LOAD_PROF_1, DLMS_SEND_BILLING_PROF ) )
		{
			shutdown_reset_watchdog();
			udp_protocol_status = UDP_PROT_CONNECTED;//__throwNewMessage();
		}
		else //if ( 1 == shutdown_get_profile_to_send(dlms_log_get_telegram_value_num_meter(), dlms_log_get_telegram_value_frame_type()) )//if (( dlms_log_get_telegram_value_frame_type() == shutdown_get_last_profile_to_send(dlms_log_get_telegram_value_num_meter()) )||( 1 == datalogger_on ) )//if ( 1 == shutdown_get_check_datalogger())//
		{
			__NOP();
//			datalogger_on = 1;
			shutdown_reset_watchdog();
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.Mem not Empty.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_log_get_telegram_value_frame_type());
			shutdown_set_generic_profile_num_msg(shutdown_get_generic_profile_num_msg() + 1);
			__throwNewMessage();
		}
	}
}

static void __getNextDLMSEventsProfileMessage( void )
{
//	static int32_t datalogger_on = 0;
	if ( 1 == start_dlms_datalogger )
	{
		num_dlms_frames++;
	}
	shutdown_set_generic_profile_num_msg(shutdown_get_generic_profile_num_msg() - 1);
	shutdown_set_generic_profile_read_num_msg(shutdown_get_generic_profile_read_num_msg() - 1);

	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Num Generic Events Profile Msg:%d. Frame type:%d. Last profile:%d. Num meter:%d\r\n", (int)Tick_Get( SECONDS ),
			(int)shutdown_get_generic_profile_num_msg(),(int)dlms_log_get_telegram_value_frame_type(),
			(int)shutdown_get_last_profile_to_send(dlms_log_get_telegram_value_num_meter()),(int)dlms_log_get_telegram_value_num_meter());

	CircularBuffer_Get(dlms_client_get_send_msg_queue());
	if ( ( (*dlms_client_get_dlms_id_request() != '\0') || ( 0 == dlms_log_CheckEventsProfileDataInMem() ) || ( ( params_get_max_datalogger_msgs() != (uint32_t)(-1) ) && ( num_modbus_frames > params_get_max_datalogger_msgs() ) ) )
			|| ( ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() )  && ( 1 == start_dlms_datalogger ) ) )
	{
		if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
		{
			udp_protocol_status = UDP_PROT_CONNECTED;
		}
		else
		{
			datalog_buffer_poll();
			if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) )
			{
				udp_protocol_status = UDP_PROT_CONNECTED;
			}
			else
			{
				if ( Telit_in_PSM() )
				{
					Telit_socket_enters_PSM();
				}
				else
				{
					Telit_socket_close_and_shutdown();
				}
//				datalogger_on = 0;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of DLMS Messages.\r\n", (int)Tick_Get( SECONDS ));
			}
			message_queue_delete(message_queue_get_elements());
			shutdown_set_check_datalogger(0);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Get(dlms_client_get_send_msg_queue());
			if (( 0 == dlms_log_CheckDataInMem() ) && ( 0 == dlms_log_CheckProfileDataInMem() ) )
			{
				if (*dlms_client_get_dlms_id_request() != '\0')
				{
					__NOP();
				}
				else
				{
					dlms_log_params_reset_telegram_values_pointers();
				}
			}
		}
	}
	else
	{
//		if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= DLMS_SEND_LOAD_PROF_1 )
//		 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= DLMS_SEND_EVENT_LOG_PROF ))
		if ( 1 == CircularBuffer_Find(dlms_client_get_send_msg_queue(), DLMS_SEND_EVENT_LOG_PROF, DLMS_SEND_EVENT_LOG_PROF ) )
		{
			shutdown_reset_watchdog();
			udp_protocol_status = UDP_PROT_CONNECTED;//__throwNewMessage();
		}
		else //if ( 1 == shutdown_get_profile_to_send(dlms_log_get_telegram_value_num_meter(), dlms_log_get_telegram_value_frame_type()) )//if (( dlms_log_get_telegram_value_frame_type() == shutdown_get_last_profile_to_send(dlms_log_get_telegram_value_num_meter()) )||( 1 == datalogger_on ) )//if ( 1 == shutdown_get_check_datalogger())//
		{
			__NOP();
//			datalogger_on = 1;
			shutdown_reset_watchdog();
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.Mem not Empty.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_log_get_telegram_value_frame_type());
			shutdown_set_generic_profile_num_msg(shutdown_get_generic_profile_num_msg() + 1);
			__throwNewMessage();
		}
	}
}

static void __processModbusSensorServerResponse( void )
{
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> RTT Modbus Sensor: %d.\r\n", (int)Tick_Get( SECONDS ), (int)( rtt_end - rtt_ini ));
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();

	if ((MODBUS_RAW == generic_485_get_type()) || (1 == params_wq_on()))
	{
		__checkNextModbusMessage();
	}
	else if (dlms_client_get_dlms_enable())
	{
		if ((dlms_log_get_telegram_value_frame_type()>=DLMS_SEND_LOAD_PROF_1)&&(dlms_log_get_telegram_value_frame_type()<=DLMS_SEND_EVENT_LOG_PROF))
		{
			if (dlms_log_get_telegram_value_frame_type()==DLMS_SEND_EVENT_LOG_PROF)
			{
				__getNextDLMSEventsProfileMessage();
			}
			else
			{
				__getNextDLMSProfileMessage();
			}
		}
		else if ((dlms_log_get_telegram_value_frame_type()>=DLMS_SEND_INST_PROF_1)&&(dlms_log_get_telegram_value_frame_type()<=DLMS_SEND_MAX_DEMAND_PROF_1))
		{
			__getNextDLMSObisMessage();
		}
	}
}

static void __processNetParamsServerResponse( void )
{
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> RTT Params: %d.\r\n", (int)Tick_Get( SECONDS ), (int)( rtt_end - rtt_ini ));
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	if ( 0 == dlms_client_get_send_meter() )
	{
		datalog_buffer_poll();
	}
	if ( message_queue_get_elements() ) {
		udp_protocol_status = UDP_PROT_CONNECTED;
	} else {
		params_attach_insert_log(0);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> End Of Messages.\r\n", (int)Tick_Get( SECONDS ));
		if ( Telit_in_PSM() ) {
			Telit_socket_enters_PSM();
		} else {
			Telit_socket_close_and_shutdown();
		}
		message_queue_delete(message_queue_get_elements());
	}
}

static void __processFWUpgradeResponse( void )
{
	message_queue_delete( 1 );
	DataLogger_SendEnd();
	udp_protocol_set_send_pending( 0 );
	shutdown_reset_watchdog();
//	__restart_session( PARSING_NEW_FRAME );
	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> Erase Flash Start.\r\n", (int)Tick_Get( SECONDS ));
	fw_update_erase_flash();
	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> Erase Flash End.\r\n", (int)Tick_Get( SECONDS ));
//	Telit_socket_HTTP_connection();
	udp_protocol_status = UDP_PROT_GET_FW_UPGRADE;
	if ( 1 == params_pulses_on() ) {
		pulses_deinit();
	}
}

void __processTimeoutWaitingResponse( void )
{
	params_maintenance_inc_number_nok_sendings();
	params_attach_error_inc_TSE_error();
	params_attach_insert_log(TSE_ERR);
	params_check_for_changes();
	if ( MESSAGE_SERVER_RESP == udp_server_resp_type) {
#if defined(UNE82326)
		datalog_recover_sent_fail_frame();
		__send_more_packets(READING_FRAMES);
#elif defined(MBUS)
		Datalogger_RecoverSendFailRecord();
		__send_more_packets(READING_FRAMES);
		datalog_set_retry_meter_msg(1);
#endif
	} else if ( SENSOR_SERVER_RESP == udp_server_resp_type) {
		if (params_input_pulse_as_sensor_get_num() != 0)
		{
			generic_sensor_log_recover_rd_address_backup(current_sensor);
		}
		else
		{
			sensor_log_recover_rd_address_backup();
		}
		retry_sensor_msg = 1;
	} else if ( MODBUS_SENSOR_SERVER_RESP == udp_server_resp_type) {
		if ((MODBUS_RAW == generic_485_get_type()) || (1 == params_wq_on()))
		{
			modbus_sensors_log_recover_rd_address_backup();
			__throwNewMessage();
		}
		else if (dlms_client_get_dlms_enable())
		{
			if ((dlms_log_get_telegram_value_frame_type()>=DLMS_SEND_INST_PROF_1)&&(dlms_log_get_telegram_value_frame_type()<=DLMS_SEND_MAX_DEMAND_PROF_1))
			{
				dlms_log_recover_rd_address_backup();
				udp_protocol_status = UDP_PROT_CONNECTED;
			}
			else if ((dlms_log_get_telegram_value_frame_type()>=DLMS_SEND_LOAD_PROF_1)&&(dlms_log_get_telegram_value_frame_type()<=DLMS_SEND_EVENT_LOG_PROF))
			{
				dlms_log_profile_recover_rd_address_backup();
				udp_protocol_status = UDP_PROT_CONNECTED;
			}
		}
		retry_modbus_msg = 1;
	}
//	shutdown_reset_watchdog();
	leds_set_NET_status( NB_MODULE_ERROR );
//	Telit_socket_quick_close_and_shutdown();
	Telit_socket_set_available( NOT_CONNECTED );
	if ( timeout_tries++ < params_get_retries_server() ) {
		Telit_socket_reconnect();
	} else {
		message_queue_delete(message_queue_get_elements());
		shutdown_set(1, params_config_read_time());
	}
	udp_protocol_status = UDP_PROT_IDLE;
}

void udp_protocol_restore_pointers( void )
{
//	LOGLIVE(LEVEL_1, "Restore pointers:%d", (int)Tick_Get( SECONDS ));
	if ( MESSAGE_SERVER_RESP == udp_server_resp_type) {
#if defined(UNE82326)
		datalog_recover_sent_fail_frame();
#elif defined(MBUS)
		Datalogger_RecoverSendFailRecord();
#endif
	} else if ( SENSOR_SERVER_RESP == udp_server_resp_type) {
		sensor_log_recover_rd_address_backup();
	} else if ( MODBUS_SENSOR_SERVER_RESP == udp_server_resp_type) {
		modbus_sensors_log_recover_rd_address_backup();
	}
}

void udp_protocol_reset_datalogger_max_msgs_params( void )
{
	num_meter_frames  = 0;
	num_frames        = 0;
	num_modbus_frames = 0;
	retry_sensor_msg  = 0;
	retry_modbus_msg  = 0;
	start_datalogger  = 0;
	start_modbus_datalogger   = 0;
	msg_sensor_num_total      = 0;
	msg_sensor_plus_num_total = 0;
	msg_modbus_num_total      = 0;
	datalog_set_retry_meter_msg(0);
	datalog_set_start_meter_datalogger(0);
	datalog_set_meter_msg_num_total(0);
}

static void __processServerResp( uint32_t resp )
{
	udp_protocol_inc_send_ok();
	battery_voltage_meas(1);
	switch(resp) {
	case MESSAGE_SERVER_RESP:
		__processMessageServerResponse();
		break;
	case SENSOR_SERVER_RESP:
		if (params_input_pulse_as_sensor_get_num() != 0)
		{
			__processGenericSensorServerResponse();
		}
		else
		{
			__processSensorServerResponse();
		}
		break;
	case MODBUS_SENSOR_SERVER_RESP:
		__processModbusSensorServerResponse();
		break;
	case NETPARAMS_SERVER_RESP:
		__processNetParamsServerResponse();
		break;
	default:
		break;
	}
}

uint32_t fw_upgrading = 0;
uint32_t fw_length;
uint32_t get_server_time = 0;

/**
 * @fn void udp_protocol_task(void)
 * @brief Network protocol system thread. It handles the transmission and reception
 * 		of data to and from the server
 */
void udp_protocol_task(void)
{
	static uint32_t msg_size = 0;

	if ( 0 == mqtt_timer_get_idle_mode() )//if ( 1 == params_get_mqtt_dc_on() ) TODO:Add??
	{
		return;
	}

	switch (udp_protocol_status)
	{
		/* remains here until socket is opened */
		case UDP_PROT_IDLE:
			if (1 == udp_is_socket_open())
			{
				if ( ( 0 == params_get_mqtt_dc_on() ) || (( 1 == params_get_mqtt_dc_on() ) && ( mqtt_timer_get_idle_mode() != 0 )) )
				{
					leds_set_NET_status(SSL_N);
					udp_protocol_status = UDP_PROT_CONNECTED;
					time_udp_protocol = 10;
					Tick_update_tick(TICK_UDPPROT);
				}
			}
		break;

		/* socket opened device connected */
		case UDP_PROT_CONNECTED:
			/* socket opened and there are data in the buffer to transmit */
			if (udp_is_socket_open() && comm_serial_is_put_ready())
			{
				/* something to send? */
				if (message_queue_get_elements())
				{
					/* */
					__checkQueueMessage(message_queue_read());
				}
				/* time elapsed has reached the end */
				else if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
				{
					Telit_socket_quick_close_and_shutdown();
					LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> ERROR! Empty message queue. No data to send.\r\n", (int)Tick_Get( SECONDS ));
					udp_protocol_status = UDP_PROT_IDLE;
				}
			}
			/* time elapsed has reached the end */
			else if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
			{
				Telit_socket_quick_close_and_shutdown();
				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> TIMEOUT CONN! No data to send.\r\n", (int)Tick_Get( SECONDS ));
				udp_protocol_status = UDP_PROT_IDLE;
			}
		break;

		case UD_PROT_GETTING_FRAMES:
		break;

		case UDP_PROT_SEND:
			if (udp_is_socket_open() && comm_serial_is_put_ready())
			{
				udp_protocol_set_waiting_answer(1);
				udp_protocol_status = UDP_PROT_WAITING_FOR_RESPONSE;
				time_udp_protocol = params_get_timeout_server();//TIME_UDP_PROTOCOL;//20;
				Tick_update_tick(TICK_UDPPROT);
			}
		break;

		case UDP_PROT_WAITING_FOR_RESPONSE:
			if (udp_is_socket_open())
			{
				if (1 == fw_upgrading)
				{
					if (rest_get_fw_update_response(str_rx, &fw_length))
					{
						Tick_update_tick(TICK_UDPPROT);
						time_udp_protocol = 1;
						udp_protocol_set_waiting_answer(0);
						udp_protocol_status = UDP_PROT_FW_UPDATING;
						LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> FIRMWARE UPDATING! Please wait...\r\n", (int)Tick_Get( SECONDS ));
					}
					else if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
					{
						fw_upgrading = 0;
						Telit_socket_quick_close_and_shutdown();
						udp_protocol_status = UDP_PROT_IDLE;
					}
				}
				else if (1 == get_server_time)
				{
					if (rest_time_response(str_rx))
					{
						get_server_time     = 0;
						rtc_refresh_time    = 0;
						uint32_t timeout_conn = params_get_timeout_connection() - Tick_Get(MILLISECONDS)/1000;//HAL_GetTick()/1000;
						if ( Tick_Get(MILLISECONDS)/1000 <  params_get_timeout_connection() ) {
							shutdown_set_wchdg_time(timeout_conn);
						} else {
							shutdown_set_wchdg_time(params_get_timeout_connection() - 10);//In case of negative timeout_conn value, we consider 10 seconds lapsed.
						}
						udp_protocol_set_waiting_answer(0);
						udp_protocol_status = UDP_PROT_IDLE;
//						Telit_socket_set_available(NOT_CONNECTED);
//						udp_protocol_status = UDP_PROT_IDLE;
//						Telit_socket_reconnect();
					}
					else if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
					{
						get_server_time = 0;
						Telit_socket_quick_close_and_shutdown();
						udp_protocol_status = UDP_PROT_IDLE;
					}
				}
				else if ( 1== udp_protocol_get_no_ack() )
				{
					if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
					{
						__processTimeoutWaitingResponse();

						if (MODBUS_SENSOR_SERVER_RESP == udp_server_resp_type)
						{
							modbus_sensors_log_measures_reset();
						}
					}
					else
					{
						leds_set_NET_status( LOW_POWER );
						uint32_t resp = SERVER_RESP;
						if ( SERVER_RESP == resp )
						{
							timeout_tries = 0;
							message_queue_delete( 1 );
							udp_protocol_set_waiting_answer(0);
							__processServerResp(udp_server_resp_type);
						}
						else if ( SERVER_FW_RESP == resp )
						{
							udp_protocol_set_waiting_answer(0);
							__processFWUpgradeResponse();
						}
						else if ( SERVER_RESET_DEV == resp )
						{
							udp_protocol_set_waiting_answer(0);
							params_check_for_changes();
							__disable_irq();
							NVIC_SystemReset();
						}
						else if ( CHECK_ELAPSED_TIME( time_udp_protocol, TICK_UDPPROT ) )
						{
							__processTimeoutWaitingResponse();
							if ( MODBUS_SENSOR_SERVER_RESP == udp_server_resp_type)
							{
								modbus_sensors_log_measures_reset();
							}
						}
					}
				}
				else if (comm_serial_rcx_bytes_n())
				{
					leds_set_NET_status(LOW_POWER);
					uint8_t resp = 0;
					int len = comm_serial_rcx_n((uint8_t *) str_rx, comm_serial_rcx_bytes_n());
//					LOGLIVE(LEVEL_1, "LOGLIVE> %u ME910>\r\nRX: %s.\r\n", (uint16_t)Tick_Get( SECONDS ), str_rx);

					if (PROTOCOL_UDP == params_protocol_mode())
					{
						resp = udp_protocol_process_response(str_rx, len);
					}
					else
					{
						resp = udp_protocol_rest_process_response(str_rx);
					}

					if (SERVER_RESP == resp)
					{
						timeout_tries = 0;
						message_queue_delete( 1 );
						udp_protocol_set_waiting_answer(0);
						__processServerResp(udp_server_resp_type);
					}
					else if (SERVER_FW_RESP == resp)
					{
						udp_protocol_set_waiting_answer(0);
						__processFWUpgradeResponse();
					}
					else if (SERVER_RESET_DEV == resp)
					{
						udp_protocol_set_waiting_answer(0);
						params_check_for_changes();
						__disable_irq();
						NVIC_SystemReset();
					}
					else if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
					{
						__processTimeoutWaitingResponse();

						if (MODBUS_SENSOR_SERVER_RESP == udp_server_resp_type)
						{
							if (generic_485_get_read_time() != 0)
							{
								modbus_sensors_log_measures_reset();
							}
						}
					}
				}
				else if (CHECK_ELAPSED_TIME(time_udp_protocol, TICK_UDPPROT))
				{
					__processTimeoutWaitingResponse();

					if (MODBUS_SENSOR_SERVER_RESP == udp_server_resp_type)
					{
						modbus_sensors_log_measures_reset();
					}
				}
			}
		break;

		case UDP_PROT_GET_FW_UPGRADE:
			if (1 == udp_is_socket_open())
			{
				msg_size = rest_send_get_fw_update(str_tx);
				if (msg_size != 0)
				{
					Telit_write_data(str_tx);
					Tick_update_tick(TICK_UDPPROT);
					time_udp_protocol   = 60;
					udp_protocol_set_waiting_answer(1);
					udp_protocol_status = UDP_PROT_WAITING_FOR_RESPONSE;
					fw_upgrading        = 1;
				}
				else
				{
					Telit_socket_quick_close_and_shutdown();
					udp_protocol_status = UDP_PROT_IDLE;
				}
			}
		break;

		case UDP_PROT_FW_UPDATING:
			if (0 == fw_update_FSM(str_rx, 0, &fw_length))
			{
				Telit_socket_quick_close_and_shutdown();
				fw_upgrading = 0;
				udp_protocol_status = UDP_PROT_IDLE;
			}
			else
			{
				Tick_update_tick(TICK_UDPPROT);
				time_udp_protocol = 1;
			}
		break;

		case UDP_PROT_GET_TIME:
			if (1 == udp_is_socket_open())
			{
				msg_size = rest_send_head(str_tx);
				if (msg_size != 0)
				{
					Telit_write_data(str_tx);
					Tick_update_tick(TICK_UDPPROT);
					time_udp_protocol = 60;
					udp_protocol_set_waiting_answer(1);
					udp_protocol_status = UDP_PROT_WAITING_FOR_RESPONSE;
					get_server_time = 1;
				}
				else
				{
					Telit_socket_quick_close_and_shutdown();
					udp_protocol_status = UDP_PROT_IDLE;
				}
			}
		break;

		case UDP_PROT_CLOSE:
			udp_protocol_status = UDP_PROT_IDLE;
		break;

		default:
		break;
	}
}


int __decodeMonth( char *str, int * month)
{
    if (0 == strcmp(str,"Jan")) {
    	*month = 1;
    }
    else if (0 == strcmp(str,"Feb")) {
    	*month = 2;
    }
    else if (0 == strcmp(str,"Mar")) {
    	*month = 3;
    }
    else if (0 == strcmp(str,"Apr")) {
    	*month = 4;
    }
    else if (0 == strcmp(str,"May")) {
    	*month = 5;
    }
    else if (0 == strcmp(str,"Jun")) {
    	*month = 6;
    }
    else if (0 == strcmp(str,"Jul")) {
    	*month = 7;
    }
    else if (0 == strcmp(str,"Aug")) {
    	*month = 8;
    }
    else if (0 == strcmp(str,"Sep")) {
    	*month = 9;
    }
    else if (0 == strcmp(str,"Oct")) {
    	*month = 10;
    }
    else if (0 == strcmp(str,"Nov")) {
    	*month = 11;
    }
    else if (0 == strcmp(str,"Dec")) {
    	*month = 12;
    }

    return *month;
}

time_t udp_protocol_get_date( char *date )
{
    struct tm fecha = { 0 };
    char *str;
    int month;
    // www.epochconverter.com

    str = strtok( date, " :" );
    //Date
    str = strtok( NULL, " :" );
    //Weekday
    str = strtok( NULL, " :" );
    fecha.tm_mday = atoi( str );
    str = strtok( NULL, " :" );

    //Month
    __decodeMonth(str, &month);
    fecha.tm_mon = month - 1;
    //Year
    str = strtok( NULL, " :" );
    fecha.tm_year = atoi( str ) - 1900;

    str = strtok( NULL, " :" );
    fecha.tm_hour = atoi( str );
    str = strtok( NULL, " :" );
    fecha.tm_min = atoi( str );
    str = strtok( NULL, " :" );
    fecha.tm_sec = atoi( str );

    return mktime( &fecha );
}

char * __findStringInResponse( char *str_header, char * str, size_t lenstr, uint32_t recv_bytes )
{
    return (char *) memmem( (char *)str_header, recv_bytes, str, lenstr );
}

char * __findStringInResponseHttp( char *str_header, char * str, size_t lenstr )
{
	uint32_t  n   = comm_serial_rcx_bytes_n();
	n = comm_serial_rcx_n( (uint8_t *)str_header, n );

    return (char *) memmem( (char *)str_header, n, str, lenstr );
}

uint8_t udp_protocol_process_response( char *ptr_header, uint32_t recv_bytes )
{
	uint8_t  ret = 0;
	uint8_t  r   = 0;
	uint32_t len = 0;

    static char ok_tok[]       = "{\"";
    static char end_json[]     = "}";
    static char error_tok[]    = "KO Process Error";

	char *body_start, *json_end;

    body_start = __findStringInResponseHttp(ptr_header, ok_tok, strlen(ok_tok));

    if ( body_start == NULL ) {
    	body_start = __findStringInResponseHttp(ptr_header, error_tok, strlen(error_tok));
    	if (body_start != NULL) {
    		ret = JSON_ERROR;
    	}
    } else { // 200 OK
    	json_end  = __findStringInResponseHttp(body_start, end_json, strlen(end_json));
    	if ( json_end != NULL ) {
    		len       = json_end + 1 - body_start;
    		r         = json_decode(body_start, len);
    		ret 	  = r;
    		comm_serial_delete(comm_serial_rcx_bytes_n());
    		comm_serial_rx_vars_init();
    	}
    }

	return ret;
}

#if 0
uint8_t udp_protocol_rest_process_response( char *ptr )
{
	uint8_t  ret = 0;
	uint8_t  r   = 0;
	uint32_t len = 0;

    static char end_body_tok[] = "\r\n\r\n";
    static char ok_tok[]       = "HTTP/1.1 200 OK\r\n";
    static char date[]         = "Date:";
    static char end_json[]     = "}";
    static char error_tok[]    = "KO Process Error";

    char *body_start, *body_end, *body_date, *json_end;

    body_start = __findStringInResponseHttp( ptr, ok_tok, strlen(ok_tok) );

    if ( body_start == NULL ) {
    	body_start = __findStringInResponseHttp(ptr, error_tok, strlen(error_tok));
    	if (body_start != NULL) {
    		ret = JSON_ERROR;
    	}
	} else { // 200 OK
		body_end = __findStringInResponseHttp(body_start, end_body_tok, strlen(end_body_tok));
		if ( body_end != NULL ) {
			body_date = __findStringInResponseHttp(body_start, date, strlen(date));
			if ( body_date != NULL ) {
				json_end  = __findStringInResponseHttp(body_start, end_json, strlen(end_json));
				if ( json_end != NULL ) {
//					if ( NETPARAMS_SERVER_RESP == udp_server_resp_type )
					{
						LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> \r\nRX(resp): %s.\r\n", (int)Tick_Get( SECONDS ), str_rx);
					}
					len       = json_end - body_end - 4 + 1;
					r         = json_decode(body_end + 4, len);
					comm_serial_delete(comm_serial_rcx_bytes_n());
					ret = r;
				}
			}
		}
	}

	return ret;
}
#endif

uint8_t udp_protocol_rest_process_response( char *ptr )
{
	uint8_t  ret = 0;
	uint8_t  r   = 0;
	uint32_t len = 0;

    static char end_body_tok[] = "\r\n\r\n";
    static char ok_tok[]       = "TTP/1.1 200 OK\r\n";
    static char date[]         = "Date:";
    static char end_json[]     = "}";
    static char error_tok[]    = "KO Process Error";
    static char content_length[] = "Content-Length: ";

    char *body_start, *body_end, *body_date, *json_end, *body_content_length, *params_start;
//    static uint32_t offset_json_end = 0;
    uint32_t body_len = 0;

    body_start = __findStringInResponseHttp( ptr, ok_tok, strlen(ok_tok) );

    if ( body_start == NULL ) {
    	body_start = __findStringInResponseHttp(ptr, error_tok, strlen(error_tok));
    	if (body_start != NULL) {
    		ret = JSON_ERROR;
    	}
	} else { // 200 OK
		body_end = __findStringInResponseHttp(body_start, end_body_tok, strlen(end_body_tok));
		if ( body_end != NULL ) {
			body_date = __findStringInResponseHttp(body_start, date, strlen(date));
			if ( body_date != NULL ) {
				body_content_length = __findStringInResponseHttp(body_start, content_length, strlen(content_length));
				if ( body_content_length != NULL ) {
					if (0 == body_len)
					{
						body_len     = atoi(body_content_length + strlen(content_length));
						params_start = body_end + 4;
					}
//					if (offset_json_end == 0)
//					{
//						json_end  = __findStringInResponseHttp(body_start, end_json, strlen(end_json));
//					}
//					else
//					{
//						json_end  = __findStringInResponse(body_start + offset_json_end, end_json, strlen(end_json), comm_serial_rcx_bytes_n());
//					}
					json_end  = __findStringInResponse(params_start + body_len - 1, end_json, strlen(end_json), 2);
//					LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> \r\nRX(resp): %s.\r\n", (int)Tick_Get( SECONDS ), params_start + body_len - 2);
					if ( json_end != NULL ) {
						uint32_t end_message = json_end - params_start + 1;
//						if (*(json_end + 1) == '\0')
						if (body_len == end_message)
						{
							{
								LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> \r\nRX(resp): %s.\r\n", (int)Tick_Get( SECONDS ), str_rx);
							}
//							offset_json_end = 0;
							len       = json_end - body_end - 4 + 1;
							r         = json_decode(body_end + 4, len);
							comm_serial_delete(comm_serial_rcx_bytes_n());
							comm_serial_rx_vars_init();
							ret = r;
						}
//						else
//						{
//							offset_json_end = json_end + 1 - body_start;
//						}
					}
				}
			}
		}
	}

	return ret;
}
uint8_t rest_get_fw_update_response( char *ptr_body, uint32_t *body_len )
{
	uint8_t  ret          = 0;
	uint32_t header_bytes = 0;

	static char end_body_tok[]   = "\r\n\r\n";
	static char ok_tok[]         = "TTP/1.1 200 OK\r\n";
    static char date[]           = "Date: ";
    static char content_length[] = "Content-Length: ";

    char *body_start, *body_date, *body_end, *body_content_length;

    body_start = __findStringInResponseHttp(ptr_body, ok_tok, strlen(ok_tok));

    if ( body_start == NULL ) {
    	asm("nop");
	} else { // 200 OK
		body_end = __findStringInResponseHttp(body_start, end_body_tok, strlen(end_body_tok));
		if ( body_end != NULL ) {
			header_bytes = body_end + 4 - body_start;
			body_date = __findStringInResponseHttp(body_start, date, strlen(date));
			if ( body_date != NULL ) {
				body_content_length = __findStringInResponseHttp(body_date, content_length, strlen(content_length));
				if ( body_content_length != NULL ) {
					*body_len = atoi(body_content_length + strlen(content_length));
					ptr_body = body_end + 4;
					ret = 1;
				}
				comm_serial_delete(header_bytes);
				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> \r\nRX(fw update): %s.\r\n", (int)Tick_Get( SECONDS ), str_rx);
			}
		}
	}
	return ret;
}

time_t rest_get_date( char *date )
{
    struct tm fecha = { 0 };
    char *str;
    int month;
    // www.epochconverter.com

    str = strtok( date, " :" );
    //Date
    str = strtok( NULL, " :" );
    //Weekday
    str = strtok( NULL, " :" );
    fecha.tm_mday = atoi( str );
    str = strtok( NULL, " :" );

    //Month
    __decodeMonth(str, &month);
    fecha.tm_mon = month - 1;
    //Year
    str = strtok( NULL, " :" );
    fecha.tm_year = atoi( str ) - 1900;

    str = strtok( NULL, " :" );
    fecha.tm_hour = atoi( str );
    str = strtok( NULL, " :" );
    fecha.tm_min = atoi( str );
    str = strtok( NULL, " :" );
    fecha.tm_sec = atoi( str );

//    rtc_localUTCTime((uint32_t *)&fecha.tm_hour);

    return mktime( &fecha );
}

uint8_t rest_time_response( char *ptr_body)
{
	uint8_t ret = 0;

	static char end_body_tok[] = "\r\n\r\n";
    static char date[]         = "Date:";
    char *body_date, *body_end;

    body_end = __findStringInResponseHttp(ptr_body, end_body_tok, strlen(end_body_tok));

    if ( body_end != NULL ) {
		body_date = __findStringInResponseHttp(ptr_body, date, strlen(date));

		if ( body_date != NULL ) {
			LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP> \r\nRX(server time): %s.\r\n", (int)Tick_Get( SECONDS ), ptr_body);
			Tick_system_time_init( rest_get_date(body_date) );
			rtc_system_SetServerTime( Tick_Get(SECONDS) );
			Tick_system_time_init(rtc_system_GetServerTime());// Update to Local Time.
			comm_serial_delete( comm_serial_rcx_bytes_n() );
			ret = 1;
		}
    }

    return ret;
}

#endif
