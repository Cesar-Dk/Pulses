/*
 * mqtt_frames.c
 *
 *  Created on: 3 feb. 2021
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE

#include "params.h"
#ifdef MQTT
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "mqtt_frames.h"
#include "mqtt_task.h"
#include "mqtt_buffer.h"
#include "mqtt_timer.h"
#include "message_queue.h"
#include "tick.h"
#include "leds.h"
#include "shutdown.h"
#include "datalogger_buffer.h"
#include "log.h"
#include "sensor_log.h"
#include "modbus_sensors.h"
#include "modbus_sensors_log.h"
#include "generic_modbus.h"
#include "json.h"
#include "fw_update.h"
#include "pulses.h"
#include "ad.h"
#include "battery.h"

typedef struct _MqttData_t {
	mqtt_server_resp wait_server_resp;
	uint32_t         send;
	uint32_t         rt_frame;
} MqttData_t;

MqttData_t mqtt_data = {
		.wait_server_resp = LAST_SERVER_RESP,
		.send             = 0,
};

char       fw_version[ 15 ];

uint32_t   rtt_ini, rtt_end, send_ok;
static uint32_t   timeout_tries = 0, start_datalogger = 0, retry_sensor_msg = 0, start_modbus_datalogger = 0, retry_modbus_msg = 0;

void mqtt_frames_set_wait_server_resp( mqtt_server_resp _mqtt_data )
{
	mqtt_data.wait_server_resp = _mqtt_data;
}

mqtt_server_resp mqtt_frames_get_wait_server_resp( void )
{
	return mqtt_data.wait_server_resp;
}

void mqtt_frames_set_send( uint32_t _send )
{
	mqtt_data.send = _send;
}

uint32_t mqtt_frames_get_send( void )
{
	return mqtt_data.send;
}

void mqtt_frames_set_rt_frame( uint32_t _rt )
{
	mqtt_data.rt_frame = _rt;
}

uint32_t mqtt_frames_get_rt_frame( void )
{
	return mqtt_data.rt_frame;
}

uint32_t mqtt_frames_get_send_ok( void )
{
	send_ok = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR20);
	return send_ok;
}

void mqtt_frames_set_send_ok( uint32_t ok )
{
	send_ok = ok;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR20, send_ok);
}

void mqtt_frames_inc_send_ok( void )
{
	send_ok++;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR20, send_ok);
}

void mqtt_frames_get_fw_version( char * _fw_version )
{
	strncpy(fw_version, _fw_version, 15);
}

uint32_t mqtt_frames_network_params_pubmsg( char *ptr_header )
{
	char     header[ 256 ];
	char	 message[ 1500 ];
	char	 date[ 20 ];
	char     read_intervals[120];
	char     send_intervals[120];
	char     read_sensor_intervals[120];
	char     send_sensor_intervals[120];
	char     send_params_intervals[120];
	char     read_value[8], send_value[8];
	char     pulse_params[20];
	char     modbus_params[480];
	char     modbus_commands[360];
	char     param_pressure_val[7];
	char     param_pressure_avg[7];
	char     param_pressure_max[7];
	char     param_pressure_min[7];
	uint32_t read_hours_len = 0, send_hours_len = 0, i;
	uint32_t measure_count = 0;

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
	memset( modbus_params, '0', 480 );
	modbus_params[0] ='\0';
	memset( modbus_commands, '0', 360 );
	modbus_commands[0] ='\0';

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
		sprintf(pulse_params, "%d;%d;%d;%d", (int)params_pulses_on(), (int)params_pulses_pulse_factor(), (int)params_pulses_pulse_unit_k_factor_out_1(), (int)params_pulses_pulse_unit_k_factor_out_2() );
	} else {
		sprintf(pulse_params, ";;;");
	}

	if ( params_get_modbus_type() != LAST_GENERIC_485_TYPE ) {
		sprintf(modbus_params, "%d;%d;%d;%d;%d;%s;%d;%d",
				(int)params_get_modbus_type(),
				(int)modbus_sensors_get_serial_config_baud_rate(),
				(int)modbus_sensors_get_serial_config_stop_bits(),
				(int)modbus_sensors_get_serial_config_parity(),
				(int)generic_485_get_slave_id(),
				modbus_commands,
				(int)generic_485_get_read_time(),
				(int)generic_485_get_send_time()
		);
	} else {
		sprintf(modbus_params, ";;;;;;;;;");
	}

	memset( message, 0, 512 );
	memset( header,  0, 256 );

	memcpy(param_pressure_val, sensor_log_get_pressure(),   7);
	memcpy(param_pressure_avg, sensor_log_GetPressureAvg(), 7);
	memcpy(param_pressure_min, sensor_log_GetPressureMin(), 7);
	memcpy(param_pressure_max, sensor_log_GetPressureMax(), 7);

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
			"%s;"														  //Modbus params.
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
			(int)shutdown_get_tamper_param(),      //1.
			//IMSI.
			ME910_IMSI(),//1.
			//Datalogger Info.4.
			(int)0,    //1
			Datalogger_Get_Info(),                  //2.
			sensor_log_GetDatalogger_Info(),        //3.
			modbus_sensors_log_GetDatalogger_Info(),//4.
			//Params.
			"",
			pulse_params,
			modbus_params,
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
			sensor_log_pressure_params(&measure_count)
	);
	shutdown_set_tamper_param(TAMPER_TSEND);

	sprintf(ptr_header,
			"%s",
			message);

	return strlen(ptr_header);
}

static uint32_t msg_modbus_num_total = 0;
uint32_t mqtt_frames_modbus_sensor_raw_pubmsg( char *ptr_header )
{
	size_t   str_len = 0;
	uint32_t measure_count = 0;
	uint32_t datalogger_threshold = 0;
	char    *str_log;

	rtt_ini = Tick_Get( MILLISECONDS );
	str_log = modbus_sensors_log_sensor_raw_telegram(&measure_count);
	if ( NULL == str_log ) {
		return 0;
	}
	if ( 0 == retry_modbus_msg ) {
		msg_modbus_num_total += measure_count;
	}
	datalogger_threshold = (uint32_t)((msg_modbus_num_total * generic_485_get_read_time())/(generic_485_get_send_time()));
	if ( datalogger_threshold >= 1 ) {
		start_modbus_datalogger = 1;
		msg_modbus_num_total    = 0;
	}
	retry_modbus_msg = 0;
	shutdown_set_tamper_param(TAMPER_TSEND);
	snprintf(ptr_header,
			strlen(str_log) + 30,
			"%s|MF|%s|%s",
			ME910_IMEI(),
			Telit_dev_identifier(),
			str_log
	);

	if ( 1 == mqtt_frames_get_rt_frame() ) {
		mqtt_frames_set_rt_frame(0);
		strcat(ptr_header,"RT|");
	}
	str_len = strlen(ptr_header);

	return str_len;
}

static uint32_t msg_sensor_num_total = 0;
uint32_t mqtt_frames_sensor_pubmsg( char *ptr_header )
{
	size_t   str_len = 0;
	uint32_t measure_count = 0;
	uint32_t datalogger_threshold = 0;
	char    *str_log;
	uint32_t sensor_id;

	rtt_ini = Tick_Get( MILLISECONDS );
	str_log = sensor_log_pressure_telegram( &measure_count, &sensor_id );
	if ( NULL == str_log ) {
		return 0;
	}
	if ( 0 == retry_sensor_msg ) {
		msg_sensor_num_total += measure_count;
	}
	datalogger_threshold = (uint32_t)((msg_sensor_num_total/2)/(params_config_send_time()/60));
	if ( datalogger_threshold >= 1 ) {
		start_datalogger = 1;
		msg_sensor_num_total    = 0;
	}
	retry_sensor_msg = 0;
	shutdown_set_tamper_param(TAMPER_TSEND);
	sprintf(ptr_header,
			"%s|S|%d|%d|6|5|"
			"%s",
			ME910_IMEI(),
			(int)sensor_id,
			(int)measure_count,
			str_log
	);

	str_len = strlen(ptr_header);

	return str_len;
}

/* GatewayIMEI|S+|idSensors|measuresCount|sensorsCount|valuelength|sensorsBits(xsensorsCount)|decimals(xsensorsCount)|timestep|datetime|Value1Value2...ValueN*/
/* 35308109022824|S+|0106|4|2|2|2|1006|20|15|20190910123524|1654se3f|*/
static uint32_t msg_sensor_plus_num_total = 0;
uint32_t mqtt_frames_sensor_plus_pubmsg( char *ptr_header, uint32_t num_sensors, uint32_t sensors )
{
	size_t   str_len = 0;
	uint32_t measure_count = 0, i, id_sensor[8], send_pressure = 0, send_pulses = 0;
	uint32_t datalogger_threshold = 0;
	char     str_sensors_id[5];
	char    *str_log;

	rtt_ini = Tick_Get( MILLISECONDS );

	for ( i = 0; i < num_sensors; i++ ) {
		id_sensor[i] = ( ( sensors >> (i*4) ) & 0x0F );
		if ( 1 == id_sensor[i] ) {
			send_pressure = 1;
		} else if ( 6 == id_sensor[i] ) {
			send_pulses = 1;
		}
	}

	/*str_log:datetime|Value1Value2...ValueN|*/
	str_log = sensor_log_pressure_instant_pulses_telegram( &measure_count, send_pulses, send_pressure );
	if ( NULL == str_log ) {
		return 0;
	}
	if ( 0 == retry_sensor_msg ) {
		msg_sensor_plus_num_total += measure_count;
	}
	datalogger_threshold = (uint32_t)((msg_sensor_plus_num_total/2)/(params_config_send_time()/60));
	if ( datalogger_threshold >= 1 ) {
		start_datalogger = 1;
		msg_sensor_plus_num_total    = 0;
	}
	retry_sensor_msg = 0;
	shutdown_set_tamper_param(TAMPER_TSEND);
	memset( str_sensors_id, 0, sizeof(str_sensors_id) );
	if ( 1 == send_pulses ) {
		snprintf(str_sensors_id, sizeof(str_sensors_id), "0106");
	} else {
		snprintf(str_sensors_id, sizeof(str_sensors_id), "01");
	}
	sprintf(ptr_header,
			"%s|S+|%s|%d|2|2|1006|20|%d|"
			"%s",
			ME910_IMEI(),
			str_sensors_id,
			(int)measure_count,
			(int)params_sensor_read_time_cycle(0),
			str_log
	);

	str_len = strlen(ptr_header);

	return str_len;
}

static void __send_more_packets( uint8_t sending )
{
#if defined (UNE82326)
	DataLogger_SetSend(sending);
	udp_protocol_set_send_pending(1);
#elif defined (MBUS)
	DataLogger_SendEnd();
	mqtt_task_set_send_pending(1);
#endif
	shutdown_reset_watchdog();
//	udp_protocol_status = UDP_PROT_IDLE;
}

void mqtt_task_reset_datalogger_max_msgs_params( void )
{
//	num_meter_frames  = 0;
//	num_frames        = 0;
//	num_modbus_frames = 0;
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

static void __processMessageServerResponse( void )
{
	rtt_end = Tick_Get(MILLISECONDS);
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	if ( 1 == datalog_check_all_mesgs_sent( ) ) {
#if defined(UNE82326)
		if ( ( ( 0 == Datalogger_CheckLIFOPointers() ) && ( 0 == datalog_buffer_n() ) )
			|| ( 1 == Datalogger_CheckLogMemIsEmpty() ) ) {
#elif defined(MBUS)
		if ( ( ( 0 == Datalogger_CheckLIFOPointers() ) && ( 0 == datalog_buffer_n() ) )
			|| ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() ) ) {
#endif
			params_attach_insert_log(0);
			mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
//			udp_protocol_status = UDP_PROT_CLOSE;
			DataLogger_ResetPointers();
			shutdown_set(1, params_config_read_time());
		} else {
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
		}
	} else {
		__send_more_packets(PARSING_NEW_FRAME);
	}
}

static void __processSensorServerResponse( void )
{
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	if ( ( 0 == sensor_log_CheckDataInMem() )
	  || ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() ) ) {
		datalog_buffer_poll();
		if ( message_queue_get_elements() ) {
//			udp_protocol_status = UDP_PROT_CONNECTED;
		} else {
			params_attach_insert_log(0);
			mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
			shutdown_set(1, params_config_read_time());
		}
		sensor_log_pressure_reset_telegram_values();
	} else {
//		mqtt_buffer_write( SEND_SENSOR );
		shutdown_reset_watchdog();
//		udp_protocol_status = UDP_PROT_CONNECTED;
	}

}

static void __throwNewMessage( void )
{
//	mqtt_buffer_write( SEND_MODBUS_SENSOR );
	shutdown_reset_watchdog();
//	udp_protocol_status = UDP_PROT_CONNECTED;
}

static void __checkNextModbusMessage( void )
{
	LOGLIVE(LEVEL_1, "Memory: %d,%s", (int)Tick_Get( SECONDS ), modbus_sensors_log_GetDatalogger_Info());
	if ( ( 0 == modbus_sensors_log_CheckDataInMem() )
	  || ( DATALOGGER_SEND_DISABLED == Datalogger_Get_Send_Disable() ) ) {
		datalog_buffer_poll();
		if ( message_queue_get_elements() && ( DATALOGGER_SEND_ENABLED == Datalogger_Get_Send_Disable() ) ) {
//			udp_protocol_status = UDP_PROT_CONNECTED;
			LOGLIVE(LEVEL_1, "message_queue num:%d time: %d", (int)message_queue_get_elements(), (int)Tick_Get( SECONDS ));
		} else {
			params_attach_insert_log(0);
			mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
//			if ( ( 0 == mqtt_get_send_activation() ) && ( 0 == mqtt_get_reply_get_topic() ) ) {
				shutdown_set(1, params_config_read_time());
//			}
		}
		modbus_sensors_log_params_reset_telegram_values_pointers();
	} else {
		__throwNewMessage();
	}
}

static void __processModbusSensorServerResponse( void )
{
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	__checkNextModbusMessage();
}

static void __processNetParamsServerResponse( void )
{
	rtt_end = Tick_Get( MILLISECONDS );
	ME910_network_params_set_rtt_mean( rtt_end - rtt_ini );
	params_maintenance_inc_number_ok_sendings();
	params_check_for_changes();
	datalog_buffer_poll();
	if ( message_queue_get_elements() ) {
//		udp_protocol_status = UDP_PROT_CONNECTED;
	} else {
		LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_FRAMES>p_frame response.\r\n", (int)Tick_Get( SECONDS ));
		params_attach_insert_log(0);
		mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
		if ( MODBUS_COMM_STARTED == modbus_sensors_get_comm_state() ) {//Modbus read pending

		} else {
			shutdown_set(1, params_config_read_time());
		}
	}
}

void mqtt_frames_processTimeoutWaitingResponse( void )
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_FRAMES>Timeout waiting response!!!!!\r\n", (int)Tick_Get( SECONDS ));
	params_maintenance_inc_number_nok_sendings();
	params_attach_error_inc_TSE_error();
	params_attach_insert_log(TSE_ERR);
	params_check_for_changes();
	if ( MESSAGE_SERVER_RESP == mqtt_frames_get_wait_server_resp()) {
#if defined(UNE82326)
		datalog_recover_sent_fail_frame();
#elif defined(MBUS)
		Datalogger_RecoverSendFailRecord();
		__send_more_packets(READING_FRAMES);
#endif
	} else if ( SENSOR_SERVER_RESP == mqtt_frames_get_wait_server_resp()) {
		sensor_log_recover_rd_address_backup();
	} else if ( MODBUS_SENSOR_SERVER_RESP == mqtt_frames_get_wait_server_resp()) {
		modbus_sensors_log_recover_rd_address_backup();
		__throwNewMessage();
	}
	shutdown_reset_watchdog();
	leds_set_NET_status( NB_MODULE_ERROR );
//	Telit_socket_set_available( NOT_CONNECTED );
	if ( timeout_tries++ < params_get_retries_server() ) {
//		Telit_socket_reconnect();
		mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
	} else {
		shutdown_set(1, params_config_read_time());
	}
}

void mqtt_frames_restore_pointers( void )
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_FRAMES> Restore pointers: %s \r\n", (int)Tick_Get( SECONDS ), modbus_sensors_log_GetDatalogger_Info());
	if ( MESSAGE_SERVER_RESP == mqtt_frames_get_wait_server_resp()) {
#if defined(UNE82326)
		datalog_recover_sent_fail_frame();
#elif defined(MBUS)
		Datalogger_RecoverSendFailRecord();
#endif
	} else if ( SENSOR_SERVER_RESP == mqtt_frames_get_wait_server_resp()) {
		sensor_log_recover_rd_address_backup();
	} else if ( MODBUS_SENSOR_SERVER_RESP == mqtt_frames_get_wait_server_resp()) {
		modbus_sensors_log_recover_rd_address_backup();
	}
}

void mqtt_frames_processNextMsg( uint32_t resp )
{
	mqtt_frames_inc_send_ok();
	switch(resp) {
	case MESSAGE_SERVER_RESP:
		__processMessageServerResponse();
		break;
	case SENSOR_SERVER_RESP:
		__processSensorServerResponse();
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

#endif
