/*
 * mqtt_frames.h
 *
 *  Created on: 3 feb. 2021
 *      Author: Sergio Millán López
 */

#ifndef MQTT_INC_MQTT_FRAMES_H_
#define MQTT_INC_MQTT_FRAMES_H_


#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

#include "params.h"

typedef enum {
	MESSAGE_SERVER_RESP = 1,
	SENSOR_SERVER_RESP,
	NETPARAMS_SERVER_RESP,
	NETWORK_SERVER_TIME_RESP,
	MODBUS_SENSOR_SERVER_RESP,
	LAST_SERVER_RESP = 0xFF,
} mqtt_server_resp;

void             mqtt_frames_set_wait_server_resp( mqtt_server_resp _mqtt_data );
mqtt_server_resp mqtt_frames_get_wait_server_resp( void );
void             mqtt_frames_set_send( uint32_t _send );
uint32_t         mqtt_frames_get_send( void );
void             mqtt_frames_set_rt_frame( uint32_t _rt );
uint32_t         mqtt_frames_get_rt_frame( void );
void             mqtt_frames_get_fw_version( char * _fw_version );

uint32_t mqtt_frames_network_params_pubmsg( char *ptr_header );
uint32_t mqtt_frames_modbus_sensor_raw_pubmsg( char *ptr_header );
uint32_t mqtt_frames_sensor_pubmsg( char *ptr_header );
uint32_t mqtt_frames_sensor_plus_pubmsg( char *ptr_header, uint32_t num_sensors, uint32_t sensors );

void     mqtt_task_reset_datalogger_max_msgs_params( void );
void     mqtt_frames_restore_pointers( void );
void     mqtt_frames_processNextMsg( uint32_t resp );
void     mqtt_frames_processTimeoutWaitingResponse( void );

#endif /* MQTT_INC_MQTT_FRAMES_H_ */
