/**
  ******************************************************************************
  * @file           json.h
  * @author 		Datakorum Development Team
  * @brief          Header file for json.c file.
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
 * json.h
 *
 *  Created on: 15 may. 2018
 *      Author: Sergio Mill�n L�pez
 */

#ifndef JSON_H_
#define JSON_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
#include <stdint.h>

#include "jsmn.h"
#include "params.h"
#include "tick.h"
#include "datalogger_buffer.h"
#include "log.h"
#include "rtc_system.h"
#include "ME910.h"
#include "udp_protocol.h"

#define CLEAR_MEMORY_DELETE_DATALOGGER   (1)
#define CLEAR_MEMORY_DISABLE_DATALOGGER  (2)
#define CLEAR_MEMORY_FORCE_HARD_RESET    (3)
#define CLEAR_MEMORY_DISABLE_HARD_RESET  (4)

char   * json_read_hours( void );

uint32_t json_decode( char *str, uint32_t len );
#ifdef MQTT
void     json_mqtt_decode( uint8_t type, char *str, uint32_t len );
uint32_t json_mqtt_encode( uint8_t type, char *str );
#endif
uint32_t json_modbus_decode( char *str, uint32_t len );

int32_t  json_search_section( int32_t tk_i, char *name, char *str );
int32_t  json_search_next_section( int32_t tk_i, char *name, char *str, int32_t *t );
int32_t  json_get_int( int32_t tk_i, char *str );
float    json_get_float( int32_t tk_i, char *str );
void     json_get_str( int32_t tk_i, char *src, char *dst );
void     json_get_mbus_add( char *str, uint32_t clients_num );
time_t   json_get_date( char *date );
time_t   json_get_hour( char *date );
void     json_get_psm( char *str );
void     json_decode_edrx( char *str );
void     json_get_read_hours( uint8_t num, char *str );
void     json_get_sensor_read_hours( uint8_t num, char *str );
void     json_get_modbus_read_hours( uint8_t num, char *str );
void 	 json_get_send_hours( char *str );
void     json_get_sensor_send_hours( char *str );
void     json_get_modbus_send_hours( char *str );
void     json_get_send_network_params_hours( char *str );
void     json_get_sensor_over_sensor_alarms( char *str );
void     json_get_sensor_low_sensor_alarms( char *str );
void     json_get_i2c_address( char *str );

void     json_get_modbus_slave_id( char *str, uint32_t j );
void     json_get_modbus_address_645( char *str, uint32_t j );
void     json_get_modbus_function( char *str, uint32_t j );
void     json_get_modbus_address( char *str, uint32_t j );
void     json_get_modbus_quantity( char *str, uint32_t j );
char   * json_get_function_str(void);
char   * json_get_address_str(void);
char   * json_get_quantity_str(void);
void     json_get_modbus_value( char *str, uint32_t j );
void     json_set_date2str( time_t hora_UTCs, char *str );

void     json_get_dlms_address( char *str, uint32_t clients_num );
void     json_get_dlms_connection_config( char *str, uint32_t clients_num );
void     json_get_dlms_profiles_config( char *str, uint32_t clients_num );

uint32_t json_encode( uint8_t type, char *str );
void     json_set_weekdays2str( uint8_t i, char *str );
void     json_set_time2str( uint8_t i, uint8_t j, char *str );
void     json_set_date2str( time_t hora_UTCs, char *str );

#endif /* JSON_H_ */
