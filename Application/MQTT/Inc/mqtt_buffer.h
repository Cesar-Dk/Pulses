/*
 * mqtt_buffer.h
 *
 *  Created on: 1 feb. 2021
 *      Author: Sergio Millán López
 */

#ifndef MQTT_INC_MQTT_BUFFER_H_
#define MQTT_INC_MQTT_BUFFER_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

#if 1
// buffer functions
void    mqtt_buffer_init( void );
uint8_t mqtt_buffer_read( void );
void    mqtt_buffer_delete( uint8_t n );
uint8_t mqtt_buffer_get_elements( void );
void    mqtt_buffer_write( uint8_t x );
#endif
// ACKs buffer functions
void    mqtt_buffer_ACKs_init( void );
uint8_t mqtt_buffer_ACKs_get_elements( void );
void    mqtt_buffer_ACKs_write( uint8_t type, uint16_t id );
uint8_t mqtt_buffer_ACKs_read( uint16_t *id );
void    mqtt_buffer_ACKs_delete( uint8_t n );

#endif /* MQTT_INC_MQTT_BUFFER_H_ */
