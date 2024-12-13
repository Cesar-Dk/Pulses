/*
 * mqtt_timer.h
 *
 *  Created on: 1 feb. 2021
 *      Author: Sergio Millán López
 */

#ifndef MQTT_INC_MQTT_TIMER_H_
#define MQTT_INC_MQTT_TIMER_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "mqtt_task.h"

typedef enum {
	T_SUBS,
	T_PUB,
	T_ACK,
	T_KEEPALIVE,
	T_PING,
	T_TIMEOUT_SERVER,
	T_UPDATE_READY,
	T_IDLE
} t_type;

// Timer functions
void     mqtt_timer_init( void );
void     mqtt_timer_load_idle_timer( uint32_t _idle_timer );
void     mqtt_timer_set_idle_mode( uint32_t _idle_mode );
uint32_t mqtt_timer_get_idle_mode( void );
void     mqtt_timer_set( t_type i );
void     mqtt_timer_reset( t_type i );
uint32_t mqtt_timer_get(  t_type i );

#endif /* MQTT_INC_MQTT_TIMER_H_ */
