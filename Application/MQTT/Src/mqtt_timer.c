/*
 * mqtt_timer.c
 *
 *  Created on: 1 feb. 2021
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE

#include "params.h"
#ifdef MQTT
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "mqtt_timer.h"
#include "mqtt_task.h"
#include "tick.h"

uint32_t idle_timer;
uint32_t mqtt_idle_mode = 1;
struct
{
	uint32_t subs, pub, ack, keepalive, ping, timeout_server, update_ready, idle;
} t;

void mqtt_timer_init()
{
    mqtt_timer_reset( T_SUBS );
    mqtt_timer_reset( T_PUB );
    mqtt_timer_reset( T_ACK );
    mqtt_timer_set(   T_KEEPALIVE );
    mqtt_timer_reset( T_PING );
    mqtt_timer_set(   T_TIMEOUT_SERVER );
    mqtt_timer_reset( T_UPDATE_READY );
    mqtt_timer_reset( T_IDLE );
}

void mqtt_timer_load_idle_timer( uint32_t _idle_timer )
{
	idle_timer = _idle_timer;
}

void mqtt_timer_set_idle_mode( uint32_t _idle_mode )
{
	mqtt_idle_mode = _idle_mode;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TIMER> MQTT Idle Mode:%d\r\n", (int)Tick_Get( SECONDS ), (int)mqtt_idle_mode);
}

uint32_t mqtt_timer_get_idle_mode( void )
{
	return mqtt_idle_mode;
}

void mqtt_timer_set( t_type i )
{
    switch( i )
    {
        case T_SUBS:
            t.subs = HAL_GetTick() / TICKS_PER_SECOND + 10;
            break;
        case T_PUB:
            t.pub = HAL_GetTick() / TICKS_PER_SECOND + 10;
            break;
        case T_ACK:
            t.ack = HAL_GetTick() / TICKS_PER_SECOND + 10;
            break;
        case T_KEEPALIVE:
            t.keepalive = HAL_GetTick() / TICKS_PER_SECOND + param.server[ (uint8_t)mqtt_server_get_index() ].keepalive;
            break;
        case T_PING:
            t.ping = HAL_GetTick() / TICKS_PER_SECOND + 5;
            break;
        case T_TIMEOUT_SERVER:
            t.timeout_server = HAL_GetTick() / TICKS_PER_SECOND + params_get_timeout_server();
            break;
        case T_UPDATE_READY:
            t.update_ready = HAL_GetTick() / TICKS_PER_SECOND + 5;
            break;
        case T_IDLE:
            t.idle = HAL_GetTick() / TICKS_PER_SECOND + idle_timer;
            break;
        default:
            break;
    }
}

void mqtt_timer_reset( t_type i )
{
    switch( i )
    {
        case T_SUBS:
            t.subs = 0;
            break;
        case T_PUB:
            t.pub = 0;
            break;
        case T_ACK:
            t.ack = 0;
            break;
        case T_KEEPALIVE:
            t.keepalive = 0;
            break;
        case T_PING:
            t.ping = 0;
            break;
        case T_TIMEOUT_SERVER:
            t.timeout_server = 0;
            break;
        case T_UPDATE_READY:
            t.update_ready = 0;
            break;
        case T_IDLE:
            t.idle = 0;
            break;
        default:
            break;
    }
}

uint32_t mqtt_timer_get( t_type i )
{
	uint32_t tn = HAL_GetTick() / TICKS_PER_SECOND;
	static uint32_t print_count = 0;
    switch( i )
    {
        case T_SUBS:
            if( t.subs > tn ) {
            	tn = t.subs - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_PUB:
            if( t.pub > tn ) {
            	tn = t.pub - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_ACK:
            if( t.ack > tn ) {
            	tn = t.ack - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_KEEPALIVE:
            if( t.keepalive > tn ) {
            	tn = t.keepalive - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_PING:
            if( t.ping > tn ) {
            	tn = t.ping - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_TIMEOUT_SERVER:
            if( t.timeout_server > tn ) {
            	tn = t.timeout_server - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_UPDATE_READY:
            if( t.update_ready > tn ) {
            	tn = t.update_ready - tn;
            }
            else {
            	tn = 0;
            }
            break;
        case T_IDLE:
            if( t.idle > tn ) {
            	tn = t.idle - tn;
            }
            else {
            	tn = 0;
            }
            if ( print_count++ >= 100000  ) {//45 secs waiting.//500000
            	print_count = 0;
            	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TIMER> T_IDLE:%d.\r\n", (int)Tick_Get( SECONDS ), (int)tn);
            }
            break;
        default:
            tn = 0;
            break;
    }

    return tn;
}

#endif

