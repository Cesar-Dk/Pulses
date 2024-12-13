/*
 * mqtt_buffer.c
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
#include "mqtt_task.h"
#include "mqtt_buffer.h"

// PUBLISH BUFFER //////////////////////////////////////////////////////////////

#define MQTT_PUB_BUFFER_SIZE 10
#if 1
struct
{
	uint8_t data[ MQTT_PUB_BUFFER_SIZE ];
	uint8_t n;
	uint8_t start;
	uint8_t end;
} mqtt_buffer;

void mqtt_buffer_init( void )
{
    memset( &mqtt_buffer, 0, sizeof( mqtt_buffer ));
}

uint8_t mqtt_buffer_get_elements( void )
{
    return mqtt_buffer.n;
}

void mqtt_buffer_write( uint8_t x )
{
    mqtt_buffer.data[ mqtt_buffer.end ] = x;
    if( mqtt_buffer.end < (MQTT_PUB_BUFFER_SIZE-1) ) mqtt_buffer.end++;
    else mqtt_buffer.end = 0;
    if( mqtt_buffer.end == mqtt_buffer.start )
    {
        if( mqtt_buffer.start < (MQTT_PUB_BUFFER_SIZE-1) ) mqtt_buffer.start++;
        else mqtt_buffer.start = 0;
    }
    else mqtt_buffer.n++;
}

uint8_t mqtt_buffer_read( void )
{
    return mqtt_buffer.data[ mqtt_buffer.start ];
}

void mqtt_buffer_delete( uint8_t n )
{
	uint8_t x;

    if( ! mqtt_buffer.n ) return;

//    if( mqtt_buffer_read() == PT_LOG_DATALOGGER ) Datalogger_UpdateReadBufferPointer();

    x = mqtt_buffer.start + n;
    if( x > ( MQTT_PUB_BUFFER_SIZE - 1 )) mqtt_buffer.start = x - MQTT_PUB_BUFFER_SIZE;
    else mqtt_buffer.start = x;

    mqtt_buffer.n -= n;
}
#endif
// ACKs BUFFER /////////////////////////////////////////////////////////////////

struct
{
	uint8_t type[ MQTT_PUB_BUFFER_SIZE ];
    unsigned short id[ MQTT_PUB_BUFFER_SIZE ];
    uint8_t n;
    uint8_t start;
    uint8_t end;
} mqtt_ACKs_buffer;

void mqtt_buffer_ACKs_init()
{
    memset( &mqtt_ACKs_buffer, 0, sizeof( mqtt_ACKs_buffer ));
}

uint8_t mqtt_buffer_ACKs_get_elements( )
{
    return mqtt_ACKs_buffer.n;
}

void mqtt_buffer_ACKs_write( uint8_t type, unsigned short id )
{
    mqtt_ACKs_buffer.type[ mqtt_ACKs_buffer.end ] = type;
    mqtt_ACKs_buffer.id[ mqtt_ACKs_buffer.end ] = id;
    if( mqtt_ACKs_buffer.end < (MQTT_PUB_BUFFER_SIZE-1) ) mqtt_ACKs_buffer.end++;
    else mqtt_ACKs_buffer.end = 0;
    if( mqtt_ACKs_buffer.end == mqtt_ACKs_buffer.start )
    {
        if( mqtt_ACKs_buffer.start < (MQTT_PUB_BUFFER_SIZE-1) ) mqtt_ACKs_buffer.start++;
        else mqtt_ACKs_buffer.start = 0;
    }
    else mqtt_ACKs_buffer.n++;
}

uint8_t mqtt_buffer_ACKs_read( unsigned short *id )
{
    *id = mqtt_ACKs_buffer.id[ mqtt_ACKs_buffer.start ];
    return mqtt_ACKs_buffer.type[ mqtt_ACKs_buffer.start ];
}

void mqtt_buffer_ACKs_delete( uint8_t n )
{
	uint8_t x;

    x = mqtt_ACKs_buffer.start + n;
    if( x > (MQTT_PUB_BUFFER_SIZE-1) ) mqtt_ACKs_buffer.start = x - MQTT_PUB_BUFFER_SIZE;
    else mqtt_ACKs_buffer.start = x;

    mqtt_ACKs_buffer.n -= n;
}

#endif

