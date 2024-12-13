/*
 * tick.h
 *
 *  Created on: 15 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_USER_COMMON_INC_TICK_H_
#define APPLICATION_USER_COMMON_INC_TICK_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "rtc_system.h"
#include "params.h"

typedef enum {
	MILLISECONDS = 0,
	SECONDS,
	LAST = 0xFF,
} time_unit;

#define TICK_SECS_PER_HOUR        ( 3600 )
#define TICK_SECS_PER_MINUTE      ( 60 )

#define TICKS_PER_SECOND 		  ( 1000 )

#define TICK_TELIT 		0
#define TICK_REST  		1
#define TICK_FW    		2
#define TICK_UDP   		3
#define TICK_UDPPROT   	4
#if defined(UNE82326)
#define TICK_UNE82326   5
#elif defined(MBUS)
#define TICK_MBUS       5
#endif
#define TICK_AD         6
#ifdef MODBUS
#define TICK_MODBUS     7
#endif
#define TICK_LAST  		8

#define CHECK_ELAPSED_TIME( t, i )    ((( HAL_GetTick() ) - ( Tick_get_tick( i ) )) > (( t ) * ( TICKS_PER_SECOND )))
#define CHECK_ELAPSED_MILISEC( t, i ) ((( HAL_GetTick() ) - ( Tick_get_tick( i ) )) > (( t )))

uint32_t Tick_Second( void );
uint32_t Tick_Force_1Second_Task( void );
uint32_t Tick_100Miliseconds( void );
uint32_t Tick_10Miliseconds( void );
uint32_t Tick_1Miliseconds( void );
uint32_t Tick_get_tick_init( void );
void	 Tick_init( void );
uint32_t TickGet( void );
uint32_t TickGetSeconds( void );
uint32_t Tick_Get( time_unit time_unit);

void     Tick_system_time_init( uint32_t s );
uint32_t Tick_delay_ms(uint32_t ms);
void     Tick_update_tick( uint8_t i );
uint32_t Tick_get_tick( uint8_t i );
uint8_t  Tick_cloud_time_init( void );
time_t   Tick_get_date( char *date );
#endif /* APPLICATION_USER_COMMON_INC_TICK_H_ */
