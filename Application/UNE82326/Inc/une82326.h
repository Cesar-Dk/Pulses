/*
 * une82326.h
 *
 *  Created on: 18 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_UNE82326_INC_UNE82326_H_
#define APPLICATION_UNE82326_INC_UNE82326_H_

#include "stm32u5xx_hal.h"
#include "params.h"

#ifdef UNE82326

typedef enum {
	NO_FRAME = 0,
	A_PLUS_FRAME,
	WATER_METER_GATHERING_VALUES,
	WATER_METER_FRAME,
	DEVICE_NETWORK_PARAMETERS_FRAME,
	LAST_TYPE_FRAME = 0xFF
}frame_type;

typedef struct _une82326_st
{
	uint8_t    start_comm;
	uint8_t    end_comm;
	uint8_t    last_device;
	uint8_t    write_record;
	frame_type type_frame;
}Une82326;

void 	une82326_init( void );
void    une82326_reset( void );
uint8_t une82326_get_start_comm( void );
void    une82326_set_start_comm( uint8_t _start_comm );
uint8_t une82326_get_end_comm( void );
void    une82326_set_end_comm( uint8_t _end_comm );
uint8_t une82326_get_last_device( void );
void    une82326_set_last_device( uint8_t _last_device );
uint8_t une82326_get_write_record( void );
void    une82326_set_write_record( uint8_t _write_record );
uint8_t une82326_get_frame_type( void );
void    une82326_set_frame_type( uint8_t _frame_type );
void    une82326_task( void );

#endif
#endif /* APPLICATION_UNE82326_INC_UNE82326_H_ */
