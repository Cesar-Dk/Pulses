//
// --------------------------------------------------------------------------
//  Gurux Ltd
//
//
//
// Filename:        $HeadURL:  $
//
// Version:         $Revision:  $,
//                  $Date:  $
//                  $Author: $
//
// Copyright (c) Gurux Ltd
//
//---------------------------------------------------------------------------

#ifndef MEDIA_H
#define MEDIA_H

#include <stdlib.h> // malloc and free needs this or error is generated.
#include <stdio.h>
#include <string.h> // string function definitions
#include <time.h>   // time calls

#include "bytebuffer.h"
#include "dlmssettings.h"

#define MAX_RETRIES             (3)

#define LOAD_PROFILE_OBIS 		(15)
#define BILLING_PROFILE_OBIS 	(16)
#define EVENT_PROFILE_OBIS 		(17)
#define ID_OBIS 				(18)

static const unsigned int RECEIVE_BUFFER_SIZE = 200;

#ifdef  __cplusplus
extern "C" {
#endif

typedef enum tagDLSMstate
{
	DLMS_STATE_IDLE = 0,
	DLMS_STATE_INIT = 1,
	DLMS_STATE_SNRM,
	DLMS_STATE_AARQ,
	DLMS_STATE_GET_CHALLENGE,
	DLMS_STATE_CHECK_METER_TIME,
	DLMS_STATE_GET_METER_ID,
	DLMS_STATE_OBIS_1,
	DLMS_STATE_DATA,
	DLMS_STATE_EVENT_DATA,
	DLMS_DISCONNECT_CONTROL,
	DLMS_STATE_LOAD_PROFILE,
	DLMS_STATE_METER_SYNCH,
	DLMS_STATE_ON_DEMAND_COMMAND,
	DLMS_STATE_BILLING_TUNNELING,

	DLMS_STATE_DISCONNECT = 254,
	DLMS_STATE_ERROR = 255
} dlms_state_t;

typedef struct
{
	struct io_descriptor *io;
	unsigned short waitTime;
	//Is trace used.
	GX_TRACE_LEVEL trace;
	//Received data.
	gxByteBuffer data;
	dlmsSettings settings;
	dlms_state_t state; //added RM to be used in the FSM
} connection;

char   * con_dlms_get_device_from_cmd(uint32_t index);
void     con_dlms_set_end_of_data_from_cmd(uint32_t __end);
uint32_t con_dlms_get_next_device_for_cmd(uint32_t index);
uint32_t con_dlms_get_curr_id_device_from_cmd_table(uint32_t index);

uint32_t con_dlms_get_curr_device( void );
void     con_dlms_set_curr_device( uint32_t _curr_dev );
void     con_dlms_inc_curr_device( void );
void     con_dlms_set_release_curr_com(uint32_t _con_release_curr_com);
uint32_t con_dlms_release_curr_com(connection* con);

int con_initializeBuffers(
		connection* connection,
		int size);

connection *con_dlms_get_con_singleton(void);

//Initialize connection settings.
void con_init(
		connection* con,
		GX_TRACE_LEVEL trace);

//Close connection..
void con_close(connection* con);
uint32_t con_get_num_items_device(void);
uint32_t con_dec_num_items_device(void);

uint32_t con_dlms_get_meter_clk_obis_value(void);
uint32_t con_dlms_get_format_myd(void);
uint32_t con_dlms_getDateFormat( uint32_t *__meter_time );
uint32_t con_dlms_setDateFormat( uint32_t *__meter_time );
int      con_dlms_Task(connection* con);

void         con_dlms_change_state(connection* con, dlms_state_t state);
dlms_state_t con_dlms_get_state(connection* con);
uint32_t     con_dlms_in_process( void );

struct tm *  con_convert_time( time_t  timer);
uint16_t     con_get_shortName( uint32_t num_obis );
uint16_t     con_get_generic_profile_shortName( void );


#ifdef  __cplusplus
}
#endif

#endif //MEDIA_H
