/*
 * dlms_client.c
 *
 *  Created on: 20 may. 2021
 *      Author: Sergio Millán López
 */
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <ctype.h>
#include "common_lib.h"
#include "circular_buffer.h"
#include "tick.h"
#include "generic_modbus.h"
#include "shutdown.h"

#include "dlms_client.h"
#include "dlms_client_table.h"
#include "dlms_log.h"

#define OBIS_CAPTURE_NUM_MAX (42)

#define DLMS_CLIENT_MSG_BUFF_SIZE (400)

CircularBufferStruct dlms_client_read_msg_queue;
CircularBufferStruct dlms_client_write_msg_queue;
CircularBufferStruct dlms_client_send_msg_queue;
read_task_t  		 dlms_client_read_msg_queue_data[DLMS_CLIENT_MSG_BUFF_SIZE];
write_task_t 		 dlms_client_write_msg_queue_data[DLMS_CLIENT_MSG_BUFF_SIZE];
send_task_t 		 dlms_client_send_msg_queue_data[DLMS_CLIENT_MSG_BUFF_SIZE];

struct __dlms_client_t
{
	uint8_t  enable;
	uint32_t num_obis_profiles;
	uint32_t num_generic_profiles;
	uint32_t read_time[DLMS_PROFILE_NUM];
	uint32_t send_time[DLMS_PROFILE_NUM];
//	char     id_request[40];
	char     auth_pass[20];
	uint32_t pass_is_hex;
	uint32_t pass_level;
	uint32_t logical_address;
	uint16_t cl_address;
	uint32_t srv_address;
	uint32_t baudrate;
	uint32_t shortNameRef;
	uint32_t billingday;
	uint32_t num_devices;
//	char     dlms_meter_id[32];
};

dlms_client_t dlms_client, dlms_client_backup;

struct __dlms_client_obis_profile_t
{
	uint32_t read_time;
	uint32_t send_time;
	uint32_t frame_type_obis_profile;
	uint32_t num_obis;
	char     dlms_obis[20][24];
	uint32_t obis_from_command;
	uint32_t max_demand_profile_extra;
};

dlms_client_obis_profile_t dlms_client_obis_profile;
char     dlms_obis_values[20][64];

typedef struct __dlms_client_vars
{
	uint32_t comm_state;
	uint32_t read_time;
	uint32_t send_time;
	uint32_t read_meter;
	uint32_t send_meter;
	uint32_t param_change;
	uint32_t resend;
	uint32_t disconnected;
	uint32_t command;
	uint32_t synch_time;
	uint32_t read_energy_profile;
	write_task_t frame_type_last_write;
	uint32_t next_dlms_obis_value;
	uint32_t next_dlms_obis_value_rd;
	uint32_t id_request_index;
	uint32_t id_request_index_rd;
	char     id_request[10][40];
	char     dlms_meter_id[30][32];
	uint32_t inst_profile_1;
	uint32_t inst_profile_2;
	uint32_t inst_profile_3;
	uint32_t inst_profile_4;
	uint32_t energy_profile_1;
	uint32_t energy_profile_2;
	uint32_t energy_profile_3;
	uint32_t load_profile_1;
	uint32_t load_profile_2;
	uint32_t dlms_next_obis_capture_object;
	uint32_t dlms_reading_items[30];
	char     dlms_data_cmd_on_demand[400];
}dlms_client_vars_t;

dlms_client_vars_t dlms_client_vars;

struct __dlms_client_generic_profile_t
{
	uint32_t read_time;
	uint32_t send_time;
	uint32_t frame_type_generic_profile;
	uint32_t num_obis;
//	uint32_t dlms_next_obis_capture_object;
	char     dlms_obis_generic_profile[1][24];
	uint32_t next_dlms_obis_generic_profile_value_rd;
	char     dlms_obis_capture_objects[OBIS_CAPTURE_NUM_MAX][24];
	uint32_t message_length;
	uint32_t start_time;
	uint32_t end_time;
	uint32_t wait_for_comm;
	uint32_t profile_from_command;
};

dlms_client_generic_profile_t dlms_generic_profile;
char dlms_checksum_param[33] = "abcdefg1234500000011111111111111\0";//b3217606c32a543d40cbf9b3dab0a4fc

#define NUM_BLOCKS (712)
typedef struct _blockentry
{
   void * addr;
   size_t size;
} BlockEntry;
static BlockEntry blocks[NUM_BLOCKS];

static long	G_inUse 		   = 0;
static int  num_blocks		   = 0;
static long FS_totalAllocated  = 0;

uint32_t dlms_client_init(void)
{
	dlms_log_init();

	CircularBuffer_Create((CircularBuffer)&dlms_client_read_msg_queue,  DLMS_CLIENT_MSG_BUFF_SIZE, (uint8_t *)dlms_client_read_msg_queue_data);
	CircularBuffer_Create((CircularBuffer)&dlms_client_write_msg_queue, DLMS_CLIENT_MSG_BUFF_SIZE, (uint8_t *)dlms_client_write_msg_queue_data);
	CircularBuffer_Create((CircularBuffer)&dlms_client_send_msg_queue,  DLMS_CLIENT_MSG_BUFF_SIZE, (uint8_t *)dlms_client_send_msg_queue_data);
	dlms_client_table_read_client_id( dlms_client_get_client(), 0 );
	if ((dlms_client.enable != 0) && (dlms_client.enable != 1))
	{
		memset(&dlms_client,0,sizeof(dlms_client));
		dlms_client_table_write_client_id( dlms_client_get_client(), 0 );
	}
	shutdown_init_dlms_count();
	dlms_client_vars.command = DLMS_HES_COMMAND_NONE;
	dlms_client_vars.resend  = 0;
	dlms_client_vars.id_request_index    = 0;
	dlms_client_vars.id_request_index_rd = 0;
	return 0;
}

CircularBuffer dlms_client_get_read_msg_queue( void)
{
	return (CircularBuffer)&dlms_client_read_msg_queue;
}

CircularBuffer dlms_client_get_write_msg_queue( void)
{
	return (CircularBuffer)&dlms_client_write_msg_queue;
}

CircularBuffer dlms_client_get_send_msg_queue( void)
{
	return (CircularBuffer)&dlms_client_send_msg_queue;
}

uint32_t dlms_client_get_checksum_param( char * modbus_params )
{
	volatile int dst_size = sizeof(modbus_params);
	memset( modbus_params, '0', dst_size );
	dst_size = sizeof(dlms_checksum_param);
	snprintf(modbus_params, dst_size, "%s;", dlms_checksum_param);

	return 0;
}

void dlms_client_set_checksum_param( char * _checksum_param )
{
	strncpy((char *) dlms_checksum_param, _checksum_param, sizeof(dlms_checksum_param));
}

uint32_t dlms_client_get_dlms_load_profile_read_time(void)
{
	return dlms_generic_profile.read_time;
}

uint32_t dlms_client_get_dlms_load_profile_send_time(void)
{
	return dlms_generic_profile.send_time;
}

uint32_t dlms_client_set_dlms_load_profile_read_time( uint32_t __read_time )
{
	dlms_generic_profile.read_time = __read_time;

	return __read_time;
}

uint32_t dlms_client_set_dlms_load_profile_send_time( uint32_t __send_time )
{
	dlms_generic_profile.send_time = __send_time;

	return __send_time;
}

uint32_t dlms_client_get_dlms_load_profile_get_wait_for_command(void)
{
	return dlms_generic_profile.wait_for_comm;
}

uint32_t dlms_client_set_dlms_load_profile_set_wait_for_command( uint32_t __wait_for_comm )
{
	dlms_generic_profile.wait_for_comm = __wait_for_comm;

	return __wait_for_comm;
}

uint32_t dlms_client_get_dlms_load_profile_get_from_command(void)
{
	return dlms_generic_profile.profile_from_command;
}

uint32_t dlms_client_set_dlms_load_profile_set_from_command( uint32_t __from_comm )
{
	dlms_generic_profile.profile_from_command = __from_comm;

	return __from_comm;
}

uint32_t dlms_client_upload_read_send_time_with_load_profile( void )
{
	if (1 == dlms_client_get_dlms_enable())
	{
		if (dlms_client_get_dlms_load_profile_read_time() != 0)
		{
			generic_485_set_read_time(dlms_client_get_dlms_load_profile_read_time());
			generic_485_set_send_time(dlms_client_get_dlms_load_profile_read_time());
			dlms_client_set_read_time(dlms_client_get_dlms_load_profile_read_time());
			dlms_client_set_send_time(dlms_client_get_dlms_load_profile_read_time());
		}
	}
	return 0;
}

uint32_t dlms_client_get_dlms_load_profile_message_length(void)
{
	return dlms_generic_profile.message_length;
}

uint32_t dlms_client_set_dlms_load_profile_message_length( uint32_t __message_length )
{
	dlms_generic_profile.message_length = __message_length;

	return __message_length;
}

uint32_t dlms_client_get_dlms_load_profile_num_obis(void)
{
	return dlms_generic_profile.num_obis;
}

uint32_t dlms_client_set_dlms_load_profile_num_obis( uint32_t __num_obis )
{
	dlms_generic_profile.num_obis = __num_obis;

	return __num_obis;
}

char * dlms_client_get_load_profile_obis_n( uint32_t n)
{
	return dlms_generic_profile.dlms_obis_capture_objects[n];
}

void dlms_client_set_load_profile_obis_n(char *obis_n)
{
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> CAPTURE OBIS:%d\r\n", (int)Tick_Get(SECONDS),(int)dlms_client_vars.dlms_next_obis_capture_object);
	strncpy((char *) dlms_generic_profile.dlms_obis_capture_objects[dlms_client_vars.dlms_next_obis_capture_object], obis_n, sizeof(dlms_generic_profile.dlms_obis_capture_objects[dlms_client_vars.dlms_next_obis_capture_object]));

	if (dlms_client_vars.dlms_next_obis_capture_object < OBIS_CAPTURE_NUM_MAX)
	{
		dlms_client_vars.dlms_next_obis_capture_object++;
	}
	else
	{
		dlms_client_vars.dlms_next_obis_capture_object = 0;
	}
}

void dlms_client_set_load_profile_obis_block(char *obis_n)
{
	strncpy((char *) dlms_generic_profile.dlms_obis_capture_objects, obis_n, strlen(obis_n));
}

uint32_t dlms_client_get_dlms_load_profile_start_time(void)
{
	return dlms_generic_profile.start_time;
}

uint32_t dlms_client_set_dlms_load_profile_start_time( uint32_t __start_time )
{
	dlms_generic_profile.start_time = __start_time;

	return __start_time;
}

uint32_t dlms_client_reset_id_request_index(void)
{
	dlms_client_vars.id_request_index    = 0;
	dlms_client_vars.id_request_index_rd = 0;

	return 0;
}

uint32_t dlms_client_reset_id_request_queue(void)
{
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> %s.\r\n", (int)Tick_Get(SECONDS), dlms_client_vars.dlms_meter_id[0]);
	if (dlms_client_vars.dlms_meter_id[0][0] == '\0')
	{
		__NOP();
	}
	memset(dlms_client_vars.id_request[0], 0 , 5*40*sizeof(char));
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> Reset id_request queue.\r\n", (int)Tick_Get(SECONDS));
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> %s.\r\n", (int)Tick_Get(SECONDS), dlms_client_vars.dlms_meter_id[0]);
	return 0;
}

uint32_t dlms_client_inc_id_request_index_rd(void)
{
	if (dlms_client_vars.id_request_index_rd < 4)
	{
		dlms_client_vars.id_request_index_rd++;
	}
	else
	{
		dlms_client_vars.id_request_index_rd = 0;
	}
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> inc id_request_index_rd:%d\r\n", (int)Tick_Get(SECONDS), (int)dlms_client_vars.id_request_index_rd);

	return 0;
}

char *dlms_client_get_dlms_id_request(void)
{
	uint32_t next_id_request = 0;

	next_id_request = dlms_client_vars.id_request_index_rd;
	if ( ( dlms_client_vars.id_request[next_id_request] == NULL ) || ( 0 == params_get_mqtt_dc_on() ) )
	{
		dlms_client_vars.id_request[next_id_request][0] = '\0';
	}
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> get id_request_index_rd:%d\r\n", (int)Tick_Get(SECONDS), (int)dlms_client_vars.id_request_index_rd);
	return dlms_client_vars.id_request[dlms_client_vars.id_request_index_rd];
}

uint32_t dlms_client_set_dlms_id_request( char * __id_request )
{
	strncpy((char *) dlms_client_vars.id_request[dlms_client_vars.id_request_index], __id_request, sizeof(dlms_client_vars.id_request));
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> Set id_request_index:%d\r\n", (int)Tick_Get(SECONDS), (int)dlms_client_vars.id_request_index);
	if (dlms_client_vars.id_request_index < 10)
	{
		dlms_client_vars.id_request_index++;
	}
	else
	{
		dlms_client_vars.id_request_index = 0;
	}

	return 0;
}

uint32_t dlms_client_clear_dlms_data_cmd_on_demand(void)
{
	memset(dlms_client_vars.dlms_data_cmd_on_demand, 0, sizeof(dlms_client_vars.dlms_data_cmd_on_demand));
	return 0;
}

char *dlms_client_get_dlms_data_cmd_on_demand(void)
{
	if ( ( dlms_client_vars.dlms_data_cmd_on_demand == NULL ) || ( 0 == params_get_mqtt_dc_on() ) )
	{
		dlms_client_vars.dlms_data_cmd_on_demand[0] = '\0';
	}
	return dlms_client_vars.dlms_data_cmd_on_demand;
}

uint32_t dlms_client_set_dlms_data_cmd_on_demand( char * __dlms_data_cmd_on_demand )
{
	uint32_t data_offset;

	data_offset = strlen(dlms_client_vars.dlms_data_cmd_on_demand);
	strncpy((char *) dlms_client_vars.dlms_data_cmd_on_demand + data_offset, __dlms_data_cmd_on_demand, strlen(__dlms_data_cmd_on_demand));

	return 0;
}

uint32_t dlms_client_get_dlms_load_profile_end_time(void)
{
	return dlms_generic_profile.end_time;
}

uint32_t dlms_client_set_dlms_load_profile_end_time( uint32_t __end_time )
{
	dlms_generic_profile.end_time = __end_time;

	return __end_time;
}

void dlms_client_reset_load_profile_obis( void )
{
	uint32_t i;
	for (i = 0; i < dlms_client_vars.dlms_next_obis_capture_object; i++)
	{
		memset(dlms_generic_profile.dlms_obis_capture_objects[i], 0, sizeof(dlms_generic_profile.dlms_obis_capture_objects[i]));
	}
	dlms_client_vars.dlms_next_obis_capture_object = 0;
}

char *dlms_client_get_raw_modbus( char *buff, uint32_t *total_chars )
{
	uint8_t  i;
	uint32_t str_len  = 0;//, value = 0;

	for ( i = 0; i < dlms_client_vars.next_dlms_obis_value; i++ )
	{
		if ( 0 == i )
		{
			sprintf(buff + str_len, "%s;", dlms_obis_values[i]);
		}
		else
		{
			if ( isdigit((unsigned char)dlms_obis_values[i][0]) )
			{
//				value = atoi(dlms_client.dlms_obis_values[i]);
//				sprintf(buff + str_len, "%d;", (int)value);
				sprintf(buff + str_len, "%s;", dlms_obis_values[i]);
			}
			else
			{
				sprintf(buff + str_len, "%s;", dlms_obis_values[i]);
			}
		}
		str_len = strlen( buff );
	}

	return buff;
}

uint32_t dlms_client_get_dlms_enable(void)
{
	return dlms_client.enable;
}

uint32_t dlms_client_get_dlms_num_obis_profiles(void)
{
	return dlms_client.num_obis_profiles;
}

uint32_t dlms_client_get_dlms_num_generic_profiles(void)
{
	return dlms_client.num_generic_profiles;
}

dlms_client_t *dlms_client_get_client( void )
{
	return &dlms_client;
}

dlms_client_t *dlms_client_get_client_backup( void )
{
	return &dlms_client_backup;
}

uint32_t dlms_client_copy_read_send_times(dlms_client_t *client_backup, dlms_client_t *client)
{
	memcpy(client_backup->read_time, client->read_time, sizeof(dlms_client.read_time));
	memcpy(client_backup->send_time, client->send_time, sizeof(dlms_client.send_time));
	return 0;
}

dlms_client_obis_profile_t *dlms_client_get_client_obis_profile( void )
{
	return &dlms_client_obis_profile;
}

dlms_client_generic_profile_t *dlms_client_get_client_generic_profile( void )
{
	return &dlms_generic_profile;
}

size_t dlms_client_get_size( void )
{
	return sizeof(dlms_client);
}

size_t dlms_client_obis_profile_get_size( void )
{
	return sizeof(dlms_client_obis_profile);
}

size_t dlms_client_generic_profile_get_size( void )
{
	return sizeof(dlms_generic_profile);
}

uint32_t dlms_client_get_dlms_comm_state(void)
{
	return dlms_client_vars.comm_state;
}

uint32_t dlms_client_get_read_time(void)
{
	uint32_t i, min = 0;

	for (i = 0; i < DLMS_PROFILE_NUM; i++)
	{
		if (dlms_client.read_time[i] != 0)
		{
			min = dlms_client.read_time[i];
		}
	}
	for (i = 0; i < DLMS_PROFILE_NUM; i++)
	{
		if (dlms_client.read_time[i] == -1)
		{
			dlms_client.read_time[i] = 0;
		}
		if ((dlms_client.read_time[i] < min) && (dlms_client.read_time[i] != 0))
		{
			min = dlms_client.read_time[i];
		}
	}

	dlms_client_vars.read_time = min;

	return dlms_client_vars.read_time;
}

uint32_t dlms_client_get_read_meter(void)
{
	return dlms_client_vars.read_meter;
}

uint32_t dlms_client_get_send_meter(void)
{
	return dlms_client_vars.send_meter;
}

uint32_t dlms_client_get_send_time(void)
{
	uint32_t i, min = 0;

	if (dlms_client.num_generic_profiles == (uint32_t)-1)
	{
		dlms_client.num_generic_profiles = 0;
	}
	if (dlms_client.num_obis_profiles == (uint32_t)-1)
	{
		dlms_client.num_obis_profiles = 0;
	}
	for (i = 0; i < (dlms_client.num_generic_profiles + dlms_client.num_obis_profiles); i++)
	{
		if ((dlms_client.read_time[i] != 0) != 0)
		{
			min = dlms_client.send_time[i];
		}
	}

	for (i = 0;i < (dlms_client.num_generic_profiles + dlms_client.num_obis_profiles); i++)
	{
		if (dlms_client.send_time[i] == -1)
		{
			dlms_client.send_time[i] = 0;
		}
		if ((dlms_client.send_time[i] < min) && (dlms_client.read_time[i] != 0))
		{
			min = dlms_client.send_time[i];
		}
	}

	dlms_client_vars.send_time = min;

	return dlms_client_vars.send_time;
}

write_task_t dlms_client_get_frame_type_last_write(void)
{
	return dlms_client_vars.frame_type_last_write;
}

void dlms_client_set_frame_type_last_write(write_task_t _last_write)
{
	dlms_client_vars.frame_type_last_write = _last_write;
}

uint32_t dlms_client_get_energy_profile_1(void)
{
	return dlms_client_vars.energy_profile_1;
}

void dlms_client_set_energy_profile_1(uint32_t _profile)
{
	dlms_client_vars.energy_profile_1 = _profile;
}

uint32_t dlms_client_get_energy_profile_2(void)
{
	return dlms_client_vars.energy_profile_2;
}

void dlms_client_set_energy_profile_2(uint32_t _profile)
{
	dlms_client_vars.energy_profile_2 = _profile;
}

uint32_t dlms_client_get_energy_profile_3(void)
{
	return dlms_client_vars.energy_profile_3;
}

void dlms_client_set_energy_profile_3(uint32_t _profile)
{
	dlms_client_vars.energy_profile_3 = _profile;
}

uint32_t dlms_client_get_inst_profile_1(void)
{
	return dlms_client_vars.inst_profile_1;
}

void dlms_client_set_inst_profile_1(uint32_t _profile)
{
	dlms_client_vars.inst_profile_1 = _profile;
}

uint32_t dlms_client_get_inst_profile_2(void)
{
	return dlms_client_vars.inst_profile_2;
}

void dlms_client_set_inst_profile_2(uint32_t _profile)
{
	dlms_client_vars.inst_profile_2 = _profile;
}

uint32_t dlms_client_get_inst_profile_3(void)
{
	return dlms_client_vars.inst_profile_3;
}

void dlms_client_set_inst_profile_3(uint32_t _profile)
{
	dlms_client_vars.inst_profile_3 = _profile;
}

uint32_t dlms_client_get_inst_profile_4(void)
{
	return dlms_client_vars.inst_profile_4;
}

void dlms_client_set_inst_profile_4(uint32_t _profile)
{
	dlms_client_vars.inst_profile_4 = _profile;
}

void dlms_client_set_dlms_reading_items(uint32_t dev_id, uint32_t _items)
{
	dlms_client_vars.dlms_reading_items[dev_id] = _items;
}

void dlms_client_inc_dlms_reading_items(uint32_t dev_id)
{
	dlms_client_vars.dlms_reading_items[dev_id] = dlms_client_vars.dlms_reading_items[dev_id] + 1;
}

uint32_t dlms_client_get_dlms_reading_items(uint32_t dev_id)
{
	return dlms_client_vars.dlms_reading_items[dev_id];
}

uint32_t dlms_client_get_logical_address(void)
{
	return dlms_client.logical_address;
}

uint16_t dlms_client_get_cl_address(void)
{
	return dlms_client.cl_address;
}

uint32_t dlms_client_get_srv_address(void)
{
	return dlms_client.srv_address;
}

uint32_t dlms_client_get_baudrate(void)
{
	return dlms_client.baudrate;
}

void dlms_client_get_authen(char * authen)
{
	volatile size_t n = sizeof(dlms_client.auth_pass);
	strncpy((char *) authen, (char *) dlms_client.auth_pass, n);
}

uint32_t dlms_client_get_pass_is_hex(void)
{
	return dlms_client.pass_is_hex;
}

uint32_t dlms_client_get_pass_level(void)
{
	return dlms_client.pass_level;
}

uint32_t dlms_client_get_short_name_ref(void)
{
	return dlms_client.shortNameRef;
}

uint32_t dlms_client_get_billingday(void)
{
	return dlms_client.billingday;
}

uint32_t dlms_client_get_num_devices(void)
{
	return dlms_client.num_devices;
}

void dlms_client_set_pass_is_hex(uint32_t _is_hex)
{
	dlms_client.pass_is_hex = _is_hex;
}

void dlms_client_set_pass_level(uint32_t _pass_level)
{
	dlms_client.pass_level = _pass_level;
}

void dlms_client_set_short_name_ref(uint32_t _short_name)
{
	dlms_client.shortNameRef = _short_name;
}

void dlms_client_set_billingday(uint32_t _bill)
{
	dlms_client.billingday = _bill;
}

void dlms_client_set_num_devices(uint32_t _num_dev)
{
	dlms_client.num_devices = _num_dev;
}

void dlms_client_get_pass_in_hex(char * authen)
{
	uint32_t i;

	for(i=0;i<20;i++)
	{
		authen[i] = dlms_client.auth_pass[i] - '0';
	}
}

uint32_t dlms_client_get_send_time_obis_n(uint32_t obis_n)
{
	return dlms_client.send_time[obis_n];
}

uint32_t dlms_client_get_read_time_obis_n(uint32_t obis_n)
{
	return dlms_client.read_time[obis_n];
}

uint32_t dlms_client_set_send_time_obis_n(uint32_t obis_n, uint32_t _time)
{
	dlms_client.send_time[obis_n] = _time;

	return 0;
}

uint32_t dlms_client_set_read_time_obis_n(uint32_t obis_n, uint32_t _time)
{
	dlms_client.read_time[obis_n] = _time;

	return 0;
}

char * dlms_client_get_obis_n( uint32_t n)
{
	return dlms_client_obis_profile.dlms_obis[n];
}

char * dlms_client_get_generic_obis_n( uint32_t n)
{
	return dlms_generic_profile.dlms_obis_generic_profile[n];
}

uint32_t dlms_client_get_obis_profile_read_time( void )
{
	return dlms_client_obis_profile.read_time;
}

uint32_t dlms_client_get_obis_profile_send_time( void )
{
	return dlms_client_obis_profile.send_time;
}

void dlms_client_set_obis_profile_read_time( uint32_t _read_time )
{
	dlms_client_obis_profile.read_time = _read_time;
}

void dlms_client_set_obis_profile_send_time( uint32_t _send_time )
{
	dlms_client_obis_profile.send_time = _send_time;
}

uint32_t dlms_client_get_obis_profile_frame_type( void )
{
	return dlms_client_obis_profile.frame_type_obis_profile;
}

uint32_t dlms_client_get_obis_profile_max_demand_profile_extra( void )
{
	return dlms_client_obis_profile.max_demand_profile_extra;
}

uint32_t dlms_client_get_obis_profile_num_obis(void)
{
	return dlms_client_obis_profile.num_obis;
}

uint32_t dlms_client_set_obis_profile_num_obis( uint32_t __num_obis )
{
	dlms_client_obis_profile.num_obis = __num_obis;

	return __num_obis;
}

uint32_t dlms_client_get_obis_profile_from_command(void)
{
	return dlms_client_obis_profile.obis_from_command;
}

uint32_t dlms_client_set_obis_profile_from_command( uint32_t __obis_from_command )
{
	dlms_client_obis_profile.obis_from_command = __obis_from_command;

	return __obis_from_command;
}

uint32_t dlms_client_get_generic_profile_frame_type( void )
{
	return dlms_generic_profile.frame_type_generic_profile;
}

void dlms_client_set_obis_profile_frame_type( uint32_t _frame_type )
{
	dlms_client_obis_profile.frame_type_obis_profile = _frame_type;
}

void dlms_client_set_obis_profile_max_demand_profile_extra( uint32_t _max_demand_profile_extra )
{
	dlms_client_obis_profile.max_demand_profile_extra = _max_demand_profile_extra;
}

uint32_t dlms_client_get_max_demand_profile_extra_frame_type(uint32_t curr_device, uint32_t frame_type)
{
	uint32_t i, ret = 0;

	for( i = 0; i < 8; i++ )
	{
		dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(), i);
		if ( 1 == dlms_client_get_obis_profile_max_demand_profile_extra() )
		{
			ret = i;
			break;
		}
	}
	dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(), frame_type);
	return ret;
}

void dlms_client_set_generic_profile_frame_type( uint32_t _frame_type )
{
	dlms_generic_profile.frame_type_generic_profile = _frame_type;
}

char * dlms_client_get_meter_id( uint32_t device )
{
	return dlms_client_vars.dlms_meter_id[device];
}

char * dlms_client_get_obis_value_next(void)
{
	uint32_t next = dlms_client_vars.next_dlms_obis_value_rd;

	if (dlms_client_vars.next_dlms_obis_value_rd < DLMS_OBIS_MAX_NUM_VALUES)
	{
		dlms_client_vars.next_dlms_obis_value_rd++;
	}
	else
	{
		dlms_client_vars.next_dlms_obis_value_rd = 0;
	}

	return dlms_obis_values[ next ];
}

uint32_t dlms_client_get_param_change( void )
{
	return dlms_client_vars.param_change;
}

uint32_t dlms_client_get_resend( void )
{
	return dlms_client_vars.resend;
}

uint32_t dlms_client_get_disconnected( void )
{
	return dlms_client_vars.disconnected;
}

uint32_t dlms_client_get_command( void )
{
	return dlms_client_vars.command;
}

uint32_t dlms_client_get_synch_time( void )
{
	return dlms_client_vars.synch_time;
}

uint32_t dlms_client_get_read_energy_profile(void)
{
	return dlms_client_vars.read_energy_profile;
}

void dlms_client_set_read_energy_profile(uint32_t _energy_profile)
{
	dlms_client_vars.read_energy_profile = _energy_profile;
}

void dlms_client_set_dlms_enable(uint32_t _dlms_enable)
{
	dlms_client.enable = _dlms_enable;
}

void dlms_client_set_dlms_num_obis_profiles(uint32_t _num_obis_profiles)
{
	dlms_client_backup.num_obis_profiles = dlms_client.num_obis_profiles = _num_obis_profiles;
}

void dlms_client_set_dlms_num_generic_profiles(uint32_t _num_generic_profiles)
{
	dlms_client_backup.num_generic_profiles = dlms_client.num_generic_profiles = _num_generic_profiles;
}

void dlms_client_set_dlms_comm_state(uint32_t _dlms_comm_state)
{
	dlms_client_vars.comm_state = _dlms_comm_state;
}

void dlms_client_set_authen(char *authen)
{
	strncpy((char *) dlms_client.auth_pass, authen, sizeof(dlms_client.auth_pass));
}

void dlms_client_set_read_time(uint32_t _read_time)
{
	dlms_client_vars.read_time = _read_time;
}

void dlms_client_set_send_time(uint32_t _send_time)
{
	dlms_client_vars.send_time = _send_time;
}

void dlms_client_set_read_meter(uint32_t _read_data)
{
	dlms_client_vars.read_meter = _read_data;
}

void dlms_client_set_send_meter(uint32_t _send_data)
{
	dlms_client_vars.send_meter = _send_data;
}

void dlms_client_set_logical_address(uint32_t _logical_address)
{
	dlms_client.logical_address = (int16_t)_logical_address;
}

void dlms_client_set_cl_address(uint32_t _cl_address)
{
	dlms_client.cl_address = (int16_t)_cl_address;
}

void dlms_client_set_srv_address(uint32_t _srv_address)
{
	dlms_client.srv_address = _srv_address;
}

void dlms_client_set_baudrate(uint32_t _baudrate)
{
	dlms_client.baudrate = _baudrate;
}

void dlms_client_set_obis_n(char *obis_n, uint32_t n)
{
	strncpy((char *) dlms_client_obis_profile.dlms_obis[n], obis_n, sizeof(dlms_client_obis_profile.dlms_obis[n]));
}

void dlms_client_set_generic_obis_n(char *obis_n, uint32_t n)
{
	strncpy((char *) dlms_generic_profile.dlms_obis_generic_profile[n], obis_n, sizeof(dlms_generic_profile.dlms_obis_generic_profile[n]));
}

void dlms_client_set_obis_value(char *obis_value, uint32_t n, unsigned char attributeOrdinal)
{
	size_t size = sizeof(dlms_obis_values[dlms_client_vars.next_dlms_obis_value]);
	memset(dlms_obis_values[dlms_client_vars.next_dlms_obis_value],'\0', size);
	strncpy((char *) dlms_obis_values[dlms_client_vars.next_dlms_obis_value], obis_value, n);
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> next_dlms_obis_value:%d\n", (int)Tick_Get(SECONDS), (int)dlms_client_vars.next_dlms_obis_value);
//	if ( 3 == attributeOrdinal )
	{
		if (dlms_client_vars.next_dlms_obis_value < DLMS_OBIS_MAX_NUM_VALUES)
		{
			dlms_client_vars.next_dlms_obis_value++;
		}
		else
		{
			dlms_client_vars.next_dlms_obis_value = 0;
		}
	}
}

void dlms_client_append_obis_value(char *obis_value, unsigned char attributeOrdinal)
{
//	strcat( dlms_obis_values[dlms_client_vars.next_dlms_obis_value], obis_value);
	size_t size = sizeof(dlms_obis_values[dlms_client_vars.next_dlms_obis_value]);
	memset(dlms_obis_values[dlms_client_vars.next_dlms_obis_value],'\0', size);
	strncpy((char *) dlms_obis_values[dlms_client_vars.next_dlms_obis_value], obis_value, strlen(obis_value));
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> next_dlms_obis_value:%d\n", (int)Tick_Get(SECONDS), (int)dlms_client_vars.next_dlms_obis_value);

	if ( 2 == attributeOrdinal )
	{
		if (dlms_client_vars.next_dlms_obis_value < DLMS_OBIS_MAX_NUM_VALUES)
		{
			dlms_client_vars.next_dlms_obis_value++;
		}
		else
		{
			dlms_client_vars.next_dlms_obis_value = 0;
		}
	}
}

void dlms_client_dec_next_obis_val(void)
{
	if (dlms_client_vars.next_dlms_obis_value != 0)
	{
		dlms_client_vars.next_dlms_obis_value--;
	}
}

void dlms_client_set_param_change(uint32_t _param_change)
{
	dlms_client_vars.param_change = _param_change;
	if ( 1 == _param_change )
	{
		dlms_client_set_resend(1);
		__NOP();
	}
}

void dlms_client_set_resend(uint32_t _resend)
{
	dlms_client_vars.resend = _resend;
	if ( 1 == _resend )
	{
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_CLIENT> RESEND\r\n", (int)Tick_Get(SECONDS));
		__NOP();
	}
}

void dlms_client_set_disconnected(uint32_t _disconnected)
{
	dlms_client_vars.disconnected = _disconnected;
	if ( 1 == _disconnected )
	{
		__NOP();
	}
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> DISCONNECT CONTROL:%d\r\n", (int)Tick_Get(SECONDS), (int)_disconnected);
}

void dlms_client_set_command(uint32_t _command)
{
	dlms_client_vars.command = _command;
}

void dlms_client_set_synch_time(uint32_t _synch)
{
	dlms_client_vars.synch_time = _synch;
}

void dlms_client_reset_indexes( void)
{
	dlms_client_vars.next_dlms_obis_value    = 0;
	dlms_client_vars.next_dlms_obis_value_rd = 0;
}

char * dlms_client_get_params(char * modbus_params)
{
	uint32_t i = 0;
	char     authen[30];
	//	char     modbus_params[500];
	uint32_t len;

	dlms_client_get_authen(authen);
	dlms_client_set_param_change(0);

	volatile size_t n = sizeof(modbus_params);
	memset( modbus_params, '0', n );

	sprintf(modbus_params, "||%d;%d;%d;%d;%d;%s;",
			(int)dlms_client_get_dlms_enable(),
			(int)dlms_client_get_read_time(),
			(int)dlms_client_get_send_time(),
			(int)dlms_client_get_cl_address(),
			(int)dlms_client_get_srv_address(),
			authen
	);
	len = strlen(modbus_params);

	for(i = 0; i < DLMS_OBIS_PROFILE_NUM; i++)
	{
		dlms_client_table_read_client_obis_profile(0, dlms_client_get_client(), dlms_client_get_client_obis_profile(), i);
		if (dlms_client_get_obis_profile_read_time() != -1)
		{
			sprintf(modbus_params + len, "%d;%d;%d;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;",
					(int)dlms_client_get_obis_profile_read_time(),
					(int)dlms_client_get_obis_profile_send_time(),
					(int)dlms_client_get_obis_profile_frame_type(),
					dlms_client_get_obis_n(0),
					dlms_client_get_obis_n(1),
					dlms_client_get_obis_n(2),
					dlms_client_get_obis_n(3),
					dlms_client_get_obis_n(4),
					dlms_client_get_obis_n(5),
					dlms_client_get_obis_n(6),
					dlms_client_get_obis_n(7),
					dlms_client_get_obis_n(8),
					dlms_client_get_obis_n(9),
					dlms_client_get_obis_n(10),
					dlms_client_get_obis_n(11),
					dlms_client_get_obis_n(12),
					dlms_client_get_obis_n(13),
					dlms_client_get_obis_n(14),
					dlms_client_get_obis_n(15),
					dlms_client_get_obis_n(16),
					dlms_client_get_obis_n(17),
					dlms_client_get_obis_n(18),
					dlms_client_get_obis_n(19)
			);
			len = strlen(modbus_params);
		}
	}
	for(i = 0; i < DLMS_GENERIC_PROFILE_NUM; i++)
	{
		dlms_client_table_read_client_generic_profile(0, dlms_client_get_client(), dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + i);
		if (dlms_client_get_dlms_load_profile_read_time() != -1)
		{
			sprintf(modbus_params + len, "%d;%d;%d;%s;",
					(int)dlms_client_get_dlms_load_profile_read_time(),
					(int)dlms_client_get_dlms_load_profile_send_time(),
					(int)dlms_client_get_generic_profile_frame_type(),
					dlms_client_get_generic_obis_n(0)
			);
			len = strlen(modbus_params);
		}
	}

	return modbus_params;
}

uint64_t dlms_client_on_demand_synch( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;
	uint64_t         date_param, date_param_shift;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - ( 2000 - 1900 );

    date_param       = RTC_DateStruct.Year;
    date_param_shift = RTC_DateStruct.Month;
    date_param      |= date_param_shift<<8;
    date_param_shift = RTC_DateStruct.Date;
    date_param      |= date_param_shift<<16;
    date_param_shift = RTC_TimeStructure.Hours;
    date_param      |= date_param_shift<<24;
    date_param_shift = RTC_TimeStructure.Minutes;
    date_param      |= date_param_shift<<32;
    date_param_shift = RTC_TimeStructure.Seconds;
    date_param      |= date_param_shift<<40;

    return date_param;
}

size_t getGinUse(void)
{
	return G_inUse;
}

size_t getFS_totalAllocated(void)
{
	return FS_totalAllocated;
}

void *mmalloc(size_t size, uint8_t *file, uint32_t line)
{
	int i;
	void *newAllocation = malloc(size);

	if ( newAllocation == NULL )
	{
		return NULL;//__NOP();
	}

	for (i = 0; i < NUM_BLOCKS; i++)
	{
		if (blocks[i].addr == 0)
		{
			// found empty entry; use it
			blocks[i].addr = newAllocation;
			blocks[i].size = size;
//			incrementCountForSize(size);
			G_inUse += size;
			num_blocks = i;
			break;
		}
	}
	if ( i == ( NUM_BLOCKS - 1 ) )
	{
		__NOP();
	}
	FS_totalAllocated += size;
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> MMALLOC BLOCK:%d size:%d FS_totalAllocated:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)i, (int)size, (int)FS_totalAllocated,file, (int) line);
	return newAllocation;
#if 0
	G_inUse += size;
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> mmalloc size:%d GinUse:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)size, (int)getGinUse(),file, (int) line);
	return malloc(size);
#endif
}

void *mcalloc(size_t n, size_t size_n, uint8_t *file, uint32_t line)
{
	int i;
	size_t size = n * size_n;

	void *newAllocation = calloc(n, size_n);

	for (i = 0; i < NUM_BLOCKS; i++)
	{
		if (blocks[i].addr == 0)
		{
			// found empty entry; use it
			blocks[i].addr = newAllocation;
			blocks[i].size = size;
//			incrementCountForSize(size);
			G_inUse += size;
			num_blocks = i;
			break;
		}
	}
	if ( i == ( NUM_BLOCKS - 1 ) )
	{
		__NOP();
	}
	if ( newAllocation == NULL )
	{
		__NOP();
	}
	FS_totalAllocated += size;
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> MCALLOC BLOCK:%d size:%d FS_totalAllocated:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)i, (int)size, (int)FS_totalAllocated,file, (int) line);
	return newAllocation;
#if 0
	G_inUse += n * sizeof(size);
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> mcalloc size:%d GinUse:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)size, (int)FS_totalAllocated,file, (int) line);
	return calloc(n, size);
#endif
}

void *mrealloc(void *p, size_t size, uint8_t *file, uint32_t line)
{
	int i;
	void *newAllocation = NULL;

	for (i = 0; i < NUM_BLOCKS; i++)
	{
		if (blocks[i].addr == p)
		{
			// found block being released
//			decrementCountForSize(blocks[i].size);
			G_inUse           -= blocks[i].size;
			FS_totalAllocated -= blocks[i].size;
			newAllocation      = realloc(p, size);
			blocks[i].addr     = newAllocation;
			blocks[i].size     = size;
			G_inUse           += size;
			FS_totalAllocated += size;
			break;
		}
	}
	if ( newAllocation == NULL )
	{
		__NOP();
	}
//	int j;
//	void *newAllocation = realloc(p, size);
//
//	for (j = 0; j < NUM_BLOCKS; j++)
//	{
//		if (blocks[j].addr == 0)
//		{
//			// found empty entry; use it
//			blocks[j].addr = newAllocation;
//			blocks[j].size = size;
////			incrementCountForSize(size);
//			G_inUse += size;
//			break;
//		}
//	}
//	FS_totalAllocated += size;
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> MREALLOC BLOCK:%d size:%d FS_totalAllocated:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)i, (int)size, (int)FS_totalAllocated,file, (int) line);
	return newAllocation;
#if 0
	size_t *sizePtr = ( (size_t *) p ) - 1;
	if (*sizePtr < 0)
	{
		G_inUse -= *sizePtr;
	}
	G_inUse += size;
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> mrealloc size:%d GinUse:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)size, (int)getGinUse(),file, (int) line);
	return realloc(p, size);
#endif
}

void mfree(void *p, uint8_t *file, uint32_t line)
{
	int i, found = 0;
	size_t size_freed = 0;

	for (i = 0; i < NUM_BLOCKS; i++)
	{
		if (blocks[i].addr == p)
		{
			// found block being released
//			decrementCountForSize(blocks[i].size);
			size_freed         = blocks[i].size;
			(void)size_freed;
			G_inUse           -= blocks[i].size;
			FS_totalAllocated -= blocks[i].size;
			blocks[i].addr     = 0;
			blocks[i].size     = 0;
			num_blocks        -= 1;
			found              = 1;
			break;
		}
	}
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> MFREE BLOCK:%d size:%d FS_totalAllocated:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)i, (int)size_freed, (int)FS_totalAllocated,file, (int) line);
	if ( 1 == found )
	{
		free(p);
	}
#if 0
	size_t *sizePtr = ( (size_t *) p ) - 1;
	if (*sizePtr > 0)
	{
		G_inUse -= *sizePtr;
	}
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> mfree size:%d GinUse:%d file %s on line %d\n", (int)Tick_Get(SECONDS), (int)*sizePtr, (int)getGinUse(),file, (int) line);
	free(p);
#endif
}

void mfree_all(void)
{
	int i, block_num = 0;
	size_t size_freed = 0;
	(void)size_freed;

	for (i = 0; i < NUM_BLOCKS; i++)
	{
		if (blocks[i].addr != NULL)
		{
			// found block being released
			free(blocks[i].addr);
//			decrementCountForSize(blocks[i].size);
			size_freed         = blocks[i].size;
			G_inUse           -= blocks[i].size;
			FS_totalAllocated -= blocks[i].size;
			blocks[i].addr     = 0;
			blocks[i].size     = 0;
			num_blocks        -= 1;
			block_num++;
		}
	}
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> MFREE ALL BLOCK:%d block num:%d. size:%d FS_totalAllocated:%d\n", (int)Tick_Get(SECONDS), (int)i, (int)block_num, (int)size_freed, (int)FS_totalAllocated);
}
