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

#include "connection.h"

#include "iwdg.h"
#include "params.h"
#include "dlms_client.h"
#include "dlms_client_table.h"
#include "dlms_client_on_demand_comm.h"
#include "tick.h"
#include "communication.h"
#include "cosem.h"
#include "converters.h"
#include "udp_protocol.h"
#include "shutdown.h"
#include "message_queue.h"
#include "dlms_log.h"
#include "gxmem.h"
#include "mqtt_task.h"
#include "mbus.h"

#define WRITE_MSG_PENDING       ((0) == CircularBuffer_IsEmpty(dlms_client_get_write_msg_queue()))
#define SEND_MSG_PENDING        (con_send_task<=(DLMS_SEND_EVENT_LOG_PROF))//((0) == CircularBuffer_IsEmpty(dlms_client_get_send_msg_queue()))
#define READ_MSG_PENDING        ((0) == CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue()))
#define READ_OBIS_PROFILE       (((con_read_task>=DLMS_READ_INST_PROF_1)&&(con_read_task<=DLMS_READ_MAX_DEMAND_PROF_1))||((con_read_task>=DLMS_READ_CMD_INST_PROF_1)&&(con_read_task<=DLMS_READ_CMD_MAX_DEMAND_PROF_1)))
#define READ_GENERIC_PROFILE    (((con_read_task>=DLMS_READ_LOAD_PROF_1)&&(con_read_task<=   DLMS_READ_EVENT_LOG_PROF))||((con_read_task>=DLMS_READ_CMD_LOAD_PROF_1)&&(con_read_task<=   DLMS_READ_CMD_EVENT_LOG_PROF)))
#define WRITE_OBIS_PROFILE      (((con_write_task>=DLMS_WRITE_INST_PROF_1)&&(con_write_task<=DLMS_WRITE_MAX_DEMAND_PROF_1)))

connection       con;

gxProfileGeneric pg;
message          messages;
gxReplyData      reply;
uint32_t         curr_device;
uint32_t         num_items_device = 0;
uint32_t         on_demand_cmd    = 0, on_demand_cmd_data = 0;
uint32_t 		 retry            = 0;
uint16_t 		 shortNameAddress[14] = {
					0x700,  0x730,  0x760,
					0x790,  0x7C0,  0x7F0,
					0x820,  0x850,  0x4A80,
					0x4AB0, 0x4AE0, 0x4B10,
					0x4B40, 0x4B70
};

static uint8_t __CalculateChecksum(char *data, uint32_t index, uint32_t count)
{
	uint8_t result = 0;
	for (int pos = index; pos != index + count; ++pos)
	{
		result ^= data[pos];
	}
	return result;
}
uint32_t device_from_cmd_index = 0;
char     devices_from_cmd[30][32];
char * con_dlms_get_device_from_cmd(uint32_t index)
{
	return devices_from_cmd[index];
}

uint32_t con_dlms_get_next_device_for_cmd(uint32_t index)
{
	uint32_t i = 0, j = 0, id = 0xFF;
	for ( i = index; i < dlms_client_table_get_num_meters_from_cmd() ; i++ )
	{
		if ( *con_dlms_get_device_from_cmd(i) != '\0' )
		{
			for ( j = 0; j < dlms_client_table_get_num_devices() ; j++ )
			{
				if ( strstr(dlms_client_get_meter_id(j), (char *)con_dlms_get_device_from_cmd(i)) )
				{
					con_dlms_set_curr_device(j);
					id = j;
					break;
				}
			}
			break;
		}
	}
	return id;
}

uint32_t con_dlms_get_curr_id_device_from_cmd_table(uint32_t index)
{
	uint32_t j, id = 0;

	if ( *con_dlms_get_device_from_cmd(index) != '\0' )
	{
		for ( j = 0; j < dlms_client_table_get_num_devices() ; j++ )
		{
			if ( strstr(dlms_client_get_meter_id(j), (char *)con_dlms_get_device_from_cmd(index)) )
			{
				id = j;
				break;
			}
		}
	}
	return id;
}
// con functions.
connection *con_dlms_get_con_singleton(void)
{
	return &con;
}

void con_dlms_change_state(connection* con, dlms_state_t state)
{
	con->state =state;
}

dlms_state_t con_dlms_get_state(connection* con)
{
	return con->state;
}

uint32_t con_dlms_in_process( void )
{
	if (con.state != DLMS_STATE_IDLE)
	{
//		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Connection State = %d\n", (int)Tick_Get(SECONDS), (int)con.state);
		return 1;
	}
	else
	{
		return 0;
	}
}

uint32_t con_dlms_get_curr_device( void )
{
	return curr_device;
}

void con_dlms_set_curr_device( uint32_t _curr_dev )
{
	curr_device = _curr_dev;
//	if (0 == curr_device)
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Set curr_device:%d.\r\n", (int)Tick_Get(SECONDS), (int)curr_device);
	}
}

void con_dlms_inc_curr_device( void )
{
	curr_device++;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Inc curr_device:%d.\r\n", (int)Tick_Get(SECONDS), (int)curr_device);
}

uint32_t con_release_curr_com = 0;
void con_dlms_set_release_curr_com(uint32_t _con_release_curr_com)
{
	con_release_curr_com = _con_release_curr_com;
}

uint32_t con_dlms_release_curr_com(connection* con)
{
	com_close(con);
	con_close(con);
	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_DISCONNECT free FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
	mfree_all();
	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> mfree_all FS_totalAllocated:%d.Send Meter:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated(), (int)dlms_client_get_send_meter());

	return 0;
}

//Initialize connection buffers.
int con_initializeBuffers(connection* connection, int size)
{
    int ret;
	if (size == 0)
    {
        ret = bb_clear(&connection->data);
    }
    else
    {
        ret = bb_capacity(&connection->data, size);
    }
	return ret;
}

//Initialize connection settings.
void con_init(connection* con, GX_TRACE_LEVEL trace)
{
    con->trace = trace;
    //Reply wait time is 5 seconds.
    con->waitTime = 5000;
    bb_init(&con->data);
    bb_capacity(&con->data, 1024);//500
}

//Close connection..
void con_close(connection* con)
{
//    bb_clear(&con->data);
	cl_clear(&con->settings);
    con_initializeBuffers(con, 0);
}

struct tm *  con_convert_time( time_t  timer)
{
	struct tm * timer_tm = gxcalloc(1, sizeof(struct tm));
    timer_tm = gmtime(&timer);

    return timer_tm;
}

uint16_t con_get_shortName( uint32_t num_obis )
{
	uint16_t val = 0;

	if ( strstr(dlms_client_get_obis_n(num_obis),"."))
	{
		if (*(dlms_client_get_obis_n(num_obis)+6)=='8')
		{
			switch (*(dlms_client_get_obis_n(num_obis)+4))
			{

			case '1':
				val = 0x24e0;
				break;
			case '2':
				val = 0x1ce0;
				break;
			case '3':
				val = 0x1e40;
				break;
			case '4':
				val = 0x1ef8;
				break;
			default:
				break;
			}
		}
		else if (*(dlms_client_get_obis_n(num_obis)+7)=='7')
		{
			switch (*(dlms_client_get_obis_n(num_obis)+4))
			{

			case '3':
				if (*(dlms_client_get_obis_n(num_obis)+5) == '1')
				{
					val = 0x9618;
				}
				else if (*(dlms_client_get_obis_n(num_obis)+5) == '2')
				{
					val = 0x9450;
				}
				break;
			case '5':
				if (*(dlms_client_get_obis_n(num_obis)+5) == '1')
				{
					val = 0x96b0;
				}
				else if (*(dlms_client_get_obis_n(num_obis)+5) == '2')
				{
					val = 0x94e8;
				}
				break;
			case '7':
				if (*(dlms_client_get_obis_n(num_obis)+5) == '1')
				{
					val = 0x9748;
				}
				else if (*(dlms_client_get_obis_n(num_obis)+5) == '2')
				{
					val = 0x9580;
				}
				break;
			default:
				break;
			}
		}
	}
	else
	{
		val = (uint16_t)strtol(dlms_client_get_obis_n(num_obis), NULL, 16);
	}
	return val;
}

uint16_t con_get_generic_profile_shortName( void )
{
	uint16_t val = 0;

	val = (uint16_t)strtol(dlms_client_get_generic_obis_n(0), NULL, 16);

	return val;
}

uint32_t con_get_logical_address(uint32_t *srv_address)
{
	if (*srv_address < 0x80)
	{
		*srv_address |= 0x80;
	}
	else if (*srv_address < 0x4000)
	{
		*srv_address |= 0x4000;
	}
	else if (*srv_address < 0x10000000)
	{
		*srv_address |= 0x10000000;
	}
	return *srv_address;
}

static uint32_t __isfromCommand(void)
{
	uint32_t is_from_cmd = 0;

	read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
	if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
	{
		if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_MAX_DEMAND_PROF_1))
		{
//			dlms_client_set_obis_profile_from_command(1);
			is_from_cmd = 1;
		}
		else if ((con_read_task >= DLMS_READ_CMD_LOAD_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
		{
//			dlms_client_set_dlms_load_profile_set_from_command(1);
			is_from_cmd = 1;
		}
	}

	return is_from_cmd;
}

uint32_t con_get_num_items_device(void)
{
	return num_items_device;
}

uint32_t con_dec_num_items_device(void)
{
	num_items_device = num_items_device - 1;

	return 0;
}

static uint32_t __checkMaxDemand( uint32_t n_obis)
{
	char     *str;
	char     dlms_obis[24];
	int      max_demand_obis = 0;
	uint32_t ret             = 0;
	char     obisdata_ln[8]  = {'1', '.', '0', '.', '1', '3', '.', '5'};

	if (0 == memcmp(dlms_client_get_obis_n(n_obis), obisdata_ln, 8))
	{
		__NOP();
		ret = 1;
		return ret;
	}

	memcpy(dlms_obis, dlms_client_get_obis_n(n_obis), 24*sizeof(char));

    str = strtok(dlms_obis, ".");
    if (str != NULL)
    str = strtok(NULL, ".");
    if (str != NULL)
    str = strtok(NULL, ".");
    if (str != NULL)
    str = strtok(NULL, ".");

    if (str != NULL)
    max_demand_obis = atoi(str);

    if (max_demand_obis == 6)
    {
    	ret = 1;
    }

    return ret;
}

static uint32_t __attribute__((optimize("O0"))) __getDate(char *data_in, char * date)
{
	char temp_year[5] = {'2','0','2','3'};
	int val;
	size_t len  = sizeof(date);

	memset(date, 0, 100 * sizeof(char));

	val       = data_in[0];
	val       = val<<8;
	val      |= data_in[1];
	itoa(val, temp_year, 10);

	//Day.
	val       = data_in[2];
	itoa(val, date, 10);
	len       = strlen(date);
	date[len] = '/';
	len      += 1;
	//Month.
	val       = data_in[3];
	itoa(val, date + len, 10);
	len       = strlen(date);
	date[len] = '/';
	len      += 1;
	//Year.
	memcpy(date + len, temp_year, 4);
	len       = strlen(date);
	date[len] = ' ';
	len      += 1;

	val = data_in[5];
	itoa(val, date + len, 10);
	len = strlen(date);
	date[len] = ':';
	len += 1;
	val = data_in[6];
	itoa(val, date + len, 10);
	len = strlen(date);
	date[len] = ':';
	len += 1;
	val = data_in[7];
	itoa(val, date + len, 10);

	return 0;
}

/**
 * @fn void con_dlms_fsm(connection*)
 * @brief
 *
 * @param con
 *
 * \msc

a [label="Pipe20"],b [label="DLMS Meter"];
a-xb [label="SNMR"];
 ---  [label = "END PROCESS DLMS ERROR"];
a=>b [label="SNMR"];
a<=b [label="UA RESPONSE"];
a=>b [label="AARQ"];
a<=b [label="AARE"];

\endmsc
 *
 */
uint32_t     meter_clk_obis_value;
uint32_t con_dlms_get_meter_clk_obis_value(void)
{
	return meter_clk_obis_value;
}

//Constants for different orders of date components.
typedef enum
{
    CON_DATE_FORMAT_INVALID = -1,
	CON_DATE_FORMAT_DMY = 0,
	CON_DATE_FORMAT_MDY = 1,
	CON_DATE_FORMAT_YMD = 2,
	CON_DATE_FORMAT_YDM = 3
} CON_DATE_FORMAT;

uint32_t format_myd = 0;
uint32_t con_dlms_get_format_myd(void)
{
	return format_myd;
}

uint32_t con_dlms_getDateFormat( uint32_t *__meter_time )
{
	struct tm   *meter_date;
	struct tm   *curr_date;
//	struct tm   *new_date;
	time_t       date;
	uint8_t      curr_day, meter_day, curr_month, meter_month, curr_year, meter_year;

	time_t curr_time   = Tick_Get(SECONDS) + 4*3600;
	curr_date          = gmtime(&curr_time);
	curr_day           = curr_date->tm_mday;
	curr_month         = curr_date->tm_mon;
	curr_year          = curr_date->tm_year;

	time_t meter_time  = *__meter_time;
	meter_date         = gmtime(&meter_time);
	meter_day          = meter_date->tm_mday;
	meter_month        = meter_date->tm_mon;
	meter_year         = meter_date->tm_year;

	meter_date->tm_mday = curr_day;
	meter_date->tm_year = curr_year;
	date                = mktime(meter_date);

	format_myd    		= 0;

	if ( (meter_day + 100) == curr_year )
	{
		if ( curr_month == meter_month )
		{
			if ( (curr_day + 100) == meter_year )
			{
				format_myd    = 1;
				*__meter_time = date;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Time Format MYD!!!Time:%d Meter Time:%d\n", (int)Tick_Get(SECONDS), (int)date, (int)meter_time);
			}
		}
	}

	return format_myd;
}

uint32_t con_dlms_setDateFormat( uint32_t *__meter_time )
{
	if ( 1 == format_myd )
	{
		struct tm   *meter_date;
		struct tm   *curr_date;
//		struct tm   *new_date;
		time_t       date;
		uint8_t      curr_day, curr_year;// curr_month,meter_day, meter_month, meter_year

		time_t curr_time   = *__meter_time;
		curr_date          = gmtime(&curr_time);
		curr_day           = curr_date->tm_mday;
//		curr_month         = curr_date->tm_mon;
		curr_year          = curr_date->tm_year;

		time_t meter_time  = *__meter_time;
		meter_date         = gmtime(&meter_time);
//		meter_day          = meter_date->tm_mday;
//		meter_month        = meter_date->tm_mon;
//		meter_year         = meter_date->tm_year;

		meter_date->tm_mday = curr_year - 100;
		meter_date->tm_year = curr_day  + 100;
		date                = mktime(meter_date);

		*__meter_time = date;
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Synch Time Format MYD!!!Time:%d Meter Time:%d\n", (int)Tick_Get(SECONDS), (int)curr_time, (int)date);
	}
	return 0;
}

int con_dlms_Task(connection* con)
{
    static int ret = DLMS_ERROR_CODE_OK;
    static uint32_t n_obis = 0;//, wait_close = 0;

    if ( 1 == con_release_curr_com)
    {
    	con_release_curr_com = 0;
    	con_dlms_release_curr_com(con);
    	con->state = DLMS_STATE_IDLE;
    }

	switch(con->state)
	{
	case DLMS_STATE_IDLE:
		retry = 0;
		if ( 1 == dlms_client_get_dlms_enable() )
		{
			//		if (( 1 == mbus_get_start_comm() ) && ( 0 == mbus_get_end_comm() ) && (MBUS_SLAVES_METER_TYPE == params_get_uart_meter_type()))
			if (( 1 == mbus_get_start_comm() ) && ( 0 == mbus_get_end_comm() ) && ((MBUS_SLAVES_METER_TYPE == params_get_uart_meter_type()) || (SEVERN_TRENT_METER_TYPE == params_get_uart_meter_type())))
			{

			}
			else if READ_MSG_PENDING
			{
				on_demand_cmd = __isfromCommand();
				if (0 == on_demand_cmd)
				{
					dlms_client_table_read_client_id( dlms_client_get_client(), curr_device );
					if ( 0 == num_items_device )
					{
						num_items_device = dlms_client_get_dlms_reading_items(curr_device);
					}
					con->state = DLMS_STATE_INIT;
				}
				else
				{
					//				dlms_client_table_check_meter_id( dlms_client_get_client(), (uint8_t *)con_dlms_get_device_from_cmd(device_from_cmd_index));
					dlms_client_table_read_client_id( dlms_client_get_client(), curr_device );
					if ( 0 == num_items_device )
					{
						num_items_device = dlms_client_get_dlms_reading_items(curr_device);
					}
					con->state = DLMS_STATE_INIT;
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> con_init Device:%d.Num items device:%d\n", (int)Tick_Get(SECONDS), (int)curr_device, (int)num_items_device);
			}
			else
			{
				dlms_client_set_dlms_comm_state(DLMS_COMM_END);
			}
		}
	break;

	/* Initialize buffers */
	case DLMS_STATE_INIT:
	{
		send_task_t con_send_task = (send_task_t)CircularBuffer_Read(dlms_client_get_send_msg_queue());
		if ((WRITE_MSG_PENDING) || (SEND_MSG_PENDING))
		{
			if READ_MSG_PENDING
			{
				read_task_t con_read_task = (send_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
				if (con_read_task < DLMS_READ_CMD_INST_PROF_1)
				{
//					break;
				}
			}
			else
			{
				break;
			}
		}
//		dlms_client_table_read_client_id( dlms_client_get_client(), curr_device );
//		num_items_device = dlms_client_get_dlms_num_obis_profiles() + dlms_client_get_dlms_num_generic_profiles();
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> FS_totalAllocated:%d\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> con_init Device:%d.\n", (int)Tick_Get(SECONDS), (int)curr_device);
		con_init(con, GX_TRACE_LEVEL_OFF);
		dlms_client_set_dlms_comm_state(DLMS_COMM_STARTED);
		dlms_client_reset_indexes();
		/*
		 * client address = 0x64 -> 100d
		 * server address = logical address 0x01 -> 00000001 physical address 0x11 -> 1 00010001
		 * 						-> 7 bit address 1 0010001 = 0x91 -> 145d
		 */
		char auth_pass[20]   = {0x01, 0x09, 0x02, 0x04, 0x05, 0x00, 0x08, 0x08, 0x06, 0x00, 0x03, 0x05, 0x06, 0x00, 0x09, 0x04, 0x07, 0x07, 0x05, 0x05};
		char auth_ascii_pass[21];
		uint16_t cl_address     = 100;
		uint32_t srv_address    = 145;
		uint32_t pass_lev       = DLMS_AUTHENTICATION_LOW;
		uint32_t short_name_ref = 1;
		pass_lev       = dlms_client_get_pass_level();
		short_name_ref = dlms_client_get_short_name_ref()?0:1;
		cl_address     = dlms_client_get_cl_address();
		srv_address    = dlms_client_get_srv_address();

		if ( 1 == dlms_client_get_logical_address() )
		{
			con_get_logical_address(&srv_address);
		}
		else if ( 16 == dlms_client_get_logical_address() )
		{
			srv_address |= 0x800;
		}
		if ( 0 == dlms_client_get_pass_is_hex() )
		{
			memset(auth_ascii_pass, 0, sizeof(auth_ascii_pass));
			dlms_client_get_authen(auth_ascii_pass);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cl_init\n", (int)Tick_Get(SECONDS));
			cl_init(&con->settings, short_name_ref, cl_address, srv_address, pass_lev, auth_ascii_pass, DLMS_INTERFACE_TYPE_HDLC);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS server password: %s\n", (int)Tick_Get(SECONDS), auth_ascii_pass);
		}
		else
		{
			dlms_client_get_pass_in_hex(auth_pass);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cl_init\n", (int)Tick_Get(SECONDS));
			cl_init_pass_hex(&con->settings, short_name_ref, cl_address, srv_address, pass_lev, (uint8_t *)auth_pass, DLMS_INTERFACE_TYPE_HDLC);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS server password: %s\n", (int)Tick_Get(SECONDS), auth_pass);
		}
		con->settings.invokeID     = 1;
		con->settings.priority     = DLMS_PRIORITY_HIGH;
		con->settings.serviceClass = DLMS_SERVICE_CLASS_CONFIRMED;
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> DLMS client address: %d\n", (int)Tick_Get(SECONDS), cl_address);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> DLMS server address: %d\n", (int)Tick_Get(SECONDS), (int)srv_address);
		con->state = DLMS_STATE_SNRM;
	}
	break;

	case DLMS_STATE_SNRM:
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_SNRM FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
		mes_init(&messages);
	    reply_init(&reply);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Sending SNRM...\n", (int)Tick_Get(SECONDS));
	    if ((ret = cl_snrmRequest(&con->settings, &messages)) != 0 ||
	        (ret = com_readDataBlock(con, &messages, &reply)) != 0 ||
	        (ret = cl_parseUAResponse(&con->settings, &reply.data)) != 0)
	    {
	    	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Error = %s; Parsing UA...\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
	        mes_clear(&messages);
	        reply_clear(&reply);
	        retry++;
	        con->state = DLMS_STATE_ERROR;
	        return ret;
	    }
	    mes_clear(&messages);
	    reply_clear(&reply);
	    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_SNRM FS_totalAllocated:%d\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
	    con->state = DLMS_STATE_AARQ;
	break;

	case DLMS_STATE_AARQ:
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(5000);
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_AARQ FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
	    mes_init(&messages);
	    reply_init(&reply);
	    LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Sending AARQ...\r\n", (int)Tick_Get(SECONDS));
	    if ((ret = cl_aarqRequest(&con->settings, &messages)) != 0 ||
	        (ret = com_readDataBlock(con, &messages, &reply)) != 0 ||
	        (ret = cl_parseAAREResponse(&con->settings, &reply.data)) != 0)
	    {
	    	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Error = %s; Parsing AARQ...\r\n", (int)Tick_Get(SECONDS), hlp_getErrorMessage(ret));
	        mes_clear(&messages);
	        reply_clear(&reply);
	        retry++;
	    	con->state = DLMS_STATE_ERROR;
	    	return ret;
	    }
	    mes_clear(&messages);
	    reply_clear(&reply);
//	    retry = 0xFF;
	    int ret_buff = 0;
	    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_AARQ FS_totalAllocated:%d\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
	    con->settings.maxPduSize = 1024;
	    if (con->settings.maxPduSize == 0xFFFF)
	    {
	        ret_buff = con_initializeBuffers(con, con->settings.maxPduSize);
	    }
	    else
	    {
	        //Allocate 50 bytes more because some meters count this wrong and send few bytes too many.
	    	ret_buff = con_initializeBuffers(con, 50 + con->settings.maxPduSize);
	    }
	    if (DLMS_ERROR_CODE_OUTOFMEMORY == ret_buff)
	    {
	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> ERROR!! OUTOFMEMORY when Initialize buffers.\n", (int)Tick_Get(SECONDS));
	    }
	    // Get challenge Is HLS authentication is used.
	    if (con->settings.authentication > DLMS_AUTHENTICATION_LOW)
	    {
	    	con->state = DLMS_STATE_GET_CHALLENGE;
	    }
	    else
	    {
	    	con->state = DLMS_STATE_CHECK_METER_TIME;//DLMS_STATE_GET_METER_ID;//
	    }
	break;

	case DLMS_STATE_GET_CHALLENGE:
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
	    mes_init(&messages);
	    reply_init(&reply);
		if ((ret = cl_getApplicationAssociationRequest(&con->settings, &messages)) != 0 ||
			(ret = com_readDataBlock(con, &messages, &reply)) != 0 ||
			(ret = cl_parseApplicationAssociationResponse(&con->settings, &reply.data)) != 0)
		{
			mes_clear(&messages);
			reply_clear(&reply);
			retry++;
	    	con->state = DLMS_STATE_ERROR;
			return ret;
		}
		mes_clear(&messages);
		reply_clear(&reply);

		con->state = DLMS_STATE_CHECK_METER_TIME;//DLMS_STATE_GET_METER_ID;
	break;

	case DLMS_STATE_CHECK_METER_TIME:
	{
		int          ret;
		gxClock      clk;
		uint32_t     meter_clk;
		const unsigned char ln[6] = { 0, 0, 1, 0, 0, 255 };
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
#define DEVIATION_TIME (900)
		if ( ( ret = INIT_OBJECT(clk, DLMS_OBJECT_TYPE_CLOCK, ln) ) == 0 )
		{
			if ( 1== dlms_client_get_short_name_ref() )
			{
				clk.base.shortName = 0x2BC0;//0x4F00;//
			}
			ret 	  = com_read(con, BASE(clk), 2);
			meter_clk_obis_value = meter_clk = clk.time.value;
			con_dlms_getDateFormat(&meter_clk_obis_value);
			meter_clk = meter_clk_obis_value;
			if (METER_GST_SYNCH == params_get_synch_meters())
			{
				uint32_t gst_time = Tick_Get(SECONDS) + 4*3600;
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				if (( meter_clk == (gst_time) ) || (( meter_clk >= ( gst_time - DEVIATION_TIME ) ) && ( meter_clk <= ( gst_time + DEVIATION_TIME ) )))
				{
					LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Meter Clk: %d. GST Time:%d.\r\n", (int)Tick_Get(SECONDS), (int)meter_clk, (int)gst_time);
				}
				else
					if (meter_clk != gst_time)
					{
						meter_clk = clk.time.value = Tick_Get(SECONDS) + 4*3600;
						con_dlms_setDateFormat(&meter_clk);
						clk.time.value = meter_clk;
						ret            = com_write(con, BASE(clk), 2);
						if ( 1 == params_get_slow_meter() )
						{
							HAL_Delay(1000);
						}
						ret 		   = com_read(con, BASE(clk), 2);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Meter Clk GST SYNCH wr:%d rd:%d\r\n", (int)Tick_Get(SECONDS), (int)gst_time, (int)clk.time.value);
					}
			}
			else if (METER_UTC_SYNCH == params_get_synch_meters())
			{
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				if (( meter_clk == (Tick_Get(SECONDS)) ) || (( meter_clk >= ( Tick_Get(SECONDS) - DEVIATION_TIME ) ) && ( meter_clk <= ( Tick_Get(SECONDS) + DEVIATION_TIME ) )))
				{
					LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Meter Clk UTC Time: %d\r\n", (int)Tick_Get(SECONDS), (int)meter_clk);
				}
				else
				{
					meter_clk = clk.time.value = Tick_Get(SECONDS);
					con_dlms_setDateFormat(&meter_clk);
					clk.time.value = meter_clk;
					ret            = com_write(con, BASE(clk), 2);
					meter_clk      = clk.time.value;
					if ( 1 == params_get_slow_meter() )
					{
						HAL_Delay(1000);
					}
					ret 		   = com_read(con, BASE(clk), 2);
					LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Meter Clk UTC SYNCH wr:%d rd:%d\r\n", (int)Tick_Get(SECONDS), (int)Tick_Get(SECONDS), (int)clk.time.value);
				}
			}
			else if (METER_NOT_SYNCH == params_get_synch_meters())
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Meter Clk NOT SYNCH Time: %d\r\n", (int)Tick_Get(SECONDS), (int)meter_clk);
			}
			HAL_IWDG_Refresh(&hiwdg);
			shutdown_reset_watchdog();
			dlms_client_reset_indexes();
			obj_clear(BASE(clk));
		}
		retry = MAX_RETRIES;
		con->state = DLMS_STATE_GET_METER_ID;
	}
		break;

	case DLMS_STATE_GET_METER_ID:
	{
		gxObject meterID;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
//		dlms_client_on_demand_comand_set_change_secret_key(con);
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_GET_METER_ID FS_totalAllocated:%d\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_STATE_GET_METER_ID\r\n", (int)Tick_Get(SECONDS));
		if (CircularBuffer_Count(dlms_client_get_read_msg_queue()) != 0)
		{
			uint32_t obis_from_cmd = 0, generic_from_cmd = 0, data_from_cmd = 0;
			read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
			if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_DATA_CMD))
			{
				if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_MAX_DEMAND_PROF_1))
				{
//					dlms_client_set_obis_profile_from_command(1);
					obis_from_cmd = 1;
					con_read_task -= DLMS_READ_CMD_INST_PROF_1;
				}
				else if ((con_read_task >= DLMS_READ_CMD_LOAD_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
				{
//					dlms_client_set_dlms_load_profile_set_from_command(1);
					generic_from_cmd = 1;
					con_read_task -= DLMS_READ_CMD_INST_PROF_1;
				}
				else if (con_read_task == DLMS_READ_DATA_CMD)
				{
					data_from_cmd = 1;
				}
			}
			if (READ_OBIS_PROFILE)
			{
				con->state = DLMS_STATE_OBIS_1;
				dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(),  dlms_client_get_client_obis_profile(), con_read_task);
				if (1 == obis_from_cmd)
				{
					dlms_client_set_obis_profile_from_command(1);
					con->state = DLMS_STATE_ON_DEMAND_COMMAND;//
				}
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Read OBIS Profile:%d.\r\n", (int)Tick_Get(SECONDS), (int)con_read_task);
			}
			else if (READ_GENERIC_PROFILE)
			{
				con->state = DLMS_STATE_LOAD_PROFILE;
				uint32_t start_time = dlms_client_get_dlms_load_profile_start_time();
				uint32_t end_time   = dlms_client_get_dlms_load_profile_end_time();
				dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(),  dlms_client_get_client_obis_profile(), shutdown_get_first_profile(curr_device));
				dlms_client_table_read_client_generic_profile(curr_device, dlms_client_get_client(),  dlms_client_get_client_generic_profile(), con_read_task);
				dlms_client_set_dlms_load_profile_start_time( start_time );
				dlms_client_set_dlms_load_profile_end_time( end_time );
				if (1 == generic_from_cmd)
				{
					dlms_client_set_dlms_load_profile_set_from_command(1);
					con->state = DLMS_STATE_ON_DEMAND_COMMAND;//DLMS_STATE_LOAD_PROFILE;
				}
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Read GENERIC Profile:%d.\r\n", (int)Tick_Get(SECONDS), (int)con_read_task);
			}
			else if (con_read_task == DLMS_READ_DATA_CMD)
			{
				uint32_t obis_profile = 0, i = 0;
				for (i = 0; i < DLMS_OBIS_PROFILE_NUM; i++)
				{
					if ( dlms_client_get_read_time_obis_n(i) != 0 )
					{
						obis_profile = i;
						break;
					}
				}
				dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(),  dlms_client_get_client_obis_profile(), obis_profile);
				if ( 1 == data_from_cmd )
				{
					con->state = DLMS_STATE_ON_DEMAND_COMMAND;
				}
			}
			else
			{
		    	retry++;//      = MAX_RETRIES;
				con->state = DLMS_STATE_ERROR;
				return ret;
			}
		}
		else
		{
			retry++;// 	   = MAX_RETRIES;
			con->state = DLMS_STATE_ERROR;
			return -1;
		}
		ret = cosem_init(&meterID, DLMS_OBJECT_TYPE_DATA, dlms_client_get_obis_n(ID_OBIS));//"0.0.42.0.0.255"//1.1.96.1.0.255
		if ( 1== dlms_client_get_short_name_ref() )
		{
			meterID.shortName = con_get_shortName(ID_OBIS);//0xFD00;
		}
		ret = com_read(con, &meterID, 2);
		if (ret != DLMS_ERROR_CODE_OK)
		{
	    	retry++;
			con->state = DLMS_STATE_ERROR;
	    	obj_clear(&meterID);
			return ret;
		}
		char *dlms_meter_ID = dlms_client_get_obis_value_next();
		memset(dlms_client_get_meter_id(curr_device), '\0', 32*sizeof(uint8_t));
		strncpy(dlms_client_get_meter_id(curr_device), dlms_meter_ID, strlen(dlms_meter_ID));
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_GET_METER_ID Meter ID:%s. Device:%d.\r\n", (int)Tick_Get(SECONDS), dlms_client_get_meter_id(curr_device), (int)curr_device);
//		if (ret != DLMS_ERROR_CODE_OK)
//		{
//	    	retry++;
//			con->state = DLMS_STATE_ERROR;
//	    	obj_clear(&meterID);
//			return ret;
//		}

		n_obis     = 0;
		obj_clear(&meterID);

		char meter_clk_obis_data[15];
		itoa(meter_clk_obis_value, meter_clk_obis_data, 10);
		dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);
//		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_GET_METER_ID FS_totalAllocated:%d.Next State:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated(), (int)con->state);
	}
	break;

	case DLMS_STATE_OBIS_1:
	{
		gxObject obj;
		char obisdata_ln[6]       = {'0', '.', '0', '.', '9', '6'};
		char obisdata2_ln[6]      = {'1', '.', '1', '.', '9', '6'};
		char obisAeventdata_ln[9] = {'0', '.', '0', '.', '9', '7', '.','9','7'};
		char obisBeventdata_ln[9] = {'1', '.', '1', '.', '9', '7', '.','9','7'};
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		if ((0 == memcmp(dlms_client_get_obis_n(n_obis), obisdata_ln,  6))
		 || (0 == memcmp(dlms_client_get_obis_n(n_obis), obisdata2_ln, 6)))
		{
			con->state = DLMS_STATE_DATA;
			return ret;
		}
		if (0 == memcmp(dlms_client_get_obis_n(n_obis), obisAeventdata_ln, 9))
		{
			con->state = DLMS_STATE_EVENT_DATA;
			return ret;
		}
		if (0 == memcmp(dlms_client_get_obis_n(n_obis), obisBeventdata_ln, 9))
		{
			con->state = DLMS_STATE_EVENT_DATA;
			return ret;
		}
		if ( ( DLMS_READ_MAX_DEMAND_PROF_1 == dlms_client_get_obis_profile_frame_type()) || ( 1 == __checkMaxDemand(n_obis) ) )
		{
//			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_OBIS_1 FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
			if ( DLMS_READ_MAX_DEMAND_PROF_1 != dlms_client_get_obis_profile_frame_type())
			{
				dlms_log_set_profile_max_demand_extra(1);
				if (0 == dlms_client_get_obis_profile_max_demand_profile_extra())
				{
					dlms_client_set_obis_profile_max_demand_profile_extra(1);
					dlms_client_table_write_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
					dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				}
			}
			else
			{
				dlms_log_set_profile_max_demand_extra(0);
				if (dlms_client_get_obis_profile_max_demand_profile_extra() != 0)
				{
					dlms_client_set_obis_profile_max_demand_profile_extra(0);
					dlms_client_table_write_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
					dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				}
			}
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_OBJECT_TYPE_EXTENDED_REGISTER.\r\n", (int)Tick_Get(SECONDS));
			ret = cosem_init(&obj, DLMS_OBJECT_TYPE_EXTENDED_REGISTER, dlms_client_get_obis_n(n_obis++));//"1.0.32.7.0.255");
		}
		else
		{
			dlms_log_set_profile_max_demand_extra(0);
			if (dlms_client_get_obis_profile_max_demand_profile_extra() != 0)
			{
				dlms_client_set_obis_profile_max_demand_profile_extra(0);
				dlms_client_table_write_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
				dlms_client_table_read_client_obis_profile(curr_device, dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
			}
//			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_OBIS_1 FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_OBJECT_TYPE_REGISTER.\r\n", (int)Tick_Get(SECONDS));
			ret = cosem_init(&obj, DLMS_OBJECT_TYPE_REGISTER, dlms_client_get_obis_n(n_obis++));//"1.0.32.7.0.255");
		}
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			obj.shortName = shortNameAddress[n_obis - 1];
			obj.shortName = con_get_shortName(n_obis - 1);
		}

//		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> com_read 2.\r\n", (int)Tick_Get(SECONDS));
		ret = com_read(con, &obj, 2);
//		if (ret != DLMS_ERROR_CODE_OK)
//		{
//			obj_clear(&obj);
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			return ret;
//		}
		HAL_IWDG_Refresh(&hiwdg);

//		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> com_read 3\n", (int)Tick_Get(SECONDS));
		if (( DLMS_READ_MAX_DEMAND_PROF_1 == dlms_client_get_obis_profile_frame_type()) || ( 1 == __checkMaxDemand(n_obis - 1) ))
		{
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, &obj, 5);
			dlms_client_dec_next_obis_val();
//			if (ret != DLMS_ERROR_CODE_OK)
//			{
//				obj_clear(&obj);
//				retry++;
//				con->state = DLMS_STATE_ERROR;
//				return ret;
//			}
			HAL_IWDG_Refresh(&hiwdg);
		}

		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}

		ret = com_read(con, &obj, 3);
		if (ret != DLMS_ERROR_CODE_OK)
		{
//			obj_clear(&obj);
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			return ret;
			dlms_client_set_obis_value("  ", 2, 2);
		}
		HAL_IWDG_Refresh(&hiwdg);

		if (( *(dlms_client_get_obis_n(n_obis)) == '\0' ) || (n_obis >= (DLMS_OBIS_MAX_NUM_VALUES-2)) )
		{
//			char meter_clk_obis_data[15];
//			itoa(meter_clk_obis_value, meter_clk_obis_data, 10);
//			dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);

			dlms_log_write_lock(0);
			dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
			CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
//			if (( CircularBuffer_Read(dlms_client_get_send_msg_queue()) >= (dlms_client_get_obis_profile_frame_type() + DLMS_WAIT_FOR_SEND_INST_PROF_1) )
//			 && ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) <= (dlms_client_get_obis_profile_frame_type() + DLMS_WAIT_FOR_SEND_MAX_DEMAND_PROF_1) ))
//			{
//    			shutdown_setInitTelitModule(1);
//				CircularBuffer_Get(dlms_client_get_send_msg_queue());
//    			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_client_get_obis_profile_frame_type());
//    			message_queue_write(SEND_MODBUS_SENSOR);
//    			shutdown_reset_watchdog();
//    		}
			if ( 0 == dlms_client_get_disconnected() )
			{
				con->state = DLMS_STATE_DISCONNECT;
			}
			else if ( dlms_client_get_disconnected() != 0 )
			{
				con->state = DLMS_DISCONNECT_CONTROL;
			}
			obj_clear(&obj);
//			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_OBIS_1 FS_totalAllocated:%d. Send Meter:%d.\r\n",
//					(int)Tick_Get(SECONDS), (int)getFS_totalAllocated(), (int)dlms_client_get_send_meter());
		}
	}
	break;

	case DLMS_STATE_DATA:
	{
		gxObject obisdata;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = cosem_init(&obisdata, DLMS_OBJECT_TYPE_DATA, dlms_client_get_obis_n(n_obis++));//"0.0.42.0.0.255"//1.1.96.1.0.255
		if ( 1== dlms_client_get_short_name_ref() )
		{
			obisdata.shortName = con_get_shortName(n_obis-1);//0xFD00;
		}
		ret = com_read(con, &obisdata, 2);
		if (ret != DLMS_ERROR_CODE_OK)
		{
			retry++;
			con->state = DLMS_STATE_ERROR;
			obj_clear(&obisdata);
			return ret;
		}
		char obisdata_ln[11] = {'0', '.', '0', '.', '9', '6', '.', '7', '.', '1', '0'};
		if (0 == memcmp(dlms_client_get_obis_n(n_obis-1), obisdata_ln, 11))
		{
	    	char  obis_trans[100];
	    	gxData * object = (gxData *)&obisdata;
	        __getDate((char *)object->value.strVal->data, obis_trans);
	    	dlms_client_dec_next_obis_val();
	    	dlms_client_append_obis_value(obis_trans, 2);
		}
		if (( *(dlms_client_get_obis_n(n_obis)) == '\0' ) || (n_obis >= (DLMS_OBIS_MAX_NUM_VALUES-2)) )
		{
//			char meter_clk_obis_data[15];
//			itoa(meter_clk_obis_value, meter_clk_obis_data, 10);
//			dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);

			dlms_log_write_lock(0);
			dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
			CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
			if ( 0 == dlms_client_get_disconnected() )
			{
				con->state = DLMS_STATE_DISCONNECT;
			}
			else if ( dlms_client_get_disconnected() != 0 )
			{
				con->state = DLMS_DISCONNECT_CONTROL;
			}
//			obj_clear(&obisdata);
		}
		obj_clear(&obisdata);
	}
	break;

	case DLMS_STATE_EVENT_DATA:
	{
		gxObject obisdata;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = cosem_init(&obisdata, DLMS_OBJECT_TYPE_DATA, dlms_client_get_obis_n(n_obis++));

		ret = com_read2(con, &obisdata, 2);
//		if (ret != DLMS_ERROR_CODE_OK)
//		{
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			obj_clear(&obisdata);
//			return ret;
//		}
		if (( *(dlms_client_get_obis_n(n_obis)) == '\0' ) || (n_obis >= (DLMS_OBIS_MAX_NUM_VALUES-2)) )
		{
//			char meter_clk_obis_data[15];
//			itoa(meter_clk_obis_value, meter_clk_obis_data, 10);
//			dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);

			dlms_log_set_event_ff_frame(1);
			dlms_log_write_lock(0);
			dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
			CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
			if ( 0 == dlms_client_get_disconnected() )
			{
				con->state = DLMS_STATE_DISCONNECT;
			}
			else if ( dlms_client_get_disconnected() != 0 )
			{
				con->state = DLMS_DISCONNECT_CONTROL;
			}
		}
		obj_clear(&obisdata);
	}
		break;

	case DLMS_DISCONNECT_CONTROL:
		con->state = DLMS_STATE_DISCONNECT;
		break;

	case DLMS_STATE_METER_SYNCH:

		break;

	case DLMS_STATE_LOAD_PROFILE:
	{
		int          ret;
		const char * obis;
		struct tm    start_time;
		struct tm    end_time;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
#define NUM_EVENTS (10)
#define INIT_HOUR (3)//(24)
#define END_HOUR  (2)//(23)
		read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
		if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
		{
			con_read_task -= DLMS_READ_CMD_INST_PROF_1;
		}
		switch (con_read_task) {
		case DLMS_READ_LOAD_PROF_1:
			dlms_client_set_dlms_load_profile_end_time( Tick_Get(SECONDS)   - END_HOUR*3600 );
			dlms_client_set_dlms_load_profile_start_time( Tick_Get(SECONDS) - INIT_HOUR*3600);
			break;
		case DLMS_READ_LOAD_PROF_2:
			dlms_client_set_dlms_load_profile_end_time( Tick_Get(SECONDS)   - END_HOUR*3600  );
			dlms_client_set_dlms_load_profile_start_time( Tick_Get(SECONDS) - INIT_HOUR*3600);
			break;
		case DLMS_READ_POWER_QUALITY_PROF:
			dlms_client_set_dlms_load_profile_end_time( Tick_Get(SECONDS)   - END_HOUR*3600  );
			dlms_client_set_dlms_load_profile_start_time( Tick_Get(SECONDS) - INIT_HOUR*3600);
			break;
		case DLMS_READ_INSTRUMENTATION_PROF_1:
			dlms_client_set_dlms_load_profile_end_time( Tick_Get(SECONDS)   - END_HOUR*3600  );
			dlms_client_set_dlms_load_profile_start_time( Tick_Get(SECONDS) - INIT_HOUR*3600);
			break;
		case DLMS_READ_BILLING_PROF:
			if ( 0 == strncmp("1.1.1.8.0.01", dlms_client_get_generic_obis_n(0), 12) )
			{
				ret = DLMS_ERROR_CODE_OK;
				con->state = DLMS_STATE_BILLING_TUNNELING;
				return ret;
			}
			else
			{
				dlms_client_set_dlms_load_profile_end_time( Tick_Get(SECONDS) );
				dlms_client_set_dlms_load_profile_start_time( Tick_Get(SECONDS) - 1*31*24*3600 );
			}
			break;
		case DLMS_READ_EVENT_LOG_PROF:
			dlms_client_set_dlms_load_profile_end_time( Tick_Get(SECONDS) /* - END_HOUR*3600 */  );//+ 4*3600 );
			dlms_client_set_dlms_load_profile_start_time( Tick_Get(SECONDS) - /* INIT_HOUR*/3600  );//+ 3*3600 );
			break;
		default:
			break;
		}
		LOGLIVE(LEVEL_1, "\r\nLOGLIVE> %d DLMS> DLMS_STATE_LOAD_PROFILE FS_totalAllocated:%d\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());

		obis       = dlms_client_get_generic_obis_n(0);
		start_time = *con_convert_time(dlms_client_get_dlms_load_profile_start_time());
		end_time   = *con_convert_time(dlms_client_get_dlms_load_profile_end_time());
		if ( ( ret = cosem_init(BASE(pg), DLMS_OBJECT_TYPE_PROFILE_GENERIC, obis) ) == 0 )
		{
			if ( 1 == dlms_client_get_short_name_ref() )
			{
				if (udp_protocol_get_on_billing_profile())
				{
					pg.base.shortName = 0x8000;//0x6400;
					pg.base.shortName = con_get_generic_profile_shortName();
				}
				else if (udp_protocol_get_on_event_profile())
				{
					pg.base.shortName = 0x8080;//0x6270;
					pg.base.shortName = con_get_generic_profile_shortName();
				}
				else
				{
					pg.base.shortName = 0x5000;//0x60E0;
					pg.base.shortName = con_get_generic_profile_shortName();
				}
			}
			ret = com_readValue(con, BASE(pg), 1);
			if (ret == DLMS_ERROR_CODE_SEND_FAILED)
			{
//				retry++;
				con->state = DLMS_STATE_ERROR;
			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			//Read entries in use.
			ret = com_read(con, BASE(pg), 7);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Entries in use Att 7 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
			HAL_IWDG_Refresh(&hiwdg);
			if (ret == DLMS_ERROR_CODE_SEND_FAILED)
			{
				retry++;
				con->state = DLMS_STATE_ERROR;
				obj_clear(BASE(pg));
				return ret;
			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			//Read entries.
			ret = com_read(con, BASE(pg), 8);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read entries Att 8 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);

//			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Att 2\r\n", (int)Tick_Get(SECONDS));
//			ret = com_readValue(con, BASE(pg), 2);
			HAL_IWDG_Refresh(&hiwdg);
			if (ret == DLMS_ERROR_CODE_SEND_FAILED)
			{
				retry++;
				con->state = DLMS_STATE_ERROR;
				obj_clear(BASE(pg));
				return ret;
			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, BASE(pg), 4);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Att 4 Capture Period ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
			HAL_IWDG_Refresh(&hiwdg);
//			if (ret == DLMS_ERROR_CODE_SEND_FAILED)
//			{
//				retry++;
//				con->state = DLMS_STATE_ERROR;
//				obj_clear(BASE(pg));
//				return ret;
//			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, BASE(pg), 3);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Att 3 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
			HAL_IWDG_Refresh(&hiwdg);
//			if (ret == DLMS_ERROR_CODE_SEND_FAILED)
//			{
//				retry++;
//				con->state = DLMS_STATE_ERROR;
//				obj_clear(BASE(pg));
//				return ret;
//			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, BASE(pg), 5);
			LOGLIVE(LEVEL_2, "\r\nLOGLIVE> %d DLMS> Att 5 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
			HAL_IWDG_Refresh(&hiwdg);
//			if (ret == DLMS_ERROR_CODE_SEND_FAILED)
//			{
//				retry++;
//				con->state = DLMS_STATE_ERROR;
//				obj_clear(BASE(pg));
//				return ret;
//			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			start_time.tm_sec   = 0;
			end_time.tm_sec     = 0;
			ret 	   = 0;
			if (( DLMS_READ_EVENT_LOG_PROF == con_read_task ) && (pg.profileEntries >= NUM_EVENTS))
			{
				uint32_t first_read = 0;
				if (DLMS_SORT_METHOD_LIFO == pg.sortMethod)
				{
					first_read = 0;
				}
				else if (DLMS_SORT_METHOD_FIFO == pg.sortMethod)
				{
					first_read = pg.profileEntries - NUM_EVENTS;
				}
				ret        = com_readRowsByEntry(con, &pg, first_read, NUM_EVENTS+1);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Rows. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
				if (ret != DLMS_ERROR_CODE_OK)
				{
					ret    = com_readRowsByRange(con, &pg, &start_time, &end_time);
					LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Range. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
				}
			}
			else
			{
				ret        = com_readRowsByRange(con, &pg, &start_time, &end_time);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Range. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
			}
			if (ret != DLMS_ERROR_CODE_OK)
			{
				HAL_IWDG_Refresh(&hiwdg);
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret        = com_readRowsByEntry(con, &pg, 2, 1);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Rows. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
				if ((ret != DLMS_ERROR_CODE_OK) && (ret != DLMS_ERROR_CODE_REJECTED))
				{
					HAL_IWDG_Refresh(&hiwdg);
					if ( 1 == params_get_slow_meter() )
					{
						HAL_Delay(1000);
					}
					ret = com_read_profile_object(con, BASE(pg), 2);
					HAL_IWDG_Refresh(&hiwdg);

					if (ret != DLMS_ERROR_CODE_OK)
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Profile Object. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
					}
					else
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Profile Object OK.\r\n", (int)Tick_Get(SECONDS));
					}
				}
				if ((ret != DLMS_ERROR_CODE_OK) && (retry > MAX_RETRIES))
				{
					udp_protocol_set_on_billing_profile(0);
					udp_protocol_set_on_event_profile(0);
				}
			}
			HAL_IWDG_Refresh(&hiwdg);
			shutdown_reset_watchdog();
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> OBJ PROFILE GENERIC CLEAR FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
//			obj_clear(BASE(pg));
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_LOAD_PROFILE FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
		}
		if ( ret != 0 )
		{
			if (( 1 == udp_protocol_get_on_demand_command() ))
			{
//				shutdown_set_tamper_param(TAMPER_MQTT_COMMAND_ERROR);
			}
			retry++;
			con->state = DLMS_STATE_ERROR;
		}
		else
		{
			con->state = DLMS_STATE_DISCONNECT;
		}
	}
		break;

	case DLMS_STATE_ON_DEMAND_COMMAND:
		dlms_client_on_demand_comm_exc(dlms_client_get_command(), con);
		dlms_client_on_demand_cmd_check_send();
		if (( 0 == on_demand_cmd ) && ( 0 == (con_get_num_items_device() - 1) ) )//on_demand_cmd used for on demand profiles
		{
			dlms_client_table_inc_curr_meter_from_cmd();
			if (con_dlms_get_next_device_for_cmd(dlms_client_table_get_curr_meter_from_cmd()) != 0xFF)
			{
				on_demand_cmd = 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Next CMD Meter:%d. \r\n",
				(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//				dlms_client_table_inc_curr_meter_from_cmd();
				if ((dlms_client_table_get_curr_meter_from_cmd()+1)>=dlms_client_table_get_num_meters_from_cmd())
				{
//					con_dlms_set_curr_device(0);
					mqtt_stop();
				}
			}
			else
			{
				on_demand_cmd_data = 1;
				con_dlms_set_curr_device(0);
				mqtt_stop();
			}
		}
//		else
//		{
//			mqtt_stop();
////			if (DLMS_STATE_ERROR != con->state)
////			{
////				con_dlms_set_curr_device(0);
////				on_demand_cmd = 0;
////			}
//		}
		break;

	case DLMS_STATE_BILLING_TUNNELING:
	{
		gxData tunneling;
		const unsigned char ln_tunn[6] = { 1, 1, 150, 0, 0, 255 };

		if ( ( ret = INIT_OBJECT(tunneling, DLMS_OBJECT_TYPE_DATA, ln_tunn) ) == 0 )
		{
			static char INIT[6];
			sprintf(INIT,"/?!\r\n");
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
			tunneling.value.strVal->capacity = sizeof(INIT);
			tunneling.value.strVal->size = 5;
			tunneling.value.strVal->data = (u_char *)INIT;
			com_write(con, BASE(tunneling), 2);
			com_read(con, BASE(tunneling), 2);
//			gxfree(tunneling.value.strVal);
#if 1
			static char ACK[7];
			ACK[0] = 6;
			sprintf(&ACK[1],"051\r\n");
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(ACK);
			tunneling.value.strVal->size = 6;
			tunneling.value.strVal->data = (u_char *)ACK;
			com_write(con, BASE(tunneling), 2);
			com_read(con, BASE(tunneling), 2);

			static char P1[17];
			P1[0] = 1;
			sprintf(&P1[1],"P1");
			P1[3] = 2;
			sprintf(&P1[4],"(22222222)");
			P1[14] = 3;
			P1[15] = __CalculateChecksum(P1, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(P1);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)P1;
			com_write(con, BASE(tunneling), 2);
			com_read(con, BASE(tunneling), 2);

			dlms_client_reset_indexes();

			dlms_client_set_obis_value(dlms_client_get_meter_id(curr_device), strlen(dlms_client_get_meter_id(curr_device)), 2);
			char meter_clk_obis_data[15];
			itoa(meter_clk_obis_value, meter_clk_obis_data, 10);
			dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);

			static char R5[17];
			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"1.8.0*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"1.8.1*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"1.8.2*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"1.8.3*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"1.8.4*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"2.8.0*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"3.8.0*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"1.6.0*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

			R5[0] = 1;
			sprintf(&R5[1],"R5");
			R5[3] = 2;
			sprintf(&R5[4],"9.6.0*01()");
			R5[14] = 3;
			R5[15] = __CalculateChecksum(R5, 1, 14 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(R5);
			tunneling.value.strVal->size = 16;
			tunneling.value.strVal->data = (u_char *)R5;
			com_write(con, BASE(tunneling), 2);
			com_read3(con, BASE(tunneling), 2);

//			char meter_clk_obis_data[15];
//			itoa(meter_clk_obis_value, meter_clk_obis_data, 10);
//			dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);

			static char B0[6];
			B0[0] = 1;
			sprintf(&B0[1],"B0");
			B0[3] = 3;
			B0[4] = __CalculateChecksum(B0, 1, 3 );
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			tunneling.value.strVal->capacity = sizeof(B0);
			tunneling.value.strVal->size = 5;
			tunneling.value.strVal->data = (u_char *)B0;
			com_write(con, BASE(tunneling), 2);
//			com_read(con, BASE(tunneling), 2);
#endif
			HAL_IWDG_Refresh(&hiwdg);
//			gxfree(tunneling.value.strVal);
			tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
			obj_clear(BASE(tunneling));
//			gxfree(&tunneling.base);

			dlms_log_write_lock(0);
			dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
			CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
		}
		con->state = DLMS_STATE_DISCONNECT;
	}
		break;

	case DLMS_STATE_ERROR:
	case DLMS_STATE_DISCONNECT:
	{
//		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> DLMS_STATE_DISCONNECT FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
//		write_task_t con_write_task = (write_task_t)CircularBuffer_Read(dlms_client_get_write_msg_queue());
//		if ((wait_close < 2) || WRITE_OBIS_PROFILE)
//		{
//			wait_close++;
//			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> DLMS_STATE_DISCONNECT WAIT CLOSE:%d.\r\n", (int)Tick_Get(SECONDS), (int)wait_close);
//			break;
//		}
//		else
//		{
//			wait_close = 0;
//		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		com_close(con);
		con_close(con);
//		dlms_client_set_dlms_comm_state(DLMS_COMM_END);
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_DISCONNECT free FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
		mfree_all();
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> mfree_all FS_totalAllocated:%d.Send Meter:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated(), (int)dlms_client_get_send_meter());
		if (DLMS_STATE_ERROR == con->state)
		{
			if (retry <= MAX_RETRIES)
			{
				con->state = DLMS_STATE_INIT;
			}
			else
			{
				if (1 == on_demand_cmd)
				{
	    			if (con_dlms_get_next_device_for_cmd(curr_device+1) != 0xFF)
					{
						device_from_cmd_index++;
						retry = 0;
						con->state = DLMS_STATE_INIT;
					}
					else
					{
						LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Error = %d\n", (int)Tick_Get(SECONDS), ret);
	    				device_from_cmd_index = 0;
	    				on_demand_cmd         = 0;
						dlms_client_set_dlms_comm_state(DLMS_COMM_END);
						con_dlms_set_curr_device(0);
						con->state = DLMS_STATE_IDLE;
						mqtt_stop();
					}
				}
				else if ( (con_dlms_get_curr_device() + 1) < dlms_client_table_get_num_devices() )
				{
					if ( 0 == num_items_device )
					{
						con_dlms_inc_curr_device();
						retry = 0;
						con->state = DLMS_STATE_IDLE;
					}
					else
					{
						retry = 0;
						con->state = DLMS_STATE_INIT;
				    	num_items_device = num_items_device - 1;
				    	if (0 == num_items_device)
				    	{
				    		if ( (con_dlms_get_curr_device() + 1) < dlms_client_table_get_num_devices() )
				    		{
				    			con_dlms_inc_curr_device();//curr_device++;
				    		}
				    		else
				    		{
				    			con_dlms_set_curr_device(0);
				    		}
				    		con->state = DLMS_STATE_IDLE;
				    	}
					}
				}
				else
				{
					con->state = DLMS_STATE_IDLE;
					num_items_device = num_items_device - 1;
			    	if (0 == num_items_device)
			    	{
			    		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Error = %d\n", (int)Tick_Get(SECONDS), ret);
			    		dlms_client_set_dlms_comm_state(DLMS_COMM_END);
			    		con_dlms_set_curr_device(0);
			    		con->state = DLMS_STATE_IDLE;
//			    		mqtt_stop();
			    	}
				}
				if ( CircularBuffer_Read(dlms_client_get_read_msg_queue()) == (CircularBuffer_Read(dlms_client_get_send_msg_queue()) - DLMS_WAIT_FOR_SEND_INST_PROF_1))
				{
					LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Get Send Queue.\r\n", (int)Tick_Get( SECONDS ));
					CircularBuffer_Get(dlms_client_get_send_msg_queue());
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Get Read Queue.\r\n", (int)Tick_Get( SECONDS ));
				CircularBuffer_Get(dlms_client_get_read_msg_queue());
//				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Get Send Queue.\r\n", (int)Tick_Get( SECONDS ));
//				CircularBuffer_Get(dlms_client_get_send_msg_queue());
				if (!READ_MSG_PENDING)
				{
		    		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Error = %d\n", (int)Tick_Get(SECONDS), ret);
		    		num_items_device = 0;
		    		dlms_client_set_dlms_comm_state(DLMS_COMM_END);
		    		con_dlms_set_curr_device(0);
		    		con->state = DLMS_STATE_IDLE;
					mqtt_stop();
					send_task_t con_send_task = (send_task_t)CircularBuffer_Read(dlms_client_get_send_msg_queue());
					if ( !(WRITE_MSG_PENDING) && !(SEND_MSG_PENDING) && !(READ_MSG_PENDING))
					{
						if ( 0 == message_queue_get_elements() )
						{
						shutdown_set(1, params_config_read_time());
						}
					}
				}
			}
		}
		else
		{
	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Error = %d\n", (int)Tick_Get(SECONDS), ret);
	    	dlms_client_set_dlms_comm_state(DLMS_COMM_END);
	    	con->state = DLMS_STATE_IDLE;
	    	CircularBuffer_Get(dlms_client_get_read_msg_queue());
	    	num_items_device = num_items_device - 1;
	    	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Get Read Queue. Num items device:%d.\r\n", (int)Tick_Get( SECONDS ), (int)num_items_device);
	    	if (0 == num_items_device)
	    	{
	    		if (0 == on_demand_cmd)
	    		{
	    			if ( (con_dlms_get_curr_device() + 1) < dlms_client_table_get_num_devices() )
	    			{
	    				if (0 == on_demand_cmd_data)
	    				{
	    					con_dlms_inc_curr_device();//curr_device++;
	    				}
	    				else
	    				{
	    					on_demand_cmd_data = 0;
	    				}
	    			}
	    			else
	    			{
	    				con_dlms_set_curr_device(0);
	    				mqtt_stop();
	    				send_task_t con_send_task = (send_task_t)CircularBuffer_Read(dlms_client_get_send_msg_queue());
	    				if ( !(WRITE_MSG_PENDING) && !(SEND_MSG_PENDING) && !(READ_MSG_PENDING))
						{
	    					if ( 0 == message_queue_get_elements() )
	    					{
	    					shutdown_set(1, params_config_read_time());
	    					}
						}
	    			}
	    		}
	    		else
	    		{
//	    			if (con_dlms_get_next_device_for_cmd(curr_device+1) != 0xFF)
//	    			{
//	    				device_from_cmd_index++;
//	    			}
//	    			else
//	    			{
//	    				device_from_cmd_index = 0;
//	    				on_demand_cmd         = 0;
//	    			}
	    		}
	    	}
		}
		return ret;
	}
		break;

	default:
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Error = %d\n", (int)Tick_Get(SECONDS), ret);
    	dlms_client_set_dlms_comm_state(DLMS_COMM_END);
    	con->state = DLMS_STATE_IDLE;
		return ret;
	break;

	}

	return ret;
}
