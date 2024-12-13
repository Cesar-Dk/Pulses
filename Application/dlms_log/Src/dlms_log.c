/*
 * dlms_log.c
 *
 *  Created on: 20 may. 2021
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <inttypes.h>
#include "common_lib.h"

#include "dlms_log.h"
#include "dlms_client.h"
#include "dlms_client_table.h"
#include "une82326.h"
#include "mbus.h"
#include "rtc_system.h"
#include "modbus.h"
#include "ad.h"
#include "gpio.h"
#include "spi_flash.h"
#include "tick.h"
#include "shutdown.h"
#include "params.h"
#include "ME910.h"
#include "udp_protocol.h"
#include "mqtt_task.h"

typedef struct _modbus_vals {
	uint32_t init_val;
	uint32_t samples;
} dlms_param_value;

/** Structure to save the raw MODBUS telegram*/
typedef struct _raw_modbus {
	char                raw_dlms_telegram[1024];	/*!< MODBUS raw data array */
	uint32_t            len;						/*!< length of the raw telegram */
	uint32_t            msg_num;					/*!< telegram counter */
	uint32_t            msg_num_backup;				/*!< telegram counter backup */
} raw_dlms;

/** Structure to save the MODBUS sensor data, pointers and counters in flash
 * this is saved to _modbus_start_address = 0x780000
 * */
struct _dlms_log {
	uint32_t      	    samples;			/*!< */
	uint32_t      	    init;				/*!<  */
	uint32_t            num_params;			/*!<  */
	uint32_t            msg_wr_address;		/*!<  */
	uint32_t            msg_rd_address;		/*!<  */
	uint32_t            msg_rd_address_backup;	/*!<  */
    uint32_t            msg_rd_ini_addr;	/*!<  */
    uint32_t 		    msg_rd_end_addr;	/*!<  */
    uint32_t 		    msg_rd_end_addr_backup;	/*!<  */
	uint32_t            msg_load_wr_address;		/*!<  */
	uint32_t            msg_load_rd_address;		/*!<  */
	uint32_t            msg_load_rd_address_backup;	/*!<  */
    uint32_t            msg_load_rd_ini_addr;	/*!<  */
    uint32_t 		    msg_load_rd_end_addr;	/*!<  */
    uint32_t 		    msg_load_rd_end_addr_backup;	/*!<  */
	uint32_t            msg_events_wr_address;		/*!<  */
	uint32_t            msg_events_rd_address;		/*!<  */
	uint32_t            msg_events_rd_address_backup;	/*!<  */
    uint32_t            msg_events_rd_ini_addr;	/*!<  */
    uint32_t 		    msg_events_rd_end_addr;	/*!<  */
    uint32_t 		    msg_events_rd_end_addr_backup;	/*!<  */
	dlms_param_value    dlms_val[4];		/*!<  */
	raw_dlms            raw;				/*!< structure that contains raw telegram */
} dlms_log;

typedef struct {
	char      raw_dlms[MAX_DLMS_TELEGRAM_VALUES + 1];
	char      created_sensor_value[7];
	char      created_sensor_value_date[9];
	char      dlms_meter_id[32];
	uint32_t  frame_type;
	uint32_t  frame_event_ff;
	uint32_t  frame_tunneling;
	uint32_t  profile_max_demand_extra;
	uint32_t  msg_num;
	uint32_t  raw_len;
	uint32_t  wr_addr;
	uint32_t  rd_addr;
	uint32_t  is_command;
	uint32_t  num_generic_obis;
	char      dlms_obis_capture_objects[42][24];
	uint32_t  capture_period;
	uint32_t  num_meter;
} dlms_telegram_values;//#define NITEMS(x) (sizeof(x)/sizeof(x[0]))

dlms_telegram_values dlms_telegram_value;

char dlms_telegram_string[MAX_DLMS_TELEGRAM_VALUES + 1 + 3 + 50];

extern char _modbus_start_address[], _modbus_meas_size[];

#define INIT  (0)
#define INDEX (1)

#define DLMS_PAGE_SIZE (sFLASH_SPI_PAGESIZE)

//uint32_t dlms_page_address = (uint32_t)_modbus_start_address + 0x1000;
//float_t  dlms_page[ DLMS_PAGE_SIZE ];
#define DLMS_LOG_SIZE            (0x2000)
#define DLMS_ENERGY_SIZE         (0x100000)
#define DLMS_LOAD_PROFILE_SIZE   (0x210000)//(0x1E0000)//(0x100000)//
#define DLMS_EVENTS_PROFILE_SIZE (0x2BFFF)//(0x20000)//

uint32_t  dlms_telegram_values_address       = (uint32_t)_modbus_start_address + DLMS_LOG_SIZE + DLMS_SLAVES_TABLE_SIZE;//0x2000;
uint32_t  dlms_loadprofile_address           = (uint32_t)_modbus_start_address + DLMS_LOG_SIZE + DLMS_SLAVES_TABLE_SIZE + DLMS_ENERGY_SIZE;//0x200000 - 0x2000;
uint32_t  dlms_eventsprofile_address         = (uint32_t)_modbus_start_address + DLMS_LOG_SIZE + DLMS_SLAVES_TABLE_SIZE + DLMS_ENERGY_SIZE + DLMS_LOAD_PROFILE_SIZE;//0x200000 - 0x2000;
uint32_t  dlms_telegram_values_max_address   = (uint32_t)_modbus_start_address + DLMS_LOG_SIZE + DLMS_SLAVES_TABLE_SIZE + DLMS_ENERGY_SIZE + DLMS_LOAD_PROFILE_SIZE - 1;//0x200000 - 1;
uint32_t  dlms_telegram_values_events_max_address = (uint32_t)_modbus_start_address + DLMS_LOG_SIZE + DLMS_SLAVES_TABLE_SIZE + DLMS_ENERGY_SIZE + DLMS_LOAD_PROFILE_SIZE + DLMS_EVENTS_PROFILE_SIZE - 1;//0x200000 - 1;
static uint32_t   dlms_telegram_log_size   = 0, dlms_telegram_log_profile_size   = 0, dlms_telegram_log_events_profile_size   = 0;

char dataCmdOnDemand[40];

uint32_t dlms_event_ff_frame = 0;

static uint8_t __programWord( void );
static uint8_t __programTelegramValueWord( void );
static uint8_t __readDLMSLog( void );
static void    __incReadDLMSRawEventsProfileLog( void );

uint8_t dlms_log_init(void)
{
	HAL_Delay(10);
	__readDLMSLog();
	if (dlms_log.init != 0xAAAA)
	{
		dlms_log.init               = 0xAAAA;
		dlms_log.raw.msg_num        = 0;
		dlms_log.msg_wr_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_ini_addr    = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr    = dlms_telegram_values_address;
		dlms_log.msg_rd_address_backup = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr_backup = dlms_telegram_values_address;
		dlms_log.msg_load_wr_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_ini_addr    = dlms_loadprofile_address;
		dlms_log.msg_load_rd_end_addr    = dlms_loadprofile_address;
		dlms_log.msg_load_rd_address_backup = dlms_loadprofile_address;
		dlms_log.msg_load_rd_end_addr_backup = dlms_loadprofile_address;
		dlms_log.msg_events_wr_address     = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_address     = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_ini_addr    = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_end_addr    = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_address_backup = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_end_addr_backup = dlms_eventsprofile_address;
		dlms_telegram_value.msg_num = 0;
		dlms_telegram_value.wr_addr = dlms_telegram_values_address;
		dlms_telegram_value.rd_addr = dlms_telegram_values_address;

		__programWord();
	}
	return 0;
}

uint32_t dlms_log_set_event_ff_frame( uint32_t __ff_frame )
{
	dlms_event_ff_frame = __ff_frame;
	return 0;
}

uint32_t dlms_log_get_frame_tunneling( void )
{
//	sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
//			dlms_log.msg_load_rd_ini_addr,
//			sizeof(dlms_telegram_value) );
	return dlms_telegram_value.frame_tunneling;
}

void dlms_log_set_frame_tunneling( uint32_t _tunn )
{
	dlms_telegram_value.frame_tunneling = _tunn;
}

uint32_t dlms_log_get_profile_max_demand_extra( void )
{
	return dlms_telegram_value.profile_max_demand_extra;
}

void dlms_log_set_profile_max_demand_extra( uint32_t __xtra )
{
	dlms_telegram_value.profile_max_demand_extra = __xtra;
}

char * dlms_log_get_dataCmdOnDemand( void )
{
	return dataCmdOnDemand;
}

uint32_t dlms_log_set_dataCmdOnDemand( char * __dataCmdOnDemand)
{
	strncpy((char *) dataCmdOnDemand, __dataCmdOnDemand, sizeof(dataCmdOnDemand));

	return 0;
}

uint8_t dlms_log_CheckDataInMem( void )
{
	uint8_t ret = ( dlms_log.msg_rd_ini_addr != dlms_log.msg_rd_end_addr )?1:0;

	return ret;
}

uint8_t dlms_log_CheckProfileDataInMem( void )
{
	uint8_t ret = ( dlms_log.msg_load_rd_ini_addr != dlms_log.msg_load_rd_end_addr )?1:0;

	return ret;
}

uint8_t dlms_log_CheckEventsProfileDataInMem( void )
{
	uint8_t ret = ( dlms_log.msg_events_rd_ini_addr != dlms_log.msg_events_rd_end_addr )?1:0;

	return ret;
}

uint32_t dlms_log_get_telegram_value_frame_type( void )
{
	return dlms_telegram_value.frame_type;
}

uint32_t dlms_log_get_telegram_value_num_meter( void )
{
	return dlms_telegram_value.num_meter;
}

uint32_t dlms_log_get_telegram_value_capture_period( void )
{
	return dlms_telegram_value.capture_period;
}

void dlms_log_set_telegram_value_capture_period( uint32_t _capture_period )
{
	dlms_telegram_value.capture_period = _capture_period;
}

/**
  * @brief Writes structure modbus_sensor_log in sector of address _modbus_start_address (0x780000)
  * @retval 0
  */
static uint8_t __programWord( void )
{
	uint32_t  size, ret = 0;
	uint8_t  *ptr;

    size = sizeof(dlms_log);
    ptr  = (uint8_t *) &dlms_log;

    sFLASH_EraseSector((uint32_t)_modbus_start_address);
    sFLASH_WriteBuffer(ptr, (uint32_t)_modbus_start_address, size);

    sFLASH_ReadBuffer((uint8_t *) &dlms_log, (uint32_t) _modbus_start_address, sizeof(dlms_log));

    return ret;
}

/**
  * @brief Reads MODBUS sensor data from FLASH memory and stores it to modbus_sensor_log structure.
  * @retval 0
  */
static uint8_t __readDLMSLog(void)
{
	sFLASH_ReadBuffer((uint8_t *) &dlms_log, (uint32_t) _modbus_start_address, sizeof(dlms_log));

	return 0;
}

static uint32_t __nextLoadProfileAddress(unsigned int n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr   = dlms_log.msg_load_wr_address + n;
	next_sector = ( dlms_log.msg_load_wr_address + 0x1000 ) & 0xFFFFF000;

    if ( ( dlms_log.msg_load_wr_address & 0x00000FFF ) == 0 ) {
    	sFLASH_EraseSector(dlms_log.msg_load_wr_address);
    } else if (  next_addr > next_sector ) {
    	sFLASH_EraseSector( next_sector );
    }

    addr = dlms_log.msg_load_wr_address;
    dlms_log.msg_load_wr_address  += n;
    if ( dlms_log.msg_load_wr_address > dlms_telegram_values_max_address ) {
    	dlms_log.msg_load_wr_address = dlms_loadprofile_address;
    }

    if ( ( dlms_log.msg_load_wr_address == dlms_log.msg_load_rd_end_addr ) ) {
    	dlms_log.msg_load_rd_end_addr += dlms_telegram_log_profile_size;
    }
    dlms_log.msg_load_rd_ini_addr = dlms_log.msg_load_wr_address;
    dlms_log.msg_load_rd_address  = dlms_log.msg_load_wr_address;

    return addr;
}

static uint32_t __nextEventsProfileAddress(unsigned int n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr   = dlms_log.msg_events_wr_address + n;
	next_sector = ( dlms_log.msg_events_wr_address + 0x1000 ) & 0xFFFFF000;

    if ( ( dlms_log.msg_events_wr_address & 0x00000FFF ) == 0 ) {
    	sFLASH_EraseSector(dlms_log.msg_events_wr_address);
    } else if (  next_addr > next_sector ) {
    	sFLASH_EraseSector( next_sector );
    }

    addr = dlms_log.msg_events_wr_address;
    dlms_log.msg_events_wr_address  += n;
    if ( dlms_log.msg_events_wr_address > dlms_telegram_values_events_max_address ) {
    	dlms_log.msg_events_wr_address = dlms_eventsprofile_address;
    }

    if ( ( dlms_log.msg_events_wr_address == dlms_log.msg_events_rd_end_addr ) ) {
    	dlms_log.msg_events_rd_end_addr += dlms_telegram_log_events_profile_size;
    }
    dlms_log.msg_events_rd_ini_addr = dlms_log.msg_events_wr_address;
    dlms_log.msg_events_rd_address  = dlms_log.msg_events_wr_address;

    return addr;
}

static uint8_t __programTelegramLoadProfileValueWord( unsigned int n )
{
    uint32_t  			 size, size_of_telegram_value, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  			*ptr;
	uint32_t 			 next_addr;
	dlms_telegram_values record_check;

	size                   = n;
	size_of_telegram_value = sizeof(dlms_telegram_value);
    ptr                    = (uint8_t *) &dlms_telegram_value;

    if (size < size_of_telegram_value)
    {
    	size = size_of_telegram_value;
    }

    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 ) {
    	dlms_telegram_log_profile_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	dlms_telegram_log_profile_size = num_pages * sFLASH_SPI_PAGESIZE;
    }
//    if ( size < dlms_telegram_log_profile_size )
//    {
//    	size = dlms_telegram_log_profile_size;
//    }
    next_addr = __nextLoadProfileAddress( dlms_telegram_log_profile_size );
    sFLASH_WriteBuffer(ptr, next_addr, size);
    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> GENERIC Telegram Value Write address: 0x%X.--------------------msg_num:%d \r\n", (int)Tick_Get( SECONDS ), (int)next_addr, (int)dlms_log.raw.msg_num);
    if (next_addr == 0x6CA800)
    {
    	__NOP();
    }
#if 1
    HAL_Delay(100);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	__NOP();
    	check_write = 0;
    } else {
    	__NOP();
    	check_write = 1;
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> GENERIC ERROR!!!Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    }
    uint32_t num = 0;
    if ( 1 == check_write ) {
    	do {
//    		sFLASH_EraseSector(next_addr&0xFFFFF000);
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> GENERIC ERROR RECOVERED:Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }
#endif
    return ret;
}

static uint8_t __programTelegramEventsProfileValueWord( unsigned int n )
{
    uint32_t  			 size, size_of_telegram_value, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  			*ptr;
	uint32_t 			 next_addr;
	dlms_telegram_values record_check;

	size                   = n;
	size_of_telegram_value = sizeof(dlms_telegram_value);
    ptr                    = (uint8_t *) &dlms_telegram_value;

    if (size < size_of_telegram_value)
    {
    	size = size_of_telegram_value;
    }

    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 ) {
    	dlms_telegram_log_events_profile_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	dlms_telegram_log_events_profile_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    next_addr = __nextEventsProfileAddress( dlms_telegram_log_events_profile_size );
    sFLASH_WriteBuffer(ptr, next_addr, size);
    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> GENERIC EVENTS Telegram Value Write address: 0x%X.--------------------msg_num:%d \r\n", (int)Tick_Get( SECONDS ), (int)next_addr, (int)dlms_log.raw.msg_num);

    HAL_Delay(100);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	__NOP();
    	check_write = 0;
    } else {
    	__NOP();
    	check_write = 1;
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> GENERIC ERROR!!!Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    }
    uint32_t num = 0;
    if ( 1 == check_write ) {
    	do {
//    		sFLASH_EraseSector(next_addr&0xFFFFF000);
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> GENERIC EVENTS ERROR RECOVERED:Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }
    return ret;
}

int dlms_log_store_raw_loadprofile(const unsigned char* bytes, uint16_t count, uint16_t size)
{
    const char    hexArray[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    unsigned char tmp;
    int           pos, buffer_pos = 0;
    uint32_t 	  ret = 0, i = 0;
    uint32_t      ini_addr;
	uint16_t      size_to_write  = 0;
	uint16_t      remaining_size = size;

    /** Reads from 0x780000 base memory and copies to modbus_sensor_log*/
	__readDLMSLog();

	/* Only comes here the first time or after a memory erase */
	if (dlms_log.init != 0xAAAA)
	{
		dlms_log.init               = 0xAAAA;
		dlms_log.raw.msg_num        = 0;
		dlms_log.msg_wr_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_ini_addr    = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr    = dlms_telegram_values_address;
		dlms_log.msg_load_wr_address     = dlms_loadprofile_address;//TODO:SPLIT EVENTS DATALOGGER.
		dlms_log.msg_load_rd_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_ini_addr    = dlms_loadprofile_address;
		dlms_log.msg_load_rd_end_addr    = dlms_loadprofile_address;
		dlms_telegram_value.msg_num = 0;
		dlms_telegram_value.wr_addr = dlms_telegram_values_address;
		dlms_telegram_value.rd_addr = dlms_telegram_values_address;
	}

	__programWord();

	ini_addr = dlms_log.msg_load_wr_address;//TODO:SPLIT EVENTS DATALOGGER.
	(void)ini_addr;

    if (2 * count > size)
    {
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }

    LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Write Queue.\r\n", (int)Tick_Get( SECONDS ));
	write_task_t con_write_task = (read_task_t)CircularBuffer_Get(dlms_client_get_write_msg_queue());
	dlms_client_set_frame_type_last_write(con_write_task);
	dlms_telegram_value.frame_type 		 = con_write_task;
	dlms_telegram_value.is_command 		 = dlms_client_get_dlms_load_profile_get_from_command();
	dlms_telegram_value.num_generic_obis = dlms_client_get_dlms_load_profile_num_obis();
	memset(dlms_telegram_value.dlms_obis_capture_objects, 0, sizeof(dlms_telegram_value.dlms_obis_capture_objects));
	for (i=0; i<dlms_telegram_value.num_generic_obis; i++)
	{
		strncpy((char *) dlms_telegram_value.dlms_obis_capture_objects[i], dlms_client_get_load_profile_obis_n(i), sizeof(dlms_telegram_value.dlms_obis_capture_objects[i]));
	}
	memset(dlms_telegram_value.dlms_meter_id, 0, sizeof(dlms_telegram_value.dlms_meter_id));
	strncpy(dlms_telegram_value.dlms_meter_id, dlms_client_get_meter_id(con_dlms_get_curr_device()), strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())));
	dlms_telegram_value.num_meter = con_dlms_get_curr_device();
	memcpy( dlms_telegram_value.created_sensor_value_date,
			(char *)( rtc_system_getCreatedModbusValueDate() ),
			sizeof( dlms_telegram_value.created_sensor_value_date  ) );
	memcpy( dlms_telegram_value.created_sensor_value,
			(char *)( rtc_system_getCreatedModbusValueTime() ),
			sizeof( dlms_telegram_value.created_sensor_value  ) );

    if (count != 0)
    {
        for (pos = 0; pos != count; ++pos)
        {
        	tmp = bytes[pos] & 0xFF;
        	if ( 0 == pos%(MAX_DLMS_TELEGRAM_VALUES/2) )//1024
        	{
        		buffer_pos = 0;
        	}
        	dlms_telegram_value.raw_dlms[buffer_pos * 2]     = hexArray[tmp >> 4];
        	dlms_telegram_value.raw_dlms[buffer_pos * 2 + 1] = hexArray[tmp & 0x0F];
        	buffer_pos++;
        	size_to_write  += 2;
        	remaining_size -= 2;
        	if ( ( 0 == ((2*(pos+1))%MAX_DLMS_TELEGRAM_VALUES) ) && ( pos != 0 ) )
        	{
        		__programTelegramLoadProfileValueWord(MAX_DLMS_TELEGRAM_VALUES);//TODO:SPLIT EVENTS DATALOGGER.
        		size_to_write = 0;
        		dlms_log.raw.msg_num++;
        		HAL_Delay(100);
        		__programWord();
        	}
        	else if ( ( size_to_write < MAX_DLMS_TELEGRAM_VALUES ) && ( pos >= (count-1) ) )
        	{
        		dlms_telegram_value.raw_dlms[(2 * buffer_pos) /*- 1*/] = '\0';
        		__programTelegramLoadProfileValueWord(size_to_write + sizeof(dlms_telegram_value) - (MAX_DLMS_TELEGRAM_VALUES + 1)*sizeof(char));//TODO:SPLIT EVENTS DATALOGGER.
//        		dlms_client_set_dlms_load_profile_message_length(size_to_write);
        		dlms_telegram_value.raw_len = size_to_write;
        		size_to_write = 0;
        		dlms_log.raw.msg_num++;
        		HAL_Delay(100);
        		__programWord();
        	}
        }
    }
    else
    {
    	dlms_telegram_value.raw_dlms[0] = '\0';
    }
	if (( 1 == dlms_telegram_value.is_command ) && ( 0 == (con_get_num_items_device() - 1) ) )
	{
		dlms_client_table_inc_curr_meter_from_cmd();
		if (con_dlms_get_next_device_for_cmd(dlms_client_table_get_curr_meter_from_cmd()) != 0xFF)
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Next CMD Meter:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//			dlms_client_table_inc_curr_meter_from_cmd();
			if ((dlms_client_table_get_curr_meter_from_cmd()+1)>=dlms_client_table_get_num_meters_from_cmd())
			{
//					con_dlms_set_curr_device(0);
				mqtt_stop();
			}
		}
		else
		{
			con_dlms_set_curr_device(0);
			mqtt_stop();
		}
	}
    return ret;
}

char last_event[30][72];
int dlms_log_store_raw_eventsprofile(const unsigned char* bytes, uint16_t count, uint16_t size)
{
    const char    hexArray[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    unsigned char tmp;
    int           pos, buffer_pos = 0;
    uint32_t 	  ret = 0, i = 0;
    uint32_t      ini_addr;
	uint16_t      size_to_write  = 0;
	uint16_t      remaining_size = size;
	char 		  event_void[5] = "0100";
	event_void[4] = '\0';
//	char    	  last_event_back_up[1][72];
    /** Reads from 0x780000 base memory and copies to modbus_sensor_log*/
	__readDLMSLog();

	/* Only comes here the first time or after a memory erase */
	if (dlms_log.init != 0xAAAA)
	{
		dlms_log.init               = 0xAAAA;
		dlms_log.raw.msg_num        = 0;
		dlms_log.msg_wr_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_ini_addr    = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr    = dlms_telegram_values_address;
		dlms_log.msg_events_wr_address     = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_address     = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_ini_addr    = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_end_addr    = dlms_eventsprofile_address;
		dlms_telegram_value.msg_num = 0;
		dlms_telegram_value.wr_addr = dlms_telegram_values_address;
		dlms_telegram_value.rd_addr = dlms_telegram_values_address;
	}

	__programWord();

	ini_addr = dlms_log.msg_events_wr_address;
	(void)ini_addr;

    if (2 * count > size)
    {
        return DLMS_ERROR_CODE_INVALID_PARAMETER;
    }

    LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Write Queue.\r\n", (int)Tick_Get( SECONDS ));
	write_task_t con_write_task = (read_task_t)CircularBuffer_Get(dlms_client_get_write_msg_queue());
	dlms_client_set_frame_type_last_write(con_write_task);
	dlms_telegram_value.frame_type 		 = con_write_task;
	dlms_telegram_value.is_command 		 = dlms_client_get_dlms_load_profile_get_from_command();
	dlms_telegram_value.num_generic_obis = dlms_client_get_dlms_load_profile_num_obis();
	memset(dlms_telegram_value.dlms_obis_capture_objects, 0, sizeof(dlms_telegram_value.dlms_obis_capture_objects));
	for (i=0; i<dlms_telegram_value.num_generic_obis; i++)
	{
		strncpy((char *) dlms_telegram_value.dlms_obis_capture_objects[i], dlms_client_get_load_profile_obis_n(i), sizeof(dlms_telegram_value.dlms_obis_capture_objects[i]));
	}
	memset(dlms_telegram_value.dlms_meter_id, 0, sizeof(dlms_telegram_value.dlms_meter_id));
	strncpy(dlms_telegram_value.dlms_meter_id, dlms_client_get_meter_id(con_dlms_get_curr_device()), strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())));
	dlms_telegram_value.num_meter = con_dlms_get_curr_device();
	memcpy( dlms_telegram_value.created_sensor_value_date,
			(char *)( rtc_system_getCreatedModbusValueDate() ),
			sizeof( dlms_telegram_value.created_sensor_value_date  ) );
	memcpy( dlms_telegram_value.created_sensor_value,
			(char *)( rtc_system_getCreatedModbusValueTime() ),
			sizeof( dlms_telegram_value.created_sensor_value  ) );

	memset(dlms_telegram_value.raw_dlms, 0, sizeof(dlms_telegram_value.raw_dlms));
    if (count != 0)
    {
        for (pos = 0; pos != count; ++pos)
        {
        	tmp = bytes[pos] & 0xFF;
        	if ( 0 == pos%(MAX_DLMS_TELEGRAM_VALUES/2) )//1024
        	{
        		buffer_pos = 0;
        	}
        	dlms_telegram_value.raw_dlms[buffer_pos * 2]     = hexArray[tmp >> 4];
        	dlms_telegram_value.raw_dlms[buffer_pos * 2 + 1] = hexArray[tmp & 0x0F];
        	buffer_pos++;
        	size_to_write  += 2;
        	remaining_size -= 2;
        	if ( ( 0 == ((2*(pos+1))%MAX_DLMS_TELEGRAM_VALUES) ) && ( pos != 0 ) )
        	{
        		if ((0 == strncmp(dlms_telegram_value.raw_dlms, &last_event[con_dlms_get_curr_device()][0], 72*sizeof(char))) || (0 == memcmp(dlms_telegram_value.raw_dlms, event_void, 4*sizeof(char))))
        		{
        			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Same Event Log Meter:%d. \r\n",
        						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//					__incReadDLMSRawEventsProfileLog();
        			ret = 1;
        		}
        		else
        		{
        		__programTelegramEventsProfileValueWord(MAX_DLMS_TELEGRAM_VALUES);
        		size_to_write = 0;
        		dlms_log.raw.msg_num++;
        		HAL_Delay(100);
        		__programWord();
        		}
        	}
        	else if ( ( size_to_write < MAX_DLMS_TELEGRAM_VALUES ) && ( pos >= (count-1) ) )
        	{
        		dlms_telegram_value.raw_dlms[(2 * buffer_pos)] = '\0';
        		if ((0 == strncmp(dlms_telegram_value.raw_dlms, &last_event[con_dlms_get_curr_device()][0], 72*sizeof(char))) || (0 == memcmp(dlms_telegram_value.raw_dlms, event_void, 4*sizeof(char))))
        		{
        			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Same Event Log Meter:%d. \r\n",
        						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//					__incReadDLMSRawEventsProfileLog();
        			ret = 1;
        		}
        		else
        		{
        		__programTelegramEventsProfileValueWord(size_to_write + sizeof(dlms_telegram_value) - (MAX_DLMS_TELEGRAM_VALUES + 1)*sizeof(char));//TODO:SPLIT EVENTS DATALOGGER.
//        		dlms_client_set_dlms_load_profile_message_length(size_to_write);
        		dlms_telegram_value.raw_len = size_to_write;
        		size_to_write = 0;
        		dlms_log.raw.msg_num++;
        		HAL_Delay(100);
        		__programWord();
        		}
        	}
        }
    }
    else
    {
    	dlms_telegram_value.raw_dlms[0] = '\0';
    }
	if (( 1 == dlms_telegram_value.is_command ) && ( 0 == (con_get_num_items_device() - 1) ) )
	{
		dlms_client_table_inc_curr_meter_from_cmd();
		if (con_dlms_get_next_device_for_cmd(dlms_client_table_get_curr_meter_from_cmd()) != 0xFF)
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Next CMD Meter:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
//			dlms_client_table_inc_curr_meter_from_cmd();
			if ((dlms_client_table_get_curr_meter_from_cmd()+1)>=dlms_client_table_get_num_meters_from_cmd())
			{
//					con_dlms_set_curr_device(0);
				mqtt_stop();
			}
		}
		else
		{
			con_dlms_set_curr_device(0);
			mqtt_stop();
		}
	}
//	char event_void[5] = "0100";
//	event_void[4] = '\0';
//	if ((0 == memcmp(dlms_telegram_value.raw_dlms, &last_event[con_dlms_get_curr_device()][0], 72*sizeof(char))) || (0 == memcmp(dlms_telegram_value.raw_dlms, event_void, 4*sizeof(char))))
//	{
//		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Same Event Log Meter:%d. \r\n",
//					(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
////		__incReadDLMSRawEventsProfileLog();
//		ret = 1;
//	}
	strncpy(&last_event[con_dlms_get_curr_device()][0], dlms_telegram_value.raw_dlms, 72*sizeof(char));
    return ret;
}

/**
  * @brief Writes MODBUS raw data to FLASH memory
  * @retval None
  */
void dlms_log_raw_modbus(void)
{
#define BUFFER_SIZE (1024)
	uint32_t quantity = 0;
//	uint32_t i,j;
	char raw_data_buffer[1024];

	/* union to handle different type of data in memory*/
    union
    {
		uint32_t w[BUFFER_SIZE/4];
        uint16_t s[BUFFER_SIZE/2];
        uint8_t  b[BUFFER_SIZE];
    } raw_temp, raw;

    memset(raw_temp.b, 0, sizeof(raw_temp.b));
    memset(raw.b, 0, sizeof(raw.b));
    /** Reads from 0x780000 base memory and copies to modbus_sensor_log*/
	__readDLMSLog();

	/* Only comes here the first time or after a memory erase */
	if (dlms_log.init != 0xAAAA)
	{
		dlms_log.init               = 0xAAAA;
		dlms_log.raw.msg_num        = 0;
		dlms_log.msg_wr_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_address     = dlms_telegram_values_address;
		dlms_log.msg_rd_ini_addr    = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr    = dlms_telegram_values_address;
		dlms_log.msg_load_wr_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_ini_addr    = dlms_loadprofile_address;
		dlms_log.msg_load_rd_end_addr    = dlms_loadprofile_address;
		dlms_telegram_value.msg_num = 0;
		dlms_telegram_value.wr_addr = dlms_telegram_values_address;
		dlms_telegram_value.rd_addr = dlms_telegram_values_address;
	}

	/** Clears the raw data MODBUS telegram */
	memset(dlms_log.raw.raw_dlms_telegram, 0, sizeof(dlms_log.raw.raw_dlms_telegram));
	/** Converts raw data MODBUS telegram to a string in hexadecimal */
//	common_lib_string2hexString((char *) raw.b, dlms_log.raw.raw_dlms_telegram, 2 * quantity);
	sprintf(dlms_log.raw.raw_dlms_telegram, "%s", dlms_client_get_raw_modbus(raw_data_buffer, &quantity));
	dlms_log.raw.len = strlen(dlms_log.raw.raw_dlms_telegram);
	/** Saves modbus_sensors_log raw telegram to FLASH address _modbus_start_address = 0x780000*/
	__programWord();
}

static uint32_t __nextAddress(unsigned int n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr   = dlms_log.msg_wr_address + n;
	next_sector = ( dlms_log.msg_wr_address + 0x1000 ) & 0xFFFFF000;

    if ( ( dlms_log.msg_wr_address & 0x00000FFF ) == 0 ) {
    	sFLASH_EraseSector(dlms_log.msg_wr_address );
    } else if (  next_addr > next_sector ) {
    	sFLASH_EraseSector( next_sector );
    }

    addr = dlms_log.msg_wr_address;
    dlms_log.msg_wr_address  += n;
    if ( dlms_log.msg_wr_address > dlms_telegram_values_max_address ) {
    	dlms_log.msg_wr_address = dlms_telegram_values_address;
    }

    if ( ( dlms_log.msg_wr_address == dlms_log.msg_rd_end_addr ) ) {
    	dlms_log.msg_rd_end_addr += dlms_telegram_log_size;
    }
    dlms_log.msg_rd_ini_addr = dlms_log.msg_wr_address;
    dlms_log.msg_rd_address  = dlms_log.msg_wr_address;

    return addr;
}

static uint8_t __programTelegramValueWord( void )
{
    uint32_t  size, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  *ptr;
	uint32_t next_addr;
	dlms_telegram_values record_check;

	size = sizeof(dlms_telegram_value);
    ptr  = (uint8_t *) &dlms_telegram_value;

    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 ) {
    	dlms_telegram_log_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	dlms_telegram_log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    next_addr = __nextAddress( dlms_telegram_log_size );
    sFLASH_WriteBuffer(ptr, next_addr, size);
    HAL_Delay(100);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> OBIS Telegram Value Write address: 0x%X.--------------------msg_num:%d \r\n", (int)Tick_Get( SECONDS ), (int)next_addr, (int)dlms_log.raw.msg_num);

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	__NOP();
    	check_write = 0;
    } else {
    	__NOP();
    	check_write = 1;
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> OBIS ERROR!!!!!!Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    }
    uint32_t num = 0;
    if ( 1 == check_write ) {
    	do {
//    		sFLASH_EraseSector(next_addr&0xFFFFF000);
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> OBIS ERROR RECOVERED:Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }

    return ret;
}

char last_event_ff[30][63];
uint32_t dlms_log_params_store_raw_modbus( uint32_t frame_type )
{
	static uint32_t send_max_extra = 0;
	uint32_t ret = 0;
	if ( dlms_log.init == 0xAAAA )
	{
		dlms_log.raw.msg_num++;
		memcpy( dlms_telegram_value.raw_dlms,
				dlms_log.raw.raw_dlms_telegram,
				sizeof(dlms_log.raw.raw_dlms_telegram) );
		dlms_telegram_value.raw_len = dlms_log.raw.len;
		dlms_telegram_value.msg_num++;
	}

	memcpy( dlms_telegram_value.created_sensor_value_date,
			(char *)( rtc_system_getCreatedModbusValueDate() ),
			sizeof( dlms_telegram_value.created_sensor_value_date  ) );
	memcpy( dlms_telegram_value.created_sensor_value,
			(char *)( rtc_system_getCreatedModbusValueTime() ),
			sizeof( dlms_telegram_value.created_sensor_value  ) );
	dlms_telegram_value.frame_type     = frame_type;
	dlms_telegram_value.frame_event_ff = dlms_event_ff_frame;
	dlms_event_ff_frame                = 0;
	dlms_telegram_value.is_command     = dlms_client_get_obis_profile_from_command();
	memset(dlms_telegram_value.dlms_meter_id, 0, sizeof(dlms_telegram_value.dlms_meter_id));
	strncpy(dlms_telegram_value.dlms_meter_id, dlms_client_get_meter_id(con_dlms_get_curr_device()), strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())));
//	if (0 == con_dlms_get_curr_device())
//	{
//		dlms_telegram_value.num_meter = dlms_client_table_get_num_devices()-1;
//	}
//	else
//	{
//		dlms_telegram_value.num_meter = con_dlms_get_curr_device() - 1;
//	}
	dlms_telegram_value.num_meter      = con_dlms_get_curr_device();
	if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == DLMS_WAIT_FOR_SEND_BILLING_PROF )
	{
		dlms_telegram_value.frame_tunneling = 1;
		dlms_telegram_value.is_command      = dlms_client_get_dlms_load_profile_get_from_command();
	}
	else
	{
		dlms_telegram_value.frame_tunneling = 0;
	}
	if ( 1 == dlms_telegram_value.frame_event_ff)
	{
		if ((0 == strncmp(dlms_telegram_value.raw_dlms + strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())) + 12, &last_event_ff[dlms_telegram_value.num_meter][0], 62*sizeof(char))))
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Same Event FF Log Meter:%d. \r\n",
					(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
			ret = 1;
		}
		else
		{
//			dlms_telegram_value.num_meter = con_dlms_get_curr_device();
			__programTelegramValueWord();
			HAL_Delay(100);
			__programWord();
			ret = 2;
		}
		strncpy(&last_event_ff[dlms_telegram_value.num_meter][0], dlms_telegram_value.raw_dlms + strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())) + 12, 62*sizeof(char));
	}
	else
	{
//		dlms_telegram_value.num_meter = con_dlms_get_curr_device();
		__programTelegramValueWord();
		HAL_Delay(100);
		__programWord();
	}
	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> ID Meter:%s. \r\n", (int)Tick_Get( SECONDS ), dlms_telegram_value.dlms_meter_id);
//	HAL_Delay(100);
//	__programWord();
	if (( 1 == dlms_telegram_value.is_command ) && ( 0 == (con_get_num_items_device() - 1) ) )
	{
		uint32_t extra_frame_type = dlms_client_get_max_demand_profile_extra_frame_type(con_dlms_get_curr_device(),dlms_telegram_value.frame_type);
		if ((extra_frame_type != 0)
		 && (0 == send_max_extra)
		 && (DLMS_WRITE_MAX_DEMAND_PROF_1 == dlms_telegram_value.frame_type)
		 )
		{
//			dlms_client_set_dlms_id_request(dlms_client_get_dlms_id_request());
			send_max_extra = 1;
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INST_PROF_1 + extra_frame_type);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INST_PROF_1 + extra_frame_type);
//			dlms_client_inc_dlms_reading_items(con_dlms_get_curr_device());
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Max Demand Profile: %d.\r\n",
					(int)Tick_Get(SECONDS), (int)dlms_client_get_obis_profile_max_demand_profile_extra());
		}
		else
		{
			send_max_extra = 0;
			dlms_client_table_inc_curr_meter_from_cmd();
			if (con_dlms_get_next_device_for_cmd(dlms_client_table_get_curr_meter_from_cmd()) != 0xFF)
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Next CMD Meter:%d. \r\n",
						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
				//			dlms_client_table_inc_curr_meter_from_cmd();
				if ((dlms_client_table_get_curr_meter_from_cmd()+1)>=dlms_client_table_get_num_meters_from_cmd())
				{
					//				con_dlms_set_curr_device(0);
					mqtt_stop();
				}
			}
			else
			{
				con_dlms_set_curr_device(0);
				mqtt_stop();
			}
		}
	}
//	strncpy(&last_event_ff[dlms_telegram_value.num_meter][0], dlms_telegram_value.raw_dlms + strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())) + 12, 62*sizeof(char));
	return ret;
}

uint32_t dlms_log_params_store_raw_modbus_tunneling( uint32_t frame_type )
{
	static uint32_t send_max_extra = 0;
	uint32_t ret = 0;
	if ( dlms_log.init == 0xAAAA )
	{
		dlms_log.raw.msg_num++;
		memcpy( dlms_telegram_value.raw_dlms,
				dlms_log.raw.raw_dlms_telegram,
				sizeof(dlms_log.raw.raw_dlms_telegram) );
		dlms_telegram_value.raw_len = dlms_log.raw.len;
		dlms_telegram_value.msg_num++;
	}

	memcpy( dlms_telegram_value.created_sensor_value_date,
			(char *)( rtc_system_getCreatedModbusValueDate() ),
			sizeof( dlms_telegram_value.created_sensor_value_date  ) );
	memcpy( dlms_telegram_value.created_sensor_value,
			(char *)( rtc_system_getCreatedModbusValueTime() ),
			sizeof( dlms_telegram_value.created_sensor_value  ) );
	dlms_telegram_value.frame_type     = frame_type;
	dlms_telegram_value.frame_event_ff = dlms_event_ff_frame;
	dlms_event_ff_frame                = 0;
	dlms_telegram_value.is_command     = dlms_client_get_obis_profile_from_command();
	memset(dlms_telegram_value.dlms_meter_id, 0, sizeof(dlms_telegram_value.dlms_meter_id));
	strncpy(dlms_telegram_value.dlms_meter_id, dlms_client_get_meter_id(con_dlms_get_curr_device()), strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())));
	dlms_telegram_value.num_meter      = con_dlms_get_curr_device();

	dlms_telegram_value.frame_tunneling = 1;
	dlms_telegram_value.is_command      = dlms_client_get_dlms_load_profile_get_from_command();

	__programTelegramValueWord();
	HAL_Delay(100);
	__programWord();

	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> ID Meter:%s. \r\n", (int)Tick_Get( SECONDS ), dlms_telegram_value.dlms_meter_id);

	if (( 1 == dlms_telegram_value.is_command ) && ( 0 == con_get_num_items_device() ) )
	{
		uint32_t extra_frame_type = dlms_client_get_max_demand_profile_extra_frame_type(con_dlms_get_curr_device(),dlms_telegram_value.frame_type);
		if ((extra_frame_type != 0)
		 && (0 == send_max_extra)
		 && (DLMS_WRITE_MAX_DEMAND_PROF_1 == dlms_telegram_value.frame_type)
		 )
		{
//			dlms_client_set_dlms_id_request(dlms_client_get_dlms_id_request());
			send_max_extra = 1;
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INST_PROF_1 + extra_frame_type);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INST_PROF_1 + extra_frame_type);
//			dlms_client_inc_dlms_reading_items(con_dlms_get_curr_device());
			LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> Max Demand Profile: %d.\r\n",
					(int)Tick_Get(SECONDS), (int)dlms_client_get_obis_profile_max_demand_profile_extra());
		}
		else
		{
			send_max_extra = 0;
			dlms_client_table_inc_curr_meter_from_cmd();
			if (con_dlms_get_next_device_for_cmd(dlms_client_table_get_curr_meter_from_cmd()) != 0xFF)
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_LOG> Next CMD Meter:%d. \r\n",
						(int)Tick_Get( SECONDS ), (int)con_dlms_get_curr_device());
				//			dlms_client_table_inc_curr_meter_from_cmd();
				if ((dlms_client_table_get_curr_meter_from_cmd()+1)>=dlms_client_table_get_num_meters_from_cmd())
				{
					//				con_dlms_set_curr_device(0);
					mqtt_stop();
				}
			}
			else
			{
				con_dlms_set_curr_device(0);
				mqtt_stop();
			}
		}
	}
	return ret;
}

void dlms_log_recover_rd_address_backup( void )
{
	dlms_log.raw.msg_num     = dlms_log.raw.msg_num_backup;
	dlms_log.msg_rd_address  = dlms_log.msg_rd_address_backup;
	dlms_log.msg_wr_address  = dlms_log.msg_rd_address;
	dlms_log.msg_rd_ini_addr = dlms_log.msg_rd_address_backup;

	HAL_Delay(100);
	__programWord();
}

void dlms_log_profile_recover_rd_address_backup( void )
{
	dlms_log.raw.msg_num          = dlms_log.raw.msg_num_backup;
	dlms_log.msg_load_rd_address  = dlms_log.msg_load_rd_address_backup;
	dlms_log.msg_load_wr_address  = dlms_log.msg_load_rd_address;
	dlms_log.msg_load_rd_ini_addr = dlms_log.msg_load_rd_address_backup;

	HAL_Delay(100);
	__programWord();
}

char datalogger_dlms_info[150];
char *dlms_log_GetDatalogger_Info( void )
{
	memset(datalogger_dlms_info,0,sizeof(datalogger_dlms_info));
	if ( dlms_client_get_read_time() != 0 ) {
		sprintf(datalogger_dlms_info,
				"%d,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X",
				(int)dlms_log.raw.msg_num,
				(int)dlms_log.msg_rd_ini_addr,
				(int)dlms_log.msg_rd_end_addr,
				(int)dlms_log.msg_load_rd_ini_addr,
				(int)dlms_log.msg_load_rd_end_addr,
				(int)dlms_log.msg_events_rd_ini_addr,
				(int)dlms_log.msg_events_rd_end_addr
		);
	}

	return datalogger_dlms_info;
}

static void __incReadDLMSRawLog( void )
{
	if ( dlms_log.msg_rd_ini_addr !=  dlms_log.msg_rd_end_addr)
	{
	if ( dlms_log.msg_rd_ini_addr - dlms_telegram_log_size >= (uint32_t) dlms_telegram_values_address ) {
    	dlms_log.msg_rd_ini_addr -= dlms_telegram_log_size;
    } else {
    	if ( dlms_telegram_values_max_address - dlms_telegram_log_size > dlms_log.msg_rd_end_addr ) {
    		dlms_log.msg_rd_ini_addr = dlms_telegram_values_max_address - dlms_telegram_log_size + 1;
    	} else {
    		dlms_log.msg_rd_ini_addr = dlms_log.msg_rd_end_addr;
    	}
    }
	}

    if ( ( dlms_log.msg_rd_ini_addr <= dlms_log.msg_rd_end_addr ) ) {
//        ret = 0;
    }

    dlms_log.msg_wr_address = dlms_log.msg_rd_ini_addr;
    dlms_log.msg_rd_address = dlms_log.msg_rd_ini_addr;
}

static void __incReadDLMSRawProfileLog( void )
{
    if (dlms_log.msg_load_rd_ini_addr != dlms_log.msg_load_rd_end_addr)
    {
    	if ( dlms_log.msg_load_rd_ini_addr - dlms_telegram_log_profile_size >= (uint32_t) dlms_loadprofile_address ) {
    		dlms_log.msg_load_rd_ini_addr -= dlms_telegram_log_profile_size;
    	} else {
    		if ( dlms_telegram_values_max_address - dlms_telegram_log_profile_size > dlms_log.msg_load_rd_end_addr ) {
    			dlms_log.msg_load_rd_ini_addr = dlms_telegram_values_max_address - dlms_telegram_log_profile_size + 1;
    		} else {
    			dlms_log.msg_load_rd_ini_addr = dlms_log.msg_load_rd_end_addr;
    		}
    	}
    }

    if ( ( dlms_log.msg_load_rd_ini_addr <= dlms_log.msg_load_rd_end_addr ) ) {
//        ret = 0;
    }

    dlms_log.msg_load_wr_address = dlms_log.msg_load_rd_ini_addr;
    dlms_log.msg_load_rd_address = dlms_log.msg_load_rd_ini_addr;
}

static void __incReadDLMSRawEventsProfileLog( void )
{
    if (dlms_log.msg_events_rd_ini_addr != dlms_log.msg_events_rd_end_addr)
    {
    	if ( dlms_log.msg_events_rd_ini_addr - dlms_telegram_log_events_profile_size >= (uint32_t) dlms_eventsprofile_address ) {
    		dlms_log.msg_events_rd_ini_addr -= dlms_telegram_log_events_profile_size;
    	} else {
    		if ( dlms_telegram_values_events_max_address - dlms_telegram_log_events_profile_size > dlms_log.msg_events_rd_end_addr ) {
    			dlms_log.msg_events_rd_ini_addr = dlms_telegram_values_events_max_address - dlms_telegram_log_events_profile_size + 1;
    		} else {
    			dlms_log.msg_events_rd_ini_addr = dlms_log.msg_events_rd_end_addr;
    		}
    	}
    }

    if ( ( dlms_log.msg_events_rd_ini_addr <= dlms_log.msg_events_rd_end_addr ) ) {
//        ret = 0;
    }

    dlms_log.msg_events_wr_address = dlms_log.msg_events_rd_ini_addr;
    dlms_log.msg_events_rd_address = dlms_log.msg_events_rd_ini_addr;
}

char * __decode_tag(write_task_t con_write_last)
{
	char * ret = NULL;

	switch(con_write_last) {
	case DLMS_WRITE_INST_PROF_1:
		ret = "IF|0";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|0";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_INST_PROF_2:
		ret = "IF|1";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|1";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_INST_PROF_3:
		ret = "IF|2";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|2";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_INST_PROF_4:
		ret = "IF|3";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|3";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_ENERGY_PROF_1:
		ret = "DF|0";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|4";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_ENERGY_PROF_2:
		ret = "DF|1";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|5";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_ENERGY_PROF_3:
		ret = "DF|2";
		if (1 == dlms_log_get_profile_max_demand_extra())
		{
			ret = "MXF|6";
			dlms_log_set_profile_max_demand_extra(0);
		}
		break;
	case DLMS_WRITE_MAX_DEMAND_PROF_1:
		ret = "MXF|0";
		break;
	case DLMS_WRITE_LOAD_PROF_1:
		ret = "LF|0";
		break;
	case DLMS_WRITE_LOAD_PROF_2:
		ret = "LF|1";
		break;
	case DLMS_WRITE_POWER_QUALITY_PROF:
		ret = "LF|2";
		break;
	case DLMS_WRITE_INSTRUMENTATION_PROF_1:
		ret = "LF|3";
		break;
	case DLMS_WRITE_BILLING_PROF:
		ret = "BF|0";
		break;
	case DLMS_WRITE_EVENT_LOG_PROF:
		ret = "EF|0";
		break;
	default:
		break;

	}
	if (1 == dlms_log_get_frame_tunneling())
	{
		ret = "BTF|0";
		dlms_log_set_frame_tunneling(0);
	}
//	if (1 == dlms_log_get_profile_max_demand_extra())
//	{
//		ret = "MXF|1";
//		dlms_log_set_profile_max_demand_extra(0);
//	}
	return ret;
}

char * dlms_log_sensor_raw_telegram( uint32_t *measure_count )
{
	uint32_t i, str_len = 0, num_it = 0, backup_msg_num = 0, rd_ini_backup = 0, wr_backup = 0, rd_backup = 0;
	char date[9];
	char *tag = "EFF|0";//"DF|0";
	char id_request[40];
	static int32_t curr_meter_from_cmd = -1;

	memset(id_request,0,40*sizeof(char));

//	dlms_client_table_read_client_id( dlms_client_get_client(), con_dlms_get_curr_device() );
//	write_task_t con_write_last = dlms_client_get_frame_type_last_write();//dlms_client_get_obis_profile_frame_type();//

	__readDLMSLog();

	num_it = dlms_log.raw.msg_num;
	if ( 0 == num_it )
	{
//		return NULL;
		num_it = 1;
	}

	rd_ini_backup                  = dlms_log.msg_rd_ini_addr;
	wr_backup                      = dlms_log.msg_wr_address;
	rd_backup                      = dlms_log.msg_rd_address;
	dlms_log.raw.msg_num_backup    = dlms_log.raw.msg_num;
	dlms_log.msg_rd_address_backup = dlms_log.msg_rd_ini_addr;
	*measure_count                 = *measure_count + 1;
	__incReadDLMSRawLog();
	dlms_log.raw.msg_num--;
	if ( dlms_log.raw.msg_num > 0 )
	{
//		num_it = modbus_sensors_log.raw.msg_num;
	}

	if ( 0 == num_it )
	{
		sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
				dlms_log.msg_rd_ini_addr,
				sizeof(dlms_telegram_value) );
		return NULL;
	}

	memset(dlms_telegram_string,             0, sizeof(dlms_telegram_string));
	memset((uint8_t *)&dlms_telegram_value,  0, sizeof(dlms_telegram_value));
	backup_msg_num = dlms_log.raw.msg_num;
	num_it         = 1;
	for ( i = 0; i < num_it; i++ )
	{
		sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
				dlms_log.msg_rd_ini_addr,
				sizeof(dlms_telegram_value) );
		LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> ID Meter: %s. OBIS rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n",(int)Tick_Get( SECONDS ), dlms_telegram_value.dlms_meter_id, (int)dlms_log.msg_rd_ini_addr, (int)dlms_log.msg_rd_end_addr, (int)dlms_log.raw.msg_num, (int)dlms_telegram_value.raw_len);
		if (1 == dlms_telegram_value.is_command)
		{
			dlms_telegram_value.is_command = 0;
			memcpy(id_request, dlms_client_get_dlms_id_request(), 40*sizeof(char));
			if (*dlms_client_get_dlms_id_request()!='\0')
			{
				if ((dlms_client_table_get_num_meters_from_cmd() > 1) && (0 == curr_meter_from_cmd))
				{
					curr_meter_from_cmd = -1;
					dlms_client_inc_id_request_index_rd();
				}
				else if (-1 == curr_meter_from_cmd)
				{
					curr_meter_from_cmd = dlms_client_table_get_num_meters_from_cmd();
				}
				else
				{
					curr_meter_from_cmd = curr_meter_from_cmd - 1;
				}
			}
		}
		else
		{
//			memset(dlms_client_get_dlms_id_request(), 0 , 40*sizeof(char));
		}

		if ( (  str_len
				+ dlms_telegram_value.raw_len
		     ) >=  ( MAX_DLMS_TELEGRAM_VALUES - 250 ) )
		{
			dlms_log.msg_rd_ini_addr = rd_ini_backup;
			dlms_log.msg_wr_address  = wr_backup;
			dlms_log.msg_rd_address  = rd_backup;
			backup_msg_num           = backup_msg_num + 1;
			*measure_count           = *measure_count - 1;
			break;
		}
		if ( ( dlms_telegram_value.raw_len == 0 ) )
		{
			*measure_count = *measure_count + 1;
			if ( backup_msg_num > 0 )
			{
				rd_ini_backup = dlms_log.msg_rd_ini_addr;
				wr_backup     = dlms_log.msg_wr_address;
				rd_backup     = dlms_log.msg_rd_address;
				__incReadDLMSRawLog();
				backup_msg_num--;
			}
			else
			{
				break;
			}
			memcpy( date,
					dlms_telegram_value.created_sensor_value_date,
					sizeof(dlms_telegram_value.created_sensor_value_date) + 1 );
			continue;
		}

		if ( dlms_telegram_value.created_sensor_value_date[6] == (char)0xFF )
		{
			backup_msg_num = 0;
			break;
		}
		if ( 0 == dlms_telegram_value.frame_event_ff )
		{
			tag = __decode_tag(dlms_telegram_value.frame_type);
		}
		snprintf(dlms_telegram_string + str_len,
				sizeof( dlms_telegram_value.created_sensor_value_date ) + 60,
				"%s|%s|%s;%d|%s",
				ME910_IMEI(),
				tag,
//				dlms_client_get_meter_id(),
				dlms_telegram_value.dlms_meter_id,
				(int)dlms_telegram_value.num_meter,
				dlms_telegram_value.created_sensor_value_date);
		str_len = strlen(dlms_telegram_string);

		if ( dlms_telegram_string[0]                          == '\0'    )
		{
			dlms_log.msg_rd_ini_addr = rd_ini_backup;
			dlms_log.msg_wr_address  = wr_backup;
			dlms_log.msg_rd_address  = rd_backup;
			backup_msg_num           = backup_msg_num + 1;
			*measure_count           = *measure_count - 1;
			break;
		}
		sprintf(dlms_telegram_string + str_len,
				"%s|",
				dlms_telegram_value.created_sensor_value);
		str_len = strlen(dlms_telegram_string);

		sprintf(dlms_telegram_string + str_len,
				"%s|%s\r\n",
				dlms_telegram_value.raw_dlms,
				id_request);//dlms_client_get_dlms_id_request());
		str_len = strlen(dlms_telegram_string);

		*measure_count = *measure_count + 1;
		memcpy( date,
				dlms_telegram_value.created_sensor_value_date,
				sizeof(dlms_telegram_value.created_sensor_value_date) + 1 );
	}

	dlms_log.raw.msg_num = backup_msg_num;

	HAL_Delay(100);
	__programWord();

	return dlms_telegram_string;
}

char * dlms_log_sensor_raw_profile_telegram( uint32_t *frame_size )
{
	uint32_t read_size  = 0;
	uint32_t bytes_left = 0;
	uint32_t str_len = 0, obj_len = 0, idRequest_len = 0, i = 0, num_it = 0;//, backup_msg_num = 0, wr_backup = 0, rd_ini_backup = 0, rd_backup = 0
	char     header[512];
	char     objects[512];
	char     id_request[40];
	static int32_t curr_meter_from_cmd = -1;

	memset(id_request,0,40*sizeof(char));

	__readDLMSLog();

	num_it = dlms_log.raw.msg_num;
	if ( 0 == num_it )
	{
//		return NULL;
	}

//	rd_ini_backup                       = dlms_log.msg_load_rd_ini_addr;
//	wr_backup                           = dlms_log.msg_load_wr_address;
//	rd_backup                           = dlms_log.msg_load_rd_address;
	dlms_log.raw.msg_num_backup         = dlms_log.raw.msg_num;
	dlms_log.msg_load_rd_address_backup = dlms_log.msg_load_rd_ini_addr;
//	*measure_count                      = *measure_count + 1;
	__incReadDLMSRawProfileLog();
	dlms_log.raw.msg_num--;
	if ( dlms_log.raw.msg_num > 0 )
	{
//		num_it = modbus_sensors_log.raw.msg_num;
	}

	if ( 0 == num_it )
	{
		sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
				dlms_log.msg_load_rd_ini_addr,
				sizeof(dlms_telegram_value) );
		return NULL;
	}

	memset(dlms_telegram_string,             		  0, sizeof(dlms_telegram_string));
	memset((uint8_t *)&dlms_telegram_value.raw_dlms,  0, sizeof(dlms_telegram_value.raw_dlms));
	memset(header,                           		  0, sizeof(header));

	dlms_client_table_read_client_id( dlms_client_get_client(), con_dlms_get_curr_device() );

	sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
			dlms_log.msg_load_rd_ini_addr,
			sizeof(dlms_telegram_value) );
	dlms_client_table_read_client_generic_profile(0, dlms_client_get_client(),  dlms_client_get_client_generic_profile(), dlms_telegram_value.frame_type);
	LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> GENERIC rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n", (int)Tick_Get( SECONDS ), (int)dlms_log.msg_load_rd_ini_addr, (int)dlms_log.msg_load_rd_end_addr, (int)dlms_log.raw.msg_num, (int)strlen(dlms_telegram_value.raw_dlms));

	if (1 == dlms_telegram_value.is_command)
	{
		dlms_telegram_value.is_command = 0;
		memcpy(id_request, dlms_client_get_dlms_id_request(), 40*sizeof(char));
		if (*dlms_client_get_dlms_id_request()!='\0')
		{
			if ((dlms_client_table_get_num_meters_from_cmd() > 1) && (0 == curr_meter_from_cmd))
			{
				curr_meter_from_cmd = -1;
				dlms_client_inc_id_request_index_rd();
			}
			else if (-1 == curr_meter_from_cmd)
			{
				curr_meter_from_cmd = dlms_client_table_get_num_meters_from_cmd();
			}
			else
			{
				curr_meter_from_cmd = curr_meter_from_cmd - 1;
			}
		}
	}
	else if (0xffffffff == dlms_telegram_value.is_command)
	{
		LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> MEMORY ERROR\r\n", (int)Tick_Get( SECONDS ));
		return NULL;
	}
	else
	{
//		memset(dlms_client_get_dlms_id_request(), 0 , 40*sizeof(char));
	}
	LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> Check GENERIC 2 rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n", (int)Tick_Get( SECONDS ), (int)dlms_log.msg_load_rd_ini_addr, (int)dlms_log.msg_load_rd_end_addr, (int)dlms_log.raw.msg_num, (int)strlen(dlms_telegram_value.raw_dlms));
	snprintf(header,
			sizeof( dlms_telegram_value.created_sensor_value_date ) + sizeof(dlms_telegram_value.created_sensor_value) + 60,//+1
			"%s|%s|%s;%d|%s%s|%d;%d;",
			ME910_IMEI(),
			__decode_tag(dlms_telegram_value.frame_type),
//			dlms_client_get_meter_id(),
			dlms_telegram_value.dlms_meter_id,
			(int)dlms_telegram_value.num_meter,
			dlms_telegram_value.created_sensor_value_date,
			dlms_telegram_value.created_sensor_value,
			(int)dlms_telegram_value.capture_period,
			(int)dlms_telegram_value.num_generic_obis);//(int)dlms_client_get_dlms_load_profile_num_obis());
	str_len = strlen(header);

	memset(objects, 0, sizeof(objects));
	for ( i = 0; i < dlms_telegram_value.num_generic_obis; i++ )//dlms_client_get_dlms_load_profile_num_obis()
	{
		sprintf(objects + obj_len,
				"%s;",
				dlms_telegram_value.dlms_obis_capture_objects[i]//dlms_client_get_load_profile_obis_n(i)
				);
		obj_len = strlen(objects);
	}
	LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> Check GENERIC 3 rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n", (int)Tick_Get( SECONDS ), (int)dlms_log.msg_load_rd_ini_addr, (int)dlms_log.msg_load_rd_end_addr, (int)dlms_log.raw.msg_num, (int)strlen(dlms_telegram_value.raw_dlms));
	idRequest_len = strlen(id_request) + 2;//strlen(dlms_client_get_dlms_id_request()) + 2;
	memset(header,  0, sizeof(header));
	memset(objects, 0, sizeof(objects));

	sprintf(header,
			"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
			"Content-Type: text/plain\r\n"
			"Host:%s\r\n"
			"Content-Length: %d\r\n"
			"\r\n",
			Telit_dev_identifier(),
			param.server[ME910_server_get_index()].name,
			(int)((int)strlen(dlms_telegram_value.raw_dlms) + str_len + obj_len + idRequest_len /*- 1*/)//dlms_client_get_dlms_load_profile_message_length()
	);
	str_len = 0;
	obj_len = 0;
	str_len = strlen(header);
	snprintf(header + str_len,
			sizeof( dlms_telegram_value.created_sensor_value_date )+ sizeof(dlms_telegram_value.created_sensor_value) + 60,//+1
			"%s|%s|%s;%d|%s%s|",
			ME910_IMEI(),
			__decode_tag(dlms_telegram_value.frame_type),
//			dlms_client_get_meter_id(),
			dlms_telegram_value.dlms_meter_id,
			(int)dlms_telegram_value.num_meter,
			dlms_telegram_value.created_sensor_value_date,
			dlms_telegram_value.created_sensor_value);
	str_len = strlen(header);
	memset(objects, 0, sizeof(objects));
	for ( i = 0; i < dlms_telegram_value.num_generic_obis; i++ )//dlms_client_get_dlms_load_profile_num_obis()
	{
		sprintf(objects + obj_len,
				"%s;",
				dlms_telegram_value.dlms_obis_capture_objects[i]//dlms_client_get_load_profile_obis_n(i)
				);
		obj_len = strlen(objects);
	}
	sprintf(header + str_len,
			"%d;%d;%s",
			(int)dlms_telegram_value.capture_period,
			(int)dlms_telegram_value.num_generic_obis,//(int)dlms_client_get_dlms_load_profile_num_obis(),
			objects
		);
	str_len = strlen(header);
	Telit_write_data_length(header, strlen(header));

	do
	{
		if ( dlms_telegram_value.raw_len/*dlms_client_get_dlms_load_profile_message_length()*/ > MAX_DLMS_TELEGRAM_VALUES )
		{
			sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value.raw_dlms,
					dlms_log.msg_load_rd_ini_addr + read_size,//dlms_loadprofile_address + read_size,
					MAX_DLMS_TELEGRAM_VALUES);
			if (dlms_telegram_value.raw_dlms[0] == 0xFF)
			{
				dlms_telegram_value.raw_dlms[0] = 0;
				LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> MEMORY CORRUPTED!!!!!! Erase sector: rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n",
						(int)Tick_Get( SECONDS ), (int)dlms_log.msg_load_rd_ini_addr,
						(int)dlms_log.msg_load_rd_end_addr, (int)dlms_log.raw.msg_num,
						(int)strlen(dlms_telegram_value.raw_dlms));
	    		sFLASH_EraseSector(dlms_log.msg_load_rd_ini_addr&0xFFFFF000);
	    		HAL_Delay(100);
				break;
			}
			else
			{
				snprintf(dlms_telegram_string,
						MAX_DLMS_TELEGRAM_VALUES + 1 + 2 + 40,
						"%s|"
						"%s|",
						dlms_telegram_value.raw_dlms,
						id_request);//dlms_client_get_dlms_id_request());
			}
			str_len     = strlen(dlms_telegram_string);
			read_size  += MAX_DLMS_TELEGRAM_VALUES;
			bytes_left  = dlms_telegram_value.raw_len/*dlms_client_get_dlms_load_profile_message_length()*/ - read_size;
			dlms_telegram_value.raw_len = bytes_left;//dlms_client_set_dlms_load_profile_message_length(bytes_left);
			*frame_size = MAX_DLMS_TELEGRAM_VALUES;
			Telit_write_data_length(dlms_telegram_string, str_len);
		}
		else
		{
			sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value.raw_dlms,
					dlms_log.msg_load_rd_ini_addr + read_size,//dlms_loadprofile_address + read_size,
					dlms_telegram_value.raw_len);//dlms_client_get_dlms_load_profile_message_length() );
			sprintf(dlms_telegram_string,
					"%s|"
					"%s|",
					dlms_telegram_value.raw_dlms,
					id_request);//dlms_client_get_dlms_id_request());
			str_len     = strlen(dlms_telegram_string);
			read_size   = dlms_telegram_value.raw_len;//dlms_client_get_dlms_load_profile_message_length();
			*frame_size = dlms_telegram_value.raw_len;//dlms_client_get_dlms_load_profile_message_length();
			bytes_left  = dlms_telegram_value.raw_len/*dlms_client_get_dlms_load_profile_message_length()*/ - read_size;
			Telit_write_data_length(dlms_telegram_string, str_len);
		}
	}while (bytes_left != 0);

	*frame_size = str_len;

	HAL_Delay(100);
	__programWord();

	return dlms_telegram_string;
}

char * dlms_log_sensor_raw_eventsprofile_telegram( uint32_t *frame_size )
{
	uint32_t read_size  = 0;
	uint32_t bytes_left = 0;
	uint32_t str_len = 0, obj_len = 0, idRequest_len = 0, i = 0, num_it = 0;
	char     header[512];
	char     objects[512];
	char     id_request[40];
	static int32_t curr_meter_from_cmd = -1;

	memset(id_request,0,40*sizeof(char));

	__readDLMSLog();

	num_it = dlms_log.raw.msg_num;
	if ( 0 == num_it )
	{
//		return NULL;
	}

	dlms_log.raw.msg_num_backup           = dlms_log.raw.msg_num;
	dlms_log.msg_events_rd_address_backup = dlms_log.msg_events_rd_ini_addr;
	__incReadDLMSRawEventsProfileLog();
	dlms_log.raw.msg_num--;
	if ( dlms_log.raw.msg_num > 0 )
	{
//		num_it = modbus_sensors_log.raw.msg_num;
	}

	if ( 0 == num_it )
	{
		sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
				dlms_log.msg_events_rd_ini_addr,
				sizeof(dlms_telegram_value) );
		return NULL;
	}

	memset(dlms_telegram_string,             		  0, sizeof(dlms_telegram_string));
	memset((uint8_t *)&dlms_telegram_value.raw_dlms,  0, sizeof(dlms_telegram_value.raw_dlms));
	memset(header,                           		  0, sizeof(header));

	dlms_client_table_read_client_id( dlms_client_get_client(), con_dlms_get_curr_device() );

	sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value,
			dlms_log.msg_events_rd_ini_addr,
			sizeof(dlms_telegram_value) );
	dlms_client_table_read_client_generic_profile(0, dlms_client_get_client(),  dlms_client_get_client_generic_profile(), dlms_telegram_value.frame_type);
	LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> GENERIC EVENTS rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n", (int)Tick_Get( SECONDS ), (int)dlms_log.msg_events_rd_ini_addr, (int)dlms_log.msg_events_rd_end_addr, (int)dlms_log.raw.msg_num, (int)strlen(dlms_telegram_value.raw_dlms));

	if (1 == dlms_telegram_value.is_command)
	{
		dlms_telegram_value.is_command = 0;
		memcpy(id_request, dlms_client_get_dlms_id_request(), 40*sizeof(char));
		if (*dlms_client_get_dlms_id_request()!='\0')
		{
			if ((dlms_client_table_get_num_meters_from_cmd() > 1) && (0 == curr_meter_from_cmd))
			{
				curr_meter_from_cmd = -1;
				dlms_client_inc_id_request_index_rd();
			}
			else if (-1 == curr_meter_from_cmd)
			{
				curr_meter_from_cmd = dlms_client_table_get_num_meters_from_cmd();
			}
			else
			{
				curr_meter_from_cmd = curr_meter_from_cmd - 1;
			}
		}
	}
	else if (0xffffffff == dlms_telegram_value.is_command)
	{
		LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> MEMORY ERROR\r\n", (int)Tick_Get( SECONDS ));
		return NULL;
	}
	else
	{
//		memset(dlms_client_get_dlms_id_request(), 0 , 40*sizeof(char));
	}
	LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> Check GENERIC 2 rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n", (int)Tick_Get( SECONDS ), (int)dlms_log.msg_events_rd_ini_addr, (int)dlms_log.msg_events_rd_end_addr, (int)dlms_log.raw.msg_num, (int)strlen(dlms_telegram_value.raw_dlms));
	snprintf(header,
			sizeof( dlms_telegram_value.created_sensor_value_date ) + sizeof(dlms_telegram_value.created_sensor_value) + 60,//+1
			"%s|%s|%s;%d|%s%s|%d;%d;",
			ME910_IMEI(),
			__decode_tag(dlms_telegram_value.frame_type),
			dlms_telegram_value.dlms_meter_id,
			(int)dlms_telegram_value.num_meter,
			dlms_telegram_value.created_sensor_value_date,
			dlms_telegram_value.created_sensor_value,
			(int)dlms_telegram_value.capture_period,
			(int)dlms_telegram_value.num_generic_obis);
	str_len = strlen(header);

	memset(objects, 0, sizeof(objects));
	for ( i = 0; i < dlms_telegram_value.num_generic_obis; i++ )
	{
		sprintf(objects + obj_len,
				"%s;",
				dlms_telegram_value.dlms_obis_capture_objects[i]
				);
		obj_len = strlen(objects);
	}
	LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> Check GENERIC 3 rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n", (int)Tick_Get( SECONDS ), (int)dlms_log.msg_events_rd_ini_addr, (int)dlms_log.msg_events_rd_end_addr, (int)dlms_log.raw.msg_num, (int)strlen(dlms_telegram_value.raw_dlms));
	idRequest_len = strlen(id_request) + 2;
	memset(header,  0, sizeof(header));
	memset(objects, 0, sizeof(objects));

	sprintf(header,
			"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
			"Content-Type: text/plain\r\n"
			"Host:%s\r\n"
			"Content-Length: %d\r\n"
			"\r\n",
			Telit_dev_identifier(),
			param.server[ME910_server_get_index()].name,
			(int)((int)strlen(dlms_telegram_value.raw_dlms) + str_len + obj_len + idRequest_len )
	);
	str_len = 0;
	obj_len = 0;
	str_len = strlen(header);
	snprintf(header + str_len,
			sizeof( dlms_telegram_value.created_sensor_value_date )+ sizeof(dlms_telegram_value.created_sensor_value) + 60,//+1
			"%s|%s|%s;%d|%s%s|",
			ME910_IMEI(),
			__decode_tag(dlms_telegram_value.frame_type),
			dlms_telegram_value.dlms_meter_id,
			(int)dlms_telegram_value.num_meter,
			dlms_telegram_value.created_sensor_value_date,
			dlms_telegram_value.created_sensor_value);
	str_len = strlen(header);
	memset(objects, 0, sizeof(objects));
	for ( i = 0; i < dlms_telegram_value.num_generic_obis; i++ )
	{
		sprintf(objects + obj_len,
				"%s;",
				dlms_telegram_value.dlms_obis_capture_objects[i]
				);
		obj_len = strlen(objects);
	}
	sprintf(header + str_len,
			"%d;%d;%s",
			(int)dlms_telegram_value.capture_period,
			(int)dlms_telegram_value.num_generic_obis,
			objects
		);
	str_len = strlen(header);
	Telit_write_data_length(header, strlen(header));

	do
	{
		if ( dlms_telegram_value.raw_len > MAX_DLMS_TELEGRAM_VALUES )
		{
			sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value.raw_dlms,
					dlms_log.msg_events_rd_ini_addr + read_size,
					MAX_DLMS_TELEGRAM_VALUES);
			if (dlms_telegram_value.raw_dlms[0] == 0xFF)
			{
				dlms_telegram_value.raw_dlms[0] = 0;
				LOGLIVE(LEVEL_1,"LOGLIVE> %d DLMS LOG> MEMORY CORRUPTED!!!!!! Erase sector: rd_ini:0x%X rd_end:0x%X msg_num:%d len:%d\r\n",
						(int)Tick_Get( SECONDS ), (int)dlms_log.msg_events_rd_ini_addr,
						(int)dlms_log.msg_events_rd_end_addr, (int)dlms_log.raw.msg_num,
						(int)strlen(dlms_telegram_value.raw_dlms));
	    		sFLASH_EraseSector(dlms_log.msg_events_rd_ini_addr&0xFFFFF000);
	    		HAL_Delay(100);
				break;
			}
			else
			{
				snprintf(dlms_telegram_string,
						MAX_DLMS_TELEGRAM_VALUES + 1 + 2 + 40,
						"%s|"
						"%s|",
						dlms_telegram_value.raw_dlms,
						id_request);
			}
			str_len     = strlen(dlms_telegram_string);
			read_size  += MAX_DLMS_TELEGRAM_VALUES;
			bytes_left  = dlms_telegram_value.raw_len - read_size;
			dlms_telegram_value.raw_len = bytes_left;
			*frame_size = MAX_DLMS_TELEGRAM_VALUES;
			Telit_write_data_length(dlms_telegram_string, str_len);
		}
		else
		{
			sFLASH_ReadBuffer( (uint8_t *)&dlms_telegram_value.raw_dlms,
					dlms_log.msg_events_rd_ini_addr + read_size,
					dlms_telegram_value.raw_len);
			sprintf(dlms_telegram_string,
					"%s|"
					"%s|",
					dlms_telegram_value.raw_dlms,
					id_request);
			str_len     = strlen(dlms_telegram_string);
			read_size   = dlms_telegram_value.raw_len;
			*frame_size = dlms_telegram_value.raw_len;
			bytes_left  = dlms_telegram_value.raw_len - read_size;
			Telit_write_data_length(dlms_telegram_string, str_len);
		}
	}while (bytes_left != 0);

	*frame_size = str_len;

	HAL_Delay(100);
	__programWord();

	return dlms_telegram_string;
}

void dlms_log_measures_reset( void )
{
	uint32_t param_index;
	uint32_t num_params;

	__readDLMSLog();

	num_params = 4;//modbus_sensors_log.num_params;

	for ( param_index = 0; param_index < num_params; param_index++ ) {
		if ( dlms_log.dlms_val[param_index].init_val == 0xAAAA ) {
			dlms_log.samples                               = 0;
			dlms_log.dlms_val[param_index].samples       = 0;
		}
	}
	__programWord();
}

void dlms_log_params_reset_telegram_values_pointers( void )
{
	if ( dlms_log.init == 0xAAAA ) {
		dlms_log.msg_wr_address          = dlms_telegram_values_address;
		dlms_log.msg_rd_address          = dlms_telegram_values_address;
		dlms_log.msg_rd_address_backup   = dlms_telegram_values_address;
		dlms_log.msg_rd_ini_addr         = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr         = dlms_telegram_values_address;
		dlms_log.msg_rd_end_addr_backup  = dlms_telegram_values_address;
		dlms_log.msg_load_wr_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_address     = dlms_loadprofile_address;
		dlms_log.msg_load_rd_ini_addr    = dlms_loadprofile_address;
		dlms_log.msg_load_rd_end_addr    = dlms_loadprofile_address;
		dlms_log.msg_events_wr_address   = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_address   = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_ini_addr  = dlms_eventsprofile_address;
		dlms_log.msg_events_rd_end_addr  = dlms_eventsprofile_address;
		dlms_log.raw.msg_num             = 0;
		dlms_log.raw.msg_num_backup      = 0;
		dlms_log.raw.len                 = 0;
		dlms_telegram_value.wr_addr      = dlms_telegram_values_address;
		dlms_telegram_value.rd_addr      = dlms_telegram_values_address;
		__programWord();
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS_LOG> Reset Telegram Values.\r\n", (int)Tick_Get( SECONDS ));
	}
}

uint8_t write_dlms_lock;
void dlms_log_write_lock( uint8_t _write_lock )
{
	write_dlms_lock = _write_lock;
}

/**
  * @brief Writes MODBUS raw data to FLASH memory
  * @retval 1
  */
#define WRITE_MSG_PENDING       ((0) == CircularBuffer_IsEmpty(dlms_client_get_write_msg_queue()))
uint32_t dlms_raw_frame_register(void)
{
	/* First write ever or Local Tool wants MODBUS sensor measure.
	 * Also, time should be initialized or in process of taking it from server. */
	if ((0 == write_dlms_lock) && (1 == Tick_cloud_time_init()))
	{
		write_dlms_lock = 1;
//		if (( 0 == udp_protocol_get_on_demand_command() ) || ( 2 == udp_protocol_get_on_demand_command() ) || ( 3 == udp_protocol_get_on_demand_command() ))
		if WRITE_MSG_PENDING
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Write Queue.\r\n", (int)Tick_Get( SECONDS ));
			write_task_t con_write_task = (read_task_t)CircularBuffer_Get(dlms_client_get_write_msg_queue());
//			dlms_client_table_read_client_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(),  dlms_client_get_client_obis_profile(), con_write_task);
			dlms_client_set_frame_type_last_write(con_write_task);
			dlms_log_raw_modbus();
			uint32_t not_send_frame = dlms_log_params_store_raw_modbus(con_write_task);
//			CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
			if ((( CircularBuffer_Read(dlms_client_get_send_msg_queue() ) >= (dlms_client_get_obis_profile_frame_type() + DLMS_WAIT_FOR_SEND_INST_PROF_1) )
			  && ( CircularBuffer_Read(dlms_client_get_send_msg_queue() ) <= (dlms_client_get_obis_profile_frame_type() + DLMS_WAIT_FOR_SEND_MAX_DEMAND_PROF_1)) && ( 0 == not_send_frame ) )
			 || ( 1 == dlms_log_get_frame_tunneling() )
			 || ( 2 == not_send_frame )
			   )
			{
    			shutdown_setInitTelitModule(1);
    			CircularBuffer_Get(dlms_client_get_send_msg_queue());
    			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
    			CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_client_get_obis_profile_frame_type());
    			message_queue_write(SEND_MODBUS_SENSOR);
    			shutdown_reset_watchdog();
    		}
			if ((( CircularBuffer_Read(dlms_client_get_send_msg_queue() ) >= (dlms_client_get_obis_profile_frame_type() + DLMS_WAIT_FOR_SEND_INST_PROF_1) )
			  && ( CircularBuffer_Read(dlms_client_get_send_msg_queue() ) <= (dlms_client_get_obis_profile_frame_type() + DLMS_WAIT_FOR_SEND_MAX_DEMAND_PROF_1))
			  && ( 1 == not_send_frame ) )
			 )
			{
				CircularBuffer_Get(dlms_client_get_send_msg_queue());
			}
//			if ( 3 == udp_protocol_get_on_demand_command() )
//			{
//				udp_protocol_set_on_demand_command(1);
//			}
		}
		else if ( 1 == udp_protocol_get_on_demand_command() )
		{
			if ( 1 == dlms_client_get_read_meter() )
			{
				dlms_client_set_read_meter(0);
			}
		}
#if defined(UNE82326)
		if ((0 == shutdown_initTelitModule()) && (0 == une82326_get_start_comm()))
		{
#elif defined (MBUS)
			if ((0 == shutdown_initTelitModule()) && (0 == mbus_get_start_comm())
			&& (( ( 1 == dlms_client_get_dlms_enable() ) && (( 1 == CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue()) ) ) )
			|| ( 0 == dlms_client_get_dlms_enable()) )
			)
		{
#endif
#ifdef DLMS
			if (MODBUS_SESSION_END == modbus_get_end_session())
			{
#endif
				shutdown_set( 1, rtc_system_getReadSensorAlarmCycleNextInSeconds() );
#ifdef DLMS
			}
#endif
		}
	}
	return 1;
}

uint32_t dlms_raw_frame_tunneling_register(void)
{
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Get Write Queue.\r\n", (int)Tick_Get( SECONDS ));
	write_task_t con_write_task = (read_task_t)CircularBuffer_Get(dlms_client_get_write_msg_queue());
	dlms_client_set_frame_type_last_write(con_write_task);
	dlms_log_raw_modbus();
	uint32_t not_send_frame = dlms_log_params_store_raw_modbus_tunneling(con_write_task);
	shutdown_setInitTelitModule(1);
//	CircularBuffer_Get(dlms_client_get_send_msg_queue());
	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
	CircularBuffer_Put(dlms_client_get_send_msg_queue(), dlms_client_get_obis_profile_frame_type());
	message_queue_write(SEND_MODBUS_SENSOR);
	shutdown_reset_watchdog();

	return not_send_frame;
}
