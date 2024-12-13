/*
 * modbus_sensors_log.c
 *
 *  Created on: 27 ene. 2020
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

#include "modbus_sensors_log.h"
#include "modbus_sensors.h"
#include "aqualabo_modbus.h"
#include "generic_modbus.h"
#include "une82326.h"
#include "mbus.h"
#include "rtc_system.h"
#include "modbus.h"
#include "generic_modbus_table.h"
#include "ad.h"
#include "gpio.h"
#include "spi_flash.h"
#include "tick.h"
#include "shutdown.h"
#include "params.h"
#include "ME910.h"

//#define MAX_MODBUS_TELEGRAM_VALUES (2048)//(512)
#define MODBUS_PARAMS_NUM          (4)
#define DECIMAL_DIGITS             (3)

#define GENERIC_MODBUS_TABLE GENERIC_MODBUS_TABLE_TOTAL_SIZE
#define GENERIC_MODBUS_SIZE  ((0x4FFFFF) - GENERIC_MODBUS_TABLE)

typedef struct _modbus_vals {
	char     param_val[7];
	char     param_avg[7];
	char     param_max[7];
	char     param_min[7];
	float_t  param_val_dec;
	float_t  param_acc_dec;
	float_t  param_avg_dec;
	float_t  param_max_dec;
	float_t  param_min_dec;
	uint32_t init_val;
	uint32_t samples;
	uint32_t sensor_id;
	uint32_t num_chars;
	uint32_t msg_num;
} modbus_param_value;

/** Structure to save the raw MODBUS telegram*/
typedef struct _raw_modbus {
	char                raw_modbus_telegram[512];	/*!< MODBUS raw data array */
	uint32_t            len;						/*!< length of the raw telegram */
	uint32_t            msg_num;					/*!< telegram counter */
	uint32_t            msg_num_backup;				/*!< telegram counter backup */
	uint32_t            slave_id;                   /*!< slave id telegram */
	uint8_t             dlt645_address[14];         /*!< dlt address telegram */
} raw_modbus;

/** Structure to save the MODBUS sensor data, pointers and counters in flash
 * this is saved to _modbus_start_address = 0x780000
 * */
struct _modbus_sensors_log{
	uint32_t      	    samples;			/*!< */
	uint32_t      	    init;				/*!<  */
	uint32_t            num_params;			/*!<  */
	uint32_t            msg_wr_address;		/*!<  */
	uint32_t            msg_rd_address;		/*!<  */
	uint32_t            msg_rd_address_backup;	/*!<  */
    uint32_t            msg_rd_ini_addr;	/*!<  */
    uint32_t 		    msg_rd_end_addr;	/*!<  */
    uint32_t 		    msg_rd_end_addr_backup;	/*!<  */
	modbus_param_value  modbus_val[4];		/*!<  */
	raw_modbus          raw;				/*!< structure that contains raw telegram */
} modbus_sensors_log;

typedef struct {
	char      param_val[4][7];
	char      param_avg[4][7];
	char      param_max[4][7];
	char      param_min[4][7];
	char      raw_modbus[MAX_MODBUS_TELEGRAM_VALUES];
	char      created_sensor_value[7];
	char      created_sensor_value_date[9];
	uint32_t  slave_id;
	uint8_t   dlt645_address[14];
	uint32_t  msg_num;
	uint32_t  raw_len;
	uint32_t  wr_addr;
	uint32_t  rd_addr;
	uint32_t  num_params;
	uint32_t  is_command;
} modbus_telegram_values;//#define NITEMS(x) (sizeof(x)/sizeof(x[0]))

modbus_telegram_values modbus_telegram_value;

char modbus_telegram_string[MAX_MODBUS_TELEGRAM_VALUES];

typedef struct {
	uint32_t overpressure_alarm;
	uint32_t lowpressure_alarm;
	uint32_t operating_alarm;
} modbus_alarms;

modbus_alarms modbus_params_alarms;

extern char _modbus_start_address[], _modbus_meas_size[];

#define INIT  (0)
#define INDEX (1)

#define MODBUS_SENSORS_PAGE_SIZE sFLASH_SPI_PAGESIZE

static char message_value_modbus[7];

uint32_t modbus_sensors_page_address = (uint32_t)_modbus_start_address + 0x1000;
float_t  modbus_sensors_page[ MODBUS_SENSORS_PAGE_SIZE ];

uint32_t  modbus_sensors_telegram_values_address     = (uint32_t)_modbus_start_address + 0x2000 + GENERIC_MODBUS_TABLE;//0x2000;
uint32_t  modbus_sensors_telegram_values_max_address = (uint32_t)_modbus_start_address + GENERIC_MODBUS_TABLE + GENERIC_MODBUS_SIZE - 1;//0x200000 - 1;//(uint32_t)_modbus_start_address + 0x80000 - 1;
static uint32_t   modbus_sensors_telegram_log_size   = 0;

static uint8_t __programWord( void );
static uint8_t __programTelegramValueWord( void );
static uint8_t __readModbusSensorsLog( void );

uint32_t modbus_sensors_log_init( uint32_t _num_params )
{
	modbus_telegram_value.num_params = _num_params;
	modbus_sensors_log.num_params    = _num_params;
	return 0;
}

uint32_t modbus_sensors_get_num_params( void )
{
	return modbus_sensors_log.num_params;
}

uint8_t modbus_sensors_log_CheckDataInMem( void )
{
//	uint8_t ret = ( modbus_sensors_log.msg_wr_address != modbus_sensors_log.msg_rd_address )?1:0;
	uint8_t ret = ( modbus_sensors_log.msg_rd_ini_addr != modbus_sensors_log.msg_rd_end_addr )?1:0;

	return ret;
}

uint8_t __readModbusSensorsValuePage( void )
{
	sFLASH_ReadBuffer( (uint8_t *)&modbus_sensors_page, modbus_sensors_page_address, sizeof( modbus_sensors_page ) );

	return 0;
}

static void __programModbusSensorsValuePage( uint32_t i )
{
	uint8_t index;
	uint32_t size_page = sizeof( modbus_sensors_page );
	uint32_t token;

	sFLASH_ReadBuffer( (uint8_t *)&modbus_sensors_page, modbus_sensors_page_address, size_page  );

	token = (uint32_t)modbus_sensors_page[0];
	if ( token != 0xA5 ) {
		memset( modbus_sensors_page, 0, sizeof( modbus_sensors_page ));
		modbus_sensors_page[ INIT ]  = 0xA5;
		modbus_sensors_page[ INDEX ] = 1;
	}

	index = (uint8_t)modbus_sensors_page[ INDEX ] + 1;

	if ( index == MODBUS_SENSORS_PAGE_SIZE - 14 ) {
		index = INDEX + 1;
	}

	modbus_sensors_page[ INDEX ] = index;
	modbus_sensors_page[ index ] = modbus_sensors_log.modbus_val[i].param_val_dec;

    sFLASH_EraseSector((uint32_t)modbus_sensors_page_address);
    sFLASH_WriteBuffer((uint8_t *)&modbus_sensors_page, (uint32_t)modbus_sensors_page_address, size_page);
}

/**
  * @brief Writes structure modbus_sensor_log in sector of address _modbus_start_address (0x780000)
  * @retval 0
  */
static uint8_t __programWord( void )
{
	uint32_t  size, ret = 0;
	uint8_t  *ptr;

    size = sizeof(modbus_sensors_log);
    ptr  = (uint8_t *) &modbus_sensors_log;

    sFLASH_EraseSector((uint32_t)_modbus_start_address);
    sFLASH_WriteBuffer(ptr, (uint32_t)_modbus_start_address, size);

    return ret;
}

static uint32_t __getSensorId( uint32_t index )
{
#define TEMP  (0)
#define PH    (1)
#define REDOX (2)
#define OTHER (3)
	uint32_t sensor_id = 0;

	switch (index) {
	case TEMP:
		sensor_id = 4;
		break;
	case PH:
		sensor_id = 2;
		break;
	case REDOX:
		sensor_id = 3;
		break;
	case OTHER:
		break;
	default:
		break;
	}
	return sensor_id;
}

static uint8_t __writeModbusSensorsLog( uint32_t i )
{
	float_t m;

	modbus_sensors_log.samples++;
	modbus_sensors_log.modbus_val[i].samples++;

	m = modbus_sensors_log.modbus_val[i].param_val_dec;
	modbus_sensors_log.modbus_val[i].sensor_id = __getSensorId(i);

	if ( modbus_sensors_log.modbus_val[i].init_val != 0xAAAA ) {
		modbus_sensors_log.modbus_val[i].init_val      = 0xAAAA;
		modbus_sensors_log.modbus_val[i].param_min_dec = m;
		modbus_sensors_log.modbus_val[i].param_max_dec = m;
		modbus_sensors_log.modbus_val[i].param_acc_dec = 0;
		modbus_sensors_log.modbus_val[i].param_avg_dec = modbus_sensors_log.modbus_val[i].param_acc_dec/modbus_sensors_log.modbus_val[i].samples;
		modbus_sensors_log.modbus_val[i].samples       = 1;
		modbus_sensors_log.modbus_val[i].msg_num       = 0;
		if ( modbus_sensors_log.init != 0xAAAA ) {
			modbus_sensors_log.init                        = 0xAAAA;
			modbus_sensors_log.msg_wr_address              = modbus_sensors_telegram_values_address;
			modbus_sensors_log.msg_rd_address              = modbus_sensors_telegram_values_address;
			modbus_sensors_log.msg_rd_ini_addr             = modbus_sensors_telegram_values_address;
			modbus_sensors_log.msg_rd_end_addr             = modbus_sensors_telegram_values_address;
			modbus_telegram_value.wr_addr        		   = modbus_sensors_telegram_values_address;
			modbus_telegram_value.rd_addr        		   = modbus_sensors_telegram_values_address;
			modbus_sensors_log.samples                     = 1;
		}
	}

	modbus_sensors_log.modbus_val[i].param_acc_dec += m;
	modbus_sensors_log.modbus_val[i].param_avg_dec  = modbus_sensors_log.modbus_val[i].param_acc_dec/modbus_sensors_log.modbus_val[i].samples;

    if( m > modbus_sensors_log.modbus_val[i].param_max_dec ) {
    	modbus_sensors_log.modbus_val[i].param_max_dec = m;
    }

    if( m < modbus_sensors_log.modbus_val[i].param_min_dec ) {
    	modbus_sensors_log.modbus_val[i].param_min_dec = m;
    }

	common_lib_ftoa(modbus_sensors_log.modbus_val[i].param_avg_dec, modbus_sensors_log.modbus_val[i].param_avg, DECIMAL_DIGITS);
    common_lib_ftoa(modbus_sensors_log.modbus_val[i].param_min_dec, modbus_sensors_log.modbus_val[i].param_min, DECIMAL_DIGITS);
    common_lib_ftoa(modbus_sensors_log.modbus_val[i].param_max_dec, modbus_sensors_log.modbus_val[i].param_max, DECIMAL_DIGITS);

	__programWord();

    return 0;
}

/**
  * @brief Reads MODBUS sensor data from FLASH memory and stores it to modbus_sensor_log structure.
  * @retval 0
  */
static uint8_t __readModbusSensorsLog(void)
{
	sFLASH_ReadBuffer((uint8_t *) &modbus_sensors_log, (uint32_t) _modbus_start_address, sizeof(modbus_sensors_log));

	return 0;
}

uint32_t modbus_sensors_log_get_sensor_id( uint32_t index )
{
	__readModbusSensorsLog();

	return modbus_sensors_log.modbus_val[index].sensor_id;
}

uint32_t modbus_sensors_log_get_num_chars_param( uint32_t index )
{
	__readModbusSensorsLog();

	return modbus_sensors_log.modbus_val[index].num_chars;
}

char * modbus_sensors_log_GetParamAvg( uint32_t i )
{
	__readModbusSensorsLog();

	common_lib_ftoa(modbus_sensors_log.modbus_val[i].param_avg_dec, modbus_sensors_log.modbus_val[i].param_avg, DECIMAL_DIGITS);

	return modbus_sensors_log.modbus_val[i].param_avg;
}

char * modbus_sensors_log_GetParamMin( uint32_t i )
{
	__readModbusSensorsLog();

	common_lib_ftoa(modbus_sensors_log.modbus_val[i].param_min_dec, modbus_sensors_log.modbus_val[i].param_min, DECIMAL_DIGITS);

	return modbus_sensors_log.modbus_val[i].param_min;
}

char * modbus_sensors_log_GetParamMax( uint32_t i )
{
	__readModbusSensorsLog();

	common_lib_ftoa(modbus_sensors_log.modbus_val[i].param_max_dec, modbus_sensors_log.modbus_val[i].param_max, DECIMAL_DIGITS);

	return modbus_sensors_log.modbus_val[i].param_max;
}

char * modbus_sensors_log_get_param( uint32_t param_index, uint32_t index )
{
	aqualabo_modbus_get(param_index, &(modbus_sensors_log.modbus_val[index].param_val_dec));

	common_lib_ftoa(modbus_sensors_log.modbus_val[index].param_val_dec, modbus_sensors_log.modbus_val[index].param_val, DECIMAL_DIGITS);
	modbus_sensors_log.modbus_val[index].num_chars = strlen(modbus_sensors_log.modbus_val[index].param_val);
	return modbus_sensors_log.modbus_val[index].param_val;
}

void modbus_sensors_log_Log( void )
{
	uint32_t i;
	uint32_t num_params = modbus_telegram_value.num_params;

	__readModbusSensorsLog();
	 modbus_sensors_log.num_params = num_params;

	modbus_sensors_log_get_param( 0, 0 );//Temperature
	__writeModbusSensorsLog(0);
	__programModbusSensorsValuePage(0);

	for ( i = 0; i < num_params - 1; i++ ) {
		modbus_sensors_log_get_param( modbus_sensors_get_param_index() + i, i + 1 );
		__writeModbusSensorsLog( i + 1 );
		__programModbusSensorsValuePage( i + 1 );
//		sFLASH_ReadBuffer( (uint8_t *)&modbus_telegram_value,
//				modbus_sensors_log.msg_rd_address,
//				modbus_telegram_value.len );
	}

	__readModbusSensorsValuePage();
}

/**
  * @brief Writes MODBUS raw data to FLASH memory
  * @retval None
  */
void modbus_sensors_log_raw_modbus(void)
{
	uint16_t quantity;
	uint32_t i,j,slave;
	uint8_t  on_demand = 0;

	/* union to handle different type of data in memory*/
    union
    {
		uint32_t w[128];
        uint16_t s[256];
        uint8_t  b[512];
    } raw_temp, raw;
    uint16_t raw_data_buffer[256];
    uint8_t  raw_dlt645_data_buffer[512];

    memset(raw_temp.b, 0, sizeof(raw_temp.b));
    memset(raw.b, 0, sizeof(raw.b));
    memset(raw_dlt645_data_buffer, 0, sizeof(raw_dlt645_data_buffer));
    memset(raw_data_buffer, 0, sizeof(raw_data_buffer));

    /** Reads from 0x780000 base memory and copies to modbus_sensor_log*/
	__readModbusSensorsLog();

	if (1 == params_wq_on())
	{
		/* TOASK Aqualabo 10 measurements?*/
		memcpy(raw_temp.w, aqualabo_modbus_get_raw_data(), 10 * sizeof(float_t));
		quantity = 10;
		for (i = 0; i < 512; i = i + 4)
		{
			for (j = 0; j < 4; j++)
			{
				raw.b[i + (3 - j)] = raw_temp.b[i + j];
			}
		}

		/* Only comes here the first time or after a memory erase */
		if (modbus_sensors_log.init != 0xAAAA)
		{
			modbus_sensors_log.init            = 0xAAAA;
			modbus_sensors_log.raw.msg_num     = 0;
			modbus_sensors_log.msg_wr_address  = modbus_sensors_telegram_values_address;
			modbus_sensors_log.msg_rd_address  = modbus_sensors_telegram_values_address;
			modbus_sensors_log.msg_rd_ini_addr = modbus_sensors_telegram_values_address;
			modbus_sensors_log.msg_rd_end_addr = modbus_sensors_telegram_values_address;
			modbus_telegram_value.msg_num      = 0;
			modbus_telegram_value.wr_addr      = modbus_sensors_telegram_values_address;
			modbus_telegram_value.rd_addr      = modbus_sensors_telegram_values_address;
		}

		/** Clears the raw data MODBUS telegram */
		memset(modbus_sensors_log.raw.raw_modbus_telegram, 0, sizeof(modbus_sensors_log.raw.raw_modbus_telegram));
		/** Converts raw data MODBUS telegram to a string in hexadecimal */
		common_lib_string2hexString((char *) raw.b, modbus_sensors_log.raw.raw_modbus_telegram, 2 * quantity);
		modbus_sensors_log.raw.len = strlen(modbus_sensors_log.raw.raw_modbus_telegram);
//		/** Saves modbus_sensors_log raw telegram to FLASH address _modbus_start_address = 0x780000*/
//		__programWord();

		if ( modbus_sensors_log.init == 0xAAAA ) {
			modbus_sensors_log.raw.msg_num++;
			memcpy( modbus_telegram_value.raw_modbus,
					modbus_sensors_log.raw.raw_modbus_telegram,
					sizeof(modbus_sensors_log.raw.raw_modbus_telegram) );
			modbus_telegram_value.raw_len = modbus_sensors_log.raw.len;
			modbus_telegram_value.msg_num++;
		}

		memcpy( modbus_telegram_value.created_sensor_value_date,
				(char *)( rtc_system_getCreatedSensorValueDate() ),
				sizeof( modbus_telegram_value.created_sensor_value_date  ) );
		memcpy( modbus_telegram_value.created_sensor_value,
				(char *)( rtc_system_getCreatedSensorValueTime() ),
				sizeof( modbus_telegram_value.created_sensor_value  ) );

		__programTelegramValueWord();
		HAL_Delay(100);
		/** Saves modbus_sensors_log raw telegram to FLASH address _modbus_start_address = 0x780000*/
		__programWord();
	}
	else
	{
		for (slave = 0; slave < params_get_modbus_slave_num(); slave++)
		{
			if ( 1 == generic_485_get_dtl645() )
			{
				/* Generic MODBUS quantity is updated in the function call */
				memcpy(raw_temp.b,
						generic_485_get_raw_dlt645(&quantity, raw_dlt645_data_buffer, slave,
								modbus_sensors_log.raw.dlt645_address), sizeof(modbus_sensors_log.raw.raw_modbus_telegram));
				for (i = 0; i < 512; i++)
				{
					raw.b[i] = raw_temp.b[i];
				}
			}
			else
			{
				/* Generic MODBUS quantity is updated in the function call */
				memcpy(raw_temp.s,
						generic_485_get_raw_modbus(&quantity, raw_data_buffer, slave,
								&modbus_sensors_log.raw.slave_id,&on_demand), sizeof(modbus_sensors_log.raw.raw_modbus_telegram) );
				for (i = 0; i < 512; i = i + 2)
				{
					for (j = 0; j < 2; j++)
					{
						raw.b[i + (1 - j)] = raw_temp.b[i + j];
					}
				}
			}

			/* Only comes here the first time or after a memory erase */
			if (modbus_sensors_log.init != 0xAAAA)
			{
				modbus_sensors_log.init            = 0xAAAA;
				modbus_sensors_log.raw.msg_num     = 0;
				modbus_sensors_log.msg_wr_address  = modbus_sensors_telegram_values_address;
				modbus_sensors_log.msg_rd_address  = modbus_sensors_telegram_values_address;
				modbus_sensors_log.msg_rd_ini_addr = modbus_sensors_telegram_values_address;
				modbus_sensors_log.msg_rd_end_addr = modbus_sensors_telegram_values_address;
				modbus_telegram_value.msg_num      = 0;
				modbus_telegram_value.wr_addr      = modbus_sensors_telegram_values_address;
				modbus_telegram_value.rd_addr      = modbus_sensors_telegram_values_address;
			}

			/** Clears the raw data MODBUS telegram */
			memset(modbus_sensors_log.raw.raw_modbus_telegram, 0, sizeof(modbus_sensors_log.raw.raw_modbus_telegram));
			/** Converts raw data MODBUS telegram to a string in hexadecimal */
			common_lib_string2hexString_with_separators((char *) raw.b, modbus_sensors_log.raw.raw_modbus_telegram, quantity);// 2* quantity
			if ( 1 == generic_485_get_dtl645())
			{
				modbus_sensors_log.raw.raw_modbus_telegram[2*quantity] = '\0';
			}
			modbus_sensors_log.raw.len       = strlen(modbus_sensors_log.raw.raw_modbus_telegram);
			modbus_telegram_value.is_command = on_demand;

			if ( modbus_sensors_log.init == 0xAAAA ) {
				modbus_sensors_log.raw.msg_num++;
				memcpy( modbus_telegram_value.raw_modbus,
						modbus_sensors_log.raw.raw_modbus_telegram,
						sizeof(modbus_sensors_log.raw.raw_modbus_telegram) );
				modbus_telegram_value.raw_len  = modbus_sensors_log.raw.len;
				modbus_telegram_value.slave_id = modbus_sensors_log.raw.slave_id;
				memcpy(modbus_telegram_value.dlt645_address, modbus_sensors_log.raw.dlt645_address, sizeof(modbus_sensors_log.raw.dlt645_address));
				modbus_telegram_value.msg_num++;
			}

			memcpy( modbus_telegram_value.created_sensor_value_date,
					(char *)( rtc_system_getCreatedSensorValueDate() ),
					sizeof( modbus_telegram_value.created_sensor_value_date  ) );
			memcpy( modbus_telegram_value.created_sensor_value,
					(char *)( rtc_system_getCreatedSensorValueTime() ),
					sizeof( modbus_telegram_value.created_sensor_value  ) );

			__programTelegramValueWord();
			HAL_Delay(100);
			/** Saves modbus_sensors_log raw telegram to FLASH address _modbus_start_address = 0x780000*/
			__programWord();
		}
	}
}

static uint32_t __nextAddress(unsigned int n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr   = modbus_sensors_log.msg_wr_address + n;
	next_sector = ( modbus_sensors_log.msg_wr_address + 0x1000 ) & 0xFFFFF000;

    if ( ( modbus_sensors_log.msg_wr_address & 0x00000FFF ) == 0 ) {
    	sFLASH_EraseSector(modbus_sensors_log.msg_wr_address );
    } else if (  next_addr > next_sector ) {
    	sFLASH_EraseSector( next_sector );
    }

    addr = modbus_sensors_log.msg_wr_address;
    modbus_sensors_log.msg_wr_address  += n;
    if ( modbus_sensors_log.msg_wr_address > modbus_sensors_telegram_values_max_address ) {
    	modbus_sensors_log.msg_wr_address = modbus_sensors_telegram_values_address;
    }

    if ( ( modbus_sensors_log.msg_wr_address == modbus_sensors_log.msg_rd_end_addr ) ) {
    	modbus_sensors_log.msg_rd_end_addr += modbus_sensors_telegram_log_size;
    }
    modbus_sensors_log.msg_rd_ini_addr = modbus_sensors_log.msg_wr_address;
    modbus_sensors_log.msg_rd_address  = modbus_sensors_log.msg_wr_address;

    return addr;
}

static uint8_t __programTelegramValueWord( void )
{
    uint32_t  size, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  *ptr;
	uint32_t next_addr;
	modbus_telegram_values record_check;

	size = sizeof(modbus_telegram_value);
    ptr  = (uint8_t *) &modbus_telegram_value;

    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 ) {
    	modbus_sensors_telegram_log_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	modbus_sensors_telegram_log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    next_addr = __nextAddress( modbus_sensors_telegram_log_size );
    sFLASH_WriteBuffer(ptr, next_addr, size);
    HAL_Delay(100);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    LOGLIVE(LEVEL_1, "LOGLIVE> %d MODBUS_SENSORS_LOG> Value Write address: 0x%X.--------------------msg_num:%d \r\n", (int)Tick_Get( SECONDS ), (int)next_addr, (int)modbus_sensors_log.raw.msg_num);

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	__NOP();
    	check_write = 0;
    } else {
    	__NOP();
    	check_write = 1;
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d MODBUS SENSORS LOG> WARNING!! CHECK WRITE in Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    }
    uint32_t num = 0;
    if ( 1 == check_write ) {
    	do {
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }

    return ret;
}

char *modbus_sensors_log_message_value_params_sensor( uint32_t i )
{
	float_t param;

	param = 12;

	common_lib_ftoa( param, message_value_modbus, DECIMAL_DIGITS );

//	if ( 0 == param_val ) {
//		modbus_sensors_log_SetOperatingAlarm(1);
//	}

	return message_value_modbus;
}

void modbus_sensors_log_params_store_telegram_value( void )
{
	uint32_t i;
	uint32_t num_params =  modbus_sensors_log.num_params;

	for ( i = 0; i < num_params; i++ ) {
		if ( modbus_sensors_log.modbus_val[i].init_val == 0xAAAA ) {
			modbus_sensors_log.modbus_val[i].msg_num++;
			strncpy(modbus_telegram_value.param_val[i], modbus_sensors_log.modbus_val[i].param_val, 7);
			strncpy(modbus_telegram_value.param_avg[i], modbus_sensors_log.modbus_val[i].param_avg, 7);
			strncpy(modbus_telegram_value.param_max[i], modbus_sensors_log.modbus_val[i].param_max, 7);
			strncpy(modbus_telegram_value.param_min[i], modbus_sensors_log.modbus_val[i].param_min, 7);
		}
	}

	if ( modbus_sensors_log.init == 0xAAAA ) {
		modbus_telegram_value.msg_num++;
	}

	memcpy( modbus_telegram_value.created_sensor_value_date,
			(char *)( rtc_system_getCreatedSensorValueDate() ),
			sizeof( modbus_telegram_value.created_sensor_value_date  ) );
	memcpy( modbus_telegram_value.created_sensor_value,
			(char *)( rtc_system_getCreatedSensorValueTime() ),
			sizeof( modbus_telegram_value.created_sensor_value  ) );

	__programTelegramValueWord();
	HAL_Delay(100);
	__programWord();
}

void modbus_sensors_log_params_store_raw_modbus( void )
{

	if ( modbus_sensors_log.init == 0xAAAA ) {
		modbus_sensors_log.raw.msg_num++;
		memcpy( modbus_telegram_value.raw_modbus,
				modbus_sensors_log.raw.raw_modbus_telegram,
				sizeof(modbus_sensors_log.raw.raw_modbus_telegram) );
		modbus_telegram_value.raw_len = modbus_sensors_log.raw.len;
		modbus_telegram_value.msg_num++;
	}

	memcpy( modbus_telegram_value.created_sensor_value_date,
			(char *)( rtc_system_getCreatedSensorValueDate() ),
			sizeof( modbus_telegram_value.created_sensor_value_date  ) );
	memcpy( modbus_telegram_value.created_sensor_value,
			(char *)( rtc_system_getCreatedSensorValueTime() ),
			sizeof( modbus_telegram_value.created_sensor_value  ) );

	__programTelegramValueWord();
	HAL_Delay(100);
	__programWord();
}

void modbus_sensors_log_recover_rd_address_backup( void )
{
	modbus_sensors_log.raw.msg_num     = modbus_sensors_log.raw.msg_num_backup;
	modbus_sensors_log.msg_rd_address  = modbus_sensors_log.msg_rd_address_backup;
	modbus_sensors_log.msg_wr_address  = modbus_sensors_log.msg_rd_address;
	modbus_sensors_log.msg_rd_ini_addr = modbus_sensors_log.msg_rd_address_backup;

	HAL_Delay(100);
	__programWord();
}

char datalogger_modbus_info[100];
char *modbus_sensors_log_GetDatalogger_Info( void )
{
	memset(datalogger_modbus_info,0,sizeof(datalogger_modbus_info));
	if ( generic_485_get_read_time() != 0 ) {
		sprintf(datalogger_modbus_info,
				"%d,0x%X,0x%X",
				(int)modbus_sensors_log.raw.msg_num,
				(int)modbus_sensors_log.msg_rd_address_backup,
				(int)modbus_sensors_log.msg_wr_address
		);
	}

	return datalogger_modbus_info;
}

char * modbus_sensors_log_params_sensor_param_telegram( uint32_t param_index, uint32_t value_length, uint32_t *measures_count  )
{
	uint32_t i, str_len = 0, num_it = 0, backup_address = 0;
	char date[9];

	num_it                                   = modbus_sensors_log.modbus_val[param_index].msg_num;
	backup_address                           = modbus_sensors_log.msg_rd_address;
	modbus_sensors_log.msg_rd_address_backup = modbus_sensors_log.msg_rd_address;

	for ( i = 0; i < num_it; i++ ) {
		sFLASH_ReadBuffer( (uint8_t *)&modbus_telegram_value,
//				modbus_sensors_log.msg_rd_address,
				backup_address,
				sizeof(modbus_telegram_value) );
		if ( (  str_len
				+ sizeof(modbus_telegram_value)
		     ) >= ( MAX_MODBUS_TELEGRAM_VALUES - 250 ) ) {
			break;
		}
		if ( ( 0 == i )
		  || ( modbus_telegram_value.created_sensor_value_date[6] != date[6] )
		  || ( modbus_telegram_value.created_sensor_value_date[7] != date[7] )
		  ) {
			if ( modbus_telegram_value.created_sensor_value_date[6] == (char)0xFF ) {
				modbus_sensors_log.modbus_val[param_index].msg_num = 0;
				modbus_sensors_log.samples = 0;
				modbus_sensors_log.modbus_val[param_index].samples = 0;
				modbus_sensors_log.modbus_val[param_index].param_min_dec = modbus_sensors_log.modbus_val[param_index].param_val_dec;
				modbus_sensors_log.modbus_val[param_index].param_max_dec = modbus_sensors_log.modbus_val[param_index].param_val_dec;
				modbus_sensors_log.modbus_val[param_index].param_acc_dec = 0;
				break;
			}
			snprintf(modbus_telegram_string,
					sizeof( modbus_telegram_value.created_sensor_value_date ) + 1,
					"%s|", modbus_telegram_value.created_sensor_value_date);
			str_len = strlen(modbus_telegram_string);
		}
		sprintf(modbus_telegram_string + str_len,
				"%s",
				modbus_telegram_value.created_sensor_value);
		str_len = strlen(modbus_telegram_string);
		sprintf(modbus_telegram_string + str_len,
				"%*s%*s%*s%*s",
				(int)value_length, modbus_telegram_value.param_val[param_index],
				(int)value_length, modbus_telegram_value.param_avg[param_index],
				(int)value_length, modbus_telegram_value.param_min[param_index],
				(int)value_length, modbus_telegram_value.param_max[param_index]);
		str_len = strlen(modbus_telegram_string);
		backup_address += modbus_sensors_telegram_log_size;
	    if ( backup_address > modbus_sensors_telegram_values_max_address ) {
	    	backup_address =  modbus_sensors_telegram_values_address;
	    }
	    modbus_sensors_log.modbus_val[param_index].msg_num--;
		memcpy( date,
				modbus_telegram_value.created_sensor_value_date,
				sizeof(modbus_telegram_value.created_sensor_value_date) + 1 );
	}
	if (modbus_telegram_string[0] == '\0') {
			sprintf(modbus_telegram_string,
					"%s;%s;%s;%s|",
					modbus_sensors_log_get_param( modbus_sensors_get_param_index() + param_index, param_index + 1 ),
					modbus_sensors_log_GetParamAvg(param_index),
					modbus_sensors_log_GetParamMax(param_index),
					modbus_sensors_log_GetParamMin(param_index));
	}

	modbus_sensors_log.samples                               = 0;
	modbus_sensors_log.modbus_val[param_index].samples       = 0;
	modbus_sensors_log.modbus_val[param_index].param_min_dec = modbus_sensors_log.modbus_val[param_index].param_val_dec;
	modbus_sensors_log.modbus_val[param_index].param_max_dec = modbus_sensors_log.modbus_val[param_index].param_val_dec;
	modbus_sensors_log.modbus_val[param_index].param_acc_dec = 0;

	if ( param_index == ( modbus_sensors_get_num_params() - 1 ) ) {
		modbus_sensors_log.msg_rd_address = backup_address;
		if ( modbus_sensors_log.msg_rd_address > modbus_sensors_telegram_values_max_address ) {
			modbus_sensors_log.msg_rd_address =  modbus_sensors_telegram_values_address;
		}
	}

	HAL_Delay(100);
	__programWord();

	return modbus_telegram_string;
}

static void __incReadModbusRawLog( void )
{
    if ( modbus_sensors_log.msg_rd_ini_addr - modbus_sensors_telegram_log_size >= (uint32_t) modbus_sensors_telegram_values_address ) {
    	modbus_sensors_log.msg_rd_ini_addr -= modbus_sensors_telegram_log_size;
    } else {
    	if ( modbus_sensors_telegram_values_max_address - modbus_sensors_telegram_log_size > modbus_sensors_log.msg_rd_end_addr ) {
    		modbus_sensors_log.msg_rd_ini_addr = modbus_sensors_telegram_values_max_address - modbus_sensors_telegram_log_size + 1;
    	} else {
    		modbus_sensors_log.msg_rd_ini_addr = modbus_sensors_log.msg_rd_end_addr;
    	}
    }

    if ( ( modbus_sensors_log.msg_rd_ini_addr <= modbus_sensors_log.msg_rd_end_addr ) ) {
//        ret = 0;
    }

    modbus_sensors_log.msg_wr_address = modbus_sensors_log.msg_rd_ini_addr;
    modbus_sensors_log.msg_rd_address = modbus_sensors_log.msg_rd_ini_addr;
}

uint32_t on_demand_backup = 0;
char * modbus_sensors_log_sensor_raw_telegram( uint32_t *measure_count )
{
	uint32_t i, str_len = 0, num_it = 0, backup_msg_num = 0, rd_ini_backup = 0, wr_backup = 0, rd_backup = 0;
	char date[10];

	__readModbusSensorsLog();

	num_it = modbus_sensors_log.raw.msg_num;
	if ( 0 == num_it ) {
		__NOP();
		return NULL;
	}

	rd_ini_backup                            = modbus_sensors_log.msg_rd_ini_addr;
	wr_backup                                = modbus_sensors_log.msg_wr_address;
	rd_backup                                = modbus_sensors_log.msg_rd_address;
	modbus_sensors_log.raw.msg_num_backup    = modbus_sensors_log.raw.msg_num;
	modbus_sensors_log.msg_rd_address_backup = modbus_sensors_log.msg_rd_ini_addr;
	*measure_count                           = *measure_count + 1;
	__incReadModbusRawLog();
	modbus_sensors_log.raw.msg_num--;
	if ( modbus_sensors_log.raw.msg_num > 0 ) {
//		num_it = modbus_sensors_log.raw.msg_num;
	}

	if ( 0 == num_it ) {
		sFLASH_ReadBuffer( (uint8_t *)&modbus_telegram_value,
				modbus_sensors_log.msg_rd_ini_addr,
				sizeof(modbus_telegram_value) );
		return NULL;
	}

	memset(modbus_telegram_string,             0, sizeof(modbus_telegram_string));
	memset((uint8_t *)&modbus_telegram_value,  0, sizeof(modbus_telegram_value));
	backup_msg_num = modbus_sensors_log.raw.msg_num;

	for ( i = 0; i < num_it; i++ )
	{
		sFLASH_ReadBuffer( (uint8_t *)&modbus_telegram_value,
				modbus_sensors_log.msg_rd_ini_addr,
				sizeof(modbus_telegram_value) );
		LOGLIVE(LEVEL_1,"LOGLIVE> %d MODBUS SENSOR LOG> rd_ini:0x%X num_it:%d. len:%d\r\n",(int)Tick_Get( SECONDS ), (int)modbus_sensors_log.msg_rd_ini_addr, (int)num_it, (int)str_len);
		if ( (  str_len
				+ modbus_telegram_value.raw_len
		) >=  ( MAX_MODBUS_TELEGRAM_VALUES - 250 ) )
		{
			modbus_sensors_log.msg_rd_ini_addr = rd_ini_backup;
			modbus_sensors_log.msg_wr_address  = wr_backup;
			modbus_sensors_log.msg_rd_address  = rd_backup;
			backup_msg_num                     = backup_msg_num + 1;
			*measure_count                     = *measure_count - 1;
			break;
		}
		if ( ( modbus_telegram_value.raw_len == 0 ) )
		{
			*measure_count = *measure_count + 1;
			if ( backup_msg_num > 0 )
			{
				rd_ini_backup = modbus_sensors_log.msg_rd_ini_addr;
				wr_backup     = modbus_sensors_log.msg_wr_address;
				rd_backup     = modbus_sensors_log.msg_rd_address;
				__incReadModbusRawLog();
				backup_msg_num--;
			}
			else
			{
				break;
			}
			memcpy( date,
					modbus_telegram_value.created_sensor_value_date,
					sizeof(modbus_telegram_value.created_sensor_value_date) + 1 );
			continue;
		}
		if ( modbus_telegram_value.created_sensor_value_date[6] == (char)0xFF )
		{
			backup_msg_num = 0;
			break;
		}
		if ( ( 1 == on_demand_backup ) && ( modbus_telegram_value.is_command != 1 ) )//Previous frame was on demand command.
		{
			modbus_sensors_log.msg_rd_ini_addr = rd_ini_backup;
			modbus_sensors_log.msg_wr_address  = wr_backup;
			modbus_sensors_log.msg_rd_address  = rd_backup;
			backup_msg_num                     = backup_msg_num + 1;
			*measure_count                     = *measure_count - 1;
			break;
		}
		else
		{
			on_demand_backup = modbus_telegram_value.is_command;
		}

		if ( 1 == generic_485_get_dtl645())
		{

			snprintf(modbus_telegram_string + str_len,
					sizeof( modbus_telegram_value.created_sensor_value_date ) + sizeof(modbus_telegram_value.slave_id) + 30,
					"%s|MF|%s|%s",
					ME910_IMEI(),
					modbus_telegram_value.dlt645_address,
					modbus_telegram_value.created_sensor_value_date);
		}
		else
		{
			if ( 1 == modbus_telegram_value.is_command)
			{
				snprintf(modbus_telegram_string + str_len,
						sizeof( modbus_telegram_value.created_sensor_value_date ) + sizeof(modbus_telegram_value.slave_id) + 30,
						"%s|MFW|%d|%s",
						ME910_IMEI(),
						(int)modbus_telegram_value.slave_id,
						modbus_telegram_value.created_sensor_value_date);
			}
			else
			{
				snprintf(modbus_telegram_string + str_len,
						sizeof( modbus_telegram_value.created_sensor_value_date ) + sizeof(modbus_telegram_value.slave_id) + 30,
						"%s|MF|%d|%s",
						ME910_IMEI(),
						(int)modbus_telegram_value.slave_id,
						modbus_telegram_value.created_sensor_value_date);
			}
		}
		str_len = strlen(modbus_telegram_string);
		if ( modbus_telegram_string[0] == '\0' )
		{
			modbus_sensors_log.msg_rd_ini_addr = rd_ini_backup;
			modbus_sensors_log.msg_wr_address  = wr_backup;
			modbus_sensors_log.msg_rd_address  = rd_backup;
			backup_msg_num                     = backup_msg_num + 1;
			*measure_count                     = *measure_count - 1;
			break;
		}
		sprintf(modbus_telegram_string + str_len,
				"%s|",
				modbus_telegram_value.created_sensor_value);
		str_len = strlen(modbus_telegram_string);

		sprintf(modbus_telegram_string + str_len,
				"%s|\r\n",
				modbus_telegram_value.raw_modbus);
		str_len = strlen(modbus_telegram_string);

		*measure_count = *measure_count + 1;
		if ( backup_msg_num > 0 )
		{
			rd_ini_backup = modbus_sensors_log.msg_rd_ini_addr;
			wr_backup     = modbus_sensors_log.msg_wr_address;
			rd_backup     = modbus_sensors_log.msg_rd_address;
			__incReadModbusRawLog();
			backup_msg_num--;
		}
		else
		{
			break;
		}
		memcpy( date,
				modbus_telegram_value.created_sensor_value_date,
				sizeof(modbus_telegram_value.created_sensor_value_date) + 1 );
	}

	modbus_sensors_log.raw.msg_num = backup_msg_num;

	HAL_Delay(100);
	__programWord();

	return modbus_telegram_string;
}

void modbus_sensors_log_measures_reset( void )
{
	uint32_t param_index;
	uint32_t num_params;

	__readModbusSensorsLog();

	num_params = 4;//modbus_sensors_log.num_params;

	for ( param_index = 0; param_index < num_params; param_index++ ) {
		if ( modbus_sensors_log.modbus_val[param_index].init_val == 0xAAAA ) {
			modbus_sensors_log.samples                               = 0;
			modbus_sensors_log.modbus_val[param_index].samples       = 0;
			modbus_sensors_log.modbus_val[param_index].param_min_dec = modbus_sensors_log.modbus_val[param_index].param_val_dec;
			modbus_sensors_log.modbus_val[param_index].param_max_dec = modbus_sensors_log.modbus_val[param_index].param_val_dec;
			modbus_sensors_log.modbus_val[param_index].param_acc_dec = 0;
		}
	}
	__programWord();
}

void modbus_sensors_log_params_reset_telegram_values_pointers( void )
{
	if ( modbus_sensors_log.init == 0xAAAA ) {
		modbus_sensors_log.msg_wr_address        = modbus_sensors_telegram_values_address;
		modbus_sensors_log.msg_rd_address        = modbus_sensors_telegram_values_address;
		modbus_sensors_log.msg_rd_address_backup = modbus_sensors_telegram_values_address;
		modbus_sensors_log.msg_rd_ini_addr       = modbus_sensors_telegram_values_address;
		modbus_sensors_log.msg_rd_end_addr       = modbus_sensors_telegram_values_address;
		modbus_sensors_log.msg_rd_end_addr_backup= modbus_sensors_telegram_values_address;
		modbus_sensors_log.raw.msg_num           = 0;
		modbus_sensors_log.raw.msg_num_backup    = 0;
		modbus_telegram_value.wr_addr            = modbus_sensors_telegram_values_address;
		modbus_telegram_value.rd_addr            = modbus_sensors_telegram_values_address;
		__programWord();
	}
}

uint8_t write_modbus_lock;
void modbus_sensors_log_write_lock( uint8_t _write_lock )
{
	write_modbus_lock = _write_lock;
}

/**
  * @brief Writes MODBUS raw data to FLASH memory
  * @retval 1
  */
uint32_t modbus_sensors_raw_frame_register(void)
{
	/* First write ever or Local Tool wants MODBUS sensor measure.
	 * Also, time should be initialized or in process of taking it from server. */
	if ((0 == write_modbus_lock) && (1 == Tick_cloud_time_init()))
	{
		write_modbus_lock = 1;
		modbus_sensors_log_raw_modbus();
//		modbus_sensors_log_params_store_raw_modbus();
#if defined(UNE82326)
		if ((0 == shutdown_initTelitModule()) && (0 == une82326_get_start_comm()))
		{
#elif defined (MBUS)
		if ((0 == shutdown_initTelitModule()) && (0 == mbus_get_start_comm()))
		{
#endif
#ifdef MODBUS
			if (MODBUS_SESSION_END == modbus_get_end_session())
			{
#endif
				shutdown_set( 1, rtc_system_getReadSensorAlarmCycleNextInSeconds() );
#ifdef MODBUS
			}
#endif
		}
	}
	return 1;
}

uint32_t modbus_sensors_log_params_register( void )
{
	/* First write ever or Local Tool wants modbus sensor measure.
	 * Also, time should be initialized or in process of taking it from server. */
	if ( ( 0 == write_modbus_lock )
	  && ( 1 == Tick_cloud_time_init() )
	  ) {
		write_modbus_lock = 1;
		aqualabo_modbus_set_get_data(0);
		modbus_sensors_log_Log();
		modbus_sensors_log_params_store_telegram_value();
#if defined(UNE82326)
		if ( ( 0 == shutdown_initTelitModule() ) && ( 0 == une82326_get_start_comm() ) ) {
#elif defined (MBUS)
		if ( ( 0 == shutdown_initTelitModule() ) && ( 0 == mbus_get_start_comm() ) ) {
#endif
#ifdef MODBUS
		if ( MODBUS_SESSION_END == modbus_get_end_session() ) {
#endif
			shutdown_set( 1, rtc_system_getReadSensorAlarmCycleNextInSeconds() );
#ifdef MODBUS
		}
#endif
		}
	}

	return 1;
}
