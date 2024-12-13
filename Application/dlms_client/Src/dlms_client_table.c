/*
 * dlms_client_table.c
 *
 *  Created on: 8 mar. 2022
 *      Author: smill
 */
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <ctype.h>
#include "common_lib.h"
#include "tick.h"
#include "generic_modbus.h"

#include "dlms_client_table.h"

#define DLMS_CLIENT_NUM_FLASH_SECTORS   (1)//(5)
#define DLMS_PROFILES_NUM_FLASH_SECTORS (14)//(5)
#define DLMS_CLIENT_MEMORY_SIZE         (((DLMS_CLIENT_NUM_FLASH_SECTORS) + (DLMS_PROFILES_NUM_FLASH_SECTORS)) * (sFLASH_SPI_SECTOR_SIZE))

uint32_t num_devices = 0;

uint32_t dlms_client_table_get_num_devices( void )
{
	if ( 0 == num_devices )
	{
		num_devices = dlms_client_get_num_devices();
	}
	return num_devices;
}

void dlms_client_table_set_num_devices( uint32_t _num_devices )
{
	dlms_client_set_num_devices(_num_devices);
	num_devices = _num_devices;
}

uint32_t dlms_client_table_flush_client_obis_profile(dlms_client_obis_profile_t_Ptr dlms_client_obis_profile)
{
	memset(dlms_client_obis_profile, '\0', dlms_client_obis_profile_get_size());
	return 0;
}

uint32_t dlms_client_table_flush_client_generic_profile(dlms_client_obis_profile_t_Ptr dlms_client_generic_profile)
{

	memset(dlms_client_generic_profile, '\0', dlms_client_generic_profile_get_size());
	return 0;
}

uint32_t dlms_client_read_client(uint32_t client_id, dlms_client_t_Ptr dlms_client)
{
	memset(dlms_client, '\0', dlms_client_get_size());
	sFLASH_ReadBuffer((uint8_t *)dlms_client, DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE, dlms_client_get_size());
	return 0;
}

uint32_t dlms_client_table_read_client_obis_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_obis_profile_t_Ptr dlms_client_obis_profile, uint32_t obis_profile_index)
{
	if (obis_profile_index < DLMS_OBIS_PROFILE_NUM)
	{
		memset(dlms_client_obis_profile, '\0', dlms_client_obis_profile_get_size());
		sFLASH_ReadBuffer((uint8_t *)dlms_client_obis_profile,
				DLMS_SLAVES_TABLE_INIT_ADDRESS + sFLASH_SPI_SECTOR_SIZE/*dlms_client_get_size()*/ + client_id * DLMS_CLIENT_MEMORY_SIZE + obis_profile_index * sFLASH_SPI_SECTOR_SIZE/*dlms_client_obis_profile_get_size()*/,
				dlms_client_obis_profile_get_size());
	}
	return 0;
}

uint32_t dlms_client_table_read_client_generic_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_generic_profile_t_Ptr dlms_client_generic_profile, uint32_t generic_profile_index)
{
	if ( (generic_profile_index - DLMS_OBIS_PROFILE_NUM) < DLMS_GENERIC_PROFILE_NUM)
	{
		memset(dlms_client_generic_profile, '\0', dlms_client_generic_profile_get_size());
		sFLASH_ReadBuffer((uint8_t *)dlms_client_generic_profile, DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + sFLASH_SPI_SECTOR_SIZE/*dlms_client_get_size()*/
				/*+ dlms_client_get_dlms_num_obis_profiles() * sFLASH_SPI_SECTOR_SIZE*//*dlms_client_obis_profile_get_size()*/ + generic_profile_index * sFLASH_SPI_SECTOR_SIZE/*dlms_client_generic_profile_get_size()*/,
				dlms_client_generic_profile_get_size());
	}
	return 0;
}

uint32_t dlms_client_table_init_sector(uint32_t client_id)
{
	uint8_t i;
	for (i = 0; i < (DLMS_CLIENT_NUM_FLASH_SECTORS + DLMS_PROFILES_NUM_FLASH_SECTORS); i++)
	{
		sFLASH_EraseSector( DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * sFLASH_SPI_SECTOR_SIZE + i * sFLASH_SPI_SECTOR_SIZE);
	}
	return 0;
}
uint32_t dlms_client_table_write_client(uint32_t client_id, dlms_client_t_Ptr dlms_client)
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_CLIENT_TABLE> CLIENT address: 0x%X. \r\n", (int)Tick_Get( SECONDS ), (int)(DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE));
	sFLASH_EraseSector(DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE);
	sFLASH_WriteBuffer( (uint8_t *)dlms_client, DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE, dlms_client_get_size() );
	return 0;
}

uint32_t dlms_client_table_write_obis_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_obis_profile_t_Ptr dlms_client_obis_profile, uint32_t obis_profile_index)
{
//	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_CLIENT_TABLE> GENERIC PROFILE address: 0x%X. OBIS Profile:%d \r\n", (int)Tick_Get( SECONDS ), (int)(DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + obis_profile_index * sFLASH_SPI_SECTOR_SIZE), (int)obis_profile_index);
	sFLASH_EraseSector(DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + sFLASH_SPI_SECTOR_SIZE/*dlms_client_get_size()*/
			+ obis_profile_index * sFLASH_SPI_SECTOR_SIZE/*dlms_client_obis_profile_get_size()*/);

	sFLASH_WriteBuffer( (uint8_t *)dlms_client_obis_profile,
			DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + sFLASH_SPI_SECTOR_SIZE/*dlms_client_get_size()*/
			+ obis_profile_index * sFLASH_SPI_SECTOR_SIZE/*dlms_client_obis_profile_get_size()*/,
			dlms_client_obis_profile_get_size() );

	return 0;
}

uint32_t dlms_client_table_write_generic_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_generic_profile_t_Ptr dlms_client_generic_profile, uint32_t generic_profile_index)
{
//	LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS_CLIENT_TABLE> GENERIC PROFILE address: 0x%X. GENERIC Profile:%d \r\n", (int)Tick_Get( SECONDS ), (int)(DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + generic_profile_index * sFLASH_SPI_SECTOR_SIZE),(int)generic_profile_index);
	sFLASH_EraseSector(DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + sFLASH_SPI_SECTOR_SIZE/*dlms_client_get_size()*/
			/*+ dlms_client_get_dlms_num_obis_profiles() * sFLASH_SPI_SECTOR_SIZE*//*dlms_client_obis_profile_get_size()*/ + generic_profile_index * sFLASH_SPI_SECTOR_SIZE/*dlms_client_generic_profile_get_size()*/);

	sFLASH_WriteBuffer( (uint8_t *)dlms_client_generic_profile,
			DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE + sFLASH_SPI_SECTOR_SIZE/*dlms_client_get_size()*/
			/*+ dlms_client_get_dlms_num_obis_profiles() * sFLASH_SPI_SECTOR_SIZE*//*dlms_client_obis_profile_get_size()*/ + generic_profile_index * sFLASH_SPI_SECTOR_SIZE/*dlms_client_generic_profile_get_size()*/,
			dlms_client_generic_profile_get_size() );

	return 0;
}

uint32_t dlms_client_table_read_client_id( dlms_client_t_Ptr dlms_client, uint32_t client_id )
{
	sFLASH_ReadBuffer((uint8_t *)dlms_client, DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE, dlms_client_get_size());
//	memcpy(dlms_client, &dlms_client, sizeof(dlms_client));

	return 0;
}

uint32_t dlms_client_table_write_client_id( dlms_client_t_Ptr dlms_client, uint32_t client_id )
{
	sFLASH_EraseSector( DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE );
	sFLASH_WriteBuffer( (uint8_t *)dlms_client, DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE, dlms_client_get_size() );
	sFLASH_ReadBuffer((uint8_t *)dlms_client, DLMS_SLAVES_TABLE_INIT_ADDRESS + client_id * DLMS_CLIENT_MEMORY_SIZE, dlms_client_get_size());

	return 0;
}

uint32_t dlms_client_table_check_meter_id( dlms_client_t_Ptr dlms_client, uint8_t *str )
{
	uint32_t i;

	for ( i = 0; i <= dlms_client_table_get_num_devices() ; i++ )
	{
		if (strstr(dlms_client_get_meter_id(i), (char *)str))
		{
			if ( i < con_dlms_get_curr_device() )
			{
				con_dlms_set_curr_device(i);
				break;
			}
		}
	}
	return i;
}

uint32_t num_meters_from_cmd = 0;
uint32_t curr_meter_from_cmd = 0;
uint32_t dlms_client_table_get_curr_meter_from_cmd(void)
{
	return curr_meter_from_cmd;
}

void dlms_client_table_inc_curr_meter_from_cmd(void)
{
	curr_meter_from_cmd += 1;
}

uint32_t dlms_client_table_get_num_meters_from_cmd(void)
{
	return num_meters_from_cmd;
}

void dlms_client_table_inc_num_meters_from_cmd(void)
{
	num_meters_from_cmd += 1;
}

void dlms_client_table_set_num_meters_from_cmd(uint32_t __num_meters)
{
	num_meters_from_cmd = __num_meters;
}

uint32_t dlms_client_table_reset_devices_from_cmd(void)
{
	num_meters_from_cmd = 0;
	curr_meter_from_cmd = 0;
	memset(con_dlms_get_device_from_cmd(0),0, 30*32*sizeof(char));

	return 0;
}

uint32_t dlms_client_table_add_meter_id_from_cmd( dlms_client_t_Ptr dlms_client, uint8_t *str )
{
	uint32_t i, j;

	for ( i = 0; i < dlms_client_table_get_num_devices(); i++ )
	{
		if ( strstr(dlms_client_get_meter_id(i), (char *)str) )
		{
			for ( j = 0; j < 30; j++ )
			{
				if (*con_dlms_get_device_from_cmd(j) == 0)
				{
					dlms_client_set_dlms_reading_items(i, 0);
					num_meters_from_cmd++;
					memcpy(con_dlms_get_device_from_cmd(j), (char *)str, 32 * sizeof(char) );
					break;
				}
			}
		}
	}
	return i;
}

uint32_t dlms_client_table_inc_reading_items(void)
{
	uint32_t i = 0, j;

//	for ( i = 0; i < dlms_client_table_get_num_devices(); i++ )
//	{
//		if (*dlms_client_get_meter_id(i) != '\0')
//		{
//			dlms_client_inc_dlms_reading_items(i);
//		}
//	}
//	return i;
	for (j = 0; j < dlms_client_table_get_num_devices(); j++)
	{
		if (*dlms_client_get_meter_id(j) != '\0')
		{
			for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
			{
				if ( strstr(dlms_client_get_meter_id(j), con_dlms_get_device_from_cmd(i)) )
				{
					dlms_client_inc_dlms_reading_items(j);
					break;
				}
			}
		}
	}
	return i;
}

uint32_t dlms_client_table_inc_reading_items_index_meter(uint32_t meter_index)
{
	uint32_t i, index = 0;

	for ( i = 0; i < dlms_client_table_get_num_devices(); i++ )
	{
//		if (*dlms_client_get_meter_id(i) != '\0')
		if (*con_dlms_get_device_from_cmd(i) != 0)
		{
			if ( index == meter_index )
			{
				dlms_client_inc_dlms_reading_items(i);
			}
			index = index + 1;
		}
	}
	return i;
}
