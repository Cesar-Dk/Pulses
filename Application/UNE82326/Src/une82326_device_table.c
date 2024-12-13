/*
 * une82326_device_table.c
 *
 *  Created on: 21 ago. 2019
 *      Author: smill
 */
#define _GNU_SOURCE
#include "une82326_device_table.h"
#include "une82326.h"
#include "une82326_protocol.h"
#include "params.h"

#ifdef UNE82326
une82326_device_table_st une82326_device_table;

void une82326_device_table_manager_init( void )
{
	uint8_t i;

	if ( 1 == leds_device_switch_on() ) {
		une82326_device_table.isfull = 0;
		for (i=0; i < NUM_MAX_DEVICES; i++) {
			une82326_device_table.device[i].primary         = 0;
			une82326_device_table.device[i].frame_type      = 0;
			une82326_device_table.device[i].frame_b_support = 0;
			snprintf(&une82326_device_table.device[i].serial_num[0], 9, "%s", "FFFFFFFF");
		}
		une82326_device_table.devices_num = 0;
		une82326_device_table.baudrate    = 0;
		une82326_device_table_manager_save_table();
		une82326_device_table_manager_read_table();
	}
}

une82326_device_table_st * une82326_device_table_manager_read_table( void )
{
	sFLASH_ReadBuffer(&une82326_device_table.isfull, 0, sizeof(une82326_device_table));
	if (une82326_device_table.isfull == 0xFF) {
		uint8_t i;
		une82326_device_table.isfull = 0;
		for (i=0; i < NUM_MAX_DEVICES; i++) {
			une82326_device_table.device[i].primary         = 0;
			une82326_device_table.device[i].frame_type      = 0;
			une82326_device_table.device[i].frame_b_support = 0;
			snprintf(&une82326_device_table.device[i].serial_num[0], 9, "%s", "FFFFFFFF");
		}
		une82326_device_table.devices_num = 0;
		une82326_device_table.baudrate    = 0;
		une82326_device_table_manager_save_table();
		une82326_device_table_manager_read_table();
	}
	return &une82326_device_table;
}

uint32_t une82326_device_table_manager_is_new_device( char * serial_num, uint8_t *device_num )
{
	uint32_t i;

	une82326_device_table_manager_read_table();

	for ( i = 0; i < une82326_device_table.devices_num; i++ ) {
		if ( 0 == memcmp( une82326_device_table.device[i].serial_num, serial_num, strlen(une82326_device_table.device[i].serial_num) ) ) {
			*device_num = i;
			return 0xFF;
		}
	}

	return i;
}

void une82326_device_table_manager_set_new_device_parameters( char * serial_num, char *crc )
{
	snprintf( &une82326_device_table.device[une82326_device_table.devices_num].serial_num[0], 18, "%s", serial_num );
	snprintf( &une82326_device_table.device[une82326_device_table.devices_num].crc[0], 5, "%s", crc );
	une82326_device_table.device[une82326_device_table.devices_num].primary           = une82326_device_table.devices_num;
	une82326_device_table.device[une82326_device_table.devices_num++].frame_b_support = une82326_b_frame_support();
}

uint8_t une82326_device_table_manager_check_frame_type( uint8_t device_num, char *crc )
{
	if ( une82326_device_table.device[device_num].crc[0] == '\0' ) {
		snprintf( &une82326_device_table.device[device_num].crc[0], 5, "%s", crc );
		une82326_device_table.device[device_num].frame_type = A_PLUS_FRAME;
	} else {
		if ( memcmp(&une82326_device_table.device[device_num].crc[0], crc,
				sizeof(une82326_device_table.device[device_num].crc) ) != 0 ) {
			une82326_device_table.device[device_num].frame_type = A_PLUS_FRAME; // New CRC, we send new A+ frame full size.
		} else {
			une82326_device_table.device[device_num].frame_type = WATER_METER_GATHERING_VALUES; // Same CRC, same fields, we only send caudal.
		}
	}

	return une82326_device_table.device[device_num].frame_type ;
}

char * une82326_device_table_manager_get_device_serial_num( uint8_t device_num )
{
	une82326_device_table_manager_read_table();
	if ( NULL == une82326_device_table.device[device_num].serial_num )
	{
		une82326_device_table.device[device_num].serial_num[0] = ' ';
	}
	return une82326_device_table.device[device_num].serial_num;
}

uint8_t une82326_device_table_get_num_of_devices( void )
{
	return une82326_device_table.devices_num;
}

void une82326_device_table_manager_save_table( void )
{
	sFLASH_EraseSector( 0x0000 );
	sFLASH_EraseSector( 0x1000 );
	sFLASH_WriteBuffer( &une82326_device_table.isfull, 0x0000, sizeof(une82326_device_table) );
}
#endif
