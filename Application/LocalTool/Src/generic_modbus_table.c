/*
 * generic_modbus_table.c
 *
 *  Created on: 6 feb. 2024
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
#include "tick.h"
#include "generic_modbus.h"

#include "generic_modbus_table.h"

#define GENERIC_MODBUS_TABLE_SIZE 			   GENERIC_MODBUS_TABLE_TOTAL_SIZE
#define GENERIC_MODBUS_SLAVE_SIZE  			   (0x2000)
#define GENERIC_MODBUS_SLAVE_TABLE_NUM_SECTORS (GENERIC_MODBUS_TABLE_SIZE/sFLASH_SPI_SECTOR_SIZE)

extern char _modbus_start_address[];
uint32_t 	num_slaves = 0;

uint32_t generic_modbus_table_get_num_devices( void )
{
	if ( 0 == num_slaves )
	{
		num_slaves = params_get_modbus_slave_num();
	}
	return num_slaves;
}

void generic_modbus_table_set_num_devices( uint32_t _num_devices )
{
	params_set_modbus_slave_num(_num_devices);
	num_slaves = _num_devices;
}

void generic_modbus_init_table(void)
{
	uint32_t i;

	generic_485_init();

	for (i = 0; i < GENERIC_MODBUS_SLAVE_TABLE_NUM_SECTORS; i++)
	{
		sFLASH_EraseSector((uint32_t)_modbus_start_address + 0x2000 + i * sFLASH_SPI_SECTOR_SIZE);
	}
	for (i = 0; i < GENERIC_MODBUS_SLAVE_TABLE_NUM_SECTORS; i++)
	{
		generic_modbus_table_manager_write_slave(i);
	}
	generic_modbus_table_manager_read_slave(0);
}

uint8_t generic_modbus_table_manager_read_slave( uint32_t _slaves )
{
	sFLASH_ReadBuffer((uint8_t *)generic_485_get_generic_485(),
			(uint32_t)_modbus_start_address + 0x2000 + _slaves * GENERIC_MODBUS_SLAVE_SIZE,
			generic_485_get_size_of_generic_485());

	return 0;
}

void generic_modbus_table_manager_write_slave( uint32_t _slaves )
{
	sFLASH_EraseSector((uint32_t)_modbus_start_address + 0x2000 + _slaves * GENERIC_MODBUS_SLAVE_SIZE);
	sFLASH_EraseSector((uint32_t)_modbus_start_address + 0x2000 + _slaves * GENERIC_MODBUS_SLAVE_SIZE + sFLASH_SPI_SECTOR_SIZE);

	sFLASH_WriteBuffer( (uint8_t *)generic_485_get_generic_485(),
			(uint32_t)_modbus_start_address + 0x2000 + _slaves * GENERIC_MODBUS_SLAVE_SIZE,
			generic_485_get_size_of_generic_485() );

	generic_modbus_table_manager_read_slave(_slaves);

	LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC MODBUS LOG> Slave_num:%d. Slave Id:%d. rd_ini:0x%X len:%d\r\n",
			(int)Tick_Get( SECONDS ),
			(int)_slaves,
			(int)generic_485_get_slave_id(),
			(int)(uint32_t)(_modbus_start_address + 0x2000 + _slaves * GENERIC_MODBUS_SLAVE_SIZE),
			(int)generic_485_get_size_of_generic_485());
}
