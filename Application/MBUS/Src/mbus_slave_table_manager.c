/*
 * mbus_slave_table_manager.c
 *
 *  Created on: 30 dic. 2018
 *      Author: smillan
 */
#include "mbus_slave_table_manager.h"
#include "tick.h"
#include "params.h"

extern char _slaves_table_start_address[];
mbus_slaves_address_table mbus_slaves_table;

uint8_t mbus_slave_table_get_slaves_num(void)
{
	return mbus_slaves_table.slaves_num;
}

void mbus_slave_table_set_slaves_num(uint8_t __slaves_num)
{
	mbus_slaves_table.slaves_num= __slaves_num;
}

void mbus_slave_table_manager_init( uint32_t init )
{
	uint8_t i;

	mbus_slave_table_manager_read_table();
//	if ( 1 == leds_device_switch_on() )
	if ( 1 == init )
	{
		mbus_slaves_table.isfull = 0;
		for (i=0; i < MBUS_MAX_PRIMARY_SLAVES; i++) {
			mbus_slaves_table.slave[i].primary  = 0;
			mbus_slaves_table.slave[i].baudrate = MBUS_BAUDRATE_2400;
			snprintf(&mbus_slaves_table.slave[i].secondary[0], 17, "%s", "FFFFFFFFFFFFFFFF");
		}
		mbus_slaves_table.slaves_num = 0;
		mbus_slave_table_manager_save_table();
		mbus_slave_table_manager_read_table();
	}
//	else
//	{
//		LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS_SLAVE_TABLE> TABLE NOT INIT\r\n", (int)Tick_Get( SECONDS ));
//	}
}

mbus_slaves_address_table * mbus_slave_table_manager_read_table( void )
{
	sFLASH_ReadBuffer(&mbus_slaves_table.isfull, (uint32_t)_slaves_table_start_address, sizeof(mbus_slaves_table));
	return &mbus_slaves_table;
}

void mbus_slave_table_manager_set_new_slave_secondary_address( char * secondary_address )
{
	mbus_slaves_table.slave[mbus_slaves_table.slaves_num].baudrate = mbus_slaves_table.baudrate;
	snprintf(&mbus_slaves_table.slave[mbus_slaves_table.slaves_num++].secondary[0], 17, "%s", secondary_address);
	LOGLIVE(LEVEL_2, "LOGLIVE> %d MBUS_SLAVE_TABLE> NEW SLAVE!!!Slave:%s.Slave Num:%d\r\n", (int)Tick_Get( SECONDS ), secondary_address,(int)mbus_slaves_table.slaves_num);
}

char * mbus_slave_table_manager_get_slave_secondary_address( uint8_t slave_num )
{
	mbus_slave_table_manager_read_table();
	return mbus_slaves_table.slave[slave_num].secondary;
}

void mbus_slave_table_manager_save_table( void )
{
	sFLASH_EraseSector((uint32_t)_slaves_table_start_address);
	sFLASH_EraseSector((uint32_t)_slaves_table_start_address + 0x1000);
	sFLASH_WriteBuffer( &mbus_slaves_table.isfull, (uint32_t)_slaves_table_start_address, sizeof(mbus_slaves_table) );
}
