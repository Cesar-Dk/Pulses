/*
 * mbus_slave_table_manager.h
 *
 *  Created on: 30 dic. 2018
 *      Author: smillan
 */

#ifndef MBUS_SLAVE_TABLE_MANAGER_H_
#define MBUS_SLAVE_TABLE_MANAGER_H_

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "spi_flash.h"
#include "leds.h"
#include "mbus.h"
#include "mbus_protocol.h"
#include "mbus_protocol_aux.h"

typedef struct _slave_address_item {
	uint8_t  primary;	    /**< Primary address (for slaves shall be from 1 to 250) */
	uint8_t  baudrate;
	char     secondary[17]; /**< Secondary address (shall be 16 digits) */
}mbus_slave_adress_item;

typedef struct _slaves_address_table {
	uint8_t                isfull;
	uint8_t                slaves_num;
	uint8_t                baudrate;
	mbus_slave_adress_item slave[MBUS_MAX_PRIMARY_SLAVES];
}mbus_slaves_address_table;

uint8_t 					mbus_slave_table_get_slaves_num(void);
void 						mbus_slave_table_set_slaves_num(uint8_t __slaves_num);

char                      * mbus_slave_table_manager_get_slave_secondary_address( uint8_t slave_item );
void                        mbus_slave_table_manager_init( uint32_t init );
mbus_slaves_address_table * mbus_slave_table_manager_read_table( void );
void                        mbus_slave_table_manager_set_new_slave_secondary_address( char * secondary_address );
char                      * mbus_slave_table_manager_get_slave_secondary_address( uint8_t slave_num );
void                        mbus_slave_table_manager_save_table( void );

#endif /* MBUS_SLAVE_TABLE_MANAGER_H_ */
