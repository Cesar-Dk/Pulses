/*
 * une82326_device_table.h
 *
 *  Created on: 21 ago. 2019
 *      Author: smill
 */
#ifndef APPLICATION_UNE82326_INC_UNE82326_DEVICE_TABLE_H_
#define APPLICATION_UNE82326_INC_UNE82326_DEVICE_TABLE_H_

#include "stm32u5xx_hal.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "spi_flash.h"
#include "leds.h"
#include "une82326.h"
#include "une82326_protocol.h"

#ifdef UNE82326
#define NUM_MAX_DEVICES (50)

typedef struct _slave_address_item {
	uint8_t  primary;	       /**< Primary address (for slaves shall be from 1 to 50) */
	char     serial_num[ 18 ]; /**< Serial number (shall be 8 digits) */
	char     crc[ 5 ];
	uint8_t  frame_type;
	uint8_t  frame_b_support;
} une82326_device_table_item;

typedef struct _slaves_address_table {
	uint8_t                	   isfull;
	uint8_t                	   devices_num;
	uint8_t                	   baudrate;
	une82326_device_table_item device[ NUM_MAX_DEVICES ];
} une82326_device_table_st;

void                       une82326_device_table_manager_init( void );
une82326_device_table_st * une82326_device_table_manager_read_table( void );
uint32_t 				   une82326_device_table_manager_is_new_device( char * serial_num, uint8_t *device_num );
void                       une82326_device_table_manager_set_new_device_parameters( char * serial_num, char *crc );
uint8_t 				   une82326_device_table_manager_check_frame_type( uint8_t device_num, char *crc );
char                     * une82326_device_table_manager_get_device_serial_num( uint8_t device_num );
uint8_t                    une82326_device_table_get_num_of_devices( void );
void                       une82326_device_table_manager_save_table( void );

#endif
#endif
