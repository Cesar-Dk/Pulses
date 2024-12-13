/**
  ******************************************************************************
  * @file           generic_modbus.c
  * @author 		Datakorum Development Team
  * @brief          Driver to handle generic modbus transmission and reception data.
  *
  * @note	Data structures are ready for more than one Modbus slave, but the code is
  *  hardcoded to use only one SLAVE.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * generic_modbus.c
 *
 *  Created on: 9 mar. 2020
 *      Author: Sergio Millán López
 */
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "common_lib.h"

#include "message_queue.h"
#include "modbus.h"
#include "modbus_sensors.h"
#include "generic_modbus.h"
#include "generic_modbus_table.h"
#include "mbcrc.h"
#include "serial_une82326.h"
#include "leds.h"
#include "tick.h"

/**
 * @addtogroup System_Modbus
 * @{
 *
 */

/**
 * @defgroup System_Modbus_Generic Generic Modbus
 * @brief
 *
 * @{
 *
 */
#define READ_CMDS  (0)
#define WRITE_CMDS (1)
generic485_st generic_485[2];

/**
 * @struct
 * @brief
 *
 */
static struct
{
    uint8_t slave;
    uint8_t command;
} next, response;

static uint8_t  r;

/**
 * @fn void generic_485_init(void)
 * @brief Initialize generic RS485 variables
 *
 */
void generic_485_init(void)
{
    memset(generic_485, 0, sizeof(generic_485));
    next.slave     = next.command     = 0;
    response.slave = response.command = 0;
    r = 0;

    if ( 0 == generic_485[READ_CMDS].modbus.send_time )
    {
//    	generic_485[0].modbus.send_time = 900;
//    	generic_485[0].modbus.send_time = params_config_get_generic_modbus_send_time();
    }
    if ( 0 == generic_485[READ_CMDS].modbus.read_time )
    {
//    	generic_485[0].modbus.read_time = 900;
//    	generic_485[0].modbus.read_time = params_config_get_generic_modbus_read_time();
    }
}

generic485_st *generic_485_get_generic_485(void)
{
	return generic_485;
}

size_t generic_485_get_size_of_generic_485(void)
{
	return sizeof(generic_485[READ_CMDS]);
}

/**
 * @fn uint8_t generic_485_get_enable(void)
 * @brief Checks whether the modbus slave is enabled or not.
 *
 * @return generic_485[0].modbus.enable
 * 			@arg 0 - Modbus slave is disabled.
 * 			@arg 1 - Modbus slabe is enabled.
 */
uint8_t generic_485_get_enable(void)
{
//	generic_modbus_table_manager_read_slave(0);
	return generic_485[READ_CMDS].modbus.enable;
}

/**
 * @fn void generic_485_set_enable(uint8_t)
 * @brief Enables/Disables the Modbus slave.
 *
 * @param _enable generic_485[0].modbus.enable
 * 			@arg 0 - Modbus slave is disabled.
 * 			@arg 1 - Modbus slabe is enabled.
 *
 */
void generic_485_set_enable(uint8_t _enable)
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].modbus.enable = _enable;
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn void generic_485_set_tx_on(uint8_t)
 * @brief
 *
 * @param _tx_on
 */
void generic_485_set_tx_on(uint8_t _tx_on)
{
//	uint32_t i = 0;

	generic_485[0].modbus.tx_on = _tx_on;
#if 0
	for (i = 0; i < generic_modbus_table_get_num_devices(); i++)
	{
//		generic_485[i].modbus.tx_on = _tx_on;
		generic_modbus_table_manager_write_slave(i);
	}
#endif
}

/**
 * @fn uint8_t generic_485_get_comm_state(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t generic_485_get_comm_state(void)
{
	generic_modbus_table_manager_read_slave(0);
	return generic_485[READ_CMDS].modbus.comm_state;
}

/**
 * @fn void generic_485_set_comm_state(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _comm_state
 */
void generic_485_set_comm_state(uint8_t _comm_state)
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].modbus.comm_state = _comm_state;
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn void generic_485_set_device(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _device
 */
void generic_485_set_device(uint32_t _device)
{
	generic_485[READ_CMDS].device = _device;
//	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn void generic_485_set_slave_id(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _slave_id, reg
 */
void generic_485_set_slave_id_reg( uint8_t _slave_id, uint32_t reg )
{
	generic_485[READ_CMDS].modbus.slave = _slave_id;
//	generic_modbus_table_manager_write_slave(_slave_id);
	if (0 == reg)
	{
		params_set_modbus_slave_id(_slave_id);
	}
}

/**
 * @fn void generic_485_set_address_645( uint8_t _bcd, uint32_t pos)
 * @brief
 *
 * @pre
 * @post
 * @param _slave_id, reg
 */
void generic_485_set_address_645( uint8_t _bcd, uint32_t pos, uint8_t slave )
{
	generic_modbus_table_manager_read_slave(slave);
	generic_485[READ_CMDS].modbus.address_645[pos] = _bcd - 0x30;
	generic_modbus_table_manager_write_slave(slave);

}

/**
 * @fn void generic_485_set_data_id_645(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _slave_id, reg
 */
void generic_485_set_data_id_645(uint32_t slave, uint32_t _data_id, uint32_t reg)
{
	generic_485[READ_CMDS].modbus.command[ reg ].data_id_645 = _data_id;
//	generic_modbus_table_manager_write_slave(slave);
}

void generic_485_set_function_reg( uint32_t slave, uint8_t _function, uint32_t reg )
{
	generic_485[READ_CMDS].modbus.command[reg].function = _function;
//	generic_modbus_table_manager_write_slave(slave);
	if (0 == reg)
	{
		params_set_modbus_function(_function);
	}
}

uint8_t generic_485_get_function_slave_reg( uint32_t slave, uint32_t reg )
{
	generic_modbus_table_manager_read_slave(slave);
	return generic_485[READ_CMDS].modbus.command[reg].function;
}

void generic_485_set_addr_reg( uint32_t slave, uint16_t _addr, uint32_t reg )
{
	generic_485[READ_CMDS].modbus.command[reg].addr = _addr;
//	generic_modbus_table_manager_write_slave(slave);
	if (0 == reg)
	{
		params_set_modbus_address(_addr);
	}
}

uint16_t generic_485_get_address_slave_reg( uint32_t slave, uint32_t reg )
{
	generic_modbus_table_manager_read_slave(slave);
	return generic_485[READ_CMDS].modbus.command[reg].addr;
}

void generic_485_set_quantity_reg( uint32_t slave, uint16_t _quantity, uint32_t reg )
{
	generic_485[READ_CMDS].modbus.command[reg].quantity = _quantity;
//	generic_modbus_table_manager_write_slave(slave);
	if (0 == reg)
	{
		params_set_modbus_quantity(_quantity);
	}
}

void generic_485_set_value_reg( uint32_t slave, uint16_t _value, uint32_t reg, uint32_t value_num )
{
	generic_485[READ_CMDS].modbus.command[reg].values[value_num] = _value;
}

uint8_t generic_485_get_quantity_slave_reg( uint32_t slave, uint32_t reg )
{
//	generic_modbus_table_manager_read_slave(slave);
	return generic_485[READ_CMDS].modbus.command[reg].quantity;
}

/**
 * @fn uint8_t generic_485_get_slave_id(void)
 * @brief
 *
 * @pre
 * @post
 * @param
 */
uint8_t generic_485_get_slave_id(void)
{
//	generic_485[READ_CMDS].modbus.slave = params_get_modbus_slave_id();
	return generic_485[READ_CMDS].modbus.slave;
}

/**
 * @fn uint8_t generic_485_get_function(void)
 * @brief
 *
 * @pre
 * @post
 * @param
 */
uint8_t generic_485_get_function(void)
{
	generic_485[READ_CMDS].modbus.command[0].function = params_get_modbus_function();
	return generic_485[READ_CMDS].modbus.command[0].function;
}

/**
 * @fn uint16_t generic_485_get_addr(void)
 * @brief
 *
 * @pre
 * @post
 * @param
 */
uint16_t generic_485_get_addr(void)
{
	generic_485[READ_CMDS].modbus.command[0].addr = params_get_modbus_address();
	return generic_485[READ_CMDS].modbus.command[0].addr;
}

/**
 * @fn uint16_t generic_485_get_quantity(void)
 * @brief
 *
 * @pre
 * @post
 * @param _quantity
 */
uint16_t generic_485_get_quantity(void)
{
	generic_485[READ_CMDS].modbus.command[0].quantity = params_get_modbus_quantity();
	return generic_485[READ_CMDS].modbus.command[0].quantity;
}

/**
 * @fn uint32_t generic_485_get_send_time(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t generic_485_get_send_time( void )
{
//	generic_485[0].modbus.send_time = 3600;//DEBUG:
	generic_485[READ_CMDS].modbus.send_time = params_get_modbus_send_time();
	return generic_485[READ_CMDS].modbus.send_time;
}

/**
 * @fn void generic_485_set_send_time(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _send_time
 */
void generic_485_set_send_time( uint32_t _send_time )
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].modbus.send_time = _send_time;
	params_set_modbus_send_time(generic_485[READ_CMDS].modbus.send_time);
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn uint32_t generic_485_get_read_time(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t generic_485_get_read_time( void )
{
//	generic_485[0].modbus.read_time = 900;//DEBUG:
	generic_485[READ_CMDS].modbus.read_time = params_get_modbus_read_time();
	return generic_485[READ_CMDS].modbus.read_time;
}

/**
 * @fn void generic_485_set_read_time(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _read_time
 */
void generic_485_set_read_time( uint32_t _read_time )
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].modbus.read_time = _read_time;
	params_set_modbus_read_time(generic_485[READ_CMDS].modbus.read_time);
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn uint32_t generic_485_get_type(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t generic_485_get_type( void )
{
//	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].type = params_get_modbus_type();
	return generic_485[READ_CMDS].type;
}

/**
 * @fn uint32_t generic_485_get_dtl645(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t generic_485_get_dtl645( void )
{
//	generic_485[0].type = params_get_modbus_type();
//	generic_modbus_table_manager_read_slave(0);
	return generic_485[READ_CMDS].dtl645_on;
}

/**
 * @fn void generic_485_set_type(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _type
 */
void generic_485_set_type( uint32_t _type )
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].type = _type;
	params_set_modbus_type(_type);
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn void generic_485_set_type(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _type
 */
void generic_485_set_type_and_disable( uint32_t _type )
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].modbus.enable = 0;
	generic_485[READ_CMDS].type = _type;
	params_set_modbus_type(_type);
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn void generic_485_set_dtl465(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _type
 */
void generic_485_set_dtl645( uint8_t _on )
{
	generic_modbus_table_manager_read_slave(0);
	generic_485[READ_CMDS].dtl645_on = _on;
//	params_set_modbus_type(_type);
	generic_modbus_table_manager_write_slave(0);
}

/**
 * @fn void generic_485_set_slave_id_type(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _type
 */
void generic_485_set_slave_id_type( uint8_t slave_id, uint32_t _type )
{
	generic_485[READ_CMDS].type = _type;
//	generic_modbus_table_manager_write_slave(slave_id);
}

/**
 * @fn void __disableGeneric485Sensor(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __disableGeneric485Sensor( uint32_t _slave )
{
	leds_LED_Off(LED_GREEN);

	generic_modbus_table_manager_read_slave(_slave);
//	modbus_sensors_set_get_data(0);
	generic_485[READ_CMDS].modbus.comm_state = END_COMMUNICATION;
	generic_485[READ_CMDS].modbus.enable     = DISABLED;

	generic_modbus_table_manager_write_slave(_slave);
}

uint8_t generic_485_get_is_on_demand(uint8_t __command)
{
	return generic_485[READ_CMDS].modbus.command[__command].response.on_demand;
}

uint8_t generic_485_set_is_on_demand(uint8_t __command, uint8_t __on_demand)
{
	generic_485[READ_CMDS].modbus.command[__command].response.on_demand = __on_demand;

	return 0;
}

/**
 * @fn uint8_t generic_485_available(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t generic_485_available( void )
{
    uint8_t i, j;

//    generic_485[ 0 ].type                         = MODBUS_RAW;// DEBUG: Modbus read.
//    generic_485[ 0 ].modbus.slave                 = 1;         // DEBUG: Modbus read.
//    generic_485[ 0 ].modbus.command[ 0 ].function = 3;         // DEBUG: Modbus read.
//    generic_485[ 0 ].modbus.command[ 0 ].addr     = 4000;      // DEBUG: Modbus read.
//    generic_485[ 0 ].modbus.command[ 0 ].quantity = 18*2;      // DEBUG: Modbus read.

    if (ENABLED == generic_485_get_enable())
    {
    	if (r)
    	{
    		next.command++;

    		if (next.command == MAX_COMMANDS)
    		{
    			next.command = 0;

    			if( next.slave < (generic_modbus_table_get_num_devices() - 1) )
    			{
    				__disableGeneric485Sensor(next.slave);
    				next.slave++;
    			}
    			else
    			{
    				__disableGeneric485Sensor(next.slave);
    				next.slave = 0;
    				modbus_sensors_set_get_data(0);
    			}
    		}
    	}

    	r = 0;

    	generic_modbus_table_manager_read_slave(next.slave);
    	for (i = 0; i < params_get_modbus_slave_num(); i++)
    	{
    		if ( generic_485[READ_CMDS].type == MODBUS_RAW )
    		{
    			for ( j = 0; j < MAX_COMMANDS; j++ )
    			{
    				generic_modbus_table_manager_read_slave(next.slave);
    				if ( ( generic_485[READ_CMDS].modbus.command[ next.command ].function  != 0 )
    				  && ( generic_485[READ_CMDS].modbus.command[ next.command ].function  != 0xFF )
					   )
    				{
    					generic_modbus_table_manager_read_slave(next.slave);
    					generic_485[READ_CMDS].modbus.enable = 1;
    					r = 1;
    					generic_modbus_table_manager_write_slave(next.slave);
    					break;
    				}
    				else
    				{
    					next.command = 0;
    					break;
    				}

    				if ( next.command < MAX_COMMANDS - 1 )
    				{
    					next.command++;
    				}
    				else
    				{
    					next.command = 0;
    					j            = MAX_COMMANDS;
    				}
    			}
    		}
    		else if ( generic_485[READ_CMDS].type == MODBUS_DECODED )
    		{

    		}

    		if ( r )
    		{
    			break;
    		}
    		else
    		{
    			if( next.slave < (params_get_modbus_slave_num() - 1) )
    			{
    				__disableGeneric485Sensor(next.slave);
    				next.slave++;
    			}
    			else
    			{
    				__disableGeneric485Sensor(next.slave);
    				next.slave = next.command = 0;
    				modbus_sensors_set_get_data(0);
    				break;
    			}
    		}
    	}
    }

    return r;
}

/*
 * DLT645 PROTOCOL.
 */
#define START_FRAME_BYTE         	(0x68)
#define WAKE_UP_BYTE             	(0xFE)
#define DLT645_MSG_TX_START_BODY 	(14)
#define MAX_DLT645_MSG_BODY      	(200)

/**
 * @fn void __dlt_645_send(void)
 * @brief
 *
 * @pre
 * @post
 * @param
 */
static uint32_t __dtl_645_send(unsigned char *data)
{
    uint32_t i = 0, len = 4;
//    uint8_t val;
    generic_modbus_table_manager_read_slave(next.slave);
    LOGLIVE(LEVEL_1, "LOGLIVE> %d DLT-645> TX Add:%d%d%d%d%d%d%d\r\n", (int)Tick_Get( SECONDS ),
    		generic_485[READ_CMDS].modbus.address_645[0],
			generic_485[READ_CMDS].modbus.address_645[1],
			generic_485[READ_CMDS].modbus.address_645[2],
			generic_485[READ_CMDS].modbus.address_645[3],
			generic_485[READ_CMDS].modbus.address_645[4],
			generic_485[READ_CMDS].modbus.address_645[5],
			generic_485[READ_CMDS].modbus.address_645[6]
    );

    data[0] = WAKE_UP_BYTE;
    data[1] = WAKE_UP_BYTE;
    data[2] = WAKE_UP_BYTE;
    data[3] = WAKE_UP_BYTE;
    data[4] = START_FRAME_BYTE;
    data[5] = (generic_485[READ_CMDS].modbus.address_645[0]<<4) + generic_485[READ_CMDS].modbus.address_645[1];
    data[6] = (generic_485[READ_CMDS].modbus.address_645[2]<<4) + generic_485[READ_CMDS].modbus.address_645[3];
    data[7] = (generic_485[READ_CMDS].modbus.address_645[4]<<4) + generic_485[READ_CMDS].modbus.address_645[5];
    data[8] = (generic_485[READ_CMDS].modbus.address_645[6]<<4) + generic_485[READ_CMDS].modbus.address_645[7];
    data[9] = (generic_485[READ_CMDS].modbus.address_645[8]<<4) + generic_485[READ_CMDS].modbus.address_645[9];
    data[10] = (generic_485[READ_CMDS].modbus.address_645[10]<<4) + generic_485[READ_CMDS].modbus.address_645[11];
    data[11] = START_FRAME_BYTE;
    data[12] = generic_485[READ_CMDS].modbus.command[ next.command ].function;
    data[13] = len;
    data[DLT645_MSG_TX_START_BODY + len]     = 0;
    data[DLT645_MSG_TX_START_BODY + len + 1] = 0x16;
    i = DLT645_MSG_TX_START_BODY;
	data[ i++ ] = ((generic_485[READ_CMDS].modbus.command[ next.command ].data_id_645>>24)&0xFF) + 0x33;
	data[ i++ ] = ((generic_485[READ_CMDS].modbus.command[ next.command ].data_id_645>>16)&0xFF) + 0x33;
	data[ i++ ] = ((generic_485[READ_CMDS].modbus.command[ next.command ].data_id_645>>8)&0xFF) + 0x33;
	data[ i++ ] = ((generic_485[READ_CMDS].modbus.command[ next.command ].data_id_645>>0)&0xFF) + 0x33;
    for (i = 4;  i < DLT645_MSG_TX_START_BODY + len;  i++)
    {
        data[DLT645_MSG_TX_START_BODY + len] += data[i];
    }
    len = DLT645_MSG_TX_START_BODY + len + 2;
    if (len > 4 + 12 + MAX_DLT645_MSG_BODY)
    {
    	return 0;
    }
    else
    {
    	return len;
    }

}
/**
 * @fn uint32_t generic_485_send(unsigned char*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
#define  CHECK_TAMPER	(HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ))
uint32_t generic_485_send(unsigned char *data)
{
    union
    {
        uint16_t v16;
        uint8_t  v8[2];
    } value;

    uint32_t i    = 0;
    uint8_t  n, j = 0;
    uint16_t crc;

    generic_modbus_table_manager_read_slave(next.slave);

    if (generic_485[READ_CMDS].modbus.enable ==  ENABLED)
    {
    	if ((generic_485_get_dtl645() == 1) && (generic_485[READ_CMDS].type == MODBUS_RAW) && (0 == generic_485[READ_CMDS].modbus.tx_on))
    	{
    		if ( 0 == CHECK_TAMPER )
    		{
    			leds_LED_On(LED_GREEN);
    		}
    		generic_485[READ_CMDS].modbus.tx_on = 1;
    		i = __dtl_645_send(data);
    		generic_modbus_table_manager_write_slave(next.slave);
    		return i;
    	}
    	else if ((generic_485[READ_CMDS].type == MODBUS_RAW) && (0 == generic_485[READ_CMDS].modbus.tx_on))
    	{
    		if ( 0 == CHECK_TAMPER )
    		{
    			leds_LED_On(LED_GREEN);
    		}
    		generic_485[READ_CMDS].modbus.tx_on = 1;
    		data[ i++ ] = generic_485[READ_CMDS].modbus.slave;
    		data[ i++ ] = generic_485[READ_CMDS].modbus.command[ next.command ].function;
    		value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].addr;
    		data[ i++ ] = value.v8[ 1 ];
    		data[ i++ ] = value.v8[ 0 ];

    		switch(generic_485[READ_CMDS].modbus.command[ next.command ].function)
    		{
				case 3:
				/*FLUSHTHRU*/
				case 4:
					value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].quantity;
					data[ i++ ] = value.v8[ 1 ];
					data[ i++ ] = value.v8[ 0 ];
				break;

				case 5:
				/*FLUSHTHRU*/
				case 6:
					value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].values[ 0 ];
					data[ i++ ] = value.v8[ 1 ];
					data[ i++ ] = value.v8[ 0 ];
				break;

				case 15:
					value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].quantity;
					data[ i++ ] = value.v8[ 1 ];
					data[ i++ ] = value.v8[ 0 ];
					if( generic_485[READ_CMDS].modbus.command[ next.command ].quantity % 8 )
					{
						n = ( generic_485[READ_CMDS].modbus.command[ next.command ].quantity >> 3 ) + 1;
					}
					else
					{
						n = generic_485[READ_CMDS].modbus.command[ next.command ].quantity >> 3;
					}
					if( n > MAX_VALUES * 2 )
					{
						n = MAX_VALUES * 2;
					}
					data[ i++ ] = n; // # bytes
					while( n )
					{
						if( n > 1 )
						{
							value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].values[ j ];
							data[ i++ ] = value.v8[ 1 ];
							data[ i++ ] = value.v8[ 0 ];
							n -= 2;
						}
						else
						{
							value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].values[ j ];
							data[ i++ ] = value.v8[ 0 ];
							n--;
						}
						j++;
					}
					break;
				case 16:
					value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].quantity;
					data[ i++ ] = value.v8[ 1 ];
					data[ i++ ] = value.v8[ 0 ];
					if ( generic_485[READ_CMDS].modbus.command[ next.command ].quantity > MAX_VALUES )
					{
						generic_485[READ_CMDS].modbus.command[ next.command ].quantity = MAX_VALUES;
						generic_modbus_table_manager_write_slave(next.slave);
					}
					data[ i++ ] = generic_485[READ_CMDS].modbus.command[ next.command ].quantity * 2;
					for ( j = 0; j < generic_485[READ_CMDS].modbus.command[ next.command ].quantity; j++ )
					{
						value.v16   = generic_485[READ_CMDS].modbus.command[ next.command ].values[ j ];
						data[ i++ ] = value.v8[ 1 ];
						data[ i++ ] = value.v8[ 0 ];
					}
					break;
				default: // function NOT supported
					i = 0;
					generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = 0x01;
					generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
					generic_modbus_table_manager_write_slave(next.slave);
				break;
    		}
    		if ( i )
    		{
    			crc         = usMBCRC16( data, i );
    			data[ i++ ] = crc;
    			data[ i++ ] = crc >> 8;
    		}
    	}
    	else if ( generic_485[READ_CMDS].type == MODBUS_DECODED )
    	{

    	}
    }

    return i;
}

static void __errorFrame( uint8_t error_code )
{
	generic_modbus_table_manager_read_slave(next.slave);

	generic_485[READ_CMDS].modbus.attempt++;

	if ( generic_485[READ_CMDS].modbus.attempt == 4 )//2
	{//10
		generic_485[READ_CMDS].modbus.tx_on = 0;
		generic_modbus_table_manager_write_slave(next.slave);
	}
	else if ( generic_485[READ_CMDS].modbus.attempt == 8 )//4
	{//20
//		if ( ! generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code )
		{
			generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = error_code;
//			generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC MODBUS> Error Frame Slave:%d Command:%d\r\n", (int)Tick_Get( SECONDS ),next.slave, next.command);
		generic_485[READ_CMDS].modbus.attempt = 0;
//		next.command = 0;
		static uint32_t max_retries = 0;
		if ( 1 == generic_modbus_table_get_num_devices() )
		{
//			next.command++;
//			generic_modbus_table_manager_write_slave(next.slave);
//			if ( max_retries++ > 3 )
//			{
//				__disableGeneric485Sensor(next.slave);
//				max_retries = next.slave = next.command = 0;
//				modbus_sensors_set_get_data(0);
//				modbus_sensors_set_comm_state(MODBUS_COMM_END);
//			}
        	if ( max_retries++ >= 2 )
        	{
        		generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
        		next.command++;
        		generic_modbus_table_manager_write_slave(next.slave);
        		max_retries = 0;
        	}
		}
		else if ( next.slave < (generic_modbus_table_get_num_devices() - 1) )
		{
			generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
			generic_modbus_table_manager_write_slave(next.slave);
//			next.slave++;
			next.command++;
		}
		else
		{
			generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
			generic_modbus_table_manager_write_slave(next.slave);
			__disableGeneric485Sensor(next.slave);
			next.slave = next.command = 0;
			modbus_sensors_set_get_data(0);
			modbus_sensors_set_comm_state(MODBUS_COMM_END);
		}
	}
	else
	{
		generic_modbus_table_manager_write_slave(next.slave);
	}
	r = 0;
}

static void __writeSingleCoilSingleRegister( uint8_t *ch )
{
	generic_modbus_table_manager_read_slave(next.slave);
	if ( mb_swaps( ch ) == generic_485[READ_CMDS].modbus.command[ next.command ].addr )
	{
		ch += 2;
		if ( mb_swaps( ch ) == generic_485[READ_CMDS].modbus.command[ next.command ].values[ 0 ] )
		{
			generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = 0x80;
			generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
			generic_modbus_table_manager_write_slave(next.slave);
		}
	}
}

static void __writeMultipleCoilsMultipleRegisters( uint8_t *ch )
{
	uint8_t i = 0;

	generic_modbus_table_manager_read_slave(next.slave);

	if ( mb_swaps( ch ) == generic_485[READ_CMDS].modbus.command[ next.command ].addr ) {
        ch += 2;
        if ( mb_swaps( ch ) == generic_485[READ_CMDS].modbus.command[ next.command ].quantity )
        {
            generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = 0;
            generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
            for ( i = 0; i < generic_485[0].modbus.command[ next.command ].quantity; i++ )
            {
            	generic_485[READ_CMDS].modbus.command[ next.command ].response.values[i]  = generic_485[READ_CMDS].modbus.command[ next.command ].addr;
            }
//            generic_485[READ_CMDS].modbus.command[ next.command ].quantity            = 1;
            generic_modbus_table_manager_write_slave(next.slave);
        }
    }
}

static void __readHoldingRegistersInputRegisters( uint8_t *ch )
{
	uint8_t *ch2, i = 0;

	generic_modbus_table_manager_read_slave(next.slave);

	if ( *ch++ == ( generic_485[READ_CMDS].modbus.command[ next.command ].quantity << 1 ) )
	{
		ch2 = ch;
		generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = 0;
		generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
		for ( i = 0; i < generic_485[0].modbus.command[ next.command ].quantity; i++ )
		{
			generic_485[READ_CMDS].modbus.command[ next.command ].response.values[ i ] = mb_swaps( ch2 );
			ch2 += 2;
		}
		generic_modbus_table_manager_write_slave(next.slave);
		LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC MODBUS> NEW Frame Slave:%d Command:%d\r\n", (int)Tick_Get( SECONDS ),next.slave, next.command);
	}
}

uint32_t generic_485_set_alarm_message( uint8_t *data, uint32_t n )
{
	uint8_t  ch[20];
	uint32_t ret = 0;

	if ( n > 4 ) {
		memcpy( ch, data, 20 * sizeof( uint8_t ));
		if ( ch[0] == generic_485[READ_CMDS].modbus.slave ) {
			if ( ch[1] == 0x03 ) {
				generic_485[READ_CMDS].modbus.command[ next.command ].function = 0x03;
				ret = 1;
			}
		}
	}
	return ret;
}

static uint8_t _crc_check( uint8_t *data, uint32_t size)
{
	uint8_t  CRCHi = 0x00;

	while( size-- )
	{
		CRCHi += *data++;
	}
	return CRCHi;
}
/**
 * @fn void __dlt_645_send(void)
 * @brief
 *
 * @pre
 * @post
 * @param
 */
static uint32_t __dtl_645_recv(uint8_t *data, uint32_t n)
{
	uint8_t crc= 0, i = 0;

	generic_modbus_table_manager_read_slave(next.slave);
	if ( n > 14 )
	{
    	generic_485[READ_CMDS].modbus.tx_on = 0;
    	memcpy( &crc, &data[ n - 2 ], sizeof( crc ));

    	if (
    	   (data[4]  == START_FRAME_BYTE)
    	&& (data[5]  == (generic_485[READ_CMDS].modbus.address_645[0]<<4)  + generic_485[READ_CMDS].modbus.address_645[1])
        && (data[6]  == (generic_485[READ_CMDS].modbus.address_645[2]<<4)  + generic_485[READ_CMDS].modbus.address_645[3])
        && (data[7]  == (generic_485[READ_CMDS].modbus.address_645[4]<<4)  + generic_485[READ_CMDS].modbus.address_645[5])
        && (data[8]  == (generic_485[READ_CMDS].modbus.address_645[6]<<4)  + generic_485[READ_CMDS].modbus.address_645[7])
        && (data[9]  == (generic_485[READ_CMDS].modbus.address_645[8]<<4)  + generic_485[READ_CMDS].modbus.address_645[9])
        && (data[10] == (generic_485[READ_CMDS].modbus.address_645[10]<<4) + generic_485[READ_CMDS].modbus.address_645[11])
		&& (data[11] == START_FRAME_BYTE)
		&& (data[12] == 0x91)
		&& (_crc_check(&data[4], n-6) == crc)
		)
    	{
    		uint8_t len = data[13];
    		for ( i = 0; i < len; i++ )
    		{
    			generic_485[READ_CMDS].modbus.command[ next.command ].response.values[ i ] = data[DLT645_MSG_TX_START_BODY + i] - 0x33;
    		}
    		generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = 0;
    		generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
    		generic_modbus_table_manager_write_slave(next.slave);
    		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLT-645> RX Add:%d%d%d%d%d%d%d\r\n", (int)Tick_Get( SECONDS ),
    				generic_485[READ_CMDS].modbus.address_645[0],
					generic_485[READ_CMDS].modbus.address_645[1],
					generic_485[READ_CMDS].modbus.address_645[2],
					generic_485[READ_CMDS].modbus.address_645[3],
					generic_485[READ_CMDS].modbus.address_645[4],
					generic_485[READ_CMDS].modbus.address_645[5],
					generic_485[READ_CMDS].modbus.address_645[6]
					);
    	}
    	else
    	{
    		__errorFrame(0x10);
    	}
	}
	else
	{
		__errorFrame(0x10);
	}
	return 0;
}

uint32_t wait = 0;
void generic_485_receive( uint8_t *data, uint32_t n )
{
	uint8_t *ch, i;
	uint16_t aux16, aux216;

	wait++;
	generic_modbus_table_manager_read_slave(next.slave);

	if ((generic_485_get_dtl645() == 1) && (generic_485[READ_CMDS].type == MODBUS_RAW))
	{
		__dtl_645_recv(data, n);
	}
	else if ( n > 4 )
	{
		generic_485[READ_CMDS].modbus.tx_on = 0;
		memcpy( &aux16, &data[ n - 2 ], sizeof( aux16 ));
		memcpy( &aux216, &data[ 6 ], sizeof( aux216 ));
		if ( ( aux216 == usMBCRC16( data, 6 ) ) || ( aux16 == usMBCRC16( data, n - 2 ) ) )
		{
			ch = data;
			if ( *ch++ == generic_485[READ_CMDS].modbus.slave )
			{
				generic_485[READ_CMDS].modbus.attempt = 0;
				if ( *ch == generic_485[READ_CMDS].modbus.command[ next.command ].function )
				{
					wait = 0;
					ch++;
					switch( generic_485[READ_CMDS].modbus.command[ next.command ].function )
					{
					case 3:
					case 4:
						__readHoldingRegistersInputRegisters(ch);
						break;
					case 5:
					case 6:
						__writeSingleCoilSingleRegister(ch);
						break;
					case 15:
					case 16:
						__writeMultipleCoilsMultipleRegisters(ch);
						break;
					default:
						generic_modbus_table_manager_write_slave(next.slave);
						break;
					}
				}
				else if ( *ch++ == ( generic_485[READ_CMDS].modbus.command[ next.command ].function | 0x80 ))
				{
					uint8_t ch2[2];
					ch2[1] = 0;
					ch2[0] = *ch;
					for ( i = 0; i < generic_485[READ_CMDS].modbus.command[ next.command ].quantity; i++ )
					{
						generic_485[READ_CMDS].modbus.command[ next.command ].response.values[ i ] = mb_swaps( ch2 );
					}
					generic_485[READ_CMDS].modbus.command[ next.command ].response.error_code = 0x10;
					generic_485[READ_CMDS].modbus.command[ next.command ].response.available  = 1;
					generic_modbus_table_manager_write_slave(next.slave);
					LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC MODBUS> Error Function Slave:%d Command:%d Error Code:%d.\r\n",
							(int)Tick_Get( SECONDS ), next.slave, next.command, (int)ch2[0]);
				}
			}
		}
		else
		{
			__errorFrame(8);
			wait = 0;
		}
	}
	else //if (wait > 5)
	{
		__errorFrame(4);
		wait = 0;
	}
}

uint16_t *generic_485_get_raw_modbus( uint16_t *quantity, uint16_t *buff, uint32_t slave_num, uint32_t *slave_id, uint8_t *on_demand )
{
	uint8_t j, prev_err = 0;
	uint32_t str_len = 0;

	generic_modbus_table_manager_read_slave(slave_num);

	*quantity = 0;
	*slave_id = generic_485[READ_CMDS].modbus.slave;

	for ( j = 0; j < MAX_COMMANDS; j++ )
	{
		if ( 1 == generic_485[READ_CMDS].modbus.command[ j ].response.available )
		{
			if ( 0 == generic_485[ READ_CMDS ].modbus.command[ j ].response.error_code )
			{
				*quantity += (generic_485[READ_CMDS].modbus.command[ j ].quantity + 1);
				generic_485[READ_CMDS].modbus.command[ j ].response.available = 0;/* delete frame.*/
				memcpy(buff + str_len, generic_485[READ_CMDS].modbus.command[ j ].response.values, generic_485[READ_CMDS].modbus.command[ j ].quantity * sizeof(uint16_t));
				str_len += generic_485[READ_CMDS].modbus.command[ j ].quantity;
				buff[str_len] = 0x7C7C;
				str_len += 1;
				*on_demand = generic_485[READ_CMDS].modbus.command[ j ].response.on_demand;
				prev_err = 0;
			}
			else
			{
				*quantity += (1 + 1 + 1);
				generic_485[ READ_CMDS ].modbus.command[ j ].response.values[0] = ( ( ((0x80 + generic_485[ READ_CMDS ].modbus.command[ j ].function) << 8) & 0xFF00 ) | ( generic_485[ READ_CMDS ].modbus.command[ j ].response.error_code & 0x00FF ) );
				generic_485[ READ_CMDS ].modbus.command[ j ].response.available = 0;/* delete frame.*/
				if ( ( 0 == j ) || ( 1 == prev_err ))
				{
					buff[str_len]  = 0x2A7C;
				}
				else
				{
					buff[str_len] = 0x7C7C;
				}
				memcpy(buff + 1 + str_len, generic_485[ READ_CMDS ].modbus.command[ j ].response.values, generic_485[ READ_CMDS ].modbus.command[ j ].quantity * sizeof(uint16_t));
				str_len       += (1 + 1);
				buff[str_len]  = 0x2A7C;
				str_len       += 1;
				prev_err = 1;
			}
		}
	}
	generic_modbus_table_manager_write_slave(slave_num);
	return buff;
}

uint8_t *generic_485_get_raw_dlt645( uint16_t *quantity, uint8_t *buff, uint32_t slave_num, uint8_t *slave_id )
{
	uint8_t i, j;
	uint32_t str_len = 0;

	generic_modbus_table_manager_read_slave(slave_num);

	*quantity = 0;
	for (i = 0; i < 12; i++)
	{
		slave_id[i] =  generic_485[READ_CMDS].modbus.address_645[i] + '0';
	}
	slave_id[12] = '\0';

	for ( j = 0; j < MAX_COMMANDS; j++ )
	{
		if ( 1 == generic_485[READ_CMDS].modbus.command[ j ].response.available )
		{
			*quantity += generic_485[READ_CMDS].modbus.command[ j ].quantity;
			generic_485[READ_CMDS].modbus.command[ j ].response.available = 0;/* delete frame.*/
			for (i = 0; i < generic_485[READ_CMDS].modbus.command[ j ].quantity; i++)
			{
				buff[i + str_len] = (uint8_t)generic_485[READ_CMDS].modbus.command[ j ].response.values[i];
			}
			str_len += generic_485[READ_CMDS].modbus.command[ j ].quantity;
		}
	}
	generic_modbus_table_manager_write_slave(slave_num);
	return buff;
}

void generic_485_reset_on_demand( void )
{
	uint32_t j = 0, slave = 0;
	for (slave = 0; slave < params_get_modbus_slave_num(); slave++)
	{
		generic_modbus_table_manager_read_slave(slave);

		for ( j = 0; j < MAX_COMMANDS; j++ )
		{
//			if ( 1 == generic_485[READ_CMDS].modbus.command[ j ].response.available )
			{
//				generic_485[READ_CMDS].modbus.command[ j ].response.available = 0;/* delete frame.*/
				generic_485_set_is_on_demand(j, 0);
			}
		}
		generic_485[READ_CMDS].modbus.enable = 0;
		generic_modbus_table_manager_write_slave(slave);
	}
}

/**
 * @}
 */

/**
 * @}
 */
