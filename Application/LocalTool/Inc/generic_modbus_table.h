/*
 * generic_modbus_table.h
 *
 *  Created on: 6 feb. 2024
 *      Author: Sergio Millán López
 */

#ifndef LOCALTOOL_INC_GENERIC_MODBUS_TABLE_H_
#define LOCALTOOL_INC_GENERIC_MODBUS_TABLE_H_

#include "stm32u5xx_hal.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "spi_flash.h"
#include "generic_modbus.h"

#include "params.h"

#define GENERIC_MODBUS_TABLE_TOTAL_SIZE (0x42000)

uint32_t generic_modbus_table_get_num_devices( void );
void     generic_modbus_table_set_num_devices( uint32_t _num_devices );
void     generic_modbus_init_table(void);
uint8_t  generic_modbus_table_manager_read_table( uint32_t _slaves );
uint8_t  generic_modbus_table_manager_read_slave( uint32_t _slaves );
void     generic_modbus_table_manager_write_slave( uint32_t _slaves );
#endif /* LOCALTOOL_INC_GENERIC_MODBUS_TABLE_H_ */
