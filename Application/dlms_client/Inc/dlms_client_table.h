/*
 * dlms_client_table.h
 *
 *  Created on: 8 mar. 2022
 *      Author: smill
 */

#ifndef DLMS_CLIENT_INC_DLMS_CLIENT_TABLE_H_
#define DLMS_CLIENT_INC_DLMS_CLIENT_TABLE_H_

#include "stm32u5xx_hal.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "spi_flash.h"

#include "dlms_client.h"
#include "connection.h"
#include "communication.h"
#include "params.h"

uint32_t dlms_client_table_get_num_devices( void );
void     dlms_client_table_set_num_devices( uint32_t _num_devices );

uint32_t dlms_client_table_flush_client_obis_profile(dlms_client_obis_profile_t_Ptr dlms_client_obis_profile);
uint32_t dlms_client_table_flush_client_generic_profile(dlms_client_obis_profile_t_Ptr dlms_client_generic_profile);
uint32_t dlms_client_read_client(uint32_t client_id, dlms_client_t_Ptr dlms_client);
uint32_t dlms_client_table_read_client_obis_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_obis_profile_t_Ptr dlms_client_obis_profile, uint32_t obis_profile_index);
uint32_t dlms_client_table_read_client_generic_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_generic_profile_t_Ptr dlms_client_generic_profile, uint32_t generic_profile_index);

uint32_t dlms_client_table_init_sector(uint32_t client_id);
uint32_t dlms_client_table_write_client(uint32_t client_id, dlms_client_t_Ptr dlms_client);
uint32_t dlms_client_table_write_obis_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_obis_profile_t_Ptr dlms_client_obis_profile, uint32_t obis_profile_index);
uint32_t dlms_client_table_write_generic_profile(uint32_t client_id, dlms_client_t_Ptr dlms_client, dlms_client_generic_profile_t_Ptr dlms_client_generic_profile, uint32_t generic_profile_index);

uint32_t dlms_client_table_read_client_id( dlms_client_t_Ptr dlms_client, uint32_t id );
uint32_t dlms_client_table_write_client_id( dlms_client_t_Ptr dlms_client, uint32_t client_id );
uint32_t dlms_client_table_check_meter_id( dlms_client_t_Ptr dlms_client, uint8_t *str );

uint32_t dlms_client_table_get_curr_meter_from_cmd(void);
void     dlms_client_table_inc_curr_meter_from_cmd(void);
uint32_t dlms_client_table_get_num_meters_from_cmd(void);
void     dlms_client_table_inc_num_meters_from_cmd(void);
void     dlms_client_table_set_num_meters_from_cmd(uint32_t __num_meters);
uint32_t dlms_client_table_reset_devices_from_cmd(void);
uint32_t dlms_client_table_add_meter_id_from_cmd( dlms_client_t_Ptr dlms_client, uint8_t *str );
uint32_t dlms_client_table_inc_reading_items(void);
uint32_t dlms_client_table_inc_reading_items_index_meter(uint32_t meter_index);

#endif /* DLMS_CLIENT_INC_DLMS_CLIENT_TABLE_H_ */
