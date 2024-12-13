/*
 * dlms_client_on_demand_comm.h
 *
 *  Created on: 23 may. 2022
 *      Author: EQUIPO
 */

#ifndef DLMS_CLIENT_INC_DLMS_CLIENT_ON_DEMAND_COMM_H_
#define DLMS_CLIENT_INC_DLMS_CLIENT_ON_DEMAND_COMM_H_

#include "connection.h"

#include <stdlib.h> // malloc and free needs this or error is generated.
#include <stdio.h>
#include <string.h> // string function definitions
#include <time.h>   // time calls

#include "iwdg.h"
#include "params.h"
#include "dlms_client.h"
#include "dlms_client_table.h"
#include "tick.h"
#include "communication.h"
#include "cosem.h"
#include "converters.h"
#include "udp_protocol.h"
#include "shutdown.h"
#include "message_queue.h"
#include "dlms_log.h"
#include "gxmem.h"

uint32_t dlms_client_on_demand_comm_exc(dlms_client_command_t comm, connection* con);

uint32_t dlms_client_on_demand_set_calendar_name(char *calendarName);
uint32_t dlms_client_on_demand_set_activate_calendar_time(char *calendarTime);
uint32_t dlms_client_on_demand_set_season_profile_id(uint32_t index,char *seasonProfileId);
uint32_t dlms_client_on_demand_set_season_start(uint32_t index,char *seasonStart);
uint32_t dlms_client_on_demand_set_season_profile_week_profile_id(uint32_t index,char *weekProfileId);
uint32_t dlms_client_on_demand_set_week_profile_id(uint32_t index,char *weekProfileId);
uint32_t dlms_client_on_demand_set_week_profile_monday(uint32_t index,char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_week_profile_tuesday(uint32_t index,char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_week_profile_wednesday(uint32_t index, char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_week_profile_thursday(uint32_t index, char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_week_profile_friday(uint32_t index, char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_week_profile_saturday(uint32_t index, char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_week_profile_sunday(uint32_t index, char *weekProfileMonday);
uint32_t dlms_client_on_demand_set_day_profile_dayId(uint32_t index, char *dayId);
uint32_t dlms_client_on_demand_set_day_profile_qty(uint32_t index, uint32_t qty);
uint32_t dlms_client_on_demand_set_day_profile_dayItem_startTime(uint32_t index, uint32_t item, char *startTime);
uint32_t dlms_client_on_demand_set_day_profile_dayItem_scriptLogicalName(uint32_t index, uint32_t item, char *scriptLogicalName);
uint32_t dlms_client_on_demand_set_day_profile_dayItem_scriptSelector(uint32_t index, uint32_t item, uint32_t scriptSelector);

void dlms_client_on_demand_set_profile_capture_load1( uint32_t _profile );
void dlms_client_on_demand_set_profile_capture_load2( uint32_t _profile );
void dlms_client_on_demand_set_profile_capture_powerquality( uint32_t _profile );
void dlms_client_on_demand_set_profile_capture_instrumentation( uint32_t _profile );
void dlms_client_on_demand_set_threshold( uint32_t __threshold );
void dlms_client_on_demand_set_threshDuration( uint32_t __threshDuration );
void dlms_client_on_demand_set_control_mode( uint32_t __mode );
void dlms_client_on_demand_set_voltage_range( uint32_t __range );
void dlms_client_on_demand_set_current_range( uint32_t __range );
void dlms_client_on_demand_set_billing_date( char * __billing_date );
void dlms_client_on_demand_set_demand_integration_period( uint32_t __demand_integration );
void dlms_client_on_demand_set_payment_mode( uint32_t __payment_mode );
void dlms_client_on_demand_set_metering_mode( uint32_t __metering_mode );

uint32_t dlms_client_on_demand_gw_get_time(connection* con);
uint32_t dlms_client_on_demand_gw_set_synch(connection* con);
uint32_t dlms_client_on_demand_gw_set_interval(connection* con);
uint32_t dlms_client_on_demand_gw_get_interval(connection* con);
uint32_t dlms_client_on_demand_gw_get_nameplate(connection* con);
uint32_t dlms_client_on_demand_gw_ping(connection* con);

uint32_t dlms_client_on_demand_cut_off_reconnection(connection* con);
uint32_t dlms_client_on_demand_mdi_eob(connection* con);
uint32_t dlms_client_on_demand_set_load_limitation(connection* con);
uint32_t dlms_client_on_demand_get_load_limitation(connection* con);
uint32_t dlms_client_on_demand_set_billing_date_sched(connection* con);
uint32_t dlms_client_on_demand_get_billing_date_sched(connection* con);
uint32_t dlms_client_on_demand_set_tariff_agreement(connection* con);
uint32_t dlms_client_on_demand_read_clock(connection* con);
uint32_t dlms_client_on_demand_read_name_meter_plate_info(connection* con);
uint32_t dlms_client_on_demand_get_voltage_range_low(connection* con);
uint32_t dlms_client_on_demand_get_voltage_range_up(connection* con);
uint32_t dlms_client_on_demand_set_voltage_range_low(connection* con);
uint32_t dlms_client_on_demand_set_voltage_range_up(connection* con);
uint32_t dlms_client_on_demand_get_current_range_low(connection* con);
uint32_t dlms_client_on_demand_get_current_range_up(connection* con);
uint32_t dlms_client_on_demand_set_current_range_low(connection* con);
uint32_t dlms_client_on_demand_set_current_range_up(connection* con);
uint32_t dlms_client_on_demand_comand_set_demand_integration_period(connection* con);
uint32_t dlms_client_on_demand_comand_get_demand_integration_period(connection* con);
uint32_t dlms_client_on_demand_comand_set_payment_mode(connection* con);
uint32_t dlms_client_on_demand_comand_get_payment_mode(connection* con);
uint32_t dlms_client_on_demand_comand_set_metering_mode(connection* con);
uint32_t dlms_client_on_demand_comand_get_metering_mode(connection* con);
uint32_t dlms_client_on_demand_comand_get_meter_status(connection* con);
uint32_t dlms_client_on_demand_meter_ping(connection* con);
uint32_t dlms_client_on_demand_comm_synchronize(connection* con);
uint32_t dlms_client_on_demand_comm_obis_profile(connection* con);
uint32_t dlms_client_on_demand_comm_max_demand_profile(connection *con);
uint32_t dlms_client_on_demand_comm_capture_period(connection* con);
uint32_t dlms_client_on_demand_comm_set_capture_period(connection* con);
uint32_t dlms_client_on_demand_comm_load_profile(connection* con);
uint32_t dlms_client_on_demand_comm_clear_alarms(connection* con);
uint32_t dlms_client_on_demand_comand_get_curr_active_tariff(connection* con);

uint32_t dlms_client_on_demand_cmd_check_send( void );

#endif /* DLMS_CLIENT_INC_DLMS_CLIENT_ON_DEMAND_COMM_H_ */
