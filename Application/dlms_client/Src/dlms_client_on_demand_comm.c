/*
 * dlms_client_on_demand_comm.c
 *
 *  Created on: 23 may. 2022
 *      Author: EQUIPO
 */
#include "dlms_client_on_demand_comm.h"
#include "dlms_client.h"

typedef struct _capture_profile
{
	uint32_t load1;
	uint32_t load2;
	uint32_t powerquality;
	uint32_t instrumentation;
}profile_capture_t;

profile_capture_t profile_capture;

typedef struct _season_profile
{
	char seasonProfileID[20];
	char seasonStart[20];
	char weekProfileID[20];
}season_profile_t;

typedef struct _week_profile
{
	char weekProfileID[20];
	char monday[20];
	char tuesday[20];
	char wednesday[20];
	char thursday[20];
	char friday[20];
	char saturday[20];
	char sunday[20];
}week_profile_t;

typedef struct _day_item
{
	char startTime[20];
	char scriptLogicalName[20];
	uint32_t scriptSelector;
}day_item_t;

typedef struct _day_profile
{
	char dayID[20];
	uint32_t qty;
	day_item_t dayItem[3];
}day_profile_t;

typedef struct _activity_calendar
{
	char calendarName[20];
	char activateCalendarTime[20];
	season_profile_t seasonProfile[3];
	week_profile_t weekProfile[3];
	day_profile_t dayProfile[3];
}activity_calendar_t;

activity_calendar_t activity_calendar_params;

uint32_t threshold = 0, threshDuration = 0;
uint32_t control_mode = 0;
uint32_t volt_range   = 0;
uint32_t curr_range   = 0;
char billing_date[20]="yyyy-MM-dd HH:mm:ss\0"; //yyyy-MM-dd HH:mm:ss
uint32_t demand_integration_period = 0;
uint32_t payment_mode              = 0;
uint32_t metering_mode             = 0;

static uint32_t (*f_list[ DLMS_HES_COMMAND_LAST ])( connection* con ) =
{
		(void *)DLMS_HES_COMMAND_NONE,
		dlms_client_on_demand_comm_obis_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_ENERGY_REGISTERS,
		dlms_client_on_demand_comm_max_demand_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS,
		dlms_client_on_demand_comm_load_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_1,
		dlms_client_on_demand_comm_load_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_2,
		dlms_client_on_demand_comm_load_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE,
		dlms_client_on_demand_comm_load_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE,
		dlms_client_on_demand_comm_obis_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES,
		dlms_client_on_demand_comm_load_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_BILLING_PROFILE,
		dlms_client_on_demand_comm_load_profile,//(void *)DLMS_HES_COMMAND_ON_DEMAND_EVENTS_PROFILE,
		dlms_client_on_demand_read_clock,//(void *)DLMS_HES_COMMAND_READ_TIME_CLOCK
		dlms_client_on_demand_comm_capture_period,//((void *)DLMS_HES_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD,
		dlms_client_on_demand_comm_set_capture_period,//((void *)DLMS_HES_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD,
		dlms_client_on_demand_cut_off_reconnection,//((void *)DLMS_HES_COMMAND_GET_SWITCH_STATUS,
		dlms_client_on_demand_cut_off_reconnection,//(void *)DLMS_HES_COMMAND_CUTTOFF_RECONNECTION,
		dlms_client_on_demand_mdi_eob,//(void *)DLMS_HES_COMMAND_MAXIMUM_DEMAND_RESET,
		dlms_client_on_demand_set_load_limitation,//(void *)DLMS_HES_COMMAND_SET_LOAD_LIMITATION,
		dlms_client_on_demand_get_load_limitation,//(void *)DLMS_HES_COMMAND_GET_LOAD_LIMIT_THRESHOLD,
		dlms_client_on_demand_set_billing_date_sched,//(void *)DLMS_HES_COMMAND_SET_BILLING_DATE,
		dlms_client_on_demand_get_billing_date_sched,//(void *)DLMS_HES_COMMAND_GET_BILLING_DATE,
		dlms_client_on_demand_comm_clear_alarms,//(void *)DLMS_HES_COMMAND_CLEAR_ALARMS,
		dlms_client_on_demand_comand_set_demand_integration_period,//(void *)DLMS_HES_COMMAND_SET_DEMAND_INTEGRATION_PERIOD,
		dlms_client_on_demand_comand_get_demand_integration_period,//(void *)DLMS_HES_COMMAND_GET_DEMAND_INTEGRATION_PERIOD,
		dlms_client_on_demand_comand_set_payment_mode,//(void *)DLMS_HES_COMMAND_SET_PAYMENT_MODE,
		dlms_client_on_demand_comand_get_payment_mode,//(void *)DLMS_HES_COMMAND_GET_PAYMENT_MODE,
		dlms_client_on_demand_comand_set_metering_mode,//(void *)DLMS_HES_COMMAND_SET_METERING_MODE,
		dlms_client_on_demand_comand_get_metering_mode,//(void *)DLMS_HES_COMMAND_GET_METERING_MODE,
		dlms_client_on_demand_set_voltage_range_low,//(void *)DLMS_HES_COMMAND_SET_VOLT_RANGE_LOW,
		dlms_client_on_demand_set_voltage_range_up,//(void *)DLMS_HES_COMMAND_SET_VOLT_RANGE_UP,
		dlms_client_on_demand_get_voltage_range_low,//(void *)DLMS_HES_COMMAND_GET_VOLT_RANGE_LOW,
		dlms_client_on_demand_get_voltage_range_up,//(void *)DLMS_HES_COMMAND_GET_VOLT_RANGE_UP,
		dlms_client_on_demand_set_current_range_low,//(void *)DLMS_HES_COMMAND_SET_CURRENT_RANGE_LOW,
		dlms_client_on_demand_set_current_range_up,//(void *)DLMS_HES_COMMAND_SET_CURRENT_RANGE_UP,
		dlms_client_on_demand_get_current_range_low,//(void *)DLMS_HES_COMMAND_GET_CURRENT_RANGE_LOW,
		dlms_client_on_demand_get_current_range_up,//(void *)DLMS_HES_COMMAND_GET_CURRENT_RANGE_UP,
		dlms_client_on_demand_comand_get_meter_status,//(void *)DLMS_HES_COMMAND_GET_METER_STATUS,
		dlms_client_on_demand_read_name_meter_plate_info,//(void *)DLMS_HES_COMMAND_READ_METER_NAMEPLATE
		dlms_client_on_demand_comm_synchronize,//(void *)DLMS_HES_COMMAND_METER_SYNCHRONIZE,
		(void *)DLMS_HES_COMMAND_RECONNECTION,
		dlms_client_on_demand_comand_get_curr_active_tariff,//(void *)DLMS_HES_COMMAND_GET_ACTIVE_TARIFF,
		dlms_client_on_demand_meter_ping,//(void *)DLMS_HES_COMMAND_METER_PING,
		dlms_client_on_demand_set_tariff_agreement,//(void *)DLMS_HES_COMMAND_METER_SET_TARIFF_AGREEMENT
		dlms_client_on_demand_gw_get_time,
		dlms_client_on_demand_comm_synchronize,//dlms_client_on_demand_gw_set_synch,
		dlms_client_on_demand_gw_get_interval,
		dlms_client_on_demand_gw_get_interval,
		dlms_client_on_demand_gw_get_nameplate,
		dlms_client_on_demand_gw_ping
};

//static uint32_t (*f[ DLMS_HES_COMMAND_LAST ])( void );

extern gxProfileGeneric pg;
extern uint32_t         retry;

uint32_t dlms_client_on_demand_comm_exc(dlms_client_command_t comm, connection* con)
{
	uint32_t ret = f_list[comm](con);

	return ret;
}

uint32_t dlms_client_on_demand_set_calendar_name(char *calendarName)
{
	strncpy(activity_calendar_params.calendarName, calendarName, strlen(calendarName));

	return 0;
}

uint32_t dlms_client_on_demand_set_activate_calendar_time(char *calendarTime)
{
	strncpy(activity_calendar_params.activateCalendarTime, calendarTime, strlen(calendarTime));

	return 0;
}

uint32_t dlms_client_on_demand_set_season_profile_id(uint32_t index, char *seasonProfileId)
{
	strncpy(activity_calendar_params.seasonProfile[index].seasonProfileID, seasonProfileId, strlen(seasonProfileId));

	return 0;
}

uint32_t dlms_client_on_demand_set_season_start(uint32_t index, char *seasonStart)
{
	strncpy(activity_calendar_params.seasonProfile[index].seasonStart, seasonStart, strlen(seasonStart));

	return 0;
}

uint32_t dlms_client_on_demand_set_season_profile_week_profile_id(uint32_t index, char *weekProfileId)
{
	strncpy(activity_calendar_params.seasonProfile[index].weekProfileID, weekProfileId, strlen(weekProfileId));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_id(uint32_t index, char *weekProfileId)
{
	strncpy(activity_calendar_params.weekProfile[index].weekProfileID, weekProfileId, strlen(weekProfileId));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_monday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].monday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_tuesday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].tuesday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_wednesday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].wednesday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_thursday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].thursday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_friday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].friday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_saturday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].saturday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_week_profile_sunday(uint32_t index, char *weekProfileMonday)
{
	strncpy(activity_calendar_params.weekProfile[index].sunday, weekProfileMonday, strlen(weekProfileMonday));

	return 0;
}

uint32_t dlms_client_on_demand_set_day_profile_dayId(uint32_t index, char *dayId)
{
	strncpy(activity_calendar_params.dayProfile[index].dayID, dayId, strlen(dayId));

	return 0;
}

uint32_t dlms_client_on_demand_set_day_profile_qty(uint32_t index, uint32_t qty)
{
	activity_calendar_params.dayProfile[index].qty = qty;

	return 0;
}

uint32_t dlms_client_on_demand_set_day_profile_dayItem_startTime(uint32_t index, uint32_t item, char *startTime)
{
	strncpy(activity_calendar_params.dayProfile[index].dayItem[item].startTime, startTime, strlen(startTime));

	return 0;
}

uint32_t dlms_client_on_demand_set_day_profile_dayItem_scriptLogicalName(uint32_t index, uint32_t item, char *scriptLogicalName)
{
	strncpy(activity_calendar_params.dayProfile[index].dayItem[item].scriptLogicalName, scriptLogicalName, strlen(scriptLogicalName));

	return 0;
}

uint32_t dlms_client_on_demand_set_day_profile_dayItem_scriptSelector(uint32_t index, uint32_t item, uint32_t scriptSelector)
{
	activity_calendar_params.dayProfile[index].dayItem[item].scriptSelector = scriptSelector;

	return 0;
}

void dlms_client_on_demand_set_profile_capture_load1( uint32_t _profile )
{
	profile_capture.load1 = _profile;
}

void dlms_client_on_demand_set_profile_capture_load2( uint32_t _profile )
{
	profile_capture.load2 = _profile;
}

void dlms_client_on_demand_set_profile_capture_powerquality( uint32_t _profile )
{
	profile_capture.powerquality = _profile;
}

void dlms_client_on_demand_set_profile_capture_instrumentation( uint32_t _profile )
{
	profile_capture.instrumentation = _profile;
}

void dlms_client_on_demand_set_threshold( uint32_t __threshold )
{
	threshold = __threshold;
}

void dlms_client_on_demand_set_threshDuration( uint32_t __threshDuration )
{
	threshDuration = __threshDuration;
}

void dlms_client_on_demand_set_control_mode( uint32_t __mode )
{
	control_mode = __mode;
}

void dlms_client_on_demand_set_voltage_range( uint32_t __range )
{
	volt_range= __range;
}

void dlms_client_on_demand_set_current_range( uint32_t __range )
{
	curr_range= __range;
}

void dlms_client_on_demand_set_billing_date( char * __billing_date )
{
	strncpy((char *)billing_date , __billing_date, sizeof(billing_date));
}

void dlms_client_on_demand_set_demand_integration_period( uint32_t __demand_integration )
{
	demand_integration_period = __demand_integration;
}

void dlms_client_on_demand_set_payment_mode( uint32_t __payment_mode )
{
	payment_mode = __payment_mode;
}

void dlms_client_on_demand_set_metering_mode( uint32_t __metering_mode )
{
	metering_mode = __metering_mode;
}

uint32_t dlms_client_on_demand_gw_get_time(connection* con)
{
	char gw_get_time_string[200];
	sprintf(gw_get_time_string, "%d;%d;#", (int)rtc_system_GetServerTime(), (int)rtc_system_GetServerTime());
	dlms_client_set_dlms_data_cmd_on_demand(gw_get_time_string);

	con->state = DLMS_STATE_IDLE;

	return DLMS_ERROR_CODE_OK;
}

uint32_t dlms_client_on_demand_gw_set_synch(connection* con)
{
	return DLMS_ERROR_CODE_OK;
}

uint32_t dlms_client_on_demand_gw_set_interval(connection* con)
{
	char gw_get_time_string[200];
	uint32_t len = 0, i = 0;
	for(i = 0; i < DLMS_OBIS_PROFILE_NUM; i++)
	{
		dlms_client_table_read_client_obis_profile(0, dlms_client_get_client(), dlms_client_get_client_obis_profile(), i);
		if (dlms_client_get_obis_profile_read_time() != -1)
		{
			sprintf(gw_get_time_string + len, "%d;%d;",
					(int)dlms_client_get_obis_profile_send_time(),
					(int)dlms_client_get_obis_profile_frame_type()
			);
			len = strlen(gw_get_time_string);
		}
	}
	for(i = 0; i < DLMS_GENERIC_PROFILE_NUM; i++)
	{
		dlms_client_table_read_client_generic_profile(0, dlms_client_get_client(), dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + i);
		if (dlms_client_get_dlms_load_profile_read_time() != -1)
		{
			sprintf(gw_get_time_string + len, "%d;%d;",
					(int)dlms_client_get_dlms_load_profile_send_time(),
					(int)dlms_client_get_generic_profile_frame_type()
			);
			len = strlen(gw_get_time_string);
		}
	}
	dlms_client_set_dlms_data_cmd_on_demand(gw_get_time_string);

	con->state = DLMS_STATE_IDLE;

	return DLMS_ERROR_CODE_OK;
}

uint32_t dlms_client_on_demand_gw_get_interval(connection* con)
{
	char gw_get_time_string[400];
	uint32_t len = 0, i = 0 , j = 0;
	for (j = 0; j < dlms_client_table_get_num_devices(); j++)
	{
		sprintf(gw_get_time_string + len, "%s;", dlms_client_get_meter_id(j));
		len = strlen(gw_get_time_string);
		for(i = 0; i < DLMS_OBIS_PROFILE_NUM; i++)
		{
			dlms_client_table_read_client_obis_profile(j, dlms_client_get_client(), dlms_client_get_client_obis_profile(), i);
			if (dlms_client_get_obis_profile_read_time() != -1)
			{
				sprintf(gw_get_time_string + len, "%d;",
						(int)dlms_client_get_obis_profile_send_time()
				);
				len = strlen(gw_get_time_string);
			}
			else
			{
				sprintf(gw_get_time_string + len, ";");
				len = strlen(gw_get_time_string);
			}
		}
		for(i = 0; i < DLMS_GENERIC_PROFILE_NUM; i++)
		{
			dlms_client_table_read_client_generic_profile(j, dlms_client_get_client(), dlms_client_get_client_generic_profile(), DLMS_OBIS_PROFILE_NUM + i);
			if (dlms_client_get_dlms_load_profile_read_time() != -1)
			{
				sprintf(gw_get_time_string + len, "%d;",
						(int)dlms_client_get_dlms_load_profile_send_time()
				);
				len = strlen(gw_get_time_string);
			}
			else
			{
				sprintf(gw_get_time_string + len, ";");
				len = strlen(gw_get_time_string);
			}
		}
		sprintf(gw_get_time_string + len, "%d;", (int)param.config.st);//heartbeat
		len = strlen(gw_get_time_string);

		sprintf(gw_get_time_string + len, "%d;", (int)dlms_client_get_billingday());//billingdate
		len = strlen(gw_get_time_string);

		sprintf(gw_get_time_string + len, "#");
		len = strlen(gw_get_time_string);
	}
	dlms_client_set_dlms_data_cmd_on_demand(gw_get_time_string);

	con->state = DLMS_STATE_IDLE;

	return DLMS_ERROR_CODE_OK;
}

uint32_t dlms_client_on_demand_gw_get_nameplate(connection* con)
{
	return DLMS_ERROR_CODE_OK;
}

uint32_t dlms_client_on_demand_gw_ping(connection* con)
{
	return DLMS_ERROR_CODE_OK;
}

uint32_t dlms_client_on_demand_cut_off_reconnection(connection* con)
{
	int                 ret, mt372 = 0;
	gxObject            obj;
	gxDisconnectControl dc;
	dlmsVARIANT         params;
//	const unsigned char ln[6]   = { 0,0,96,3,10,255 };
//	const unsigned char ln_2[6] = { 0,1,96,3,10,255 };
	const unsigned char pln[2][6] = { {0,1,96,3,10,255}, {0,0,96,3,10,255} };
	char                reconnection_string[200];
	static char         *state_strings[ 3 ] = {"disconnected", "connected", "Ready_for_reconnection",};

    gxData disconnect_state, disconnect_mode;
    const unsigned char ln_disc_mode[6]  = { 0, 0, 128, 30, 20, 255 };
    const unsigned char ln_disc_state[6] = { 0, 0, 128, 30, 21, 255 };

	uint8_t            outputState;
	DLMS_CONTROL_STATE controlState;
	DLMS_CONTROL_MODE  controlMode;
	uint32_t next_obis_loop = 0;
	params.vt   = DLMS_DATA_TYPE_INT8;
	params.cVal = 0;
    do {
	if ( ( ret = INIT_OBJECT(dc, DLMS_OBJECT_TYPE_DISCONNECT_CONTROL, pln[next_obis_loop]) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(dc), 2);
		HAL_IWDG_Refresh(&hiwdg);

		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(dc), 3);
		HAL_IWDG_Refresh(&hiwdg);

		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(dc), 4);
		HAL_IWDG_Refresh(&hiwdg);

		if  (ret != 0)
		{
		    if ((ret = INIT_OBJECT(disconnect_mode, DLMS_OBJECT_TYPE_DATA, ln_disc_mode)) == 0)
		    {
		    	mt372 = 1;

				dlms_client_reset_indexes();
				ret 			= com_read(con, BASE(disconnect_mode), 2);
				char *obis_mode = dlms_client_get_obis_value_next();
				dc.controlMode  = controlMode  = atoi(obis_mode);
		    }
		    if ((ret = INIT_OBJECT(disconnect_state, DLMS_OBJECT_TYPE_DATA, ln_disc_state)) == 0)
		    {
		    	mt372 = 1;

				dlms_client_reset_indexes();
				ret				 = com_read(con, BASE(disconnect_state), 2);
				char *obis_state = dlms_client_get_obis_value_next();
				dc.outputState   = outputState  = atoi(obis_state);
				dc.controlState  = controlState = atoi(obis_state);
		    }
		}
		if ( dlms_client_get_disconnected() != 0 )
		{
			if (0 == mt372)
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> com_write Control Mode:%d\r\n", (int)Tick_Get(SECONDS), (int)control_mode);

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				dc.controlMode = control_mode;
				ret = com_write(con, BASE(dc), 4);

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> com_method disconnect:%d\r\n", (int)Tick_Get(SECONDS), (int)dlms_client_get_disconnected());
				ret = com_method(con, BASE(dc), dlms_client_get_disconnected(), &params);
				HAL_IWDG_Refresh(&hiwdg);
				if ((DLMS_ERROR_CODE_READ_WRITE_DENIED == ret) && (0 == next_obis_loop))
				{
					next_obis_loop = 1;
					continue;
				}
				else
				{
					next_obis_loop = 2;
				}
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, BASE(dc), 2);
				HAL_IWDG_Refresh(&hiwdg);

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, BASE(dc), 3);
				HAL_IWDG_Refresh(&hiwdg);

				ret = com_read(con, BASE(dc), 4);
				HAL_IWDG_Refresh(&hiwdg);

				outputState  = dc.outputState;
				controlState = dc.controlState;
				controlMode  = dc.controlMode;

				dlms_client_reset_indexes();
				ret = cosem_init(&obj, DLMS_OBJECT_TYPE_REGISTER, "1.0.1.8.0.255");
				if ( 1 == dlms_client_get_short_name_ref() )
				{
					obj.shortName = 0x24e0;
				}

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, &obj, 2);
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, &obj, 3);

				obj_clear(&obj);
			}
			else
			{
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				GX_UINT8(disconnect_mode.value) = control_mode;
				ret = com_write(con, BASE(disconnect_mode), 2);

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				dlms_client_reset_indexes();
				ret = com_read(con, BASE(disconnect_mode), 2);
				char *obis_mode = dlms_client_get_obis_value_next();

				controlMode  = atoi(obis_mode);

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				GX_UINT8(disconnect_state.value) = dlms_client_get_disconnected() - 1;
				ret = com_write(con, BASE(disconnect_state), 2);

				dlms_client_reset_indexes();
				ret = com_read(con, BASE(disconnect_state), 2);
				char *obis_state = dlms_client_get_obis_value_next();

				outputState  = atoi(obis_state);
				controlState = atoi(obis_state);

				dlms_client_reset_indexes();
				ret = cosem_init(&obj, DLMS_OBJECT_TYPE_REGISTER, "1.0.1.8.0.255");
				if ( 1 == dlms_client_get_short_name_ref() )
				{
					obj.shortName = 0x24e0;//con_get_shortName(0);
				}
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, &obj, 2);
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, &obj, 3);

				obj_clear(&obj);
			}
			if ( DLMS_ERROR_CODE_UNAVAILABLE_OBJECT == ret )
			{
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> DLMS_ERROR_CODE_UNAVAILABLE_OBJECT Code:%d\r\n", (int)Tick_Get(SECONDS), (int)dlms_client_get_disconnected());
				sprintf(reconnection_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
				dlms_client_set_dlms_data_cmd_on_demand(reconnection_string);
			}
			else
			{
				sprintf(reconnection_string, "%s;%s;%s;%d;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), state_strings[outputState], state_strings[controlState], (int)controlMode, dlms_client_get_obis_value_next());
				dlms_client_set_dlms_data_cmd_on_demand(reconnection_string);
			}
			shutdown_set_tamper_param_cmd(TAMPER_MQTT_RECONNECT);
		}
		else
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> SWITCH STATUS:%d\r\n", (int)Tick_Get(SECONDS), (int)dc.controlState);
			sprintf(reconnection_string, "%s;%s;%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), state_strings[dc.outputState], state_strings[dc.controlState], (int)dc.controlMode);
			dlms_client_set_dlms_data_cmd_on_demand(reconnection_string);
			shutdown_set_tamper_param_cmd(TAMPER_MQTT_SWITCH_STATUS);
			next_obis_loop = 2;
		}
		obj_clear(BASE(dc));
		if (1 == mt372)
		{
			obj_clear(BASE(disconnect_mode));
			obj_clear(BASE(disconnect_state));
		}
	}
    }while(next_obis_loop!=2);
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_mdi_eob(connection* con)
{
	int           ret;
	gxScriptTable script;
	dlmsVARIANT   params;
	const unsigned char ln[6] = { 0, 0, 10, 0, 1, 255 };
	char script_string[200];

	params.vt    = DLMS_DATA_TYPE_UINT16;
	params.uiVal = 1;

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(script, DLMS_OBJECT_TYPE_SCRIPT_TABLE, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
//			script.base.shortName = 0x4F00;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(script), 1);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_method(con, BASE(script), 1, &params);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(script_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Cleared");
			dlms_client_set_dlms_data_cmd_on_demand(script_string);
		}
		else
		{
			sprintf(script_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(script_string);
		}
		obj_clear(BASE(script));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_set_load_limitation(connection* con)
{
	int           ret;
	gxLimiter     limiter;
//	dlmsVARIANT   params;
	const unsigned char ln[6] = { 0, 0, 17, 0, 0, 255 };
	char limiter_string[200];

//	params.vt    = DLMS_DATA_TYPE_UINT32;
//	params.ulVal = 1;

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(limiter, DLMS_OBJECT_TYPE_LIMITER, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			limiter.base.shortName = 0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		GX_UINT32(limiter.thresholdNormal) = threshold;
		ret = com_write(con, BASE(limiter), 4);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		limiter.minOverThresholdDuration = threshDuration;
		ret = com_write(con, BASE(limiter), 6);
//		ret = com_read(con, BASE(limiter), 2);
//		ret = com_read(con, BASE(limiter), 4);
//		ret = com_read(con, BASE(limiter), 6);
//		ret = com_read(con, BASE(limiter), 7);
//		ret = com_method(con, BASE(script), 1, &params);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(limiter_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "OK");
			dlms_client_set_dlms_data_cmd_on_demand(limiter_string);
		}
		else
		{
			sprintf(limiter_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(limiter_string);
		}
		obj_clear(BASE(limiter));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_get_load_limitation(connection* con)
{
	int           ret;
	gxLimiter     limiter;
//	dlmsVARIANT   params;
	const unsigned char ln[6] = { 0, 0, 17, 0, 0, 255 };
	char limiter_string[200];

//	params.vt    = DLMS_DATA_TYPE_UINT32;
//	params.ulVal = 1;

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(limiter, DLMS_OBJECT_TYPE_LIMITER, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			limiter.base.shortName = 0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
//		ret = com_read(con, BASE(limiter), 2);
		ret = com_read(con, BASE(limiter), 4);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(limiter), 6);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(limiter), 7);
//		ret = com_method(con, BASE(script), 1, &params);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(limiter_string, "%s;%d;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)limiter.thresholdNormal.lVal, (int)limiter.minOverThresholdDuration);
			dlms_client_set_dlms_data_cmd_on_demand(limiter_string);
		}
		else
		{
			sprintf(limiter_string, "%s;%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail", "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(limiter_string);
		}
		obj_clear(BASE(limiter));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

//0.0.15.0.0.255 Single-action Schedule for Billing Dates
gxActionSchedule singleActionScheduleForBillingDates;
//0.0.10.0.1.255 MD Reset Action
gxScriptTable mdResetAction;
uint32_t dlms_client_on_demand_set_billing_date_sched(connection* con)
{
	int                 ret;
//	gxActionSchedule    action_schedule;
	//Add new action schedule.
//	unsigned char target[6]   = { 0, 1, 10, 1, 101, 255 };
    //Action schedule execution times.
    static gxtime EXECUTION_TIMES[1];
	const unsigned char ln[6] = { 0, 0, 15, 0, 0, 255 };
	char act_sched_string[200];

	char date[3], date_year[5], date_month[3], date_hour[3], date_minute[3], date_seconds[3];
	uint32_t day, year, month, hour, minute, second;

	date_hour[0] = billing_date[11];
	date_hour[1] = billing_date[12];
	date_hour[2] = '\0';
	hour         = atoi(date_hour);

	date_minute[0] = billing_date[14];
	date_minute[1] = billing_date[15];
	date_minute[2] = '\0';
	minute         = atoi(date_minute);

	date_seconds[0] = billing_date[17];
	date_seconds[1] = billing_date[18];
	date_seconds[2] = '\0';
	second          = atoi(date_seconds);

	date_hour[0] = billing_date[11];
	date_hour[1] = billing_date[12];
	date_hour[2] = '\0';
	hour         = atoi(date_hour);

	date[0] = billing_date[8];
	date[1] = billing_date[9];
	date[2] = '\0';
	day     = atoi(date);

	date_year[0] = billing_date[0];
	date_year[1] = billing_date[1];
	date_year[2] = billing_date[2];
	date_year[3] = billing_date[3];
	date_year[4] = '\0';
	year         = atoi(date_year);
	if (billing_date[0] == '*')
	{
		year = -1;
	}

	date_month[0] = billing_date[5];
	date_month[1] = billing_date[6];
	date_month[2] = '\0';
	month         = atoi(date_month);
	if ( billing_date[5] == '*')
	{
		month = -1;
	}

	if ( ( ret = INIT_OBJECT(singleActionScheduleForBillingDates, DLMS_OBJECT_TYPE_ACTION_SCHEDULE, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			singleActionScheduleForBillingDates.base.shortName = 0xC670;
		}
		ret = com_read(con, BASE(singleActionScheduleForBillingDates), 2);
//		memcpy(singleActionScheduleForBillingDates.executedScript->base.logicalName, target, 6);//singleActionScheduleForBillingDates.executedScript         = &mdResetAction;
        singleActionScheduleForBillingDates.executedScriptSelector = 1;
        singleActionScheduleForBillingDates.type                   = DLMS_SINGLE_ACTION_SCHEDULE_TYPE5;

//        ret = com_read(con, BASE(singleActionScheduleForBillingDates), 4);

    	//Add new execution time to the list.
    	time_init(&EXECUTION_TIMES[0], year, month, day, hour, minute, second, -1, -1);
    	arr_push(&singleActionScheduleForBillingDates.executionTime, EXECUTION_TIMES);
        ret = com_write(con, BASE(singleActionScheduleForBillingDates), 4);

		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(act_sched_string, "%s;%s;#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "OK");
			dlms_client_set_dlms_data_cmd_on_demand(act_sched_string);
		}
		else
		{
			sprintf(act_sched_string, "%s;%s;#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(act_sched_string);
		}
		obj_clear(BASE(singleActionScheduleForBillingDates));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t __setStartTime(uint8_t index, uint32_t *year, uint8_t *day, uint8_t *month,uint8_t *hour,uint8_t *minute,uint8_t *second)
{
	char aday[3], amonth[3], ayear[5], ahour[5], aminute[5];//, asecond[5];

	ayear[0] = activity_calendar_params.seasonProfile[index].seasonStart[0];
	ayear[1] = activity_calendar_params.seasonProfile[index].seasonStart[1];
	ayear[2] = activity_calendar_params.seasonProfile[index].seasonStart[2];
	ayear[3] = activity_calendar_params.seasonProfile[index].seasonStart[3];
	ayear[4] = '\0';
	*year = atoi(ayear);

	amonth[0] = activity_calendar_params.seasonProfile[index].seasonStart[5];
	amonth[1] = activity_calendar_params.seasonProfile[index].seasonStart[6];
	amonth[2] = '\0';
	*month = atoi(amonth);

	aday[0] = activity_calendar_params.seasonProfile[index].seasonStart[8];
	aday[1] = activity_calendar_params.seasonProfile[index].seasonStart[9];
	aday[2] = '\0';
	*day = atoi(aday);

	ahour[0] = activity_calendar_params.seasonProfile[index].seasonStart[11];
	ahour[1] = activity_calendar_params.seasonProfile[index].seasonStart[12];
	ahour[2] = '\0';
	*hour = atoi(ahour);

	aminute[0] = activity_calendar_params.seasonProfile[index].seasonStart[14];
	aminute[1] = activity_calendar_params.seasonProfile[index].seasonStart[15];
	aminute[2] = '\0';
	*minute = atoi(aminute);

	return 0;
}

uint32_t __setStartTimeDayProfile(uint8_t index, uint8_t dayindex, uint8_t *hour,uint8_t *minute)
{
	char ahour[5], aminute[5];

	ahour[0] = activity_calendar_params.dayProfile[index].dayItem[dayindex].startTime[0];
	ahour[1] = activity_calendar_params.dayProfile[index].dayItem[dayindex].startTime[1];
	ahour[2] = '\0';
	*hour = atoi(ahour);

	aminute[0] = activity_calendar_params.dayProfile[index].dayItem[dayindex].startTime[3];
	aminute[1] = activity_calendar_params.dayProfile[index].dayItem[dayindex].startTime[4];
	aminute[2] = '\0';
	*minute = atoi(aminute);

	return 0;
}

uint32_t __setActiveTime(void)
{
	struct tm        date;
	char ayear[5], amonth[3], aday[3], ahour[3], aminute[3], asecond[3];
	uint32_t ret = 0;

	ayear[0] = activity_calendar_params.activateCalendarTime[0];
	ayear[1] = activity_calendar_params.activateCalendarTime[1];
	ayear[2] = activity_calendar_params.activateCalendarTime[2];
	ayear[3] = activity_calendar_params.activateCalendarTime[3];
	ayear[4] = '\0';
	date.tm_year = atoi(ayear) - 1900;

	amonth[0] = activity_calendar_params.activateCalendarTime[5];
	amonth[1] = activity_calendar_params.activateCalendarTime[6];
	amonth[2] = '\0';
	date.tm_mon = atoi(amonth) - 1;

	aday[0] = activity_calendar_params.activateCalendarTime[8];
	aday[1] = activity_calendar_params.activateCalendarTime[9];
	aday[2] = '\0';
	date.tm_mday = atoi(aday);

	ahour[0] = activity_calendar_params.activateCalendarTime[11];
	ahour[1] = activity_calendar_params.activateCalendarTime[12];
	ahour[2] = '\0';
	date.tm_hour = atoi(ahour);

	aminute[0] = activity_calendar_params.activateCalendarTime[14];
	aminute[1] = activity_calendar_params.activateCalendarTime[15];
	aminute[2] = '\0';
	date.tm_min = atoi(aminute);

	asecond[0] = activity_calendar_params.activateCalendarTime[17];
	asecond[1] = activity_calendar_params.activateCalendarTime[18];
	asecond[2] = '\0';
	date.tm_sec = atoi(asecond);

	ret = mktime(&date);

	return ret;
}

//0.0.13.0.0.255 Activity Calendar Active Time
gxActivityCalendar activityCalendarActiveTime;
//
gxActionSchedule actionSchedule;
//Define active calendar name.
unsigned char ACTIVE_CALENDAR_NAME[20];
gxSeasonProfile ACTIVE_SEASON_PROFILE[10];
gxWeekProfile ACTIVE_WEEK_PROFILE[10];
gxDayProfile ACTIVE_DAY_PROFILE[3];
//Own day profile action for each day profile.
gxDayProfileAction ACTIVE_DAY_PROFILE_ACTIONS1[10];
gxDayProfileAction ACTIVE_DAY_PROFILE_ACTIONS2[10];
gxDayProfileAction ACTIVE_DAY_PROFILE_ACTIONS3[10];
gxRegisterActivation registerActivation;
gxRegister activePowerL1;
uint32_t dlms_client_on_demand_set_tariff_agreement(connection* con)
{
	int ret;
	gxWeekProfile* wp;
//	gxWeekProfile* wp2;
//	gxWeekProfile* wp3;
	gxDayProfile* dp;
	gxDayProfileAction* act;
    gxSeasonProfile* sp;

	uint32_t year;
	uint8_t day, month, hour, minute, second;

	char activity_calendar_string[200];

	const unsigned char ln[6] = { 0, 0, 13, 0, 0, 255 };
	if ( ( ret = INIT_OBJECT(activityCalendarActiveTime, DLMS_OBJECT_TYPE_ACTIVITY_CALENDAR, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			activityCalendarActiveTime.base.shortName = 0xC738;
		}

//		const char* activeSeasonName = activity_calendar_params.calendarName;//ACTIVE_SEASON_NAME;
//		strcpy((char*)ACTIVE_CALENDAR_NAME, activeSeasonName);
//
//		BB_ATTACH(activityCalendarActiveTime.calendarNamePassive, ACTIVE_CALENDAR_NAME, (unsigned short)strlen((char *)ACTIVE_CALENDAR_NAME));

		activity_calendar_params.calendarName[8] = '\0';
		bb_addString(&activityCalendarActiveTime.calendarNamePassive, activity_calendar_params.calendarName);
		ret = com_write(con, BASE(activityCalendarActiveTime), 6);

		/////////////////////////////////////////////////////////////////////////
		//Add active season profile 1.
	    sp = (gxSeasonProfile*)malloc(sizeof(gxSeasonProfile));
	    bb_init(&sp->name);
	    bb_addString(&sp->name, (char *)activity_calendar_params.seasonProfile[0].seasonProfileID);
	    __setStartTime(0, &year, &day, &month, &hour,&minute,&second);
	    time_init(&sp->start, year, month, day, hour, minute, 0, -1, -1);
	    bb_init(&sp->weekName);
	    bb_addString(&sp->weekName, (char *)activity_calendar_params.seasonProfile[0].weekProfileID);
	    arr_push(&activityCalendarActiveTime.seasonProfilePassive, sp);

		//Add active season profile 2.
	    sp = (gxSeasonProfile*)malloc(sizeof(gxSeasonProfile));
	    bb_init(&sp->name);
	    bb_addString(&sp->name, (char *)activity_calendar_params.seasonProfile[1].seasonProfileID);
	    __setStartTime(1, &year, &day, &month, &hour,&minute,&second);
	    time_init(&sp->start, year, month, day, hour, minute, 0, -1, -1);
	    bb_init(&sp->weekName);
	    bb_addString(&sp->weekName, (char *)activity_calendar_params.seasonProfile[1].weekProfileID);
	    arr_push(&activityCalendarActiveTime.seasonProfilePassive, sp);

		//Add active season profile 3.
	    sp = (gxSeasonProfile*)malloc(sizeof(gxSeasonProfile));
	    bb_init(&sp->name);
	    bb_addString(&sp->name, (char *)activity_calendar_params.seasonProfile[2].seasonProfileID);
	    __setStartTime(2, &year, &day, &month, &hour,&minute,&second);
	    time_init(&sp->start, year, month, day, hour, minute, 0, -1, -1);
	    bb_init(&sp->weekName);
	    bb_addString(&sp->weekName, (char *)activity_calendar_params.seasonProfile[2].weekProfileID);
	    arr_push(&activityCalendarActiveTime.seasonProfilePassive, sp);

//		ARR_ATTACH(activityCalendarActiveTime.seasonProfilePassive, ACTIVE_SEASON_PROFILE, 1);
//
//		bb_addString(&ACTIVE_SEASON_PROFILE[0].name,
//				(char *)activity_calendar_params.seasonProfile[0].seasonProfileID);
//		bb_addString(&ACTIVE_SEASON_PROFILE[1].name,
//				(char *)activity_calendar_params.seasonProfile[1].seasonProfileID);
//		bb_addString(&ACTIVE_SEASON_PROFILE[2].name,
//				(char *)activity_calendar_params.seasonProfile[2].seasonProfileID);
//
//		bb_addString(&ACTIVE_SEASON_PROFILE[0].weekName,
//				(char *)activity_calendar_params.seasonProfile[0].weekProfileID);
//		bb_addString(&ACTIVE_SEASON_PROFILE[1].weekName,
//				(char *)activity_calendar_params.seasonProfile[1].weekProfileID);
//		bb_addString(&ACTIVE_SEASON_PROFILE[2].weekName,
//				(char *)activity_calendar_params.seasonProfile[2].weekProfileID);
//
//		__setStartTime(0, &year, &day, &month, &hour,&minute,&second);
//		time_init(&ACTIVE_SEASON_PROFILE[0].start, year, month, day, hour, minute, 0, -1, -1);
//		__setStartTime(1, &year, &day, &month, &hour,&minute,&second);
//		time_init(&ACTIVE_SEASON_PROFILE[1].start, year, month, day, hour, minute, 0, -1, -1);
//		__setStartTime(2, &year, &day, &month, &hour,&minute,&second);
//		time_init(&ACTIVE_SEASON_PROFILE[2].start, year, month, day, hour, minute, 0, -1, -1);

		ret = com_write(con, BASE(activityCalendarActiveTime), 7);

		/////////////////////////////////////////////////////////////////////////
		//Add week profile 1.
	    wp = (gxWeekProfile*)malloc(sizeof(gxWeekProfile));
	    bb_init(&wp->name);
	    bb_addString(&wp->name, (char *)activity_calendar_params.weekProfile[0].weekProfileID);
	    wp->monday = wp->tuesday = wp->wednesday = wp->thursday = wp->friday = wp->saturday = wp->sunday = activity_calendar_params.weekProfile[0].monday[0];
	    arr_push(&activityCalendarActiveTime.weekProfileTablePassive, wp);
		//Add week profile 2.
	    wp = (gxWeekProfile*)malloc(sizeof(gxWeekProfile));
	    bb_init(&wp->name);
	    bb_addString(&wp->name, (char *)activity_calendar_params.weekProfile[1].weekProfileID);
	    wp->monday = wp->tuesday = wp->wednesday = wp->thursday = wp->friday = wp->saturday = wp->sunday = activity_calendar_params.weekProfile[0].monday[0];
	    arr_push(&activityCalendarActiveTime.weekProfileTablePassive, wp);
		//Add week profile 3.
	    wp = (gxWeekProfile*)malloc(sizeof(gxWeekProfile));
	    bb_init(&wp->name);
	    bb_addString(&wp->name, (char *)activity_calendar_params.weekProfile[2].weekProfileID);
	    wp->monday = wp->tuesday = wp->wednesday = wp->thursday = wp->friday = wp->saturday = wp->sunday = activity_calendar_params.weekProfile[0].monday[0];
	    arr_push(&activityCalendarActiveTime.weekProfileTablePassive, wp);
//		ARR_ATTACH(activityCalendarActiveTime.weekProfileTablePassive, ACTIVE_WEEK_PROFILE, 1);
//		bb_addString(&ACTIVE_WEEK_PROFILE[0].name,
//				(char *)activity_calendar_params.weekProfile[0].weekProfileID);
//		wp = &ACTIVE_WEEK_PROFILE[0];
//		wp->monday = wp->tuesday = wp->wednesday = wp->thursday = wp->friday = wp->saturday = wp->sunday = activity_calendar_params.weekProfile[0].monday[0];
//
//		bb_addString(&ACTIVE_WEEK_PROFILE[1].name,
//				(char *)activity_calendar_params.weekProfile[1].weekProfileID);
//		wp2 = &ACTIVE_WEEK_PROFILE[1];
//		wp2->monday = wp2->tuesday = wp2->wednesday = wp2->thursday = wp2->friday = wp2->saturday = wp2->sunday = activity_calendar_params.weekProfile[1].monday[0];
//
//		bb_addString(&ACTIVE_WEEK_PROFILE[2].name,
//				(char *)activity_calendar_params.weekProfile[2].weekProfileID);
//		wp3 = &ACTIVE_WEEK_PROFILE[2];
//		wp3->monday = wp3->tuesday = wp3->wednesday = wp3->thursday = wp3->friday = wp3->saturday = wp3->sunday = activity_calendar_params.weekProfile[2].monday[0];

		ret = com_write(con, BASE(activityCalendarActiveTime), 8);

		/////////////////////////////////////////////////////////////////////////
		//Add active day profiles. There are max three day profiles where one is in use.
		static uint16_t activePowerL1Value = 0;
		const unsigned char ln_pow[6] = { 1,1,21,25,0,255 };
	    INIT_OBJECT(activePowerL1, DLMS_OBJECT_TYPE_REGISTER, ln_pow);
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			activePowerL1.base.shortName = 0;
		}
	    //10 ^ 3 =  1000
	    GX_UINT16_BYREF(activePowerL1.value, activePowerL1Value);
	    activePowerL1.scaler = -2;
	    activePowerL1.unit = 30;
		const unsigned char ln[6] = { 0, 0, 14, 0, 1, 255 };
	    if ((ret = INIT_OBJECT(registerActivation, DLMS_OBJECT_TYPE_REGISTER_ACTIVATION, ln)) == 0)
	    {
			if ( 1 == dlms_client_get_short_name_ref() )
			{
				registerActivation.base.shortName = 0;
			}
	    	bb_init(&registerActivation.activeMask);
	        bb_addString(&registerActivation.activeMask, "RATE1");
	        oa_init(&registerActivation.registerAssignment);
	        oa_push(&registerActivation.registerAssignment, BASE(activePowerL1));

	        arr_init(&registerActivation.maskList);
	        gxByteBuffer name[1];// = malloc(sizeof(gxByteBuffer));
	        bb_init(name);
	        bb_addString(name, "RATE1");
	        gxByteBuffer indexes[1];// = malloc(sizeof(gxByteBuffer));
	        bb_init(indexes);
	        bb_setUInt8(indexes, 1);
	        arr_push(&registerActivation.maskList, key_init(name, indexes));
	        gxByteBuffer name1[1];
	        gxByteBuffer indexes1[1];
//	        name = malloc(sizeof(gxByteBuffer));
	        bb_init(name1);
	        bb_addString(name1, "RATE2");
//	        indexes = malloc(sizeof(gxByteBuffer));
	        bb_init(indexes1);
	        bb_setUInt8(indexes1, 1);
	        bb_setUInt8(indexes1, 2);
	        arr_push(&registerActivation.maskList, key_init(name1, indexes1));
	    }
		//0.0.10.0.100.255 Script Logical Name
		gxScriptTable tarifficationScriptTable;
		const unsigned char ln_script[6] = { 0, 0, 10, 0, 100, 255 };
		if ((ret = INIT_OBJECT(tarifficationScriptTable, DLMS_OBJECT_TYPE_SCRIPT_TABLE, ln_script)) == 0)
		{
			if ( 1 == dlms_client_get_short_name_ref() )
			{
				tarifficationScriptTable.base.shortName = 0;
			}
			gxScript s1[1];//= (gxScript*)malloc(sizeof(gxScript));
			gxScript s2[1];//= (gxScript*)malloc(sizeof(gxScript));
			s1->id = 1;
			arr_init(&s1->actions);
			s2->id = 2;
			arr_init(&s2->actions);
			arr_push(&tarifficationScriptTable.scripts, s1);
			arr_push(&tarifficationScriptTable.scripts, s2);
			gxScriptAction a[1] ;//= (gxScriptAction*)malloc(sizeof(gxScriptAction));
			arr_push(&s1->actions, a);
			a->type = DLMS_SCRIPT_ACTION_TYPE_EXECUTE;
			a->target = BASE(registerActivation);
			a->index = 1;
			var_init(&a->parameter);
			//Action data is Int8 zero.
			GX_INT8(a->parameter) = 0;

//			a = (gxScriptAction*)malloc(sizeof(gxScriptAction));
			arr_push(&s2->actions, a);
			a->type = DLMS_SCRIPT_ACTION_TYPE_EXECUTE;
			a->target = BASE(registerActivation);
			a->index = 1;
			var_init(&a->parameter);
			//Action data is Int8 zero.
			GX_INT8(a->parameter) = 0;
		}

		//Add day profile 1.
		dp = (gxDayProfile*)malloc(sizeof(gxDayProfile));
		arr_init(&dp->daySchedules);
		dp->dayId = activity_calendar_params.dayProfile[0].dayID[0];
		//Day Item 1.
		act = (gxDayProfileAction*)malloc(sizeof(gxDayProfileAction));
		__setStartTimeDayProfile(0, 0, &hour, &minute);
		time_init(&act->startTime,
				65535, 255, 255, hour, minute, 0, 255, -1);
		act->script         = BASE(tarifficationScriptTable);
		act->scriptSelector = activity_calendar_params.dayProfile[0].dayItem[0].scriptSelector;
		arr_push(&dp->daySchedules, act);
		//Day Item 2.
		act = (gxDayProfileAction*)malloc(sizeof(gxDayProfileAction));
		__setStartTimeDayProfile(0, 1, &hour, &minute);
		time_init(&act->startTime,
				65535, 255, 255, hour, minute, 0, 255, -1);
		act->script         = BASE(tarifficationScriptTable);
		act->scriptSelector = activity_calendar_params.dayProfile[0].dayItem[1].scriptSelector;
		arr_push(&dp->daySchedules, act);
		//Day Item 3.
		act = (gxDayProfileAction*)malloc(sizeof(gxDayProfileAction));
		__setStartTimeDayProfile(0, 2, &hour, &minute);
		time_init(&act->startTime,
				65535, 255, 255, hour, minute, 0, 255, -1);
		act->script = BASE(tarifficationScriptTable);
		act->scriptSelector = activity_calendar_params.dayProfile[0].dayItem[2].scriptSelector;
		arr_push(&dp->daySchedules, act);
		arr_push(&activityCalendarActiveTime.dayProfileTablePassive, dp);

		//Add day profile 2.
		dp = (gxDayProfile*)malloc(sizeof(gxDayProfile));
		arr_init(&dp->daySchedules);
		dp->dayId = activity_calendar_params.dayProfile[1].dayID[0];
		//Day Item 1.
		act = (gxDayProfileAction*)malloc(sizeof(gxDayProfileAction));
		__setStartTimeDayProfile(1, 0, &hour, &minute);
		time_init(&act->startTime,
				65535, 255, 255, hour, minute, 0, 255, -1);
		act->script         = BASE(tarifficationScriptTable);
		act->scriptSelector = activity_calendar_params.dayProfile[1].dayItem[0].scriptSelector;
		arr_push(&dp->daySchedules, act);
		//Day Item 2.
		act = (gxDayProfileAction*)malloc(sizeof(gxDayProfileAction));
		__setStartTimeDayProfile(1, 1, &hour, &minute);
		time_init(&act->startTime,
				65535, 255, 255, hour, minute, 0, 255, -1);
		act->script         = BASE(tarifficationScriptTable);
		act->scriptSelector = activity_calendar_params.dayProfile[1].dayItem[1].scriptSelector;
		arr_push(&dp->daySchedules, act);
		//Day Item 3.
		act = (gxDayProfileAction*)malloc(sizeof(gxDayProfileAction));
		__setStartTimeDayProfile(1, 2, &hour, &minute);
		time_init(&act->startTime,
				65535, 255, 255, hour, minute, 0, 255, -1);
		act->script         = BASE(tarifficationScriptTable);
		act->scriptSelector = activity_calendar_params.dayProfile[1].dayItem[2].scriptSelector;
		arr_push(&dp->daySchedules, act);
		arr_push(&activityCalendarActiveTime.dayProfileTablePassive, dp);

		ret = com_write(con, BASE(activityCalendarActiveTime), 9);

		activityCalendarActiveTime.time.value = __setActiveTime();

		ret = com_write(con, BASE(activityCalendarActiveTime), 10);

//		dlmsVARIANT   params;
//		params.vt   = DLMS_DATA_TYPE_UINT8;
//		params.bVal = 0;
//		uint32_t ret_ = com_method(con, BASE(activityCalendarActiveTime), 1, &params);
//		UNUSED(ret_);

		free(wp);
		free(sp);
		free(dp);
		free(act);

		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(activity_calendar_string, "%s;%s;#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "OK");
			dlms_client_set_dlms_data_cmd_on_demand(activity_calendar_string);
		}
		else
		{
			sprintf(activity_calendar_string, "%s;%s;#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(activity_calendar_string);
		}
		obj_clear(BASE(registerActivation));
		obj_clear(BASE(tarifficationScriptTable));
//		obj_clear(BASE(actionSchedule));
		obj_clear(BASE(activityCalendarActiveTime));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_get_billing_date_sched(connection* con)
{
	int                 ret;
	gxActionSchedule    action_schedule;
    //Action schedule execution times.
    static gxtime *tm;
	const unsigned char ln[6] = { 0, 0, 15, 0, 0, 255 };
	char act_sched_string[400];
    int pos = 0;
    gxByteBuffer ba;
    BYTE_BUFFER_INIT(&ba);

//    dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(action_schedule, DLMS_OBJECT_TYPE_ACTION_SCHEDULE, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			action_schedule.base.shortName = 0xc670;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(action_schedule), 4);
	    for (pos = 0; pos != action_schedule.executionTime.size; ++pos)
	    {
	        ret = arr_getByIndex(&action_schedule.executionTime, pos, (void**)&tm);
	        if (ret != DLMS_ERROR_CODE_OK)
	        {
	            return ret;
	        }
	        else
	        {
	            if (pos != 0)
	            {
	                bb_addString(&ba, ", ");
	            }
	            ret = time_toString(tm, &ba);
	            if (ret != 0)
	            {
	                return ret;
	            }
	        }
	    }
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(act_sched_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), bb_toString(&ba));
			dlms_client_set_dlms_data_cmd_on_demand(act_sched_string);
		}
		else
		{
			sprintf(act_sched_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(act_sched_string);
		}
		obj_clear(BASE(action_schedule));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_comm_clear_alarms(connection* con)
{
    int      ret;
    gxData   eventalarm;
    char     eventalarm_string[200];
    uint32_t str_len = 0;
    const unsigned char ln_ev1[6] = { 0, 0, 97, 97, 0, 255 };
    const unsigned char ln_ev2[6] = { 0, 0, 97, 97, 1, 255 };
    const unsigned char ln_al1[6] = { 0, 0, 97, 98, 0, 255 };
    const unsigned char ln_al2[6] = { 0, 0, 97, 98, 1, 255 };

//    dlms_client_clear_dlms_data_cmd_on_demand();

    if ((ret = INIT_OBJECT(eventalarm, DLMS_OBJECT_TYPE_DATA, ln_ev1)) == 0)
    {
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			eventalarm.base.shortName = 0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT32(eventalarm.value) = 0;
    	ret = com_write(con, BASE(eventalarm), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(eventalarm_string, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Cleared");
		}
		else
		{
			sprintf(eventalarm_string, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
		}
		str_len = strlen(eventalarm_string);
		obj_clear(BASE(eventalarm));
    }
    if ((ret = INIT_OBJECT(eventalarm, DLMS_OBJECT_TYPE_DATA, ln_ev2)) == 0)
    {
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			eventalarm.base.shortName = 0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT32(eventalarm.value) = 0;
    	ret = com_write(con, BASE(eventalarm), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(eventalarm_string + str_len, ";%s", "Cleared");
		}
		else
		{
			sprintf(eventalarm_string + str_len, ";%s", "Fail");
		}
		str_len = strlen(eventalarm_string);
		obj_clear(BASE(eventalarm));
    }
    if ((ret = INIT_OBJECT(eventalarm, DLMS_OBJECT_TYPE_DATA, ln_al1)) == 0)
    {
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			eventalarm.base.shortName = 0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT32(eventalarm.value) = 0;
    	ret = com_write(con, BASE(eventalarm), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(eventalarm_string + str_len, ";%s", "Cleared");
		}
		else
		{
			sprintf(eventalarm_string + str_len, ";%s", "Fail");
		}
		str_len = strlen(eventalarm_string);
		obj_clear(BASE(eventalarm));
    }
    if ((ret = INIT_OBJECT(eventalarm, DLMS_OBJECT_TYPE_DATA, ln_al2)) == 0)
    {
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			eventalarm.base.shortName = 0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT32(eventalarm.value) = 0;
    	ret = com_write(con, BASE(eventalarm), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(eventalarm_string + str_len, ";%s#", "Cleared");
		}
		else
		{
			sprintf(eventalarm_string + str_len, ";%s#", "Fail");
		}
		obj_clear(BASE(eventalarm));
    }
    dlms_client_set_dlms_data_cmd_on_demand(eventalarm_string);
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_read_clock(connection* con)
{
	int          ret;
	gxClock      clk;
	const unsigned char ln[6] = { 0, 0, 1, 0, 0, 255 };
	char time_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(clk, DLMS_OBJECT_TYPE_CLOCK, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			clk.base.shortName = 0x2BC0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(clk), 2);

		uint32_t meter_clk_obis_value = clk.time.value;
		con_dlms_getDateFormat(&meter_clk_obis_value);
		clk.time.value = meter_clk_obis_value;

		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(time_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)clk.time.value);
			dlms_client_set_dlms_data_cmd_on_demand(time_string);
		}
		else
		{
			sprintf(time_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(time_string);
		}
		obj_clear(BASE(clk));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_read_name_meter_plate_info(connection* con)
{
	int          ret;
	gxObject     meterID;
	gxData       obj;
	char 		 meterid_string[200];
	uint32_t     str_len = 0;

//	dlms_client_clear_dlms_data_cmd_on_demand();
	if ( ( ret = cosem_init(&meterID, DLMS_OBJECT_TYPE_DATA, "0.0.96.1.1.255") ) == 0 )
	{
		dlms_client_reset_indexes();
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			meterID.shortName = 0x5ab8;
		}
		HAL_IWDG_Refresh(&hiwdg);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, &meterID, 2);
		if (ret != 0)
		{
			dlms_client_reset_indexes();
			obj_clear(&meterID);
			cosem_init(&meterID, DLMS_OBJECT_TYPE_DATA, "1.0.96.1.1.255");
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, &meterID, 2);
		}
		char *dlms_meter_ID = dlms_client_get_obis_value_next();
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(meterid_string, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), dlms_meter_ID);
		}
		else
		{
			sprintf(meterid_string, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
		}
		str_len = strlen(meterid_string);
		obj_clear(&meterID);
	}
	else
	{
		sprintf(meterid_string, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
		str_len = strlen(meterid_string);
	}
	if ( ( ret = cosem_init(&meterID, DLMS_OBJECT_TYPE_DATA, "0.0.96.1.0.255") ) == 0 )
	{
		dlms_client_reset_indexes();
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			meterID.shortName = 0x5a80;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		HAL_IWDG_Refresh(&hiwdg);
		ret = com_read(con, &meterID, 2);
		if (ret != 0)
		{
			dlms_client_reset_indexes();
			obj_clear(&meterID);
			cosem_init(&meterID, DLMS_OBJECT_TYPE_DATA, "1.0.96.1.0.255");
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, &meterID, 2);
		}
		char *dlms_meter_ID = dlms_client_get_obis_value_next();
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(meterid_string + str_len, ";%s", dlms_meter_ID);
		}
		else
		{
			sprintf(meterid_string + str_len, ";%s", "Fail");
		}
		str_len = strlen(meterid_string);
		obj_clear(&meterID);
	}
	else
	{
		sprintf(meterid_string + str_len, ";%s", "Fail");
		str_len = strlen(meterid_string);
	}
#define OBIS_NUM (5)
	uint32_t i = 0;
	const unsigned char meter_data[OBIS_NUM][6] = { { 1, 0, 0, 4, 2, 255 }, { 1, 0, 0, 4, 5, 255 }, { 1, 0, 0, 4, 3, 255 }, { 1, 0, 0, 4, 6, 255 }, { 1, 0, 0, 2, 0, 255 }};
	const uint32_t meter_short_name[OBIS_NUM]   = { 0x5f18, 0, 0x5f80, 0, 0xff00 };
	for (i = 0; i < OBIS_NUM; i++)
	{
		if ( ( ret = INIT_OBJECT(obj, DLMS_OBJECT_TYPE_DATA, meter_data[i]) ) == 0 )
		{
			dlms_client_reset_indexes();
			if ( 1 == dlms_client_get_short_name_ref() )
			{
				obj.base.shortName = meter_short_name[i];
			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
				HAL_IWDG_Refresh(&hiwdg);
			}
			ret = com_read(con, BASE(obj), 2);
			if ( (ret != 0) && (i == 0) )
			{
				dlms_client_reset_indexes();
				obj_clear(BASE(obj));
				cosem_init(BASE(obj), DLMS_OBJECT_TYPE_DATA, "1.1.0.4.2.255");
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, BASE(obj), 2);
			}
			else if ( (ret != 0) && (i == 2) )
			{
				dlms_client_reset_indexes();
				obj_clear(BASE(obj));
				cosem_init(BASE(obj), DLMS_OBJECT_TYPE_DATA, "1.1.0.4.3.255");
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, BASE(obj), 2);
			}
			else if ( (ret != 0) && (i == 4) )
			{
				dlms_client_reset_indexes();
				obj_clear(BASE(obj));
				cosem_init(BASE(obj), DLMS_OBJECT_TYPE_DATA, "1.0.96.1.5.255");
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, BASE(obj), 2);
			}
			char *obis = dlms_client_get_obis_value_next();
			if (ret == DLMS_ERROR_CODE_OK)
			{
				sprintf(meterid_string + str_len, ";%s", obis);
			}
			else
			{
				sprintf(meterid_string + str_len, ";%s", "Fail");
			}
			str_len = strlen(meterid_string);
			obj_clear(BASE(obj));
		}
	}
	gxIecHdlcSetup IecHdlcSetup;
	const unsigned char   ln2[6] = { 0, 0, 22, 0, 0, 255 };
	if ( ( ret = INIT_OBJECT(IecHdlcSetup, DLMS_OBJECT_TYPE_IEC_HDLC_SETUP, ln2) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
			HAL_IWDG_Refresh(&hiwdg);
		}
		ret = com_read(con, BASE(IecHdlcSetup), 9);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(meterid_string + str_len, ";%d", (int)IecHdlcSetup.deviceAddress);
		}
		else
		{
			sprintf(meterid_string + str_len, ";%s", "Fail");
		}
		str_len = strlen(meterid_string);
		obj_clear(BASE(IecHdlcSetup));
	}
	else
	{
		sprintf(meterid_string + str_len, ";%s", "Fail");
	}

	const unsigned char meter_data_manufacturer[6] = { 0, 0, 42, 0, 0, 255 };
	if ( ( ret = INIT_OBJECT(obj, DLMS_OBJECT_TYPE_DATA, meter_data_manufacturer) ) == 0 )
	{
		dlms_client_reset_indexes();
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			obj.base.shortName = 0x5a80;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
			HAL_IWDG_Refresh(&hiwdg);
		}
		ret = com_read(con, BASE(obj), 2);
		char *obis = dlms_client_get_obis_value_next();
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(meterid_string + str_len, ";%s", obis);
		}
		else
		{
			sprintf(meterid_string + str_len, ";%s", "Fail");
		}
		str_len = strlen(meterid_string);
		obj_clear(BASE(obj));
	}
	sprintf(meterid_string + str_len, "%s","#");
	dlms_client_set_dlms_data_cmd_on_demand(meterid_string);
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_get_voltage_range_low(connection* con)
{
	int          ret;
	gxRegister   volt_low;
	const unsigned char   ln[6] = { 1, 0, 12, 31, 0, 255 };
	char         volt_low_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_low, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(volt_low), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_low_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_low.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		else
		{
			sprintf(volt_low_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		obj_clear(BASE(volt_low));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_get_voltage_range_up(connection* con)
{
	int          ret;
	gxRegister   volt_up;
	const unsigned char   ln[6] = { 1, 0, 12, 35, 0, 255 };
	char         volt_up_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_up, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(volt_up), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_up_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_up.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		else
		{
			sprintf(volt_up_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		obj_clear(BASE(volt_up));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_set_voltage_range_low(connection* con)
{
	int          ret;
	gxRegister   volt_low;
	const unsigned char   ln[6] = { 1, 0, 12, 31, 0, 255 };
	char         volt_low_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_low, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
//		volt_low.value.iVal = volt_range;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		GX_UINT16(volt_low.value) = volt_range;
		ret = com_write(con, BASE(volt_low), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_low_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_low.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		else
		{
			sprintf(volt_low_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		obj_clear(BASE(volt_low));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_set_voltage_range_up(connection* con)
{
	int          ret;
	gxRegister   volt_up;
	const unsigned char   ln[6] = { 1, 0, 12, 35, 0, 255 };
	char         volt_up_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_up, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
//		volt_up.value.iVal = volt_range;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		GX_UINT16(volt_up.value) = volt_range;
		ret = com_write(con, BASE(volt_up), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_up_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_up.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		else
		{
			sprintf(volt_up_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		obj_clear(BASE(volt_up));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_get_current_range_low(connection* con)
{
	int          ret;
	gxRegister   volt_low;
	const unsigned char   ln[6] = { 1, 0, 11, 31, 0, 255 };
	char         volt_low_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_low, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(volt_low), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_low_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_low.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		else
		{
			sprintf(volt_low_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		obj_clear(BASE(volt_low));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_get_current_range_up(connection* con)
{
	int          ret;
	gxRegister   volt_up;
	const unsigned char   ln[6] = { 1, 0, 11, 35, 0, 255 };
	char         volt_up_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_up, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(volt_up), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_up_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_up.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		else
		{
			sprintf(volt_up_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		obj_clear(BASE(volt_up));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_set_current_range_low(connection* con)
{
	int          ret;
	gxRegister   volt_low;
	const unsigned char   ln[6] = { 1, 0, 11, 31, 0, 255 };
	char         volt_low_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_low, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
//		volt_low.value.iVal = curr_range;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		GX_UINT16(volt_low.value) = curr_range;
		ret = com_write(con, BASE(volt_low), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_low_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_low.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		else
		{
			sprintf(volt_low_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_low_string);
		}
		obj_clear(BASE(volt_low));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_set_current_range_up(connection* con)
{
	int          ret;
	gxRegister   volt_up;
	const unsigned char   ln[6] = { 1, 0, 11, 35, 0, 255 };
	char         volt_up_string[200];

//	dlms_client_clear_dlms_data_cmd_on_demand();

	if ( ( ret = INIT_OBJECT(volt_up, DLMS_OBJECT_TYPE_REGISTER, ln) ) == 0 )
	{
//		volt_up.value.iVal = curr_range;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		GX_UINT16(volt_up.value) = curr_range;
		ret = com_write(con, BASE(volt_up), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(volt_up_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)volt_up.value.iVal);
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		else
		{
			sprintf(volt_up_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(volt_up_string);
		}
		obj_clear(BASE(volt_up));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_comand_set_demand_integration_period(connection* con)
{
    int ret;
    //1.0.0.8.0.255 Demand Integration Period
    gxData demandIntegrationPeriod;
    const unsigned char ln[6] = { 1, 0, 0, 8, 0, 255 };
    char         demand_integration[200];

//    dlms_client_clear_dlms_data_cmd_on_demand();

    if ((ret = INIT_OBJECT(demandIntegrationPeriod, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
//    	ret = com_read(con, BASE(demandIntegrationPeriod), 2);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT32(demandIntegrationPeriod.value) = demand_integration_period;
		ret = com_write(con, BASE(demandIntegrationPeriod), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(demand_integration, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)demandIntegrationPeriod.value.lVal);
			dlms_client_set_dlms_data_cmd_on_demand(demand_integration);
		}
		else
		{
			sprintf(demand_integration, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(demand_integration);
		}
		obj_clear(BASE(demandIntegrationPeriod));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_comand_get_demand_integration_period(connection* con)
{
    int ret;
    //1.0.0.8.0.255 Demand Integration Period
    gxData demandIntegrationPeriod;
    const unsigned char ln[6] = { 1, 0, 0, 8, 0, 255 };
    char         demand_integration[200];

//    dlms_client_clear_dlms_data_cmd_on_demand();

    if ((ret = INIT_OBJECT(demandIntegrationPeriod, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
//        GX_UINT16(demandIntegrationPeriod.value) = demand_integration_period;
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	ret = com_read(con, BASE(demandIntegrationPeriod), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(demand_integration, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)demandIntegrationPeriod.value.lVal);
			dlms_client_set_dlms_data_cmd_on_demand(demand_integration);
		}
		else
		{
			sprintf(demand_integration, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(demand_integration);
		}
		obj_clear(BASE(demandIntegrationPeriod));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_comand_set_payment_mode(connection* con)
{
    int ret;
    //1.0.0.8.0.255 Demand Integration Period
    gxData paymentMode;
    const unsigned char ln[6] = { 0, 0, 128, 60, 0, 255 };
    char         payment_mode_str[200];
    static char  *payment_strings[ 2 ] = {"Postpaid", "Prepaid"};

    if ((ret = INIT_OBJECT(paymentMode, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT8(paymentMode.value) = payment_mode;
		ret = com_write(con, BASE(paymentMode), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(payment_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), payment_strings[payment_mode]);
			dlms_client_set_dlms_data_cmd_on_demand(payment_mode_str);
		}
		else
		{
			sprintf(payment_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(payment_mode_str);
		}
		obj_clear(BASE(paymentMode));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_comand_get_payment_mode(connection* con)
{
    int ret;
    //1.0.0.8.0.255 Demand Integration Period
    gxData paymentMode;
    const unsigned char ln[6] = { 0, 0, 128, 60, 0, 255 };
    char         payment_mode_str[200];
    static char  *payment_strings[ 2 ] = {"Postpaid", "Prepaid"};

    if ((ret = INIT_OBJECT(paymentMode, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT8(paymentMode.value) = payment_mode;
		ret = com_read(con, BASE(paymentMode), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(payment_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), payment_strings[paymentMode.value.lVal]);
			dlms_client_set_dlms_data_cmd_on_demand(payment_mode_str);
		}
		else
		{
			sprintf(payment_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(payment_mode_str);
		}
		obj_clear(BASE(paymentMode));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_comand_set_metering_mode(connection* con)
{
    int ret;
    //1.0.0.8.0.255 Demand Integration Period
    gxData meteringMode;
    const unsigned char ln[6] = { 0, 0, 94, 96, 19, 255 };
    char         metering_mode_str[200];

    if ((ret = INIT_OBJECT(meteringMode, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT16(meteringMode.value) = metering_mode;
		ret = com_write(con, BASE(meteringMode), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(metering_mode_str, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)meteringMode.value.lVal);
			dlms_client_set_dlms_data_cmd_on_demand(metering_mode_str);
		}
		else
		{
			sprintf(metering_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(metering_mode_str);
		}
		obj_clear(BASE(meteringMode));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_comand_get_metering_mode(connection* con)
{
    int ret;
    gxData meteringMode;
    const unsigned char ln[6] = { 0, 0, 94, 96, 19, 255 };
    char         metering_mode_str[200];
    static char  *metering_strings[ 2 ] = {"Bidirectional", "Unidirectional"};

    if ((ret = INIT_OBJECT(meteringMode, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
    	GX_UINT16(meteringMode.value) = metering_mode;
		ret = com_read(con, BASE(meteringMode), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(metering_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), metering_strings[meteringMode.value.lVal]);
			dlms_client_set_dlms_data_cmd_on_demand(metering_mode_str);
		}
		else
		{
			sprintf(metering_mode_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(metering_mode_str);
		}
		obj_clear(BASE(meteringMode));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}

uint32_t dlms_client_on_demand_comand_get_curr_active_tariff(connection* con)
{
    int                 ret;
    gxActivityCalendar  activityCalendarActiveTime1;
    gxData              activeTariff;
    const unsigned char ln[6] = { 0, 0, 96, 14, 0, 255 };
    char                active_tariff[200];
    size_t str_len;

	sprintf(active_tariff, "%s", dlms_client_get_meter_id(con_dlms_get_curr_device()));
	str_len = strlen(active_tariff);

	const unsigned char ln1[6] = { 0, 0, 13, 0, 0, 255 };
	if ( ( ret = INIT_OBJECT(activityCalendarActiveTime1, DLMS_OBJECT_TYPE_ACTIVITY_CALENDAR, ln1) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			activityCalendarActiveTime1.base.shortName = 0xC738;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(activityCalendarActiveTime1), 2);

		if (ret == DLMS_ERROR_CODE_OK)
		{
			char * active_calendar_name = bb_toString(&activityCalendarActiveTime1.calendarNameActive);
			sprintf(active_tariff + str_len, ";%s", active_calendar_name);
			gxfree(active_calendar_name);
		}
		else
		{
			sprintf(active_tariff, ";%s", "Fail");
		}
		str_len = strlen(active_tariff);
		obj_clear(BASE(activityCalendarActiveTime1));
	}
    if ((ret = INIT_OBJECT(activeTariff, DLMS_OBJECT_TYPE_DATA, ln)) == 0)
    {
//    	dlms_client_reset_indexes();
    	ret = com_read(con, BASE(activeTariff), 2);
//		char *obis = dlms_client_get_obis_value_next();
		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(active_tariff + str_len, ";%s#", activeTariff.value.strVal->data);
		}
		else
		{
			sprintf(active_tariff + str_len, ";%s#", "Fail");
		}
		obj_clear(BASE(activeTariff));
    }
    con->state = DLMS_STATE_DISCONNECT;
    dlms_client_set_dlms_data_cmd_on_demand(active_tariff);
    return ret;
}

uint32_t dlms_client_on_demand_comand_get_meter_status(connection* con)
{
#define NOT_AVAILABLE_FAIL (3)
	int          ret;
	gxData       obj;
	const unsigned char meter_data[5][6] = { { 0, 0, 96, 1, 1, 255 }, { 0, 0, 96, 3, 0, 255 } ,
			                                 { 0, 0, 96, 3, 1, 255 } , { 0, 0, 96, 3, 2, 255 } ,
											 { 0, 0, 96, 14, 0, 255 }};
//	int                 mt372 = 0;
	gxDisconnectControl dc;
//	dlmsVARIANT         params;
	const unsigned char ln[6] = { 0,0,96,3,10,255 };
//	char                reconnection_string[200];
	static char         *state_strings[ 4 ] = {"disconnected", "connected", "Ready_for_reconnection", "Fail"};

    gxData disconnect_state;//, disconnect_mode;
//    const unsigned char ln_disc_mode[6]  = { 0, 0, 128, 30, 20, 255 };
    const unsigned char ln_disc_state[6] = { 0, 0, 128, 30, 21, 255 };

	uint8_t            outputState;
//	DLMS_CONTROL_STATE controlState;
//	DLMS_CONTROL_MODE  controlMode;
	char 		 meterid_string[300];
//	uint32_t i = 0;
	uint32_t str_len = 0;

	sprintf(meterid_string, "%s", dlms_client_get_meter_id(con_dlms_get_curr_device()));
	str_len = strlen(meterid_string);

//	for (i = 0; i < 4; i++)
//	{
//		if ( ( ret = INIT_OBJECT(obj, DLMS_OBJECT_TYPE_DATA, meter_data[i]) ) == 0 )
//		{
//			if ( 1 == dlms_client_get_short_name_ref() )
//			{
////				meterID.shortName = 0xFD00;
//			}
//			ret = com_read(con, BASE(obj), 2);
//			char *obis = dlms_client_get_obis_value_next();
//			if (ret == DLMS_ERROR_CODE_OK)
//			{
//				sprintf(meterid_string + str_len, ";%s",  obis);
//			}
//			else
//			{
//				sprintf(meterid_string + str_len, ";%s", "Fail");
//			}
//			str_len = strlen(meterid_string);
//			obj_clear(BASE(obj));
//		}
//	}
	if ( ( ret = INIT_OBJECT(dc, DLMS_OBJECT_TYPE_DISCONNECT_CONTROL, ln) ) == 0 )
	{
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		HAL_IWDG_Refresh(&hiwdg);
		ret = com_read(con, BASE(dc), 2);

		if (ret == DLMS_ERROR_CODE_OK)
		{
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, BASE(dc), 3);
			HAL_IWDG_Refresh(&hiwdg);

			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, BASE(dc), 4);
			HAL_IWDG_Refresh(&hiwdg);
		}

		if  (ret != 0)
		{
		    if ((ret = INIT_OBJECT(disconnect_state, DLMS_OBJECT_TYPE_DATA, ln_disc_state)) == 0)
		    {
//		    	mt372 = 1;

				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				HAL_IWDG_Refresh(&hiwdg);
				dlms_client_reset_indexes();
				ret				 = com_read(con, BASE(disconnect_state), 2);
				char *obis_state = dlms_client_get_obis_value_next();
				if (ret == DLMS_ERROR_CODE_OK)
				{
					dc.outputState   = outputState  = atoi(obis_state);
//					dc.controlState  = controlState = atoi(obis_state);
				}
				else
				{
					dc.outputState = outputState = NOT_AVAILABLE_FAIL;
				}
		    }
		}
		sprintf(meterid_string + str_len, ";%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), state_strings[dc.outputState]);
		str_len = strlen(meterid_string);
		obj_clear(BASE(dc));
	}
	if ( ( ret = INIT_OBJECT(obj, DLMS_OBJECT_TYPE_DATA, meter_data[4]) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
//				obj.base.shortName = 0xc738;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		HAL_IWDG_Refresh(&hiwdg);
		ret = com_read(con, BASE(obj), 2);
		if (ret == DLMS_ERROR_CODE_OK)
		{
//			sprintf(meterid_string + str_len, ";%s",  obis);
			sprintf(meterid_string + str_len, ";%d%d", obj.value.strVal->data[0], obj.value.strVal->data[1]);
		}
		else
		{
			sprintf(meterid_string + str_len, ";%s", "Fail");
		}
		str_len = strlen(meterid_string);
		obj_clear(BASE(obj));
	}
	gxRegister batteryStatus;
	const unsigned char batt[6] = { 0, 0, 96, 6, 6, 255 };
    INIT_OBJECT(batteryStatus, DLMS_OBJECT_TYPE_REGISTER, batt);
	if ( 1 == dlms_client_get_short_name_ref() )
	{
		batteryStatus.base.shortName = 0x8b90;
	}
	if ( 1 == params_get_slow_meter() )
	{
		HAL_Delay(1000);
	}
	HAL_IWDG_Refresh(&hiwdg);
	ret = com_read(con, BASE(batteryStatus), 2);
	if (ret != DLMS_ERROR_CODE_OK)
	{
		dlms_client_reset_indexes();
		obj_clear(BASE(batteryStatus));
		cosem_init(BASE(batteryStatus), DLMS_OBJECT_TYPE_DATA, "1.1.96.56.255.255");
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		HAL_IWDG_Refresh(&hiwdg);
		ret = com_read(con, BASE(batteryStatus), 2);
	}
	if (ret == DLMS_ERROR_CODE_OK)
	{
		sprintf(meterid_string + str_len, ";%d",  (int)batteryStatus.value.ulVal);
	}
	else
	{
		sprintf(meterid_string + str_len, ";%s", "Fail");
	}
	str_len = strlen(meterid_string);
	obj_clear(BASE(batteryStatus));

	sprintf(meterid_string + str_len, "%s", "#");
	dlms_client_set_dlms_data_cmd_on_demand(meterid_string);
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_meter_ping(connection* con)
{
	int          ret;
	gxClock      clk;
	const unsigned char ln[6] = { 0, 0, 1, 0, 0, 255 };
	char time_string[200];

	if ( ( ret = INIT_OBJECT(clk, DLMS_OBJECT_TYPE_CLOCK, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			clk.base.shortName = 0x2BC0;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(clk), 2);

		uint32_t meter_clk_obis_value = clk.time.value;
		con_dlms_getDateFormat(&meter_clk_obis_value);
		clk.time.value = meter_clk_obis_value;

		if (ret == DLMS_ERROR_CODE_OK)
		{
			sprintf(time_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)clk.time.value);
			dlms_client_set_dlms_data_cmd_on_demand(time_string);
		}
		else
		{
			sprintf(time_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(time_string);
		}
		obj_clear(BASE(clk));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

uint32_t dlms_client_on_demand_comm_synchronize(connection* con)
{
	int          ret;
	gxClock      clk;
	uint32_t     new_clk;
	const unsigned char ln[6] = { 0, 0, 1, 0, 0, 255 };
	char time_string[200];

	if ( 1 == params_get_slow_meter() )
	{
		HAL_Delay(1000);
	}

	if ( ( ret = INIT_OBJECT(clk, DLMS_OBJECT_TYPE_CLOCK, ln) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			clk.base.shortName = 0x2BC0;
		}
		ret = com_read(con, BASE(clk), 2);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Meter Time Read:%d\n", (int)Tick_Get(SECONDS), (int)clk.time.value);
		uint32_t meter_clk_obis_value = clk.time.value;
		uint32_t format_myd = con_dlms_getDateFormat(&meter_clk_obis_value);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		meter_clk_obis_value = clk.time.value = dlms_client_get_synch_time();
//		con_dlms_setDateFormat(&meter_clk_obis_value);
//		clk.time.value = meter_clk_obis_value;
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Meter Time Write:%d\n", (int)Tick_Get(SECONDS), (int)clk.time.value);
		ret = com_write(con, BASE(clk), 2);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		new_clk = clk.time.value;
		ret     = com_read(con, BASE(clk), 2);
		uint32_t meter_clk_check_obis_value = clk.time.value;
		if ( 1 == format_myd )
		{
//			if ( 0 == con_dlms_getDateFormat(&meter_clk_check_obis_value) )
//			{
//				new_clk = dlms_client_get_synch_time();
//			}
			con_dlms_setDateFormat(&meter_clk_check_obis_value);
			clk.time.value = meter_clk_check_obis_value;
		}
		else
		{
			if ( 1 == con_dlms_getDateFormat(&meter_clk_check_obis_value) )
			{
				clk.time.value = meter_clk_check_obis_value;
			}
		}
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Meter Time Read :%d\n", (int)Tick_Get(SECONDS), (int)clk.time.value);
//		if ((new_clk == clk.time.value) || (new_clk >= (clk.time.value - 2)))
//		{
//			__NOP();
//		}
//		else
//		{
//			shutdown_set_tamper_param(TAMPER_MQTT_COMMAND_ERROR);
//		}
//		if ((new_clk == clk.time.value) || (new_clk >= (clk.time.value - 2)))
		if (( new_clk == (clk.time.value) ) || (( new_clk >= ( clk.time.value - 5 ) ) && ( new_clk <= ( clk.time.value + 5 ) )))
		{
			sprintf(time_string, "%s;%d#", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)clk.time.value);
			dlms_client_set_dlms_data_cmd_on_demand(time_string);
		}
		else
		{
			sprintf(time_string, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(time_string);
		}
		obj_clear(BASE(clk));
	}
	con->state = DLMS_STATE_DISCONNECT;

	return ret;
}

static uint32_t __attribute__((optimize("O0"))) __getDate(char *data_in, char * date)
{
	char temp_year[5] = {'2','0','2','3'};
	int val;
	size_t len  = sizeof(date);

	memset(date, 0, 100 * sizeof(char));

	val       = data_in[0];
	val       = val<<8;
	val      |= data_in[1];
	itoa(val, temp_year, 10);

	//Day.
	val       = data_in[2];
	itoa(val, date, 10);
	len       = strlen(date);
	date[len] = '/';
	len      += 1;
	//Month.
	val       = data_in[3];
	itoa(val, date + len, 10);
	len       = strlen(date);
	date[len] = '/';
	len      += 1;
	//Year.
	memcpy(date + len, temp_year, 4);
	len       = strlen(date);
	date[len] = ' ';
	len      += 1;

	val = data_in[5];
	itoa(val, date + len, 10);
	len = strlen(date);
	date[len] = ':';
	len += 1;
	val = data_in[6];
	itoa(val, date + len, 10);
	len = strlen(date);
	date[len] = ':';
	len += 1;
	val = data_in[7];
	itoa(val, date + len, 10);

	return 0;
}

static uint32_t __checkMaxDemand( uint32_t n_obis)
{
	char     *str;
	char     dlms_obis[24];
	int      max_demand_obis = 0;
	uint32_t ret             = 0;
	char     obisdata_ln[8]  = {'1', '.', '0', '.', '1', '3', '.', '5'};

	if (0 == memcmp(dlms_client_get_obis_n(n_obis), obisdata_ln, 8))
	{
		__NOP();
		ret = 1;
		return ret;
	}
	memcpy(dlms_obis, dlms_client_get_obis_n(n_obis), 24*sizeof(char));

    str = strtok(dlms_obis, ".");
    if (str != NULL)
    str = strtok(NULL, ".");
    if (str != NULL)
    str = strtok(NULL, ".");
    if (str != NULL)
    str = strtok(NULL, ".");

    if (str != NULL)
    max_demand_obis = atoi(str);

    if (max_demand_obis == 6)
    {
    	ret = 1;
    }

    return ret;
}

//uint32_t n_obis = 0;
uint32_t dlms_client_on_demand_comm_obis_profile(connection *con)
{
	gxObject obj;
	uint32_t ret = 0, n_obis = 0, typedata = 0;
	char obisdata_ln[6] = {'0', '.', '0', '.', '9', '6'};
	do
	{
		dlms_log_set_profile_max_demand_extra(0);
		if ( ( DLMS_READ_MAX_DEMAND_PROF_1 == dlms_client_get_obis_profile_frame_type()) || ( 1 == __checkMaxDemand(n_obis) ) )
		{
			if ( DLMS_READ_MAX_DEMAND_PROF_1 != dlms_client_get_obis_profile_frame_type())
			{
				dlms_log_set_profile_max_demand_extra(1);
				if (0 == dlms_client_get_obis_profile_max_demand_profile_extra())
				{
					dlms_client_set_obis_profile_max_demand_profile_extra(1);
					dlms_client_table_write_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
					dlms_client_table_read_client_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				}
			}
			else
			{
				dlms_log_set_profile_max_demand_extra(0);
				if (dlms_client_get_obis_profile_max_demand_profile_extra() != 0)
				{
					dlms_client_set_obis_profile_max_demand_profile_extra(0);
					dlms_client_table_write_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
					dlms_client_table_read_client_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				}
			}
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_OBJECT_TYPE_EXTENDED_REGISTER.\r\n", (int)Tick_Get(SECONDS));
			ret = cosem_init(&obj, DLMS_OBJECT_TYPE_EXTENDED_REGISTER, dlms_client_get_obis_n(n_obis++));//"1.0.32.7.0.255");
		}
		else if (( DLMS_READ_INST_PROF_4 == dlms_client_get_obis_profile_frame_type()) && (0 == memcmp(dlms_client_get_obis_n(n_obis), obisdata_ln, 6)))
		{
				typedata = 1;
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_OBJECT_TYPE_DATA.\r\n", (int)Tick_Get(SECONDS));
				ret = cosem_init(&obj, DLMS_OBJECT_TYPE_DATA, dlms_client_get_obis_n(n_obis++));//"1.0.32.7.0.255");
		}
		else
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_OBJECT_TYPE_REGISTER.\r\n", (int)Tick_Get(SECONDS));
			ret = cosem_init(&obj, DLMS_OBJECT_TYPE_REGISTER, dlms_client_get_obis_n(n_obis++));//"1.0.32.7.0.255");
		}
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			obj.shortName = con_get_shortName(n_obis - 1);
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> com_read 2.\r\n", (int)Tick_Get(SECONDS));
		ret = com_read(con, &obj, 2);
		if ( 1 == typedata )
		{
			char obisdata_ln[11] = {'0', '.', '0', '.', '9', '6', '.', '7', '.', '1', '0'};
			if (0 == memcmp(dlms_client_get_obis_n(n_obis-1), obisdata_ln, 11))
			{
		    	char  obis_trans[100];
		    	gxData * object = (gxData *)&obj;
		        __getDate((char *)object->value.strVal->data, obis_trans);
		    	dlms_client_dec_next_obis_val();
		    	dlms_client_append_obis_value(obis_trans, 2);
			}
		}
		if (ret != DLMS_ERROR_CODE_OK)
		{
			obj_clear(&obj);
			retry++;
			con->state = DLMS_STATE_ERROR;
//			return ret;
			continue;
		}
		HAL_IWDG_Refresh(&hiwdg);

		//	LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> com_read 3\n", (int)Tick_Get(SECONDS));
		if ( 0 == typedata )
		{
			if (( DLMS_READ_MAX_DEMAND_PROF_1 == dlms_client_get_obis_profile_frame_type()) || ( 1 == __checkMaxDemand(n_obis - 1) ))
			{
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, &obj, 5);
				dlms_client_dec_next_obis_val();

				HAL_IWDG_Refresh(&hiwdg);
			}
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			ret = com_read(con, &obj, 3);
			if (ret != DLMS_ERROR_CODE_OK)
			{
//				obj_clear(&obj);
//				retry++;
//				con->state = DLMS_STATE_ERROR;
////			return ret;
				dlms_client_set_obis_value("  ", 2, 2);
				continue;
			}
		}
		HAL_IWDG_Refresh(&hiwdg);
		obj_clear(&obj);
	}while((( *(dlms_client_get_obis_n(n_obis)) != '\0' ) && (n_obis < (DLMS_OBIS_MAX_NUM_VALUES-2)) ) && ( retry < 4));

	if (( *(dlms_client_get_obis_n(n_obis)) == '\0' ) || (n_obis >= (DLMS_OBIS_MAX_NUM_VALUES-2)) )
	{
//		char meter_clk_obis_data[15];
//		itoa(con_dlms_get_meter_clk_obis_value(), meter_clk_obis_data, 10);
//		dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);

		dlms_log_write_lock(0);
		dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
		CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
		if ( 0 == dlms_client_get_disconnected() )
		{
			con->state = DLMS_STATE_DISCONNECT;
		}
		else if ( dlms_client_get_disconnected() != 0 )
		{
			con->state = DLMS_DISCONNECT_CONTROL;
		}
//		obj_clear(&obj);
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_OBIS_1 FS_totalAllocated:%d. Send Meter:%d.\r\n",
				(int)Tick_Get(SECONDS), (int)getFS_totalAllocated(), (int)dlms_client_get_send_meter());
	}
//	retry = 0;
	return ret;
}

uint32_t dlms_client_on_demand_comm_max_demand_profile(connection *con)
{
//	gxObject obj;
	gxExtendedRegister extreg;
	uint32_t ret = 0, n_obis = 0;
	do
	{
		dlms_log_set_profile_max_demand_extra(0);
		if ( ( DLMS_READ_MAX_DEMAND_PROF_1 == dlms_client_get_obis_profile_frame_type()) || ( 1 == __checkMaxDemand(n_obis) ) )
		{
			if ( DLMS_READ_MAX_DEMAND_PROF_1 != dlms_client_get_obis_profile_frame_type())
			{
				dlms_log_set_profile_max_demand_extra(1);
				if (0 == dlms_client_get_obis_profile_max_demand_profile_extra())
				{
					dlms_client_set_obis_profile_max_demand_profile_extra(1);
					dlms_client_table_write_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
					dlms_client_table_read_client_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				}
			}
			else
			{
				dlms_log_set_profile_max_demand_extra(0);
				if (dlms_client_get_obis_profile_max_demand_profile_extra() != 0)
				{
					dlms_client_set_obis_profile_max_demand_profile_extra(0);
					dlms_client_table_write_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(),  dlms_client_get_obis_profile_frame_type());
					dlms_client_table_read_client_obis_profile(con_dlms_get_curr_device(), dlms_client_get_client(), dlms_client_get_client_obis_profile(), dlms_client_get_obis_profile_frame_type());
				}
			}
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> cosem_init DLMS_OBJECT_TYPE_EXTENDED_REGISTER.\r\n", (int)Tick_Get(SECONDS));
		ret = cosem_init(BASE(extreg), DLMS_OBJECT_TYPE_EXTENDED_REGISTER, dlms_client_get_obis_n(n_obis++));//"1.0.32.7.0.255");

		if ( 1 == dlms_client_get_short_name_ref() )
		{
			extreg.base.shortName = con_get_shortName(n_obis - 1);
		}

		ret = com_read(con, BASE(extreg), 2);
//		if (ret != DLMS_ERROR_CODE_OK)
//		{
//			obj_clear(BASE(extreg));
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			continue;
//		}
		HAL_IWDG_Refresh(&hiwdg);

		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}

		ret = com_read(con, BASE(extreg), 5);
		dlms_client_dec_next_obis_val();
//		if (ret != DLMS_ERROR_CODE_OK)
//		{
//			obj_clear(BASE(extreg));
//			retry++;
//			con->state = DLMS_STATE_ERROR;
////			return ret;
//			continue;
//		}
		HAL_IWDG_Refresh(&hiwdg);

		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}

		ret = com_read(con, BASE(extreg), 3);
		if (ret != DLMS_ERROR_CODE_OK)
		{
//			obj_clear(BASE(extreg));
//			retry++;
//			con->state = DLMS_STATE_ERROR;
////			return ret;
//			continue;
			dlms_client_set_obis_value("  ", 2, 2);
		}
		HAL_IWDG_Refresh(&hiwdg);

		obj_clear(BASE(extreg));
	}while((( *(dlms_client_get_obis_n(n_obis)) != '\0' ) && (n_obis < (DLMS_OBIS_MAX_NUM_VALUES-2)) ) && ( retry < 4));

	if (( *(dlms_client_get_obis_n(n_obis)) == '\0' ) || (n_obis >= (DLMS_OBIS_MAX_NUM_VALUES-2)) )
	{
		dlms_log_write_lock(0);
		dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
		CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
		if ( 0 == dlms_client_get_disconnected() )
		{
			con->state = DLMS_STATE_DISCONNECT;
		}
		else if ( dlms_client_get_disconnected() != 0 )
		{
			con->state = DLMS_DISCONNECT_CONTROL;
		}
//		obj_clear(BASE(extreg));
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_OBIS_1 FS_totalAllocated:%d. Send Meter:%d.\r\n",
				(int)Tick_Get(SECONDS), (int)getFS_totalAllocated(), (int)dlms_client_get_send_meter());
	}
	retry = 0;
	return ret;
}

uint32_t dlms_client_on_demand_comm_capture_period(connection* con)
{
	int          ret, i, get_id = 0;
	const char * obis;
	char         capture_period[300];
	uint32_t     capture_period_length = 0, quit = 0;

	memset(capture_period, 0, sizeof(capture_period));

	for (i = DLMS_OBIS_PROFILE_NUM; i <= (DLMS_PROFILE_NUM - 3); i++)
	{
		dlms_client_table_read_client_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(),  dlms_client_get_client_generic_profile(), i);
		obis = dlms_client_get_generic_obis_n(0);
		switch (dlms_client_get_generic_profile_frame_type()) {
		case DLMS_WRITE_LOAD_PROF_1:
			 if (0 == profile_capture.load1)
			 {
				 quit = 1;
			 }
			break;
		case DLMS_WRITE_LOAD_PROF_2:
			 if (0 == profile_capture.load2)
			 {
				 quit = 1;
			 }
			break;
		case DLMS_WRITE_INSTRUMENTATION_PROF_1:
			 if (0 == profile_capture.instrumentation)
			 {
				 quit = 1;
			 }
			break;
		case DLMS_WRITE_POWER_QUALITY_PROF:
			 if (0 == profile_capture.powerquality)
			 {
				 quit = 1;
			 }
			break;
		default:
			break;
		}
		if ( 1 == quit )
		{
			quit = 0;
			continue;
		}
		if ((dlms_client_get_generic_profile_frame_type() >= DLMS_OBIS_PROFILE_NUM)
		 && (dlms_client_get_generic_profile_frame_type() <= (DLMS_PROFILE_NUM - 3)) )
		{
			if ( ( ret = cosem_init(BASE(pg), DLMS_OBJECT_TYPE_PROFILE_GENERIC, obis) ) == 0 )
			{
				if ( 1 == dlms_client_get_short_name_ref() )
				{
					pg.base.shortName = con_get_generic_profile_shortName();
				}
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read(con, BASE(pg), 4);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Att 4 Capture Period ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
				if (ret == DLMS_ERROR_CODE_OK)
				{
					if ( 0 == get_id )
					{
						get_id = 1;
						sprintf(capture_period, "%s;%d;", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)(pg.capturePeriod/60));
						capture_period_length = strlen(capture_period);
					}
					else
					{
						sprintf(capture_period + capture_period_length, "%d;", (int)(pg.capturePeriod/60));
						capture_period_length = strlen(capture_period);
					}

//					if (i == (DLMS_PROFILE_NUM - 3) )
//					{
//						sprintf(capture_period + capture_period_length, "#");
//						dlms_client_set_dlms_data_cmd_on_demand(capture_period);
//					}
				}
				else
				{
//					shutdown_set_tamper_param(TAMPER_MQTT_COMMAND_ERROR);
					sprintf(capture_period, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
					capture_period_length = strlen(capture_period);
				}
				obj_clear(BASE(pg));
			}
		}
	}
	if (capture_period[0] != '\0')
	{
		sprintf(capture_period + capture_period_length, "#");
		dlms_client_set_dlms_data_cmd_on_demand(capture_period);
	}
	con->state = DLMS_STATE_DISCONNECT;
	return 0;
}

uint32_t dlms_client_on_demand_comm_set_capture_period(connection* con)
{
	int          ret, i, get_id = 0, pg_period = 0;
	const char * obis;
	char         capture_period[300];
	uint32_t     capture_period_length = 0;

	memset(capture_period, 0, sizeof(capture_period));

	for (i = DLMS_OBIS_PROFILE_NUM; i <= (DLMS_PROFILE_NUM - 3); i++)
	{
		dlms_client_table_read_client_generic_profile(con_dlms_get_curr_device(), dlms_client_get_client(),  dlms_client_get_client_generic_profile(), i);
		obis = dlms_client_get_generic_obis_n(0);

		switch (dlms_client_get_generic_profile_frame_type()) {
		case DLMS_WRITE_LOAD_PROF_1:
			pg_period = profile_capture.load1;
			break;
		case DLMS_WRITE_LOAD_PROF_2:
			pg_period = profile_capture.load2;
			break;
		case DLMS_WRITE_INSTRUMENTATION_PROF_1:
			pg_period = profile_capture.instrumentation;
			break;
		case DLMS_WRITE_POWER_QUALITY_PROF:
			pg_period = profile_capture.powerquality;
			break;
		default:
			break;
		}
		if ((dlms_client_get_generic_profile_frame_type() >= DLMS_OBIS_PROFILE_NUM)
		 && (dlms_client_get_generic_profile_frame_type() <= (DLMS_PROFILE_NUM - 3))
		 && (pg_period != 0))
		{
			if ( ( ret = cosem_init(BASE(pg), DLMS_OBJECT_TYPE_PROFILE_GENERIC, obis) ) == 0 )
			{
				if ( 1 == dlms_client_get_short_name_ref() )
				{
					pg.base.shortName = con_get_generic_profile_shortName();
				}
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				pg.capturePeriod = pg_period;
				ret = com_write(con, BASE(pg), 4);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Write Att 4 Capture Period ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
				if (ret == DLMS_ERROR_CODE_OK)
				{
					if ( 0 == get_id )
					{
						get_id = 1;
						sprintf(capture_period, "%s;%d;", dlms_client_get_meter_id(con_dlms_get_curr_device()), (int)(pg.capturePeriod/60));
						capture_period_length = strlen(capture_period);
					}
					else
					{
						sprintf(capture_period + capture_period_length, "%d;", (int)(pg.capturePeriod/60));
						capture_period_length = strlen(capture_period);
					}
				}
				else
				{
//					shutdown_set_tamper_param(TAMPER_MQTT_COMMAND_ERROR);
					sprintf(capture_period, "%s;%s", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
					capture_period_length = strlen(capture_period);
				}
				obj_clear(BASE(pg));
			}
		}
	}
	if (capture_period[0] != '\0')
	{
		sprintf(capture_period + capture_period_length, "#");
		dlms_client_set_dlms_data_cmd_on_demand(capture_period);
	}
	con->state = DLMS_STATE_DISCONNECT;
	return 0;
}

static uint8_t __CalculateChecksum(char *data, uint32_t index, uint32_t count)
{
	uint8_t result = 0;
	for (int pos = index; pos != index + count; ++pos)
	{
		result ^= data[pos];
	}
	return result;
}

static uint32_t __readLoadTunneling(connection* con, uint32_t index, uint32_t count)
{
	uint32_t i, j, ret = 0;
	const char obis[9][6] =
			{{'1','.','8','.','0'},{'1','.','8','.','1'},{'1','.','8','.','2'},
			 {'1','.','8','.','3'},{'1','.','8','.','4'},{'2','.','8','.','0'},
			 {'3','.','8','.','0'},{'1','.','6','.','0'},{'9','.','6','.','0'}};
	char index_val[2], index_padding[3];

	itoa(index,index_val,10);
	sprintf(index_padding,"%02d",(int)index);

	gxData tunneling;
	const unsigned char ln_tunn[6] = { 1, 1, 150, 0, 0, 255 };

	if ( ( ret = INIT_OBJECT(tunneling, DLMS_OBJECT_TYPE_DATA, ln_tunn) ) == 0 )
	{
		static char INIT[6];
		sprintf(INIT,"/?!\r\n");
		tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
		tunneling.value.strVal = (gxByteBuffer*)gxmalloc(sizeof(gxByteBuffer));
		tunneling.value.strVal->capacity = sizeof(INIT);
		tunneling.value.strVal->size = 5;
		tunneling.value.strVal->data = (u_char *)INIT;
		com_write(con, BASE(tunneling), 2);
		com_read(con, BASE(tunneling), 2);

		static char ACK[7];
		ACK[0] = 6;
		sprintf(&ACK[1],"051\r\n");
		tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
		tunneling.value.strVal->capacity = sizeof(ACK);
		tunneling.value.strVal->size = 6;
		tunneling.value.strVal->data = (u_char *)ACK;
		com_write(con, BASE(tunneling), 2);
		com_read(con, BASE(tunneling), 2);

		static char P1[17];
		P1[0] = 1;
		sprintf(&P1[1],"P1");
		P1[3] = 2;
		sprintf(&P1[4],"(22222222)");
		P1[14] = 3;
		P1[15] = __CalculateChecksum(P1, 1, 14 );
		tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
		tunneling.value.strVal->capacity = sizeof(P1);
		tunneling.value.strVal->size = 16;
		tunneling.value.strVal->data = (u_char *)P1;
		com_write(con, BASE(tunneling), 2);
		com_read(con, BASE(tunneling), 2);

//		dlms_client_reset_indexes();
//
//		dlms_client_set_obis_value(dlms_client_get_meter_id(con_dlms_get_curr_device()), strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())), 2);
//		char meter_clk_obis_data[15];
//		itoa(con_dlms_get_meter_clk_obis_value(), meter_clk_obis_data, 10);
//		dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);
		CircularBuffer_Get(dlms_client_get_send_msg_queue());
		static char R5[17];
		for ( j = 0; j < count; j++ )
		{
			dlms_client_reset_indexes();

			dlms_client_set_obis_value(dlms_client_get_meter_id(con_dlms_get_curr_device()), strlen(dlms_client_get_meter_id(con_dlms_get_curr_device())), 2);
			char meter_clk_obis_data[15];
			itoa(con_dlms_get_meter_clk_obis_value(), meter_clk_obis_data, 10);
			dlms_client_set_obis_value(meter_clk_obis_data, strlen(meter_clk_obis_data), 2);
			for ( i = 0; i < 9; i++ )
			{
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				HAL_IWDG_Refresh(&hiwdg);
				R5[0] = 1;
				sprintf(&R5[1],"R5");
				R5[3] = 2;
				sprintf(&R5[4], obis[i]);//sprintf(&R5[4],"1.8.0*01()");
				strcat(R5,"*");
				strcat(R5,index_padding);
				strcat(R5,"()");
				R5[14] = 3;
				R5[15] = __CalculateChecksum(R5, 1, 14 );
				tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
				tunneling.value.strVal->capacity = sizeof(R5);
				tunneling.value.strVal->size = 16;
				tunneling.value.strVal->data = (u_char *)R5;
				com_write(con, BASE(tunneling), 2);
				com_read3(con, BASE(tunneling), 2);
			}
			sprintf(index_padding,"%02d",(int)index++);
			CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
			dlms_raw_frame_tunneling_register();
		}
		static char B0[6];
		B0[0] = 1;
		sprintf(&B0[1],"B0");
		B0[3] = 3;
		B0[4] = __CalculateChecksum(B0, 1, 3 );
		tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
		tunneling.value.strVal->capacity = sizeof(B0);
		tunneling.value.strVal->size = 5;
		tunneling.value.strVal->data = (u_char *)B0;
		com_write(con, BASE(tunneling), 2);

		HAL_IWDG_Refresh(&hiwdg);
		tunneling.value.vt = (DLMS_DATA_TYPE) (DLMS_DATA_TYPE_STRING);
		obj_clear(BASE(tunneling));

//		dlms_log_write_lock(0);
//		dlms_client_set_dlms_comm_state(DLMS_COMM_WRITE);
//		CircularBuffer_Put(dlms_client_get_write_msg_queue(), dlms_client_get_obis_profile_frame_type());
	}
	return 0;
}

uint32_t dlms_client_on_demand_comm_load_profile(connection* con)
{
	int          ret = 0;
	const char * obis;
	struct tm    start_time;
	struct tm    end_time;
	uint32_t     index = 2, count = 1;

	read_task_t con_read_task = (read_task_t)CircularBuffer_Read(dlms_client_get_read_msg_queue());
	if ((con_read_task >= DLMS_READ_CMD_INST_PROF_1) && (con_read_task <= DLMS_READ_CMD_EVENT_LOG_PROF))
	{
		con_read_task -= DLMS_READ_CMD_INST_PROF_1;
	}

	LOGLIVE(LEVEL_1, "\r\nLOGLIVE> %d DLMS> DLMS_STATE_LOAD_PROFILE FS_totalAllocated:%d\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
	obis       = dlms_client_get_generic_obis_n(0);
	start_time = *con_convert_time(dlms_client_get_dlms_load_profile_start_time());
	end_time   = *con_convert_time(dlms_client_get_dlms_load_profile_end_time());
	count      = (dlms_client_get_dlms_load_profile_end_time() - dlms_client_get_dlms_load_profile_start_time())/3600 + 1;
	index      = (Tick_Get(SECONDS) - dlms_client_get_dlms_load_profile_end_time())/3600 - 1;
	if ( DLMS_READ_BILLING_PROF == con_read_task )
	{
		count = count / (24 * 30) + 1;
		index = (index + 1) / (24 * 30) + 1;
	}
	if ( 0 == strncmp("1.1.1.8.0.01", dlms_client_get_generic_obis_n(0), 12) )
	{
//		count = count / (24 * 30) + 1;
//		index = index/(24*30) + 1;
		__readLoadTunneling(con, index, count);
	}
	else if ( ( ret = cosem_init(BASE(pg), DLMS_OBJECT_TYPE_PROFILE_GENERIC, obis) ) == 0 )
	{
		if ( 1 == dlms_client_get_short_name_ref() )
		{
			if (udp_protocol_get_on_billing_profile())
			{
				pg.base.shortName = 0x8000;//0x6400;
				pg.base.shortName = con_get_generic_profile_shortName();
			}
			else if (udp_protocol_get_on_event_profile())
			{
				pg.base.shortName = 0x8080;//0x6270;
				pg.base.shortName = con_get_generic_profile_shortName();
			}
			else
			{
				pg.base.shortName = 0x5000;//0x60E0;
				pg.base.shortName = con_get_generic_profile_shortName();
			}
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_readValue(con, BASE(pg), 1);
		if (ret == DLMS_ERROR_CODE_SEND_FAILED)
		{
			retry++;
			con->state = DLMS_STATE_ERROR;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		//Read entries in use.
		ret = com_read(con, BASE(pg), 7);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Entries in use Att 7 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
		HAL_IWDG_Refresh(&hiwdg);
		if (ret == DLMS_ERROR_CODE_SEND_FAILED)
		{
			retry++;
			con->state = DLMS_STATE_ERROR;
			obj_clear(BASE(pg));
			return ret;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		//Read entries.
		ret = com_read(con, BASE(pg), 8);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read entries Att 8 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);

		HAL_IWDG_Refresh(&hiwdg);
		if (ret == DLMS_ERROR_CODE_SEND_FAILED)
		{
			retry++;
			con->state = DLMS_STATE_ERROR;
			obj_clear(BASE(pg));
			return ret;
		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(pg), 4);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Att 4 Capture Period ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
		HAL_IWDG_Refresh(&hiwdg);
//		if (ret == DLMS_ERROR_CODE_SEND_FAILED)
//		{
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			obj_clear(BASE(pg));
//			return ret;
//		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(pg), 3);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Att 3 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
		HAL_IWDG_Refresh(&hiwdg);
//		if (ret == DLMS_ERROR_CODE_SEND_FAILED)
//		{
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			obj_clear(BASE(pg));
//			return ret;
//		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		ret = com_read(con, BASE(pg), 5);
		LOGLIVE(LEVEL_2, "\r\nLOGLIVE> %d DLMS> Att 5 ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
		HAL_IWDG_Refresh(&hiwdg);
//		if (ret == DLMS_ERROR_CODE_SEND_FAILED)
//		{
//			retry++;
//			con->state = DLMS_STATE_ERROR;
//			obj_clear(BASE(pg));
//			return ret;
//		}
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		start_time.tm_sec   = 0;
		end_time.tm_sec     = 0;
		ret 	   = 0;
		ret        = com_readRowsByRange(con, &pg, &start_time, &end_time);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Range. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
		if (ret != DLMS_ERROR_CODE_OK)
		{
			HAL_IWDG_Refresh(&hiwdg);
			if ( 1 == params_get_slow_meter() )
			{
				HAL_Delay(1000);
			}
			LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read RowsByEntry: Index:%d. Count:%d.\r\n", (int)Tick_Get(SECONDS), (int)index, (int)count);
			ret        = com_readRowsByEntry(con, &pg, index, count);
			if (ret != DLMS_ERROR_CODE_OK)
			{
				HAL_IWDG_Refresh(&hiwdg);
				if ( 1 == params_get_slow_meter() )
				{
					HAL_Delay(1000);
				}
				ret = com_read_profile_object(con, BASE(pg), 2);
				HAL_IWDG_Refresh(&hiwdg);
				if (ret != DLMS_ERROR_CODE_OK)
				{
					LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Profile Object. ret:%d\r\n", (int)Tick_Get(SECONDS), (int)ret);
				}
				else
				{
					LOGLIVE(LEVEL_2, "LOGLIVE> %d DLMS> Read Profile Object OK.\r\n", (int)Tick_Get(SECONDS));
				}
			}
			if ((ret != DLMS_ERROR_CODE_OK) && (retry > MAX_RETRIES))
			{
				udp_protocol_set_on_billing_profile(0);
				udp_protocol_set_on_event_profile(0);
			}
		}
		HAL_IWDG_Refresh(&hiwdg);
		shutdown_reset_watchdog();
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> OBJ PROFILE GENERIC CLEAR FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
//		obj_clear(BASE(pg));
		LOGLIVE(LEVEL_1, "LOGLIVE> %d DLMS> DLMS_STATE_LOAD_PROFILE FS_totalAllocated:%d.\r\n", (int)Tick_Get(SECONDS), (int)getFS_totalAllocated());
	}
	if ( ret != 0 )
	{
		if (( 1 == udp_protocol_get_on_demand_command() ))
		{
//			shutdown_set_tamper_param(TAMPER_MQTT_COMMAND_ERROR);
		}
		retry++;
		con->state = DLMS_STATE_ERROR;
	}
	else
	{
		con->state = DLMS_STATE_DISCONNECT;
	}

	return ret;
}

uint32_t dlms_client_on_demand_cmd_check_send( void )
{
	if ( CircularBuffer_Read(dlms_client_get_send_msg_queue()) == DLMS_WAIT_FOR_SEND_DATA_CMD)
	{
		shutdown_setInitTelitModule(1);
		CircularBuffer_Get(dlms_client_get_send_msg_queue());
		LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910-TCP> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
		CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_SEND_DATA_CMD);
//		message_queue_write(SEND_NETWORK_PARAMS);
		shutdown_reset_watchdog();
	}
	return 0;
}

/**
* Update firmware of the meter.
*
* In image update following steps are made:
1. Image_transfer_enabled is read.
2. Image block size is read.
3. image_transferred_blocks_status is read to check is image try to update before.
4. image_transfer_initiate
5. image_transfer_status is read.
6. image_block_transfer
7. image_transfer_status is read.
8. image_transfer_status is read.
9. image_verify is called.
10. image_transfer_status is read.
11. image_activate is called.
*/
int imageUpdate(connection* connection, const unsigned char* identification, uint16_t identificationSize, unsigned char* image, uint32_t imageSize)
{
    int ret;
    gxByteBuffer bb;
    bb_init(&bb);
    dlmsVARIANT param;
    var_init(&param);
    gxImageTransfer im;
    unsigned char ln[] = { 0,0,44,0,0,255 };
    INIT_OBJECT(im, DLMS_OBJECT_TYPE_IMAGE_TRANSFER, ln);

    //1. Image_transfer_enabled is read.
    if ((ret = com_read(connection, BASE(im), 5)) == 0 &&
        //2. Image block size is read.
        (ret = com_read(connection, BASE(im), 2)) == 0 &&
        //3. image_transferred_blocks_status is read to check is image try to update before.
        (ret = com_read(connection, BASE(im), 3)) == 0 &&
        //4. image_transfer_initiate
        (ret = bb_setInt8(&bb, DLMS_DATA_TYPE_STRUCTURE)) == 0 &&
        (ret = bb_setInt8(&bb, 2)) == 0 &&
        (ret = bb_setInt8(&bb, DLMS_DATA_TYPE_OCTET_STRING)) == 0 &&
        (ret = hlp_setObjectCount(identificationSize, &bb)) == 0 &&
        (ret = bb_set(&bb, identification, identificationSize)) == 0 &&
        (ret = bb_setInt8(&bb, DLMS_DATA_TYPE_UINT32)) == 0 &&
        (ret = bb_setInt32(&bb, imageSize)) == 0 &&
        (ret = var_addOctetString(&param, &bb)) == 0 &&
        (ret = com_method(connection, BASE(im), 1, &param)) == 0)
    {
        //5. image_transfer_status is read.
        if ((ret = com_read(connection, BASE(im), 6)) == 0)
        {
            // 6. image_block_transfer
            uint32_t count = im.imageBlockSize;
            uint32_t blockNumber = 0;
            while (imageSize != 0)
            {
                if (imageSize < im.imageBlockSize)
                {
                    count = imageSize;
                }
                bb_clear(&bb);
                if ((ret = bb_setInt8(&bb, DLMS_DATA_TYPE_STRUCTURE)) != 0 ||
                    (ret = bb_setInt8(&bb, 2)) != 0 ||
                    (ret = bb_setInt8(&bb, DLMS_DATA_TYPE_UINT32)) != 0 ||
                    (ret = bb_setInt32(&bb, blockNumber)) != 0 ||
                    (ret = bb_setInt8(&bb, DLMS_DATA_TYPE_OCTET_STRING)) != 0 ||
                    (ret = hlp_setObjectCount(count, &bb)) != 0 ||
                    (ret = bb_set(&bb, image, count)) != 0 ||
                    (ret = var_addOctetString(&param, &bb)) != 0 ||
                    (ret = com_method(connection, BASE(im), 2, &param)) != 0)
                {
                    break;
                }
                imageSize -= count;
                ++blockNumber;
            }
            if (ret == 0)
            {
                //7. image_transfer_status is read.
                ret = com_read(connection, BASE(im), 6);
                if (ret == 0)
                {
                    //9. image_verify is called.
                    var_clear(&param);
                    GX_INT8(param) = 0;
                    if ((ret = com_method(connection, BASE(im), 3, &param)) == 0 ||
                        ret == DLMS_ERROR_CODE_TEMPORARY_FAILURE)
                    {
                        while (1)
                        {
                            //10. image_transfer_status is read.
                            ret = com_read(connection, BASE(im), 6);
                            if (im.imageTransferStatus == DLMS_IMAGE_TRANSFER_STATUS_VERIFICATION_SUCCESSFUL)
                            {
                                break;
                            }
                            if (im.imageTransferStatus == DLMS_IMAGE_TRANSFER_STATUS_VERIFICATION_FAILED)
                            {
                                ret = DLMS_ERROR_CODE_INVALID_PARAMETER;
                                break;
                            }

                            //Wait until image is activated.
#if defined(_WIN32) || defined(_WIN64)//Windows
                            Sleep(10000);
#else
                            HAL_Delay(1000000);
#endif //defined(_WIN32) || defined(_WIN64)
                        }
                        if (ret == 0)
                        {
                            ret = com_read(connection, BASE(im), 6);
                            //11. image_activate is called.
                            ret = com_method(connection, BASE(im), 4, &param);
                        }
                    }
                }
            }
        }
    }
    bb_clear(&bb);
    var_clear(&param);
    return ret;
}

uint32_t dlms_client_on_demand_comand_set_change_secret_key(connection* con)
{
    int ret;
    //0.0.40.0.0.255 Association LN
    gxAssociationShortName llsSecretKey;
    const unsigned char ln[6] = { 0, 0, 40, 0, 0, 255 };
    char         secret_key_str[9] = {'1','2','3','4','5','6','7','8'};
    gxByteBuffer secret;
    dlmsVARIANT  params;

    if ((ret = INIT_OBJECT(llsSecretKey, DLMS_OBJECT_TYPE_ASSOCIATION_SHORT_NAME, ln)) == 0)
    {
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
		llsSecretKey.base.shortName = 0xBD30;//0xFA00;//
		com_read(con, BASE(llsSecretKey), 1);
		if ( 1 == params_get_slow_meter() )
		{
			HAL_Delay(1000);
		}
//		com_read(con, BASE(llsSecretKey), 3);
//		if ( 1 == params_get_slow_meter() )
//		{
//			HAL_Delay(1000);
//		}
//		com_read(con, BASE(llsSecretKey), 6);
//		if ( 1 == params_get_slow_meter() )
//		{
//			HAL_Delay(1000);
//		}
		BYTE_BUFFER_INIT(&secret);
		bb_addString(&secret, secret_key_str);
		params.vt     = DLMS_DATA_TYPE_OCTET_STRING;
		params.strVal = &secret;
		ret = com_method(con, BASE(llsSecretKey), 5, &params);
		if (ret == DLMS_ERROR_CODE_OK)
		{
//			sprintf(secret_key_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), payment_strings[payment_mode]);
			dlms_client_set_dlms_data_cmd_on_demand(secret_key_str);
		}
		else
		{
			sprintf(secret_key_str, "%s;%s#", dlms_client_get_meter_id(con_dlms_get_curr_device()), "Fail");
			dlms_client_set_dlms_data_cmd_on_demand(secret_key_str);
		}
		obj_clear(BASE(llsSecretKey));
    }
    con->state = DLMS_STATE_DISCONNECT;

    return ret;
}
