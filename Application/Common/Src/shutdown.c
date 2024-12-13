/**
  ******************************************************************************
  * @file           shutdown.c
  * @author 		Datakorum Development Team
  * @brief			Driver to handle system shutdown.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions S.L.
  * All rights reserved.</center></h2>
  *
  * @verbatim
  * @endverbatim
  ******************************************************************************
  */
/*
 * shutdown.c
 *
 *  Created on: 16 jul. 2019
 *      Author: smill
 */
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "gpdma.h"
#include "spi.h"
#include "iwdg.h"
#include "shutdown.h"
#include "leds.h"
#include "comm_serial.h"
#include "adc.h"
#include "ad.h"
#include "icache.h"
#include "spi_flash.h"
#include "params.h"
#include "rtc_system.h"
#include "log.h"
#include "sensor_log.h"
#include "datalogger_buffer.h"
#include "message_queue.h"
#include "tick.h"
#include "rtc_system.h"
#include "udp_protocol.h"
#include "ME910.h"
#include "une82326.h"
#include "une82326_device_table.h"
#include "mbus.h"
#include "modbus.h"
#include "modbus_sensors.h"
#include "modbus_sensors_log.h"
#include "aqualabo_modbus.h"
#include "generic_modbus.h"
#include "dlms_client.h"
#include "pulses.h"
#include "test_prod.h"
#include "json.h"
#include "battery.h"
#include "generic_sensor.h"
#include "i2c_sensor.h"

#include "connection.h"
#include "dlms_client.h"
#include "dlms_log.h"
#include "dlms_client_table.h"

#ifdef MQTT
#include "mqtt_task.h"
#include "mqtt_timer.h"
#include "mqtt_frames.h"
#endif

//#define STOP3
/**
 * @defgroup System_Shutdown
 * @brief
 * @{
 * */

/**
 * @struct
 * @brief
 *
 */
typedef struct {
	uint32_t shutdown_on;
	uint32_t shutdown_time;
	uint32_t init_TelitModule;
	uint32_t send_keepalive;
	uint8_t  reset_shutdown_wchdg;
	uint32_t wchdg_time;
}shutdown_manager;

/** Structure to manage system shutdown */
static shutdown_manager shutdown_Manager = {
		.shutdown_on          = 0,		/*!< */
		.shutdown_time        = 0,
		.init_TelitModule     = 0,		/*!< Telit initialization state
												@arg 0 - Telit OFF not initialized
												@arg 1 - Telit ON initialized */
		.reset_shutdown_wchdg = 0,
		.wchdg_time           = SHUTDOWN_STAND_BY_TIME
};

__IO uint32_t reed_on;
__IO uint32_t reed_sending;
__IO uint32_t tamper_on;
__IO uint32_t tamper_int;
__IO uint32_t tamper_sending;
__IO uint32_t powermon_sending;
__IO uint32_t vbackup_on;
__IO uint32_t vbackup_int;
__IO uint32_t vbackup_sending;
__IO uint32_t alarm_on;
__IO uint32_t alarm_int;
__IO uint32_t alarm_int_low;
__IO uint32_t alarm_sending;

uint32_t tamper_param = 0;
uint32_t tamper_param_cmd = TAMPER_MQTT_COMMAND_ERROR;
uint32_t shutdown_time_periodic_mode = 900;
uint32_t shutdown_sync_meter_read_pressure_on = 0;
uint32_t shutdown_sync_modbus_read_pressure_on = 0;
/**@brief Flag to indicate whether synchronous read is enabled or not */
uint32_t synch      = 0;
/**@brief Flag to indicate that last pulse received is the last one before going to sleep */
uint32_t last_pulse = 0;
/**@brief Flag to indicate  */
uint32_t power_mon  = 0;
/**@brief Flag to indicate that meter data is requested */
uint32_t meter_send = 0;
/**@brief Flag to indicate  */
extern uint32_t rtc_refresh_time;

extern connection con;

extern uint32_t on_demand_backup;


/**
 * @fn void shutdown_set_sync_meter(uint32_t)
 * @brief Gets the status of the synchronous meter flag
 *
* @param _sync
 * 		@arg 0 - Sensor sensing time is not synchronized with time.
 * 		@arg 1 - Sensor sensing time is timed synchronized.
 */
void shutdown_set_sync_meter(uint32_t _sync)
{
	shutdown_sync_meter_read_pressure_on = _sync;
}

/**
 * @fn void shutdown_set_sync_modbus(uint32_t)
 * @brief Enables/Disables the time synchronized meter reading.
 *
 * @param _sync
 * 		@arg 0 - Sensor sensing time is not synchronized with time.
 * 		@arg 1 - Sensor sensing time is timed synchronized.
 */
void shutdown_set_sync_modbus(uint32_t _sync)
{
	shutdown_sync_modbus_read_pressure_on = _sync;
}

/**
 * @fn void shutdown_setInitTelitModule(uint32_t)
 * @brief Set the Telit Module state flag
 *
 * @param init
 * 			@arg 0 - Telit init state flag set to OFF
 * 			@arg 1 - Telit init state flag set to ON
 */
void shutdown_setInitTelitModule(uint32_t init)
{
	shutdown_Manager.init_TelitModule = init;
}

/**
 * @fn uint32_t shutdown_initTelitModule(void)
 * @brief Reads the Telit Module sate flag
 *
 * @return  init
 * 			@arg 0 - Telit init state flag set to OFF
 * 			@arg 1 - Telit init state flag set to ON
 */
uint32_t shutdown_initTelitModule(void)
{
	return shutdown_Manager.init_TelitModule;
}

/**
 * @fn uint32_t shutdown_sendKeepalive(void)
 * @brief
 *
 * @return
 */
uint32_t shutdown_sendKeepalive( void )
{
	return shutdown_Manager.send_keepalive;
}

/**
 * @fn void shutdown_setKeepalive(uint32_t)
 * @brief
 * @param send_keepalive
 */
void shutdown_setKeepalive( uint32_t send_keepalive )
{
	shutdown_Manager.send_keepalive = send_keepalive;
}

/**
 * @fn uint32_t shutdown_get_tamper(void)
 * @brief
 *
 * @return
 */
uint32_t shutdown_get_tamper( void )
{
	return tamper_on;
}

/**
 * @fn void shutdown_set_tamper_param(uint32_t)
 * @brief
 * @param _tamper
 */
void shutdown_set_tamper_param( uint32_t _tamper )
{
	tamper_param = _tamper;
}

/**
 * @fn uint32_t shutdown_get_tamper_param(void)
 * @brief
 * @return
 */
uint32_t shutdown_get_tamper_param( void )
{
	return tamper_param;
}

/**
 * @fn void shutdown_set_tamper_param(uint32_t)
 * @brief
 * @param _tamper
 */
void shutdown_set_tamper_param_cmd( uint32_t _tamper )
{
	tamper_param_cmd = _tamper;
}

/**
 * @fn uint32_t shutdown_get_tamper_param(void)
 * @brief
 * @return
 */
uint32_t shutdown_get_tamper_param_cmd( void )
{
	return tamper_param_cmd;
}

/**
 * @fn uint32_t shutdown_is_tamper_send_on_demand(void)
 * @brief
 * @return
 */
uint32_t shutdown_is_tamper_send_on_demand( void )
{
	if ((tamper_param == TAMPER_MQTT_ENERGYPROF)
	 || (tamper_param == TAMPER_MQTT_ON_DEMAND_BILLING_PROFILE)
	 || (tamper_param == TAMPER_MQTT_WATERPROFILE)
	 || (tamper_param == TAMPER_MQTT_ON_DEMAND_LOAD_PROFILE) )
	{
		return 1;
	}

	return 0;
}

/**
 * @fn uint32_t shutdown_get_tamper_sending(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t shutdown_get_tamper_sending( void )
{
	return tamper_sending;
}

/**
 * @fn void shutdown_set_tamper_sending(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _sending
 */
void shutdown_set_tamper_sending( uint32_t _sending )
{
	tamper_sending = _sending;
}

/**
 * @fn uint32_t shutdown_get_vbackup_sending(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t shutdown_get_vbackup_sending( void )
{
	return vbackup_sending;
}

/**
 * @fn void shutdown_set_vbackup_sending(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _sending
 */
void shutdown_set_vbackup_sending( uint32_t _sending )
{
	vbackup_sending = _sending;
}

/**
 * @fn uint32_t shutdown_get_vbackup_sending(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t shutdown_get_alarm_sending( void )
{
	return alarm_sending;
}

/**
 * @fn void shutdown_set_vbackup_sending(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _sending
 */
void shutdown_set_alarm_sending( uint32_t _sending )
{
	alarm_sending = _sending;
}

/**
 * @fn void shutdown_set_power_mon(uint32_t)
 * @brief
 * @param _power_mon
 */
void shutdown_set_power_mon( uint32_t _power_mon )
{
	power_mon = _power_mon;
}

/**
 * @fn uint32_t shutdown_get_power_mon(void)
 * @brief
 * @return
 */
uint32_t shutdown_get_power_mon( void )
{
	return power_mon;
}

#if defined(PWR_STOP)

/**
 * @fn void HAL_FLASH_Freeze_IWDG(void)
 * @brief
*/
void HAL_FLASH_Freeze_IWDG( void )
{
	FLASH_OBProgramInitTypeDef pOBInit;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTWERR); // Clear the FLASH's pending flags.
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);
	HAL_FLASH_OB_Unlock();

	HAL_FLASHEx_OBGetConfig(&pOBInit); // Get the Option bytes configuration.

	pOBInit.OptionType = OPTIONBYTE_USER;
	pOBInit.USERType   = OB_USER_IWDG_STOP;
	pOBInit.USERConfig = OB_IWDG_STOP_FREEZE;

	if((pOBInit.USERConfig & FLASH_OPTR_IWDG_STOP) != OB_IWDG_STOP_FREEZE)
	{
		FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
		HAL_FLASHEx_OBProgram(&pOBInit);
		HAL_FLASH_OB_Launch();
	}

	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}

/**
 * @fn void __initSST26V(void)
 * @brief
 */
static void __initSST26V( void )
{
	uint16_t block_protection_18[1];

	block_protection_18[0] = 0x00;

	sFLASH_ResetEnable();
	sFLASH_WaitBusy();
	sFLASH_Reset();
	sFLASH_WaitBusy();

	sFLASH_GlobalBlockProtectionUnlock();
	sFLASH_WaitBusy();

	sFLASH_WriteBlockProtection(&block_protection_18[0]);
	sFLASH_WaitBusy();

	sFLASH_ReadID();
	sFLASH_WaitBusy();
}

/**
 * @fn uint8_t __checkRTCQuart(void)
 * @brief Checks whether the system time is multiple of quart of an hour.
 * @return ret
 * 			@arg 0 - System time is not multiple of quart of an hour
 * 			@arg 1 - System time is multiple of quart of an hour.
 */
static uint8_t __checkRTCQuartNoPeriod(uint32_t *minute_backup_var)
{
	uint8_t ret = 0, ret_pulse = 0;
	uint32_t minute_backup = *minute_backup_var;

	if ((0  == Sys_time.Time.Minutes ) || (15 == Sys_time.Time.Minutes) || (30 == Sys_time.Time.Minutes) || (45 == Sys_time.Time.Minutes)
	  || (59 == Sys_time.Time.Minutes) || (1  == Sys_time.Time.Minutes)
	  || ((Sys_time.Time.Minutes >= 14) && (Sys_time.Time.Minutes <= 16))
	  || ((Sys_time.Time.Minutes >= 29) && (Sys_time.Time.Minutes <= 31))
	  || ((Sys_time.Time.Minutes >= 44) && (Sys_time.Time.Minutes <= 46))
	  || ((Sys_time.Time.Minutes%params_sensor_period_date_month(1)) == 0)
	  )
	{
		if ((Sys_time.Time.Minutes%params_sensor_period_date_month(1)) == 0)
		{
			if (Sys_time.Time.Minutes == minute_backup)
			{
				ret_pulse = 1;
			}
			minute_backup = Sys_time.Time.Minutes;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Check Quart Read. Sensor send period date month:%d. Minute backup:%d.\n",
					(int)Tick_Get( SECONDS ), (int)params_sensor_period_date_month(1), (int)minute_backup);
		}
		else
		{
			if ((Sys_time.Time.Minutes >= 14) && (Sys_time.Time.Minutes <= 16))
			{
				if (15 == minute_backup)
				{
					ret_pulse = 1;
				}
				minute_backup = 15;
			}
			else if ((Sys_time.Time.Minutes >= 29) && (Sys_time.Time.Minutes <= 31))
			{
				if (30 == minute_backup)
				{
					ret_pulse = 1;
				}
				minute_backup = 30;
			}
			else if ((Sys_time.Time.Minutes >= 44) && (Sys_time.Time.Minutes <= 46))
			{
				if (45 == minute_backup)
				{
					ret_pulse = 1;
				}
				minute_backup = 45;
			}
			else if ((59 == Sys_time.Time.Minutes) || (1  == Sys_time.Time.Minutes) || (0 == Sys_time.Time.Minutes))
			{
				if (0 == minute_backup)
				{
					ret_pulse = 1;
				}
				minute_backup = 0;
			}
		}
		*minute_backup_var = minute_backup;

		if (0 == ret_pulse)
		{
			ret = 1;
		}

		LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Check Quart Read. Minute backup:%d. RET:%d\n", (int)Tick_Get( SECONDS ), (int)minute_backup, (int)ret);
	}
	return ret;
}

/**
 * @fn uint8_t __checkRTCQuart(void)
 * @brief Checks whether the system time is multiple of quart of an hour.
 * @return ret
 * 			@arg 0 - System time is not multiple of quart of an hour
 * 			@arg 1 - System time is multiple of quart of an hour.
 */
static uint8_t __checkRTCQuart(uint32_t *minute_backup_var)
{
	uint8_t ret = 0, ret_pulse = 0;
	uint32_t minute_backup = *minute_backup_var;

	if ((0  == Sys_time.Time.Minutes ) || (15 == Sys_time.Time.Minutes) || (30 == Sys_time.Time.Minutes) || (45 == Sys_time.Time.Minutes)
	  || (59 == Sys_time.Time.Minutes) || (1  == Sys_time.Time.Minutes)
	  || ((Sys_time.Time.Minutes >= 14) && (Sys_time.Time.Minutes <= 16))
	  || ((Sys_time.Time.Minutes >= 29) && (Sys_time.Time.Minutes <= 31))
	  || ((Sys_time.Time.Minutes >= 44) && (Sys_time.Time.Minutes <= 46))
	  )
	{
		if ((Sys_time.Time.Minutes >= 14) && (Sys_time.Time.Minutes <= 16))
		{
			if (15 == minute_backup)
			{
				ret_pulse = 1;
			}
			minute_backup = 15;
		}
		else if ((Sys_time.Time.Minutes >= 29) && (Sys_time.Time.Minutes <= 31))
		{
			if (30 == minute_backup)
			{
				ret_pulse = 1;
			}
			minute_backup = 30;
		}
		else if ((Sys_time.Time.Minutes >= 44) && (Sys_time.Time.Minutes <= 46))
		{
			if (45 == minute_backup)
			{
				ret_pulse = 1;
			}
			minute_backup = 45;
		}
		else if ((59 == Sys_time.Time.Minutes) || (1  == Sys_time.Time.Minutes) || (0 == Sys_time.Time.Minutes))
		{
			if (0 == minute_backup)
			{
				ret_pulse = 1;
			}
			minute_backup = 0;
		}
		*minute_backup_var = minute_backup;
		if (0 == params_pulses_on())
		{
			ret       = 1;
		}
		else
		{
			if (0 == ret_pulse)
			{
				ret = 1;
			}
		}
//		if ( 1 == ret )
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Check Quart Read. Minute backup:%d. RET:%d\n", (int)Tick_Get( SECONDS ), (int)minute_backup, (int)ret);
		}
	}
	return ret;
}

/**
 * @fn uint8_t __checkRTCQuartAcc(void)
 * @brief Checks whether the system time is multiple of quart of an hour for pulse acc.
 * @return ret
 * 			@arg 0 - System time is not multiple of quart of an hour
 * 			@arg 1 - System time is multiple of quart of an hour.
 */
static uint32_t minute_backup = (uint32_t)-1;
static uint8_t __checkRTCQuartAccCreated(void)
{
	uint8_t  ret      = 0;
	uint32_t minutes  = (Sys_time.created_time[10] - 0x30)*10 + (Sys_time.created_time[11] - 0x30);
	uint32_t prev_min = 0;
	if ( 0 == minutes )
	{
		prev_min = 45;
	}
	else if ( ( minutes > 0 ) && ( minutes < 15 ) )
	{
		prev_min = 0;
	}
	else
	{
		prev_min = minutes - 15;
	}

	if ( (0  == minutes ) || (15 == minutes) || (30 == minutes) || (45 == minutes)
	  || ( ((prev_min > minute_backup) || ((minutes>0) && (minutes<15) && (minute_backup!=0))) && (minute_backup != (uint32_t)-1) )
	   )
	{
		if (minutes != minute_backup)
		{
			ret       = 1;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Check Quart Acc Read prev_min:%d minutes:%d.\n", (int)Tick_Get( SECONDS ), (int)prev_min, (int)minutes);
		}
		if ( ( minutes > 0 ) && ( minutes < 15 ) )
		{
			minute_backup = 0;
		}
		else
		{
			minute_backup = minutes;
		}
	}
	LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> __checkRTCQuartAccCreated minutes:%d minute_backup:%d.\n", (int)Tick_Get( SECONDS ), (int)minutes, (int)minute_backup);
	return ret;
}

/**
 * @fn void __throwModbusSensorRead(void)
 * @brief
 */
static void __throwModbusSensorRead(void)
{
	static uint32_t minute_backup = (uint32_t)-1;
	if ((synch == 1) && ((shutdown_sync_modbus_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup))))
	{
		aqualabo_modbus_enable(0);
		modbus_sensors_set_get_data(0);
		aqualabo_set_comm_state(0);
	}
	else
	{
		aqualabo_modbus_enable(1);
		modbus_sensors_set_get_data(1);
		modbus_sensors_init_task();
		aqualabo_set_comm_state(1);
	}
}

/**
 * @fn void __throwModbusSensorSend(void)
 * @brief Adds to queue a Sensor transmission
 */
static void __throwModbusSensorSend(void)
{
	if (0 != generic_485_get_read_time())
	{
		message_queue_write(SEND_MODBUS_SENSOR);
		if (1 == params_get_mqtt_dc_on())
		{
			mqtt_stop();
		}
	}

	/** Triggers Modbus sensor acquisition if it is the right time */
	static uint32_t minute_backup = (uint32_t)-1;
	/* synch read is enabled but still not the time to acquire a sample */
	if ((synch == 1) && ((shutdown_sync_modbus_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup))))
	{
		/* powers off sensor */
		aqualabo_modbus_enable(0);
		/* no modbus sensor data request */
		modbus_sensors_set_get_data(0);
		/* communication disabled */
		aqualabo_set_comm_state(0);
	}
	/* If it is the time to acquire a sample */
	else
	{
		/* power on sensor */
		aqualabo_modbus_enable(1);
		/* modbus sensor data is requested */
		modbus_sensors_set_get_data(1);
		modbus_sensors_init_task();
		/* communication started */
		aqualabo_set_comm_state(1);
	}

	/* reset times */
	rtc_system_reset_remaining_time();
}

/**
 * @fn void __throwModbusGenericSensorRead(void)
 * @brief 	Reads Modbus Generic sensor acquisition if it is the right time
 *
 */
static void __throwModbusGenericSensorRead(void)
{
	static uint32_t minute_backup = (uint32_t)-1;
	/* synch read is enabled but still not the time to read sample */
	if ((synch == 1) && ((shutdown_sync_modbus_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup))))
	{
		/* flag sensor disabled */
		generic_485_set_enable(0);
		/* no modbus sensor data is requested */
		modbus_sensors_set_get_data(0);
		/* communication disabled */
		generic_485_set_comm_state(0);
	}
	else
	{
		/* flag sensor enabled */
		generic_485_set_enable(1);
		/* modbus sensor data is requested */
		modbus_sensors_set_get_data(1);
		modbus_sensors_init_task();
		/* communication started */
		generic_485_set_comm_state(1);
	}
}

/**
 * @fn void __throwModbusGenericSensorSend(void)
 * @brief Adds to queue a modbus sensor reading
 */
static void __throwModbusGenericSensorSend(void)
{
	if (0 != generic_485_get_read_time())
	{
		message_queue_write(SEND_MODBUS_SENSOR);
		if (1 == params_get_mqtt_dc_on())
		{
			mqtt_stop();
		}
	}

	/** Triggers Modbus sensor acquisition if it is the right time */
	static uint32_t minute_backup = (uint32_t)-1;
	/* synch read is enabled but still not the time to acquire a sample */
	if ((synch == 1) && ((shutdown_sync_modbus_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup))))
	{
		/* flag sensor disable */
		generic_485_set_enable(0);
		/* no modbus data is requested */
		modbus_sensors_set_get_data(0);
		/* communication disabled */
		generic_485_set_comm_state(0);
	}
	else
	{
		/* flag sensor disabled */
		generic_485_set_enable(1);
		/* modbus data is requested */
		modbus_sensors_set_get_data(1);
		modbus_sensors_init_task();
		/* communication started */
		generic_485_set_comm_state(1);
	}
	/* reset times */
	rtc_system_reset_remaining_time();
}

/**
 * @fn void __throwDLMSSend(void)
 * @brief 	Reads Modbus Generic sensor acquisition if it is the right time
 *
 */
static void __throwDLMSSend(void)
{
//	message_queue_write(SEND_MODBUS_SENSOR);
	dlms_client_set_send_meter(1);

	/** Triggers Modbus sensor acquisition if it is the right time */
	static uint32_t minute_backup = (uint32_t)-1;
	/* synch read is enabled but still not the time to read sample */
	if ((synch == 1) && (0 == params_pulses_on()) && ((shutdown_sync_modbus_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup))))
	{
		con_dlms_change_state(&con, DLMS_STATE_IDLE);
	}
	else
	{
		con_dlms_change_state(&con, DLMS_STATE_INIT);
		dlms_client_set_read_meter(1);
	}
	/* reset times */
	rtc_system_reset_remaining_time();
}

/**
 * @fn uint32_t __checkPowerMon(void)
 * @brief
 * @return
 */
static uint32_t __checkPowerMon(void)
{
	uint32_t ret = 0;

	if (1 == shutdown_get_power_mon())
	{
		powermon_sending = 1;
		ret              = 1;
		shutdown_set_tamper_param(TAMPER_PSM);
		shutdown_setInitTelitModule(1);
		message_queue_write(SEND_NETWORK_PARAMS);
#if defined (UNE82326)
		if (une82326_device_table_get_num_of_devices() != 0 )
		{
			une82326_set_start_comm(1);
		}
#elif defined(MBUS)
		mbus_set_start_comm(1);
#endif
		if (1 == params_wq_on())
		{
			__throwModbusSensorSend();
			shutdown_setInitTelitModule(1);
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
		else if (MODBUS_RAW == generic_485_get_type())
		{
			if (0 != generic_485_get_read_time())
			{
				__throwModbusGenericSensorSend();
				shutdown_setInitTelitModule(1);
#if defined(MBUS)
				if ( 1 == params_config_get_disable_meter() ) {
					mbus_set_start_comm(0);
					meter_send   = 0;
				}
#endif
			}
		}
		else if (1 == dlms_client_get_dlms_enable())
		{
			if (0 != dlms_client_get_read_time())
			{
				__throwDLMSSend();
				shutdown_setInitTelitModule(1);
#if defined(MBUS)
				if ( 1 == params_config_get_disable_meter() ) {
					mbus_set_start_comm(0);
					meter_send   = 0;
				}
#endif
			}
		}

//		message_queue_write(SEND_NETWORK_PARAMS);

		if ((1 == params_pulses_on()) || (1 == AD_GetADOn()))
		{
			if (1 == params_config_get_sensor_log())
			{
				message_queue_write( SEND_SENSOR );
			}
		}

		if (1 == AD_GetADOn())
		{
			AD_SetGetADCData(1);
			rtc_system_reset_remaining_time();
		}
	}

	return ret;
}

/**
 * @fn void __checkMeterType(void)
 * @brief
 */
static void __checkMeterType( void )
{
	shutdown_setInitTelitModule(1);
	message_queue_write(SEND_NETWORK_PARAMS);
#if defined (UNE82326)
	if ( 0 == Tick_cloud_time_init() )
	{
		une82326_set_start_comm(1);
		meter_send = 1;
	}
	else if ( une82326_device_table_get_num_of_devices() != 0 )
	{
		une82326_set_start_comm(1);
		meter_send = 1;
	}
#elif defined(MBUS)
	if ( 0 == params_config_get_disable_meter() )
	{
//		if ( MBUS_METER_TYPE == params_get_uart_meter_type() )
		{
			mbus_set_start_comm(1);
			meter_send = 1;
		}
	}
#endif
	if (1 == params_wq_on())
	{
		__throwModbusSensorSend();
		shutdown_setInitTelitModule(1);
#if defined(MBUS)
		if ( 1 == params_config_get_disable_meter() ) {
			mbus_set_start_comm(0);
			meter_send   = 0;
		}
#endif
	}
	else if (MODBUS_RAW == generic_485_get_type())
	{
		if ( (0 != generic_485_get_read_time()) && (0xFF != generic_485_get_slave_id()) )
		{
//			__throwModbusGenericSensorSend();
			message_queue_write(SEND_MODBUS_SENSOR);
			if (1 == params_get_mqtt_dc_on())
			{
				mqtt_stop();
			}
			/* flag sensor disabled */
			generic_485_set_enable(1);
			/* modbus data is requested */
			modbus_sensors_set_get_data(1);
			modbus_sensors_init_task();
			/* communication started */
			generic_485_set_comm_state(1);
			/* reset times */
			rtc_system_reset_remaining_time();
			shutdown_setInitTelitModule(1);
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
	}
	else if (1 == dlms_client_get_dlms_enable())
	{
		if (0 != dlms_client_get_read_time())
		{
			if (CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue()) == 0)
			{
				message_queue_write(SEND_MODBUS_SENSOR);
				con_dlms_change_state(&con, DLMS_STATE_INIT);
				/* reset times */
				rtc_system_reset_remaining_time();
				shutdown_setInitTelitModule(1);
			}
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
	}

	if ( (1 == params_pulses_on()) || (1 == AD_GetADOn()))
	{
		if ( ( 0 == params_get_pulse_acc() ) || ( ( 1 == params_get_pulse_acc() ) && ( 86400 == params_config_send_time() ) ) )
		{
			if ( 1 == params_config_get_sensor_log() )
			{
				message_queue_write( SEND_SENSOR );
			}
			if (params_input_pulse_as_sensor_get_num() != 0)
			{
				generic_sensor_set_on_off(0, 1);
				generic_sensor_set_sensors_to_send();
			}
		}
	}

	if (1 == AD_GetADOn())
	{
		AD_SetGetADCData(1);
		rtc_system_reset_remaining_time();
	}
}

/**
 * @fn void __checkReedSensor(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __checkReedSensor( void )
{
//	reed_on = 1;//DEBUG:
	if (1 == reed_on)
	{
		LOGLIVE(LEVEL_2,"LOGLIVE> ----------------REED ON-------------------\r\n");
		reed_on = 0;
		reed_sending = 1;
		shutdown_set_tamper_param(TAMPER_REED_SENSOR_ON);
		shutdown_setInitTelitModule(1);
		message_queue_write( SEND_NETWORK_PARAMS );
#if defined (UNE82326)
		if ( une82326_device_table_get_num_of_devices() != 0 )
		{
			une82326_set_start_comm(1);
		}
#elif defined(MBUS)
		if ( 0 == params_config_get_disable_meter() )
		{
//			if ( MBUS_METER_TYPE == params_get_uart_meter_type() )
			{
				mbus_set_start_comm(1);
				meter_send   = 1;
			}
		}
#endif
		if (1 == params_wq_on())
		{
			__throwModbusSensorSend();
			shutdown_setInitTelitModule(1);
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
		else if (MODBUS_RAW == generic_485_get_type())
		{
			if (0 != generic_485_get_read_time())
			{
//				__throwModbusGenericSensorSend();
				message_queue_write(SEND_MODBUS_SENSOR);
				if (1 == params_get_mqtt_dc_on())
				{
					mqtt_stop();
				}
				/* flag sensor disabled */
				generic_485_set_enable(1);
				/* modbus data is requested */
				modbus_sensors_set_get_data(1);
				modbus_sensors_init_task();
				/* communication started */
				generic_485_set_comm_state(1);
				/* reset times */
				rtc_system_reset_remaining_time();
				shutdown_setInitTelitModule(1);
#if defined(MBUS)
				if ( 1 == params_config_get_disable_meter() ) {
					mbus_set_start_comm(0);
					meter_send   = 0;
				}
#endif
			}
		}
		else if (1 == dlms_client_get_dlms_enable())
		{
			if (0 != dlms_client_get_read_time())
			{
				__throwDLMSSend();
				shutdown_setInitTelitModule(1);
#if defined(MBUS)
				if ( 1 == params_config_get_disable_meter() ) {
					mbus_set_start_comm(0);
					meter_send   = 0;
				}
#endif
			}
		}
//		message_queue_write( SEND_NETWORK_PARAMS );

		if ((1 == params_pulses_on()) || (1 == AD_GetADOn()))
		{
			if (1 == params_config_get_sensor_log())
			{
				message_queue_write( SEND_SENSOR );
			}
			if (params_input_pulse_as_sensor_get_num() != 0)
			{
				generic_sensor_set_on_off(0, 1);
				generic_sensor_set_sensors_to_send();
			}
		}
		if (1 == AD_GetADOn())
		{
			AD_SetGetADCData(1);
			rtc_system_reset_remaining_time();
		}
	}
	else
	{
		shutdown_setInitTelitModule(0);
	}
}

#ifdef MQTT
static uint32_t __decodeCommand(dlms_client_command_t com)
{
	uint32_t i = 0, j = 0, no_cmd = 0;
	UNUSED(no_cmd);
	switch(com) {
	case DLMS_HES_COMMAND_ON_DEMAND_ENERGY_REGISTERS:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_ENERGY_REGISTERS);
		for (j = 0; j < dlms_client_table_get_num_meters_from_cmd(); j++)
		{
			i = con_dlms_get_curr_id_device_from_cmd_table(j);
			if (*dlms_client_get_meter_id(j) != 0)
			{
				if ( 1 == dlms_client_get_energy_profile_1())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_ENERGY_PROF_1);
					if (DLMS_READ_ENERGY_PROF_1 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_ENERGY_PROF_1, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_ENERGY_PROF_1, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_energy_profile_1(0);
					}
				}
				if ( 1 == dlms_client_get_energy_profile_2())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_ENERGY_PROF_2);
					if (DLMS_READ_ENERGY_PROF_2 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_ENERGY_PROF_2, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_ENERGY_PROF_2, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_energy_profile_2(0);
					}
				}
				if ( 1 == dlms_client_get_energy_profile_3())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_ENERGY_PROF_3);
					if (DLMS_READ_ENERGY_PROF_3 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_ENERGY_PROF_3, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_ENERGY_PROF_3, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_energy_profile_3(0);
					}
				}
			}
		}
		if (( 0 == dlms_client_get_energy_profile_1()) && ( 0 == dlms_client_get_energy_profile_2())
		 && ( 0 == dlms_client_get_energy_profile_3()) )
		{
			mqtt_stop();
		}
		dlms_client_set_energy_profile_1(0);
		dlms_client_set_energy_profile_2(0);
		dlms_client_set_energy_profile_3(0);
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES);
		for (j = 0; j < dlms_client_table_get_num_meters_from_cmd(); j++)
		{
			if (*con_dlms_get_device_from_cmd(j) != 0)
			{
				i = con_dlms_get_curr_id_device_from_cmd_table(j);
				if ( 1 == dlms_client_get_inst_profile_1())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_INST_PROF_1);
					if (DLMS_READ_INST_PROF_1 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INST_PROF_1, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INST_PROF_1, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_inst_profile_1(0);
					}
				}
				if ( 1 == dlms_client_get_inst_profile_2())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_INST_PROF_2);
					if (DLMS_READ_INST_PROF_2 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INST_PROF_2, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INST_PROF_2, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_inst_profile_2(0);
					}
				}
				if ( 1 == dlms_client_get_inst_profile_3())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_INST_PROF_3);
					if (DLMS_READ_INST_PROF_3 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INST_PROF_3, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INST_PROF_3, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_inst_profile_3(0);
					}
				}
				if ( 1 == dlms_client_get_inst_profile_4())
				{
					dlms_client_table_read_client_obis_profile(i, dlms_client_get_client(),
							dlms_client_get_client_obis_profile(), DLMS_READ_INST_PROF_4);
					if (DLMS_READ_INST_PROF_4 == dlms_client_get_obis_profile_frame_type())
					{
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INST_PROF_4, con_dlms_in_process());
						LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
						CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INST_PROF_4, con_dlms_in_process());
						dlms_client_inc_dlms_reading_items(i);
					}
					else
					{
						dlms_client_set_inst_profile_4(0);
					}
				}
			}
		}
		if (( 0 == dlms_client_get_inst_profile_1()) && ( 0 == dlms_client_get_inst_profile_2())
		 && ( 0 == dlms_client_get_inst_profile_3()) && ( 0 == dlms_client_get_inst_profile_4()))
		{
			mqtt_stop();
		}
		dlms_client_set_inst_profile_1(0);
		dlms_client_set_inst_profile_2(0);
		dlms_client_set_inst_profile_3(0);
		dlms_client_set_inst_profile_4(0);
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS);
		for (j = 0; j < dlms_client_table_get_num_meters_from_cmd(); j++)
		{
			if (*con_dlms_get_device_from_cmd(j) != 0)
			{
				i = con_dlms_get_curr_id_device_from_cmd_table(j);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
				CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_MAX_DEMAND_PROF_1, con_dlms_in_process());
				LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
				CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_MAX_DEMAND_PROF_1, con_dlms_in_process());
				dlms_client_inc_dlms_reading_items(i);
			}
		}
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_1:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_1);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_LOAD_PROF_1, con_dlms_in_process());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_LOAD_PROF_1, con_dlms_in_process());
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_2:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_2);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_LOAD_PROF_2, con_dlms_in_process());//CircularBuffer_Put(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_LOAD_PROF_2);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_LOAD_PROF_2, con_dlms_in_process());//CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_LOAD_PROF_2);
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_INSTRUMENTATION_PROF_1, con_dlms_in_process());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_INSTRUMENTATION_PROF_1, con_dlms_in_process());
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_POWER_QUALITY_PROF, con_dlms_in_process());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_POWER_QUALITY_PROF, con_dlms_in_process());
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_BILLING_PROFILE:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_BILLING_PROFILE);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_BILLING_PROF, con_dlms_in_process());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_BILLING_PROF, con_dlms_in_process());
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_ON_DEMAND_EVENTS_PROFILE:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(DLMS_HES_COMMAND_ON_DEMAND_EVENTS_PROFILE);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_CMD_EVENT_LOG_PROF, con_dlms_in_process());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF, con_dlms_in_process());
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_READ_TIME_CLOCK:
	case DLMS_HES_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD:
	case DLMS_HES_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD:
	case DLMS_HES_COMMAND_CUTTOFF_RECONNECTION:
	case DLMS_HES_COMMAND_GET_SWITCH_STATUS:
	case DLMS_HES_COMMAND_MAXIMUM_DEMAND_RESET:
	case DLMS_HES_COMMAND_SET_LOAD_LIMITATION:
	case DLMS_HES_COMMAND_GET_LOAD_LIMIT_THRESHOLD:
	case DLMS_HES_COMMAND_SET_BILLING_DATE:
	case DLMS_HES_COMMAND_GET_BILLING_DATE:
	case DLMS_HES_COMMAND_GET_DEMAND_INTEGRATION_PERIOD:
	case DLMS_HES_COMMAND_SET_DEMAND_INTEGRATION_PERIOD:
	case DLMS_HES_COMMAND_GET_PAYMENT_MODE:
	case DLMS_HES_COMMAND_SET_PAYMENT_MODE:
	case DLMS_HES_COMMAND_GET_METERING_MODE:
	case DLMS_HES_COMMAND_SET_METERING_MODE:
	case DLMS_HES_COMMAND_GET_METER_STATUS:
	case DLMS_HES_COMMAND_READ_METER_NAMEPLATE:
	case DLMS_HES_COMMAND_SET_VOLT_RANGE_LOW:
	case DLMS_HES_COMMAND_SET_VOLT_RANGE_UP:
	case DLMS_HES_COMMAND_GET_VOLT_RANGE_LOW:
	case DLMS_HES_COMMAND_GET_VOLT_RANGE_UP:
	case DLMS_HES_COMMAND_SET_CURRENT_RANGE_LOW:
	case DLMS_HES_COMMAND_SET_CURRENT_RANGE_UP:
	case DLMS_HES_COMMAND_GET_CURRENT_RANGE_LOW:
	case DLMS_HES_COMMAND_GET_CURRENT_RANGE_UP:
	case DLMS_HES_COMMAND_CLEAR_ALARMS:
	case DLMS_HES_COMMAND_GET_ACTIVE_TARIFF:
	case DLMS_HES_COMMAND_METER_PING:
	case DLMS_HES_COMMAND_METER_SET_TARIFF_AGREEMENT:
	case DLMS_HES_COMMAND_METER_SYNCHRONIZE:
	case DLMS_HES_COMMAND_SET_GW_SYNCH:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(com);
		for (i = 0; i < dlms_client_table_get_num_meters_from_cmd(); i++)
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue. Command: %d.\r\n", (int)Tick_Get( SECONDS ), (int)com);
			CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_DATA_CMD, con_dlms_in_process());
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_DATA_CMD, con_dlms_in_process());
		}
		dlms_client_table_inc_reading_items();
		break;
	case DLMS_HES_COMMAND_GET_GW_TIME:
//	case DLMS_HES_COMMAND_SET_GW_SYNCH:
	case DLMS_HES_COMMAND_SET_GW_INTERVAL:
	case DLMS_HES_COMMAND_GET_GW_INTERVAL:
//	case DLMS_HES_COMMAND_GET_GW_NAMEPLATE:
//	case DLMS_HES_COMMAND_GW_PING:
		dlms_client_clear_dlms_data_cmd_on_demand();
		dlms_client_set_command(com);
//		LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue. Command: %d.\r\n", (int)Tick_Get( SECONDS ), (int)com);
//		CircularBuffer_Replace(dlms_client_get_read_msg_queue(), DLMS_READ_DATA_CMD, con_dlms_in_process());
		LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue.\r\n", (int)Tick_Get( SECONDS ));
		CircularBuffer_Replace(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_DATA_CMD, con_dlms_in_process());
		con_dlms_change_state(&con, DLMS_STATE_ON_DEMAND_COMMAND);
		no_cmd = 1;
		mqtt_stop();
		break;
	case DLMS_HES_COMMAND_GW_PING:
	case DLMS_HES_COMMAND_GET_GW_NAMEPLATE:
	case DLMS_HES_COMMAND_GW_FIRMWARE_UPDATE:
		no_cmd = 1;
		mqtt_stop();
		break;
	default:
		no_cmd = 1;
		break;
	}
	if (0 == no_cmd)
	{
		if ( 1== con_dlms_in_process() )
		{
			con_dlms_set_release_curr_com(1);
		}
	}
	return 0;
}

/**
 * @fn void shutdown_mqttWaterCommandprocess(void)
 * @brief
 *
 * @pre
 * @post
 */
void shutdown_mqttWaterCommandprocess( void )
{
	shutdown_mqtt_restoreContext();
	shutdown_set_start_count(1);
	shutdown_setInitTelitModule(1);
	if ( 0 == mqtt_is_state_idle() )
	{
		mqtt_stop();
	}
	shutdown_set_tamper_param_cmd(TAMPER_MQTT_ENERGYPROF);
	message_queue_write( SEND_NETWORK_PARAMS_CMD );
	if ( 1 == params_config_get_disable_meter() ) {
		mbus_set_start_comm(0);
		meter_send   = 0;
	}
	else
	{
		mbus_set_start_comm(1);
		meter_send   = 1;
	}
}

/**
 * @fn void shutdown_mqttCommandprocess(void)
 * @brief
 *
 * @pre
 * @post
 */
void shutdown_mqttCommandprocess( void )
{
	shutdown_mqtt_restoreContext();
	shutdown_set_start_count(1);
	shutdown_setInitTelitModule(1);
	if ( 0 == mqtt_is_state_idle() )
	{
//		mqtt_stop();
	}
	shutdown_set_tamper_param_cmd(TAMPER_MQTT_ENERGYPROF);
	message_queue_write( SEND_NETWORK_PARAMS_CMD );
#if defined (UNE82326)
	if ( une82326_device_table_get_num_of_devices() != 0 )
	{
		une82326_set_start_comm(1);
	}
#elif defined(MBUS)
//	mbus_set_start_comm(1);
#endif
	if (1 == params_wq_on())
	{
		__throwModbusSensorSend();
		shutdown_setInitTelitModule(1);
#if defined(MBUS)
		if ( 1 == params_config_get_disable_meter() ) {
			mbus_set_start_comm(0);
			meter_send   = 0;
		}
#endif
	}
	else if (MODBUS_RAW == generic_485_get_type())
	{
		if (0 != generic_485_get_read_time())
		{
			message_queue_write(SEND_MODBUS_SENSOR);
			if (1 == params_get_mqtt_dc_on())
			{
				mqtt_stop();
			}
			/* flag sensor disabled */
			generic_485_set_enable(1);
			/* modbus data is requested */
			modbus_sensors_set_get_data(1);
			modbus_sensors_init_task();
			/* communication started */
			generic_485_set_comm_state(1);
			/* reset times */
			rtc_system_reset_remaining_time();
			shutdown_setInitTelitModule(1);
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
	}
	else if (1 == dlms_client_get_dlms_enable())
	{
		if (0 != dlms_client_get_read_time())
		{
			/* reset times */
			rtc_system_reset_remaining_time();

			__decodeCommand(dlms_client_get_command());
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
//			else
//			{
//				meter_send   = 1;
//			}
#endif
		}
	}

	if ((1 == params_pulses_on()) || (1 == AD_GetADOn()))
	{
		if (1 == params_config_get_sensor_log())
		{
			message_queue_write( SEND_SENSOR );
		}
	}
	if (1 == AD_GetADOn())
	{
		AD_SetGetADCData(1);
		rtc_system_reset_remaining_time();
	}
}

/**
 * @fn void shutdown_mqttCommandprocess(void)
 * @brief
 *
 * @pre
 * @post
 */
void shutdown_mqttCommandReconnectionprocess( uint32_t reconnection )
{
	shutdown_mqtt_restoreContext();
	shutdown_set_start_count(1);
	shutdown_setInitTelitModule(1);
	if ( 0 == mqtt_is_state_idle() )
	{
//		mqtt_stop();
	}
	shutdown_set_tamper_param_cmd(TAMPER_MQTT_RECONNECT);
	message_queue_write( SEND_NETWORK_PARAMS_CMD );
	dlms_client_set_disconnected(reconnection);
//	con_dlms_change_state(&con, DLMS_STATE_INIT);
	if (1 == dlms_client_get_dlms_enable())
	{
		if (0 != dlms_client_get_read_time())
		{
			__decodeCommand(dlms_client_get_command());
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
	}
}

/**
 * @fn void shutdown_mqttCommandprocess(void)
 * @brief
 *
 * @pre
 * @post
 */
void shutdown_mqttOnDemandCommandprocess( void )
{
	shutdown_mqtt_restoreContext();
	shutdown_set_start_count(1);
	shutdown_setInitTelitModule(1);
	if ( 0 == mqtt_is_state_idle() )
	{
//		mqtt_stop();
	}
	shutdown_set_tamper_param_cmd(TAMPER_MQTT_ON_DEMAND_LOAD_PROFILE);
	message_queue_write( SEND_NETWORK_PARAMS_CMD );
	if (1 == dlms_client_get_dlms_enable())
	{
		if (0 != dlms_client_get_read_time())
		{
//			__throwDLMSOndemandRead();
//			shutdown_setInitTelitModule(1);
			__decodeCommand(dlms_client_get_command());
#if defined(MBUS)
			if ( 1 == params_config_get_disable_meter() ) {
				mbus_set_start_comm(0);
				meter_send   = 0;
			}
#endif
		}
	}
}

#endif

/**
 * @fn void __gpio_init(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __gpio_init( void )
{
	MX_GPIO_Init();//MX_GPIO_Custom_Init();//

	if (0 == leds_get_leds_enabled())
	{
		leds_LED_Init_Read(LED_GREEN);
		leds_LED_Init_Read(LED_BLUE);
	}
	else
	{
		leds_init();
	}
}

#ifdef MQTT
void shutdown_mqtt_restoreContext( void )
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN>MQTT Restore Context\r\n", (int)Tick_Get( SECONDS ));
	if ( HAL_IWDG_Refresh(&hiwdg) != HAL_OK )
	{
		/* Refresh Error */
		Error_Handler();
	}

	Tick_init();
	shutdown_Manager.shutdown_on = 0;
//	rtc_system_setAlarmOccured(0);
#if defined (UNE82326)
	une82326_reset();
#elif defined (MBUS)
	mbus_reset();
#endif
//	DataLogger_Init();
	DataLogger_SendEnd();
#ifdef UNE82326
	datalog_reset_frame_tx_buffer();
#endif
	datalog_reset_buffer();
//	message_queue_delete(message_queue_get_elements());
	mqtt_task_set_send_pending(0);
	mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
	udp_comm_reset_buffers();
	udp_reset_buffers();
	udp_protocol_set_send_pending(0);
	udp_protocol_set_status( UDP_PROT_IDLE );
	udp_set_server_resp_type(0);
	udp_protocol_reset_datalogger_max_msgs_params();
	shutdown_reset_watchdog();
	leds_set_NET_status( LOW_POWER );
	leds_set_device_switch_on(0);
//	shutdown_set_start_count(0);
	shutdown_set_alarm_pressure_check(0);
	shutdown_set_alarm_generic_sensor_check(0);
	sensor_log_write_lock(0);
	sensor_log_set_write_pulse_acc(0);

//	if ( 1 == params_config_get_period() )
	{
		if ( 1 == Tick_cloud_time_init() )
		{
			if ( 1 == AD_GetADOn() ) {
				if ( 0 == pulses_get_pulse_rx() )
				{
					last_pulse = rtc_system_last_pulse();
					AD_SetGetADCData(1);
					rtc_system_reset_remaining_time_last_pulse();
					rtc_system_set_timeout_waiting_pulse();
				}
				else
				{
					last_pulse = rtc_system_last_pulse();
					if ( 1 == last_pulse )
					{
						rtc_system_reset_remaining_time_last_pulse();
					}
				}
			}
		}
	}

	modbus_sensors_log_write_lock(0);
	modbus_sensors_set_comm_state( MODBUS_COMM_DISABLED );
	dlms_log_write_lock(0);
	dlms_client_set_dlms_comm_state(DLMS_COMM_DISABLED);
	modbus_sensors_set_get_data(0);
	generic_485_set_tx_on(0);
	leds_init();

	if ( 0 == Telit_is_socket_available() )
	{
		ME910_reset_status();
	}
	ME910_set_network_params_get(0);
	ME910_set_dk_network_params_get(0);
	__checkReedSensor();
	MX_GPIO_Enable_Power_Mon(0);
}

void shutdown_mqtt_restorePressureContext( void )
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN>MQTT Restore Pressure Context\r\n", (int)Tick_Get( SECONDS ));
	if ( HAL_IWDG_Refresh(&hiwdg) != HAL_OK )
	{
		/* Refresh Error */
		Error_Handler();
	}

	Tick_init();
	shutdown_Manager.shutdown_on = 0;
	rtc_system_setAlarmOccured(0);
#if defined (UNE82326)
	une82326_reset();
#elif defined (MBUS)
	mbus_reset();
#endif
//	DataLogger_Init();
//	DataLogger_SendEnd();
#ifdef UNE82326
	datalog_reset_frame_tx_buffer();
#endif
	datalog_reset_buffer();
//	message_queue_delete(message_queue_get_elements());
//	mqtt_task_set_send_pending(0);
//	mqtt_frames_set_wait_server_resp(LAST_SERVER_RESP);
	udp_comm_reset_buffers();
	udp_reset_buffers();
	udp_protocol_set_send_pending(0);
	udp_protocol_set_status( UDP_PROT_IDLE );
	udp_set_server_resp_type(0);
	udp_protocol_reset_datalogger_max_msgs_params();
	shutdown_reset_watchdog();
	leds_set_NET_status( LOW_POWER );
	leds_set_device_switch_on(0);
	shutdown_set_alarm_pressure_check(0);
	shutdown_set_alarm_generic_sensor_check(0);
	sensor_log_write_lock(0);
	sensor_log_set_write_pulse_acc(0);

//	if ( 1 == params_config_get_period() )
	{
		if ( 1 == Tick_cloud_time_init() )
		{
			if ( 1 == AD_GetADOn() ) {
				if ( 0 == pulses_get_pulse_rx() )
				{
					last_pulse = rtc_system_last_pulse();
					AD_SetGetADCData(1);
					rtc_system_reset_remaining_time_last_pulse();
					rtc_system_set_timeout_waiting_pulse();
					if (params_input_pulse_as_sensor_get_num() != 0)
					{
						generic_sensor_set_on_off(0, 1);
					}
				}
				else
				{
					last_pulse = rtc_system_last_pulse();
					if ( 1 == last_pulse )
					{
						rtc_system_reset_remaining_time_last_pulse();
					}
				}
			}
		}
	}

	modbus_sensors_log_write_lock(0);
	modbus_sensors_set_comm_state( MODBUS_COMM_DISABLED );
	dlms_log_write_lock(0);
	dlms_client_set_dlms_comm_state(DLMS_COMM_DISABLED);
	modbus_sensors_set_get_data(0);
	generic_485_set_tx_on(0);
	leds_init();

	ME910_set_network_params_get(0);
	ME910_set_dk_network_params_get(0);
//	__checkReedSensor();
	MX_GPIO_Enable_Power_Mon(0);
}
#endif
/**
 * @fn void __restoreContext(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __restoreContext( void )
{
	if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	{
		/* Refresh Error */
		Error_Handler();
	}
	MX_USART2_UART_Custom_Init();
	__gpio_init();
	MX_GPDMA1_Init();
	MX_SPI2_Init();
	sFLASH_Init();
	__initSST26V();
	MX_USART1_UART_Custom_Init();
//	Tick_system_time_init(rtc_system_GetServerTime());
	Tick_init();
	rtc_system_set_t_seconds_backup();
	shutdown_Manager.shutdown_on = 0;
#if defined (UNE82326)
	une82326_reset();
#elif defined (MBUS)
	mbus_reset();
#endif
	DataLogger_Init();
	DataLogger_SendEnd();
#ifdef UNE82326
	datalog_reset_frame_tx_buffer();
#endif
	datalog_reset_buffer();
	if ( 0 == params_get_mqtt_dc_on() )
	{
		message_queue_delete(message_queue_get_elements());
	}
//	udp_delete(udp_is_get_ready());
	udp_comm_reset_buffers();
	udp_reset_buffers();
	udp_protocol_set_send_pending(0);
	udp_protocol_set_status( UDP_PROT_IDLE );
	udp_set_server_resp_type(0);
	udp_protocol_reset_datalogger_max_msgs_params();
	shutdown_reset_watchdog();
	Telit_socket_set_available( NOT_CONNECTED );
	leds_set_NET_status( LOW_POWER );
	leds_set_device_switch_on(0);
	shutdown_set_start_count(0);
	shutdown_set_alarm_pressure_check(0);
	shutdown_set_alarm_generic_sensor_check(0);
	sensor_log_write_lock(0);
	sensor_log_set_write_pulse_acc(0);

	/* Periodic sensor read and data transmission operation */
//	if (1 == params_config_get_period())
	{
		/* Time from network*/
		if (1 == Tick_cloud_time_init())
		{
			/* ADC enabled */
			if (1 == AD_GetADOn())
			{
				/* no pulses to process */
				if (0 == pulses_get_pulse_rx())
				{
					pulses_set_mutex(1);
					last_pulse = rtc_system_last_pulse();
					AD_SetGetADCData(1);
					rtc_system_reset_remaining_time_last_pulse();
					rtc_system_set_timeout_waiting_pulse();
					if (params_input_pulse_as_sensor_get_num() != 0)
					{
						generic_sensor_set_on_off(0, 1);
					}
//					if (1 == params_pulses_on())
//					{
//						i2c_sensor_task();
//					}
				}
				/* pulses */
				else
				{
					last_pulse = rtc_system_last_pulse();
					if (1 == last_pulse)
					{
						rtc_system_reset_remaining_time_last_pulse();
					}
				}
			}
		}
	}

	modbus_sensors_log_write_lock(0);
	modbus_sensors_set_comm_state(MODBUS_COMM_DISABLED);
	dlms_log_write_lock(0);
	dlms_client_set_dlms_comm_state(DLMS_COMM_DISABLED);
	modbus_sensors_set_get_data(0);
	generic_485_set_tx_on(0);
//	leds_init();
	ME910_reset_status();
	ME910_set_network_params_get(0);
	ME910_set_dk_network_params_get(0);
//	ME910_APN_reset_index();
//	ME910_server_reset_index();
	comm_serial_enable();
	__checkReedSensor();
//	__checkPowerMon();
	MX_GPIO_Enable_Power_Mon(0);
}
#endif


void shutdown_check_period_from_reset( void )
{
//	if ( 1 == params_config_get_period() )
	{
		if ( 1 == Tick_cloud_time_init() )
		{
			if ( 1 == AD_GetADOn() )
			{
				if ( 0 == pulses_get_pulse_rx() )
				{
					AD_SetGetADCData(1);
					rtc_system_reset_remaining_time();
				}
			}
		}
	}
}

/**
  * @brief Sets the sytem to SLEEP2/STANDBY mode for a period of time asleep_time.
  * @param asleep_time time to sleep -> auto wake after asleep_time has elapsed.
  * @retval None
  *
  * @note This function is called when all task has finished and the application
  * requests to go to sleep mode.
  */
void shutdown_on(uint32_t asleep_time)
{
    uint32_t initial_tick = 0;
    uint32_t elapsed = 0;

    initial_tick = Tick_Get(SECONDS);
    (void)initial_tick;
    (void)elapsed;

    /** Saves datalogger pointers */
	DataLogger_SaveLogWrRdAdress();
	/** Turns off LEDs */
	leds_LED_Off(LED_BLUE);
//	leds_LED_Off(LED_GREEN);
//	leds_LED_Off(LED_WHITE);
	/** Measures voltage and disables ADC and sensor power rail */
	battery_voltage_meas(0);
	MX_ADC4_DeInit();
	MX_GPIO_Disable_Sensor();
	dlms_client_set_send_meter(0);
	reed_sending = 0;
	if ( ( TAMPER_HARD_RESET_DAYLY == shutdown_get_tamper_param() ) || ( TAMPER_WATCHDOG == shutdown_get_tamper_param() ) )
	{
		if ( 0 == Tick_cloud_time_init() )
		{
			shutdown_set_tamper_param(TAMPER_TSEND);
			Tick_system_time_init(rtc_system_get_t_seconds_backup() /*+ params_get_timeout_connection()*/ + Tick_Get(SECONDS) - 946684800);//Tick_system_time_init(params_get_time_reset());
			rtc_system_SetServerTime(Tick_Get(SECONDS));
//			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Recover reset time:%d!!!\r\n", (int)Tick_Get( SECONDS ), (int)params_get_time_reset());
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Recover reset time:%d!!!\r\n", (int)Tick_Get( SECONDS ), (int)rtc_system_get_t_seconds_backup());
		}
	}
	if ( ( 1 == params_get_mqtt_dc_on() ) && ( 0 == mqtt_timer_get_idle_mode() ) )
	{
		shutdown_reset_watchdog();
		shutdown_set(0, rtc_system_getReadSensorAlarmCycleNextInSeconds());
	}
	/** Handles TELIT Low Power Mode (PSM or ON/OFF) */
	else if (1 == shutdown_initTelitModule())
	{
		/* In ONOFF turns off TELIT */
		if ((NBIOT_PWR_ONOFF == params_nbiotpwr_mode()))
		{
			if (( 0 == params_get_mqtt_dc_on() )
			|| (( 1 == params_get_mqtt_dc_on() ) && (ERROR_TRYING_TO_CONNECT == ME910_get_error()) ))
			{
				/* Issues a Hardware detach */
				Telit_modem_off_prc();
				uint32_t tickstart = HAL_GetTick();

				do {
					asm("nop");
				} while((GPIO_PIN_RESET == ME910_CHECK_PSM) && ((HAL_GetTick() - tickstart) < 5000));

				/* Switches of NBIOT power supply rail */
				Telit_modem_disable();
			}
		}
		/* In PSM only turns off in case of error */
		else if (NBIOT_PWR_PSM == params_nbiotpwr_mode())
		{
			if (ERROR_TRYING_TO_CONNECT == ME910_get_error())
			{
				/* Issues a Hardware detach */
				Telit_modem_off_prc();
				uint32_t tickstart_1 = HAL_GetTick();

				do {
    				asm("nop");
    			} while((GPIO_PIN_RESET == ME910_CHECK_PSM) && ((HAL_GetTick() - tickstart_1) < 5000));

				/* Switches of NBIOT power supply rail */
    			Telit_modem_disable();
			}
			else if ( 0 == params_get_mqtt_dc_on() )
			{
				ME910_setPowerDown();
			}
		}
		/* If TELIT is in connection procedure -> NOK */
		if ( (ERROR_TRYING_TO_CONNECT == ME910_get_error())
		  || ( 1 == udp_protocol_get_waiting_answer() ) )
		{
			params_maintenance_inc_number_nok_sendings();
			params_check_for_changes();
			udp_protocol_set_waiting_answer(0);
			udp_protocol_restore_pointers();
		}
	}
	/** Disables Water Quality internal power supply */
	if (1 == params_wq_on())
	{
		aqualabo_modbus_sensor_disable();
	}

#ifndef PSM_MODE
		serial_nb_module_disable();
		Telit_disable();
#ifdef ME910_MODULE
		if (1 == shutdown_initTelitModule())
		{
			Telit_modem_off_prc();
		}

		Telit_modem_disable();
#endif
#endif

	uint32_t sync = params_config_get_sync_read();
	/* Used for test mode */
	if ( 1 == dlms_client_get_resend())
	{
		asleep_time = 2;
		shutdown_set_start_count(0);
	}
	if (2 == asleep_time)
	{
		rtc_system_Set_Stop_Mode(0);
	}
	/* Periodic Sensor read and transmission operation */
	else if (1 == params_config_get_period())
	{
		shutdown_reset_start_count();
		shutdown_reset_alarm_pressure_check();
		shutdown_reset_alarm_generic_sensor_check();

		if (( ALARM_STARTED == rtc_system_getAlarmState())||( TIMER_STARTED == rtc_system_getTimerState()))
		{
			__NOP();
		}
		else
		/* ADC is enabled */
		if ((1 == AD_GetADOn())) /*&& ( 1 == params_config_get_sensor_log() )*/
		{
			/* synch read disabled */
			if (0 == sync)
			{
				if ( 0 == params_get_mqtt_dc_on() )
				{
					rtc_system_set_wake_up_time();
				}
				else
				{
					HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
					/* Set the next alarm to synchronized time according to reading time*/
//					rtc_system_SetSyncAlarm(shutdown_time_periodic_mode);
					rtc_system_SetPulseTimer();
				}
			}
			/* synch read enabled */
			else if (1 == sync)
			{
				/* pulses enabled */
				if (1 == params_pulses_on())
				{
					if ( 0 == params_get_mqtt_dc_on() )
					{
						rtc_system_set_wake_up_time();
					}
					else
					{
						HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
						/* Set the next alarm to synchronized time according to reading time*/
						rtc_system_SetSyncAlarm(shutdown_time_periodic_mode);
//						rtc_system_SetPressureOnSyncAlarm(shutdown_time_periodic_mode);
//						rtc_system_SetPulseAlarm();
						rtc_system_SetPulseTimer();
					}
				}
				/* pulses disabled */
				else
				{
					HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
					/* Set the next alarm to synchronized time according to reading time*/
					rtc_system_SetPressureOnSyncAlarm(shutdown_time_periodic_mode);
				}
			}
			else {} /* impossible case synch read is either 0 or 1 */
		}
		/* ADC is not enabled */
		else
		{
			/* synch read disabled */
			if (0 == sync)
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Sleep Period : %d.\r\n",
						(int)Tick_Get(SECONDS), (int)shutdown_time_periodic_mode);
				rtc_system_Set_Stop_Mode(shutdown_time_periodic_mode);
			}
			/* synch read enabled */
			else
			{
				HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
				if ( (0 != dlms_client_get_read_time() ) && ( 1 == shutdown_check_dlms_count_send_frame() ) )
				{
					rtc_system_SetDelaySendAlarm();
					shutdown_synch_count_params();
				}
				else
				{
					rtc_system_SetSyncAlarm(shutdown_time_periodic_mode);
				}
			}
		}
	}
	/* Window based sensing and transmitting operation */
	else
	{
		rtc_system_set_wake_up_time();
	}

	/* Clears relevant flags */
	pulses_set_pulse_rx(0);
	pulses_set_interruption(0);
	pulses_set_mutex(0);
	params_set_pulse_write(0);
	udp_protocol_set_end_sensor_messages(0);
	AD_setEnd(0);
	test_prod_run(0);
	rtc_system_set_t_seconds_backup();
	on_demand_backup = 0;
	if ( 1 == params_get_mqtt_dc_on() )
	{
		if ( ( asleep_time != 2 ) && ( mqtt_timer_get_idle_mode() != 0 )/*( 1 == mqtt_timer_get_idle_mode() )*/ )
		{
			shutdown_Manager.init_TelitModule = 1;
			mqtt_start();
			mqtt_timer_set_idle_mode(0);
		}
		if ( 0 == mqtt_timer_get_idle_mode() )
		{
			shutdown_Manager.init_TelitModule = 1;
		}
//		rtc_system_setAlarmOccured(0);
		shutdown_set_start_count(0);
		shutdown_set_check_count_mqtt(0);
		shutdown_set_sending_mqtt_adc_on(0);
		shutdown_Manager.shutdown_on = 0;
//		message_queue_delete(message_queue_get_elements());
		mqtt_set_send_activation(0);
		mqtt_set_reply_get_topic(0);
		modbus_sensors_set_comm_state( MODBUS_COMM_DISABLED );
		modbus_sensors_set_get_data(0);
		generic_485_reset_on_demand();
		udp_protocol_set_on_periodic_profile(0);
		dlms_client_set_command(DLMS_HES_COMMAND_NONE);
		udp_protocol_set_on_billing_profile(0);
		udp_protocol_set_on_event_profile(0);
		udp_protocol_reset_profile_sending();
		udp_protocol_set_on_demand_command(0);
		dlms_client_set_disconnected(0);
		shutdown_set_tamper_param_cmd(TAMPER_MQTT_COMMAND_ERROR);
		dlms_client_reset_id_request_queue();//memset(dlms_client_get_dlms_id_request(), 0 , 40*sizeof(char));
		memset(dlms_client_get_dlms_data_cmd_on_demand(), 0 , 40*sizeof(char));
		memset(datalog_get_idRequest(), 0 , 40*sizeof(char));
		memset(dlms_log_get_dataCmdOnDemand(), 0 , 40*sizeof(char));
		dlms_client_reset_id_request_index();
		dlms_client_table_reset_devices_from_cmd();
		DataLogger_Init();
		DataLogger_SendEnd();
		CircularBuffer_Reset(dlms_client_get_send_msg_queue());
		CircularBuffer_Reset(dlms_client_get_write_msg_queue());
		CircularBuffer_Reset(dlms_client_get_read_msg_queue());
	}
	else
	{
		if (NBIOT_PWR_PSM == params_nbiotpwr_mode())
		{
			MX_GPIO_Enable_Power_Mon(1);
			ME910_setTimeToWait( 3 );
			Tick_update_tick( TICK_TELIT);
		}
		shutdown_set_power_mon(0);
	}
	LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> Sleep time.\r\n", (int)Tick_Get( SECONDS ));
	if ( 0 == params_get_mqtt_dc_on() )
	{
		//#ifndef DEBUG_MODE
		/** PIN CONFIGURATION FOR LOW POWER MODE STOP SELECTED NO DEBUGG MODE -> real application */
#ifdef PWR_STOP
//		MX_GPDMA1_DeInit();
		MX_USART2_USART_DeInit();
//		MX_USART3_USART_DeInit();
//		MX_SPI2_DeInit();
//		__HAL_RCC_TIM3_CLK_DISABLE();
//		__HAL_RCC_TIM16_CLK_DISABLE();
//		MX_ICACHE_DeInit();
//		HAL_PWR_DisablePVD();
		HAL_PWREx_EnableUltraLowPowerMode();
		HAL_PWREx_DisableVddIO2();
		HAL_PWREx_DisableVddA();
//		HAL_PWREx_DisableMonitoring();
//		HAL_PWREx_ConfigSRDDomain(PWR_SRD_DOMAIN_STOP);
//		HAL_RCCEx_DisableLSECSS();
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Configure all GPIO as analog to reduce current consumption on non used IOs */
	/* Enable GPIOs clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

#ifdef STOP3
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2_HIGH_1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1_HIGH_0);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN7_LOW_1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN8_LOW_1);

	/* Clear all related wakeup flags*/
	__HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_ALL_FLAG);
#endif

	GPIO_InitStructure.Mode  = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	GPIO_InitStructure.Pin   = GPIO_PIN_ALL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	if (1 == params_pulses_on())
	{
#ifndef DEBUG_MODE
		if (1 == pulses_get_input_num())
		{
			GPIO_InitStructure.Pin =
					GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
					GPIO_PIN_5  			  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
					GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
					GPIO_PIN_15
					;
			if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
				GPIO_InitStructure.Pin =
						GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
						GPIO_PIN_5  			  | GPIO_PIN_7		          | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
		}
		else
		{
			GPIO_InitStructure.Pin =
					GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
					GPIO_PIN_5  			                | GPIO_PIN_8  | GPIO_PIN_9  |
					GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
					GPIO_PIN_15
					;
			if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
				GPIO_InitStructure.Pin =
						GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
						GPIO_PIN_5  			                	          | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
		}
#else
		GPIO_InitStructure.Pin =
							  GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
				GPIO_PIN_5  						    | GPIO_PIN_8  | GPIO_PIN_9  |
				GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
				GPIO_PIN_15
				;
		if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
			GPIO_InitStructure.Pin =
								  GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
					GPIO_PIN_5  						    			  | GPIO_PIN_9  |
					GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
					GPIO_PIN_15
					;
		}
#endif
#ifdef STOP3
		/* Enable pull up on wakeup pin PA7-PA6 */
		HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_7|PWR_GPIO_BIT_6);

		 /* Enable pull-up and pull-down configuration for CPU1 */
		HAL_PWREx_EnablePullUpPullDownConfig();

		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH_1);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH_0);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_LOW_1);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN8_LOW_1);
#endif
	}
	else
	{
#ifndef DEBUG_MODE
		GPIO_InitStructure.Pin =
					  GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
		GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
	    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
		GPIO_PIN_15
		;
		if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
			GPIO_InitStructure.Pin =
						  GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
			GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7                | GPIO_PIN_9  |
		    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
			GPIO_PIN_15
			;
		}
#else
		GPIO_InitStructure.Pin =
					  GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
		GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
	    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
		GPIO_PIN_15
		;
		if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
			GPIO_InitStructure.Pin =
						  GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
			GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  			  | GPIO_PIN_9  |
		    GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
			GPIO_PIN_15
			;
		}
#endif
	}
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Mode  = GPIO_MODE_ANALOG;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
		if ( 0 == params_pulses_on() )
		{
#ifndef DEBUG_MODE
			if ( 1 == params_get_input_alarm_sensor() )
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  |               GPIO_PIN_2  | GPIO_PIN_3  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
			else
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
#else
			GPIO_InitStructure.Pin   =
					GPIO_PIN_0  | /*GPIO_PIN_1  |*/ GPIO_PIN_2  |
					GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
					GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
					GPIO_PIN_15
					;
#endif
		}
		else
		{
#ifndef DEBUG_MODE
			if ( 1 == params_get_input_alarm_sensor() )
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  |               GPIO_PIN_2  | GPIO_PIN_3  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 |  			  GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
			else
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 |  			  GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
#else
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  | /*GPIO_PIN_1  |*/ GPIO_PIN_2  | /*GPIO_PIN_3  |*/
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 |  			  GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
#endif
		}
	}
	else
	{
		if ( 0 == params_pulses_on() )
		{
#ifndef DEBUG_MODE
			if ( 1 == params_get_input_alarm_sensor() )
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  |               GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
			else
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
#else
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  | /*GPIO_PIN_1  |*/ GPIO_PIN_2  | /*GPIO_PIN_3  |*/GPIO_PIN_4  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
#endif
		}
		else
		{
#ifndef DEBUG_MODE
			if ( 1 == params_get_input_alarm_sensor() )
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  |               GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 |  			  GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
			else
			{
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 |  			  GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
			}
#else
				GPIO_InitStructure.Pin   =
						GPIO_PIN_0  |/* GPIO_PIN_1  |*/ GPIO_PIN_2  | /*GPIO_PIN_3  |*/GPIO_PIN_4  |
						GPIO_PIN_5  | GPIO_PIN_6  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9  |
						GPIO_PIN_10 | GPIO_PIN_11 |  			  GPIO_PIN_13 | GPIO_PIN_14 |
						GPIO_PIN_15
						;
#endif
		}
	}
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Disable GPIOs clock */
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOH_CLK_DISABLE();

#ifdef STOP3
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN7_HIGH_3);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_LOW_1);
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_LOW_0);
#endif
//#endif
#endif
	}
	if ( 0 == params_get_mqtt_dc_on() )
	{
#if defined(PWR_STDBY)
		/** Clear all related wakeup flags */
		__HAL_PWR_CLEAR_FLAG( PWR_FLAG_SB );
		__HAL_PWR_CLEAR_FLAG( PWR_FLAG_WU );
		__HAL_RTC_CLEAR_FLAG( RTC_FLAG_WUTF );
		HAL_DBGMCU_EnableDBGSleepMode();
		HAL_DBGMCU_EnableDBGStopMode();
		HAL_DBGMCU_EnableDBGStandbyMode();
		HAL_PWR_EnterSTANDBYMode();
		// this should not happen...
		do {
			elapsed  = Tick_Get(SECONDS) - initial_tick;
		} while( elapsed < 10 );

		if (elapsed >= 10) {
			asm("nop");
			// something went wrong, let's reset...
			NVIC_SystemReset();
		}
#elif defined(PWR_STOP)
#ifdef DEBUG_MODE
		HAL_DBGMCU_DisableDBGIWDG();
//	    HAL_FLASH_Freeze_IWDG();
//		HAL_DBGMCU_EnableDBGSleepMode();
		HAL_DBGMCU_EnableDBGStopMode();
		HAL_DBGMCU_EnableDBGStandbyMode();
#endif
//		rtc_system_ReconfigStopMode(); //TOASK hace falta?
		/* Stops tick interruptions to avoid CPU wake up every tick */
		HAL_SuspendTick();
		/* STOP3 wake up global interrupt configuration */
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM1_FULL_STOP);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM2_FULL_STOP);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM3_FULL_STOP);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_SRAM4_FULL_STOP);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_ICACHE_FULL_STOP);
		HAL_PWREx_EnableRAMsContentStopRetention(PWR_DMA2DRAM_FULL_STOP);

#ifdef STOP3
//		/* Enable pull up on wakeup pin PA6-PA7 */
//		HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, PWR_GPIO_BIT_7|PWR_GPIO_BIT_6);
//
//		 /* Enable pull-up and pull-down configuration for CPU1 */
//		HAL_PWREx_EnablePullUpPullDownConfig();

		/* Clear all related wakeup flags*/
		__HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_ALL_FLAG);

		__HAL_RCC_GPIOA_CLK_SLEEP_ENABLE();
		__HAL_RCC_SRAM2_CLK_SLEEP_ENABLE();
#endif

#if 1
		__HAL_RCC_SPI3_CLKAM_DISABLE();
		__HAL_RCC_LPUART1_CLKAM_DISABLE();
		__HAL_RCC_I2C3_CLKAM_DISABLE();
		__HAL_RCC_LPTIM1_CLKAM_DISABLE();
		__HAL_RCC_LPTIM3_CLKAM_DISABLE();
		__HAL_RCC_LPTIM4_CLKAM_DISABLE();
		__HAL_RCC_ADC4_CLKAM_DISABLE();
		__HAL_RCC_DAC1_CLKAM_DISABLE();
		__HAL_RCC_LPGPIO1_CLKAM_DISABLE();
		__HAL_RCC_OPAMP_CLKAM_DISABLE();
		__HAL_RCC_COMP12_CLKAM_DISABLE();
		__HAL_RCC_VREF_CLKAM_DISABLE();
		__HAL_RCC_ADF1_CLKAM_DISABLE();
		__HAL_RCC_LPDMA1_CLKAM_DISABLE();

		__HAL_RCC_SPI3_CLK_SLEEP_DISABLE();
		__HAL_RCC_LPUART1_CLK_SLEEP_DISABLE();
		__HAL_RCC_I2C3_CLK_SLEEP_DISABLE();
		__HAL_RCC_LPTIM1_CLK_SLEEP_DISABLE();
		__HAL_RCC_LPTIM3_CLK_SLEEP_DISABLE();
		__HAL_RCC_LPTIM4_CLK_SLEEP_DISABLE();
		__HAL_RCC_ADC4_CLK_SLEEP_DISABLE();
		__HAL_RCC_DAC1_CLK_SLEEP_DISABLE();
		__HAL_RCC_LPGPIO1_CLK_SLEEP_DISABLE();
		__HAL_RCC_OPAMP_CLK_SLEEP_DISABLE();
		__HAL_RCC_COMP_CLK_SLEEP_DISABLE();
		__HAL_RCC_VREF_CLK_SLEEP_DISABLE();
		__HAL_RCC_ADF1_CLK_SLEEP_DISABLE();
		__HAL_RCC_LPDMA1_CLK_SLEEP_DISABLE();

		__HAL_RCC_GPIOD_CLK_SLEEP_DISABLE();
		__HAL_RCC_GPIOH_CLK_SLEEP_DISABLE();
#endif

#ifdef STOP3
		HAL_NVIC_SetPriority(PWR_S3WU_IRQn, 7, 7);
		HAL_NVIC_EnableIRQ(PWR_S3WU_IRQn);
		/** STOP3 MODE WFE */
		HAL_PWREx_EnterSTOP3Mode(PWR_STOPENTRY_WFE);
#else
		/** STOP2 MODE WFE */
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);
#endif

		/** WAKES UP FROM STOP2 MODE  */
		SystemInit();
		SystemCoreClockUpdate();
		SystemClock_Config();
		/* Resume tick interruptions */
		HAL_ResumeTick();
//		rtc_system_ReconfigStopMode(); //TOASK hace falta?

#ifdef STOP3
		HAL_NVIC_DisableIRQ(PWR_S3WU_IRQn);
#endif
		__restoreContext();
		LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> Wake-up time\n", (int)Tick_Get( SECONDS ));
#endif
	}
}

uint32_t shutdown_is_shutdown_on( void )
{
	return shutdown_Manager.shutdown_on;
}

/**
 * @fn void shutdown_reset_watchdog(void)
 * @brief Sets the flag to realoads the watchdog counter
 */
void shutdown_reset_watchdog(void)
{
	shutdown_Manager.reset_shutdown_wchdg = 1;
}

/**
 * @fn void shutdown_restart_watchdog(void)
 * @brief Restarts the watchdog
 */
void shutdown_restart_watchdog(void)
{
	shutdown_Manager.reset_shutdown_wchdg = 0;
}

/**
 * @fn void shutdown_set_wchdg_time(uint32_t)
 * @brief Resets the watchdog with a new watchdog time.
 *
 * @param time
 */
void shutdown_set_wchdg_time(uint32_t time)
{
	/** Sets the flag to reload the watchdog */
	shutdown_Manager.reset_shutdown_wchdg = 1;
	/** Sets the new watchdog timer */
	shutdown_Manager.wchdg_time           = time;
}

/**
 * @fn void shutdown_watchdog(void)
 * @brief Software shutdown
 *
 */
void shutdown_watchdog( void )
{
	static uint32_t tick_sec = 0;
	static uint8_t  tick0 = 0;
	static uint8_t  tick1 = 0;

	/* After time is get from server restarts the watchdog timer */
	if (1 == rtc_system_InitByServer())
	{
		if ((tick0 == 0) || (1 == shutdown_Manager.reset_shutdown_wchdg))
		{
			tick0     = 1;
			tick_sec  = Tick_Get(SECONDS);
			shutdown_Manager.reset_shutdown_wchdg = 0;
		}
		/* Watchdog timer has elapsed */
		if (Tick_Get(SECONDS) > tick_sec)
		{
			if ((Tick_Get(SECONDS) - tick_sec) >  shutdown_Manager.wchdg_time)
			{
				shutdown_set(1, params_config_read_time());
			}
		}
		else
		{
			tick_sec  = Tick_Get(SECONDS);
		}
	}
	/* No time from server starts the watchdog timer */
	else
	{
		if ((tick1 == 0) || (1 == shutdown_Manager.reset_shutdown_wchdg))
		{
			tick1     = 1;
			tick_sec  = Tick_Get(SECONDS);
			shutdown_Manager.reset_shutdown_wchdg = 0;
		}
		/* Watchdog timer has elapsed*/
		if (Tick_Get(SECONDS) > tick_sec)
		{
			if ((Tick_Get(SECONDS) - tick_sec) > (shutdown_Manager.wchdg_time))
			{
				shutdown_set(1, params_config_read_time());
			}
		}
		else
		{
			tick_sec  = Tick_Get( SECONDS );
		}
	}
}

/**
 * @fn uint32_t __checkMeasurementsState(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
static uint32_t __checkMeasurementsState( void )
{
	uint32_t ret = 0, end_meter = 0, end_modbus = 0, end_sensor = 0;
#if defined(UNE82326)
	if ( 1 == une82326_get_start_comm() ) {
		if ( 1 == une82326_get_end_comm() ) {
			end_meter = 1;
		} else {
			end_meter = 0;
		}
	} else {
		end_meter = 1;
	}
#elif defined(MBUS)
	if ( 1 == mbus_get_start_comm() ) {
		if ( 1 == mbus_get_end_comm() ) {
			end_meter = 1;
		} else {
			end_meter = 0;
		}
	} else {
		end_meter = 1;
	}
#endif
	if ( ( 1 == aqualabo_modbus_get_enable() ) || ( 1 == generic_485_get_enable() ) ) {
		if ( MODBUS_COMM_END == modbus_sensors_get_comm_state() ) {
			end_modbus = 1;
		} else {
			end_modbus = 0;
		}
	} else {
		end_modbus = 1;
	}
	if ( 1 == AD_GetADCData() ) {
		if ( 1 == AD_getEnd() ) {
			end_sensor = 1;
		} else {
			end_sensor = 0;
		}
	} else {
		end_sensor = 1;
	}

	ret = end_meter & end_modbus & end_sensor;

	return ret;
}

static void __checkPSMMode( void )
{
	if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
#if defined(UNE82326)
		if ( ( 1 == __checkMeasurementsState() )/*une82326_get_end_comm() )*/ && ( 1 == Tick_cloud_time_init() ) ) {
#elif defined(MBUS)
			if ( ( 1 == __checkMeasurementsState() )/*( 1 == mbus_get_end_comm() )*/ && ( 1 == Tick_cloud_time_init() ) ) {
#endif
				if ( (Telit_wait_exit_stop_mode()) && (2 != ME910_getPSMMode() ) ) {
					if ( 1 == shutdown_initTelitModule() ) {
						if ( 1 == rtc_system_InitByServer() ) {
							shutdown_reset_watchdog();
							if ( GPIO_PIN_RESET == ME910_CHECK_PSM ) {
								ME910_setTimeToWait( 2 );
								Tick_update_tick( TICK_TELIT);
								ME910_setPSMMode(2);
								shutdown_setInitTelitModule( 1 );
								leds_set_NET_status( NB_N );
							} else if ( 1 == Telit_wake_up_from_PSM() ) {
								ME910_setTimeToWait( 2 );
								Tick_update_tick( TICK_TELIT);
								ME910_setPSMMode(2);
								shutdown_setInitTelitModule( 1 );
								leds_set_NET_status( NB_N );
							} else {
								ME910_reinit_device();
								shutdown_setInitTelitModule( 0 );
								shutdown_set( 1, params_config_read_time() );
							}
						} else {
							shutdown_setInitTelitModule( 1 );
							shutdown_reset_watchdog();
							leds_set_NET_status( NB_N );
						}
					}
				}
			}
		}
}


/**
 * @fn void shutdown_task(void)
 * @brief Shutdown task, decides whether the system has to go to low power
  * or remain active.
 *
 * @pre
 * @post
 */
void shutdown_task(void)
{

	/** Checks for software watchdog timer elapse */
	shutdown_watchdog();
	/** Checks for pressure alarm to trigger an alarm transmission */
	shutdown_check_pressure_alarm();
	shutdown_check_generic_sensor_alarm();

	/** Goes to SLEEP2 mode if shutdown_on flag is set */
	if (shutdown_Manager.shutdown_on)
	{
		/* Sets the system to sleep for .shutdown_time */
		shutdown_on(shutdown_Manager.shutdown_time);
	}

	/** Decides whether the Telit needs to be initilialized */

	/* Periodic sampling and sending operation mode */
	if (1 == params_config_get_period())
	{
		shutdown_Manager.init_TelitModule = shutdown_get_send_counter();
	}
	/* Window based sampling and sending operation mode */
	else
	{
		shutdown_Manager.init_TelitModule = shutdown_init_telit_module();
	}
	/**/
	__checkPSMMode();
}

/**
  * @brief Enables/Disables the shutdown flag that sets the system to sleep for
  * a time period of time
  * @param on
  * 	@arg 0 -> Do not go to SLEEP
  * 	@arg 1 -> Go to SLEEP
  * @param time time elapsed before waking up
  * @retval None
  */
void shutdown_set(uint32_t on, uint32_t time)
{
	if (1 == on) {
		LOGLIVE(LEVEL_2, "LOGLIVE> %d Set Shutdown ON.\r\n", (int)Tick_Get( SECONDS ));//__NOP();
	}
	shutdown_Manager.shutdown_on   = on;
	shutdown_Manager.shutdown_time = time;
}

/**
 * @brief Flag to allow avoid to enter to shutdown_get_send_counter fn
 */
static uint8_t start_count = 0;

/**
 * @fn void shutdown_set_start_count(uint8_t)
 * @brief Set the start_count flag to avoid to enter to shutdown_get_send_counter
 * @param _start_count
 */
void shutdown_set_start_count(uint8_t _start_count)
{
	start_count = _start_count;
}

/**
 * @fn void shutdown_reset_start_count(void)
 * @brief Resets the start_count
 */
void shutdown_reset_start_count( void )
{
	start_count = 0;
}

/**
 * @brief Flag to show whether the alarm pressure has been already checked.
 */
static uint8_t alarms_pressure_check = 0;

/**
 * @fn void shutdown_set_alarm_pressure_check(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _alarms_pressure_check
 */
void shutdown_set_alarm_pressure_check( uint8_t _alarms_pressure_check )
{
	alarms_pressure_check = _alarms_pressure_check;
}

/**
 * @fn void shutdown_reset_alarm_pressure_check(void)
 * @brief Resets the pressure alarm check flag.
 *
 */
void shutdown_reset_alarm_pressure_check( void )
{
	alarms_pressure_check = 0;
}

static uint8_t alarm_pressure_on = 0; /*!< Flag to show whether the alarm pressure is enabled or disabled.*/
/**
 * @fn void shutdown_check_pressure_alarm(void)
 * @brief
 *
 * @pre
 * @post
 */
void shutdown_check_pressure_alarm( void )
{
	static uint32_t pressure_alarm_counter = 0;

	/*alarm pressure not yet checked */
	if ((0 == alarms_pressure_check) && (1 == Tick_cloud_time_init()))
	{
		if (1 == AD_getEnd())
		{
			alarms_pressure_check = 1;
			/** Checks for pressure alarm and triggers the Extended or Reduced format transmission */
			if ((sensor_log_GetOverPressureAlarm() != 0) || (sensor_log_GetLowPressureAlarm() != 0))
			{
				if (0 == alarm_pressure_on)
				{
					alarm_pressure_on = 1;

					/* Extended frame format */
					if (1 == params_config_get_sensor_log())
					{
						message_queue_write(SEND_NETWORK_PARAMS);
						message_queue_write(SEND_SENSOR);
					}
					/* Reduced frame format */
					else
					{
						message_queue_write(SEND_NETWORK_PARAMS);
					}

					LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> -----------ALARM THRESHOLD TRIGGERED----------.\r\n", (int)Tick_Get( SECONDS ));

					shutdown_set_tamper_param(TAMPER_LEVEL_ALARM_ON);
					shutdown_set(0, rtc_system_getReadSensorAlarmCycleNextInSeconds());
					shutdown_setInitTelitModule(1);
				}
			}
			/* Sends pressure alarm every 10 pressure measures */
			else if (1 == alarm_pressure_on)
			{
				if (pressure_alarm_counter++ > 10)
				{
					pressure_alarm_counter = 0;
					alarm_pressure_on = 0;
				}
			}
		}
	}
}

/**
 * @brief Flag to show whether the alarm pressure has been already checked.
 */
static uint8_t alarms_sensor_check = 0;

/**
 * @fn void shutdown_set_alarm_pressure_check(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _alarms_pressure_check
 */
void shutdown_set_alarm_generic_sensor_check( uint8_t _alarms_sensor_check )
{
	alarms_sensor_check = _alarms_sensor_check;
}

/**
 * @fn void shutdown_reset_alarm_pressure_check(void)
 * @brief Resets the pressure alarm check flag.
 *
 */
void shutdown_reset_alarm_generic_sensor_check( void )
{
	alarms_sensor_check = 0;
}

static uint8_t alarm_sensor_on[NUM_INPUTS_SENSOR] = {0,0,0,0}; /*!< Flag to show whether the alarm pressure is enabled or disabled.*/
/**
 * @fn void shutdown_check_pressure_alarm(void)
 * @brief
 *
 * @pre
 * @post
 */
void shutdown_check_generic_sensor_alarm( void )
{
	static uint32_t generic_sensor_alarm_counter = 0;

	if ((0 == alarms_sensor_check) && (1 == Tick_cloud_time_init()))
	{
		uint32_t i = 0;
		for (i = 0; i < params_input_pulse_as_sensor_get_num(); i++ )
		{
			if (1 == generic_sensor_get_end_conv(i))
			{
				alarms_sensor_check = 1;
				if ((generic_sensor_log_GetOverGenericSensorAlarm(i) != 0) || (generic_sensor_log_GetLowGenericSensorAlarm(i) != 0))
				{
					if (0 == alarm_sensor_on[i])
					{
						alarm_sensor_on[i] = 1;
						generic_sensor_set_burst_mode(i,1);

						message_queue_write(SEND_NETWORK_PARAMS);
						shutdown_set_tamper_param(TAMPER_LEVEL_ALARM_ON);
						shutdown_set(0, rtc_system_getReadSensorAlarmCycleNextInSeconds());
						shutdown_setInitTelitModule(1);
					}
				}
				else if (1 == alarm_sensor_on[i])
				{
					if (generic_sensor_alarm_counter++ > 10)
					{
						generic_sensor_alarm_counter = 0;
						alarm_sensor_on[i] = 0;
						generic_sensor_set_burst_mode(i,0);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Reset alarm on, start checking for alarms. \r\n", (int)Tick_Get( SECONDS ));
					}
				}
			}
			i2c_sensor_set_curr_sensor(i2c_sensor_get_curr_sensor() + 1);
			if ( i2c_sensor_get_curr_sensor() >= i2c_sensor_get_num_sensors() )
			{
				i2c_sensor_set_curr_sensor(0);
			}
		}
	}
}

static uint8_t __checkRTCAlarm( uint8_t alarm_hour )
{
	uint8_t ret = 0;
	if ( ( alarm_hour == Sys_time.Time.Hours ) || ( alarm_hour == Sys_time.Time.Hours - 1 ) ) {
		ret = 1;
	}
	return ret;
}

static uint8_t __checkRTCParam( uint8_t alarm_hour )
{
	uint8_t ret = 0;
	if ( ( alarm_hour == Sys_time.Time.Hours ) || ( alarm_hour == Sys_time.Time.Hours - 1 ) ) {
		ret = 1;
	}
	return ret;
}

static uint8_t __greaterCycle( uint32_t *array, uint32_t *factor_1, uint32_t *factor_2 )
{
#define MODBUS_SENSOR  (4)
#define ALARM     	   (2)
#define SENSOR    	   (1)
#define CYCLE_NUM 	   (3)
	uint32_t alarm_cycle_next  = rtc_system_getReadAlarmCycleNextInSeconds();
	uint32_t sensor_cycle_next = rtc_system_getReadSensorAlarmCycleNextInSeconds();
	uint32_t modbus_cycle_next = rtc_system_getReadModbusAlarmCycleNextInSeconds();
	uint8_t  i = 3, ret = 0;

	array[0] = sensor_cycle_next;
	array[1] = modbus_cycle_next;
	array[2] = alarm_cycle_next;

	// Sort cycle hours array.
	for ( uint8_t ii = 0; ii < i; ii++ ) {
		for ( uint8_t j = 0; j < i; j++ ) {
			if ( array[j] != 0xFFFFFFFF ) {
				if ( array[j] > array[ii] ) {
					int tmp   = array[ii];
					array[ii] = array[j];
					array[j]  = tmp;
				}
			}
		}
	}

	if ( 0 == array[0] ) {
		array[0] = 1;
	}
	*factor_1 = array[1]/array[0];
	*factor_2 = array[2]/array[0];

	for ( i = 0; i < 3; i++ ) {
		if ( sensor_cycle_next == array[i] ) {
			sensor_cycle_next = 0xFFFFFFFF;
			array[i]          = SENSOR;
		} else if ( modbus_cycle_next == array[i] ) {
			modbus_cycle_next = 0xFFFFFFFF;
			array[i]          = MODBUS_SENSOR;
		} else if ( alarm_cycle_next == array[i] ) {
			alarm_cycle_next = 0xFFFFFFFF;
			array[i]         = ALARM;
		}
	}

	return ret;
}

static uint8_t __chooseAlarmAndSensorCycleAlarm( void )
{
#define MODBUS_SENSOR (4)
#define ALARM         (2)
#define SENSOR        (1)

	static uint32_t counter_alarm, counter_modbus, counter_sensor, i;
	       uint32_t factor_1, factor_2, factor_alarm = 0, factor_sensor = 0, factor_modbus = 0, init_read_on = 0;
	       uint32_t cycle_array[3];

	if ( ALARM_INIT_READ == rtc_system_getCurrentAlarmFromBKUP() ) {
		if ( 1 == __checkRTCAlarm( rtc_system_getReadAlarmInitNext() ) ) {
			init_read_on = 1;
		} else {
			init_read_on = 0;
		}
	}

	if ( ( ( 0 == init_read_on ) && ( ALARM_CYCLE == rtc_system_getCurrentAlarmFromBKUP() ) )
		&& ( 0 == pulses_get_interruption() )
	    && ( ( ALARM_SENSOR_CYCLE == rtc_system_getCurrentSensorAlarm() ) || ( ALARM_MODBUS_CYCLE == rtc_system_getCurrentModbusAlarm() ) )
	   ) {
		__greaterCycle( cycle_array, &factor_1, &factor_2 );
		for ( i = 0; i < 3; i++ ) {
			if ( ALARM == cycle_array[i] ) {
				if ( 1 == i ) {
					factor_alarm = factor_1;
				} else if ( 2 == i ) {
					factor_alarm = factor_2;
				} else if ( 0 == i ) {
					factor_alarm = 1;
				}
				if ( counter_alarm++ >= ( factor_alarm - 1 ) ) {
					counter_alarm = 0;
					if ( ( 0 == aqualabo_modbus_get_enable() ) && ( 0 == generic_485_get_enable() ) ) {
#if defined(UNE82326)
						une82326_set_start_comm(1);
#elif defined(MBUS)
						mbus_set_start_comm(1);
#endif
					} else {
//#if defined(UNE82326)
//						une82326_set_start_comm(0);
//#elif defined(MBUS)
//						mbus_set_start_comm(0);
//#endif
					}
				} else {
//#if defined(UNE82326)
//						une82326_set_start_comm(0);
//#elif defined(MBUS)
//						mbus_set_start_comm(0);
//#endif
				}
			} else if ( SENSOR ==  cycle_array[i] ) {
				if ( ALARM_SENSOR_CYCLE == rtc_system_getCurrentSensorAlarm() ) {
					if ( 1 == i ) {
						factor_sensor = factor_1;
					} else if ( 2 == i ) {
						factor_sensor = factor_2;
					} else if ( 0 == i ) {
						factor_sensor = 1;
					}
					if ( counter_sensor++ >= ( factor_sensor - 1 ) ) {
						counter_sensor = 0;
						AD_SetGetADCData(1);
						rtc_system_reset_remaining_time();
					} else {
						AD_SetGetADCData(0);
					}
				}
			} else if ( MODBUS_SENSOR == cycle_array[i] ) {
				if ( ALARM_MODBUS_CYCLE == rtc_system_getCurrentModbusAlarm() ) {
					if ( 1 == i ) {
						factor_modbus = factor_1;
					} else if ( 2 == i ) {
						factor_modbus = factor_2;
					} else if ( 0 == i ) {
						factor_modbus = 1;
					}
					if ( counter_modbus++ >= ( factor_modbus - 1 ) ) {
						counter_modbus = 0;
						if ( 1 == params_wq_on() ) {
							aqualabo_modbus_enable(1);
							modbus_sensors_set_get_data(1);
							aqualabo_set_comm_state(1);
							rtc_system_reset_remaining_time();
						} else if ( MODBUS_RAW == generic_485_get_type() ) {
							generic_485_set_enable(1);
							modbus_sensors_set_get_data(1);
							generic_485_set_comm_state(1);
							modbus_sensors_init_task();
							rtc_system_reset_remaining_time();
						}
					} else {
						if ( 1 == params_wq_on() ) {
							aqualabo_modbus_enable(0);
							modbus_sensors_set_get_data(0);
							aqualabo_set_comm_state(0);
						} else if ( MODBUS_RAW == generic_485_get_type() ) {
							generic_485_set_enable(1);
							modbus_sensors_set_get_data(1);
							modbus_sensors_init_task();
							generic_485_set_comm_state(1);
						}
					}
				}
			}
		}
	}
	return 0;
}

uint32_t num_pulse_readings = 0;
void shutdown_inc_num_readings( void )
{
	num_pulse_readings++;
}

uint8_t shutdown_init_telit_module( void )
{
	       uint32_t send_params = 0;
	static uint32_t end_read    = 0;
	static uint32_t got_last_pulse = 0;  /*!< Flag indicates still not  */
	enum{OFF = 0, ON = 1};

	if (1 == pulses_get_pulse_rx())
	{
		rtc_system_reset_timeout_waiting_pulse();
	}

	if (0 == start_count)
	{
//		if ( (1 == params_get_mqtt_dc_on()) && (1 == rtc_system_getAlarmOccured()) )
//		{
//			if ( (1 == AD_GetADOn()) && ( 0 == dlms_client_get_resend() ) )
//			{
//				check_count_mqtt_on = 1;
//				if ( 1 == count )
//				{
//					sending_mqtt_adc_on = 1;
//					shutdown_mqtt_restoreContext();
//				}
//				else
//				{
//					if ( ( count != 0 )
//					|| ( ( 0 == count ) && ( 0 == mqtt_is_state_idle() ) ) )
//					{
//						shutdown_mqtt_restorePressureContext();
//					}
//				}
//			}
//			else
//			{
//				check_count_mqtt_on = 0;
//				shutdown_mqtt_restoreContext();
//			}
//		}
		if ((1 == params_get_mqtt_dc_on()) && (1 == params_pulses_on()))
		{
			pulses_set_mutex(1);
			pulses_set_pulse_rx(0);
			AD_SetGetADCData(1);
		}
		else
		{
			/* If pulses has occurred and there is still time to go to sleep */
			if ((1 == pulses_get_pulse_rx()) && (1 == Tick_cloud_time_init()) && (0 == last_pulse) && (0 == rtc_system_get_timeout_waiting_pulse()))
			{
//				params_maintenance_inc_number_readings();
				AD_SetGetADCData(0);
				Tick_Force_1Second_Task();//Forces one second task execution in order to save battery.
				/* Triggers the system to go sleep */
				shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
				start_count = 1; //avoid to enter here again
				shutdown_Manager.init_TelitModule = 0;
				got_last_pulse = 0;
				/* Exits with 0 */
				return shutdown_Manager.init_TelitModule;
			}

			if (1 == params_pulses_on())
			{
//				num_pulse_readings++;
				params_maintenance_set_number_readings(num_pulse_readings);
			}
			/* last pulse means needs to go to sleep */
			if ((1 == last_pulse) && (1 == params_pulses_on()))
			{
				if (0 == got_last_pulse)
				{
					pulses_set_mutex(1);
					pulses_set_pulse_rx(0);
					AD_SetGetADCData(1);
					got_last_pulse = 1;
				}
				else
				{
					AD_SetGetADCData(0);
					Tick_Force_1Second_Task();//TOASK Forces one second task execution in order to save battery.
					/* Triggers to go to sleep */
					shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
					start_count = 1;
					shutdown_Manager.init_TelitModule = 0;
					/*Exits with 0*/
					return shutdown_Manager.init_TelitModule;
				}
				last_pulse = 0;
			}
		}
		if (0 == shutdown_Manager.init_TelitModule)
		{
			start_count = 1;
			if (1 == Tick_cloud_time_init())
			{
				if (ALARM_SEND == rtc_system_getCurrentAlarmFromBKUP())
				{
					if (1 == __checkRTCAlarm( rtc_system_getSendAlarmSendNext()))
					{
						shutdown_setInitTelitModule(1);
#if defined(UNE82326)
						une82326_set_start_comm(1);
#elif defined(MBUS)
						mbus_set_start_comm(1);
						meter_send = 1;
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ALARM_SEND. \r\n", (int)Tick_Get( SECONDS ));
#endif
					}
				}
				else if (ALARM_CYCLE == rtc_system_getCurrentAlarmFromBKUP())
				{
#if defined(UNE82326)
					une82326_set_start_comm(1);
#elif defined(MBUS)
					mbus_set_start_comm(1);
#endif
					shutdown_setInitTelitModule(0);
					params_maintenance_inc_number_readings();
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ALARM_CYCLE. \r\n", (int)Tick_Get( SECONDS ));
				}
				else if (ALARM_INIT_READ == rtc_system_getCurrentAlarmFromBKUP())
				{
					if (1 == __checkRTCAlarm( rtc_system_getReadAlarmInitNext()))
					{
						end_read = 0;
#if defined(UNE82326)
						une82326_set_start_comm(1);
#elif defined(MBUS)
						mbus_set_start_comm(1);
#endif
						shutdown_setInitTelitModule(0);
						params_maintenance_inc_number_readings();
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ALARM_INIT_READ. \r\n", (int)Tick_Get( SECONDS ));
					}
				}
				else if (ALARM_END_READ == rtc_system_getCurrentAlarmFromBKUP())
				{
					if (( 1 == __checkRTCAlarm( rtc_system_getReadAlarmEndNext())) && (0 == end_read))
					{
						end_read =1;
#if defined(UNE82326)
						une82326_set_start_comm(1);
#elif defined(MBUS)
						mbus_set_start_comm(1);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ALARM_END_READ. \r\n", (int)Tick_Get( SECONDS ));
#endif
					}
					else
					{
#if defined(UNE82326)
						une82326_set_start_comm(0);
#elif defined(MBUS)
						mbus_set_start_comm(0);
#endif
					}
					shutdown_setInitTelitModule(0);
				}
				if ( ALARM_PARAMS_SEND_PARAMS == rtc_system_getCurrentParamsAlarm() ) {
					if ( 1 == __checkRTCParam( rtc_system_getSendParamsAlarmSendNext() ) ) {
						send_params = 1;
						message_queue_write( SEND_NETWORK_PARAMS );
						shutdown_setInitTelitModule(1);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ALARM_PARAMS_SEND_PARAMS. \r\n", (int)Tick_Get( SECONDS ));
					}
				}
#ifdef EXT_SENSOR
				if ( ALARM_SENSOR_SEND == rtc_system_getCurrentSensorAlarm() ) {
					if ( 1 == __checkRTCAlarm( rtc_system_getSendSensorAlarmSendNext() ) ) {
						if ( 1 == params_config_get_sensor_log() ) {
							message_queue_write( SEND_SENSOR );
						}
						if (params_input_pulse_as_sensor_get_num() != 0)
						{
							generic_sensor_set_on_off(0, ON);
							generic_sensor_set_sensors_to_send();
						}
						else
						{
							AD_SetGetADCData(1);
						}
						rtc_system_reset_remaining_time();
						shutdown_setInitTelitModule(1);
					}
				} else if (  ( ALARM_SENSOR_CYCLE == rtc_system_getCurrentSensorAlarm() ) ) {
					if ( 0 == pulses_get_interruption() ) {
						if (params_input_pulse_as_sensor_get_num() != 0)
						{
							generic_sensor_set_on_off(0, ON);
						}
						else
						{
							AD_SetGetADCData(1);
						}
						rtc_system_reset_remaining_time();
						if ( ( rtc_system_getCurrentAlarmFromBKUP() != ALARM_SEND )
						  && ( rtc_system_getCurrentModbusAlarm()   != ALARM_MODBUS_SEND )
						  && ( send_params                          == 0 ) ) {
							shutdown_setInitTelitModule(0);
						}
					}
				} else if ( ALARM_SENSOR_INIT_READ == rtc_system_getCurrentSensorAlarm() ) {
					if ( 1 == __checkRTCAlarm( rtc_system_getReadSensorAlarmInitNext() ) ) {
						if (params_input_pulse_as_sensor_get_num() != 0)
						{
							generic_sensor_set_on_off(0, ON);
						}
						else
						{
							AD_SetGetADCData(1);
						}
						rtc_system_reset_remaining_time();
						if ( ( rtc_system_getCurrentAlarmFromBKUP() != ALARM_SEND )
						  && ( rtc_system_getCurrentModbusAlarm()   != ALARM_MODBUS_SEND )
						  && ( send_params                          == 0 ) ) {
							shutdown_setInitTelitModule(0);
						}
					}
				} else if ( ALARM_SENSOR_END_READ == rtc_system_getCurrentSensorAlarm() ) {
					if ( ( rtc_system_getCurrentAlarmFromBKUP() != ALARM_SEND )
					  && ( rtc_system_getCurrentModbusAlarm()   != ALARM_MODBUS_SEND )
					  && ( send_params                          == 0 ) ) {
						shutdown_setInitTelitModule(0);
					}
				}
#endif
				if ( ALARM_MODBUS_SEND == rtc_system_getCurrentModbusAlarm() ) {
					if ( 1 == __checkRTCAlarm( rtc_system_getSendModbusAlarmSendNext() ) ) {
						if ( 1 == params_wq_on() ) {
							__throwModbusSensorSend();
							shutdown_setInitTelitModule(1);
//#if defined(UNE82326)
//							une82326_set_start_comm(0);
//#elif defined(MBUS)
//							mbus_set_start_comm(0);
//#endif
						} else if ( MODBUS_RAW == generic_485_get_type() ) {
							__throwModbusGenericSensorSend();
							shutdown_setInitTelitModule(1);
//#if defined(UNE82326)
//							une82326_set_start_comm(0);
//#elif defined(MBUS)
//							mbus_set_start_comm(0);
//#endif
						}
					}
				} else if (  ( ALARM_MODBUS_CYCLE == rtc_system_getCurrentModbusAlarm() ) ) {
					if ( 1 == params_wq_on() ) {
						if ( 0 == pulses_get_interruption() ) {
							aqualabo_modbus_enable(1);
							modbus_sensors_set_get_data(1);
							aqualabo_set_comm_state(1);
							rtc_system_reset_remaining_time();
//#if defined(UNE82326)
//							une82326_set_start_comm(0);
//#elif defined(MBUS)
//							mbus_set_start_comm(0);
//#endif
						}
					}  else if ( MODBUS_RAW == generic_485_get_type() ) {
						if ( 0 == pulses_get_interruption() ) {
							generic_485_set_enable(1);
							modbus_sensors_set_get_data(1);
							modbus_sensors_init_task();
							generic_485_set_comm_state(1);
							rtc_system_reset_remaining_time();
//#if defined(UNE82326)
//							une82326_set_start_comm(0);
//#elif defined(MBUS)
//							mbus_set_start_comm(0);
//#endif
						}
					}
					if ( ( rtc_system_getCurrentAlarmFromBKUP() != ALARM_SEND        )
					  && ( rtc_system_getCurrentSensorAlarm()   != ALARM_SENSOR_SEND )
					  && ( send_params                          == 0                 )
					  ) {
						shutdown_setInitTelitModule(0);
					}
				} else if ( ALARM_MODBUS_INIT_READ == rtc_system_getCurrentModbusAlarm() ) {
					if ( 1 == __checkRTCAlarm( rtc_system_getReadModbusAlarmInitNext() ) ) {
						if ( 1 == params_wq_on() ) {
							aqualabo_modbus_enable(1);
							modbus_sensors_set_get_data(1);
							aqualabo_set_comm_state(1);
							rtc_system_reset_remaining_time();
//#if defined(UNE82326)
//							une82326_set_start_comm(0);
//#elif defined(MBUS)
//							mbus_set_start_comm(0);
//#endif
						} else if ( MODBUS_RAW == generic_485_get_type() ) {
							generic_485_set_enable(1);
							modbus_sensors_set_get_data(1);
							modbus_sensors_init_task();
							generic_485_set_comm_state(1);
							rtc_system_reset_remaining_time();
//#if defined(UNE82326)
//							une82326_set_start_comm(0);
//#elif defined(MBUS)
//							mbus_set_start_comm(0);
//#endif
						}
						if ( ( rtc_system_getCurrentAlarmFromBKUP() != ALARM_SEND )
						  && ( rtc_system_getCurrentSensorAlarm()   != ALARM_SENSOR_SEND )
						  && ( send_params                          == 0 ) ) {
							shutdown_setInitTelitModule(0);
						}
					}
				} else if ( ALARM_MODBUS_END_READ == rtc_system_getCurrentModbusAlarm() ) {
					if ( ( rtc_system_getCurrentAlarmFromBKUP() != ALARM_SEND )
					  && ( rtc_system_getCurrentSensorAlarm()   != ALARM_SENSOR_SEND )
					  && ( send_params                          == 0 ) ) {
						shutdown_setInitTelitModule(0);
					}
				}
			}
			__chooseAlarmAndSensorCycleAlarm();
			if ( 0 == Tick_cloud_time_init() ) {
//				message_queue_write( SEND_NETWORK_PARAMS );
//				shutdown_setInitTelitModule(1);
//				generic_sensor_log_set_save_programword(1);
				AD_SetGetADCData(1);
				generic_sensor_log_set_save_programword(1);
				sensor_log_set_save_programword(1);
				if (1 == params_pulses_on())
				{
					__checkMeterType();
				}
				else
				{
					shutdown_setInitTelitModule(1);
					message_queue_write(SEND_NETWORK_PARAMS);
				}
				if ( ( TAMPER_HARD_RESET_DAYLY == shutdown_get_tamper_param() ) || ( TAMPER_WATCHDOG == shutdown_get_tamper_param() ) )
				{
					shutdown_read_dlms_count();
				}
			}
			shutdown_set_wchdg_time( params_get_timeout_connection() );

			if (1 == rtc_system_getAlarmOccured())
			{
				rtc_system_setAlarmOccured(0);
				/* Disable the Alarm A */
				if ( ALARM_ELAPSED == rtc_system_getAlarmState())
				{
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Disable the Alarm A.\r\n", (int)Tick_Get( SECONDS ) );
					HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				}
			}

			for ( int i = 0; i < rtc_system_getReadSensorAlarmNum(); i++ )
			{
				if ( ( params_sensor_read_time_init_time( i ) <= Sys_time.Time.Hours )
				  && ( params_sensor_read_time_end_time( i )  >  Sys_time.Time.Hours )
				  && (Sys_time.Date.Date  == params_sensor_read_time_date_day(i))
				  && (Sys_time.Date.Month == params_sensor_read_time_date_month(i))
//				  && (__checkRTCQuartNoPeriod(&minute_backup))
				   )
				{
					if (__checkRTCQuartNoPeriod(&minute_backup))
					{
						if ( 1 == params_config_get_sensor_log() ) {
							message_queue_write( SEND_SENSOR );
						}
						if (params_input_pulse_as_sensor_get_num() != 0)
						{
							generic_sensor_set_on_off(0, ON);
							generic_sensor_set_sensors_to_send();
						}
						else
						{
							AD_SetGetADCData(1);
						}
						rtc_system_reset_remaining_time();
						shutdown_setInitTelitModule(1);
					}
				}
			}
			for ( int j = 0; j < NUM_INPUTS_SENSOR; j++ )
			{
				if ( 1 == generic_sensor_get_burst_mode(j))
				{
					if (__checkRTCQuartNoPeriod(&minute_backup))
					{
						if ( 1 == params_config_get_sensor_log() ) {
							message_queue_write( SEND_SENSOR );
						}
						if (params_input_pulse_as_sensor_get_num() != 0)
						{
							generic_sensor_set_on_off(0, ON);
							generic_sensor_set_sensors_to_send();
						}
						else
						{
							AD_SetGetADCData(1);
						}
						rtc_system_reset_remaining_time();
						shutdown_setInitTelitModule(1);
					}
				}
			}
			if ( 1 == rtc_system_getAlarmProgrammedReset() )
			{
				HAL_NVIC_SystemReset();
			}
			if ( 1 == shutdown_get_tamper_sending() )
			{
				shutdown_set_tamper_sending(2);
				shutdown_set_tamper_param(TAMPER_TAMPER_ON);
				message_queue_write( SEND_NETWORK_PARAMS );
				shutdown_setInitTelitModule(1);
			}
			if (1 == Tick_cloud_time_init())
			{
				if ( 0 == sensor_log_get_write_pulse_acc() )
				{
					if (( 1 == params_pulses_on() ) &&  ( 1 == params_get_pulse_acc()) && __checkRTCQuartAccCreated() )//__checkRTCQuartAcc()
					{
						sensor_log_set_write_pulse_acc(1);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Write Pulse Acc.\r\n", (int)Tick_Get( SECONDS ));
					}
				}
			}
		}
	}

	return shutdown_Manager.init_TelitModule;
}

static int32_t modbus_count = 0; /*!< number of samples per sending. number of samples before transmitting */
static int32_t dlms_count[30][DLMS_PROFILE_NUM];
static int32_t dlms_count_max[30][DLMS_PROFILE_NUM];
static int32_t dlms_last_profile[30], dlms_last_obis_profile[30], dlms_profiles_to_send[30][DLMS_PROFILE_NUM];
static int32_t meter_count  = 0;

#define DLMS_COUNT_ADDRESS  (0x302000)
uint32_t shutdown_init_dlms_count(void)
{
	shutdown_read_dlms_count();

	if (-1 == dlms_count[0][0])
	{
		memset(dlms_count, '\0', sizeof(dlms_count));
		sFLASH_EraseSector(DLMS_COUNT_ADDRESS);
		sFLASH_WriteBuffer( (uint8_t *)dlms_count, DLMS_COUNT_ADDRESS, sizeof(dlms_count) );
	}
	return 0;
}

uint32_t shutdown_write_dlms_count(void)
{
	sFLASH_EraseSector(DLMS_COUNT_ADDRESS);
	sFLASH_WriteBuffer( (uint8_t *)dlms_count, DLMS_COUNT_ADDRESS, sizeof(dlms_count) );
	return 0;
}

uint32_t shutdown_read_dlms_count(void)
{
	memset(dlms_count, '\0', sizeof(dlms_count));
	sFLASH_ReadBuffer((uint8_t *)dlms_count, DLMS_COUNT_ADDRESS, sizeof(dlms_count));
	return 0;
}

void shutdown_reset_modbus_count( void )
{
	modbus_count = 0;
}

void shutdown_reset_meter_count( void )
{
	meter_count = 0;
}

void shutdown_synch_count_params( void )
{
	modbus_count = 1;
	meter_count  = 1;
}

uint32_t shutdown_get_meter_send( void )
{
	if (meter_send == 1)
	{
		__NOP();
	}
	return meter_send;
}

void shutdown_set_meter_send( uint32_t _meter_send )
{
	if (_meter_send == 1)
	{
		__NOP();
	}
	meter_send = _meter_send;
}

uint32_t __attribute__((optimize("O0"))) shutdown_check_dlms_count_send_frame(void)
{
	uint32_t i,j,one_profile_more = 0;
	for (j = 0; j < dlms_client_table_get_num_devices(); j++)
	{
		one_profile_more = 0;
		for (i = 0; i < DLMS_PROFILE_NUM; i++)
		{
			if (i != DLMS_EVENT_PROFILE_INDEX)
			{
				if ((1 == dlms_count[j][i]) && (1==one_profile_more))
				{
					return 1;
				}
				else if ((1 == dlms_count[j][i]) && (0==one_profile_more))
				{
					one_profile_more = 1;
				}
			}
		}
	}
	return 0;
}

void __check_last_profile_to_send( uint32_t num_meter, uint32_t read_time, int32_t * minute_factor_dlms )
{
	uint32_t i = 0;
	for (i = 0; i < DLMS_PROFILE_NUM; i++)
	{
		if ( (i >= DLMS_OBIS_PROFILE_NUM) && (i != DLMS_BILLING_PROFILE_INDEX) )
		{
			if (0 == dlms_count[num_meter][i])
			{
				uint32_t offset = 0;
				if (read_time != 0)
				{
					if ( 0 == dlms_client_get_send_time_obis_n(i) )
					{
						dlms_count[num_meter][i]  = 0;
					}
					else
					{
						if (( 0 == dlms_client_get_resend() ) && ( 1 == params_config_get_sync_read()))
						{
							offset = 1;
						}
						dlms_count[num_meter][i]  = (int32_t) (((dlms_client_get_send_time_obis_n(i) / read_time) * (*minute_factor_dlms)) + offset);
					}
					if ( i > DLMS_OBIS_PROFILE_NUM )
					{
						if ((dlms_count[num_meter][i] > dlms_count[num_meter][i-1]) && (dlms_count[num_meter][i-1] != 0))
						{
							dlms_last_profile[num_meter] = i;
						}
					}
					else
					{
						dlms_last_profile[num_meter] = DLMS_OBIS_PROFILE_NUM;
					}
				}
				dlms_count_max[num_meter][i] = dlms_count[num_meter][i] - offset;
			}
		}
		else if (i != DLMS_MAX_DEMAND_PROFILE_INDEX)
		{
			if (0 == dlms_count[num_meter][i])
			{
				uint32_t offset = 0;
				if (read_time != 0)
				{
					if ( 0 == dlms_client_get_send_time_obis_n(i) )
					{
						dlms_count[num_meter][i]  = 0;
					}
					else
					{
						if (( 0 == dlms_client_get_resend() ) && ( 1 == params_config_get_sync_read()))
						{
							offset = 1;
						}
						dlms_count[num_meter][i]  = (int32_t) (((dlms_client_get_send_time_obis_n(i) / read_time) * (*minute_factor_dlms)) + offset);
					}
					if ( i > 0 )
					{
						if ((dlms_count[num_meter][i] > dlms_count[num_meter][i-1]) && (dlms_count[num_meter][i-1] != 0))
						{
							dlms_last_obis_profile[num_meter] = i;
						}
					}
					else
					{
						dlms_last_obis_profile[num_meter] = 0;
					}
				}
				dlms_count_max[num_meter][i] = dlms_count[num_meter][i] - offset;
			}
		}
	}
}

/**
 * @fn void __setCounter(int32_t*, int32_t*)
 * @brief Calculates the factor multiplier for modbus sensor and meter.
 * @param minute_factor pointer to minute_factor for mbus.
 * @param minute_factor_modbus pointer to minute_factor_modbus.
 */
static void __setCounter(int32_t * minute_factor, int32_t * minute_factor_modbus, int32_t * minute_factor_dlms)
{
	uint32_t rt  = 0;
	uint32_t div = 30; // Allways 30

	if ((params_input_pulse_as_sensor_get_num() != 0) && (1 == params_config_get_period()))
	{
		div = params_sensor_read_time_cycle(0);
	}

	/* ADC enabled */
	if ((1 == AD_GetADOn()))
	{
		/* Generic modbus reading time is enabled */
		if (0 != generic_485_get_read_time())
		{
			generic_485_get_addr();
			generic_485_get_function();
			generic_485_get_quantity();
			generic_485_get_slave_id();
//			generic_485_load_slaves();
			modbus_sensors_get_serial_config_baud_rate();
			modbus_sensors_get_serial_config_parity();
			modbus_sensors_get_serial_config_stop_bits();
			/* Generic modbus reading time is smaller than sensor reading time */
			if (generic_485_get_read_time() < param.config.rt)
			{
				*minute_factor        = (int32_t) (param.config.rt/*generic_485_get_read_time()*/ / div);
				*minute_factor_modbus = (int32_t) (generic_485_get_read_time() / div);

				if (0 == modbus_count)
				{
					modbus_count  = (int32_t) ((generic_485_get_send_time() / generic_485_get_read_time()) * (*minute_factor_modbus/*minute_factor*/));
				}
			}
			/* Generic modbus reading time is bigger than or equals to sensor reading time */
			else
			{
				*minute_factor        = (int32_t) (param.config.rt / div);
				*minute_factor_modbus = (int32_t) (generic_485_get_read_time()/*param.config.rt*/ / div);

				if (0 == modbus_count)
				{
					modbus_count  = (int32_t) ((generic_485_get_send_time() / param.config.rt) * (*minute_factor_modbus));
				}
			}
		}
		/* DLMS reading time is enabled */
		else if (0 != dlms_client_get_read_time())
		{
				/* Generic modbus reading time is smaller than sensor reading time */
				if (dlms_client_get_read_time() < param.config.rt)
				{
					*minute_factor        = (int32_t) (dlms_client_get_read_time() / div);
					*minute_factor_dlms   = (int32_t) (dlms_client_get_read_time() / div);

					uint32_t j = 0;//i = 0,
					for (j = 0; j < dlms_client_table_get_num_devices(); j++)
					{
						memset(dlms_client_get_client(), 0, dlms_client_get_size());
						dlms_client_table_read_client_id( dlms_client_get_client(), j );
						__check_last_profile_to_send(j, param.config.rt, minute_factor_dlms);
					}
				}
				/* Generic modbus reading time is bigger than or equals to sensor reading time */
				else
				{
					*minute_factor        = (int32_t) (param.config.rt / div);
					*minute_factor_dlms   = (int32_t) (param.config.rt / div);

					uint32_t j = 0;//i = 0,
					for (j = 0; j < dlms_client_table_get_num_devices(); j++)
					{
						memset(dlms_client_get_client(), 0, dlms_client_get_size());
						dlms_client_table_read_client_id( dlms_client_get_client(), j );
						__check_last_profile_to_send(j, param.config.rt, minute_factor_dlms);
					}
				}
		}
		/* If generic modbus reading time is not enabled */
		else
		{
			*minute_factor        = (int32_t) (param.config.rt / div);
			*minute_factor_modbus = (int32_t) (param.config.rt / div);
			*minute_factor_dlms   = (int32_t) (param.config.rt / div);
		}
	}
	/* ADC is disabled */
	else
	{
		/* Generic modbus reading time is enabled */
		if (0 != generic_485_get_read_time())
		{
			generic_485_get_addr();
			generic_485_get_function();
			generic_485_get_quantity();
			generic_485_get_slave_id();
//			generic_485_load_slaves();
			modbus_sensors_get_serial_config_baud_rate();
			modbus_sensors_get_serial_config_parity();
			modbus_sensors_get_serial_config_stop_bits();
			/* Generic modbus read time is smaller than sensor reading time then minimum period time is modbus read time*/
			if (generic_485_get_read_time() < param.config.rt)
			{
				/* Calculates the counter, that is number of samples per sending */
				if (0 == modbus_count)
				{
					modbus_count                = (int32_t) (generic_485_get_send_time() / generic_485_get_read_time());
					*minute_factor              = param.config.rt / generic_485_get_read_time();
					*minute_factor_modbus       = 1;
					shutdown_time_periodic_mode = generic_485_get_read_time();
				}
			}
			/* Generic modbus read time is bigger than sensor reading time then minimum period time is sensor reading */
			else
			{
				/* Calculates the counter, that is number of samples per sending */
				if (0 == modbus_count)
				{
					if (0 != param.config.rt)
					{
						rt = param.config.rt;
					}
					else
					{
						rt = generic_485_get_read_time();
					}
					modbus_count                = (int32_t) (generic_485_get_send_time() / rt);
					*minute_factor              = 1;
					*minute_factor_modbus       = generic_485_get_read_time() / param.config.rt;
					shutdown_time_periodic_mode = rt;
				}
			}
		}
		/* DLMS reading time is enabled */
		else if (0 != dlms_client_get_read_time())
		{
				memset(dlms_client_get_client(), 0, dlms_client_get_size());
				dlms_client_table_read_client_id( dlms_client_get_client(), 0 );
				/* Generic modbus read time is smaller than sensor reading time then minimum period time is modbus read time*/
				if (dlms_client_get_read_time() < param.config.rt)
				{
					/* Calculates the counter, that is number of samples per sending */
					uint32_t j = 0;//i = 0,
					for (j = 0; j < dlms_client_table_get_num_devices(); j++)
					{
						memset(dlms_client_get_client(), 0, dlms_client_get_size());
						dlms_client_table_read_client_id( dlms_client_get_client(), j );
						*minute_factor_dlms = 1;
						__check_last_profile_to_send( j, dlms_client_get_read_time(), minute_factor_dlms );
						*minute_factor              = param.config.rt / dlms_client_get_read_time();
						*minute_factor_dlms         = 1;
						shutdown_time_periodic_mode = dlms_client_get_read_time();
					}
				}
				/* Generic modbus read time is bigger than sensor reading time then minimum period time is sensor reading */
				else
				{
					/* Calculates the counter, that is number of samples per sending */
					uint32_t j = 0;//i = 0,
					for (j = 0; j < dlms_client_table_get_num_devices(); j++)
					{
						memset(dlms_client_get_client(), 0, dlms_client_get_size());
						dlms_client_table_read_client_id( dlms_client_get_client(), j );
						if (0 != param.config.rt)
						{
							rt = param.config.rt;
						}
						else
						{
							rt = dlms_client_get_read_time();
						}
						*minute_factor_dlms = 1;
						__check_last_profile_to_send(j, rt, minute_factor_dlms);
					}
					*minute_factor_dlms         = dlms_client_get_read_time() / param.config.rt;
					*minute_factor              = 1;
					shutdown_time_periodic_mode = rt;
				}
				if (0 == modbus_count)
				{
					modbus_count = (( param.config.st / param.config.rt ) * (*minute_factor));// + 1;
				}
		}
		/* Generic modbus reading time is disabled */
		else
		{
			*minute_factor              = 1;
			*minute_factor_modbus       = 0;
			*minute_factor_dlms         = 0;
			shutdown_time_periodic_mode = param.config.rt;
		}
	}
}

uint32_t shutdown_get_first_profile(uint32_t curr_dev)
{
	uint32_t i;

	dlms_client_table_read_client_id( dlms_client_get_client(), curr_dev );
	for (i = 0; i < DLMS_OBIS_PROFILE_NUM; i++)
	{
		if (dlms_client_get_read_time_obis_n(i) != 0)
		{
			break;
		}
	}
	return i;
}

int32_t shutdown_get_last_profile_to_send( uint32_t curr_dev )
{
	return dlms_last_profile[curr_dev];
}

int32_t shutdown_get_last_obis_profile_to_send( uint32_t curr_dev )
{
	return dlms_last_obis_profile[curr_dev];
}

int32_t shutdown_obis_num_msg = 0, shutdown_generic_profile_num_msg = 0;
int32_t shutdown_obis_read_num_msg = 0, shutdown_generic_profile_read_num_msg = 0;
int32_t shutdown_get_obis_num_msg( void )
{
	return shutdown_obis_num_msg;
}

int32_t shutdown_get_generic_profile_num_msg( void )
{
	return shutdown_generic_profile_num_msg;
}

void shutdown_set_obis_num_msg( int32_t _obis_num_msg )
{
	if (_obis_num_msg >= 0)
	{
		shutdown_obis_num_msg = _obis_num_msg;
	}
}

void shutdown_set_generic_profile_num_msg( int32_t _profile_num_msg )
{
	if (_profile_num_msg >= 0)
	{
		shutdown_generic_profile_num_msg = _profile_num_msg;
	}
}

int32_t shutdown_get_obis_read_num_msg( void )
{
	return shutdown_obis_read_num_msg;
}

int32_t shutdown_get_generic_profile_read_num_msg( void )
{
	return shutdown_generic_profile_read_num_msg;
}

void shutdown_set_obis_read_num_msg( int32_t _obis_num_msg )
{
	if (_obis_num_msg >= 0)
	{
		shutdown_obis_read_num_msg = _obis_num_msg;
	}
}

void shutdown_set_generic_profile_read_num_msg( int32_t _profile_num_msg )
{
	if (_profile_num_msg >= 0)
	{
		shutdown_generic_profile_read_num_msg = _profile_num_msg;
	}
}

int32_t shutdown_get_profile_to_send(uint32_t client_id, uint32_t index)
{
	return dlms_profiles_to_send[client_id][index];
}

int32_t shutdown_init_profile_to_send(void)
{
	uint32_t i, j;

	for ( i = 0; i < 30; i++ )
	{
		for ( j = 0; j<DLMS_PROFILE_NUM; j++ )
		{
			dlms_profiles_to_send[i][j] = 0;
		}
	}
	return 0;
}

static uint32_t __attribute__((optimize("O0"))) __checkCounters(uint32_t client_id, uint32_t index, int32_t *counter, int32_t minute_factor)
{
	uint32_t send_but_not_read = 0;
	if (index == DLMS_BILLING_PROFILE_INDEX)
	{
		if ( 1 == rtc_system_get_billing_day() )
		{
			dlms_client_inc_dlms_reading_items(client_id);
			if (client_id == dlms_client_table_get_num_devices() - 1)
			{
				rtc_system_set_billing_day(2);
			}
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue Billing.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_BILLING_PROF);
			dlms_profiles_to_send[client_id][index] = 1;
			shutdown_generic_profile_num_msg++;
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue Billing.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_read_msg_queue(), DLMS_READ_BILLING_PROF);
		}
	}
	else if (index == DLMS_MAX_DEMAND_PROFILE_INDEX)
	{
		if ( 1 == rtc_system_get_billing_day() )
		{
			dlms_client_inc_dlms_reading_items(client_id);
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue Max Demand.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), DLMS_WAIT_FOR_SEND_MAX_DEMAND_PROF_1);
			shutdown_obis_num_msg++;
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue Max Demand.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Put(dlms_client_get_read_msg_queue(), DLMS_READ_MAX_DEMAND_PROF_1);
		}
	}
	else
	{
		if (dlms_client_get_read_time_obis_n(index) != 0)
		{
			if (*counter != 0)
			{
				*counter-=1;
			}
		}
		else
		{
			*counter = 0;
			LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> Read disabled Index:%d.\r\n", (int)Tick_Get( SECONDS ),(int)index);
			return -1;
		}
		uint32_t div_mod = (dlms_client_get_read_time_obis_n(index)/shutdown_time_periodic_mode);// * minute_factor;
		if ((dlms_client_get_read_time() != 0)
				&& ( ((dlms_client_get_read_time_obis_n(index)!=0) && ((div_mod != 0) && (*counter % (div_mod)) == 0 )) || ( 1 == dlms_client_get_resend() )/*|| (shutdown_sync_modbus_read_pressure_on == 1)*/)
				&& (1 == Tick_cloud_time_init()
//				&& ((*counter == dlms_count_max[client_id][index]) && (div_mod == 1) /*|| ((*counter == 1)&&(dlms_count_max[client_id][index] == 1))*/)
                )
		)
		{
//			if (( (*counter == dlms_count_max[client_id][index]) && (div_mod == 1) ) || (*counter != dlms_count_max[client_id][index]))
			if (( (*counter == dlms_count_max[client_id][index]) /*&& (div_mod == 1)*/ )
			  || ((*counter != 0 ) && (*counter != dlms_count_max[client_id][index]))
			  || ( 1 == dlms_client_get_resend() )
			  || ( 0 == params_config_get_sync_read() )
			  )
			{
				LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Read Queue.\r\n", (int)Tick_Get( SECONDS ));
				dlms_client_inc_dlms_reading_items(client_id);
				CircularBuffer_Put(dlms_client_get_read_msg_queue(), index);
				if ( index < DLMS_OBIS_PROFILE_NUM )
				{
					shutdown_obis_read_num_msg++;
				}
				else
				{
					shutdown_generic_profile_read_num_msg++;
				}
			}
			else if (*counter == 0 )
			{
				send_but_not_read = 1;
			}
		}
		if ( (*counter == 0)
				&& (dlms_client_get_send_time() != 0)
				&& (1 == Tick_cloud_time_init())
		)
		{
			if (index != DLMS_EVENT_PROFILE_INDEX)
			{
			CircularBuffer_Put(dlms_client_get_send_msg_queue(), index + DLMS_WAIT_FOR_SEND_INST_PROF_1);
			if (1==send_but_not_read)
			{
    			CircularBuffer_Get(dlms_client_get_send_msg_queue());
    			LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> Put Send Queue.\r\n", (int)Tick_Get( SECONDS ));
    			CircularBuffer_Put(dlms_client_get_send_msg_queue(), index);
    			message_queue_write(SEND_MODBUS_SENSOR);
			}
			dlms_profiles_to_send[client_id][index] = 1;
			if ( index < DLMS_OBIS_PROFILE_NUM )
			{
				shutdown_obis_num_msg += shutdown_obis_read_num_msg;
			}
			else
			{
				shutdown_generic_profile_num_msg += shutdown_generic_profile_read_num_msg;
			}
			}
			if ( index == (DLMS_PROFILE_NUM - 1) )
			{
				shutdown_obis_read_num_msg            = 0;
				shutdown_generic_profile_read_num_msg = 0;
			}
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue. Num Obis Msg:%d. Num Profile Msg:%d.\r\n", (int)Tick_Get( SECONDS ), (int)shutdown_obis_num_msg, (int)shutdown_generic_profile_num_msg);
		}
	}
	return 0;
}

static void __DlmsSchedule( int32_t  minute_factor_dlms, uint32_t _shutdown_time_periodic_mode )
{
	uint32_t i = 0, j = 0;

	if (( 0 == rest_upgrading_firmware() ) &&  (1 == Tick_cloud_time_init()) && ( 1 == dlms_client_get_dlms_enable() ))
	{
		if (0 == CircularBuffer_IsEmpty(dlms_client_get_send_msg_queue()))
		{
			LOGLIVE(LEVEL_2, "LOGLIVE> %d CIRCULAR_BUFFER> Send Queue Not Empty.\r\n", (int)Tick_Get( SECONDS ));
			CircularBuffer_Reset(dlms_client_get_send_msg_queue());
		}
		shutdown_init_profile_to_send();
		for (j = 0; j < dlms_client_table_get_num_devices(); j++)
		{
			dlms_client_set_dlms_reading_items(j, 0);
			for (i = 0; i < DLMS_PROFILE_NUM; i++)
			{
				dlms_client_table_read_client_id( dlms_client_get_client(), j );
				if (i < DLMS_OBIS_PROFILE_NUM)
				{
					dlms_client_table_read_client_obis_profile(j, dlms_client_get_client(),  dlms_client_get_client_obis_profile(), i);
				}
				else
				{
					dlms_client_table_read_client_generic_profile(j, dlms_client_get_client(), dlms_client_get_client_generic_profile(), i);
				}
				__checkCounters(j, i, &dlms_count[j][i], minute_factor_dlms);
				LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> Profile:%d Count:%d. \r\n", (int)Tick_Get( SECONDS ), (int)i, (int)dlms_count[j][i]);
			}
		}
		shutdown_write_dlms_count();
		shutdown_read_dlms_count();
	}
}
/**
 * @brief
 */
uint32_t reset_psm = 0;
/**
 * @fn void shutdown_set_reset_psm(uint32_t)
 * @brief
 *
 * @param _reset_psm
 */
void shutdown_set_reset_psm( uint32_t _reset_psm )
{
	reset_psm = _reset_psm;
}

/**
 * @fn uint32_t shutdown_get_reset_psm(void)
 * @brief
 *
 * @return
 */
uint32_t shutdown_get_reset_psm( void )
{
	return reset_psm;
}

uint32_t check_count_mqtt_on = 0;
void shutdown_set_check_count_mqtt( uint32_t __check_count_mqtt )
{
	check_count_mqtt_on = __check_count_mqtt;
}

uint32_t sending_mqtt_adc_on = 0;
void shutdown_set_sending_mqtt_adc_on( uint32_t __sending_mqtt_adc_on )
{
	sending_mqtt_adc_on = __sending_mqtt_adc_on;
}

uint32_t shutdown_get_sending_mqtt_adc_on( void )
{
	return sending_mqtt_adc_on;
}

uint32_t check_datalogger = 0;
void shutdown_set_check_datalogger( uint32_t __check_datalogger )
{
	check_datalogger = __check_datalogger;
}

uint32_t shutdown_get_check_datalogger(void)
{
	return check_datalogger;
}

/**
 * @fn uint8_t shutdown_get_send_counter(void)
 * @brief  *Gestiona también el parámetro synch_read. Gestiona las tareas que tiene que hacer
	 * 		- Pedir hora
	 * 		- Leer Presión
	 * 		- Lectura Modbus
	 * 		- Lectura MBUS
	 * 		- etc..
 *
 * Ej: Presión Y lectura de mbus cada 15' toma como base de tiempo los 30" de presión
 * va gestionando los contadores de esta forma
 *
 *
 *
 * @pre
 * @post
 * @return
 */
uint8_t shutdown_get_send_counter(void)
{
	static int32_t  count = 0;
	static int32_t  minute_factor = 0;    /*!< multiplier to calculate next sensor sample */
	static int32_t  minute_factor_modbus; /*!< multiplier to calculate next MODBUS sensor sample */
	static int32_t  minute_factor_dlms;
		   uint32_t send_init_params = 0;
		   uint32_t inc_num_readings = 0;
	static uint32_t got_last_pulse = 0;  /*!< Flag indicates still not  */
	static uint32_t after_reset = 0;
	static uint32_t wake_up_num = 0;
	static uint32_t minute_backup_1 = (uint32_t)-1, minute_backup_2 = (uint32_t)-1, minute_backup_3 = (uint32_t)-1;

	enum{OFF = 0, ON = 1};

	/** Calculates the multiplier counter to adjust time base related to configuration or reading and sending
	 * time */
	if (0 == start_count)
	{
// 	__setCounter(&minute_factor, &minute_factor_modbus, &minute_factor_dlms);
	}
	synch = params_config_get_sync_read();

	if (1 == pulses_get_pulse_rx())
	{
		rtc_system_reset_timeout_waiting_pulse();
//		LOGLIVE(LEVEL_1, "LOGLIVE> %d PULSES> CH2P Int:%d %d %d Num Pulse readings:%d\r\n",
//				(int)Tick_Get( SECONDS ), (int)pulses_get_backflow_compensated_2p(), (int)pulses_get_ch1p(), (int)pulses_get_ch1d(), (int)num_pulse_readings);
	}
//	if ((0 == rtc_system_getAlarmOccured()) && ( 0 == mqtt_is_state_idle() ))
	if ((1 == mqtt_is_state_send_ping()) && (1 == rtc_system_getAlarmOccured()))
	{
		if ( ALARM_ELAPSED == rtc_system_getAlarmState())
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ALARM NOT SET!!!!.\r\n", (int)Tick_Get( SECONDS ) );
			rtc_system_SetSyncAlarm(shutdown_time_periodic_mode);
			start_count = 0;
		}
	}
	if (1 == dlms_client_get_dlms_enable())
	{
		if ( 1 == dlms_client_get_dlms_load_profile_get_wait_for_command() )
		{
			if (DLMS_STATE_IDLE == con_dlms_get_state(&con))
			{
				shutdown_mqttOnDemandCommandprocess();
			}
			dlms_client_set_dlms_load_profile_set_wait_for_command(0);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> ON DEMAND COMM WAITING.\r\n", (int)Tick_Get( SECONDS ));
		}
	}

	if ( ((1 == alarm_int) || (1 == alarm_int_low)) && (1 == params_get_input_alarm_sensor()) )
	{
		alarm_int = 0;

		//Alarm on: Falling edge interruption received. Prepare to send.
		if ((1 == shutdown_get_alarm_sending()) && (1 == alarm_on) && ( GPIO_PIN_SET == HAL_GPIO_ReadPin( UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin ) ))
		{
			LOGLIVE(LEVEL_2,"LOGLIVE> ALARM ON\r\n");
			shutdown_set_alarm_sending(2);
			shutdown_set_tamper_param(TAMPER_INPUT_ALARM_ON);
			shutdown_setInitTelitModule(1);
			message_queue_write(SEND_NETWORK_PARAMS);
			if (1 == params_get_mqtt_dc_on())
			{
				mqtt_stop();
			}
			return shutdown_Manager.init_TelitModule;
		}
		else if ((3 == shutdown_get_alarm_sending()) && (1 == alarm_int_low) && ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin ) ))
		{
			LOGLIVE(LEVEL_2,"LOGLIVE> ALARM OFF\r\n");
			alarm_on      = 0;
			alarm_int_low = 0;
			shutdown_set_alarm_sending(4);
			shutdown_set_tamper_param(TAMPER_INPUT_ALARM_OFF);
			shutdown_setInitTelitModule(1);
			message_queue_write(SEND_NETWORK_PARAMS);
			if (1 == params_get_mqtt_dc_on())
			{
				mqtt_stop();
			}
			return shutdown_Manager.init_TelitModule;
		}
		/* Exits with 0 */
//		return shutdown_Manager.init_TelitModule;
	}

	/* Only does something if start_count == 0 which is reset in shutdown_on(void)*/
	if ( (0 == start_count)
			&& ( (0 == params_get_mqtt_dc_on())
			  || ( (1 == params_get_mqtt_dc_on()) && (1 == rtc_system_getAlarmOccured()) )
			  || ( (1 == params_get_mqtt_dc_on()) && (0 == Tick_cloud_time_init()) )
			  || ( (1 == params_get_mqtt_dc_on()) && (1 == tamper_int) )
			  || ( (1 == params_get_mqtt_dc_on()) && (1 == vbackup_int) )
			  ) )
	{
		if (1 == Tick_cloud_time_init())
		{
			__setCounter(&minute_factor, &minute_factor_modbus, &minute_factor_dlms);
		}
		if ( (1 == params_get_mqtt_dc_on()) && (1 == rtc_system_getAlarmOccured()) )
		{
			if ( (1 == AD_GetADOn()) && ( 0 == dlms_client_get_resend() ) )
			{
				check_count_mqtt_on = 1;
				if ( 1 == count )
				{
					sending_mqtt_adc_on = 1;
					shutdown_mqtt_restoreContext();
				}
				else
				{
					if ( ( count != 0 )
					|| ( ( 0 == count ) && ( 0 == mqtt_is_state_idle() ) ) )
					{
						shutdown_mqtt_restorePressureContext();
					}
				}
			}
			else
			{
				check_count_mqtt_on = 0;
				shutdown_mqtt_restoreContext();
			}
		}
		if ((1 == params_get_mqtt_dc_on()) && (1 == params_pulses_on()))
		{
			pulses_set_mutex(1);
			pulses_set_pulse_rx(0);
			AD_SetGetADCData(1);
		}
		else
		{
			/* If pulses has occurred and there is still time to go to sleep */
			if ((0 == reed_sending)
			&& ((0 == shutdown_get_alarm_sending())
					|| ((shutdown_get_alarm_sending() != 0) && (0 == shutdown_Manager.init_TelitModule)))
			&&  (1 == pulses_get_pulse_rx()) && (1 == Tick_cloud_time_init()) && (0 == last_pulse) && (0 == rtc_system_get_timeout_waiting_pulse()))
			{
//				params_maintenance_inc_number_readings();
				AD_SetGetADCData(0);
				Tick_Force_1Second_Task();//Forces one second task execution in order to save battery.
				/* Triggers the system to go sleep */
				shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
				start_count = 1; //avoid to enter here again
				shutdown_Manager.init_TelitModule = 0;
				got_last_pulse = 0;
				if ( 1 == reed_sending )
				{
					pulses_set_pulse_rx(0);
				}
				/* Exits with 0 */
				return shutdown_Manager.init_TelitModule;
			}

			if (1 == params_pulses_on())
			{
				if ( ( TAMPER_HARD_RESET_DAYLY == shutdown_get_tamper_param() ) || ( TAMPER_WATCHDOG == shutdown_get_tamper_param() ) )
				{

				}
				else
				{
//				num_pulse_readings++;
				params_maintenance_set_number_readings(num_pulse_readings);
				}
			}
			/* last pulse means needs to go to sleep */
			if ((1 == last_pulse) && (1 == params_pulses_on()))
			{
				if (0 == got_last_pulse)
				{
					pulses_set_mutex(1);
					pulses_set_pulse_rx(0);
					AD_SetGetADCData(1);
					got_last_pulse = 1;
				}
				else
				{
					AD_SetGetADCData(0);
					Tick_Force_1Second_Task();//TOASK Forces one second task execution in order to save battery.
					/* Triggers to go to sleep */
					shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
					start_count = 1;
					shutdown_Manager.init_TelitModule = 0;
					/*Exits with 0*/
					return shutdown_Manager.init_TelitModule;
				}
				last_pulse = 0;
			}
		}
		/* Telit is off*/
		if ( (shutdown_Manager.init_TelitModule == 0)
		 || ((check_count_mqtt_on == 1) && (1 == params_get_mqtt_dc_on())) )
		{
			start_count = 1;
			uint32_t psm_reset_times;
			if (0 == params_psm_edrx_tt3412())
			{
				psm_reset_times = 24;
			}
			else
			{
				psm_reset_times = (uint32_t) (24 / params_psm_edrx_tt3412());
				if (0 == psm_reset_times)
				{
					psm_reset_times = 24;
				}
			}

			if ((1 == rtc_system_getAlarmProgrammedReset()) || (shutdown_get_reset_psm() >= psm_reset_times))
			{
				num_pulse_readings = params_maintenance_number_readings();
				if (params_manteinance_delete_memory() != CLEAR_MEMORY_DISABLE_HARD_RESET)
				{
					if (((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0))
					|| (shutdown_get_reset_psm() >= psm_reset_times))
					{
						shutdown_set_reset_psm(0);
						NVIC_SystemReset();
					}
					else
					{
						NVIC_SystemReset();
					}
				}
			}
			/* number of counts for the meter is reset when rt or st is changed*/
			if ((0 == meter_count) || (0xFFFFFFFF == meter_count))
			{
				/* meter reading time enabled not zero */
				if (param.config.rt != 0)
				{
					/* Calculates the number of samples before transmission with the minutes correction */
					meter_count = count = ( param.config.st / param.config.rt ) * minute_factor;

					if (count != 0)
					{
						count--;
						meter_count = count;
					}
				}
				/* avoids null division if meter reading is not enabled */
				else
				{
					meter_count = count = 0;
				}

				/* generic modbus reading time enabled, not zero*/
				if ((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0))
				{
					if (modbus_count != 0)
					{
						modbus_count--;
					}
				}
				/* avoids null division if generic modbus reading is not enabled */
				else
				{
					modbus_count = 0;
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> count:%d modbus count:%d\r\n", (int)Tick_Get( SECONDS ), (int)count, (int)modbus_count);
				/* Same sending and reading period */
				if ((0 == count) && (0 != param.config.st) && (1 == Tick_cloud_time_init()))
				{
					/* pulses disabled */
					if (0 == params_pulses_on())
					{
						/* Water meter is NOT disabled */
						if (0 == params_config_get_disable_meter())
						{
							/* Synch mode and RTC is not multiple of quart of an hour */
							if ((synch == 1)
							 && ( (shutdown_sync_meter_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup_1)))
							 && (0 == dlms_client_get_resend())
							 )
							{
#if defined(UNE82326)
								une82326_set_start_comm(0);
#elif defined(MBUS)
								/* Do not start a mbus reading */
								mbus_set_start_comm(0);
#endif
							}
							/* */
							else
							{
#if defined(UNE82326)
								une82326_set_start_comm(1);
#elif defined(MBUS)
								/* start mbus reading */
								mbus_set_start_comm(1);
#endif
								inc_num_readings = 1;
								params_maintenance_inc_number_readings();
							}
						}
					}

					/* Send parameters as meter_count = 0 and it is time to send */
					send_init_params = 1;
					message_queue_write(SEND_NETWORK_PARAMS);
					shutdown_setInitTelitModule(1);
					meter_send = 1;
					wake_up_num++;
				}

				/* Modbus counter has reached zero */
				if ( (modbus_count == 0)
				&& ((generic_485_get_send_time() != 0) || (dlms_client_get_send_time() != 0))
				&& (1 == Tick_cloud_time_init()) )
				{
					/* Add network parameters to queue if still not added to send P Frame */
//					if (0 == send_init_params)
//					{
//						send_init_params = 1;
//						message_queue_write(SEND_NETWORK_PARAMS);
//					}

					/* Send modbus sensor sample */
					if (1 == params_wq_on())
					{
						__throwModbusSensorSend();
					}
					else if (0 != generic_485_get_read_time())
					{
						__throwModbusGenericSensorSend();
					}

					if (0 == inc_num_readings)
					{
						inc_num_readings = 1;
						params_maintenance_inc_number_readings();
					}
					/* Flag to enable TELIT */
					shutdown_setInitTelitModule(1);
					wake_up_num++;
				}

				/* Meter read count is multiple of minute factor so read mbus/modbus */
				if ((((minute_factor != 0)&&(count % minute_factor == 0)) || (shutdown_sync_meter_read_pressure_on == 1)) && (1 == Tick_cloud_time_init()))
				{
					/* Pulses disabled*/
					if (0 == params_pulses_on())
					{
						/* Water meter is enabled */
						if (0 == params_config_get_disable_meter())
						{
							/* synch read enabled but not the time */
							if ((synch == 1)
						     && ( (shutdown_sync_meter_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup_2)))
						     && (0 == dlms_client_get_resend()))
							{
#if defined(UNE82326)
								une82326_set_start_comm(0);
#elif defined(MBUS)
								/* mbus communication not started */
								mbus_set_start_comm(0);
#endif
							}
							/* Triggers water meter reading */
							else
							{
#if defined(UNE82326)
								une82326_set_start_comm(1);
#elif defined(MBUS)
								mbus_set_start_comm(1);
#endif
								/* increments number of reading*/
								if (0 == inc_num_readings)
								{
									inc_num_readings = 1;
									params_maintenance_inc_number_readings();
								}
							}
						}
					}

					if (count != 0)
					{
						meter_send = 0;
					}

					/* Adds network parameters to sending queue */
					if (1 == meter_send)
					{
						if (0 == send_init_params)
						{
							send_init_params = 1;
							message_queue_write(SEND_NETWORK_PARAMS);
						}

						if ((1 == params_pulses_on()) &&  ( 1 == params_get_pulse_acc()))
						{
							if( 1 == __checkRTCQuartAccCreated() )
							{
								sensor_log_set_write_pulse_acc(1);
							}
						}

						/* Pulses enabled or ADC is enabled */
						if ((1 == params_pulses_on()) || (1 == AD_GetADOn()))
						{
							/* Extended format */
							if (1 == params_config_get_sensor_log())
							{
								message_queue_write(SEND_SENSOR);
								if (params_input_pulse_as_sensor_get_num() != 0)
								{
									generic_sensor_set_on_off(0, ON);
									generic_sensor_set_sensors_to_send();
								}
							}
						}
					}
				}
				if (((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0))
				&& (((minute_factor_modbus != 0) && (modbus_count % minute_factor_modbus == 0)) || ((minute_factor_dlms != 0) && (modbus_count % minute_factor_dlms == 0)) || (shutdown_sync_modbus_read_pressure_on == 1))
				&& (1 == Tick_cloud_time_init()))
				{
					if (1 == params_wq_on())
					{
						__throwModbusSensorRead();
					}
					else if (0 != generic_485_get_read_time())
					{
						__throwModbusGenericSensorRead();
					}

					rtc_system_reset_remaining_time();

					if (0 == inc_num_readings)
					{
						inc_num_readings = 1;
						params_maintenance_inc_number_readings();
					}

					if (1 == params_config_get_disable_meter())
					{
#if defined(UNE82326)
						une82326_set_start_comm(0);
#elif defined(MBUS)
						mbus_set_start_comm(0);
#endif
					}
				}
				__DlmsSchedule( minute_factor_dlms, shutdown_time_periodic_mode );
			} // End if ((0 == meter_count) || (0xFFFFFFFF == meter_count))
			else
			{
				count = (int32_t) meter_count;

				if (count > 0)
				{
					count--;
				}
				else
				{
					count = 0;
				}

				if (0 == count)
				{
					shutdown_setInitTelitModule(1);
					meter_count = 0;
					meter_send  = 1;
					wake_up_num++;
				}
				else
				{
					meter_count = count;
				}

				if ((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0))
				{
					if (modbus_count != 0)
					{
						modbus_count--;
					}
				}
				else
				{
					modbus_count = 0;
				}
				LOGLIVE(LEVEL_2, "LOGLIVE> %d SHUTDOWN> count:%d modbus count:%d\r\n", (int)Tick_Get( SECONDS ), (int)count, (int)modbus_count);
				if ( (modbus_count == 0)
				&& ( ((generic_485_get_send_time() != 0) || (dlms_client_get_send_time() != 0)) ) )
				{
//					if (0 == send_init_params)
//					{
//						send_init_params = 1;
//						message_queue_write(SEND_NETWORK_PARAMS);
//					}

					if (1 == params_wq_on())
					{
						__throwModbusSensorSend();
					}
					else if (0 != generic_485_get_read_time())
					{
						__throwModbusGenericSensorSend();
					}

					if (0 == inc_num_readings)
					{
						inc_num_readings = 1;
						params_maintenance_inc_number_readings();
					}

					shutdown_setInitTelitModule(1);
				}

				if (((minute_factor != 0)&&(count % minute_factor == 0)) || (shutdown_sync_meter_read_pressure_on == 1))
				{
					if (0 == params_pulses_on())
					{
						if (0 == params_config_get_disable_meter())
						{
							if ((synch == 1) && ( (shutdown_sync_meter_read_pressure_on == 0) || (0 == __checkRTCQuart(&minute_backup_3))))
							{
#if defined(UNE82326)
							une82326_set_start_comm(0);
#elif defined(MBUS)
							mbus_set_start_comm(0);
#endif
							}
							else
							{
#if defined(UNE82326)
								une82326_set_start_comm(1);
#elif defined(MBUS)
								mbus_set_start_comm(1);
#endif
								if (0 == inc_num_readings)
								{
									inc_num_readings = 1;
									params_maintenance_inc_number_readings();
								}
							}
						}
					}

					if (count != 0)
					{
						meter_send = 0;
					}

					if (1 == meter_send)
					{
						if (0 == send_init_params)
						{
							send_init_params = 1;
							message_queue_write(SEND_NETWORK_PARAMS);
						}

						if ((1 == params_pulses_on()) &&  ( 1 == params_get_pulse_acc()))
						{
							if( 1 == __checkRTCQuartAccCreated() )
							{
								sensor_log_set_write_pulse_acc(1);
							}
						}

						if ((1 == params_pulses_on()) || (1 == AD_GetADOn()))
						{
							if (1 == params_config_get_sensor_log())
							{
								message_queue_write(SEND_SENSOR);
								if (params_input_pulse_as_sensor_get_num() != 0)
								{
									generic_sensor_set_on_off(0, ON);
									generic_sensor_set_sensors_to_send();
								}
							}
						}
					}
				}

				if (((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0))
				&& (((minute_factor_modbus != 0) && (modbus_count % minute_factor_modbus == 0)) || ((minute_factor_dlms != 0) && (modbus_count % minute_factor_dlms == 0)) || (shutdown_sync_modbus_read_pressure_on == 1))
				)
				{
					if (1 == params_wq_on())
					{
						__throwModbusSensorRead();
					}
					else if (0 != generic_485_get_read_time())
					{
						__throwModbusGenericSensorRead();
					}

					rtc_system_reset_remaining_time();

					if ( ( 0 == inc_num_readings )
					&& ( ( 1 == generic_485_get_enable() ) || ( 1 == aqualabo_modbus_get_enable() ) || ( 1 == dlms_client_get_dlms_enable() ) )
					   )
					{
						inc_num_readings = 1;
						params_maintenance_inc_number_readings();
					}

					if (1 == params_config_get_disable_meter())
					{
#if defined(UNE82326)
						une82326_set_start_comm(0);
#elif defined(MBUS)
						mbus_set_start_comm(0);
#endif
					}
				}
				__DlmsSchedule( minute_factor_dlms, shutdown_time_periodic_mode );
			}

			if (((0 == Tick_cloud_time_init()) || (1 == params_modbus_log_prev_enabled()))
			  && (send_init_params == 0))
			{
				meter_count  = 0;
				modbus_count = 0;
				AD_SetGetADCData(1);
				generic_sensor_log_set_save_programword(1);
				sensor_log_set_save_programword(1);
				if (1 == params_pulses_on())
				{
					__checkMeterType();
				}
				else
				{
					shutdown_setInitTelitModule(1);
					message_queue_write(SEND_NETWORK_PARAMS);
				}
				if ( ( TAMPER_HARD_RESET_DAYLY == shutdown_get_tamper_param() ) || ( TAMPER_WATCHDOG == shutdown_get_tamper_param() ) )
				{
					shutdown_read_dlms_count();
				}
			}

			if (1 == rtc_system_getAlarmOccured())
			{
				rtc_system_setAlarmOccured(0);
				/* Disable the Alarm A */
				if ( ALARM_ELAPSED == rtc_system_getAlarmState())
				{
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Disable the Alarm A.\r\n", (int)Tick_Get( SECONDS ) );
					HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
				}
			}
#if 0
			uint32_t psm_reset_times;
			if (0 == params_psm_edrx_tt3412())
			{
				psm_reset_times = 24;
			}
			else
			{
				psm_reset_times = (uint32_t) (24 / params_psm_edrx_tt3412());
				if (0 == psm_reset_times)
				{
					psm_reset_times = 24;
				}
			}

			if ((1 == rtc_system_getAlarmProgrammedReset()) || (shutdown_get_reset_psm() >= psm_reset_times))
			{
				num_pulse_readings = params_maintenance_number_readings();
				if (params_manteinance_delete_memory() != CLEAR_MEMORY_DISABLE_HARD_RESET)
				{
					if (((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0))
					|| (shutdown_get_reset_psm() >= psm_reset_times))
					{
						shutdown_set_reset_psm(0);
						NVIC_SystemReset();
					}
					else
					{
						NVIC_SystemReset();
					}
				}
			}
#endif
			if (0 == __checkPowerMon())
			{
//				if (shutdown_Manager.init_TelitModule == 0)
				{
					if ( (1 == vbackup_int) && (params_manteinance_reed_activation() != 2) )
					{
						vbackup_int = 0;

						if ((1 == shutdown_get_vbackup_sending()) && (1 == vbackup_on))
						{
							LOGLIVE(LEVEL_2,"LOGLIVE> VBACKUP ON\r\n");
							shutdown_set_vbackup_sending(2);
							shutdown_set_tamper_param(TAMPER_BATTERY_VBACKUP);
							__checkMeterType();
							if (1 == params_get_mqtt_dc_on())
							{
								mqtt_stop();
							}
						}
						else if ((3 == shutdown_get_vbackup_sending()) && (0 == vbackup_on))
						{
							LOGLIVE(LEVEL_2,"LOGLIVE> VBACKUP OFF\r\n");
							shutdown_set_vbackup_sending(4);
							shutdown_set_tamper_param(TAMPER_BATTERY_VBACKUP_OFF);
							__checkMeterType();
							if (1 == params_get_mqtt_dc_on())
							{
								mqtt_stop();
							}
						}
						else if (0 == inc_num_readings)
						{
							if (0 == AD_GetADCData())
							{
								LOGLIVE(LEVEL_1,"LOGLIVE> VBACKUP FILTERED\r\n");
								Tick_Force_1Second_Task();
								shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
							}
						}
						/* Exits with 0 */
						return shutdown_Manager.init_TelitModule;
					}
					if (1 == tamper_int)
					{
						tamper_int = 0;

						if ((1 == shutdown_get_tamper_sending()) && (1 == tamper_on))
						{
							LOGLIVE(LEVEL_2,"LOGLIVE> TAMPER ON\r\n");
							shutdown_set_tamper_sending(2);
							shutdown_set_tamper_param(TAMPER_TAMPER_ON);
							__checkMeterType();
							if (1 == params_get_mqtt_dc_on())
							{
								mqtt_stop();
							}
						}
						else if (0 == inc_num_readings)
						{
							if (0 == AD_GetADCData())
							{
								LOGLIVE(LEVEL_1,"LOGLIVE> TAMPER FILTERED\r\n");
								Tick_Force_1Second_Task();
								shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
							}
						}
						/* Exits with 0 */
						return shutdown_Manager.init_TelitModule;
					}
				}
			}

			if (0 == after_reset)
			{
				after_reset = 1;
			}
			else if ((1 == after_reset) && ((generic_485_get_read_time() != 0) || (dlms_client_get_read_time() != 0)))
			{
				after_reset      = 2;
				rtc_refresh_time = 1;
				rtc_system_setGetTime(1);
			}
			else
			{
				after_reset = 2;
			}

//			if ( 1 == params_get_mqtt_dc_on() )
			{
				if ( 1 == dlms_client_get_resend())
				{
					dlms_client_set_resend(0);
					shutdown_set_check_datalogger(1);
					rtc_system_setAlarmOccured(0);
					if ( TIMER_ELAPSED == rtc_system_getTimerState())
					{
						HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
					}
					if (1 == dlms_client_get_dlms_enable())
					{
						shutdown_setInitTelitModule(1);
						if (0 == send_init_params)
						{

							send_init_params = 1;
							message_queue_write(SEND_NETWORK_PARAMS);
						}
						/* reset times */
						rtc_system_reset_remaining_time();
					}
				}
			}

			if (1 == Tick_cloud_time_init())
			{
				if ( 0 == sensor_log_get_write_pulse_acc() )
				{
					if (( 1 == params_pulses_on() ) &&  ( 1 == params_get_pulse_acc()) && __checkRTCQuartAccCreated() )//__checkRTCQuartAcc()
					{
						sensor_log_set_write_pulse_acc(1);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SHUTDOWN> Write Pulse Acc.\r\n", (int)Tick_Get( SECONDS ));
					}
				}
			}
		} /* End if Telit is off*/

		shutdown_set_wchdg_time(params_get_timeout_connection());

		if ( 1 == params_get_mqtt_dc_on() )
		{
			if ( 1 == shutdown_initTelitModule() )
			{
				if ( 0 == mqtt_is_state_idle() )
				{
					if ( ( 1 == AD_GetADOn() ) && ( 0 == message_queue_get_elements() ) )
					{
						__NOP();
					}
					else
					{
//						if ( (( 1 == dlms_client_get_dlms_enable() )
//						     && ( 1 == CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue())))
//						  || (( 0 == dlms_client_get_dlms_enable() )
//						     && ( message_queue_get_elements() != 0 ))
//							)
						{
							mqtt_stop();
						}
					}
				}
			}
			if (0 == __checkPowerMon())
			{
//				if (shutdown_Manager.init_TelitModule == 0)
				{
					if ( (1 == vbackup_int) && (params_manteinance_reed_activation() != 2) )
					{
						vbackup_int = 0;

						if ((1 == shutdown_get_vbackup_sending()) && (1 == vbackup_on))
						{
							LOGLIVE(LEVEL_2,"LOGLIVE> VBACKUP ON\r\n");
							shutdown_set_vbackup_sending(2);
							shutdown_set_tamper_param(TAMPER_BATTERY_VBACKUP);
							__checkMeterType();
							if (1 == params_get_mqtt_dc_on())
							{
								mqtt_stop();
							}
						}
						else if ((3 == shutdown_get_vbackup_sending()) && (0 == vbackup_on))
						{
							LOGLIVE(LEVEL_2,"LOGLIVE> VBACKUP OFF\r\n");
							shutdown_set_vbackup_sending(4);
							shutdown_set_tamper_param(TAMPER_BATTERY_VBACKUP_OFF);
							__checkMeterType();
							if (1 == params_get_mqtt_dc_on())
							{
								mqtt_stop();
							}
						}
						else if (0 == inc_num_readings)
						{
							if (0 == AD_GetADCData())
							{
								LOGLIVE(LEVEL_1,"LOGLIVE> VBACKUP FILTERED\r\n");
								Tick_Force_1Second_Task();
								shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
							}
						}
						/* Exits with 0 */
						return shutdown_Manager.init_TelitModule;
					}
					if (1 == tamper_int)
					{
						tamper_int = 0;

						if ((1 == shutdown_get_tamper_sending()) && (1 == tamper_on))
						{
							LOGLIVE(LEVEL_2,"LOGLIVE> TAMPER ON\r\n");
							shutdown_set_tamper_sending(2);
							shutdown_set_tamper_param(TAMPER_TAMPER_ON);
							__checkMeterType();
							if (1 == params_get_mqtt_dc_on())
							{
								mqtt_stop();
							}
						}
						else if (0 == inc_num_readings)
						{
							if (0 == AD_GetADCData())
							{
								LOGLIVE(LEVEL_1,"LOGLIVE> TAMPER FILTERED\r\n");
								Tick_Force_1Second_Task();
								shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
							}
						}
						shutdown_set_start_count(1);
						/* Exits with 0 */
						return shutdown_Manager.init_TelitModule;
					}
				}
			}
		}
#if 0
		if ( (1 == alarm_int) && ( 1 == params_get_input_alarm_sensor()) )
		{
			alarm_int = 0;

			if ((1 == shutdown_get_alarm_sending()) && (1 == alarm_on))
			{
				LOGLIVE(LEVEL_2,"LOGLIVE> ALARM ON\r\n");
				shutdown_set_alarm_sending(2);
				shutdown_set_tamper_param(TAMPER_INPUT_ALARM_ON);
				shutdown_setInitTelitModule(1);
				message_queue_write(SEND_NETWORK_PARAMS);
				if (1 == params_get_mqtt_dc_on())
				{
					mqtt_stop();
				}
			}
			else if ((3 == shutdown_get_alarm_sending()) && (0 == alarm_on))
			{
				LOGLIVE(LEVEL_2,"LOGLIVE> ALARM OFF\r\n");
				shutdown_set_alarm_sending(4);
				shutdown_set_tamper_param(TAMPER_INPUT_ALARM_OFF);
				shutdown_setInitTelitModule(1);
				message_queue_write(SEND_NETWORK_PARAMS);
				if (1 == params_get_mqtt_dc_on())
				{
					mqtt_stop();
				}
			}
			else if (0 == inc_num_readings)
			{
				if (0 == AD_GetADCData())
				{
					LOGLIVE(LEVEL_1,"LOGLIVE> ALARM FILTERED\r\n");
					Tick_Force_1Second_Task();
					shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
				}
			}
			/* Exits with 0 */
			return shutdown_Manager.init_TelitModule;
		}
#endif
	} /* End if start_count == 0*/
//	shutdown_set_start_count(1);
	return shutdown_Manager.init_TelitModule;
}


/**
 * @}
 */ // End defgroup System_Shudtdown
