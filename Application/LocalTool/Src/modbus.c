/**
  ******************************************************************************
  * @file           modbus.c
  * @author 		Datakorum Development Team
  * @brief          Driver to handle modbus transmission and reception data.
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
 * modbus.c
 *
 *  Created on: 19 oct. 2019
 *      Author: smill
 */
/*
 * modbus.c
 *
 *  Created on: 2 jun. 2019
 *      Author: smill
 */
#include <stdlib.h>

#include "ME910.h"
#include "usart.h"
#include "udp_protocol.h"
#include "message_queue.h"
#include "une82326.h"
#include "mbus.h"
#include "modbus.h"
#include "ad.h"
#include "sensor_log.h"
#include "modbus_sensors_log.h"
#include "params.h"
#include "json.h"
#include "leds.h"
#include "shutdown.h"
#include "tick.h"
#include "mbcrc.h"
#include "test_prod.h"

/**
 * @defgroup System_Modbus System Modbus
 * @{
 *
 */


#define GET_CONFIG_1   (4)
#define GET_CONFIG_0   (5)
#define SEND_DATA      (6)
#define GET_NETWORK    (7)
#define GET_PARAMETERS (8)
#define GET_PRESSURE   (9)
#define GET_METER      (10)
#define GET_DK_NETWORK (11)
#define SET_DEV_CONFIG (12)

#define TEST_PROD_MODE (13)
/**
 * @enum
 * @brief MODBUS protocol finite state machine
 *
 */
typedef enum {
	IDLE_MODBUS,               /**< IDLE_MODBUS */
	SEND_CONFIG,               /**< SEND_CONFIG */
	READ_CONFIG,               /**< READ_CONFIG */
	SEND_MODBUS_NETWORK_PARAMS,/**< SEND_MODBUS_NETWORK_PARAMS */
	GET_NETWORK_PARAMS,        /**< GET_NETWORK_PARAMS */
	SEND_PRESSURE,             /**< SEND_PRESSURE */
	SEND_TELEGRAMS,            /**< SEND_TELEGRAMS */
	GET_PRESSURE_DATA,         /**< GET_PRESSURE_DATA */
	SEND_NBIOT,                /**< SEND_NBIOT */
	GET_ONLY_PARAMS,           /**< GET_ONLY_PARAMS */
	GET_ONLY_PRESSURE,         /**< GET_ONLY_PRESSURE */
	GET_ONLY_METER,            /**< GET_ONLY_METER */
	WAIT_MODE,                 /**< WAIT_MODE */
	ERROR_CERT,                /**< ERROR_CERT */
	GET_DK_NETWORK_PARAMS,     /**< GET_DK_NETWORK_PARAMS */
} modbus_protocol_state;

uint8_t modbus_end_session;		/*!< Flag to check whether the modbus */
uint8_t modbus_send;
uint8_t modbus_send_lock;
uint32_t modbus_time_to_end_session;
uint32_t modbus_local_tool_started = 0;
static uint8_t modbus_data[MODBUS_FRAME_SIZE];
char modbus_tool_cert[12];

uint32_t send_nbiot = 0;

modbus_protocol_state modbus_state;

uint32_t timeout_modbus, fw_update;


/** reverses a string 'str' of length 'len'
 * @private
 *  */
static void __reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}


/**
 * Converts a given integer x to string str[]. d is the number
 * of digits required in output. If d is more than the number
 * of digits in x, then 0s are added at the beginning.
 *
 * @private
 */
static int __intToStr(int x, char str[], int d)
{
	int i = 0;
	if (x == 0) {
		str[i++] = '0';
	}
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	__reverse(str, i);
	str[i] = '\0';
	return i;
}

/** Converts a floating point number to string.
 * @private
 */
static void __ftoa(float n, char *res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = __intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		__intToStr((int)fpart, res + i + 1, afterpoint);
	}
}

/**
 * @fn void modbus_vars_init(void)
 * @brief Modbus variables initialization
 *
 */
void modbus_vars_init(void)
{
	modbus_end_session = MODBUS_SESSION_END;
	modbus_send        = 0;
	modbus_send_lock   = 0;
	modbus_state       = IDLE_MODBUS;
}

/**
 * @fn void modbus_init(void)
 * @brief Modbus interface initialization
 *
 */
void modbus_init(void)
{
	static uint32_t init_watchdog = 0;

	modbus_end_session = MODBUS_SESSION_INIT;
	modbus_send        = 0;
	modbus_send_lock   = 0;
	modbus_state       = IDLE_MODBUS;

	/** Enables RS485 */
	serialRS485_module_init(115200);
	serial_modbus_set_initialized(1);

	/** */
	if (0 == params_modbus_log_prev_enabled())
	{
		Tick_update_tick(TICK_MODBUS);
		timeout_modbus = MODBUS_SESSION_TIME;
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR14, timeout_modbus);
		if ( 0 == init_watchdog )
		{
			init_watchdog = 1;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
		}
	}
	else
	{
		modbus_time_to_end_session = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR14);
		Tick_update_tick(TICK_MODBUS);
		timeout_modbus = MODBUS_SESSION_TIME - modbus_time_to_end_session;
	}
}

/**
 * @fn void modbus_deInit(void)
 * @brief De-initialization of the RS485 interface.
 *
 */
void modbus_deInit(void)
{
	/** Disables RS485 physical interface */
	serialRS485_deInit();
}

/**
 * @fn char modbus_get_tool_cert*(void)
 * @brief
 *
 * @return pointer to modbus certification password to enable the use of local tool.
 */
char * modbus_get_tool_cert(void)
{
	if (param.tool_cert[0] == '\0')
	{
		memcpy(modbus_tool_cert, "00000000", 8);
		return modbus_tool_cert;
	}
	else
	{
		return (char *) params_modbus_get_tool_cert();
	}
}

/**
 * @fn void modbus_set_tool_cert(char*)
 * @brief
 *
 * @param tool_cert pointer to new modbus certification password to enable the use of local tool.
 */
void modbus_set_tool_cert(char * tool_cert)
{
	strncpy(modbus_tool_cert, tool_cert, 8);
	strncpy((char *) params_modbus_get_tool_cert(), tool_cert, 8);
}

/**
 * @fn uint8_t modbus_get_end_session(void)
 * @brief
 *
 * @return
 */
uint8_t modbus_get_end_session( void )
{
	return modbus_end_session;
}

/**
 * @fn uint32_t modbus_get_timeout_modbus(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t modbus_get_timeout_modbus( void )
{
	return timeout_modbus;
}

/**
 * @fn uint8_t modbus_get_send(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t modbus_get_send( void )
{
	return modbus_send;
}

/**
 * @fn void modbus_set_send(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _modbus_send
 */
void modbus_set_send( uint8_t _modbus_send )
{
	modbus_send = _modbus_send;
}

/**
 * @fn uint8_t modbus_get_send_lock(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t modbus_get_send_lock( void )
{
	return modbus_send_lock;
}

/**
 * @fn void modbus_set_send_lock(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _modbus_send_lock
 */
void modbus_set_send_lock( uint8_t _modbus_send_lock )
{
	modbus_send_lock = _modbus_send_lock;
}

/**
 * @fn void __set_modbus_end_session(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __set_modbus_end_session( void )
{
	modbus_end_session = MODBUS_SESSION_END;
	shutdown_set(1, params_config_read_time());
}

/**
 * @fn uint8_t modbus_get_send(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t modbus_get_local_tool_started( void )
{
	return modbus_local_tool_started;
}

/**
 * @fn void modbus_set_send(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _modbus_send
 */
void modbus_set_local_tool_started( uint32_t _modbus_local_tool_started )
{
	modbus_local_tool_started = _modbus_local_tool_started;
}

/**
 * @fn uint32_t modbus_rcx(void)
 * @brief only used for local tool
 *
 * @return n_bytes number of bytes received.
 */
uint32_t modbus_rcx(void)
{
    uint32_t n;
    uint8_t  r;

    /* Gets elements from the modbus_data buffer */
    n = serial_modbus_read_modbus(modbus_data);

    /** If there is data in the buffer decodes the modbus json */
    if (n > 0)
    {
    	modbus_end_session = MODBUS_SESSION_INIT;
		r = json_modbus_decode((char *) modbus_data, n);

		if (TEST_PROD_MODE == r)
		{
			test_prod_run(1);
		}

		/** Process json response*/
		if (GET_CONFIG_1 == r)
		{
#if defined(UNE82326)
			une82326_set_start_comm(1);
#elif defined(MBUS)
			mbus_set_start_comm(1);
#endif
			modbus_state = SEND_CONFIG;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			AD_SetGetADCData(1);
			AD_setEnd(0);
			AD_SetRegisterADCData(1);
			modbus_set_local_tool_started(1);
		}
		else if (GET_CONFIG_0 == r)
		{
			__set_modbus_end_session();
			modbus_set_local_tool_started(1);
		}
		else if (SEND_DATA == r)
		{
			modbus_state   = GET_PRESSURE_DATA;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			send_nbiot   = 1;
			shutdown_set_tamper_param(TAMPER_LOCAL_TOOL);
			AD_SetGetADCData(1);
			AD_setEnd(0);
			AD_SetRegisterADCData(1);
			modbus_set_local_tool_started(1);
		}
		else if (GET_NETWORK == r)
		{
			modbus_state = GET_NETWORK_PARAMS;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			modbus_set_local_tool_started(1);
		}
		else if (GET_DK_NETWORK == r)
		{
			modbus_state = GET_DK_NETWORK_PARAMS;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			modbus_set_local_tool_started(1);
		}
		else if (GET_PARAMETERS == r)
		{
			modbus_state = GET_ONLY_PARAMS;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			modbus_set_local_tool_started(1);
		}
		else if (GET_PRESSURE == r)
		{
			modbus_state = GET_ONLY_PRESSURE;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			AD_SetGetADCData(1);
			AD_setEnd(0);
			AD_SetRegisterADCData(1);
			modbus_set_local_tool_started(1);
		}
		else if (GET_METER == r)
		{
			modbus_state = GET_ONLY_METER;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
			modbus_set_send(1);
#if defined(UNE82326)
			une82326_set_write_record(0);
#elif defined(MBUS)
			mbus_set_last_device(0);
			mbus_set_end(0);
			mbus_set_start_comm(1);
			modbus_set_local_tool_started(1);
#endif
		}
		else if ( SET_DEV_CONFIG == r )
		{
			modbus_state = READ_CONFIG;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
		}
		else if ((1 == r) || (3 == r))
		{
			if (3 == r)
			{
				fw_update = 1;
			}
			else
			{
				fw_update = 0;
			}
			modbus_state = READ_CONFIG;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
		}
		else if (2 == r)
		{
			modbus_state = ERROR_CERT;
			Tick_update_tick(TICK_MODBUS);
			timeout_modbus = MODBUS_SESSION_TIME;
			shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
		}
    }

    return n;
}

/**
 * @fn uint32_t modbus_pc_send(uint8_t*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint32_t modbus_pc_send( uint8_t * data )
{
    uint32_t i = 0;

    i += datalog_buffer_read_modbus( (char *)&data[i] );

    return i;
}

/**
 * @fn uint32_t modbus_pulses_pc_send(uint8_t*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint32_t modbus_pulses_pc_send( uint8_t * data )
{
    uint32_t i = 0;

    i += datalog_buffer_read_pulses_modbus( (char *)&data[i] );

    return i;
}

/**
 * @fn uint32_t modbus_send_network_params(char*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint32_t modbus_send_network_params( char * data )
{
//	sprintf(data,
//			"%s;%s;%s;%s;%s;%d;%d;%d;%d;%d;%d;%d;%s;%d;%d;%d;%d;%d;\r",
//			ME910_ICCID(),
//			ME910_IMEI(),
//			ME910_IMSI(),
//			params_get_server(),
//			params_get_APN(),
//			(int)ME910_network_params_rssi(),
//			(int)ME910_network_params_rsrq(),
//			(int)ME910_network_params_rsrp(),
//			(int)ME910_network_params_csq(),
//			(int)ME910_network_params_pci(),
//			(int)ME910_network_params_sinr(),
//			(int)ME910_network_params_tx_power(),
//			ME910_network_params_cell_id(),
//			(int)params_maintenance_number_readings(),//TODO: Remove
//			(int)params_maintenance_number_ok_sendings(),//TODO: Remove
//			(int)params_maintenance_number_nok_sendings(),//TODO: Remove
//			(int)ME910_network_params_creg(),
//			(int)ME910_network_params_abnd()
//			);
#if 1
	sprintf(data,
			"%s;%s;%s;%s;"
			"%s;%s;"												      //Communication.2.
			"%d;%d;%d;%d;%d;%d;%s;%d;%d;"								  //Network Params.9.
			"%d;%d;%d;%d;%d;%d;%d;%d;%d;"						  		  //Send Params.9.
			"%d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d "//Err Log.48.
			"%d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d %d:%d "
			"%d:%d %d:%d %d:%d %d:%d;"
			"%d;%d;"
			"%s;%s;%s;"								                      //Datalogger Info.3.
			"\r\n",
			ME910_IMEI(),                        //1.
			ME910_ICCID(),                       //2.
			ME910_IMEI(),                        //3.
			ME910_IMSI(),                        //4.
			//Communication.
			params_get_server(),                 //1.
			params_get_APN(),                    //2.
			// Network Params.
			(int)ME910_network_params_rssi(),    //1.
			(int)ME910_network_params_rsrq(),    //2.
			(int)ME910_network_params_rsrp(),    //3.
			(int)ME910_network_params_pci(),     //4.
			(int)ME910_network_params_sinr(),    //5.
			(int)ME910_network_params_tx_power(),//6.
			ME910_network_params_cell_id(),      //7.
			(int)ME910_network_params_creg(),    //8.
			(int)ME910_network_params_abnd(),    //9.
			//Send Params.
			(int)params_maintenance_number_readings(),    //1.
			(int)params_maintenance_number_ok_sendings(), //2.
			(int)params_maintenance_number_nok_sendings(),//3.
			(int)params_attach_error_get_SIM_error(),     //4.
			(int)params_attach_error_get_CSQ_error(),     //5.
			(int)params_attach_error_get_CEREG_error(),   //6.
			(int)params_attach_error_get_TSO_error(),     //7.
			(int)params_attach_error_get_TSE_error(),     //8.
			(int)params_attach_error_get_IP_error(),      //9.
			//Err Log.
			(int)params_attach_get_log_timestamp_index(0),  (int)params_attach_get_log_index(0),
			(int)params_attach_get_log_timestamp_index(1),  (int)params_attach_get_log_index(1),
			(int)params_attach_get_log_timestamp_index(2),  (int)params_attach_get_log_index(2),
			(int)params_attach_get_log_timestamp_index(3),  (int)params_attach_get_log_index(3),
			(int)params_attach_get_log_timestamp_index(4),  (int)params_attach_get_log_index(4),
			(int)params_attach_get_log_timestamp_index(5),  (int)params_attach_get_log_index(5),
			(int)params_attach_get_log_timestamp_index(6),  (int)params_attach_get_log_index(6),
			(int)params_attach_get_log_timestamp_index(7),  (int)params_attach_get_log_index(7),
			(int)params_attach_get_log_timestamp_index(8),  (int)params_attach_get_log_index(8),
			(int)params_attach_get_log_timestamp_index(9),  (int)params_attach_get_log_index(9),
			(int)params_attach_get_log_timestamp_index(10), (int)params_attach_get_log_index(10),
			(int)params_attach_get_log_timestamp_index(11), (int)params_attach_get_log_index(11),
			(int)params_attach_get_log_timestamp_index(12), (int)params_attach_get_log_index(12),
			(int)params_attach_get_log_timestamp_index(13), (int)params_attach_get_log_index(13),
			(int)params_attach_get_log_timestamp_index(14), (int)params_attach_get_log_index(14),
			(int)params_attach_get_log_timestamp_index(15), (int)params_attach_get_log_index(15),
			(int)params_attach_get_log_timestamp_index(16), (int)params_attach_get_log_index(16),
			(int)params_attach_get_log_timestamp_index(17), (int)params_attach_get_log_index(17),
			(int)params_attach_get_log_timestamp_index(18), (int)params_attach_get_log_index(18),
			(int)params_attach_get_log_timestamp_index(19), (int)params_attach_get_log_index(19),
			(int)params_attach_get_log_timestamp_index(20), (int)params_attach_get_log_index(20),
			(int)params_attach_get_log_timestamp_index(21), (int)params_attach_get_log_index(21),
			(int)params_attach_get_log_timestamp_index(22), (int)params_attach_get_log_index(22),
			(int)params_attach_get_log_timestamp_index(23), (int)params_attach_get_log_index(23),//48.
			(int)ME910_network_params_rtt_mean(),        //1.
			(int)ME910_get_connection_time(),            //2.
			//Datalogger Info.3.
			Datalogger_Get_Info(),                  //1.
			sensor_log_GetDatalogger_Info(),        //2.
			modbus_sensors_log_GetDatalogger_Info() //3.
	);
	return strlen(data);
#endif
//	uint32_t len = udp_protocol_send_network_params_msg(data, 1);

//	return len;
}

/**
 * @fn uint32_t modbus_send_pressure(char*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint32_t modbus_send_pressure( char * data )
{
	uint32_t size_str, i, index;
	char     pressure_val_[7];
	char     pressure_val[8];
	float_t  pressure_float;
	float_t *pressure_index;

	if ( 1 == sensor_log_is_pressure_log_init() ) {
		index = sensor_log_GetPressureValuesCurrentIndex();
		sensor_log_WritePressureLogIndex( index );
		memset( data, 0, sizeof( modbus_data ));
		pressure_index = sensor_log_GetPressureValues();
		for (i = 0; i < 15; i++) {
			pressure_float = *(pressure_index + i);
			__ftoa(pressure_float, pressure_val_, 3);
			sprintf( pressure_val,
					"%s;",
					pressure_val_);
			size_str = strlen(data);
			sprintf( (data + size_str),
					"%s",
					pressure_val );
		}
		size_str = strlen(data);
		sprintf(data + size_str,
				"%s;%s;%s;\r\n",
				sensor_log_GetPressureAvg(),
				sensor_log_GetPressureMax(),
				sensor_log_GetPressureMin()
		);

		return strlen(data);
	} else {
		return 0;
	}
}

/**
 * @fn uint32_t modbus_send_config(char*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint32_t modbus_send_config( char * data )
{
#if 1
	sprintf(data,
			"{\"Gateway-ID\":\"%s\",\"Meter-ID\":\"%s\",\"Dewa-ID\":\"%s\",\"rt\":\"%d\","
			"\"st\":\"%d\",\"sendalarm\":\"%d\",\"fwupdate\":\"%d\",\"firmware_version\":\"%d.%d\",\"modbus_enabled\":\"%d\""
			",\"overpressure_alarm\":\"%d\",\"lowpressure_alarm\":\"%d\",\"readings\":\"%d\",\"ok_sends\":\"%d\",\"nok_sends\":\"%d\",\"reset_time\":\"%d\"}\r\n",
			Telit_dev_identifier(),
#if defined (MBUS)
			mbus_get_watermeter_manufacturer_serial_num(),
			mbus_get_dewa_serial_num(),
#else
			"x",
			"x",
#endif
			(int)(params_config_read_time()/60),
			(int)(params_config_send_time()/60),
			(int)params_maintenance_enable_alarm_sending(),
			(int)fw_update,
			(int)params_version_major(),
			(int)params_version_minor(),
			(int)params_modbus_log_enabled(),
			(int)params_config_get_overpressure_alarm(),
			(int)params_config_get_lowpressure_alarm(),
//			modbus_get_tool_cert()
			(int)params_maintenance_number_readings(),
			(int)params_maintenance_number_ok_sendings(),
			(int)params_maintenance_number_nok_sendings(),
			(int)params_get_time_reset()
			);
	return strlen(data);
#endif
#if 0
	uint32_t len = udp_protocol_send_network_params_msg(data, 1);

	return len;
#endif
}

/**
 * @fn uint32_t modbus_send_error(char*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint32_t modbus_send_error( char * data )
{
	sprintf(data, "error\r\n");

	return strlen(data);
}

/**
 * @fn void modbus_trx(void)
 * @brief
 *
 * @pre
 * @post
 */
void modbus_trx( void )
{
	uint32_t n;
//	static uint32_t send_nbiot = 0;

#if defined(MBUS)
	if (0 == rest_upgrading_firmware())
	{
#elif defined(UNE82326)
    if  (0 == rest_upgrading_firmware())
    {
#endif
		switch(modbus_state)
		{
			/**/
			case IDLE_MODBUS:
				/* If local tool is disabled gets tick and save timeout*/
				if (0 == params_modbus_log_prev_enabled())
				{
					Tick_update_tick(TICK_MODBUS);
					timeout_modbus = MODBUS_SESSION_TIME;
					HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR14, timeout_modbus);
				}
				else
				{
					modbus_time_to_end_session = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR14);
					Tick_update_tick(TICK_MODBUS);
					timeout_modbus = MODBUS_SESSION_TIME - modbus_time_to_end_session;
				}

				modbus_state = WAIT_MODE;
			break;
			/**/
			case SEND_TELEGRAMS:
				if (0 == params_pulses_on())
				{
#if defined(UNE82326)
					if (1 == une82326_get_last_device())
					{
#elif defined(MBUS)
					if (1 == mbus_get_last_device())
					{
#endif
						if ((1 == modbus_get_send()) && (0 == modbus_get_send_lock()))
						{
							n = modbus_pc_send(modbus_data);
							serial_modbus_write_modbus(modbus_data, n);
							if (1 == send_nbiot)
							{
								send_nbiot   = 0;
								modbus_state = SEND_NBIOT;
							}
							else
							{
								modbus_state = WAIT_MODE;
							}
						}
					}
				}
				else
				{
					n = modbus_pulses_pc_send(modbus_data);
					serial_modbus_write_modbus(modbus_data, n);
					if (1 == send_nbiot)
					{
						send_nbiot   = 0;
						modbus_state = SEND_NBIOT;
					}
					else
					{
						modbus_state = WAIT_MODE;
					}
				}
			break;
			/**/
			case SEND_CONFIG:
				n = modbus_send_config((char *)modbus_data);
				serial_modbus_write_modbus(modbus_data, n);
				modbus_state = SEND_MODBUS_NETWORK_PARAMS;
				modbus_set_send( 1 );
				break;
			/**/
			case SEND_MODBUS_NETWORK_PARAMS:
				n = modbus_send_network_params( (char *)modbus_data );
				serial_modbus_write_modbus(modbus_data, n);
				modbus_state = SEND_PRESSURE;
				modbus_set_send( 1 );
				break;
			/**/
			case GET_NETWORK_PARAMS:
				if ((0 == ME910_get_network_params_get()) && (0 == shutdown_initTelitModule()))
				{
					leds_set_NET_status(LOW_POWER);
					shutdown_reset_watchdog();
					shutdown_setInitTelitModule(1);
					ME910_set_network_params_get(1);
					ME910_set_dk_network_params_get(0);
					ME910_reinit_device();
					Telit_socket_set_available( NOT_CONNECTED );
				}
				else
				{
					if ((2 == ME910_get_network_params_get()) || (0 == Tick_cloud_time_init()))
					{
						leds_set_NET_status(NB_Y);
						ME910_set_network_params_get(0);
						ME910_set_dk_network_params_get(0);
						n = modbus_send_network_params((char *) modbus_data);
						serial_modbus_write_modbus(modbus_data, n);
						modbus_state = WAIT_MODE;
						modbus_set_send(1);
					}
				}
			break;
			/**/
			case SEND_PRESSURE:
				AD_SetRegisterADCData(0);
	//			sensor_log_Log();
	//			sensor_log_pressure_store_telegram_value();
				n = modbus_send_pressure((char *) modbus_data);
				serial_modbus_write_modbus(modbus_data, n);
				modbus_state = SEND_TELEGRAMS;
				modbus_set_send(1);
#if defined(UNE82326)
				une82326_set_write_record(0);
	//			une82326_set_start_comm(1);
#elif defined(MBUS)
				mbus_set_end(0);
				mbus_set_start_comm(1);
#endif
			break;

			case GET_PRESSURE_DATA:
				if (1 == AD_getEnd())
				{
					AD_SetRegisterADCData(0);
					sensor_log_Log();
					n = modbus_send_pressure((char *) modbus_data);
					serial_modbus_write_modbus(modbus_data, n);
					modbus_state = SEND_TELEGRAMS;
					modbus_set_send(1);
#if defined(UNE82326)
					une82326_set_write_record(0);
#elif defined(MBUS)
					mbus_set_end(0);
					mbus_set_start_comm(1);
#endif
				}
			break;

			case SEND_NBIOT:
				if ((ME910_get_error() == ERROR_CONNECTED) && (1 == Tick_cloud_time_init()))
				{
					message_queue_write(SEND_NETWORK_PARAMS);
					if (0 == params_pulses_on())
					{
#if defined(UNE82326)
						une82326_set_start_comm(1);
#elif defined(MBUS)
						mbus_set_end(0);
						mbus_set_start_comm(1);
#endif
					}
					else
					{
						message_queue_write(SEND_SENSOR);
					}
					shutdown_setInitTelitModule(1);
					udp_protocol_reconnect(0);
//					message_queue_write(SEND_NETWORK_PARAMS);
					modbus_end_session = MODBUS_SESSION_SEND_TELEGRAM;
					modbus_state = WAIT_MODE;
				}
				else
				{
					modbus_state = WAIT_MODE;
				}
			break;

			case READ_CONFIG:
				params_check_for_changes();
				n = modbus_send_config((char *) modbus_data);
				serial_modbus_write_modbus(modbus_data, n);
				modbus_end_session = MODBUS_SESSION_PARAM_CHANGE;
				modbus_state = SEND_NBIOT;
			break;

			case GET_ONLY_PARAMS:
				n = modbus_send_config((char *) modbus_data);
				serial_modbus_write_modbus(modbus_data, n);
				modbus_state = WAIT_MODE;
			break;

			case GET_ONLY_PRESSURE:
				if (1 == AD_getEnd())
				{
					AD_SetRegisterADCData(0);
					sensor_log_Log();
					n = modbus_send_pressure((char *) modbus_data );
					serial_modbus_write_modbus(modbus_data, n);
					modbus_state = WAIT_MODE;
				}
			break;

			case GET_ONLY_METER:
				if (0 == params_pulses_on())
				{
#if defined(UNE82326)
					if (1 == une82326_get_last_device())
					{
#elif defined(MBUS)
					if (1 == mbus_get_last_device())
					{
#endif
						if ((1 == modbus_get_send()) && (0 == modbus_get_send_lock()))
						{
							memset( modbus_data, 0, sizeof( modbus_data ));
							n = modbus_pc_send(modbus_data);
							serial_modbus_write_modbus(modbus_data, n);
							modbus_state = WAIT_MODE;
						}
					}
				}
				else
				{
					n = modbus_pulses_pc_send(modbus_data);
					serial_modbus_write_modbus(modbus_data, n);
					modbus_state = WAIT_MODE;
				}
			break;

			case WAIT_MODE:
			break;

			case ERROR_CERT:
				n = modbus_send_error((char *)modbus_data);
				serial_modbus_write_modbus(modbus_data, n);
				modbus_state = WAIT_MODE;
			break;

			case GET_DK_NETWORK_PARAMS:
				if ((0 == ME910_get_network_params_get()) && ((0 == shutdown_initTelitModule()) || (0 == Tick_cloud_time_init())))
				{
					leds_set_NET_status(LOW_POWER);
					shutdown_reset_watchdog();
					shutdown_setInitTelitModule(1);
					ME910_set_network_params_get(1);
					ME910_set_dk_network_params_get(55);
					ME910_reinit_device();
					Telit_socket_set_available(NOT_CONNECTED);
				}
				else
				{
					if ((2 == ME910_get_network_params_get()) /*|| ( 0 == Tick_cloud_time_init() )*/ )
					{
						leds_set_NET_status(NB_Y);
						ME910_set_network_params_get(0);
						ME910_set_dk_network_params_get(0);
						n = modbus_send_network_params( (char *)modbus_data );
						serial_modbus_write_modbus(modbus_data, n);
						modbus_state = WAIT_MODE;
						modbus_set_send(1);
					}
				}
			break;

			default:
			break;
		}

		/* Checks Local Tool timeout */
		if (CHECK_ELAPSED_TIME(timeout_modbus, TICK_MODBUS))
		{
			__set_modbus_end_session();
		}
	}
}

/**
 * @fn uint32_t modbus_local_tool_task(void)
 * @brief
 *
 * @return deinit @ref local_tool local tool operation state.
 */

uint32_t modbus_local_tool_task( void )
 {

	/**
		 * @enum local_tool
	 * @brief local tool operation state
	 *
	 */
	enum {
		LOCAL_TOOL_OFF = 0,   /**< LOCAL_TOOL_OFF */
		LOCAL_TOOL_DEINIT = 1,/**< LOCAL_TOOL_DEINIT */
		LOCAL_TOOL_ON = 2     /**< LOCAL_TOOL_ON */
	};

	uint32_t deinit = LOCAL_TOOL_OFF;

	/**/
	if ((1 == params_modbus_log_enabled()) || (1 == params_modbus_log_prev_enabled())
			|| (MODBUS_SESSION_PARAM_CHANGE == modbus_get_end_session()))
	{
		/**/
		if ((1 == leds_device_switch_on()) || ( 1 == params_modbus_log_prev_enabled()))
		{
			if (0 == serial_modbus_is_initialized())
			{
				modbus_init();
				if (1 == leds_device_switch_on())
				{
//					leds_LED_On(LED_GREEN);
					leds_set_METER_status(METER_LOCAL_TOOL);
				}
			}
			deinit = LOCAL_TOOL_ON;
			if (0 == test_prod_run_on())
			{
				modbus_rcx();
				modbus_trx();
			}
		}
		else if (1 == serial_modbus_is_initialized())
		{
			deinit = LOCAL_TOOL_DEINIT;
		}
	}
	else if (1 == serial_modbus_is_initialized())
	{
		deinit = LOCAL_TOOL_DEINIT;
	}

	if (LOCAL_TOOL_DEINIT == deinit)
	{
		modbus_end_session = MODBUS_SESSION_END;
		modbus_deInit();
		leds_set_NET_status(LOW_POWER);
	}
	return deinit;
}

/**
 * @}
 */ //End defgroup System_Modbus


/**
 * @defgroup System_Test System Test
 * @brief
 * @{
 *  */

/**
 * @defgroup System_Test_Modbus System Test Modbus
 * @brief Modbus functionality is tested with the sound AS11.
 * @{
 * */

/**
 * @struct
 * @brief AS11 to store sensor value for test purposes.
 *
 */
static struct
{
    unsigned char enable; /**< Sensor enable flag */
    unsigned short value; /**< Sensor data */
} soundAS11;

/**
 * @fn unsigned short modbus_test_as11_get(void)
 * @brief Returns the value of the AS11 module for test purposes.
 *
 * @return
 */
unsigned short modbus_test_as11_get(void)
{
    if(! soundAS11.enable)
    {
        soundAS11.enable = 1;
    }
    if (soundAS11.value < 300)
    {
        soundAS11.value = 300;
    }
    return soundAS11.value;
}

/**
 * @fn unsigned char modbus_as11_send(unsigned char*)
 * @brief Sends data to the AS11 module for test purposes
 *
 * @param data
 * @return i number of data values.
 */
unsigned char modbus_as11_send( unsigned char *data )
{
    unsigned short addr, n, crc;
    unsigned char i = 0;

    addr = 0;
    n    = 2;

    data[i++] = 1;
    data[i++] = 3;
    data[i++] = addr >> 8;
    data[i++] = addr;
    data[i++] = n >> 8;
    data[i++] = n;
    crc       = usMBCRC16( data, i );
    data[i++] = crc;
    data[i++] = crc >> 8;

    return i;
}

/**
 * @fn void modbus_as11_receive(unsigned char*, unsigned char)
 * @brief Gets data from AS11 module for test puposes.
 *
 * @param data
 * @param n
 */
void modbus_as11_receive( unsigned char *data, unsigned char n )
{
    unsigned char *ch;
    unsigned short crc1;

    if( n > 4 ) {
        memcpy( &crc1, &data[n-2], sizeof(crc1) );
        if( crc1 == usMBCRC16( data, n-2 )) {
            ch = data;
            if( *ch++ == 1 ) {
                if( *ch++ == 3 ) {
                    if( *ch++ == 4 ) {
                        soundAS11.value = mb_swaps( &data[ 3 ]);
                    }
                }
            }
        }
    }
}

/**
 * @fn void modbus_test_trx(void)
 * @brief Sends modbus_data[MODBUS_FRAME_SIZE] trough Modbus port for testing purposes.
 *
 */
void modbus_test_trx( void )
{
	uint32_t n = 0;

	n = modbus_as11_send(modbus_data);
	serial_modbus_write_modbus(modbus_data, n);

}

/**
 * @fn void modbus_test_rcx(void)
 * @brief Receives data though Modbus
 *
 * @pre
 * @post
 */
void modbus_test_rcx( void )
{
	uint32_t n;

	n = serial_modbus_read_modbus(modbus_data);
	modbus_as11_receive(modbus_data, n);
}

/**
 * @}
 */ //End defgroup System_Test_Modbus

/**
 * @}
 */ //End defgroup System_Test

