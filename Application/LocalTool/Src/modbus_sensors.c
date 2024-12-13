/*
 * modbus_sensors.c
 *
 *  Created on: 23 ene. 2020
 *      Author: Sergio Millán López
 */
#include <stdlib.h>
#include <string.h>
#include <float.h>

#include "modbus_sensors.h"
#include "aqualabo_modbus.h"
#include "generic_modbus.h"
#include "usart.h"
#include "tick.h"

/**
 * @addtogroup System_Modbus_Sensor Modbus Sensors
 * @{
 *
 */

/**
 * @defgroup System_Modbus_Sensor Modbus Sensors
 * @brief
 *
 * @{
 *
 */


typedef enum
{
    IDLE,
    GENERIC_485,
    RAIL_350V, SCHNEIDER_PM3250, EASTRON_SDM530CT, CIRCUTOR_CVM_MINI, DIGIWARE,
    INFO_PANEL, SOUND_DIOCEAN_AS11, SOUND_PDAUDIO, DISPLAY, EXTERNAL_COUNTER,
    SIMATIC_IOT2000, AQUALABO
} modbus_device_type;

typedef struct _modbus_sensors_serial {
	serial_config_stop_bits      stop_bits;
	uint32_t                     baud_rate;
	uint32_t                     parity;
}modbus_sensors_serial;

typedef enum
{
	WARM_UP,
	WARM_UP_FINISHED,
	RUNNING
}modbus_sensors_state;

static modbus_device_type    modbus_sensor_type     = IDLE;						/*!< Type of MODBUS sensor */
static uint8_t               modbus_sensor_comm     = MODBUS_COMM_DISABLED;  	/*!< MOBUS communication state */
static uint8_t               modbus_sensor_get_data = 0;						/*!< Flag */
static uint32_t              modbus_sensor_index    = 0;
static uint8_t               modbus_sensors_data[ MODBUS_FRAME_SIZE ];
static modbus_sensors_serial modbus_sensors_serial_config = {
		.stop_bits = serial_8N1,
		.baud_rate = 0,
		.parity    = no_parity,
};
modbus_sensors_state modbus_state = WARM_UP;

void modbus_sensors_init_task( void )
{
	modbus_state = WARM_UP;
}

serial_config_stop_bits modbus_sensors_get_serial_config_stop_bits( void )
{
	modbus_sensors_serial_config.stop_bits = params_get_modbus_stopbits();
	return modbus_sensors_serial_config.stop_bits;
}

uint32_t modbus_sensors_get_serial_config_baud_rate( void )
{
	modbus_sensors_serial_config.baud_rate = params_get_modbus_baudrate();
	return modbus_sensors_serial_config.baud_rate;
}

serial_config_parity modbus_sensors_get_serial_config_parity( void )
{
	modbus_sensors_serial_config.parity = params_get_modbus_parity();
	return modbus_sensors_serial_config.parity;
}

void modbus_sensors_set_serial_config_stop_bits( serial_config_stop_bits _stop_bits )
{
	modbus_sensors_serial_config.stop_bits = _stop_bits;
	params_set_modbus_stopbits(_stop_bits);
}

void modbus_sensors_set_serial_config_baud_rate( uint32_t _baud_rate )
{
	modbus_sensors_serial_config.baud_rate = _baud_rate;
	params_set_modbus_baudrate(_baud_rate);
}

void modbus_sensors_set_serial_config_parity( serial_config_parity _parity )
{
	modbus_sensors_serial_config.parity = _parity;
	params_set_modbus_parity(_parity);
}

uint8_t  modbus_sensors_get_comm_state( void )
{
	return modbus_sensor_comm;
}

void modbus_sensors_set_comm_state( uint8_t _comm_state )
{
	modbus_sensor_comm = _comm_state;
}

/**
 * @fn void modbus_sensors_set_get_data(uint8_t)
 * @brief Flag to indicate that modbus_sensor data is required or not.
 *
 * @param _get_data
 * 			@arg 0 - No modbus sensor data is required.
 * 			@arg 1 - Modbus sensor data is required.
 */
void modbus_sensors_set_get_data( uint8_t _get_data )
{
	if (1==_get_data)
	{
		__NOP();
	}
	modbus_sensor_get_data = _get_data;
}

/**
 * @fn uint8_t modbus_sensors_get_get_data(void)
 * @brief Reads the modbus sensor data request indicator.
 *
 * @return modbus_sensor_get_data
 * 			@arg 0 - No modbus sensor data is required.
 * 			@arg 1 - Modbus sensor data is required.
 */
uint8_t  modbus_sensors_get_get_data( void )
{
	return modbus_sensor_get_data;
}

uint32_t modbus_sensors_serial_config_usart( void )
{
	uint32_t ret = 0;
	MX_USART2_UART_Custom_Init();
	return ret;
}

uint32_t modbus_sensors_get_param_index( void )
{
	return modbus_sensor_index;
}

void modbus_sensors_task( void )
{
#define DELAY_MODBUS (0)
	static uint8_t rx = 0;
	static uint32_t tickstart, warm_time = 0;
	switch(modbus_state)
	{
	case WARM_UP:
		if ( 1 == modbus_sensors_get_get_data() )
		{
			modbus_sensors_serial_config_usart();
			tickstart    = HAL_GetTick();
			warm_time    = params_get_modbus_warm_time();
			modbus_state = WARM_UP_FINISHED;
			LOGLIVE(LEVEL_1,"LOGLIVE> %d MODBUS SENSORS> START WARM-UP: %d.\r\n", (int)Tick_Get( SECONDS ), (int)warm_time);
		}
		break;
	case WARM_UP_FINISHED:
		if ( (HAL_GetTick() - tickstart) >= warm_time)
		{
			modbus_state = RUNNING;
			LOGLIVE(LEVEL_1,"LOGLIVE> %d MODBUS SENSORS> END WARM-UP: %d.\r\n", (int)Tick_Get( SECONDS ), (int)warm_time);
		}
		break;
	case RUNNING:
		if ( 1 == modbus_sensors_get_get_data() )
		{
			if (DELAY_MODBUS == rx++)
			{
				rx = 0;
				modbus_sensors_rx();
				modbus_sensors_tx();
			}
//			modbus_sensors_tx();
		}
		break;
	default:
		break;
	}
}
/* todo puntero a funciones para tratar cada sensor independiente */
void modbus_sensors_tx( void )
{
    uint32_t n = 0;

    switch( modbus_sensor_type )
    {
        case IDLE:
            break;
        case GENERIC_485:
            n = generic_485_send( modbus_sensors_data );
            modbus_sensors_set_serial_config_baud_rate( modbus_sensors_get_serial_config_baud_rate() );
            modbus_sensors_set_serial_config_stop_bits( modbus_sensors_get_serial_config_stop_bits() );
            modbus_sensors_set_serial_config_parity( modbus_sensors_get_serial_config_parity() );
            modbus_sensors_serial_config_usart();
            break;
        case RAIL_350V:
            break;
        case SCHNEIDER_PM3250:
            break;
        case EASTRON_SDM530CT:
            break;
        case CIRCUTOR_CVM_MINI:
            break;
        case DIGIWARE:
            break;
        case INFO_PANEL:
            break;
        case SOUND_DIOCEAN_AS11:
            break;
        case SOUND_PDAUDIO:
            break;
        case DISPLAY:
            break;
        case EXTERNAL_COUNTER:
            break;
        case SIMATIC_IOT2000:
            break;
        case AQUALABO:
            n = aqualabo_modbus_send( modbus_sensors_data );
            modbus_sensors_set_serial_config_baud_rate( 9600 );
            modbus_sensors_set_serial_config_stop_bits( serial_8N2 );
            modbus_sensors_serial_config_usart();
            break;
        default:
            break;
    }

    serial_modbus_write_modbus( modbus_sensors_data, n );
}

static void __modbusSensorsNextSlave( void )
{
	switch( modbus_sensor_type )
	{
	case IDLE:
		if ( 1 == generic_485_available() ) {
			modbus_sensors_set_comm_state(MODBUS_COMM_STARTED);
			modbus_sensor_type  = GENERIC_485;
		} else if ( 1 == aqualabo_modbus_available() ) {
			modbus_sensors_set_comm_state(MODBUS_COMM_STARTED);
			modbus_sensor_index = aqualabo_get_aqualabo_offset();
			modbus_sensor_type  = AQUALABO;
		} else {
			modbus_sensors_set_comm_state(MODBUS_COMM_END);
		}
		break;
	case GENERIC_485:
		if ( 1 == generic_485_available() ) {
			modbus_sensors_set_comm_state(MODBUS_COMM_STARTED);
			modbus_sensor_type  = GENERIC_485;
		} else if ( 1 == aqualabo_modbus_available() ) {
			modbus_sensors_set_comm_state(MODBUS_COMM_STARTED);
			modbus_sensor_index = aqualabo_get_aqualabo_offset();
			modbus_sensor_type  = AQUALABO;
		} else {
			modbus_sensors_set_comm_state(MODBUS_COMM_END);
			modbus_sensor_type = IDLE;
		}
		break;
	case RAIL_350V:
	case SCHNEIDER_PM3250:
	case EASTRON_SDM530CT:
	case CIRCUTOR_CVM_MINI:
	case DIGIWARE:
		break;
	case INFO_PANEL:
		break;
	case SOUND_DIOCEAN_AS11:
		break;
	case SOUND_PDAUDIO:
		break;
	case DISPLAY:
		break;
	case SIMATIC_IOT2000:
		break;
	case AQUALABO:
		if ( 1 == aqualabo_modbus_available() ) {
			modbus_sensors_set_comm_state(MODBUS_COMM_STARTED);
			modbus_sensor_index = aqualabo_get_aqualabo_offset();
			modbus_sensor_type  = AQUALABO;
		} else {
			modbus_sensors_set_comm_state(MODBUS_COMM_END);
			modbus_sensor_type = IDLE;
		}
		break;
	case EXTERNAL_COUNTER:
	default:
		break;
	}
}

void modbus_sensors_rx( void )
{
    uint32_t n;

    n = serial_modbus_read_modbus( modbus_sensors_data );

    switch( modbus_sensor_type )
    {
        case IDLE:
//        	if ( n > 4 ) {
//            	generic_485_reset_raw_modbus();
//            	generic_485_set_alarm_message( modbus_sensors_data, n );
//            	generic_485_receive( modbus_sensors_data, n );
//        	}
            break;
        case GENERIC_485:
//        	generic_485_reset_raw_modbus();
        	generic_485_receive( modbus_sensors_data, n );
            break;
        case RAIL_350V:
            break;
        case SCHNEIDER_PM3250:
            break;
        case EASTRON_SDM530CT:
            break;
        case CIRCUTOR_CVM_MINI:
            break;
        case DIGIWARE:
            break;
        case INFO_PANEL:
            break;
        case SOUND_DIOCEAN_AS11:
            break;
        case SOUND_PDAUDIO:
            break;
        case DISPLAY:
            break;
        case EXTERNAL_COUNTER:
            break;
        case SIMATIC_IOT2000:
            break;
        case AQUALABO:
            aqualabo_modbus_receive( modbus_sensors_data, n );
            break;
        default:
            break;
    }

    __modbusSensorsNextSlave();
    memset( modbus_sensors_data, 0, sizeof( modbus_sensors_data ));
}

