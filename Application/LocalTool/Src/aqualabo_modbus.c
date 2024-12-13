/**
  ******************************************************************************
  * @file           aqualabo_modbus.c
  * @author 		Datakorum Development Team
  * @brief			Driver to read data from aqualabo water quality sensors
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
 * aqualabo_modbus.c
 *
 *  Created on: 23 ene. 2020
 *      Author: Sergio Millán López
 */
#include <string.h>
#include <float.h>

#include "modbus.h"
#include "modbus_sensors.h"
#include "aqualabo_modbus.h"
#include "mbcrc.h"
#include "serial_une82326.h"
#include "leds.h"


/**
 * @addtogroup System_Modbus_Sensor Modbus Sensors
 * @{
 *
 */


/**
 * @defgroup System_Modbus_Aqualabo Aqualabo Modbus
 * @brief
 *
 * @{
 *
 */


/**
 * @enum aqualabo_sensor
 * @brief
 *
 */
enum aqualabo_sensor
{
	O2,             	/*!< O2 */
	pH,     	        /*!< pH */
	SAL,        	    /*!< SAL */
	TURB,          		/*!< TURB */
	LAST_AQUA_SENSOR	/*!< LAST_AQUA_SENSOR */
};

/**
 * @enum aqualabo_cmd
 * @brief
 *
 */
enum aqualabo_cmd
{
	CMD_70,	    	/*!< CMD_70 */
	CMD_80, 	    /*!< CMD_80 */
	CMD_100,	    /*!< CMD_100 */
	LAST_AQUA_CMD	/*!< LAST_AQUA_CMD */
};

/**
 * @enum aqualabo_measures
 * @brief
 *
 */
enum aqualabo_measures
{
	MT = 0x01,/**< MT */
	M1 = 0x02,/**< M1 */
	M2 = 0x04,/**< M2 */
	M3 = 0x08,/**< M3 */
	M4 = 0x10 /**< M4 */
};

/**
 * @enum aqualabo_status
 * @brief
 *
 */
enum aqualabo_status
{
	DISABLED,    	            /*!< DISABLED */
	HEATING_TIME,   	        /*!< HEATING_TIME */
	START_MEASURE,      	    /*!< START_MEASURE */
	WAITING_FOR_MEASURE_TIME,	/*!< WAITING_FOR_MEASURE_TIME */
	GET_STATUS,          	    /*!< GET_STATUS */
	GET_MEASURE             	/*!< GET_MEASURE */
};

/**
 * @enum aqualabo_comm_status
 * @brief
 *
 */
enum aqualabo_comm_status
{
	COMMUNICATON_DISABLED = 0,/**< COMMUNICATON_DISABLED */
	START_COMMUNICATION = 1,  /**< START_COMMUNICATION */
	END_COMMUNICATION = 2     /**< END_COMMUNICATION */
};


/**
 * @struct
 * @brief Aqualabo sensor structure, holds all variables and sensor data
 *
 */
struct
{
    uint8_t                 enable;		/*!< Status of power rail that feeds the MC1308 step up regulator */
	uint8_t 				id;
	uint8_t					response_present;
	uint8_t					comm_state;		/*!< Communication state
	 	 	 	 	 	 	 	 	 	 	 	 	 @arg 0 - COMMUNICATON_DISABLED
	 	 	 	 	 	 	 	 	 	 	 	 	 @arg 1 - START COMMUNICATION
	 	 	 	 	 	 	 	 	 	 	 	 	 @arg 2 - END_COMMUNICATION */
	uint8_t					get_data;
    const uint8_t           addr[LAST_AQUA_SENSOR];
    const uint8_t			measures_selected[LAST_AQUA_SENSOR];
    const uint8_t			reg_addr[LAST_AQUA_CMD];
    const uint8_t			reg_size[LAST_AQUA_CMD];
	const uint8_t			offset[LAST_AQUA_SENSOR];
    enum aqualabo_status    status;
    uint16_t                response[5];
    float                   value[10];
    float					temperature[LAST_AQUA_SENSOR];
} aqualabo =
{
    .status = DISABLED,
	.comm_state = COMMUNICATON_DISABLED,
    .id = pH,//TURB,
    .addr[O2] = 10,
    .addr[pH] = 20,
    .addr[SAL] = 30,
    .addr[TURB] = 40,
    .measures_selected[O2] = MT | M1 | M2 | M3,
    .measures_selected[pH] = MT | M1 | M2,
    .measures_selected[SAL] = MT | M1 | M2 | M3,
    .measures_selected[TURB] = MT | M1,
    .offset[O2] = 1,
    .offset[pH] = 4,
    .offset[SAL]= 6,
    .offset[TURB]= 9,
    .reg_addr[CMD_70] = 0x01,
    .reg_addr[CMD_80] = 0x64,
    .reg_addr[CMD_100] = 0x53,
    .reg_size[CMD_70] = 1,
    .reg_size[CMD_80] = 5,
    .reg_size[CMD_100] = 10,
    .temperature[O2] = FLT_MAX,
    .temperature[pH] = FLT_MAX,
    .temperature[SAL] = FLT_MAX,
    .temperature[TURB] = FLT_MAX
};


/**
 * @fn void aqualabo_set_comm_state(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _comm_state
 */
void aqualabo_set_comm_state( uint8_t _comm_state )
{
	aqualabo.comm_state = _comm_state;
}

/**
 * @fn uint8_t aqualabo_get_comm_state(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t aqualabo_get_comm_state( void )
{
	return aqualabo.comm_state;
}

/**
 * @fn void __disableAqualaboSensor(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __disableAqualaboSensor( void )
{
	leds_LED_Off(LED_GREEN);

	DISABLE_AQUALABO_SENSOR
	HAL_Delay(10);
	DISABLE_AQUALABO_PWR_SUPPLY
	modbus_sensors_set_get_data(0);
	aqualabo.comm_state = END_COMMUNICATION;
    aqualabo.status     = DISABLED;
}

/**
 * @fn void aqualabo_modbus_sensor_disable(void)
 * @brief Disables internal UNE power supply pin for external sensor supply.
 *
 * @note Two pins are needed for suppling externally the Aqualabo sensor.
 * 	- One pin to enable the power rail to feed MC1308 step up regulator.
 * 	- One pin to enable the SEL line that outputs the line.
 */
void aqualabo_modbus_sensor_disable( void )
{
	DISABLE_AQUALABO_SENSOR
	HAL_Delay(10);
	DISABLE_AQUALABO_PWR_SUPPLY
}

/**
 * @fn uint8_t aqualabo_modbus_available(void)
 * @brief
 * @return
 */
uint8_t aqualabo_modbus_available( void )
{
    static uint8_t try, heat;
    uint8_t        r = 0, i, mask;
#define  CHECK_TAMPER	(HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ))
    if( aqualabo.enable ) {
        switch( aqualabo.status )
        {
            case DISABLED:
            	ENABLE_AQUALABO_SENSOR
            	if ( START_COMMUNICATION == aqualabo.comm_state ) {
            		if ( 0 == CHECK_TAMPER )
            		{
            			leds_LED_On(LED_GREEN);
            		}
            		heat            = 0;
            		aqualabo.status = HEATING_TIME;
            	}
                r = 1;
                break;
            case HEATING_TIME:
            	if ( heat++ >= 3 ) {
            		heat            = 0;
            		aqualabo.status = START_MEASURE;
            	}
            	r = 1;
            	break;
            case START_MEASURE:
                if ( aqualabo.response_present == 0x10 ) {
                    try = 3;
                    aqualabo.status = GET_STATUS; // timeout = 2; status = WAITING_FOR_MEASURE_TIME;
                    r = 1;
                } else {
                	__disableAqualaboSensor();
                }
                break;
//            case WAITING_FOR_MEASURE_TIME:
//                break;
            case GET_STATUS:
                // Si NO response, status = DISABLED; r = 0;
                // Si response, pero procesando, try_n--, r = 1;
                // Si response y procesado, status = GET_MEASURE, r = 1;
                if ( aqualabo.response_present == 0x03 ) {
                    // Hay q verificar q están a 0xx los LSBs de las medidas q importan, las q hemos solicitado
                    // 111 measurement under way
                    // 1xx measurement impossible
                    // 0xx measurement OK
                    i = 0;
                    mask = aqualabo.measures_selected[ aqualabo.id ];
                    while( mask ) {
                        aqualabo.response[ i ] &= 0x07;
                        if ( aqualabo.response[ i ] == 0x07 ) {
                            try--;
                            if( try ){
                            	r = 1;
                            } else {
//                            	aqualabo.status = DISABLED;
                            	__disableAqualaboSensor();
                            }
                            break;
                        } else if ( aqualabo.response[ i ] >= 0x04 ) {
//                            aqualabo.status = DISABLED;
                        	__disableAqualaboSensor();
                            break;
                        }
                        i++;
                        mask >>= 1;
                    }
                    if ( mask == 0 ) {
                        aqualabo.status = GET_MEASURE;
                        r = 1;
                    }
                } else {
                	__disableAqualaboSensor();
                }
                break;
            case GET_MEASURE:
                // Si response, guardo el dato -- ya se hace en receive
                // Si NO response, NO guardo
                // status = DISABLED, r = 0;
            	__disableAqualaboSensor();
                break;
            default:
            	__disableAqualaboSensor();
                break;
        }
    }

    return r;
}

/**
 * @fn uint8_t aqualabo_modbus_send(uint8_t*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @return
 */
uint8_t aqualabo_modbus_send( uint8_t *data )
{
    uint8_t  i = 0;
    uint16_t crc;

    aqualabo.response_present = 0;
    memset( aqualabo.response, 0, sizeof( aqualabo.response ));

    switch( aqualabo.status )
    {
        case START_MEASURE://Order to run a given measurement (measurements can be run simultaneously).Modbus address:0x0001.
            data[ i++ ] = aqualabo.addr[ aqualabo.id ];              // slave dir
            data[ i++ ] = 0x10;                                      // function
            data[ i++ ] = 0x00;                                      // addr - high
            data[ i++ ] = aqualabo.reg_addr[ CMD_70 ];               // addr - low
            data[ i++ ] = 0x00;                                      // size - high
            data[ i++ ] = aqualabo.reg_size[ CMD_70 ];               // size - low
            data[ i++ ] = aqualabo.reg_size[ CMD_70 ] * 2;           // size in bytes
            data[ i++ ] = 0x00;                                      // data - high
            data[ i++ ] = aqualabo.measures_selected[ aqualabo.id ]; // data - low
            break;
        case GET_STATUS:
            data[ i++ ] = aqualabo.addr[ aqualabo.id ];              // slave dir
            data[ i++ ] = 0x03;                                      // function
            data[ i++ ] = 0x00;                                      // addr - high
            data[ i++ ] = aqualabo.reg_addr[ CMD_80 ];               // addr - low
            data[ i++ ] = 0x00;                                      // size - high
            data[ i++ ] = aqualabo.reg_size[ CMD_80 ];               // size - low
            break;
        case GET_MEASURE:
            data[ i++ ] = aqualabo.addr[ aqualabo.id ];              // slave dir
            data[ i++ ] = 0x03;                                      // function
            data[ i++ ] = 0x00;                                      // addr - high
            data[ i++ ] = aqualabo.reg_addr[ CMD_100 ];              // addr - low
            data[ i++ ] = 0x00;                                      // size - high
            data[ i++ ] = aqualabo.reg_size[ CMD_100 ];              // size - low
            break;
        default:
            break;
    }

    if ( i ) {
        crc = usMBCRC16( data, i );
        data[ i++ ] = crc;
        data[ i++ ] = crc >> 8;
    }

    return i;
}

/**
 * @fn void aqualabo_modbus_receive(uint8_t*, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @param n
 */
void aqualabo_modbus_receive( uint8_t *data, uint32_t n )
{
    uint8_t  i;
    uint16_t crc;

    if ( n > 4 ) {
        memcpy( &crc, &data[ n - 2 ], sizeof( crc ));
        if (( crc == usMBCRC16( data, n - 2 )) && ( data[ 0 ] == aqualabo.addr[ aqualabo.id ]))
        {
            if ( ( aqualabo.status == START_MEASURE ) && ( data[ 1 ] == 0x10 ) &&
                ( aqualabo.reg_addr[ CMD_70 ] == ( uint8_t ) mb_swaps( &data[ 2 ])) &&
                ( aqualabo.reg_size[ CMD_70 ] == ( uint8_t ) mb_swaps( &data[ 4 ])) ) {
                aqualabo.response_present = 0x10;
            } else if (( aqualabo.status == GET_STATUS ) && ( data[ 1 ] == 0x03 ) && ( aqualabo.reg_size[ CMD_80 ] == ( data[ 2 ] >> 1 ))) {
                aqualabo.response_present = 0x03;
                for ( i = 0; i < aqualabo.reg_size[ CMD_80 ]; i++ ) {
                    aqualabo.response[ i ] = mb_swaps( &data[ 3 + 2 * i ]);
                }
            } else if (( aqualabo.status == GET_MEASURE ) && ( data[ 1 ] == 0x03 ) && ( aqualabo.reg_size[ CMD_100 ] == ( data[ 2 ] >> 1 ))) {
                uint8_t count, mask;
                for( count = 0, mask = 0x02; mask != 0x10; mask <<= 1 ) {
                    if ( aqualabo.measures_selected[ aqualabo.id ] & mask ) {
                    	count++;
                    }
                }
                for ( i = 0; i < count; i++ ) {
                    mb_swApf( &data[ 7 + 4 * i ], &aqualabo.value[ aqualabo.offset[ aqualabo.id ] + i ]);
                }

                float rms_temp = 0.0;
                mb_swApf( &data[ 3 ], &aqualabo.temperature[ aqualabo.id ]);
                for ( i = 0, count = 0; i < LAST_AQUA_SENSOR; i++ ) {
                    if( aqualabo.temperature[ i ] != FLT_MAX ) {
                        rms_temp += aqualabo.temperature[ i ];
                        count++;
                    }
                }
                aqualabo.value[ 0 ] = rms_temp / count;
            }
        }
    }
}

/**
 * @fn uint8_t aqualabo_get_aqualabo_offset(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t aqualabo_get_aqualabo_offset( void )
{
	return aqualabo.offset[ aqualabo.id ];
}

/**
 * @fn int32_t aqualabo_modbus_get(uint8_t, float_t*)
 * @brief
 *
 * @pre
 * @post
 * @param i
 * @param value
 * @return
 */
int32_t aqualabo_modbus_get( uint8_t i, float_t * value )
{
    if ( ! aqualabo.enable ) {
        aqualabo.enable = 1;
        return 0;
    } else {
//    	return ( int ) ( aqualabo.value[ i ] * 100.0 );
    	*value = aqualabo.value[ i ];
    }
    return 1;
}

/**
  * @brief Returns a pointer to the first element of aqualabo.value[10]
  * @retval Returns a pointer to the first element of aqualabo.value[10]
  */
float_t * aqualabo_modbus_get_raw_data(void)
{
	return aqualabo.value;
}

/**
 * @fn void aqualabo_modbus_set_get_data(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _get_data
 */
void aqualabo_modbus_set_get_data( uint8_t _get_data )
{
	aqualabo.get_data = _get_data;
}

/**
 * @fn uint32_t aqualabo_modbus_enable(uint8_t)
 * @brief Enables/Disables the power rail that feeds the MC1308B step up regulator.
 *
 * @param _enable
 * 		@arg 0 - Not enabled.
 * 		@arg 1 - Enabled, Step up can be used.
 * @return  aqualabo.enable
 * 		@arg 0 - Not enabled.
 * 		@arg 1 - Enabled, Step up can be used.
 */
uint32_t aqualabo_modbus_enable(uint8_t _enable)
{
	if (1 == _enable)
	{
		ENABLE_AQUALABO_PWR_SUPPLY
	}
	else
	{
		DISABLE_AQUALABO_PWR_SUPPLY
	}

	aqualabo.enable = _enable;
	return (uint32_t)_enable;
}

/**
 * @fn uint32_t aqualabo_modbus_get_enable(void)
 * @brief Reads the status of the power rail that feeds the MC1308 step up regulator.
 *
 * @return  aqualabo.enable
 * 		@arg 0 - Not enabled.
 * 		@arg 1 - Enabled, Step up can be used.
 */
uint32_t aqualabo_modbus_get_enable( void )
{
	return aqualabo.enable;
}

/**
 * @}
 *
 */

/**
 * @}
 */
