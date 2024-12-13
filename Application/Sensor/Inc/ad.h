/**
  ******************************************************************************
  * @file           ad.h
  * @author 		Datakorum Development Team
  * @brief          Header file for ad.c
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
 * ad.h
 *
 *  Created on: 13 oct. 2019
 *      Author: Sergio Millán López
 */

/**
 * @addtogroup System_ADC
 * @{
 *
 */

#ifndef APPLICATION_SENSOR_INC_AD_H_
#define APPLICATION_SENSOR_INC_AD_H_

#include <math.h>

#include "stm32u5xx_hal.h"
#include "adc.h"
#include "params.h"
#include "battery.h"

#ifdef EXT_SENSOR
#define SAMPLES_N  (125)//(25)//

//#define AD_LSB (float) ( 3600.0 / 4096.0 )//( 3300.0 / 4096.0 )//
#define AD_LSB (float) ( battery_get_Vinst() / 65520.0 )//#define AD_LSB (float) ( 3600.0 / 65520.0 )//( 3300.0 / 4096.0 )//oversampling.//( 3300.0 / 4096.0 )//oversampling.

uint32_t AD_GetData( uint8_t sample);
void     AD_SetData( uint8_t sample, uint32_t in_data);
float_t  AD_GetAverage( void );
void     AD_SetAverage( float_t __average );
uint32_t AD_GetAverage_int( void );
uint8_t  AD_GetCalcState( void );
void     AD_SetCalcState( uint8_t );
uint8_t  AD_getEnd( void );
void     AD_setEnd( uint8_t _end_adc );
void 	 AD_SetGetADCData( uint8_t _get_adc_data );
uint8_t  AD_GetADCData( void );
void     AD_SetRegisterADCData( uint8_t _register_adc_data );
uint8_t  AD_GetRegisterADCData( void );
float_t  AD_average( uint32_t offset );
uint8_t  AD_GetADOn( void );
void     AD_SetADOn( uint8_t _ad_on );
void     AD_read_sensor_pressure_sensor(void);
void     AD_FSM( void );

#endif
#endif /* APPLICATION_SENSOR_INC_AD_H_ */

/**
 * @}
 */
