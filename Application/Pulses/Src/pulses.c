/**
  ******************************************************************************
  * @file           pulses.c
  * @author 		Datakorum Development Team
  * @brief          Low level driver to handle Pulses reading for external water
  * meter reader.
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
 * pulses.c
 *
 *  Created on: 3 dic. 2019
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pulses.h"
#include "gpio.h"
#include "params.h"
#include "tick.h"
#include "rtc.h"
#include "shutdown.h"

/**
 * @defgroup System_pulses System Pulses
 * @brief
 *
 * @{
 * */

#define F_FRAME 0
#define M_FRAME 1

#define METER_FACTOR     	(1)
#define EMITTER_FACTOR_1 	(1)
#define EMITTER_FACTOR_2 	(10)

pulses Pulse;

void pulses_init( void )
{
	switch (pulses_get_input_num()) {
	case 1:
		MX_GPIO_Enable_Pulses_CH1P(1);
		Pulse.init     = 1;
		Pulse.pulse_rx = 0;
		break;
	case 2:
		MX_GPIO_Enable_Pulses_CH1P(1);
		MX_GPIO_Enable_Pulses_CH1D(1);
		Pulse.init     = 1;
		Pulse.pulse_rx = 0;
		break;
	case 3:
		MX_GPIO_Enable_Pulses_CH1P(1);
		MX_GPIO_Enable_Pulses_CH1D(1);
		Pulse.init     = 1;
		Pulse.pulse_rx = 0;
		break;
	default:
		MX_GPIO_Enable_Pulses_CH1P(1);
		MX_GPIO_Enable_Pulses_CH1D(1);
		Pulse.init     = 1;
		Pulse.pulse_rx = 0;
		break;
	}
//	MX_GPIO_Enable_Pulses_CH2P(1);// First used.
//	MX_GPIO_Enable_Pulses_CH1D(1);
//	MX_GPIO_Enable_Pulses_CH1P(1);
////	MX_GPIO_Enable_Pulses_CH2C(1);
//	Pulse.init     = 1;
//	Pulse.pulse_rx = 0;
}

void pulses_deinit( void )
{
	MX_GPIO_Enable_Pulses_CH1P(0);
	MX_GPIO_Enable_Pulses_CH1D(0);
//	MX_GPIO_Enable_Pulses_CH2P(0);
//	MX_GPIO_Enable_Pulses_CH2C(0);
	Pulse.init     = 0;
//	Pulse.pulse_rx = 0;
	pulses_set_interruption(0);
}

uint8_t pulses_get_init( void )
{
	return Pulse.init;
}

uint32_t pulses_get_input_num( void )
{
	return Pulse.input_num;
}

void pulses_set_input_num( uint32_t _input_num )
{
	Pulse.input_num = _input_num;
}

uint32_t pulses_get_pulse_ch2p_rx( void )
{
	return Pulse.ch2p_pulse_rx;
}

uint32_t pulses_get_pulse_ch1p_rx( void )
{
	return Pulse.ch1p_pulse_rx;
}

uint32_t pulses_get_interruption( void )
{
	return Pulse.interruption;
}

void pulses_set_interruption( uint32_t _interruption )
{
	Pulse.interruption = _interruption;
}

uint32_t pulses_get_mutex( void )
{
	return Pulse.mutex;
}

void pulses_set_mutex( uint32_t _mutex )
{
	Pulse.mutex = _mutex;
}

uint8_t pulses_get_last_device( void )
{
	return Pulse.last_device;
}

void pulses_set_last_device( uint8_t _last_device )
{
	Pulse.last_device = _last_device;
}

uint8_t pulses_get_write_record( void )
{
	return Pulse.write_record;
}

void pulses_set_write_record( uint8_t _write_record )
{
	Pulse.write_record = _write_record;
}

uint32_t pulses_get_litres_forward( void )
{
	return Pulse.litres_forward;
}

uint32_t pulses_get_litres_reverse( void )
{
	return Pulse.litres_reverse;
}

uint32_t pulses_get_litres_compensated( void )
{
	return Pulse.litres_compensated;
}

uint32_t pulses_get_backflow_compensated_2p( void )
{
	return Pulse.backflow_compensated_ch2p;
}

void pulses_set_backflow_compensated_2p( uint32_t _num_pulses )
{
	Pulse.backflow_compensated_ch2p = _num_pulses;
	HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR17, Pulse.backflow_compensated_ch2p );
}

uint32_t pulses_get_ch1p( void )
{
	return Pulse.forward_ch1p;
}

void pulses_set_ch1p( uint32_t _num_pulses )
{
	Pulse.forward_ch1p = _num_pulses;
	HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR15, Pulse.forward_ch1p );
}

uint32_t pulses_get_ch1d( void )
{
	return Pulse.forward_ch1d;
}

void pulses_set_ch1d( uint32_t _num_pulses )
{
	Pulse.forward_ch1d = _num_pulses;
	HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR16, Pulse.forward_ch1d );
}

/**
 * @fn uint32_t pulses_get_pulse_rx(void)
 * @brief returns whether a pulse has been received.
 *
 * @return _pulse_on pulse received.
 * 			@arg 0 - No pulse is received.
 * 			@arg 1 - Pulse is received.
 */
uint32_t pulses_get_pulse_rx(void)
{
	uint32_t ret = 0;

	if (1 == params_pulses_on())
	{
		ret = Pulse.pulse_rx;
	}

	return ret;
}

/**
 * @fn void pulses_set_pulse_rx(uint32_t)
 * @brief Set the pulse received flag.
 * @param _pulse_on pulse received.
 * 			@arg 0 - No pulse is received.
 * 			@arg 1 - Pulse is received.
 */
void pulses_set_pulse_rx(uint32_t _pulse_on)
{
	if (0==_pulse_on)
	{
		__NOP();
	}
	Pulse.pulse_rx = _pulse_on;
}

/**
 * @fn uint32_t pulses_get_frame_type( void )
 * @brief
 *
 * @pre
 * @post
 */
uint32_t pulses_get_frame_type( void )
{
	return Pulse.frame_type;
}

/**
 * @fn void pulses_set_frame_type( uint32_t _frame_type )
 * @brief
 *
 * @pre
 * @post
 */
void pulses_set_frame_type( uint32_t _frame_type )
{
	Pulse.frame_type = _frame_type;
}

/**
 * @fn char *pulses_get_crc( void )
 * @brief
 *
 * @pre
 * @post
 */
char *pulses_get_crc( void )
{
	return Pulse.crc;
}

/**
 * @fn void pulses_set_crc( char * crc)
 * @brief
 *
 * @pre
 * @post
 */
void pulses_set_crc( char * crc)
{
	strncpy( Pulse.crc, crc, 5 );
}

/**
 * @fn void pulses_ch1p_handler(void)
 * @brief
 *
 * @pre
 * @post
 */
void pulses_ch1p_handler( void )
{
#if 0
	pulses_set_pulse_rx(1);
	Pulse.ch1p_pulse_rx = PULSE_RX_ON;
	if ( PULSE_CHECK_DIRECTION == GPIO_PIN_RESET ) {
		Pulse.reverse_ch1p = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR16);
		Pulse.reverse_ch1p++;
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR16, Pulse.reverse_ch1p );
		Pulse.direction_ch1d = REVERSE;
		Pulse.litres_reverse = params_pulses_pulse_factor() * params_pulses_pulse_unit_k_factor_out_1() * Pulse.reverse_ch1p;
//		pulses_set_write_record(1);
	} else if ( PULSE_CHECK_DIRECTION == GPIO_PIN_SET ) {
		Pulse.forward_ch1p = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR15);
		Pulse.forward_ch1p++;
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR15, Pulse.forward_ch1p );
		Pulse.direction_ch1d = FORWARD;
		Pulse.litres_forward = params_pulses_pulse_factor() * params_pulses_pulse_unit_k_factor_out_1() * Pulse.forward_ch1p;
//		pulses_set_write_record(1);
	}
#endif
//	pulses_set_pulse_rx(1);
	pulses_set_interruption(1);
	Pulse.ch1p_pulse_rx = PULSE_RX_ON;
	Pulse.forward_ch1p  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR15);
	Pulse.forward_ch1p++;
	shutdown_inc_num_readings();
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d PULSES> CH1P Int:%d\r\n",
//			(int)Tick_Get( SECONDS ), (int)Pulse.forward_ch1p);
	HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR15, Pulse.forward_ch1p );
}

/**
 * @fn void pulses_ch1d_handler(void)
 * @brief
 *
 * @pre
 * @post
 */
void pulses_ch1d_handler( void )
{
#if 0
	if ( PULSE_CHECK_DIRECTION == GPIO_PIN_RESET ) {
		Pulse.direction_ch1d = REVERSE;
	} else if ( PULSE_CHECK_DIRECTION == GPIO_PIN_SET ) {
		Pulse.direction_ch1d = FORWARD;
	}
#endif
//	pulses_set_pulse_rx(1);
	pulses_set_interruption(1);
	Pulse.ch1d_pulse_rx = PULSE_RX_ON;
	Pulse.forward_ch1d  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR16);
	Pulse.forward_ch1d++;
	shutdown_inc_num_readings();
	params_set_inc_pulses_totalizer();
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d PULSES> CH1D Int:%d\r\n",
//			(int)Tick_Get( SECONDS ), (int)Pulse.forward_ch1d);
	HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR16, Pulse.forward_ch1d );
}

/**
 * @fn void pulses_ch2p_handler(void)
 * @brief
 *
 * @pre
 * @post
 */
void pulses_ch2p_handler( void )
{
//	pulses_set_pulse_rx(1);
	pulses_set_interruption(1);
	Pulse.ch2p_pulse_rx             = PULSE_RX_ON;
	Pulse.backflow_compensated_ch2p = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR17);
	Pulse.backflow_compensated_ch2p++;
	shutdown_inc_num_readings();
	params_set_inc_pulses_totalizer();
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d PULSES> CH2P Int:%d\r\n",
//			(int)Tick_Get( SECONDS ), (int)Pulse.backflow_compensated_ch2p);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR17, Pulse.backflow_compensated_ch2p);
	Pulse.litres_compensated = params_pulses_pulse_factor() * params_pulses_pulse_unit_k_factor_out_2() * Pulse.backflow_compensated_ch2p;
	pulses_ch2c_handler();
//	pulses_set_write_record(1);
}

/**
 * @fn void pulses_ch2c_handler(void)
 * @brief
 *
 * @pre
 * @post
 */
void pulses_ch2c_handler( void )
{
	if ( CH2C_CHECK_DIRECTION == GPIO_PIN_RESET )
	{
		Pulse.compensation_ch2c = COMPENSATION_ON;
	}
	else if ( CH2C_CHECK_DIRECTION == GPIO_PIN_SET )
	{
		Pulse.compensation_ch2c = COMPENSATION_OFF;
	}
}

/**
 * @fn void pulses_tamper_handler(void)
 * @brief
 *
 * @pre
 * @post
 */
void pulses_tamper_handler( void )
{

}

/**
 * @fn void pulses_int_handler( GPIO_TypeDef *int_port, uint16_t num_pin, int_handler_func int_handler )
 * @brief
 *
 * @pre
 * @post
 */
uint32_t pulses_int_handler( GPIO_TypeDef *int_port, uint16_t num_pin, int_handler_func int_handler )
{
	static uint32_t low_edge = 0, tick_ini_pulses = 0, tick_end_pulses = 0;
	static uint32_t l = 0, h = 0, i = 0;

	i++;
	if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( int_port, num_pin ) )
	{
		l++;
		if ( 0 == low_edge )
		{
			low_edge             = 1;
			tick_ini_pulses      = HAL_GetTick();
		}
		else if ( ( HAL_GetTick() > tick_ini_pulses ) && ( ( HAL_GetTick() - tick_ini_pulses ) > 1 ) )
		{
			tick_ini_pulses      = HAL_GetTick();
		}
	}
	else if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( int_port, num_pin ) )
	{
		tick_end_pulses = HAL_GetTick();
		h++;
		if ( ( low_edge == 1 ) && ( tick_end_pulses > tick_ini_pulses ) && ( ( tick_end_pulses - tick_ini_pulses ) >= 1 ) )
		{
			i = 0;
			h = 0;
			l = 0;
			low_edge        = 0;
			tick_ini_pulses = tick_end_pulses;
			int_handler();
		}
	}
	return 0;
}

/**
 * @fn void pulses_int_handler( GPIO_TypeDef *int_port, uint16_t num_pin, int_handler_func int_handler )
 * @brief
 *
 * @pre
 * @post
 */
uint32_t pulses_int_2_handler( GPIO_TypeDef *int_port, uint16_t num_pin, int_handler_func int_handler )
{
	static uint32_t low_edge_2 = 0, tick_ini_pulses_2 = 0, tick_end_pulses_2 = 0;
	static uint32_t l_2 = 0, h_2 = 0, i_2 = 0;

	i_2++;
	if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( int_port, num_pin ) )
	{
		l_2++;
		if ( 0 == low_edge_2 )
		{
			low_edge_2             = 1;
			tick_ini_pulses_2      = HAL_GetTick();
		}
		else if ( ( HAL_GetTick() > tick_ini_pulses_2 ) && ( ( HAL_GetTick() - tick_ini_pulses_2 ) > 1 ) )
		{
			tick_ini_pulses_2      = HAL_GetTick();
		}
	}
	else if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( int_port, num_pin ) )
	{
		tick_end_pulses_2 = HAL_GetTick();
		h_2++;
		if ( ( low_edge_2 == 1 ) && ( tick_end_pulses_2 > tick_ini_pulses_2 ) && ( ( tick_end_pulses_2 - tick_ini_pulses_2 ) >= 1 ) )
		{
			i_2 = 0;
			h_2 = 0;
			l_2 = 0;
			low_edge_2        = 0;
			tick_ini_pulses_2 = tick_end_pulses_2;
			int_handler();
		}
	}
	return 0;
}

/**
 * @fn int __cz2_append_crc_frame(char[])
 * @brief
 *
 * @pre
 * @post
 * @param read_frame
 * @return
 */
static int __cz2_append_crc_frame( char read_frame[] )
{
	static unsigned short i, data;
	static unsigned int pos;
	static unsigned short accum, genpoly = 0x1021;

	accum = 0;
	pos   = 1;

	uint32_t len_frame = strlen(read_frame);

	if ( len_frame > 0 ) {
		while( pos != len_frame ) {
			data = read_frame[pos++] << 8;
			for ( i = 8; i > 0; i-- ) {
				if ( ( data ^ accum ) & 0x8000 ) {
					accum = ( accum << 1 ) ^ genpoly;
				} else {
					accum <<= 1;
				}
				data <<= 1;
			}
		}

	} else {
		accum = 1;
	}

	return accum;
}

/**
 * @fn void pulses_frame_append_checksum(char*, char*)
 * @brief
 *
 * @pre
 * @post
 * @param data
 * @param crc_field
 */
void pulses_frame_append_checksum( char *data, char *crc_field )
{
	unsigned short accum;
	uint32_t val;

	accum = __cz2_append_crc_frame( data );
	val   = ( accum & 0xF000 ) >> 12 | 0x30;
	crc_field[0] = val;
	val   = ( accum & 0x0F00 ) >> 8  | 0x30;
	crc_field[1] = val;
	val   = ( accum & 0x00F0 ) >> 4  | 0x30;
	crc_field[2] = val;
	val   = ( accum & 0x000F ) >> 0  | 0x30;
	crc_field[3] = val;
	crc_field[4] = '\0';
}

char crc_calc_pulses[512]; /*!< */

/**
 * @fn void pulses_build_typeM_telegram(char*, uint32_t*)
 * @brief
 *
 * @pre
 * @post
 * @param telegram_1
 * @param num_dig
 */
void pulses_build_typeM_telegram(char *telegram_1, uint32_t *num_dig)
{
	char litres_forward[15],litres_forward_2[15],litres_forward_3[15];
	itoa(Pulse.backflow_compensated_ch2p, litres_forward, 10);
	itoa(Pulse.ch1p_pulse_rx, litres_forward_2, 10);
	itoa(Pulse.ch1d_pulse_rx, litres_forward_3, 10);
	char crc[5];
	uint32_t len = strlen(litres_forward);
	uint32_t len_2 = strlen(litres_forward_2);
	uint32_t len_3 = strlen(litres_forward_3);

	memset(crc_calc_pulses, 0, 512);

//	rtc_system_SetCreatedMeterValueTime(Tick_Get(SECONDS));
//	rtc_system_SetCreatedMeterValueDate(Tick_Get(SECONDS));

	*num_dig = len + len_2 + len_3;
//	memcpy(&telegram_1[0], rtc_system_getCreatedMeterValueTime(), 5);
//	telegram_1[4] = '0';
	memcpy(&telegram_1[0], litres_forward, len);
	memcpy(&telegram_1[0 + len_2], litres_forward_2, len_2);
	memcpy(&telegram_1[len_2 + len_3], litres_forward_3, len_3);
	memcpy(crc_calc_pulses, telegram_1, strlen(telegram_1));

	pulses_frame_append_checksum(crc_calc_pulses, crc);

    if (Pulse.crc[0] == '\0')
    {
    	pulses_set_frame_type(F_FRAME);
    }
    else
    {
    	pulses_set_frame_type(M_FRAME);
    }

    pulses_set_crc(crc);
}

/**
 * @}
 *
 */ //End defgroup System pulses
