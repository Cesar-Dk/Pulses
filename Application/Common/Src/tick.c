/**
  ******************************************************************************
  * @file           tick.c
  * @author 		Datakorum Development Team
  * @brief          Low level time driver to handle tick time operations
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
 * tick.c
 *
 *  Created on: 15 jul. 2019
 *      Author: smill
 */

#include <stdio.h>
#include "tick.h"
#include "rtc_system.h"
#include "comm_serial.h"
#include "leds.h"
#include "ad.h"
#include "pulses.h"


/**
 * @defgroup System_tick System tick
 * @brief Tick API for time elapse and time measurement.
 *
 * @{
 */

static uint32_t t_ms, t_seconds;
static uint32_t tick_in_seconds_init;
static uint32_t timerflag_1s, timerflag_1ms, timerflag_10ms, timerflag_100ms;

/**
 * @fn uint32_t Tick_Second(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t Tick_Second(void)
{
	uint32_t flag = timerflag_1s;

	if (1 == flag)
	{
		timerflag_1s = 0;
	}
	return flag;
}

/**
 * @fn uint32_t Tick_Force_1Second_Task(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t Tick_Force_1Second_Task(void)
{
	timerflag_1s = 1;
	return 0;
}

/**
 * @fn uint32_t Tick_100Miliseconds(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t Tick_100Miliseconds(void)
{
	uint32_t flag = timerflag_100ms;

	if ( 1 == flag )
	{
		timerflag_100ms = 0;
	}
	return flag;
}

/**
 * @fn uint32_t Tick_10Miliseconds(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t Tick_10Miliseconds(void)
{
	uint32_t flag = timerflag_10ms;

	if ( 1 == flag )
	{
		timerflag_10ms = 0;
	}
	return flag;
}

/**
 * @fn uint32_t Tick_1Miliseconds(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t Tick_1Miliseconds(void)
{
	uint32_t flag = timerflag_1ms;

	if (1 == flag)
	{
		timerflag_1ms = 0;
	}
	return flag;
}

/**
 * @fn uint32_t Tick_get_tick_init(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t Tick_get_tick_init(void)
{
	return tick_in_seconds_init;
}

/**
 * @fn void Tick_init(void)
 * @brief
 *
 * @pre
 * @post
 */
void Tick_init(void)
{
	static uint32_t ticks = 0;
//	uint32_t secs = 0;
	t_seconds  = 0;
	tick_in_seconds_init = HAL_GetTick()/1000;
	Tick_system_time_init(rtc_system_GetServerTime());
	t_ms       = 0;
	if (1 == Tick_cloud_time_init())
	{
		if (1 == Tick_cloud_time_init())
		{
			if (0 == pulses_get_pulse_rx())//(0 == params_pulses_on())
			{
				if ( 1 == AD_GetADOn() )
				{
					if (ticks++ >= 1)
					{
//						ticks = 0;
//						LOGLIVE(LEVEL_1, "LOGLIVE> %d TICK> Time:%d Time adj:%d\r\n", (int)Tick_Get( SECONDS ), (int)Tick_Get( SECONDS ),  (int)Tick_Get( SECONDS ) + 1);
//						Tick_system_time_init(Tick_Get(SECONDS) + 1);
//						secs  = Tick_Get(SECONDS);
//						rtc_system_SetServerTime(secs);
					}
				}
				else
				{
//					Tick_system_time_init(Tick_Get(SECONDS) - 15);
//					rtc_system_SetServerTime(Tick_Get(SECONDS));//??????????
				}
			}
		}
//		rtc_system_SetServerTime(Tick_Get(SECONDS));//??????????
		rtc_System_InitRemainingTime();
	}
}

/**
  * @brief Returns the internal system time.
  * @param @ref time_unit Possible options:
  * 		@arg MILLISENCONDS -> returns t_ms
  * 		@arg SECONDS -> returns t_seconds
  * @retval Internal system time.
  */
uint32_t Tick_Get(time_unit time_unit)
{
	uint32_t ret = 0;

	if (time_unit == MILLISECONDS)
	{
		ret = t_ms;
	}
	else if (time_unit == SECONDS)
	{
		ret = t_seconds;
	}
	return ret;
}

/**
 * @fn void Tick_system_time_init(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param s
 */
void Tick_system_time_init(uint32_t s)
{
	t_seconds = s;
}

/**
 * @fn uint32_t Tick_delay_ms(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param ms
 * @return
 */
uint32_t Tick_delay_ms(uint32_t ms)
{
	uint32_t i = Tick_Get(MILLISECONDS);

	while((Tick_Get(MILLISECONDS) - i) < ms)
	{};
	return Tick_Get(MILLISECONDS);
}

/**
 * @fn void HAL_SYSTICK_Callback(void)
 * @brief
 *
 * @pre
 * @post
 */
void HAL_SYSTICK_Callback(void)
{
	static uint32_t T_10ms, T_100ms, T_seconds;

	t_ms++;
	timerflag_1ms = 1;

	if(T_10ms >= 9)
	{
		T_10ms = 0;
		timerflag_10ms = 1;
	}
	else
	{
		T_10ms++;
	}

	if(T_100ms >= 99)
	{
		T_100ms = 0;
		timerflag_100ms = 1;
	}
	else
	{
		T_100ms++;
	}

	if (T_seconds >= 999)
	{
		T_seconds    = 0;
		timerflag_1s = 1;
		t_seconds++;
	}
	else
	{
		T_seconds++;
	}
#ifdef EXT_SENSOR
	AD_FSM();
#endif
}

static uint32_t tick[ TICK_LAST ] = { 0 };

/**
 * @fn void Tick_update_tick(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param i
 */
void Tick_update_tick( uint8_t i )
{
	tick[ i ] = HAL_GetTick();
}

/**
 * @fn uint32_t Tick_get_tick(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param i
 * @return
 */
uint32_t Tick_get_tick( uint8_t i )
{
	return tick[ i ];
}


/**
 * @fn uint8_t Tick_cloud_time_init(void)
 * @brief Checks whether the time has been updated from the network or not.
 *
 * @retval @arg 0 - Not initialized.
 * 		   @arg 1 - Initialized.
 */
uint8_t Tick_cloud_time_init( void )
{
	if( t_seconds > 1533278897 )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @fn time_t Tick_get_date(char*)
 * @brief
 *
 * @param date
 * @return
 *
 * @see www.epochconverter.com
 * @link www.epochconverter.com {epoch}
 *
 * @todo Check date integrity.
 */
time_t Tick_get_date(char *date)
{
    struct tm fecha = {0};
    char *str;
    //@see www.epochconverter.com

    str = strtok(date, "/,:");
    fecha.tm_year = atoi(str) + 2000 - 1900;
    str = strtok(NULL, "/,:");
    fecha.tm_mon = atoi(str) - 1;
    str = strtok(NULL, "/,:");
    fecha.tm_mday = atoi(str);

    str = strtok(NULL, "/,:");
    fecha.tm_hour = atoi(str);
    str = strtok(NULL, "/,:");
    fecha.tm_min = atoi(str);

//    rtc_localUTCTime((uint32_t *)&fecha.tm_hour); // TODO: Set UTC Time in Telit module and calculate local time.

    return mktime(&fecha);
}

/**
 *
 * @}
 */
