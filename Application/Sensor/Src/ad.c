/**
  ******************************************************************************
  * @file           ad.c
  * @author 		Datakorum Development Team
  * @brief          Low level driver to handle ADC for external analog sensor.
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
 * ad.c
 *
 *  Created on: 13 oct. 2019
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ad.h"
#include "gpio.h"
#include "tick.h"
#include "params.h"
#include "pulses.h"
#include "shutdown.h"
#include "leds.h"
#include "battery.h"

#ifdef EXT_SENSOR

/**
 * @defgroup System_ADC System ADC
 * @{
 * */

/**
 * @def ENABLE_SENSOR
 * @brief Enables the external sensor power supply rail.
 */
#define ENABLE_SENSOR() 	MX_GPIO_Enable_Sensor()
/**
 * @def DISABLE_SENSOR
 * @brief Disables the external sensor power supply rail.
 *
 */
#define DISABLE_SENSOR()	MX_GPIO_Disable_Sensor()


uint8_t calc_FLAG         = 1;
uint8_t event_Poll        = 0;
uint8_t end_adc           = 0;
uint8_t get_adc_data      = 0;
uint8_t register_adc_data = 0;
uint8_t adc_on            = 0;
uint32_t ad_status        = 0;

extern uint32_t end_of_adc_conversion;

/**
 * @struct
 * @brief ADC sampling data structure
 *
 */
static struct
{
	int32_t  ch1[SAMPLES_N];	/*!< Array to store ADC data int32*/
	float_t  average;			/*!< Average of ADC data float*/
} data;

#define OFFSET_MEASURES (100)//(5)//10//

/**
 * @fn uint32_t AD_GetData(uint8_t)
 * @brief
 *
 * @param sample
 * @return
 */
uint32_t AD_GetData( uint8_t sample)
{
    return data.ch1[sample];
}

/**
 * @fn void AD_SetData(uint8_t, uint32_t)
 * @brief
 *
 * @param sample
 * @param in_data
 */
void AD_SetData(uint8_t sample, uint32_t in_data)
{
    data.ch1[sample] = in_data;
}

/**
  * @brief Returns the ADC data average stored data.average member.
  * @pre Data structure needs to be updated first
  *
  * @retval Returns the ADC data average stored in data.average member.
  */
float_t AD_GetAverage( void )
{
    return data.average;
}

/**
 * @fn void AD_SetAverage( float_t __average )
 * @brief
 *
 * @param state
 */
void AD_SetAverage( float_t __average )
{
	data.average = __average;
}

/**
  * @brief Returns the ADC data average stored data.average member.
  * @pre Data structure needs to be updated first
  *
  * @retval Returns the ADC data average stored in data.average member.
  */
uint32_t AD_GetAverage_int( void )
{
    return (data.average);
}

/**
 * @fn uint8_t AD_GetCalcState(void)
 * @brief
 *
 * @return
 */
uint8_t AD_GetCalcState( void )
{
    return calc_FLAG;
}

/**
 * @fn void AD_SetCalcState(uint8_t)
 * @brief
 *
 * @param state
 */
void AD_SetCalcState(uint8_t state)
{
    calc_FLAG = state;
}

/**
  * @brief  Reads the status of the ADC process.
  * @retval end_adc
  * 		@arg 0 - Not finished.
  * 		@arg 1 - Finished.
  */
uint8_t AD_getEnd( void )
{
	return end_adc;
}

/**
  * @brief  Status of the ADC process.
  * @param	_end_adc: set the current status
  * 		@arg 0 - Not finished.
  * 		@arg 1 - Finished.
  */
void AD_setEnd( uint8_t _end_adc )
{
	end_adc = _end_adc;
}

/**
 * @fn void AD_SetGetADCData(uint8_t)
 * @brief Writes the semaphore to start or stop the ADC sampling process.
 *
 * @param _get_adc_data
 * 			@arg 0 - Do not start the ADC conversion.
 * 			@arg 1 - Start the ADC conversion.
 */
void AD_SetGetADCData( uint8_t _get_adc_data )
{
	if ( 1 == _get_adc_data )
	{
		ad_status = 0;
	}

	get_adc_data = _get_adc_data;
}

/**
 * @fn uint8_t AD_GetADCData(void)
 * @brief Reads the semaphore to start or stop the ADC sampling process.
 *
 * @return get_adc_data
 * 			@arg 0 - Do not start the ADC conversion.
 * 			@arg 1 - Start the ADC conversion.
 */
uint8_t AD_GetADCData( void )
{
	return get_adc_data;
}

/**
 * @fn void AD_SetRegisterADCData(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param _register_adc_data
 */
void AD_SetRegisterADCData( uint8_t _register_adc_data )
{
	register_adc_data = _register_adc_data;
}

/**
 * @fn uint8_t AD_GetRegisterADCData(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t AD_GetRegisterADCData( void )
{
	return register_adc_data;
}

/**
 * @fn void AD_SetADOn(uint8_t)
 * @brief Sets the ADC State flag.
 * @param _ad_on
 * 		@arg 0 - ADC is Disabled.
 * 		@arg 1 - ADC is Enabled.
 */
void AD_SetADOn(uint8_t _ad_on)
{
	adc_on = _ad_on;
	params_config_set_adc_on(adc_on);
}

/**
 * @fn uint8_t AD_GetADOn(void)
 * @brief Reads the ADC State flag
 * @return _ad_on
 * 		@arg 0 - ADC is Disabled.
 * 		@arg 1 - ADC is Enabled.
 */
uint8_t AD_GetADOn(void)
{
	adc_on = params_config_get_adc_on();
	return adc_on;
}

/**
 * @fn
 * @brief
 * @return
 */
__STATIC_INLINE uint32_t __read_pressure_sensor(void)
{
	if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
#if 0
	if (HAL_ADC_Start(&hadc4) != HAL_OK)
	{
		/* Start Conversation Error */
		Error_Handler();
	}

	if (HAL_ADC_PollForConversion(&hadc4, 200) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		Error_Handler();
	}
#endif
	/** Start conversion in DMA mode. */
	if (HAL_ADC_Start_DMA(&hadc4,
			(uint32_t *)data.ch1,
			SAMPLES_N
	) != HAL_OK)
	{
		Error_Handler();
	}

	uint32_t tickstart = HAL_GetTick();
	do
	{

	}while (( 0 == end_of_adc_conversion ) && ((HAL_GetTick() - tickstart) < 5000));

	if ( 1 == end_of_adc_conversion )
	{
		end_of_adc_conversion = 0;
		/** Calculates the average value. */
		data.average = AD_average(OFFSET_MEASURES);
	}
	return data.average;
}

void AD_read_sensor_pressure_sensor(void)
{
	battery_voltage_meas(1);

	ENABLE_SENSOR();

	HAL_Delay(100);

	MX_ADC4_DeInit();
	MX_ADC4_Init();
	float_t average = (float_t)__read_pressure_sensor();
	average = (float_t)__LL_ADC_CALC_DATA_TO_VOLTAGE(ADC4, battery_get_Vinst(), average,  LL_ADC_RESOLUTION_12B);
	UNUSED(average);
//	data.average = AD_LSB*average;
	DISABLE_SENSOR();
	MX_ADC4_DeInit();
}

/**
 * @fn void AD_FSM(void)
 * @brief Analog to Digital converter sampling process finite state machine.
 *
 * @see http://hung2492.blogspot.com.es/2015/02/lesson-5-adcdac-stm32f0.html
 *
 * @todo Create FSM enumeration for the different AD states of the FSM.
 *
 */
void AD_FSM( void )
{
	static uint32_t tick_ini = 0, tick_diff = 0;
//TOASK: Por qué no se usa interrupciones o DMA? esta función es llamada cada 1ms.
#define  CHECK_TAMPER	(HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ))
	switch(ad_status)
	{
	case 0: // Init.
		if ((1 == AD_GetADCData())
		 && (0 == pulses_get_pulse_rx())
		 &&   (params_input_pulse_as_sensor_get_num() != 1)
		   && (params_input_pulse_as_sensor_get_num() != 2)
		   && (params_input_pulse_as_sensor_get_num() != 3)
		   && (params_input_pulse_as_sensor_get_num() != 4)
		   && (params_get_i2c_sensor()                == 0)
		   )
		{
			/** Enables ADC power rail. */
			Tick_update_tick(TICK_AD);
			battery_voltage_meas(1);
			ENABLE_SENSOR();
			Tick_update_tick(TICK_AD);
			ad_status++;
		}
	break;

	case 1:
		if (CHECK_ELAPSED_MILISEC(10, TICK_AD))
		{
			if (CHECK_TAMPER == 0)
			{
				leds_LED_On(LED_GREEN);
			}
			/** Initializes ADC peripheral. */
			MX_ADC4_Init();
			tick_ini = Tick_Get( MILLISECONDS );
			/** Run the ADC calibration in single-ended mode. */
			if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
			{
				/* Calibration Error */
				Error_Handler();
			}
			/** Start conversion in DMA mode. */
			if (HAL_ADC_Start_DMA(&hadc4,
					(uint32_t *)data.ch1,
					SAMPLES_N
			) != HAL_OK)
			{
				Error_Handler();
			}
			ad_status=3;
			Tick_update_tick(TICK_AD);
		}
		break;

		case 2:// Start conversion.
			/** Waits for the end of conversion in polling mode. */
			/*  For simplicity reasons, this example is just waiting till the end of the
			      conversion, but application may perform other tasks while conversion
			      operation is ongoing. */
			if (CHECK_ELAPSED_MILISEC(2, TICK_AD))
			{
				ad_status++;
			}
		break;

		case 3:
			if ( ( 1 == end_of_adc_conversion ) || (CHECK_ELAPSED_MILISEC(500, TICK_AD)) ) {
				end_of_adc_conversion = 0;
				/** Calculates the average value. */
				data.average = AD_average(OFFSET_MEASURES);
				AD_setEnd(1);
				AD_SetGetADCData(0);
				/** Disables ADC power rail. */
				DISABLE_SENSOR();
				leds_LED_Off(LED_GREEN);
				/** De-initializes the ADC peripheral */
				MX_ADC4_DeInit();
				tick_diff = Tick_Get( MILLISECONDS ) - tick_ini;
				(void)tick_diff;
				ad_status++;
			}
			if (CHECK_ELAPSED_MILISEC(500, TICK_AD))
			{
				__NOP();
			}
			break;

		case 4:
			if (1 == AD_GetADCData())
			{
				ad_status = 0;
			}
		break;

		default:
			ad_status = 0;
			break;
	}
}

/**
 * @fn float_t AD_average(uint32_t)
 * @brief Calculates the average value of the SAMPLES_N samples from the ADC.
 *
 * @pre static data structure needs to be previously filled with SAMPLES_N
 * values from the adc using ADC_SetData.

 * @param offset
 * @return float_t average value
 */
float_t AD_average(uint32_t offset)
{
	float_t av = 0;

	for(uint32_t j = offset; j < SAMPLES_N; j++)
	{
	    av += data.ch1[j];
	}
	av = av / (SAMPLES_N - offset);

	return (AD_LSB * av);
}

/**
 * @fn uint32_t AD_rms()
 * @brief Calculates the rms value of the SAMPLES_N samples from the ADC.
 *
 * @pre static data structure needs to be previously filled with SAMPLES_N
 * values from the adc using ADC_SetData.
 *
 * @return uint32_t rms value.
 */
uint32_t AD_rms()
{
	uint32_t av = 0, rms = 0;

	for(uint32_t j = 0; j < SAMPLES_N; j++)
	{
	    av += data.ch1[j];
	}
	av = av / SAMPLES_N;
	for(uint32_t j = 0; j < SAMPLES_N; j++)
	{
	    data.ch1[j] -= av;
	}
	for(uint32_t j = 0; j < SAMPLES_N; j++)
	{
	    rms += data.ch1[j] * data.ch1[j];
	}
	rms = rms / SAMPLES_N;
	rms = AD_LSB * sqrt(rms);

	return rms;
}
#endif

/**
 * @}
 * */ //End group System_ADC
