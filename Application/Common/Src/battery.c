/*
 * battery.c
 *
 *  Created on: 29 ene. 2021
 *      Author: smill
 */
#include <stdio.h>
#include <math.h>
#include "battery.h"
#include "tick.h"
#include "adc.h"
#include "gpio.h"
#include "params.h"

#define VREFINT_CAL 		   ((uint16_t*) VREFINT_CAL_ADDR)
#define DECIMAL(a)             ((int)((fabsf((a-(float)((int)a))))*1000))

typedef struct _battery {
	uint32_t Vdda;
	uint32_t Vacc;
	float_t  Vinst;
	uint32_t num_meas;
}Battery_st;

Battery_st Battery = {
		.Vdda     = 3600,
		.Vacc     = 0,
		.Vinst    = 3600,
		.num_meas = 0,
};

void battery_reset_num_meas( void )
{
	Battery.num_meas = 0;
	Battery.Vacc     = 0;
}

uint32_t battery_get_Vdda( void )
{
	Battery.Vdda = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR21);
	return Battery.Vdda;
}

float_t battery_get_Vinst( void )
{
	return Battery.Vinst;
}

void battery_set_Vdda( uint32_t _Vdda )
{
	Battery.Vdda = _Vdda;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR21, Battery.Vdda);
}

uint32_t battery_get_Vbat(void)
{

        volatile uint32_t Vadc = 0;
//        float Vadc_norm = 0.0;
    	uint32_t timeout_ms = 100;
    	__IO uint32_t wait_loop_index = 0UL;

		if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
		{
			/* Calibration Error */
			Error_Handler();
		}
	    /* Delay between ADC end of calibration and ADC enable.                   */
	    /* Note: Variable divided by 2 to compensate partially                    */
	    /*       CPU processing cycles (depends on compilation optimization).     */
	    wait_loop_index = ((LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32) >> 1);
	    while(wait_loop_index != 0)
	    {
	      wait_loop_index--;
	    }
		if (HAL_ADC_Start(&hadc1) != HAL_OK)
		{
			/* Start Conversation Error */
			Error_Handler();
		}
		if (HAL_ADC_PollForConversion(&hadc1, timeout_ms) != HAL_OK)
		{
			/* End Of Conversion flag not set on time */
			Error_Handler();
		}
        Vadc = HAL_ADC_GetValue(&hadc1);
//        float factor = (float)*VREFINT_CAL/(float)Vadc;
//        volatile uint32_t Vdda = 3000UL * (float)*VREFINT_CAL/(float)Vadc; /* ref. manual p. 252; constant and result in millivolts */

        /* Computation of ADC conversions raw data to physical values           */
        /* using helper macro.                                                  */
        __IO uint32_t Vdda = __LL_ADC_CALC_VREFANALOG_VOLTAGE(ADC1, Vadc,  LL_ADC_RESOLUTION_14B);//__LL_ADC_CALC_DATA_TO_VOLTAGE(ADC1, ((*VREFINT_CAL)*VREFINT_CAL_VREF), Vadc, LL_ADC_RESOLUTION_14B);//
//        Vdda += 70;
//        DBG_PRINT("Vbat_inst: %d mV", (int)Vdda);
//        Vadc = HAL_ADC_GetValue(&hadc1);
//        Vadc_norm = (float)( 3600.0 / 4096.0 ) * (float)Vadc;
//        float vrefint = (float)*VREFINT_CAL;
//        volatile uint32_t Vdda = 3000UL * (float)*VREFINT_CAL/(float)Vadc; /* ref. manual p. 252; constant and result in millivolts */
//        volatile float Vdda_norm = 3000UL * vrefint/Vadc_norm;
//        printf("Vbat: %d mV\n", (int)Vdda);
//        printf("Vbat: %d.%03d mV\n", (int)Vdda_norm, DECIMAL(Vdda_norm));

        HAL_ADC_Stop(&hadc1);
//        battery_set_Vdda(Vdda);

        return Vdda;
}

void battery_voltage_meas( uint32_t no_printf )
{
	uint32_t Vinst = 0;
//	static uint32_t log_count = 0;

//	MX_ADC1_DeInit();
	MX_ADC1_DeInit();
	MX_ADC1_Init();
	Battery.num_meas++;
	Vinst = battery_get_Vbat();
	Battery.Vinst = (float_t)Vinst;
	battery_get_Vdda();
	if (Vinst < Battery.Vdda)
	{
		battery_set_Vdda(Vinst);
	}
	Battery.Vacc += Vinst;
	MX_ADC1_DeInit();

	if ( 0 == no_printf )
	{
//		if (2 == log_count++)
		{
//			log_count = 0;
			LOGLIVE(LEVEL_2, "\r\nLOGLIVE> %d BATTERY> Vbat_inst: %dmV; Vbat_min: %dmV; Vacc: %dmV; Num Meas: %d\r\n",
					(int)Tick_Get( SECONDS ),
					(int)battery_get_Vinst(), (int)Battery.Vdda,
					(int)(Battery.Vacc/Battery.num_meas), (int)Battery.num_meas);
		}
	}
}
