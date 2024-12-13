/**
  ******************************************************************************
  * @file           rtc_system.c
  * @author 		Datakorum Development Team
  * @brief          Driver to handle RTC system and Alarms
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
 * rtc_system.c
 *
 *  Created on: 15 jul. 2019
 *      Author: smill
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "rtc_system.h"
#include "pulses.h"
#include "ad.h"
#include "shutdown.h"
#include "generic_modbus.h"
#include "json.h"
#include "dlms_client.h"
#include "sensor_log.h"
#include "generic_sensor.h"
#include "generic_sensor_log.h"

/**
 * @addtogroup System_RTC_System System RTC
 * @brief
 * @{
 *  */

/* Private typedef -----------------------------------------------------------*/
#define STAND_BY_TIME (30)//(29)//(28)//(26)
#define STAND_BY_TIME_MS (1 == params_config_get_period())?(STAND_BY_TIME*1000):(rtc_system_getReadSensorAlarmCycleNextInSeconds()*1000)

#define READ_WINDOWS (8)
#define SEND_WINDOWS (8)

/**
 * @struct
 * @brief
 *
 */
typedef struct
{
	uint8_t          alarmOccured;
	uint8_t          alarmState;
	uint8_t          alarmNext;
	uint8_t          alarmsNum;
	uint8_t          initTimeServer;
	uint8_t          waitingTimeServer;
	uint8_t          programmedReset;
	uint8_t          getTime;
	uint8_t          timerOccured;
	uint8_t          timerState;
}rtc;

/**
 * @struct
 * @brief
 *
 */
typedef struct {
	uint8_t         sendTime;
	uint8_t         endTime;
	uint16_t        cycle;
}rtc_readAlarms;

/**
 * @struct
 * @brief
 *
 */
typedef struct {
	uint8_t         sendTime;
}rtc_sendAlarms;

/** Structure that holds all system alarms to wake the system from SLEEP MODE*/
/**
 * @struct
 * @brief
 *
 */
typedef struct
{
	uint8_t        		  sendAlarmsNum;				/*!< */
	uint8_t        		  sendAlarmNext;				/*!< */
	uint8_t        		  sendParamsAlarmsNum;			/*!< */
	uint8_t        		  sendParamsAlarmNext;			/*!< */
	uint8_t        		  sendSensorAlarmsNum;			/*!< */
	uint8_t        		  sendSensorAlarmNext;			/*!< */
	uint8_t        		  sendModbusAlarmsNum;			/*!< */
	uint8_t        		  sendModbusAlarmNext;			/*!< */
	rtc_sendAlarms 		  sendAlarms[SEND_WINDOWS];		/*!< */
	rtc_sendAlarms 		  sendParamsAlarms[SEND_WINDOWS];	/*!< */
	rtc_sendAlarms 		  sendSensorAlarms[SEND_WINDOWS];	/*!< */
	rtc_sendAlarms 		  sendModbusAlarms[SEND_WINDOWS];	/*!< */
	uint8_t        		  readAlarmsNum;				/*!< */
	uint8_t        		  readInitAlarmNext;			/*!< */
	uint8_t        		  readEndAlarmNext;				/*!< */
	uint8_t        		  readCycleNext;				/*!< */
	rtc_readAlarms 		  readAlarms[READ_WINDOWS];		/*!< */
	uint8_t        		  readSensorAlarmsNum;			/*!< */
	uint8_t        		  readSensorInitAlarmNext;		/*!< */
	uint8_t        		  readSensorEndAlarmNext;		/*!< */
	uint16_t       		  readSensorCycleNext;			/*!< */
	rtc_readAlarms 		  readSensorAlarms[READ_WINDOWS];	/*!< */
	uint8_t        		  readModbusAlarmsNum;			/*!< */
	uint8_t        		  readModbusInitAlarmNext;		/*!< */
	uint8_t        		  readModbusEndAlarmNext;		/*!< */
	uint16_t       		  readModbusCycleNext;			/*!< */
	rtc_readAlarms 		  readModbusAlarms[READ_WINDOWS];	/*!< */
	current_params_alarm  currentParamsAlarm;			/*!< */
	current_alarm         currentAlarm;					/*!< */
	current_sensor_alarm  currentSensorAlarm;			/*!< */
	current_modbus_alarm  currentModbusAlarm;			/*!< */
}rtc_Alarm;

uint32_t calib_period = 6, odd_time = 0, billing_day = 0, event_log_hour = 0, load_profile_hour = 0;
int32_t time_btw_pulses;
time_t  time_in_miliseconds_before_low_power = 0, time_in_miliseconds_after_low_power = 0;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WAKEUP_TIMER_ENABLE 0x32F2

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//RTC_HandleTypeDef RtcHandle;
TIM_HandleTypeDef Input_Handle;

uint16_t tmpCCTIM_CHANNEL_1[2] = {0, 0};
__IO uint32_t uwLsiFreq = 0;

__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;

uint32_t rtc_refresh_time = 0;
int32_t  res_cycle_time = 0;
int32_t  res_cycle_time_prev = 0;

rtc       rtc_Calendar;
rtc_Alarm rtc_Alarms;

/* Private function prototypes -----------------------------------------------*/
static uint32_t __getLSEFrequency(void);
static void     __next_date(uint8_t *year, uint8_t *month, uint8_t *day);

extern RTC_HandleTypeDef hrtc;
/* Private functions ---------------------------------------------------------*/
/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
	if (TIM16 == htim->Instance) {
		/* TIM16 Peripheral clock enable */
		__HAL_RCC_TIM16_CLK_ENABLE();

		/* Configure the NVIC for TIM16 */
		HAL_NVIC_SetPriority(TIM16_IRQn,0,0);

		/* Enable the TIM16 global Interrupt */
		HAL_NVIC_EnableIRQ(TIM16_IRQn);
	}
}

/**
  * @brief RTC MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
	if (TIM16 == htim->Instance) {
		/* Enable the TIM16 global Interrupt */
		HAL_NVIC_DisableIRQ(TIM16_IRQn);

		/* TIM16 Peripheral clock disable */
		__HAL_RCC_TIM16_CLK_DISABLE();
	}
}

static void __initAlarmsStruct(void)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {
    	rtc_Alarms.readAlarms[i].sendTime       = 0xFF;
    	rtc_Alarms.readAlarms[i].endTime        = 0xFF;
    	rtc_Alarms.readAlarms[i].cycle          = 0xFF;
    	rtc_Alarms.readSensorAlarms[i].sendTime = 0xFF;
    	rtc_Alarms.readSensorAlarms[i].endTime  = 0xFF;
    	rtc_Alarms.readSensorAlarms[i].cycle    = 0xFF;
	}

	for (i = 0; i < 8; i++) {
		rtc_Alarms.sendAlarms[i].sendTime       = 0xFF;
		rtc_Alarms.sendSensorAlarms[i].sendTime = 0xFF;
	}

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.readAlarms[i].sendTime = params_read_time_init_time(i);
    	rtc_Alarms.readAlarms[i].endTime  = params_read_time_end_time(i);
    	rtc_Alarms.readAlarms[i].cycle    = params_read_time_cycle(i);
    	if ( 0xFF == rtc_Alarms.readAlarms[i].sendTime ) {
    		rtc_Alarms.readAlarmsNum = i;
    		i = 8;
    	}
    }

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.sendAlarms[i].sendTime = params_send_time_send_time(i);
    	if ( 0xFF == rtc_Alarms.sendAlarms[i].sendTime ) {
    		rtc_Alarms.sendAlarmsNum = i;
    		i = 8;
    	}
    }

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.sendParamsAlarms[i].sendTime = params_send_network_params_time_send_time(i);
    	if ( 0xFF == rtc_Alarms.sendParamsAlarms[i].sendTime ) {
    		rtc_Alarms.sendParamsAlarmsNum = i;
    		i = 8;
    	}
    }

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.readSensorAlarms[i].sendTime = params_sensor_read_time_init_time(i);
    	rtc_Alarms.readSensorAlarms[i].endTime  = params_sensor_read_time_end_time(i);
    	rtc_Alarms.readSensorAlarms[i].cycle    = params_sensor_read_time_cycle(i);
    	if ( 0xFF == rtc_Alarms.readSensorAlarms[i].sendTime ) {
    		rtc_Alarms.readSensorAlarmsNum = i;
    		i = 8;
    	}
    }

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.sendSensorAlarms[i].sendTime = params_sensor_send_time_send_time(i);
    	if ( 0xFF == rtc_Alarms.sendSensorAlarms[i].sendTime ) {
    		rtc_Alarms.sendSensorAlarmsNum = i;
    		i = 8;
    	}
    }

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.readModbusAlarms[i].sendTime = params_modbus_read_time_init_time(i);
    	rtc_Alarms.readModbusAlarms[i].endTime  = params_modbus_read_time_end_time(i);
    	rtc_Alarms.readModbusAlarms[i].cycle    = params_modbus_read_time_cycle(i);
    	if ( 0xFF == rtc_Alarms.readModbusAlarms[i].sendTime ) {
    		rtc_Alarms.readModbusAlarmsNum = i;
    		i = 8;
    	}
    }

    for (i = 0; i < 8; i++) {
    	rtc_Alarms.sendModbusAlarms[i].sendTime = params_modbus_send_time_send_time(i);
    	if ( 0xFF == rtc_Alarms.sendModbusAlarms[i].sendTime ) {
    		rtc_Alarms.sendModbusAlarmsNum = i;
    		i = 8;
    	}
    }

    rtc_system_getCurrentAlarmFromBKUP();
    rtc_system_getReadInitAlarmFromBKUP();
    rtc_system_getReadEndAlarmFromBKUP();
    rtc_system_getSendAlarmFromBKUP();
}

uint32_t rtc_system_get_FreqLSI( void )
{
	return uwLsiFreq;
}

/**
  * @brief  Configure the RTC peripheral by selecting the clock source.
  * @param  None
  * @retval None
  */
void rtc_system_config(void)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
//	RTC_AlarmTypeDef sAlarm = {0};

	/** Initialize RTC Only To Create htrc.
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* Check if the StandBy flag is set */
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET) {
		/* Clear StandBy flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);

		/* Check if the StandBy flag is cleared */
		if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET) {
			while (1);
		}

		HAL_RTC_WaitForSynchro(&hrtc);
		/* No need to configure the RTC as the RTC config(clock source, enable,
		 prescaler,...) are kept after wake-up from STANDBY */
	} else {
		/* RTC Configuration ******************************************************/
		/* To change the source clock of the RTC feature (LSE, LSI), You have to:
		     - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
		     - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
		       configure the RTC clock source (to be done once after reset).
		     - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
		       __HAL_RCC_BACKUPRESET_RELEASE().
		     - Configure the needed RTC clock source */
		__HAL_RCC_PWR_CLK_ENABLE();
		HAL_PWR_EnableBkUpAccess();
//		HAL_PWR_DisableBkUpAccess();
//		__HAL_RCC_BACKUPRESET_FORCE();
//	    __HAL_RCC_BACKUPRESET_RELEASE();

		/* Select the RTC Clock Source */
		RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
		PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			Error_Handler();
		}

		/* RTC clock enable */
		__HAL_RCC_RTC_ENABLE();
		__HAL_RCC_RTCAPB_CLK_ENABLE();

		/* Wait for RTC APB registers synchronisation */
		HAL_RTC_WaitForSynchro(&hrtc);

		/* Get the LSI frequency:  TIM16 is used to measure the LSI frequency */
		uwLsiFreq = __getLSEFrequency();

		/*##-1- Configure the RTC peripheral #######################################*/
		/* Configure RTC prescaler and RTC data registers */
		/* RTC configured as follows:
		 - Hour Format    = Format 24
		 - Asynch Prediv  = Value according to source clock
		 - Synch Prediv   = Value according to source clock
		 - OutPut         = Output Disable
		 - OutPutPolarity = High Polarity
		 - OutPutType     = Open Drain */
		hrtc.Instance = RTC;
		hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
		hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV; /* 0x7F */
		hrtc.Init.SynchPrediv = (uwLsiFreq / 128) - 1; /* (32 kHz / 128) - 1 = 0xF9 */
		hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
		hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

		if (HAL_RTC_Init(&hrtc) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}

		/** Initialize RTC and set the Time and Date
		 */
		sTime.Hours = 0x0;
		sTime.Minutes = 0x0;
		sTime.Seconds = 0x0;
		sTime.TimeFormat = RTC_HOURFORMAT12_AM;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 0x1;
		sDate.Year = 0x0;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
//		/** Enable the Alarm A
//		 */
//		sAlarm.AlarmTime.Hours = 0x0;
//		sAlarm.AlarmTime.Minutes = 0x0;
//		sAlarm.AlarmTime.Seconds = 0x0;
//		sAlarm.AlarmTime.SubSeconds = 0x0;
//		sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//		sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
//		sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
//		sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
//		sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//		sAlarm.AlarmDateWeekDay = 0x1;
//		sAlarm.Alarm = RTC_ALARM_A;
//		if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
//		{
//			Error_Handler();
//		}
//		/** Enable the Alarm B
//		 */
//		sAlarm.AlarmDateWeekDay = 0x1;
//		sAlarm.Alarm = RTC_ALARM_B;
//		if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
//		{
//			Error_Handler();
//		}

//		/*##-2- Check if data stored in BackUp register1: Wakeup timer enable #######*/
//		/* Read the Back Up Register 1 Data */
//		if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) == WAKEUP_TIMER_ENABLE) {
//			/* if the wakeup timer is enabled then deactivate it to disable the wakeup timer interrupt */
//			if (HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandle) != HAL_OK) {
//				/* Initialization Error */
//				Error_Handler();
//			}
//		}
//
//		/*##-3- Configure the RTC Wakeup peripheral #################################*/
//		/* Setting the Wakeup time to 1 s
//						 If RTC_WAKEUPCLOCK_CK_SPRE_16BITS is selected, the frequency is 1Hz,
//						 this allows to get a wakeup time equal to 1 s if the counter is 0x0 */
//		HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, 0x0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
//
//		/*##-4- Write 'wakeup timer enabled' tag in RTC Backup data Register 1 #######*/
//		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, WAKEUP_TIMER_ENABLE);
	}
	/* RTC_Alarm_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_IRQn);
//	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

	__initAlarmsStruct();

}

//static uint32_t __getSeconds( void )
//{
//	uint32_t seconds;
//	char imei[16];
//
//	memset( imei, 0, 16 );
//	memcpy(imei, params_get_imei(), 16*sizeof(char));
//
//	seconds = atoi(&imei[12]);
//
//	return seconds;
//}

void rtc_system_configStopMode(void)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
//	RTC_AlarmTypeDef sAlarm = {0};

//	uint32_t sec_reset = 0, minutes_add = 0, seconds_add = 0;

	/** Initialize RTC Only To Create htrc.
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}

	/* Check if the StandBy flag is set */
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET) {
		/* Clear StandBy flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);

		/* Check if the StandBy flag is cleared */
		if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET) {
			while (1);
		}

		HAL_RTC_WaitForSynchro(&hrtc);
		/* No need to configure the RTC as the RTC config(clock source, enable,
		 prescaler,...) are kept after wake-up from STANDBY */
		rtc_refresh_time = 1;
		rtc_system_setGetTime(1);
		shutdown_set_tamper_param(TAMPER_HARD_RESET_DAYLY);
	} else {
		/* RTC Configuration ******************************************************/
		/* To change the source clock of the RTC feature (LSE, LSI), You have to:
     	 - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
     	 - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
       	   configure the RTC clock source (to be done once after reset).
     	 - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
       	   __HAL_RCC_BACKUPRESET_RELEASE().
     	 - Configure the needed RTC clock source */
		__HAL_RCC_PWR_CLK_ENABLE();
		HAL_PWR_EnableBkUpAccess();
		//  HAL_PWR_DisableBkUpAccess();
//		__HAL_RCC_BACKUPRESET_FORCE();
//		__HAL_RCC_BACKUPRESET_RELEASE();

		/* Select the RTC Clock Source */
		RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
		PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			Error_Handler();
		}

		/* RTC clock enable */
		__HAL_RCC_RTC_ENABLE();
		__HAL_RCC_RTCAPB_CLK_ENABLE();

		/* Wait for RTC APB registers synchronisation */
		HAL_RTC_WaitForSynchro(&hrtc);

		/* Get the LSI frequency:  TIM16 is used to measure the LSE frequency */
		uwLsiFreq = __getLSEFrequency();

		/*##-1- Configure the RTC peripheral #######################################*/
		/* Configure RTC prescaler and RTC data registers */
		/* RTC configured as follows:
 	 	 - Hour Format    = Format 24
 	 	 - Asynch Prediv  = Value according to source clock
 	 	 - Synch Prediv   = Value according to source clock
 	 	 - OutPut         = Output Disable
 	 	 - OutPutPolarity = High Polarity
 	 	 - OutPutType     = Open Drain */
		hrtc.Instance = RTC;
		hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
		hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV; /* 0x7F */
		hrtc.Init.SynchPrediv = 0xFF;//(uwLsiFreq / 128) - 1; /* (32 kHz / 128) - 1 = 0xF9 */
		hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
		hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
		hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

		if (HAL_RTC_Init(&hrtc) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}

		/** Initialize RTC and set the Time and Date
		 */
		sTime.Hours = 0x0;
		sTime.Minutes = 0x0;
		sTime.Seconds = 0x0;
		sTime.TimeFormat = RTC_HOURFORMAT12_AM;
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Month = RTC_MONTH_JANUARY;
		sDate.Date = 0x1;
		sDate.Year = 0x0;

		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
		if ( CLEAR_MEMORY_FORCE_HARD_RESET == params_manteinance_delete_memory() ) {
			shutdown_set_tamper_param(TAMPER_HARD_RESET_MW);
		} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
			shutdown_set_tamper_param(TAMPER_HARD_RESET_DAYLY);
		} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) {
			shutdown_set_tamper_param(TAMPER_SWITCH_ON);
		} else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
			shutdown_set_tamper_param(TAMPER_WATCHDOG);
		}
	}
	HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_8SEC, RTC_SMOOTHCALIB_PLUSPULSES_RESET, 0);
	HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_8SEC, RTC_SMOOTHCALIB_PLUSPULSES_SET, 511);
	/** Enable the Alarm B
	 */
	RTC_AlarmTypeDef sAlarm = {0};

//	sec_reset   = __getSeconds();
//	minutes_add = sec_reset/60;
//	if ( minutes_add > 55 ) {
//		minutes_add = minutes_add - 5;
//	}
//	seconds_add = sec_reset%60;
	struct tm       *date;
	time_t reset_time = params_get_time_reset();

	date = gmtime(&reset_time);

	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);/* Disable the Alarm B */
	if ( reset_time != 0 ) {
	sAlarm.Alarm                = RTC_ALARM_B;
	sAlarm.AlarmDateWeekDay     = date->tm_mday; //RTC_WEEKDAY_MONDAY;
//	sAlarm.AlarmDateWeekDaySel  = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	sAlarm.AlarmMask            = RTC_ALARMMASK_DATEWEEKDAY/* | RTC_ALARMMASK_SECONDS*/;
	sAlarm.AlarmSubSecondMask   = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	sAlarm.AlarmTime.Hours      = date->tm_hour;//0;//1;
	sAlarm.AlarmTime.Minutes    = date->tm_min;//5 + minutes_add;//5;//35;
	sAlarm.AlarmTime.Seconds    = date->tm_sec;//seconds_add;//10;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
	/* Enable RTC Alarm B Interrupt: this Interrupt will wake-up the system from
	STANDBY mode (RTC Alarm IT not enabled in NVIC) */
	__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRB);

	__HAL_RTC_ALARMB_ENABLE(&hrtc);

	__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);
	}
	rtc_Calendar.programmedReset = 0;

	/* RTC_Alarm_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_IRQn);

	__initAlarmsStruct();
}

/**
 * @fn void rtc_system_ReconfigStopMode(void)
 * @brief
 *
 * @pre
 * @post
 */
void rtc_system_ReconfigStopMode(void)
{
#if 1
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	/* Select the RTC Clock Source */
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	/* RTC clock enable */
	__HAL_RCC_RTC_ENABLE();
	__HAL_RCC_RTCAPB_CLK_ENABLE();

	/* Wait for RTC APB registers synchronisation */
	HAL_RTC_WaitForSynchro(&hrtc);

	/* Get the LSI frequency:  TIM16 is used to measure the LSE frequency */
	uwLsiFreq = __getLSEFrequency();

	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
	hrtc.Init.SynchPrediv = (uwLsiFreq / 128) - 1;//249;//255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
//	__HAL_RCC_PWR_CLK_ENABLE();
//	HAL_PWR_EnableBkUpAccess();
//
//
//
//	/* RTC clock enable */
//	__HAL_RCC_RTC_ENABLE();
//	__HAL_RCC_RTCAPB_CLK_ENABLE();

	/* Wait for RTC APB registers synchronisation */
//	HAL_RTC_WaitForSynchro(&hrtc);
#endif
#if 0
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/** Initialize RTC Only To Create htrc.
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* RTC Configuration ******************************************************/
	/* To change the source clock of the RTC feature (LSE, LSI), You have to:
     - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
     - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
       configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
       __HAL_RCC_BACKUPRESET_RELEASE().
     - Configure the needed RTC clock source */
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	//  HAL_PWR_DisableBkUpAccess();

	/* Select the RTC Clock Source */
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	/* RTC clock enable */
	__HAL_RCC_RTC_ENABLE();

	/* Wait for RTC APB registers synchronisation */
	HAL_RTC_WaitForSynchro(&hrtc);

	/* Get the LSI frequency:  TIM14 is used to measure the LSI frequency */
	uwLsiFreq = __getLSIFrequency();

	/*##-1- Configure the RTC peripheral #######################################*/
	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follows:
 	 - Hour Format    = Format 24
 	 - Asynch Prediv  = Value according to source clock
 	 - Synch Prediv   = Value according to source clock
 	 - OutPut         = Output Disable
 	 - OutPutPolarity = High Polarity
 	 - OutPutType     = Open Drain */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV; /* 0x7F */
	hrtc.Init.SynchPrediv = (uwLsiFreq / 128) - 1; /* (32 kHz / 128) - 1 = 0xF9 */
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
#endif
}
#if 0
void rtc_system_configStopMode(void)
{
	/* RTC Configuration ******************************************************/
	/* To change the source clock of the RTC feature (LSE, LSI), You have to:
	 - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
	 - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
	 configure the RTC clock source (to be done once after reset).
	 - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
	 __HAL_RCC_BACKUPRESET_RELEASE().
	 - Configure the needed RTC clock source */
	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_PWR_EnableBkUpAccess();
	HAL_PWR_DisableBkUpAccess();

	/* Select the RTC Clock Source */
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/* RTC clock enable */
	__HAL_RCC_RTC_ENABLE();

	/* Wait for RTC APB registers synchronisation */
	HAL_RTC_WaitForSynchro(&hrtc);

	/* Get the LSI frequency:  TIM14 is used to measure the LSI frequency */
	uwLsiFreq = __getLSIFrequency();

	/*##-1- Configure the RTC peripheral #######################################*/
	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follows:
	 - Hour Format    = Format 24
	 - Asynch Prediv  = Value according to source clock
	 - Synch Prediv   = Value according to source clock
	 - OutPut         = Output Disable
	 - OutPutPolarity = High Polarity
	 - OutPutType     = Open Drain */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = RTC_ASYNCH_PREDIV; /* 0x7F */
	hrtc.Init.SynchPrediv = (uwLsiFreq / 100) - 1; /* (32 kHz / 128) - 1 = 0xF9 */
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Check if data stored in BackUp register1: Wakeup timer enable #######*/
	/* Read the Back Up Register 1 Data */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == WAKEUP_TIMER_ENABLE) {
		/* if the wakeup timer is enabled then deactivate it to disable the wakeup timer interrupt */
		if (HAL_RTCEx_DeactivateWakeUpTimer(&hrtc) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}
	}

	/*##-3- Configure the RTC Wakeup peripheral #################################*/
	/* Setting the Wakeup time to 1 s
	 If RTC_WAKEUPCLOCK_CK_SPRE_16BITS is selected, the frequency is 1Hz,
	 this allows to get a wakeup time equal to 1 s if the counter is 0x0 */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	/*##-4- Write 'wakeup timer enabled' tag in RTC Backup data Register 1 #######*/
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, WAKEUP_TIMER_ENABLE);
}
#endif
/**
  * @brief  Configures system clock after wake-up from STOP: enable MSI, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  */
void rtc_system_configStopModeWhenElapses(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* Enable PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}
#if 1
/**
  * @brief  Configures TIM16 to measure the LSI oscillator frequency.
  * @param  None
  * @retval LSI Frequency
  */
static uint32_t __getLSEFrequency(void)
{
  TIM_IC_InitTypeDef    TIMInput_Config = {0}; /* Timer Input Capture Configuration Structure declaration */;

  /* Configure the TIM peripheral *********************************************/
  /* Set TIMx instance */
  Input_Handle.Instance = TIM16;

  /* Enable TIMx clock */
  __HAL_RCC_TIM16_CLK_ENABLE();

  /* Reset TIMx registers */
  HAL_TIM_IC_DeInit(&Input_Handle);

  /* Connect internally the TIM16 Input Capture of TIM_CHANNEL_1 to the LSI clock output */
//  HAL_TIMEx_RemapConfig(&Input_Handle, TIM_TIM16_TI1_LSE);

  /* TIM16 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM16 TIM_CHANNEL_1.
     The Rising edge is used as active edge.
     The TIM16 Capture/Compare register associated to TIM_CHANNEL_1
     is used to compute the frequency value.
  ------------------------------------------------------------ */
  Input_Handle.Init.Prescaler         = 0;
  Input_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Input_Handle.Init.Period            = 0xFFFF;
  Input_Handle.Init.ClockDivision     = 0;
  if(HAL_TIM_IC_Init(&Input_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Input Capture of TIM_CHANNEL_1 */
  TIMInput_Config.ICPolarity  = TIM_ICPOLARITY_RISING;
  TIMInput_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIMInput_Config.ICPrescaler = TIM_ICPSC_DIV8;
  TIMInput_Config.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&Input_Handle, &TIMInput_Config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  if (HAL_TIMEx_TISelection(&Input_Handle, TIM_TIM16_TI1_LSE, TIM_CHANNEL_1) != HAL_OK)
  {
	/* Initialization Error */
	Error_Handler();
  }

  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&Input_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Wait until the TIM16 gets 2 LSI edges */
  while(uwCaptureNumber != 2)
  {
  }

  /* Disable TIM16 Capture/Compare channel Interrupt Request */
  HAL_TIM_IC_Stop_IT(&Input_Handle, TIM_CHANNEL_1);

  /* Deinitialize the TIM16 peripheral registers to their default reset values */
  HAL_TIM_IC_DeInit(&Input_Handle);

  return uwLsiFreq;
}
#endif
#if 0
/**
  * @brief  Configures TIM16 to measure the LSI oscillator frequency.
  * @param  None
  * @retval LSI Frequency
  */
#define LSI_NUMBER_OF_LOOPS         ((uint32_t)10)
static uint32_t __getLSIFrequency(void)
{
  TIM_IC_InitTypeDef    TIMInput_Config;

  /* Configure the TIM peripheral *********************************************/
  /* Set TIMx instance */
  Input_Handle.Instance = TIM16;

  /* TIM16 configuration: Input Capture mode ---------------------
     The LSI oscillator is connected to TIM16 TIM_CHANNEL_1.
     The Rising edge is used as active edge.
     The TIM16 Capture/Compare register associated to TIM_CHANNEL_1
     is used to compute the frequency value.
  ------------------------------------------------------------ */
  Input_Handle.Init.Prescaler         = 0;
  Input_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Input_Handle.Init.Period            = 0xFFFF;
  Input_Handle.Init.ClockDivision     = 0;
  if(HAL_TIM_IC_Init(&Input_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Connect internally the TIM16 Input Capture of TIM_CHANNEL_1 to the LSI clock output */
  HAL_TIMEx_RemapConfig(&Input_Handle, TIM_TIM16_TI1_LSI);

  /* Configure the Input Capture of TIM_CHANNEL_1 */
  TIMInput_Config.ICPolarity  = TIM_ICPOLARITY_RISING;
  TIMInput_Config.ICSelection = TIM_ICSELECTION_DIRECTTI;
  TIMInput_Config.ICPrescaler = TIM_ICPSC_DIV8;
  TIMInput_Config.ICFilter    = 0;
  if(HAL_TIM_IC_ConfigChannel(&Input_Handle, &TIMInput_Config, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  uint32_t loopcounter = 0;
  while(loopcounter < LSI_NUMBER_OF_LOOPS)
  {
  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&Input_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  TIM16->SR = 0;
  /* Enable the TIMx IRQ channel */
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* Wait until the TIM16 gets 2 LSI edges */
  while(uwCaptureNumber != 2)
  {
  }
  HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* Disable TIM16 Capture/Compare channel Interrupt Request */
  HAL_TIM_IC_Stop_IT(&Input_Handle, TIM_CHANNEL_1);

  if (loopcounter != 0) {
	  /* Compute the frequency value: multiplying by 8 is due to TIM_ICPSC_DIV8 */
	  /* Then add the current frequency to previous accumulation */
	  uwLsiFreq += uwLsiFreq;
  }
  loopcounter++;
  }
  /* Deinitialize the TIM16 peripheral registers to their default reset values */
  HAL_TIM_IC_DeInit(&Input_Handle);

  uwLsiFreq = uwLsiFreq / LSI_NUMBER_OF_LOOPS;
  return uwLsiFreq;
}

#endif
/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM IC handle
  * @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Get the Input Capture value */
  tmpCCTIM_CHANNEL_1[uwCaptureNumber++] = HAL_TIM_ReadCapturedValue(&Input_Handle, TIM_CHANNEL_1);

  if (uwCaptureNumber >= 2)
  {
    if ( tmpCCTIM_CHANNEL_1[0] > tmpCCTIM_CHANNEL_1[1] )
    {
      /* Compute the period length */
      uwPeriodValue = (uint16_t)(0xFFFF - tmpCCTIM_CHANNEL_1[0] + tmpCCTIM_CHANNEL_1[1] + 1);
    }
    else
    {
      /* Compute the period length */
      uwPeriodValue = (uint16_t)(tmpCCTIM_CHANNEL_1[1] - tmpCCTIM_CHANNEL_1[0]);
    }
    /* Frequency computation */
    uwLsiFreq  = (uint32_t) SystemCoreClock / uwPeriodValue;
    uwLsiFreq *= 8;
  }
}
void rtc_system_setBackUpRegister( uint8_t _register, uint32_t data )
{
	/**@todo MACRO for code efficiency */
	HAL_RTCEx_BKUPWrite(&hrtc, _register, data);
}

uint32_t rtc_system_readBackUpRegister( uint32_t _register )
{
	/**@todo MACRO for code efficiency */
	return HAL_RTCEx_BKUPRead(&hrtc,_register);
}

//rtc_Alarms getters and setters.
//////////////////////////////////
uint32_t rtc_system_time_to_reset( uint32_t _diff_in_secs )
{
	uint32_t ret = 0;
	struct tm       *date;
	time_t reset_time = params_get_time_reset();

	date = gmtime(&reset_time);

	date->tm_mday = Sys_time.Date.Date;
	date->tm_mon  = Sys_time.Date.Month - 1;
	date->tm_year = Sys_time.Date.Year  + 2000 - 1900;
	date->tm_isdst = -1;
	date->tm_yday = 0;

	reset_time = mktime(date);

	if (Tick_Get(SECONDS) < reset_time)
	{
		if (( (reset_time - Tick_Get(SECONDS)) <= 4*_diff_in_secs ) || ( (reset_time - Tick_Get(SECONDS)) <= 2*_diff_in_secs ))
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC RTC SYSTEM> SET __programWord()  \r\n", (int)Tick_Get( SECONDS ));
			ret = 1;
		}
	}

	return ret;
}

void rtc_system_setReadAlarmInitTime( uint8_t alarm_num, uint8_t _read_init_time )
{
	rtc_Alarms.readAlarms[alarm_num].sendTime = _read_init_time;
}

uint8_t rtc_system_getReadAlarmInitTime( uint8_t alarm_num )
{
	return rtc_Alarms.readAlarms[alarm_num].sendTime;
}

void rtc_system_setReadAlarmEndTime( uint8_t alarm_num, uint8_t _read_end_time )
{
	rtc_Alarms.readAlarms[alarm_num].endTime = _read_end_time;
}

uint8_t rtc_system_getReadAlarmEndTime( uint8_t alarm_num )
{
	return rtc_Alarms.readAlarms[alarm_num].endTime;
}

void rtc_system_setReadAlarmCycle( uint8_t alarm_num, uint8_t _cycle )
{
	rtc_Alarms.readAlarms[alarm_num].cycle = _cycle;
}

uint8_t rtc_system_getReadAlarmCycle( uint8_t alarm_num )
{
	return rtc_Alarms.readAlarms[alarm_num].cycle;
}

void rtc_system_setReadAlarmInitNext( uint8_t alarmInitNext )
{
	rtc_Alarms.readInitAlarmNext = alarmInitNext;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, rtc_Alarms.readInitAlarmNext);
}

uint8_t rtc_system_getReadAlarmInitNext( void )
{
	return rtc_Alarms.readInitAlarmNext;
}

void rtc_system_setReadAlarmEndNext( uint8_t alarmNext )
{
	rtc_Alarms.readEndAlarmNext= alarmNext;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, rtc_Alarms.readEndAlarmNext);
}

uint8_t rtc_system_getReadAlarmEndNext( void )
{
	return rtc_Alarms.readEndAlarmNext;
}

void rtc_system_setReadAlarmCycleNext( uint8_t alarmNext )
{
	rtc_Alarms.readCycleNext= alarmNext;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR12, rtc_Alarms.readCycleNext);
}

uint32_t rtc_system_getReadAlarmCycleNextInSeconds( void )
{
	return (rtc_Alarms.readCycleNext*60);
}

uint32_t rtc_system_getReadAlarmCycleNext( void )
{
	return rtc_Alarms.readCycleNext;
}

uint8_t rtc_system_getReadAlarmNum( void )
{
	return rtc_Alarms.readAlarmsNum;
}

void rtc_system_setReadAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.readAlarmsNum = alarmNum;
}

void rtc_system_setReadSensorAlarmInitTime( uint8_t alarm_num, uint8_t _read_init_time )
{
	rtc_Alarms.readSensorAlarms[alarm_num].sendTime = _read_init_time;
}

uint8_t rtc_system_getReadSensorAlarmInitTime( uint8_t alarm_num )
{
	return rtc_Alarms.readSensorAlarms[alarm_num].sendTime;
}

void rtc_system_setReadSensorAlarmEndTime( uint8_t alarm_num, uint8_t _read_end_time )
{
	rtc_Alarms.readSensorAlarms[alarm_num].endTime = _read_end_time;
}

uint8_t rtc_system_getReadSensorAlarmEndTime( uint8_t alarm_num )
{
	return rtc_Alarms.readSensorAlarms[alarm_num].endTime;
}

void rtc_system_setReadSensorAlarmCycle( uint8_t alarm_num, uint16_t _cycle )
{
	rtc_Alarms.readSensorAlarms[alarm_num].cycle = _cycle;
}

uint8_t rtc_system_getReadSensorAlarmCycle( uint8_t alarm_num )
{
	return rtc_Alarms.readSensorAlarms[alarm_num].cycle;
}

void rtc_system_setReadSensorAlarmInitNext( uint8_t alarmInitNext )
{
	rtc_Alarms.readSensorInitAlarmNext = alarmInitNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, rtc_Alarms.readInitAlarmNext);
}

uint8_t rtc_system_getReadSensorAlarmInitNext( void )
{
	return rtc_Alarms.readSensorInitAlarmNext;
}

void rtc_system_setReadSensorAlarmEndNext( uint8_t alarmNext )
{
	rtc_Alarms.readSensorEndAlarmNext= alarmNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, rtc_Alarms.readEndAlarmNext);
}

uint8_t rtc_system_getReadSensorAlarmEndNext( void )
{
	return rtc_Alarms.readSensorEndAlarmNext;
}

void rtc_system_setReadSensorAlarmCycleNext( uint16_t alarmNext )
{
	rtc_Alarms.readSensorCycleNext= alarmNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR12, rtc_Alarms.readCycleNext);
}

/**
  * @brief
  * @retval Returns the period in seconds for the next sample
  */
uint32_t rtc_system_getReadSensorAlarmCycleNextInSeconds( void )
{
	return (rtc_Alarms.readSensorCycleNext);
}

uint32_t rtc_system_getReadSensorAlarmCycleNext( void )
{
	return rtc_Alarms.readSensorCycleNext;
}

uint8_t rtc_system_getReadSensorAlarmNum( void )
{
	return rtc_Alarms.readSensorAlarmsNum;
}

void rtc_system_setReadSensorAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.readSensorAlarmsNum = alarmNum;
}

void rtc_system_setReadModbusAlarmInitTime( uint8_t alarm_num, uint8_t _read_init_time )
{
	rtc_Alarms.readModbusAlarms[alarm_num].sendTime = _read_init_time;
}

uint8_t rtc_system_getReadModbusAlarmInitTime( uint8_t alarm_num )
{
	return rtc_Alarms.readModbusAlarms[alarm_num].sendTime;
}

void rtc_system_setReadModbusAlarmEndTime( uint8_t alarm_num, uint8_t _read_end_time )
{
	rtc_Alarms.readModbusAlarms[alarm_num].endTime = _read_end_time;
}

uint8_t rtc_system_getReadModbusAlarmEndTime( uint8_t alarm_num )
{
	return rtc_Alarms.readModbusAlarms[alarm_num].endTime;
}

void rtc_system_setReadModbusAlarmCycle( uint8_t alarm_num, uint16_t _cycle )
{
	rtc_Alarms.readModbusAlarms[alarm_num].cycle = _cycle;
}

uint8_t rtc_system_getReadModbusAlarmCycle( uint8_t alarm_num )
{
	return rtc_Alarms.readModbusAlarms[alarm_num].cycle;
}

void rtc_system_setReadModbusAlarmInitNext( uint8_t alarmInitNext )
{
	rtc_Alarms.readModbusInitAlarmNext = alarmInitNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR6, rtc_Alarms.readInitAlarmNext);
}

uint8_t rtc_system_getReadModbusAlarmInitNext( void )
{
	return rtc_Alarms.readModbusInitAlarmNext;
}

void rtc_system_setReadModbusAlarmEndNext( uint8_t alarmNext )
{
	rtc_Alarms.readModbusEndAlarmNext= alarmNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR7, rtc_Alarms.readEndAlarmNext);
}

uint8_t rtc_system_getReadModbusAlarmEndNext( void )
{
	return rtc_Alarms.readModbusEndAlarmNext;
}

void rtc_system_setReadModbusAlarmCycleNext( uint16_t alarmNext )
{
	rtc_Alarms.readModbusCycleNext= alarmNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR12, rtc_Alarms.readCycleNext);
}

uint32_t rtc_system_getReadModbusAlarmCycleNextInSeconds( void )
{
	return (rtc_Alarms.readModbusCycleNext);
}

uint32_t rtc_system_getReadModbusAlarmCycleNext( void )
{
	return rtc_Alarms.readModbusCycleNext;
}

uint8_t rtc_system_getReadModbusAlarmNum( void )
{
	return rtc_Alarms.readModbusAlarmsNum;
}

void rtc_system_setReadModbusAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.readModbusAlarmsNum = alarmNum;
}
void rtc_system_setSendAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time )
{
	rtc_Alarms.sendAlarms[alarm_num].sendTime = _send_init_time;
}

uint8_t rtc_system_getSendAlarmSendTime( uint8_t alarm_num )
{
	return rtc_Alarms.sendAlarms[alarm_num].sendTime;
}

void rtc_system_setSendAlarmSendNext( uint8_t alarmNext )
{
	rtc_Alarms.sendAlarmNext = alarmNext;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8, rtc_Alarms.sendAlarmNext);
}

uint8_t rtc_system_getSendAlarmSendNext( void )
{
	return rtc_Alarms.sendAlarmNext;
}

void rtc_system_setSendAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.sendAlarmsNum = alarmNum;
}

void rtc_system_setSendSensorAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time )
{
	rtc_Alarms.sendSensorAlarms[alarm_num].sendTime = _send_init_time;
}

uint8_t rtc_system_getSendSensorAlarmSendTime( uint8_t alarm_num )
{
	return rtc_Alarms.sendSensorAlarms[alarm_num].sendTime;
}

void rtc_system_setSendSensorAlarmSendNext( uint8_t alarmNext )
{
	rtc_Alarms.sendSensorAlarmNext = alarmNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8, rtc_Alarms.sendAlarmNext);
}

uint8_t rtc_system_getSendSensorAlarmSendNext( void )
{
	return rtc_Alarms.sendSensorAlarmNext;
}

void rtc_system_setSendSensorAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.sendSensorAlarmsNum = alarmNum;
}

void rtc_system_setSendModbusAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time )
{
	rtc_Alarms.sendModbusAlarms[alarm_num].sendTime = _send_init_time;
}

uint8_t rtc_system_getSendModbusAlarmSendTime( uint8_t alarm_num )
{
	return rtc_Alarms.sendModbusAlarms[alarm_num].sendTime;
}

void rtc_system_setSendModbusAlarmSendNext( uint8_t alarmNext )
{
	rtc_Alarms.sendModbusAlarmNext = alarmNext;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR8, rtc_Alarms.sendAlarmNext);
}

uint8_t rtc_system_getSendModbusAlarmSendNext( void )
{
	return rtc_Alarms.sendModbusAlarmNext;
}

void rtc_system_setSendModbusAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.sendModbusAlarmsNum = alarmNum;
}

void rtc_system_setSendParamsAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time )
{
	rtc_Alarms.sendParamsAlarms[alarm_num].sendTime = _send_init_time;
}

uint8_t rtc_system_getSendParamsAlarmSendTime( uint8_t alarm_num )
{
	return rtc_Alarms.sendParamsAlarms[alarm_num].sendTime;
}

void rtc_system_setSendParamsAlarmSendNext( uint8_t alarmNext )
{
	rtc_Alarms.sendParamsAlarmNext = alarmNext;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR13, rtc_Alarms.sendParamsAlarmNext);
}

uint8_t rtc_system_getSendParamsAlarmSendNext( void )
{
	return rtc_Alarms.sendParamsAlarmNext;
}

void rtc_system_setSendParamsAlarmNum( uint8_t alarmNum )
{
	rtc_Alarms.sendParamsAlarmsNum = alarmNum;
}

// rtc_Calendar getters and setters.
/////////////////////////////////////

/**
 * @fn uint8_t rtc_system_InitByServer(void)
 * @brief Returns whether the RTC has been initialized by the server or it is not yet configured.
 *
 * @return rtc_Calendar.initTimeserver
 * 			@arg 0 - RTC not initialized by the server.
 * 			@arg 1 - RTC initilaized by the server.
 */
uint8_t rtc_system_InitByServer(void)
{
	return rtc_Calendar.initTimeServer;
}

/**
 * @fn void rtc_system_resetInitByServer(void)
 * @brief Resets the RTC initilized by server flag
 *
 */
void rtc_system_resetInitByServer(void)
{
	rtc_Calendar.initTimeServer = 0;
}

uint32_t t_seconds_backup = 0;
/**
 * @fn void rtc_system_setCurrentAlarm(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _current_alarm
 */
void rtc_system_set_t_seconds_backup(void)
{
	t_seconds_backup = Tick_Get(SECONDS);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, t_seconds_backup);
}

uint32_t rtc_system_get_t_seconds_backup( void )
{
	t_seconds_backup = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);

	return t_seconds_backup;
}

void rtc_system_setAlarmNext( uint8_t alarmNext )
{
	rtc_Calendar.alarmNext = alarmNext;
}

uint8_t rtc_system_getAlarmNext( void )
{
	return rtc_Calendar.alarmNext;
}

void rtc_system_setAlarmsNum( uint8_t alarmsNum )
{
	rtc_Calendar.alarmsNum = alarmsNum;
}

uint8_t rtc_system_getAlarmsNum( void )
{
	return rtc_Calendar.alarmsNum;
}

void rtc_system_setAlarmOccured( uint8_t alarmOccured )
{
	rtc_Calendar.alarmOccured = alarmOccured;
}

uint8_t rtc_system_getAlarmOccured( void )
{
	return rtc_Calendar.alarmOccured;
}

uint8_t rtc_system_checkAlarmOccured( void )
{
//	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET) {
//		rtc_Calendar.alarmOccured = 1;
//		RTC_ClearITPendingBit(RTC_IT_ALRA);
//		EXTI_ClearITPendingBit(EXTI_Line17);
//	}
	return rtc_Calendar.alarmOccured;
}

void rtc_system_setAlarmState( uint8_t alarmState )
{
	rtc_Calendar.alarmState = alarmState;
}

uint8_t rtc_system_getAlarmState( void )
{
	return rtc_Calendar.alarmState;
}

void rtc_system_setTimerState( uint8_t alarmState )
{
	rtc_Calendar.timerState = alarmState;
}

uint8_t rtc_system_getTimerState( void )
{
	return rtc_Calendar.timerState;
}

void rtc_system_setGetTime( uint8_t __getTime )
{
	rtc_Calendar.getTime = __getTime;
}

uint8_t rtc_system_getGetTime( void )
{
	return rtc_Calendar.getTime;
}

void rtc_system_setAlarmProgrammedReset( uint8_t programmedReset )
{
	rtc_Calendar.programmedReset = programmedReset;
}

uint8_t rtc_system_getAlarmProgrammedReset( void )
{
	return rtc_Calendar.programmedReset;
}

void rtc_system_setWaitingTimeServer( uint8_t waitingTimeServer )
{
	rtc_Calendar.waitingTimeServer = waitingTimeServer;
}

uint8_t rtc_system_getWaitingTimeServer( void )
{
	return rtc_Calendar.waitingTimeServer;
}

#define DEC_TO_BCD(Number) ((((Number)/10)*16)+((Number)%10))
static void __setSystemTime( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct )
{
	Sys_time.Time.Hours   = RTC_TimeStructure.Hours;
	Sys_time.Time.Minutes = RTC_TimeStructure.Minutes;
	Sys_time.Time.Seconds = RTC_TimeStructure.Seconds;
	Sys_time.Date.Date    = RTC_DateStruct.Date;
	Sys_time.Date.Month   = RTC_DateStruct.Month;
	Sys_time.Date.Year    = RTC_DateStruct.Year;

	Sys_time.created_time[0]  = '2';
	Sys_time.created_time[1]  = '0';
	Sys_time.created_time[2]  = ((DEC_TO_BCD(RTC_DateStruct.Year)>>4)       & 0x0F)+'0';
	Sys_time.created_time[3]  = ((DEC_TO_BCD(RTC_DateStruct.Year)   )       & 0x0F)+'0';
	Sys_time.created_time[4]  = ((DEC_TO_BCD(RTC_DateStruct.Month)>>4)      & 0x0F)+'0';
	Sys_time.created_time[5]  = ((DEC_TO_BCD(RTC_DateStruct.Month)   )      & 0x0F)+'0';
	Sys_time.created_time[6]  = ((DEC_TO_BCD(RTC_DateStruct.Date)>>4)       & 0x0F)+'0';
	Sys_time.created_time[7]  = ((DEC_TO_BCD(RTC_DateStruct.Date)   )       & 0x0F)+'0';
	Sys_time.created_time[8]  = ((DEC_TO_BCD(RTC_TimeStructure.Hours)>>4)   & 0x0F)+'0';
	Sys_time.created_time[9]  = ((DEC_TO_BCD(RTC_TimeStructure.Hours)   )   & 0x0F)+'0';
	Sys_time.created_time[10] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)>>4) & 0x0F)+'0';
	Sys_time.created_time[11] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)   ) & 0x0F)+'0';
	Sys_time.created_time[12] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)>>4) & 0x0F)+'0';
	Sys_time.created_time[13] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)   ) & 0x0F)+'0';

	Sys_time.created_time_telegram[0]  = '2';
	Sys_time.created_time_telegram[1]  = '0';
	Sys_time.created_time_telegram[2]  = ((DEC_TO_BCD(RTC_DateStruct.Year)>>4)		 & 0x0F) + '0';
	Sys_time.created_time_telegram[3]  = ((DEC_TO_BCD(RTC_DateStruct.Year)   )	     & 0x0F) + '0';
	Sys_time.created_time_telegram[4]  = '-';
	Sys_time.created_time_telegram[5]  = ((DEC_TO_BCD(RTC_DateStruct.Month)>>4) 	 & 0x0F)+'0';
	Sys_time.created_time_telegram[6]  = ((DEC_TO_BCD(RTC_DateStruct.Month)   )      & 0x0F)+'0';
	Sys_time.created_time_telegram[7]  = '-';
	Sys_time.created_time_telegram[8]  = ((DEC_TO_BCD(RTC_DateStruct.Date)>>4)       & 0x0F)+'0';
	Sys_time.created_time_telegram[9]  = ((DEC_TO_BCD(RTC_DateStruct.Date)   )       & 0x0F)+'0';
	Sys_time.created_time_telegram[10] = ' ';
	Sys_time.created_time_telegram[11] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)>>4)   & 0x0F) +'0';
	Sys_time.created_time_telegram[12] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)   )   & 0x0F)+'0';
	Sys_time.created_time_telegram[13] = ':';
	Sys_time.created_time_telegram[14] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)>>4) & 0x0F)+'0';
	Sys_time.created_time_telegram[15] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)   ) & 0x0F)+'0';
	Sys_time.created_time_telegram[16] = ':';
	Sys_time.created_time_telegram[17] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)>>4) & 0x0F)+'0';
	Sys_time.created_time_telegram[18] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)   ) & 0x0F)+'0';
}

char * rtc_system_getCreatedTimeFileName( void )
{
	return Sys_time.created_time;
}

char * rtc_system_GetCreatedTime( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    /* Set the date */
    RTC_DateStruct.Date    = date->tm_mday;
    RTC_DateStruct.Month   = date->tm_mon + 1;
    RTC_DateStruct.WeekDay = date->tm_wday + 1;
    RTC_DateStruct.Year    = date->tm_year - ( 2000 - 1900 );

    __setSystemTime(RTC_TimeStructure, RTC_DateStruct);

	return Sys_time.created_time;
}

void rtc_system_SetServerTime( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;
    RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;


    HAL_RTC_SetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);

    RTC_DateStruct.Date    = date->tm_mday;
    RTC_DateStruct.Month   = date->tm_mon + 1;
    RTC_DateStruct.WeekDay = date->tm_wday + 1;
    RTC_DateStruct.Year    = date->tm_year - ( 2000 - 1900 );

    HAL_RTC_SetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

    __setSystemTime(RTC_TimeStructure, RTC_DateStruct);

    LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Set Server Time: %d/%d/%d-%d:%d:%d uwLsiFreq:%d\r\n",
    		(int)server_time,
			(int)RTC_DateStruct.Date,     (int)RTC_DateStruct.Month,      (int)RTC_DateStruct.Year,
			(int)RTC_TimeStructure.Hours, (int)RTC_TimeStructure.Minutes, (int)RTC_TimeStructure.Seconds,
			(int)uwLsiFreq);

    rtc_Calendar.initTimeServer = 1;
}

static inline void __calibRTC( struct tm *date, uint32_t *calib, uint32_t current_time )
{
#define CALIB_TIME (4)
	uint32_t cal = *calib;
	static uint32_t add = 1;
//	static uint32_t init_time = 0, backup_time = 0;

	if ( date->tm_sec != 59 ) {
		if ( 0 == AD_GetADOn() ) {
			if ( date->tm_sec >= 45 ) {
				date->tm_sec = 58;
			} else {
				date->tm_sec = date->tm_sec + 15;
			}
		} else {
#if 1
			if ( 0 == odd_time )
			{
				if ( cal++ < calib_period ) {//6//30
					if ( cal != calib_period - 1 )//5//29
					{
						if ( date->tm_sec != 59 )
						{
//						date->tm_sec = date->tm_sec + 1;//1
//						LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> EVEN CAL:%d\r\n",
//								(int)Tick_Get(SECONDS), (int)cal);
						}
					}
				} else {
					if ( date->tm_sec != 59 )
					{
					cal = 0;
//					date->tm_sec = date->tm_sec + 1;//3;
//					LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> EVEN CAL:%d\r\n",
//							(int)Tick_Get(SECONDS), (int)cal);
					}
				}
			}
			else if ( 1 == odd_time )
			{
				if ((cal < (calib_period + 1)) /*&& (cal != 0)*/)
				{//else if (( cal == calib_period ) || ( cal == 7 ) || ( cal == 5 ) || ( cal == 3 ) || ( cal == 1 ) ) {
					if ( 1 == add )
					{
						if ( date->tm_sec != 59 )
						{
						date->tm_sec = date->tm_sec + 1;
						add = 0;
						LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> ODD CAL:%d\r\n",
								(int)Tick_Get(SECONDS), (int)cal);
						}
					}
					else
					{
						add = 1;
					}
				}
				cal++;
				if (cal >= (calib_period + 1))
				{
					cal = 0;
					if ( date->tm_sec != 58 )
					{
					date->tm_sec = date->tm_sec + 2;
					LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> ODD CAL:%d\r\n",
							(int)Tick_Get(SECONDS), (int)cal);
					}
				}
			}
#endif
#if 0
			if ( 0 == init_time )
			{
				if ( 1 == Tick_cloud_time_init() )
				{
					init_time   = 1;
					backup_time = current_time;
				}
			}
//			if (((current_time - backup_time) >= STAND_BY_TIME) && ((current_time - backup_time) <= (STAND_BY_TIME + 5)))
			if ( (0 == pulses_get_pulse_rx()) && ((current_time - backup_time) >= STAND_BY_TIME) && ((current_time - backup_time) <= (STAND_BY_TIME * 2)) )
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d Add %d seconds\r\n", (int)current_time, (int)(STAND_BY_TIME/CALIB_TIME - 2));
				if ( date->tm_sec != 59 )
				{
					date->tm_sec += STAND_BY_TIME/CALIB_TIME - 2;
					backup_time   = current_time;
				}
				else if ( date->tm_min != 59 )
				{
					date->tm_min += 1;
					date->tm_sec += STAND_BY_TIME/CALIB_TIME - 2;
					date->tm_sec  = date->tm_sec % 60;
					backup_time   = current_time;
				}
			}
#endif
		}
	}
	*calib = cal;
}

uint32_t rtc_system_get_billing_day( void )
{
	return billing_day;
}

uint32_t rtc_system_get_event_log_hour( void )
{
	return event_log_hour;
}

uint32_t rtc_system_get_load_profile_hour( void )
{
	return load_profile_hour;
}

void rtc_system_set_billing_day( uint32_t _bill_st )
{
	billing_day = _bill_st;
}

void rtc_system_set_event_log_hour( uint32_t _ev_st )
{
	event_log_hour = _ev_st;
}

void rtc_system_set_load_profile_hour( uint32_t _ld_prof_st )
{
	load_profile_hour = _ld_prof_st;
}

void __checkDLMSSpecialDates( struct tm        date )
{
#define BILLING_DAY (dlms_client_get_billingday())//(11)
#define EVENT_HOUR  (23)

#define WAIT_FOR_SEND    (0)
#define PREPARED_TO_SEND (1)
#define SENT             (2)

static int      tm_hour_before = EVENT_HOUR;
static uint32_t ld_prof_time_before = 0;
	if ( ( 1 == dlms_client_get_dlms_enable() ) && ( 1 == Tick_cloud_time_init() ) )
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> EVENT LOG hour before:%d\r\n",
				(int)Tick_Get(SECONDS), tm_hour_before);
		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> LOAD PROF LOG hour before:%d\r\n",
				(int)Tick_Get(SECONDS), (int)ld_prof_time_before);

		/* Load Profile send time. */
		if (ld_prof_time_before > Tick_Get(SECONDS))
		{
			ld_prof_time_before = Tick_Get(SECONDS);
		}
		if ( ((Tick_Get(SECONDS) >= (ld_prof_time_before + dlms_client_get_dlms_load_profile_read_time()) )) && ( WAIT_FOR_SEND == load_profile_hour) )
		{
			load_profile_hour   = PREPARED_TO_SEND;
			ld_prof_time_before = Tick_Get(SECONDS);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> LOAD PROF LOG before:%d\r\n",
					(int)Tick_Get(SECONDS), (int)ld_prof_time_before);
		}
		else
		{
			if ( PREPARED_TO_SEND == load_profile_hour )
			{
				load_profile_hour = SENT;
			}
			else if ( date.tm_hour != EVENT_HOUR )
			{
				load_profile_hour = WAIT_FOR_SEND;
			}
		}

		/* Event Profile send time. */
		if (tm_hour_before > date.tm_hour)
		{
			tm_hour_before = date.tm_hour;
		}
		if ( (( EVENT_HOUR == date.tm_hour ) || (date.tm_hour >= (tm_hour_before + 2) )) && ( WAIT_FOR_SEND == event_log_hour) )
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> EVENT LOG hour:%d before:%d\r\n",
					(int)Tick_Get(SECONDS), date.tm_hour,tm_hour_before);
			event_log_hour = PREPARED_TO_SEND;
			tm_hour_before = date.tm_hour;
		}
		else
		{
			if ( ( EVENT_HOUR == date.tm_hour ) && ( PREPARED_TO_SEND == event_log_hour) )
			{
				event_log_hour = SENT;
			}
			else if ( date.tm_hour != EVENT_HOUR )
			{
				event_log_hour = WAIT_FOR_SEND;
			}
		}

		/* Billing Profile send time. */
		if ( ( BILLING_DAY == date.tm_mday) && ( WAIT_FOR_SEND == billing_day ) && ( event_log_hour != PREPARED_TO_SEND ))
		{
			billing_day = PREPARED_TO_SEND;
		}
		else
		{
			if ( ( date.tm_mday != BILLING_DAY ) && ( SENT == billing_day) )
			{
				billing_day = WAIT_FOR_SEND;
			}
		}
	}
}

uint32_t rtc_system_GetServerTime( void )
{
	RTC_TimeTypeDef  RTC_TimeStructure, RTC_TimeStructureNext;
	RTC_DateTypeDef  RTC_DateStruct, RTC_DateStructNext;
	struct tm        date;
	time_t           ret;
//	uint32_t         same_date = 0;
	static time_t    time_prev = 0;
	static uint32_t  calib = 0, milisecs_acc = 0;

	(void)calib;

	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

	ret = HAL_RTC_WaitForSynchro(&hrtc);
	/* Enable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	do {
		/* Get the current time */
		HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
		/* Get the current date */
		HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
		/* Get the current time */
		HAL_RTC_GetTime(&hrtc, &RTC_TimeStructureNext, RTC_FORMAT_BIN);
		/* Get the current date */
		HAL_RTC_GetDate(&hrtc, &RTC_DateStructNext, RTC_FORMAT_BIN);
	} while (RTC_TimeStructure.SubSeconds != RTC_TimeStructureNext.SubSeconds);

	date.tm_hour  = RTC_TimeStructure.Hours;
	date.tm_min   = RTC_TimeStructure.Minutes;
	date.tm_sec   = RTC_TimeStructure.Seconds;
	date.tm_year  = RTC_DateStruct.Year  + ( 2000 - 1900 );
	date.tm_mon   = RTC_DateStruct.Month - 1;
	date.tm_mday  = RTC_DateStruct.Date;
	date.tm_wday  = RTC_DateStruct.WeekDay;
	date.tm_isdst = -1;

	__checkDLMSSpecialDates(date);

	uint32_t secfrac = ((RTC_TimeStructure.SecondFraction-RTC_TimeStructure.SubSeconds)*1000UL) / ((RTC_TimeStructure.SecondFraction+1));
	ret = mktime(&date);
	if ( ret == time_prev )
	{
//		same_date = 1;
		milisecs_acc += Tick_Get(MILLISECONDS);
		if (milisecs_acc >= 1000)
		{
			milisecs_acc = 0;
		}
	}
	time_in_miliseconds_after_low_power = ret * 1000 + secfrac;
//	__calibRTC(&date, &calib, ret);
	ret = mktime(&date);
	time_prev = ret;

    __setSystemTime(RTC_TimeStructure, RTC_DateStruct);
    rtc_Calendar.initTimeServer = 1;

	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Time calib: %d:%d:%d.%d Miliseconds Acc:%d uwLsiFreq:%d.\r\n",
			(int)ret, (int)date.tm_hour, (int)date.tm_min, (int)date.tm_sec, (int)secfrac,
			(int)milisecs_acc, (int)uwLsiFreq);

	return ret;
}

time_t rtc_system_get_time_in_miliseconds( void )
{
	RTC_TimeTypeDef  RTC_TimeStructure, RTC_TimeStructureNext;
	RTC_DateTypeDef  RTC_DateStruct, RTC_DateStructNext;
	struct tm        date;
	time_t           time_in_secs, time_in_milisecs;

	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

	HAL_RTC_WaitForSynchro(&hrtc);
	/* Enable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);
	do {
		/* Get the current time */
		HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
		/* Get the current date */
		HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
		/* Get the current time */
		HAL_RTC_GetTime(&hrtc, &RTC_TimeStructureNext, RTC_FORMAT_BIN);
		/* Get the current date */
		HAL_RTC_GetDate(&hrtc, &RTC_DateStructNext, RTC_FORMAT_BIN);
	} while (RTC_TimeStructure.SubSeconds != RTC_TimeStructureNext.SubSeconds);

	date.tm_hour  = RTC_TimeStructure.Hours;
	date.tm_min   = RTC_TimeStructure.Minutes;
	date.tm_sec   = RTC_TimeStructure.Seconds;
	date.tm_year  = RTC_DateStruct.Year  + ( 2000 - 1900 );
	date.tm_mon   = RTC_DateStruct.Month - 1;
	date.tm_mday  = RTC_DateStruct.Date;
	date.tm_wday  = RTC_DateStruct.WeekDay;
	date.tm_isdst = -1;

	time_in_secs     = mktime(&date);
	uint32_t secfrac = (((RTC_TimeStructure.SecondFraction - RTC_TimeStructure.SubSeconds) * 1000UL) / (RTC_TimeStructure.SecondFraction + 1));
	time_in_milisecs = time_in_secs * 1000 + secfrac;

	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Time in miliseconds: %d:%d:%d.%d\r\n",
			(int)time_in_secs, (int)date.tm_hour, (int)date.tm_min, (int)date.tm_sec, (int)secfrac);

	return (time_in_milisecs);
}

static void __setCreatedMessageTime( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct )
{
	Sys_time.created_time_telegram[0]  = '2';
	Sys_time.created_time_telegram[1]  = '0';
	Sys_time.created_time_telegram[2]  = ((DEC_TO_BCD(RTC_DateStruct.Year)>>4)       & 0x0F)+'0';
	Sys_time.created_time_telegram[3]  = ((DEC_TO_BCD(RTC_DateStruct.Year)   )       & 0x0F)+'0';
	Sys_time.created_time_telegram[4]  = ((DEC_TO_BCD(RTC_DateStruct.Month)>>4)      & 0x0F)+'0';
	Sys_time.created_time_telegram[5]  = ((DEC_TO_BCD(RTC_DateStruct.Month)   )      & 0x0F)+'0';
	Sys_time.created_time_telegram[6]  = ((DEC_TO_BCD(RTC_DateStruct.Date)>>4)       & 0x0F)+'0';
	Sys_time.created_time_telegram[7]  = ((DEC_TO_BCD(RTC_DateStruct.Date)   )       & 0x0F)+'0';
	Sys_time.created_time_telegram[8]  = ((DEC_TO_BCD(RTC_TimeStructure.Hours)>>4)   & 0x0F)+'0';
	Sys_time.created_time_telegram[9]  = ((DEC_TO_BCD(RTC_TimeStructure.Hours)   )   & 0x0F)+'0';
	Sys_time.created_time_telegram[10] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)>>4) & 0x0F)+'0';
	Sys_time.created_time_telegram[11] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)   ) & 0x0F)+'0';
	Sys_time.created_time_telegram[12] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)>>4) & 0x0F)+'0';
	Sys_time.created_time_telegram[13] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)   ) & 0x0F)+'0';
	Sys_time.created_time_telegram[14] = '\0';
}

void rtc_system_SetCreatedMessageTime( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    = date->tm_mday;
    RTC_DateStruct.Month   = date->tm_mon + 1;
    RTC_DateStruct.WeekDay = date->tm_wday + 1;
    RTC_DateStruct.Year    = date->tm_year - ( 2000 - 1900 );

    __setCreatedMessageTime(RTC_TimeStructure, RTC_DateStruct);
}

char * rtc_system_getCreatedMessageTime( void )
{
	return Sys_time.created_time_telegram;
}

static void __setCreatedMeterValueTime( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct )
{
	Sys_time.created_meter_value[0] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)>>4)   & 0x0F)+'0';
	Sys_time.created_meter_value[1] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)   )   & 0x0F)+'0';
	Sys_time.created_meter_value[2] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)>>4) & 0x0F)+'0';
	Sys_time.created_meter_value[3] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)   ) & 0x0F)+'0';
	Sys_time.created_meter_value[4] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)>>4) & 0x0F)+'0';
	Sys_time.created_meter_value[5] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)   ) & 0x0F)+'0';
	Sys_time.created_meter_value[6] = '\0';
}

void rtc_system_SetCreatedMeterValueTime( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - ( 2000 - 1900 );

    __setCreatedMeterValueTime(RTC_TimeStructure, RTC_DateStruct);
}

char * rtc_system_getCreatedMeterValueTime( void )
{
	return Sys_time.created_meter_value;
}

static void __setCreatedMeterValueDate( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct )
{
	Sys_time.created_meter_value_date[0] = '2';
	Sys_time.created_meter_value_date[1] = '0';
	Sys_time.created_meter_value_date[2] = ((DEC_TO_BCD(RTC_DateStruct.Year)>>4)  & 0x0F) + '0';
	Sys_time.created_meter_value_date[3] = ((DEC_TO_BCD(RTC_DateStruct.Year)   )  & 0x0F) + '0';
	Sys_time.created_meter_value_date[4] = ((DEC_TO_BCD(RTC_DateStruct.Month)>>4) & 0x0F) + '0';
	Sys_time.created_meter_value_date[5] = ((DEC_TO_BCD(RTC_DateStruct.Month)   ) & 0x0F) + '0';
	Sys_time.created_meter_value_date[6] = ((DEC_TO_BCD(RTC_DateStruct.Date)>>4)  & 0x0F) + '0';
	Sys_time.created_meter_value_date[7] = ((DEC_TO_BCD(RTC_DateStruct.Date)   )  & 0x0F) + '0';
	Sys_time.created_meter_value_date[8] = '\0';
}

void rtc_system_SetCreatedMeterValueDate( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - ( 2000 - 1900 );

    __setCreatedMeterValueDate(RTC_TimeStructure, RTC_DateStruct);
}

char * rtc_system_getCreatedMeterValueDate( void )
{
	return Sys_time.created_meter_value_date;
}

/**
  * @brief Copy to Sys_time.created_sensor_value_time field the actual time given by RTC_TimeStructure
  * @param RTC_TimeStructure: RTC structure that holds current time
  * @retval None
  */
static void __setCreatedSensorValueTime(RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct)
{
	Sys_time.created_sensor_value[0] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)>>4) & 0x0F) + '0';
	Sys_time.created_sensor_value[1] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)) & 0x0F) + '0';
	Sys_time.created_sensor_value[2] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)>>4) & 0x0F) + '0';
	Sys_time.created_sensor_value[3] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)) & 0x0F) + '0';
	Sys_time.created_sensor_value[4] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)>>4) & 0x0F) + '0';
	Sys_time.created_sensor_value[5] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)) & 0x0F) + '0';
	Sys_time.created_sensor_value[6] = '\0';
}

/**
  * @brief Set a time stamp mark for sensor measure stored in Sys_time.created_sensor_value
  * @param server_time: Current system time
  */
void rtc_system_SetCreatedSensorValueTime(time_t server_time)
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	/** Dates
	 * */
	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - (2000 - 1900);

    /**
     *       */
    __setCreatedSensorValueTime(RTC_TimeStructure, RTC_DateStruct);
}

/**
  * @brief Return a pointer to the first element of Sys_time.created_sensor_value
  * @retval Pointer to the first element of Sys_time.created_sensor_value
  */
char * rtc_system_getCreatedSensorValueTime(void)
{
	return Sys_time.created_sensor_value;
}

/**
  * @brief Copy to Sys_time.created_sensor_value_date field the actual time given by RTC_DateStructure
  * @param RTC_DateStructure: RTC structure that holds current date
  * @retval None
  */
static void __setCreatedSensorValueDate(RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct)
{
	Sys_time.created_sensor_value_date[0] = '2';
	Sys_time.created_sensor_value_date[1] = '0';
	Sys_time.created_sensor_value_date[2] = ((DEC_TO_BCD(RTC_DateStruct.Year)>>4) & 0x0F) + '0';
	Sys_time.created_sensor_value_date[3] = ((DEC_TO_BCD(RTC_DateStruct.Year)) & 0x0F) + '0';
	Sys_time.created_sensor_value_date[4] = ((DEC_TO_BCD(RTC_DateStruct.Month)>>4) & 0x0F) + '0';
	Sys_time.created_sensor_value_date[5] = ((DEC_TO_BCD(RTC_DateStruct.Month)) & 0x0F) + '0';
	Sys_time.created_sensor_value_date[6] = ((DEC_TO_BCD(RTC_DateStruct.Date)>>4) & 0x0F) + '0';
	Sys_time.created_sensor_value_date[7] = ((DEC_TO_BCD(RTC_DateStruct.Date)) & 0x0F) + '0';
	Sys_time.created_sensor_value_date[8] = '\0';
}

/**
  * @brief
  * @retval
  */
char * rtc_system_getCreatedSensorValueTimeFromFirstMeas(time_t first_value, uint32_t msg_num)
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;
	time_t           measure_time;

	if ( 0 == params_get_pulse_acc() )
	{
		measure_time = first_value + STAND_BY_TIME * (msg_num - 1);//params_sensor_read_time_cycle(0)
	}
	else if ( 1 == params_get_pulse_acc() )
	{
		measure_time = first_value + params_config_read_time() * (msg_num - 1);//900
	}

	/** Dates
	 * */
	date = gmtime(&measure_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - (2000 - 1900);

    /**
     *       */
    __setCreatedSensorValueTime(RTC_TimeStructure, RTC_DateStruct);
    __setCreatedSensorValueDate(RTC_TimeStructure, RTC_DateStruct);
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Time Created Message: %d:%d:%d msg_num:%d\r\n",
			(int)Tick_Get(SECONDS), (int)RTC_TimeStructure.Hours, (int)RTC_TimeStructure.Minutes, (int)RTC_TimeStructure.Seconds, (int)msg_num);
	return Sys_time.created_sensor_value;
}

/**
  * @brief Set a date stamp mark for sensor measure stored in Sys_time.created_sensor_value_date
  * @param server_time: Current system time
  * @retval None
  */
void rtc_system_SetCreatedSensorValueDate(time_t server_time)
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday;
    RTC_DateStruct.Year    		 = date->tm_year - ( 2000 - 1900 );

    __setCreatedSensorValueDate(RTC_TimeStructure, RTC_DateStruct);
}

/**
  * @brief Return a pointer to the first element of Sys_time.created_sensor_value_date
  * @retval Pointer to the first element of Sys_time.created_sensor_date
  */
char * rtc_system_getCreatedSensorValueDate( void )
{
	return Sys_time.created_sensor_value_date;
}

static void __setCreatedModbusValueTime( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct )
{
	Sys_time.created_modbus_value[0] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)>>4)   & 0x0F)+'0';
	Sys_time.created_modbus_value[1] = ((DEC_TO_BCD(RTC_TimeStructure.Hours)   )   & 0x0F)+'0';
	Sys_time.created_modbus_value[2] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)>>4) & 0x0F)+'0';
	Sys_time.created_modbus_value[3] = ((DEC_TO_BCD(RTC_TimeStructure.Minutes)   ) & 0x0F)+'0';
	Sys_time.created_modbus_value[4] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)>>4) & 0x0F)+'0';
	Sys_time.created_modbus_value[5] = ((DEC_TO_BCD(RTC_TimeStructure.Seconds)   ) & 0x0F)+'0';
	Sys_time.created_modbus_value[6] = '\0';
}

void rtc_system_SetCreatedModbusValueTime( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - ( 2000 - 1900 );

    __setCreatedModbusValueTime(RTC_TimeStructure, RTC_DateStruct);
}

char * rtc_system_getCreatedModbusValueTime( void )
{
	return Sys_time.created_modbus_value;
}

static void __setCreatedModbusValueDate( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct )
{
	Sys_time.created_modbus_value_date[0] = '2';
	Sys_time.created_modbus_value_date[1] = '0';
	Sys_time.created_modbus_value_date[2] = ((DEC_TO_BCD(RTC_DateStruct.Year)>>4)  & 0x0F) + '0';
	Sys_time.created_modbus_value_date[3] = ((DEC_TO_BCD(RTC_DateStruct.Year)   )  & 0x0F) + '0';
	Sys_time.created_modbus_value_date[4] = ((DEC_TO_BCD(RTC_DateStruct.Month)>>4) & 0x0F) + '0';
	Sys_time.created_modbus_value_date[5] = ((DEC_TO_BCD(RTC_DateStruct.Month)   ) & 0x0F) + '0';
	Sys_time.created_modbus_value_date[6] = ((DEC_TO_BCD(RTC_DateStruct.Date)>>4)  & 0x0F) + '0';
	Sys_time.created_modbus_value_date[7] = ((DEC_TO_BCD(RTC_DateStruct.Date)   )  & 0x0F) + '0';
	Sys_time.created_modbus_value_date[8] = '\0';
}

void rtc_system_SetCreatedModbusValueDate( time_t server_time )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	struct tm       *date;

	date = gmtime(&server_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    		 = date->tm_mday;
    RTC_DateStruct.Month   		 = date->tm_mon + 1;
    RTC_DateStruct.WeekDay 		 = date->tm_wday + 1;
    RTC_DateStruct.Year    		 = date->tm_year - ( 2000 - 1900 );

    __setCreatedModbusValueDate(RTC_TimeStructure, RTC_DateStruct);
}

char * rtc_system_getCreatedModbusValueDate( void )
{
	return Sys_time.created_modbus_value_date;
}
int8_t rtc_system_localUTCTime( uint32_t *hour )
{
	int8_t ret = 0;
	time_t UTC_time;
	int hour_aux = *hour;

	UTC_time = Tick_Get( SECONDS );
    if( UTC_time < 0x54C8F21F ) {
        return 0;    // No he obtenido la hora por SNTP a�n...
    }

    hour_aux = hour_aux + 4;//EUA:GMT+4, Spain:GMT+2.

    if( hour_aux > 23 ) {
    	hour_aux = hour_aux - 24;
    	ret = 1;
    }
    else if( hour_aux < 0 ) {
    	hour_aux = 24 + hour_aux;
    	ret = -1;
    }

    *hour = hour_aux;

    return ret;
}

int8_t rtc_system_UTCTime( uint32_t *hour )
{
	int8_t ret = 0;
	time_t UTC_time;
	int hour_aux = *hour;

	UTC_time = Tick_Get( SECONDS );
    if( UTC_time < 0x54C8F21F ) {
        return 0;    // No he obtenido la hora por SNTP a�n...
    }

    if( hour_aux > 23 ) {
    	hour_aux = hour_aux - 24;
    	ret = 1;
    }
    else if( hour_aux < 0 ) {
    	hour_aux = 24 + hour_aux;
    	ret = -1;
    }

    *hour = hour_aux;

    return ret;
}

/**
 * @fn void rtc_system_setCurrentAlarm(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _current_alarm
 */
void rtc_system_setCurrentAlarm(uint32_t _current_alarm)
{
	rtc_Alarms.currentAlarm = (current_alarm)_current_alarm;
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, rtc_Alarms.currentAlarm);
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Current Alarm:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)rtc_Alarms.currentAlarm);
}

uint32_t rtc_system_getCurrentAlarmFromBKUP( void )
{
	rtc_Alarms.currentAlarm = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5);

	return rtc_Alarms.currentAlarm;
}

uint32_t rtc_system_getCurrentAlarm( void )
{
	return rtc_Alarms.currentAlarm;
}

void rtc_system_setCurrentSensorAlarm( uint32_t _current_alarm )
{
	rtc_Alarms.currentSensorAlarm = (current_alarm)_current_alarm;
//	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, rtc_Alarms.currentAlarm);
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Current Sensor Alarm:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)rtc_Alarms.currentSensorAlarm);
}

uint32_t rtc_system_getCurrentSensorAlarm( void )
{
	return rtc_Alarms.currentSensorAlarm;
}

void rtc_system_setCurrentModbusAlarm( uint32_t _current_alarm )
{
	rtc_Alarms.currentModbusAlarm = (current_alarm)_current_alarm;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Current Modbus Alarm:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)rtc_Alarms.currentModbusAlarm);
}

uint32_t rtc_system_getCurrentModbusAlarm( void )
{
	return rtc_Alarms.currentModbusAlarm;
}

uint32_t rtc_system_getReadInitAlarmFromBKUP( void )
{
	rtc_Alarms.readInitAlarmNext = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR6);

	return rtc_Alarms.readInitAlarmNext;
}

uint32_t rtc_system_getReadInitAlarm( void )
{
	return rtc_Alarms.readInitAlarmNext;
}

uint32_t rtc_system_getReadEndAlarmFromBKUP( void )
{
	rtc_Alarms.readEndAlarmNext = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR7);

	return rtc_Alarms.readEndAlarmNext;
}

uint32_t rtc_system_getReadEndAlarm( void )
{
	return rtc_Alarms.readEndAlarmNext;
}

uint32_t rtc_system_getSendAlarmFromBKUP( void )
{
	rtc_Alarms.sendAlarmNext = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR8);

	return rtc_Alarms.sendAlarmNext;
}

uint32_t rtc_system_getSendAlarm( void )
{
	return rtc_Alarms.sendAlarmNext;
}

uint32_t rtc_system_getCurrentParamsAlarm( void )
{
	return rtc_Alarms.currentParamsAlarm;
}

void rtc_system_setCurrentParamsAlarm( uint32_t _current_alarm )
{
	rtc_Alarms.currentParamsAlarm = (current_params_alarm)_current_alarm;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Current Params Alarm:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)rtc_Alarms.currentParamsAlarm);
}

static uint32_t __getNextAlarmInitRead( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;
//	RTC_DateTypeDef  RTC_DateStruct;

	if ( 0 == rtc_Alarms.readAlarmsNum ) {
		rtc_Alarms.readAlarmsNum = READ_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
//	/* Get the current date */
//	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.readAlarmsNum; i++ ) {
		if ( params_read_time_init_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.readAlarmsNum;
		}
		else if ( params_read_time_init_time( i ) > rtc_hours ) {
			set_hour = ret = params_read_time_init_time( i );
			rtc_system_setReadAlarmCycleNext(params_read_time_cycle( i ));
			i   = rtc_Alarms.readAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_read_time_init_time( 0 ) != 255 ) {
			set_hour = ret = params_read_time_init_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_read_time_init_time( 0 ) != 0xFF ) {
			ret = 24 + params_read_time_init_time( 0 );
		}
	}
	rtc_system_setReadAlarmInitNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

static uint32_t __getNextAlarmEndRead( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;
//	RTC_DateTypeDef  RTC_DateStruct;

	if ( 0 == rtc_Alarms.readAlarmsNum ) {
		rtc_Alarms.readAlarmsNum = READ_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
//	/* Get the current date */
//	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.readAlarmsNum; i++ ) {
		if ( params_read_time_end_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.readAlarmsNum;
		}
		else if ( params_read_time_end_time( i ) > rtc_hours ) {
			set_hour = ret = params_read_time_end_time( i );
			rtc_system_setReadAlarmCycleNext(params_read_time_cycle( i ));
			i   = rtc_Alarms.readAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_read_time_end_time( 0 ) != 255 ) {
			set_hour = ret = params_read_time_end_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_read_time_end_time( 0 ) != 0xFF ) {
			ret = 24 + params_read_time_end_time( 0 );
		}
	}
	rtc_system_setReadAlarmEndNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

uint32_t start_fast_reading = 0, start_fast_reading_sensor = 0;
static uint32_t __getNextSensorAlarmInitRead( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef RTC_DateStructure  )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;

	if ( 0 == rtc_Alarms.readSensorAlarmsNum )
	{
		rtc_Alarms.readSensorAlarmsNum = READ_WINDOWS;
	}

	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < NUM_INPUTS_SENSOR; i++ )
	{
		if ( 1 == generic_sensor_get_burst_mode(i))
		{
			start_fast_reading_sensor = 1;
			i = NUM_INPUTS_SENSOR;
		}
		else
		{
			start_fast_reading_sensor = 0;
		}
	}
	for ( int i = 0; i < rtc_Alarms.readSensorAlarmsNum; i++ ) {
		if (( (params_sensor_read_time_init_time( i ) == rtc_hours)
		  &&(RTC_DateStructure.Date                 == params_sensor_read_time_date_day(i))
		  &&(RTC_DateStructure.Month                == params_sensor_read_time_date_month(i)) )
		)
		{
			start_fast_reading = 1;
		}
		if ( params_sensor_read_time_init_time( i ) == (unsigned int)(-1) )
		{
			i   = rtc_Alarms.readSensorAlarmsNum;
		}
		else if (((params_sensor_read_time_init_time( i ) == rtc_hours)&&(RTC_DateStructure.Date == params_sensor_read_time_date_day(i))&&(RTC_DateStructure.Month == params_sensor_read_time_date_month(i)))
			   || ((params_sensor_read_time_init_time( i ) > rtc_hours )&&(0xFF == params_sensor_read_time_date_day(i)))
			   || (1==start_fast_reading)
				)
		{
			set_hour = ret = params_sensor_read_time_init_time( i );
			rtc_system_setReadSensorAlarmCycleNext(params_sensor_read_time_cycle( i ));
			i   = rtc_Alarms.readSensorAlarmsNum;
		}
		else if (1==start_fast_reading_sensor)
		{
			set_hour = ret = rtc_hours;
			rtc_system_setReadSensorAlarmCycleNext(params_sensor_read_time_cycle( i ));
			i   = rtc_Alarms.readSensorAlarmsNum;
		}
	}

	if ( 24 == ret )
	{
		set_hour = 0;
	} else if ( 255 == ret )
	{
		if ( params_sensor_read_time_init_time( 0 ) != 255 )
		{
			set_hour = ret = params_sensor_read_time_init_time( 0 );
		}
	} else if ( 0 == ret )
	{
		if ( params_sensor_read_time_init_time( 0 ) != 0xFF )
		{
			ret = 24 + params_sensor_read_time_init_time( 0 );
		}
	}
	rtc_system_setReadSensorAlarmInitNext(set_hour);

	if ( ret >= 0 && ret <= 23 )
	{
		diff_aux = ret - diff;
		if (diff_aux < 0)
		{
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 )
		{
			ret = diff_aux - 24;
		}
		else
		{
			ret = diff_aux;
		}
	}
	return ret;
}

static uint32_t __getNextSensorAlarmEndRead( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;

	if ( 0 == rtc_Alarms.readSensorAlarmsNum )
	{
		rtc_Alarms.readSensorAlarmsNum = READ_WINDOWS;
	}

	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.readSensorAlarmsNum; i++ )
	{
		if (((params_sensor_read_time_end_time( i ) < rtc_hours)
		  &&(RTC_DateStructure.Date                 == params_sensor_read_time_date_day(i))
		  &&(RTC_DateStructure.Month                == params_sensor_read_time_date_month(i))))
		{
			start_fast_reading = 0;
//			for ( int i = 0; i < NUM_INPUTS_SENSOR; i++ ) {
//				if ( 1 == generic_sensor_get_burst_mode(i))
//				{
//					start_fast_reading = 1;
//					i = NUM_INPUTS_SENSOR;
//				}
//			}
		}
		if ( params_sensor_read_time_end_time( i ) == (unsigned int)(-1) )
		{
			i   = rtc_Alarms.readSensorAlarmsNum;
		}
		else if (((params_sensor_read_time_end_time( i ) > rtc_hours)&&(params_sensor_read_time_init_time( i ) <= rtc_hours)&&(RTC_DateStructure.Date == params_sensor_read_time_date_day(i))&&(RTC_DateStructure.Month == params_sensor_read_time_date_month(i)))
				|| (( params_sensor_read_time_end_time( i ) > rtc_hours )&&(0xFF == params_sensor_read_time_date_day(i)))
				|| (1==start_fast_reading)
				)
		{
			set_hour = ret = params_sensor_read_time_end_time( i );
			rtc_system_setReadSensorAlarmCycleNext(params_sensor_read_time_cycle( i ));
			i   = rtc_Alarms.readSensorAlarmsNum;
		}
		else if (1==start_fast_reading_sensor)
		{
			set_hour = ret = rtc_hours;
			rtc_system_setReadSensorAlarmCycleNext(params_sensor_read_time_cycle( i ));
			i   = rtc_Alarms.readSensorAlarmsNum;
		}
	}

	if ( 24 == ret )
	{
		set_hour = 0;
	} else if ( 255 == ret )
	{
		if ( params_sensor_read_time_end_time( 0 ) != 255 )
		{
			set_hour = ret = params_sensor_read_time_end_time( 0 );
		}
	} else if ( 0 == ret )
	{
		if ( params_sensor_read_time_end_time( 0 ) != 0xFF )
		{
			ret = 24 + params_sensor_read_time_end_time( 0 );
		}
	}
	rtc_system_setReadSensorAlarmEndNext(set_hour);

	if ( ret >= 0 && ret <= 23 )
	{
		diff_aux = ret - diff;
		if (diff_aux < 0)
		{
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 )
		{
			ret = diff_aux - 24;
		}
		else
		{
			ret = diff_aux;
		}
	}
	return ret;
}

static uint32_t __getNextModbusAlarmInitRead( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;
//	RTC_DateTypeDef  RTC_DateStruct;

	if ( 0 == rtc_Alarms.readModbusAlarmsNum ) {
		rtc_Alarms.readModbusAlarmsNum = READ_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
//	/* Get the current date */
//	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.readModbusAlarmsNum; i++ ) {
		if ( params_modbus_read_time_init_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.readModbusAlarmsNum;
		}
		else if ( params_modbus_read_time_init_time( i ) > rtc_hours ) {
			set_hour = ret = params_modbus_read_time_init_time( i );
			rtc_system_setReadModbusAlarmCycleNext(params_modbus_read_time_cycle( i ));
			i   = rtc_Alarms.readModbusAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_modbus_read_time_init_time( 0 ) != 255 ) {
			set_hour = ret = params_modbus_read_time_init_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_modbus_read_time_init_time( 0 ) != 0xFF ) {
			ret = 24 + params_modbus_read_time_init_time( 0 );
		}
	}
	rtc_system_setReadModbusAlarmInitNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

static uint32_t __getNextModbusAlarmEndRead( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;
//	RTC_DateTypeDef  RTC_DateStruct;

	if ( 0 == rtc_Alarms.readModbusAlarmsNum ) {
		rtc_Alarms.readModbusAlarmsNum = READ_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
//	/* Get the current date */
//	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.readModbusAlarmsNum; i++ ) {
		if ( params_modbus_read_time_end_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.readModbusAlarmsNum;
		}
		else if ( params_modbus_read_time_end_time( i ) > rtc_hours ) {
			set_hour = ret = params_modbus_read_time_end_time( i );
			rtc_system_setReadModbusAlarmCycleNext(params_modbus_read_time_cycle( i ));
			i   = rtc_Alarms.readModbusAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_modbus_read_time_end_time( 0 ) != 255 ) {
			set_hour = ret = params_modbus_read_time_end_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_modbus_read_time_end_time( 0 ) != 0xFF ) {
			ret = 24 + params_modbus_read_time_end_time( 0 );
		}
	}
	rtc_system_setReadModbusAlarmEndNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

int32_t rtc_system_SetReadTimeInitTimeAlarm( void )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour = __getNextAlarmInitRead(RTC_TimeStructure);
	if (hour == 0) {
		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
	}
	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_NONE;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = 0;
		RTC_AlarmStructure.AlarmTime.Seconds   = 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else {
		return -1;
	}
}

int32_t rtc_system_SetReadTimeEndTimeAlarm( void )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour = __getNextAlarmEndRead(RTC_TimeStructure);
	if (hour == 0) {
		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
	}
	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_NONE;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = 0;
		RTC_AlarmStructure.AlarmTime.Seconds   = 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else {
		return -1;
	}
}

static uint32_t __getNextAlarmInitSend( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;
//	RTC_DateTypeDef  RTC_DateStruct;

	if ( 0 == rtc_Alarms.sendAlarmsNum ) {
		rtc_Alarms.sendAlarmsNum = SEND_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
//	/* Get the current date */
//	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.sendAlarmsNum; i++ ) {
		if ( params_send_time_send_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.sendAlarmsNum;
		}
		else if ( params_send_time_send_time( i ) > rtc_hours ) {
			set_hour = ret = params_send_time_send_time( i );
			i   = rtc_Alarms.sendAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_send_time_send_time( 0 ) != 255 ) {
			set_hour = ret = params_send_time_send_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_send_time_send_time( 0 ) != 0xFF ) {
			ret = 24 + params_send_time_send_time( 0 );
		}
	}
	rtc_system_setSendAlarmSendNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

int32_t rtc_system_SetSendTimeInitTimeAlarm( void )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour = __getNextAlarmInitSend(RTC_TimeStructure);
	if (hour == 0) {
		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
	}
	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_NONE;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = 0;
		RTC_AlarmStructure.AlarmTime.Seconds   = 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else {
		return -1;
	}
}

static uint32_t __getNextAlarmInitSendParams( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;

	if ( 0 == rtc_Alarms.sendParamsAlarmsNum ) {
		rtc_Alarms.sendParamsAlarmsNum = SEND_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.sendParamsAlarmsNum; i++ ) {
		if ( params_send_network_params_time_send_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.sendParamsAlarmsNum;
		}
		else if ( params_send_network_params_time_send_time( i ) > rtc_hours ) {
			set_hour = ret = params_send_network_params_time_send_time( i );
			i   = rtc_Alarms.sendParamsAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_send_network_params_time_send_time( 0 ) != 255 ) {
			set_hour = ret = params_send_network_params_time_send_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_send_network_params_time_send_time( 0 ) != 0xFF ) {
			ret = 24 + params_send_network_params_time_send_time( 0 );
		}
	}
	rtc_system_setSendParamsAlarmSendNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

int32_t rtc_system_SetSendParamsTimeInitTimeAlarm( void )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour = __getNextAlarmInitSend(RTC_TimeStructure);
	if (hour == 0) {
		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
	}
	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_NONE;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = 0;
		RTC_AlarmStructure.AlarmTime.Seconds   = 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else {
		return -1;
	}
}

static uint32_t __getNextSensorAlarmInitSend( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;

	if ( 0 == rtc_Alarms.sendSensorAlarmsNum ) {
		rtc_Alarms.sendSensorAlarmsNum = SEND_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.sendSensorAlarmsNum; i++ ) {
		if ( params_sensor_send_time_send_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.sendSensorAlarmsNum;
		}
		else if ( params_sensor_send_time_send_time( i ) > rtc_hours ) {
			set_hour = ret = params_sensor_send_time_send_time( i );
			i   = rtc_Alarms.sendSensorAlarmsNum;
		}
		else if ( 1 == start_fast_reading_sensor ) {
			set_hour = ret = rtc_hours + 1;
			i   = rtc_Alarms.sendSensorAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_sensor_send_time_send_time( 0 ) != 255 ) {
			set_hour = ret = params_sensor_send_time_send_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_sensor_send_time_send_time( 0 ) != 0xFF ) {
			ret = 24 + params_sensor_send_time_send_time( 0 );
		}
	}
	rtc_system_setSendSensorAlarmSendNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}
static uint32_t __getNextModbusAlarmInitSend( RTC_TimeTypeDef  RTC_TimeStructure )
{
	uint32_t         ret = 0xFF, rtc_hours, rtc_utc_hours, set_hour = 0;
	int8_t           diff, diff_aux;
//	RTC_TimeTypeDef  RTC_TimeStructure;

	if ( 0 == rtc_Alarms.sendModbusAlarmsNum ) {
		rtc_Alarms.sendModbusAlarmsNum = SEND_WINDOWS;
	}

//	/* Get the current time */
//	HAL_RTC_GetTime( &hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN );
	rtc_utc_hours = rtc_hours = RTC_TimeStructure.Hours;

	rtc_system_UTCTime(&rtc_hours);

	diff = rtc_hours - rtc_utc_hours;

	for ( int i = 0; i < rtc_Alarms.sendModbusAlarmsNum; i++ ) {
		if ( params_modbus_send_time_send_time( i ) == (unsigned int)(-1) ) {
			i   = rtc_Alarms.sendModbusAlarmsNum;
		}
		else if ( params_modbus_send_time_send_time( i ) > rtc_hours ) {
			set_hour = ret = params_modbus_send_time_send_time( i );
			i   = rtc_Alarms.sendModbusAlarmsNum;
		}
	}

	if ( 24 == ret ) {
		set_hour = 0;
	} else if ( 255 == ret ) {
		if ( params_modbus_send_time_send_time( 0 ) != 255 ) {
			set_hour = ret = params_modbus_send_time_send_time( 0 );
		}
	} else if ( 0 == ret ) {
		if ( params_modbus_send_time_send_time( 0 ) != 0xFF ) {
			ret = 24 + params_modbus_send_time_send_time( 0 );
		}
	}
	rtc_system_setSendModbusAlarmSendNext(set_hour);

	if ( ret >= 0 && ret <= 23 ) {
		diff_aux = ret - diff;
		if (diff_aux < 0) {
			ret = 24 + diff_aux;
		}
		else if ( diff_aux > 23 ) {
			ret = diff_aux - 24;
		}
		else {
			ret = diff_aux;
		}
	}
	return ret;
}

int32_t rtc_system_SetSensorSendTimeInitTimeAlarm( void )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour = __getNextSensorAlarmInitSend(RTC_TimeStructure);
	if (hour == 0) {
		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
	}
	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_NONE;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = 0;
		RTC_AlarmStructure.AlarmTime.Seconds   = 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else {
		return -1;
	}
}

void rtc_System_InitRemainingTime( void )
{
	static uint32_t init = 0;
	if ( 1 == params_pulses_on() ) {
		if ( 0 == init ) {
			init = 1;
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR18, Tick_Get(SECONDS) );
		}
	}
}

uint32_t timeout_waiting_pulse = 0;
uint32_t rtc_system_set_timeout_waiting_pulse( void )
{
	timeout_waiting_pulse = 1;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> TIMEOUT WAITING PULSE!!!\r\n",
			(int)Tick_Get( SECONDS ) );
	return 0;
}

uint32_t rtc_system_reset_timeout_waiting_pulse( void )
{
	timeout_waiting_pulse = 0;
//	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RESET TIMEOUT WAITING PULSE\r\n",
//			(int)Tick_Get( SECONDS ) );
	return 0;
}

uint32_t rtc_system_get_timeout_waiting_pulse( void )
{
	return timeout_waiting_pulse;
}

time_t   previous_time = 0;
uint32_t remaining_time = 0;
/**
 * @fn uint32_t __getRemainingWakeUpTime(time_t)
 * @brief
 *
 * @pre
 * @post
 * @param current_time
 * @return
 */
static uint32_t __getRemainingWakeUpTime(time_t current_time)
{
	static uint32_t init_previous_time = 0;
	        int32_t rem_time     = 0;

	        uint32_t stand_by_time = STAND_BY_TIME_MS;
//	current_time = current_time * 1000 + Tick_Get(MILLISECONDS);//HAL_GetTick();
//	current_time = current_time + Tick_Get(MILLISECONDS);//HAL_GetTick();
	current_time += Tick_Get(MILLISECONDS);
	/* Gets here the first time and initializes previous time */
	if (0 == init_previous_time)
	{
		previous_time      = current_time;
		init_previous_time = 1;
	}

	/* Woken up by a pulse */
	if ((1 == params_pulses_on()) /*&& (1 == pulses_get_pulse_rx())*/ && (1 == Tick_cloud_time_init()))
	{
		/* Clear pulse interrupt flag (this is set in the pulse pulses_ch2p_handler)*/
		pulses_set_interruption(0);

		/* */
		if (previous_time > current_time)
		{
			previous_time = current_time;
		}

		if ( (1 == pulses_get_pulse_rx()) && (0 == params_get_pulse_write()) && (0 == timeout_waiting_pulse) )
		{
			/* time elapsed since last time it went to sleep */
//			rem_time = current_time - previous_time;
//			rem_time = current_time - time_in_miliseconds_before_low_power - 1000;
			rem_time = time_in_miliseconds_after_low_power + Tick_Get(MILLISECONDS) - time_in_miliseconds_before_low_power;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> GET PULSE:1 time btw pulse:%d time_in_miliseconds_after_low_power:%d. time_in_miliseconds_before_low_power:%d. calib_period:%d t_ms:%d\r\n",
					(int)Tick_Get( SECONDS ), (int)rem_time, (int)time_in_miliseconds_after_low_power, (int)time_in_miliseconds_before_low_power, (int)calib_period, (int)Tick_Get(MILLISECONDS) );
		}
		else
		{
//			rem_time = current_time - previous_time - res_cycle_time_prev;// - 1000;
//			rem_time = current_time - time_in_miliseconds_before_low_power - res_cycle_time_prev - 1000;
			rem_time = time_in_miliseconds_after_low_power + Tick_Get(MILLISECONDS) - time_in_miliseconds_before_low_power - res_cycle_time_prev;
			if ( rem_time > stand_by_time/*STAND_BY_TIME_MS*/ )
			{
				rem_time = 0;
			}
			params_set_pulse_write(0);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> GET PULSE:0 time btw pulse:%d res_cycle_time_prev:%d calib_period:%d t_ms:%d\r\n",
					(int)Tick_Get( SECONDS ), (int)rem_time, (int)res_cycle_time_prev, (int)calib_period, (int)Tick_Get(MILLISECONDS) );
		}
		timeout_waiting_pulse = 0;
		if ( rem_time >= 0 )
		{
			odd_time     = (((rem_time % 1000) < 900) && ((rem_time % 1000) > 100))?1:0;
			calib_period = (stand_by_time/*STAND_BY_TIME_MS*//(rem_time + 1000) + 1);
		}
		else
		{
			odd_time     = 0;
			calib_period = (30 + 1);
		}
		/* if no time has elapsed ... TOASK adjust time??*/
		if (rem_time < 0)
		{
			rem_time = 0;//1000;
		}
		//DELETE
//		if (( 0 == rem_time) && (1 == pulses_get_pulse_rx()))
//		{
//			rem_time = 400;
//		}
		time_btw_pulses = rem_time;
	}
//	else
//	{
//		rem_time = 2;//0;
//	}

	/* updates time to be compared next time */
	previous_time = current_time;
	return rem_time;
}

static void __calculateAlarmDate( uint32_t sleeptime, uint32_t *seconds, uint32_t *minutes, uint32_t *hours, uint32_t *date )
{
	uint32_t sec = 0, min = 0, hour = 0;

	hour   = sleeptime/3600;
	*hours = *hours + hour;
	if ( *hours >= 24 ) {
		*hours = *hours % 24;
		* date = 1;
	}
	if ( hour == 0 ) {
		min      = sleeptime/60;
		*minutes = *minutes + min;
		if ( min == 0 ) {
			sec = sleeptime;
			*seconds = *seconds + sec;
			if ( *seconds >= 60 ) {
				*seconds = *seconds % 60;
				*minutes = *minutes + 1;
				if ( *minutes >= 60 ) {
					*minutes = *minutes % 60;
					*hours   = *hours + 1;
					if ( *hours >= 24 ) {
						*hours = *hours % 24;
						*date  = 1;
					}
				}
			}
		} else if ( *minutes >= 60 ) {
			*minutes = *minutes % 60;
			*hours   = *hours + 1;
			if ( *hours >= 24 ) {
				*hours = *hours % 24;
				*date  = 1;
			}
		}
	}
}

uint32_t __getCurrentAlarm( RTC_TimeTypeDef  RTC_TimeStructure, RTC_DateTypeDef  RTC_DateStruct  )
{
	uint32_t hour, hour_sensor, hour_modbus, hour_param = 0,
	         hour_init_read, hour_end_read, hour_send,
			 hour_params_send,
			 hour_sensor_init_read, hour_sensor_end_read, hour_sensor_send,
			 hour_modbus_init_read, hour_modbus_end_read, hour_modbus_send;
	uint32_t hour_array[3], hour_sensor_array[3], hour_modbus_array[3], hour_params_array[1];
	uint8_t  i = 3, i_s = 3;
	uint32_t seconds = 0, minutes = 0, hours = 0, date = 0;
	static uint32_t seconds_bk = 0, minutes_bk = 0, hours_bk = 0, date_bk = 0;
#define HOUR_ARRAY   (3)
#define SENSOR_ARRAY (3)

	hour_init_read        = __getNextAlarmInitRead( RTC_TimeStructure );
	hour_end_read         = __getNextAlarmEndRead(RTC_TimeStructure);
	hour_send             = __getNextAlarmInitSend(RTC_TimeStructure);
	hour_params_send      = __getNextAlarmInitSendParams(RTC_TimeStructure);
	hour_sensor_init_read = __getNextSensorAlarmInitRead(RTC_TimeStructure,RTC_DateStruct);
	hour_sensor_end_read  = __getNextSensorAlarmEndRead(RTC_TimeStructure,RTC_DateStruct);
	hour_sensor_send      = __getNextSensorAlarmInitSend(RTC_TimeStructure);
	hour_modbus_init_read = __getNextModbusAlarmInitRead(RTC_TimeStructure);
	hour_modbus_end_read  = __getNextModbusAlarmEndRead(RTC_TimeStructure);
	hour_modbus_send      = __getNextModbusAlarmInitSend(RTC_TimeStructure);

	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> MBus Init read:%d. MBus End read:%d. MBus Send:%d."
			"Modbus Init read:%d. Modbus End read:%d. Modbus Send:%d. Params send:%d. "
			"Sensor init read:%d. Sensor end read:%d. Sensor send:%d.\r\n",
			(int)Tick_Get( SECONDS ), (int)hour_init_read, (int)hour_end_read, (int)hour_send,
			(int)hour_modbus_init_read, (int)hour_modbus_end_read, (int)hour_modbus_send,
			(int)hour_params_send,
			(int)hour_sensor_init_read, (int)hour_sensor_end_read, (int)hour_sensor_send);

	hour_array[0]    	  = hour_end_read;
	hour_array[1]    	  = hour_init_read;
	hour_array[2]    	  = hour_send;

	hour_sensor_array[0]  = hour_sensor_end_read;
	hour_sensor_array[1]  = hour_sensor_init_read;
	hour_sensor_array[2]  = hour_sensor_send;

	hour_modbus_array[0]  = hour_modbus_end_read;
	hour_modbus_array[1]  = hour_modbus_init_read;
	hour_modbus_array[2]  = hour_modbus_send;

	hour_params_array[0]  = hour_params_send;

	// Sort watermeter hours array.
	for ( uint8_t ii = 0; ii < i; ii++ ) {
		for ( uint8_t j = 0; j < i; j++ ) {
			if ( hour_array[j] != 0xFF ) {
				if ( hour_array[j] > hour_array[ii] ) {
					int tmp        = hour_array[ii];
					hour_array[ii] = hour_array[j];
					hour_array[j]  = tmp;
				}
			}
		}
	}

	uint32_t hour_found = 0;
	for ( uint8_t k = 0; k < HOUR_ARRAY; k++ ) {
		if ( hour_array[k] > RTC_TimeStructure.Hours ) {
			hour       = hour_array[k];
			hour_found = 1;
			break;
		}
	}
	if ( 0 == hour_found ) {
		hour = hour_array[0];
	}

	// Sort sensor array.
	for ( uint8_t ii = 0; ii < i_s; ii++ ) {
		for ( uint8_t j = 0; j < i_s; j++ ) {
			if ( hour_sensor_array[j] != 0xFF ) {
				if ( hour_sensor_array[j] > hour_sensor_array[ii] ) {
					int tmp               = hour_sensor_array[ii];

					hour_sensor_array[ii] = hour_sensor_array[j];
					hour_sensor_array[j]  = tmp;
				}
			}
		}
	}

	uint32_t hour_sensor_found = 0;
	for ( uint8_t k = 0; k < SENSOR_ARRAY; k++ ) {
		if ( hour_sensor_array[k] > RTC_TimeStructure.Hours ) {
			hour_sensor       = hour_sensor_array[k];
			hour_sensor_found = 1;
			break;
		}
	}
	if ( 0 == hour_sensor_found ) {
		hour_sensor = hour_sensor_array[0];
	}

	// Sort modbus array.
	for ( uint8_t ii = 0; ii < i_s; ii++ ) {
		for ( uint8_t j = 0; j < i_s; j++ ) {
			if ( hour_modbus_array[j] != 0xFF ) {
				if ( hour_modbus_array[j] > hour_modbus_array[ii] ) {
					int tmp               = hour_modbus_array[ii];

					hour_modbus_array[ii] = hour_modbus_array[j];
					hour_modbus_array[j]  = tmp;
				}
			}
		}
	}

	uint32_t hour_modbus_found = 0;
	for ( uint8_t k = 0; k < SENSOR_ARRAY; k++ ) {
		if ( hour_modbus_array[k] > RTC_TimeStructure.Hours ) {
			hour_modbus       = hour_modbus_array[k];
			hour_modbus_found = 1;
			break;
		}
	}
	if ( 0 == hour_modbus_found ) {
		hour_modbus = hour_modbus_array[0];
	}

	if ( hour_params_array[0] > RTC_TimeStructure.Hours ) {
		hour_param = hour_params_array[0];
	}

//	hours   = RTC_TimeStructure.Hours;
//	minutes = RTC_TimeStructure.Minutes;
//	seconds = RTC_TimeStructure.Seconds;
//	date    = 0;
	static uint32_t max_num_mbus_sensor_cycles = 0;
	static uint32_t current_mbus_sensor_cycle  = 0;
	if ( 0 == rtc_system_getReadSensorAlarmCycleNextInSeconds() )
	{
		max_num_mbus_sensor_cycles = 0;
	}
	else
	{
		max_num_mbus_sensor_cycles = rtc_system_getReadAlarmCycleNextInSeconds()/rtc_system_getReadSensorAlarmCycleNextInSeconds();
		if (current_mbus_sensor_cycle == 0)
		{
			hours   = RTC_TimeStructure.Hours;
			minutes = RTC_TimeStructure.Minutes;
			seconds = RTC_TimeStructure.Seconds;
			date    = 0;
			__calculateAlarmDate( rtc_system_getReadAlarmCycleNextInSeconds(), &seconds, &minutes, &hours, &date );
			hours_bk   = hours;
			minutes_bk = minutes;
			seconds_bk = seconds;
			date_bk    = date;
			current_mbus_sensor_cycle = 1;
		}
		else if (current_mbus_sensor_cycle >= (max_num_mbus_sensor_cycles - 1) )
		{
			current_mbus_sensor_cycle = 0;
		}
		else
		{
			hours   = hours_bk;
			minutes = minutes_bk;
			seconds = seconds_bk;
			date    = date_bk;
			current_mbus_sensor_cycle++;
		}
	}
	LOGLIVE(LEVEL_1,
			"LOGLIVE> %d RTC> Alarm MBus Date:sec:%d min:%d. hours:%d. date:%d. Alarm Cycle:%d. "
			"Current MBus sensor cycle:%d. Max num MBus sensor cycles:%d.\r\n",
			(int)Tick_Get( SECONDS ), (int)seconds, (int)minutes, (int)hours, (int)date,
			(int)rtc_system_getReadAlarmCycleNextInSeconds(), (int)current_mbus_sensor_cycle, (int)max_num_mbus_sensor_cycles);

	if ( hour == hour_end_read ) {
		if ( ( 1 == date )
		  || ( hours > hour_end_read )
		  || ( ( hours == hour_end_read ) && ( ( minutes > 0 ) || ( seconds > 0 ) ) )
		  || ( ( ( hours == ( hour_send - 1 ) ) && (( ( 60 - minutes) * 60 - seconds ) < rtc_system_getReadAlarmCycleNextInSeconds())) )
		   ) {
			if ( hour_end_read == hour_send ) {
				rtc_system_setCurrentAlarm(ALARM_SEND);
			} else if ( hour_end_read == hour_init_read ) {
				rtc_system_setCurrentAlarm(ALARM_INIT_READ);
			} else {
				rtc_system_setCurrentAlarm(ALARM_END_READ);
			}
		} else if (0 == current_mbus_sensor_cycle){
			rtc_system_setCurrentAlarm(ALARM_CYCLE);
		} else {
			rtc_system_setCurrentAlarm(ALARM_END_READ);
		}

	} else if (hour == hour_init_read) {
		rtc_system_setCurrentAlarm(ALARM_INIT_READ);
	} else if (hour == hour_send) {
		if ( ( 1 == date )
		  || ( hours > hour_send )
		  || ( ( hours == hour_send ) && ( ( minutes > 0 ) || ( seconds > 0 ) ) )// Is next read cycle before sending time?
		  || ( ( ( hours == ( hour_send - 1 ) ) && (( ( 60 - minutes) * 60 - seconds ) < rtc_system_getReadAlarmCycleNextInSeconds())) )
		) {
			rtc_system_setCurrentAlarm(ALARM_SEND);
		} else if ( /*( hour >= hour_init_read ) &&*/ ( hour <= hour_end_read ) && (0 == current_mbus_sensor_cycle) ) {
			rtc_system_setCurrentAlarm(ALARM_CYCLE);
		} else {
			rtc_system_setCurrentAlarm(ALARM_SEND);
		}
	} else if (( hours > hour ) && (0 == current_mbus_sensor_cycle)){
		rtc_system_setCurrentAlarm(ALARM_CYCLE);
	} else {
		rtc_system_setCurrentAlarm(ALARM_END_READ);
	}
	if (0xFF == hour_init_read) {
		rtc_system_setCurrentAlarm(ALARM_NONE);
	}

	if (hour_param == hour_params_send) {
		rtc_system_setCurrentParamsAlarm(ALARM_PARAMS_SEND_PARAMS);
	} else {
		rtc_system_setCurrentParamsAlarm(ALARM_PARAMS_NONE);
	}

	hours   = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = RTC_TimeStructure.Seconds;
	date    = 0;

	__calculateAlarmDate( rtc_system_getReadSensorAlarmCycleNextInSeconds(), &seconds, &minutes, &hours, &date );
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Alarm Date Sensor:sec:%d min:%d. hours:%d. date:%d. Alarm Cycle:%d. \r\n",
			(int)Tick_Get( SECONDS ), (int)seconds, (int)minutes, (int)hours, (int)date, (int)rtc_system_getReadSensorAlarmCycleNextInSeconds());

	if ( hour_sensor_init_read == 24 ) {
		hour_sensor_init_read = 0;
	}
	if ( hour_sensor_end_read == 24 ) {
		hour_sensor_end_read = 0;
	}
	if ( hour_sensor_send == 24 ) {
		hour_sensor_send = 0;
	}

	if ( hour_sensor == hour_sensor_send ) {
		if ( ( 1 == date )
		  || ( hours > hour_sensor_send )
		  || ( ( hours == hour_sensor_send ) && ( ( minutes >= 0 ) || ( seconds >= 0 ) ) )
		  || ( ( ( hours == ( hour_sensor_send - 1 ) ) && ((( ( 60 - minutes) * 60 - seconds ) ) < rtc_system_getReadSensorAlarmCycleNextInSeconds())) )
		) {
			rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_SEND);
		} else if ( ( ( hour_sensor >= hour_sensor_init_read) && ( hour_sensor <= hour_sensor_end_read ) )
				   || ( hour_sensor_init_read == hour_sensor_end_read ) ) {
			rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_CYCLE);
		}
	} else if ( hour_sensor == hour_sensor_end_read ) {
		if ( ( 1 == date )
		  || ( hours > hour_sensor_end_read )
		  || ( ( hours == hour_sensor_end_read ) && ( ( minutes > 0 ) || ( seconds > 0 ) ) )
		  || ( ( ( hours == ( hour_sensor_send - 1 ) ) && ((( ( 60 - minutes) * 60 - seconds ) ) < rtc_system_getReadSensorAlarmCycleNextInSeconds())) )
		) {
			if ( hour_sensor_end_read == hour_sensor_send ) {
				rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_SEND);
			} else {
				rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_END_READ);
			}
		} else {
			rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_CYCLE);
		}
	} else if ((hour_sensor == hour_sensor_init_read)
		&& ( (RTC_DateStruct.Date == params_sensor_read_time_date_day(rtc_system_getReadSensorAlarmCycleNextInSeconds())
		  && (RTC_DateStruct.Date == params_sensor_read_time_date_month(rtc_system_getReadSensorAlarmCycleNextInSeconds())))
		|| (0xFF == params_sensor_read_time_date_day(rtc_system_getReadSensorAlarmCycleNextInSeconds())))
		) {
		rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_INIT_READ);
	} else if ( ( ( hour_sensor >= hour_sensor_init_read) && ( hour_sensor <= hour_sensor_end_read ) )
			   || ( hour_sensor_init_read == hour_sensor_end_read ) ) {
		rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_CYCLE);
	} else {
		rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_END_READ);
	}
	if (0xFF == hour_sensor_init_read) {
		rtc_system_setCurrentAlarm(ALARM_SENSOR_NONE);
	}

	hours   = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = RTC_TimeStructure.Seconds;
	date    = 0;

	static uint32_t max_num_modbus_sensor_cycles = 0;
	static uint32_t current_modbus_sensor_cycle  = 0;
	if ( 0 == rtc_system_getReadSensorAlarmCycleNextInSeconds() )
	{
		max_num_modbus_sensor_cycles = 0;
	}
	else
	{
		max_num_modbus_sensor_cycles = rtc_system_getReadModbusAlarmCycleNextInSeconds()/rtc_system_getReadSensorAlarmCycleNextInSeconds();
		if (current_modbus_sensor_cycle == 0)
		{
			hours   = RTC_TimeStructure.Hours;
			minutes = RTC_TimeStructure.Minutes;
			seconds = RTC_TimeStructure.Seconds;
			date    = 0;
			__calculateAlarmDate( rtc_system_getReadModbusAlarmCycleNextInSeconds(), &seconds, &minutes, &hours, &date );
			hours_bk   = hours;
			minutes_bk = minutes;
			seconds_bk = seconds;
			date_bk    = date;
			current_modbus_sensor_cycle = 1;
		}
		else if (current_modbus_sensor_cycle >= (max_num_modbus_sensor_cycles - 1) )
		{
			current_modbus_sensor_cycle = 0;
		}
		else
		{
			hours   = hours_bk;
			minutes = minutes_bk;
			seconds = seconds_bk;
			date    = date_bk;
			current_modbus_sensor_cycle++;
		}
	}

	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Alarm Date Modbus:sec:%d min:%d. hours:%d. date:%d. Alarm Cycle:%d. "
			"Current Modbus sensor cycle:%d. Max num Modbus sensor cycles:%d\r\n",
			(int)Tick_Get( SECONDS ), (int)seconds, (int)minutes, (int)hours, (int)date,
			(int)rtc_system_getReadModbusAlarmCycleNextInSeconds(),
			(int)current_modbus_sensor_cycle, (int)max_num_modbus_sensor_cycles);

	if ( hour_modbus_init_read == 24 ) {
		hour_modbus_init_read = 0;
	}
	if ( hour_modbus_end_read == 24 ) {
		hour_modbus_end_read = 0;
	}
	if ( hour_modbus_send == 24 ) {
		hour_modbus_send = 0;
	}

	if ( hour_modbus == hour_modbus_end_read ) {
		if ( ( 1 == date )
				|| ( hours > hour_modbus_end_read )
				|| ( ( hours == hour_modbus_end_read ) && ( ( minutes > 0 ) || ( seconds > 0 ) ) )
				|| ( ( ( hours == ( hour_modbus_send - 1 ) ) && ((( ( 60 - minutes) * 60 - seconds ) ) < rtc_system_getReadModbusAlarmCycleNextInSeconds())) )
		) {
			if ( hour_modbus_end_read == hour_modbus_send ) {
				rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_SEND);
			} else if ( hour_modbus_end_read == hour_modbus_init_read ){
				rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_INIT_READ);
			} else {
				rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_END_READ);
			}
		} else if (0 == current_modbus_sensor_cycle){
			rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_CYCLE);
		} else {
			rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_END_READ);
		}
	} else if (hour_modbus == hour_modbus_init_read) {
		rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_INIT_READ);
	} else if ( hour_modbus == hour_modbus_send ) {
		if ( ( 1 == date )
				|| ( hours > hour_modbus_send )
				|| ( ( hours == hour_modbus_send ) && ( ( minutes >= 0 ) || ( seconds >= 0 ) ) )
				|| ( ( ( hours == ( hour_modbus_send - 1 ) ) && ((( ( 60 - minutes) * 60 - seconds ) ) < rtc_system_getReadModbusAlarmCycleNextInSeconds())) )
		) {
			rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_SEND);
		} else if ( ( /*( hour_modbus >= hour_modbus_init_read) &&*/ ( hour_modbus <= hour_modbus_end_read ) && (0 == current_modbus_sensor_cycle) )
		/*|| ( hour_modbus_init_read == hour_modbus_end_read )*/ ) {
			rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_CYCLE);
		}  else {
			rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_SEND);
		}
	}/* else if ( ( ( hour_modbus >= hour_modbus_init_read) && ( hour_modbus <= hour_modbus_end_read ) )
			   || ( hour_modbus_init_read == hour_modbus_end_read ) ) {
		rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_CYCLE);
	}*/
	else if (( hours > hour_modbus ) && (0 == current_modbus_sensor_cycle)) {
		rtc_system_setCurrentAlarm(ALARM_MODBUS_CYCLE);
	} else {
		rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_END_READ);
	}
	if (0xFF == hour_modbus_init_read) {
		rtc_system_setCurrentAlarm(ALARM_MODBUS_NONE);
	}
//	rtc_system_setCurrentAlarm(ALARM_SEND);						  //DEBUG
//	rtc_system_setReadAlarmCycleNext(params_read_time_cycle( 0 ));//DEBUG
//	rtc_system_setCurrentSensorAlarm(ALARM_SENSOR_SEND);          //DEBUG
//	rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_SEND);		  //DEBUG
//	rtc_system_setCurrentModbusAlarm(ALARM_MODBUS_CYCLE);		  //DEBUG

	hours   = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = RTC_TimeStructure.Seconds;

	if ( ( hour == hours ) || (hour == hours - 1 ) ) {
		hour = hours;
	} else {
		if ( ( hour_sensor >= hours ) && ( hour_sensor < hour_modbus ) ) {
			hour = hour_sensor;
		}

		if ( ( hour_modbus >= hours ) && ( hour_modbus < hour_sensor ) ) {
			hour = hour_modbus;
		}

		if ( ( hour_modbus >= hours ) && ( hour_sensor >= hours ) && ( hour_modbus == hour_sensor ) ) {
			hour = hour_modbus;
		}
	}

	if ( ( hour_param >= hours) && ( hour_param < hour ) ) {
		hour = hour_param;
	}

	return hour;
}

int32_t rtc_system_SetAlarm( time_t sys_time_ms )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         prev_hour, next_day;
	static uint32_t  hour;
	struct tm       *date;
	time_t           sys_time = (time_t)(sys_time_ms/1000);

	date = gmtime(&sys_time);

    /* Set the time */
    RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
    RTC_TimeStructure.Hours      = date->tm_hour;
    RTC_TimeStructure.Minutes    = date->tm_min;
    RTC_TimeStructure.Seconds    = date->tm_sec;

    RTC_DateStruct.Date    = date->tm_mday;
    RTC_DateStruct.Month   = date->tm_mon + 1;
    RTC_DateStruct.WeekDay = date->tm_wday + 1;
    RTC_DateStruct.Year    = date->tm_year - ( 2000 - 1900 );

#if 0
	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
#endif

	prev_hour = hour;
	hour      = __getCurrentAlarm( RTC_TimeStructure, RTC_DateStruct );

	if ( ( prev_hour < 24 ) && ( prev_hour > 0 ) && ( hour > 0 ) && ( prev_hour > hour ) ) {
		next_day = 1;
	}

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	if ( ( 0 == hour ) || ( 24 == hour ) || ( 1 == next_day ) ) {
		if ( 24 == hour ) {
			hour = 0;
		}
		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
	}
	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_SECONDS/* | RTC_ALARMMASK_DATEWEEKDAY*/ ;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;//RTC_TimeStructure.Hours;       //
		RTC_AlarmStructure.AlarmTime.Minutes   = 0;   //RTC_TimeStructure.Minutes + 1; //
		RTC_AlarmStructure.AlarmTime.Seconds   = 0;
		RTC_AlarmStructure.AlarmTime.SubSeconds= 0;

		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Time Alarm: %d:0:0.\r\n",
				(int)Tick_Get(SECONDS), (int)hour);

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;
		rtc_Calendar.alarmState   = ALARM_STARTED;

		return 0;
	}
	else {
		return -1;
	}
}

int32_t rtc_system_SetQuarterAlarm( void )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour, minutes, seconds;//, date;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

//	hour    = RTC_TimeStructure.Hours;
//	minutes = RTC_TimeStructure.Minutes;
//	seconds = RTC_TimeStructure.Seconds;
//	__calculateAlarmDate( STAND_BY_TIME, &seconds, &minutes, &hour, &date );
//
//	if ( 1 == date ) {
//		RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
//	}

	hour    = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = 0;
	if ( ( minutes >= 0 ) && ( minutes < 15 ) ) {
		minutes = 15;
	} else if ( ( minutes >= 15 ) && ( minutes < 30 ) ) {
		minutes = 30;
	} else if ( ( minutes >= 30 ) && ( minutes < 45 ) ) {
		minutes = 45;
	} else if ( minutes >= 45 ) {
		minutes = 0;
		if ( 23 == hour ) {
			hour = 0;
			RTC_DateStruct.Date = RTC_DateStruct.Date + 1;
		} else {
			hour = RTC_TimeStructure.Hours + 1;
		}
	}

	if ( ( hour >= 0 ) && ( hour < 24 ) ) {
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;//RTC_ALARMMASK_SECONDS;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = minutes;
		RTC_AlarmStructure.AlarmTime.Seconds   = seconds;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);

		__HAL_RTC_ALARMA_ENABLE(&hrtc);

		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else {
		return -1;
	}
}

/**
 * @fn void __next_date(uint8_t*, uint8_t*, uint8_t*)
 * @brief Calculates the next data
 *
 * @pre
 * @post
 * @param year
 * @param month
 * @param day
 */
static void __next_date(uint8_t *year, uint8_t *month, uint8_t *day)
{
	uint8_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    *day = *day + 1;
    if ((*month == 2) && (*day == 29))
    {
        /* Leap year checking, if yes, Feb is 29 days.*/
        if((*year % 400 == 0) || ((*year % 100 != 0) && (*year % 4 == 0)))
        {
            daysInMonth[1] = 29;
        }
    }

    if (*day > daysInMonth[*month -1])
    {
        *day   = 1;
        *month = *month +1;
        if (*month > 12)
        {
            *month = 1;
            *year  = *year +1;
        }
    }
}

/**
 * @fn int32_t rtc_system_SetSyncAlarm(uint32_t)
 * @brief Set the time elapsed between sensor samples to read_time time.
 *
 * @param read_time time elapsed between sensor samples.
 * @return
 */
int32_t rtc_system_SetSyncAlarm( uint32_t read_time )
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour, minutes, seconds;//, date;

	/** Set modbus and meter synch read flags enabled */
	shutdown_set_sync_modbus(1);
	shutdown_set_sync_meter(1);
	/** Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/** Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/** Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour    = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = 1;

	/** Calculates the next RTC synchronized Alarm A*/

	/* Sensor reading time configured 15min */
	if (900 == read_time)
	{
		if ((minutes >= 0) && (minutes < 15))
		{
			minutes = 15;
		}
		else if ((minutes >= 15) && (minutes < 30))
		{
			minutes = 30;
		}
		else if ((minutes >= 30) && (minutes < 45))
		{
			minutes = 45;
		}
		else if (minutes >= 45)
		{
			minutes = 0;

			if (23 == hour)
			{
				hour = 0;
				/* Calculates next date depending on the day of the month and leap year */
				__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
			}
			else
			{
				hour = RTC_TimeStructure.Hours + 1;
			}
		}
	}
	/* Sensor reading time configured 15min */
	else if (1800 == read_time)
	{
		if ((minutes >= 0) && (minutes < 30))
		{
			minutes = 30;
		}
		else if (minutes >= 30)
		{
			minutes = 0;

			if (23 == hour)
			{
				hour = 0;
				/* Calculates next date depending on the day of the month and leap year */
				__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
			}
			else
			{
				hour = RTC_TimeStructure.Hours + 1;
			}
		}
	}
	/* Sensor reading configured for one hour */
	else if (3600 == read_time)
	{
		minutes = 0;

		if (23 == hour)
		{
			hour = 0;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = RTC_TimeStructure.Hours + 1;
		}
	}
	/* Sensor reading configured for two hour */
	else if (7200 == read_time)
	{
		minutes = 0;

		if (22 == hour)
		{
			hour = 0;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (23 == hour)
		{
			hour = 1;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = RTC_TimeStructure.Hours + 2;
		}
	}
	/* Sensor reading configured for two hour */
	else if (10800 == read_time)
	{
		minutes = 0;

		if (21 == hour)
		{
			hour = 0;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (22 == hour)
		{
			hour = 1;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (23 == hour)
		{
			hour = 2;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = RTC_TimeStructure.Hours + 3;
		}
	}
	else if (21600 == read_time)
	{
		minutes = 0;

		if (18 == hour)
		{
			hour = 0;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (19 == hour)
		{
			hour = 1;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (20 == hour)
		{
			hour = 2;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (21 == hour)
		{
			hour = 3;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (22 == hour)
		{
			hour = 4;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (23 == hour)
		{
			hour = 5;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = RTC_TimeStructure.Hours + 6;
		}
	}
	else if (read_time < 900)
	{
		if ((minutes >= 0) && (minutes < 15))
		{
			minutes = 15;
		}
		else if ((minutes >= 15) && (minutes < 30))
		{
			minutes = 30;
		}
		else if ((minutes >= 30) && (minutes < 45))
		{
			minutes = 45;
		}
		else if (minutes >= 45)
		{
			minutes = 0;

			if (23 == hour)
			{
				hour = 0;
				/* Calculates next date depending on the day of the month and leap year */
				__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
			}
			else
			{
				hour = RTC_TimeStructure.Hours + 1;
			}
		}
	}
	else
	{
		minutes = 0;

		if (6 == hour)
		{
			hour = 0;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (7 == hour)
		{
			hour = 1;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (8 == hour)
		{
			hour = 2;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (9 == hour)
		{
			hour = 3;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (10 == hour)
		{
			hour = 4;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (11 == hour)
		{
			hour = 5;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (12 == hour)
		{
			hour = 6;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (13 == hour)
		{
			hour = 7;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (14 == hour)
		{
			hour = 8;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (15 == hour)
		{
			hour = 9;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (16 == hour)
		{
			hour = 10;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (17 == hour)
		{
			hour = 11;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (18 == hour)
		{
			hour = 12;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (19 == hour)
		{
			hour = 13;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (20 == hour)
		{
			hour = 14;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (21 == hour)
		{
			hour = 15;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (22 == hour)
		{
			hour = 16;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (23 == hour)
		{
			hour = 17;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = RTC_TimeStructure.Hours + 18;
		}
	}
	/** @todo Exit with -1 if alarm values is not 900 or 3600 */

	if ((hour >= 0) && (hour < 24 ))
	{
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_SECONDS;//RTC_ALARMMASK_NONE;//
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = minutes;
		RTC_AlarmStructure.AlarmTime.Seconds   = seconds;
		RTC_AlarmStructure.AlarmTime.SubSeconds = 0;

		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Time Alarm: %d:%d:%d.\r\n",
				(int)Tick_Get(SECONDS), (int)hour, (int)minutes, (int)seconds);

		/** Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);
		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
		__HAL_RTC_ALARMA_ENABLE(&hrtc);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

//		rtc_Calendar.alarmOccured = 0;
		rtc_Calendar.alarmState = ALARM_STARTED;
		return 0;
	}
	else
	{
		return -1;
	}
}

/**
 * @fn int32_t rtc_system_SetPressureOnSyncAlarm(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param read_time
 * @return
 */
int32_t rtc_system_SetPressureOnSyncAlarm(uint32_t read_time)
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour, minutes, seconds, set_alarm = 1;
	static uint32_t  first_read = 0;

	/** Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/** Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/** Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour    = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = RTC_TimeStructure.Seconds;

	uint32_t minutes_pressure, seconds_pressure;
	if (params_input_pulse_as_sensor_get_num() != 0)
	{
		seconds_pressure = seconds + params_sensor_read_time_cycle(0);
	}
	else
	{
		seconds_pressure = seconds + 30;
	}

	/** Adds the 30 hard coded pressure period time to the next alarm and calculate the new alarm time */
	if (seconds_pressure > 59)
	{
		minutes_pressure = minutes + 1;
		seconds          = seconds_pressure % 60;
	}
	else
	{
		minutes_pressure = minutes;
		seconds          = seconds_pressure;
	}

	if (minutes_pressure > 59)
	{
		minutes_pressure = 0;
		if (23 == hour)
		{
			hour = 0;
			/* Calculates next date depending on the day of the month and leap year */
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = hour + 1;
		}
	}

	/* Reading time every 15 minutes*/
	if (900 == read_time)
	{
		/*  */
		if ((first_read == 0) && ((minutes_pressure == 15) || (minutes_pressure == 30) || (minutes_pressure == 45) || (minutes_pressure == 0)))
		{
			/* water meter disabled and modbus disabled*/
			if ((1 == params_config_get_disable_meter()) && (generic_485_get_read_time() == 0))
			{
				set_alarm  = 0;
				first_read = 0;
				shutdown_set_sync_meter(0);
				shutdown_set_sync_modbus(0);
				rtc_system_set_wake_up_time();
			}
			/* water metar and/or modbus enabled */
			else
			{
				minutes    = minutes_pressure;
				set_alarm  = 1;
				first_read = 1;
				shutdown_set_sync_meter(1);

				if ((generic_485_get_read_time() != 0) && (generic_485_get_read_time() <= 900))
				{
					shutdown_set_sync_modbus(1);
				}
			}
		}
		/* subsequent alarm set */
		else
		{
			set_alarm  = 0;
			first_read = 0;
			shutdown_set_sync_meter(0);
			shutdown_set_sync_modbus(0);
			rtc_system_set_wake_up_time();
		}

	}

	/* Reading time every hour */
	else if (3600 == read_time)
	{
		/*  */
		if  ((first_read == 0) && (minutes_pressure == 0))
		{
			if ((1 == params_config_get_disable_meter()) && (generic_485_get_read_time() == 0))
			{
				set_alarm  = 0;
				first_read = 0;
				shutdown_set_sync_meter(0);
				shutdown_set_sync_modbus(0);
				rtc_system_set_wake_up_time();
			}
			else
			{
				minutes    = minutes_pressure;
				set_alarm  = 1;
				first_read = 1;
				shutdown_set_sync_meter(1);

				if ((generic_485_get_read_time() != 0) && (generic_485_get_read_time() <= 900))
				{
					shutdown_set_sync_modbus(1);
				}
			}
		}
		else
		{
			set_alarm  = 0;
			first_read = 0;
			shutdown_set_sync_meter(0);
			shutdown_set_sync_modbus(0);
			rtc_system_set_wake_up_time();
		}
	}
	/* Sensor reading configured for two hour */
	else if (10800 == read_time)
	{
		minutes = 0;

		if (21 == hour)
		{
			hour = 0;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (22 == hour)
		{
			hour = 1;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else if (23 == hour)
		{
			hour = 2;
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = RTC_TimeStructure.Hours + 2;
		}
	}
	if ((1 == set_alarm) && (hour >= 0) && (hour < 24))
	{
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;//RTC_ALARMMASK_SECONDS;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = minutes;
		RTC_AlarmStructure.AlarmTime.Seconds   = seconds;
		RTC_AlarmStructure.AlarmTime.SubSeconds= 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);
		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
		__HAL_RTC_ALARMA_ENABLE(&hrtc);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

		rtc_Calendar.alarmOccured = 0;

		return 0;
	}
	else
	{
		return -1;
	}
}

int32_t rtc_system_SetPulseTimer(void)
{
	rtc_system_Set_Stop_Mode_RTCCLK_DIV16(30000);
	rtc_Calendar.timerState = TIMER_STARTED;

	return 0;
}

int32_t rtc_system_SetPulseAlarm(void)
{
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t         hour, minutes, seconds;//, set_alarm = 1;
//	static uint32_t  first_read = 0;

	/** Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/** Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/** Disable the Alarm A */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

	hour    = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = RTC_TimeStructure.Seconds;

	uint32_t minutes_pressure, seconds_pressure;
	if (params_input_pulse_as_sensor_get_num() != 0)
	{
		seconds_pressure = seconds + params_sensor_read_time_cycle(0);
	}
	else
	{
		seconds_pressure = seconds + 30;
	}

	/** Adds the 30 hard coded pressure period time to the next alarm and calculate the new alarm time */
	if (seconds_pressure > 59)
	{
		minutes_pressure = minutes + 1;
		seconds          = seconds_pressure % 60;
	}
	else
	{
		minutes_pressure = minutes;
		seconds          = seconds_pressure;
	}

	if (minutes_pressure > 59)
	{
		minutes_pressure = 0;
		if (23 == hour)
		{
			hour = 0;
			/* Calculates next date depending on the day of the month and leap year */
			__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
		}
		else
		{
			hour = hour + 1;
		}
	}

	if ((hour >= 0) && (hour < 24))
	{
		RTC_AlarmStructure.Alarm               = RTC_ALARM_A;
		RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
		RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
		RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_NONE;//RTC_ALARMMASK_SECONDS;
		RTC_AlarmStructure.AlarmSubSecondMask  = RTC_ALARMSUBSECONDMASK_ALL;
		RTC_AlarmStructure.AlarmTime.TimeFormat= RTC_HOURFORMAT12_AM;
		RTC_AlarmStructure.AlarmTime.Hours     = hour;
		RTC_AlarmStructure.AlarmTime.Minutes   = minutes;
		RTC_AlarmStructure.AlarmTime.Seconds   = seconds;
		RTC_AlarmStructure.AlarmTime.SubSeconds= 0;

		/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
		STANDBY mode (RTC Alarm IT not enabled in NVIC) */
		HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);
		__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
		__HAL_RTC_ALARMA_ENABLE(&hrtc);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

//		rtc_Calendar.alarmOccured = 0;
		rtc_Calendar.alarmState   = ALARM_STARTED;
	}

	return 0;
}

int32_t rtc_system_SetResetAlarm( void )
{
	/** Enable the Alarm B
	 */
	RTC_AlarmTypeDef sAlarm = {0};
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;

	struct tm       *date;
	time_t reset_time = params_get_time_reset();

	date = gmtime(&reset_time);

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	/* Disable the Alarm B */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
	if (0 == reset_time)
	{
		return -1;
	}
	sAlarm.Alarm                = RTC_ALARM_B;
	sAlarm.AlarmDateWeekDay     = date->tm_mday;//RTC_WEEKDAY_MONDAY;
//	sAlarm.AlarmDateWeekDaySel  = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
	sAlarm.AlarmMask            = RTC_ALARMMASK_DATEWEEKDAY/* | RTC_ALARMMASK_SECONDS*/;
	sAlarm.AlarmSubSecondMask   = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	sAlarm.AlarmTime.Hours      = date->tm_hour;
	sAlarm.AlarmTime.Minutes    = date->tm_min;
	sAlarm.AlarmTime.Seconds    = date->tm_sec;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
	/* Enable RTC Alarm B Interrupt: this Interrupt will wake-up the system from
	STANDBY mode (RTC Alarm IT not enabled in NVIC) */
	__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRB);

	__HAL_RTC_ALARMB_ENABLE(&hrtc);

	__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);

	/* Calculates next date depending on the day of the month and leap year */
	__next_date(&RTC_DateStruct.Year,&RTC_DateStruct.Month,&RTC_DateStruct.Date);
	date->tm_year  = RTC_DateStruct.Year  + ( 2000 - 1900 );
	date->tm_mon   = RTC_DateStruct.Month - 1;
	date->tm_mday  = RTC_DateStruct.Date;
	date->tm_wday  = RTC_DateStruct.WeekDay;
	date->tm_isdst = -1;
	params_set_time_reset(mktime(date));

	return 0;

}

int32_t rtc_system_SetDelaySendAlarm( void )
{
	/** Enable the Alarm B
	 */
	RTC_AlarmTypeDef sAlarm = {0};
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_DateTypeDef  RTC_DateStruct;

	struct tm       *date;
	time_t reset_time = params_get_time_reset();

	date = gmtime(&reset_time);

	/** Set modbus and meter synch read flags enabled */
	shutdown_set_sync_modbus(1);
	shutdown_set_sync_meter(1);
	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);
	/* Disable the Alarm B */
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
	if (0 == reset_time)
	{
		return -1;
	}
	sAlarm.Alarm                = RTC_ALARM_A;
	sAlarm.AlarmDateWeekDay     = RTC_DateStruct.Date;//RTC_WEEKDAY_MONDAY;
//	sAlarm.AlarmMask            = RTC_ALARMMASK_DATEWEEKDAY/* | RTC_ALARMMASK_SECONDS*/;
	sAlarm.AlarmDateWeekDaySel  = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmMask            = RTC_ALARMMASK_SECONDS;//RTC_ALARMMASK_NONE;//
	sAlarm.AlarmSubSecondMask   = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	sAlarm.AlarmTime.Hours      = RTC_TimeStructure.Hours;
	if (date->tm_min > RTC_TimeStructure.Minutes)
	{
		sAlarm.AlarmTime.Minutes    = date->tm_min;
	}
	else
	{
		if ((RTC_TimeStructure.Minutes + 5) < 59)
		{
			sAlarm.AlarmTime.Minutes = RTC_TimeStructure.Minutes + 5;
		}
		else
		{
			sAlarm.AlarmTime.Hours   = RTC_TimeStructure.Hours + 1;
			sAlarm.AlarmTime.Minutes = date->tm_min;
		}
	}
	sAlarm.AlarmTime.Seconds    = date->tm_sec;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> RTC Time Alarm: %d:%d:%d.\r\n",
			(int)Tick_Get(SECONDS), (int)sAlarm.AlarmTime.Hours, (int)sAlarm.AlarmTime.Minutes, (int)sAlarm.AlarmTime.Seconds);
	/** Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
	STANDBY mode (RTC Alarm IT not enabled in NVIC) */
	HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
	__HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
	__HAL_RTC_ALARMA_ENABLE(&hrtc);
	__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

	rtc_Calendar.alarmState = ALARM_STARTED;

	return 0;

}

/**
 * @fn void rtc_system_reset_remaining_time(void)
 * @brief
 *
 * @pre
 * @post
 */
void rtc_system_reset_remaining_time( void )
{
	res_cycle_time = 0;
	remaining_time = 0;
}

/**
 * @fn void rtc_system_reset_remaining_time_last_pulse(void)
 * @brief Resets the time to go to sleep.
 *
 */
void rtc_system_reset_remaining_time_last_pulse( void )
{
	res_cycle_time = STAND_BY_TIME_MS - 1000;
	remaining_time = 0;
}

/**
 * @fn uint32_t rtc_system_last_pulse(void)
 * @brief Flag to set that the last pulse before going to sleep.
 *
 * @return 	@arg 0 - still time remaining to go to sleep.
 * 			@arg 1 - No remaining time to go to sleep.
 */
uint32_t rtc_system_last_pulse( void )
{
	if (0 == res_cycle_time)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
 * @fn void rtc_system_set_wake_up_time(void)
 * @brief Sets the auto wake up timer to
 *
 * @pre
 * @post
 */
void rtc_system_set_wake_up_time(void)
{

	uint32_t set_alarm = 0, reset_interruption_num = 0;
	static uint32_t interruption_num = 0;
	time_t sys_time_in_ms;
	uint32_t stand_by_tm_ms = STAND_BY_TIME_MS;
	(void)reset_interruption_num;

	if ((params_input_pulse_as_sensor_get_num() != 0) && (1 == params_config_get_period()))
	{
		stand_by_tm_ms =  params_sensor_read_time_cycle(0)*1000;
	}

	/* Time compensation when multiple pulses are received */
	if ((1 == params_pulses_on()) && (1 == pulses_get_pulse_rx()) && (1 == Tick_cloud_time_init()))
	{
		interruption_num += 1;

		/* Fix to adjust time, every 6 times increments remaining time one second */
		if (0 == interruption_num % (calib_period)) //6//2*calib_period
		{
//			remaining_time += (calib_period) * 36;//500;//1000;//2*calib_period*36
			LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> interruption_num:%d. remaining_time:%d\r\n", (int)Tick_Get( SECONDS ), (int)interruption_num, (int)remaining_time);
		}
	}

	/* get elapsed time since */
	sys_time_in_ms = rtc_system_get_time_in_miliseconds();
	remaining_time += __getRemainingWakeUpTime(sys_time_in_ms);//__getRemainingWakeUpTime(Tick_Get(SECONDS));

	/* */
	if (remaining_time >= stand_by_tm_ms/*STAND_BY_TIME_MS*/)
	{
		if ( stand_by_tm_ms/*STAND_BY_TIME_MS*/ > 0)
		{
			remaining_time = stand_by_tm_ms/*STAND_BY_TIME_MS*/ - 10;//1000;
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> remaining_time >= STAND_BY_TIME_MS:%d\r\n", (int)Tick_Get( SECONDS ), (int)remaining_time);
	}


	/* Periodic sensor and transmission operation mode */
//	if (1 == params_config_get_period())
	{
		if (remaining_time < stand_by_tm_ms/*STAND_BY_TIME_MS*/)
		{
			res_cycle_time = res_cycle_time_prev = (stand_by_tm_ms/*STAND_BY_TIME_MS*/ - remaining_time);// - 1000;

			if (res_cycle_time <= -1)
			{
				rtc_system_reset_remaining_time();
				interruption_num = 0;
				reset_interruption_num = 1;
			}
			else if (res_cycle_time == 0)
			{
				interruption_num = 0;
				reset_interruption_num = 1;
			}
		}
#if defined(PWR_STDBY)
		rtc_system_Set(STAND_BY_TIME);
#elif defined(PWR_STOP)
		if (0 == params_pulses_on())
		{
//			if (res_cycle_time > 0)
//			{
//				res_cycle_time = res_cycle_time - 1000;//750//1375;//1000;
//			}
		}
		time_in_miliseconds_before_low_power = sys_time_in_ms;//rtc_system_get_time_in_miliseconds();
		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> remaining_time:%d res_cycle_time:%d\r\n",
				(int)Tick_Get( SECONDS ), (int)remaining_time, (int)res_cycle_time);
		/* Sets the system to wake up from sleep after res_cycle_time time*/
		if (1 == params_config_get_period())
		{
			if (params_input_pulse_as_sensor_get_num() != 0)
			{
				rtc_system_Set_Stop_Mode(res_cycle_time/1000);
			}
			else
			{
				rtc_system_Set_Stop_Mode_RTCCLK_DIV16(res_cycle_time);
			}
			generic_sensor_log_check_set_programword(stand_by_tm_ms/1000);
			sensor_log_check_set_programword(stand_by_tm_ms/1000);
#endif
			return; /* Exits routine */
		}
		else
		{
			generic_sensor_log_check_set_programword(stand_by_tm_ms/1000);
			sensor_log_check_set_programword(stand_by_tm_ms/1000);
		}
	}

	/* For Window based mode */
	rtc_system_SetAlarm(sys_time_in_ms);

	if (((ALARM_CYCLE     == rtc_system_getCurrentAlarmFromBKUP())
	  || (ALARM_SEND      == rtc_system_getCurrentAlarmFromBKUP())
	  || (ALARM_INIT_READ == rtc_system_getCurrentAlarmFromBKUP()))
	  && (1 == params_pulses_on()))
	{
		rtc_system_setCurrentAlarm(ALARM_NONE);
	}

	if ((ALARM_CYCLE == rtc_system_getCurrentAlarmFromBKUP()) && (0 == params_pulses_on()))
	{
		if (rtc_system_getReadAlarmCycleNextInSeconds() != 0)
		{
#if defined(PWR_STDBY)
			rtc_system_Set(rtc_system_getReadAlarmCycleNextInSeconds());
#elif defined(PWR_STOP)
			rtc_system_Set_Stop_Mode(rtc_system_getReadAlarmCycleNextInSeconds());
			set_alarm = 1;
#endif
		}
	}

	if (ALARM_SENSOR_CYCLE == rtc_system_getCurrentSensorAlarm())
	{
		if (rtc_system_getReadSensorAlarmCycleNextInSeconds() != 0)
		{
//			if (remaining_time < rtc_system_getReadSensorAlarmCycleNextInSeconds())
//			{
//				res_cycle_time = (rtc_system_getReadSensorAlarmCycleNextInSeconds() - remaining_time);
//
//				if (res_cycle_time <= 0)
//				{
//					rtc_system_reset_remaining_time();
//				}
//			}
//			else
//			{
//				res_cycle_time = rtc_system_getReadSensorAlarmCycleNextInSeconds();
//				remaining_time = 0;
//			}
			if (res_cycle_time <= 0)
			{
				res_cycle_time = remaining_time = rtc_system_getReadSensorAlarmCycleNextInSeconds()*1000;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> remaining_time:%d res_cycle_time:%d\r\n",
						(int)Tick_Get( SECONDS ), (int)remaining_time, (int)res_cycle_time);
			}
#if defined(PWR_STDBY)
			rtc_system_Set(  rtc_system_getReadSensorAlarmCycleNextInSeconds() );
#elif defined(PWR_STOP)
			if (res_cycle_time > (0xFFFF>>1))
			{
				rtc_system_Set_Stop_Mode( res_cycle_time/1000 );
				LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> rtc_system_Set_Stop_Mode:%d\r\n",
						(int)Tick_Get( SECONDS ), (int)(res_cycle_time/1000));
			}
			else
			{
				rtc_system_Set_Stop_Mode_RTCCLK_DIV16(res_cycle_time);
				LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> rtc_system_Set_Stop_Mode_RTCCLK_DIV16:%d\r\n",
						(int)Tick_Get( SECONDS ), (int)res_cycle_time);
			}
			set_alarm = 1;
#endif
		}
	}

	if (ALARM_MODBUS_CYCLE == rtc_system_getCurrentModbusAlarm())
	{
		if (rtc_system_getReadModbusAlarmCycleNextInSeconds() != 0)
		{
			if (rtc_system_getReadSensorAlarmCycleNextInSeconds() > rtc_system_getReadModbusAlarmCycleNextInSeconds())
			{
				if (remaining_time < rtc_system_getReadModbusAlarmCycleNextInSeconds())
				{
					remaining_time += remaining_time;
					res_cycle_time = (rtc_system_getReadSensorAlarmCycleNextInSeconds() - remaining_time);

					if (res_cycle_time <= 0)
					{
						rtc_system_reset_remaining_time();
					}
				}
				else
				{
					res_cycle_time = rtc_system_getReadModbusAlarmCycleNextInSeconds();
					remaining_time = 0;
				}
#if defined(PWR_STDBY)
				rtc_system_Set(rtc_system_getReadModbusAlarmCycleNextInSeconds());
#elif defined(PWR_STOP)
//				rtc_system_Set_Stop_Mode( res_cycle_time/*rtc_system_getReadModbusAlarmCycleNextInSeconds()*/ );
				rtc_system_Set_Stop_Mode_RTCCLK_DIV16(res_cycle_time*1000);
				set_alarm = 1;
#endif
			}
		}
	}

	if ( 0 == set_alarm )
	{
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	}
	generic_sensor_log_check_set_programword(res_cycle_time/1000);
	sensor_log_check_set_programword(res_cycle_time/1000);
//	rtc_system_Set_Stop_Mode( rtc_system_getReadAlarmCycleNextInSeconds() );//DEBUG
}

void rtc_system_StopModeAlarmConfig( uint32_t sleeptime )
{
	RTC_TimeTypeDef  RTC_TimeStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;
	RTC_DateTypeDef  RTC_DateStruct;
	uint32_t seconds = 0, minutes = 0, hours = 0, date = 0;

	/* Get the current time */
	HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);
	/* Get the current date */
	HAL_RTC_GetDate(&hrtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	/* Initialize alarm to current date */
	hours   = RTC_TimeStructure.Hours;
	minutes = RTC_TimeStructure.Minutes;
	seconds = RTC_TimeStructure.Seconds;

	__calculateAlarmDate( sleeptime, &seconds, &minutes, &hours, &date );

	/* Set the alarm to current time + sleeptime */
	RTC_AlarmStructure.AlarmTime.Hours     = hours;
	RTC_AlarmStructure.AlarmTime.Minutes   = minutes;
	RTC_AlarmStructure.AlarmTime.Seconds   = seconds;
	RTC_AlarmStructure.AlarmDateWeekDay    = RTC_DateStruct.Date;
	RTC_AlarmStructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	RTC_AlarmStructure.AlarmMask           = RTC_ALARMMASK_DATEWEEKDAY;//RTC_AlarmMask_None;

	/* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
	STANDBY mode (RTC Alarm IT not enabled in NVIC) */
	HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);
}

/**
  * @brief  Enters STANDBY mode, RTC Alarm within 3 second or an external RESET
  *         will wake-up the system from STANDBY
  * @param  None
  * @retval None
  */
void rtc_system_EnterSTANDBYMode(void)
{
  RTC_AlarmTypeDef  RTC_AlarmStructure;
  RTC_TimeTypeDef   RTC_TimeStructure;

  /* Disable the Alarm A */
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

  /* Get the current time */
  HAL_RTC_GetTime(&hrtc, &RTC_TimeStructure, RTC_FORMAT_BIN);

  /* Set the alarm to current time + 3s */
  RTC_AlarmStructure.AlarmTime.TimeFormat  = RTC_TimeStructure.TimeFormat;
  RTC_AlarmStructure.AlarmTime.Hours       = RTC_TimeStructure.Hours;
  RTC_AlarmStructure.AlarmTime.Minutes 	   = RTC_TimeStructure.Minutes;
  RTC_AlarmStructure.AlarmTime.Seconds 	   = (RTC_TimeStructure.Seconds + 0x3) % 60;
  RTC_AlarmStructure.AlarmDateWeekDay      = 31;
  RTC_AlarmStructure.AlarmDateWeekDaySel   = RTC_ALARMMASK_DATEWEEKDAY;
  RTC_AlarmStructure.AlarmMask             = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
  HAL_RTC_SetAlarm_IT(&hrtc, &RTC_AlarmStructure, RTC_FORMAT_BIN);

//  /* Enable RTC Alarm A Interrupt: this Interrupt will wake-up the system from
//  STANDBY mode (RTC Alarm IT not enabled in NVIC) */
//  RTC_ITConfig(RTC_IT_ALRA, ENABLE);
//
//  /* Enable the Alarm A */
//  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
//
//  /* Clear RTC Alarm Flag */
//  RTC_ClearFlag(RTC_FLAG_ALRAF);

  /* Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function) */
  HAL_PWR_EnterSTANDBYMode();
}

void rtc_system_Set( uint32_t t_seconds )
{
	/* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
	 mainly  when using more than one wakeup source this is to not miss any wakeup event.
	 - Disable all used wakeup sources,
	 - Clear all related wakeup flags,
	 - Re-enable all used wakeup sources,
	 - Enter the Standby mode.
	 */
	/* Disable all used wakeup sources*/
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/* Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG( PWR_FLAG_SBF );
	__HAL_PWR_CLEAR_FLAG( PWR_WAKEUP_ALL_FLAG );
	__HAL_RTC_CLEAR_FLAG( &hrtc, RTC_CLEAR_WUTF );

	/* Re-enable all used wakeup sources*/
	/* ## Setting the Wake up time ############################################*/
	/* RTC Wakeup Interrupt Generation: */
	/* Setting the Wakeup time to 1 s	*/
	/*				 If RTC_WAKEUPCLOCK_CK_SPRE_16BITS is selected, the frequency is 1Hz,*/
	/*				 this allows to get a wakeup time equal to 1 s if the counter is 0x0 */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, t_seconds & 0x0000FFFF, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
}

/**
  * @brief  Sets the system to WAKE-UP after t_seconds time.
  * @param  t_seconds : time to wake-up after.
  * @retval None
*/
void rtc_system_Set_Stop_Mode(uint32_t t_seconds)
 {
	/** Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/** Ensure that MSI is wake-up system clock */
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

	/** Disable all used wakeup source */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/** Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
	__HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_ALL_FLAG);
	__HAL_RTC_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	/** Re-enable wake up timer configured to t_seconds */
	/* ## Setting the Wake up time ############################################*/
	/* RTC Wakeup Interrupt Generation: */
	/* Setting the Wakeup time to 1 s	*/
	/*				 If RTC_WAKEUPCLOCK_CK_SPRE_16BITS is selected, the frequency is 1Hz,*/
	/*				 this allows to get a wakeup time equal to 1 s if the counter is 0x0 */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, t_seconds & 0x0FFFF, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);

//	/* Enter STOP 2 mode */
//	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
//
//	/* ... STOP2 mode ... */
	rtc_Calendar.timerState = TIMER_STARTED;
}

/**
  * @brief  Sets the system to WAKE-UP after t_seconds time.
  * @param  t_seconds : time to wake-up after.
  * @retval None
*/
void rtc_system_Set_Stop_Mode_RTCCLK_DIV16(uint32_t wake_up_time_ms)
{
	uint32_t counter_value = wake_up_time_ms*2;

	/** Enable Power Clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/** Ensure that MSI is wake-up system clock */
	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

	/** Disable all used wakeup source */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/** Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);
	__HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_ALL_FLAG);
	__HAL_RTC_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	/* Re-enable wakeup source */
	/* ## Setting the Wake up time ############################################*/
	/* RTC Wakeup Interrupt Generation:
	    the wake-up counter is set to its maximum value to yield the longuest
	    stop time to let the current reach its lowest operating point.
	    The maximum value is 0xFFFF, corresponding to about 33 sec. when
	    RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16

	    Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI))
	    Wakeup Time = Wakeup Time Base * WakeUpCounter
	      = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSI)) * WakeUpCounter
	      ==> WakeUpCounter = Wakeup Time / Wakeup Time Base

	    To configure the wake up timer to 60s the WakeUpCounter is set to 0xFFFF:
	    Wakeup Time Base = 16 /(~32.000KHz) = ~0.5 ms
	    Wakeup Time = 0.5 ms  * WakeUpCounter
	    Therefore, with wake-up counter =  0xFFFF  = 65,535
	       Wakeup Time =  0,5 ms *  65,535 = 32,7675 s ~ 33 sec. */
	/*  Wakeup Time = 0.5 ms  * WakeUpCounter
	 *  WakeUpCounter = WakeUpTime(ms)/0.5ms*/
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, counter_value, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0);
}

/**
  * @brief  RTC wakeup timer callback
  * @param  htim : TIM IC handle
  * @retval None
*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_Calendar.alarmOccured = 1;
	rtc_Calendar.timerOccured = 1;
	rtc_Calendar.timerState   = TIMER_ELAPSED;
	if ((1 == params_get_mqtt_dc_on()) && (1 == params_pulses_on()))
	{
//		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Timer Elapsed.\r\n", (int)Tick_Get( SECONDS ));
		sensor_log_write_lock(0);
		shutdown_set_start_count(0);
	}
}

/**
  * @brief  Alarm A callback.
  * @param  hrtc RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_Calendar.alarmOccured = 1;
	rtc_Calendar.alarmState   = ALARM_ELAPSED;
//	LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> Alarm A Elapsed.\r\n", (int)Tick_Get( SECONDS ));
}

/**
  * @brief  Alarm B callback.
  * @param  hrtc RTC handle
  * @retval None
  */
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
	rtc_system_setAlarmProgrammedReset(1);
}

/**
 * @}
 */ //End defgroup Sytem_RTC_System
