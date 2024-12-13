/**
  ******************************************************************************
  * @file           rtc_system.h
  * @author 		Datakorum Development Team
  * @brief          Header file for rtc_system.c
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
 * rtc.h
 *
 *  Created on: 15 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_USER_COMMON_INC_RTC_SYSTEM_H_
#define APPLICATION_USER_COMMON_INC_RTC_SYSTEM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "rtc.h"
#include "tick.h"
#include "params.h"

#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0xF9

#define ALARMS_NUM 24

#define TIMER_STARTED (1)
#define TIMER_ELAPSED (2)

#define ALARM_STARTED (1)
#define ALARM_ELAPSED (2)

typedef enum {
	ALARM_NONE,
	ALARM_INIT_READ,
	ALARM_END_READ,
	ALARM_CYCLE,
	ALARM_SEND,
	ALARM_PARAMS_SEND,
	ALARM_LAST = 0xFF
} current_alarm;

typedef enum {
	ALARM_SENSOR_NONE,
	ALARM_SENSOR_INIT_READ,
	ALARM_SENSOR_END_READ,
	ALARM_SENSOR_CYCLE,
	ALARM_SENSOR_SEND,
	ALARM_SENSOR_LAST = 0xFF
} current_sensor_alarm;

typedef enum {
	ALARM_MODBUS_NONE,
	ALARM_MODBUS_INIT_READ,
	ALARM_MODBUS_END_READ,
	ALARM_MODBUS_CYCLE,
	ALARM_MODBUS_SEND,
	ALARM_MODBUS_LAST = 0xFF
} current_modbus_alarm;

typedef enum {
	ALARM_PARAMS_NONE,
	ALARM_PARAMS_SEND_PARAMS,
	ALAR_PARMS_LAST = 0xFF
} current_params_alarm;

/** System Timing structure */
typedef struct {
	RTC_TimeTypeDef Time;							/*!< RTC Time structure  */
	RTC_DateTypeDef Date;							/*!< RTC Date structure  */
	char            created_time[15];				/*!<   */
	char            created_time_telegram[20];		/*!<   */
	char            created_meter_value[7];			/*!<   */
	char            created_meter_value_date[9];	/*!<   */
	char            created_sensor_value[7];		/*!< Holds sensor measure time stamp HHmmss  */
	char            created_sensor_value_date[9];	/*!< Holds sensor measure date YYYYMMdd */
	char            created_modbus_value[7];		/*!< Holds modbus measure time stamp HHmmss */
	char            created_modbus_value_date[9];	/*!< Holds modbus measure date YYYYMMdd */
} SysTime;
SysTime Sys_time;

typedef enum {
	GOT_TIME_SERVER = 0,
	WAITING_TIME_SERVER,
	REQUESTING_TIME_SERVER,
}time_server;

uint32_t rtc_system_get_FreqLSI( void );
void     rtc_system_config(void);
void     rtc_system_configStopMode(void);
void     rtc_system_ReconfigStopMode(void);
void     rtc_system_configStopModeWhenElapses(void);
uint8_t  rtc_system_InitByServer( void );
void     rtc_system_resetInitByServer( void );
void     rtc_system_setBackUpRegister( uint8_t _register, uint32_t data );
uint32_t rtc_system_readBackUpRegister( uint32_t _register );
uint32_t rtc_system_time_to_reset( uint32_t _diff_in_secs );

void     rtc_system_setReadAlarmInitTime( uint8_t alarm_num, uint8_t _read_init_time );
uint8_t  rtc_system_getReadAlarmInitTime( uint8_t alarm_num );
void     rtc_system_setReadAlarmEndTime( uint8_t alarm_num, uint8_t _read_end_time );
uint8_t  rtc_system_getReadAlarmEndTime( uint8_t alarm_num );
void     rtc_system_setReadAlarmCycle( uint8_t alarm_num, uint8_t _cycle );
uint8_t  rtc_system_getReadAlarmCycle( uint8_t alarm_num );
void 	 rtc_system_setReadAlarmInitNext( uint8_t alarmNext );
uint8_t  rtc_system_getReadAlarmInitNext( void );
void 	 rtc_system_setReadAlarmEndNext( uint8_t alarmNext );
uint8_t  rtc_system_getReadAlarmEndNext( void );
void     rtc_system_setReadAlarmCycleNext( uint8_t alarmNext );
uint32_t rtc_system_getReadAlarmCycleNextInSeconds( void );
uint32_t rtc_system_getReadAlarmCycleNext( void );
uint8_t  rtc_system_getReadAlarmNum( void );
void     rtc_system_setReadAlarmNum( uint8_t alarmNum );
void     rtc_system_setReadSensorAlarmInitTime( uint8_t alarm_num, uint8_t _read_init_time );
uint8_t  rtc_system_getReadSensorAlarmInitTime( uint8_t alarm_num );
void     rtc_system_setReadSensorAlarmEndTime( uint8_t alarm_num, uint8_t _read_end_time );
uint8_t  rtc_system_getReadAlarmEndTime( uint8_t alarm_num );
void     rtc_system_setReadSensorAlarmCycle( uint8_t alarm_num, uint16_t _cycle );
uint8_t  rtc_system_getReadSensorAlarmCycle( uint8_t alarm_num );
void     rtc_system_setReadSensorAlarmInitNext( uint8_t alarmInitNext );
uint8_t  rtc_system_getReadSensorAlarmInitNext( void );
void     rtc_system_setReadSensorAlarmEndNext( uint8_t alarmNext );
uint8_t  rtc_system_getReadSensorAlarmEndNext( void );
void     rtc_system_setReadSensorAlarmCycleNext( uint16_t alarmNext );
uint32_t rtc_system_getReadSensorAlarmCycleNextInSeconds( void );
uint32_t rtc_system_getReadSensorAlarmCycleNext( void );
uint8_t  rtc_system_getReadSensorAlarmNum( void );
void 	 rtc_system_setReadSensorAlarmNum( uint8_t alarmNum );
void     rtc_system_setReadModbusAlarmInitTime( uint8_t alarm_num, uint8_t _read_init_time );
uint8_t  rtc_system_getReadModbusAlarmInitTime( uint8_t alarm_num );
void     rtc_system_setReadModbusAlarmEndTime( uint8_t alarm_num, uint8_t _read_end_time );
uint8_t  rtc_system_getReadModbusAlarmEndTime( uint8_t alarm_num );
void     rtc_system_setReadModbusAlarmCycle( uint8_t alarm_num, uint16_t _cycle );
uint8_t  rtc_system_getReadModbusAlarmCycle( uint8_t alarm_num );
void     rtc_system_setReadModbusAlarmInitNext( uint8_t alarmInitNext );
uint8_t  rtc_system_getReadModbusAlarmInitNext( void );
void     rtc_system_setReadModbusAlarmEndNext( uint8_t alarmNext );
uint8_t  rtc_system_getReadModbusAlarmEndNext( void );
void     rtc_system_setReadModbusAlarmCycleNext( uint16_t alarmNext );
uint32_t rtc_system_getReadModbusAlarmCycleNextInSeconds( void );
uint32_t rtc_system_getReadModbusAlarmCycleNext( void );
uint8_t  rtc_system_getReadModbusAlarmNum( void );
void     rtc_system_setReadModbusAlarmNum( uint8_t alarmNum );
void 	 rtc_system_setSendAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time );
uint8_t  rtc_system_getSendAlarmSendTime( uint8_t alarm_num );
void     rtc_system_setSendAlarmSendNext( uint8_t alarmNext );
uint8_t  rtc_system_getSendAlarmSendNext( void );
void 	 rtc_system_setSendAlarmNum( uint8_t alarmNum );
void     rtc_system_setSendSensorAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time );
uint8_t  rtc_system_getSendSensorAlarmSendTime( uint8_t alarm_num );
void     rtc_system_setSendSensorAlarmSendNext( uint8_t alarmNext );
uint8_t  rtc_system_getSendSensorAlarmSendNext( void );
void     rtc_system_setSendSensorAlarmNum( uint8_t alarmNum );
void     rtc_system_setSendModbusAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time );
uint8_t  rtc_system_getSendModbusAlarmSendTime( uint8_t alarm_num );
void     rtc_system_setSendModbusAlarmSendNext( uint8_t alarmNext );
uint8_t  rtc_system_getSendModbusAlarmSendNext( void );
void     rtc_system_setSendModbusAlarmNum( uint8_t alarmNum );
void 	 rtc_system_setSendParamsAlarmSendTime( uint8_t alarm_num, uint8_t _send_init_time );
void 	 rtc_system_setSendParamsAlarmSendNext( uint8_t alarmNext );
uint8_t  rtc_system_getSendParamsAlarmSendNext( void );
void 	 rtc_system_setSendParamsAlarmNum( uint8_t alarmNum );
void     rtc_system_set_t_seconds_backup(void);
uint32_t rtc_system_get_t_seconds_backup( void );
void     rtc_system_setAlarmNext( uint8_t alarmNext );
uint8_t  rtc_system_getAlarmNext( void );
void     rtc_system_setAlarmsNum( uint8_t alarmsNum );
uint8_t  rtc_system_getAlarmsNum( void );
void     rtc_system_setAlarmOccured( uint8_t alarmOccured );
uint8_t  rtc_system_getAlarmOccured( void );
uint8_t  rtc_system_checkAlarmOccured( void );
void     rtc_system_setAlarmState( uint8_t alarmState );
uint8_t  rtc_system_getAlarmState( void );
void     rtc_system_setTimerState( uint8_t alarmState );
uint8_t  rtc_system_getTimerState( void );
void     rtc_system_setGetTime( uint8_t __getTime );
uint8_t  rtc_system_getGetTime( void );
void     rtc_system_setAlarmProgrammedReset( uint8_t programmedReset );
uint8_t  rtc_system_getAlarmProgrammedReset( void );
void     rtc_system_setWaitingTimeServer( uint8_t waitingTimeServer );
uint8_t  rtc_system_getWaitingTimeServer( void );
char *   rtc_system_getCreatedTimeFileName( void );
char *   rtc_system_getCreatedTimeTelegram( void );
char *   rtc_system_GetCreatedTime( time_t server_time );
void     rtc_system_SetServerTime( time_t server_time );
uint32_t rtc_system_get_billing_day( void );
uint32_t rtc_system_get_event_log_hour( void );
uint32_t rtc_system_get_load_profile_hour( void );
void 	 rtc_system_set_billing_day( uint32_t _bill_st );
void 	 rtc_system_set_event_log_hour( uint32_t _ev_st );
void     rtc_system_set_load_profile_hour( uint32_t _ld_prof_st );
uint32_t rtc_system_GetServerTime( void );
time_t   rtc_system_get_time_in_miliseconds( void );
void     rtc_system_SetCreatedMessageTime( time_t server_time );
char   * rtc_system_getCreatedMessageTime( void );
void     rtc_system_SetCreatedMeterValueTime( time_t server_time );
char   * rtc_system_getCreatedMeterValueTime( void );
void     rtc_system_SetCreatedMeterValueDate( time_t server_time );
char   * rtc_system_getCreatedMeterValueDate( void );
void     rtc_system_SetCreatedSensorValueTime( time_t server_time );
char   * rtc_system_getCreatedSensorValueTime( void );
char   * rtc_system_getCreatedSensorValueTimeFromFirstMeas(time_t first_value, uint32_t msg_num);
void     rtc_system_SetCreatedSensorValueDate( time_t server_time );
char   * rtc_system_getCreatedSensorValueDate( void );
void     rtc_system_SetCreatedModbusValueTime( time_t server_time );
char   * rtc_system_getCreatedModbusValueTime( void );
void     rtc_system_SetCreatedModbusValueTime( time_t server_time );
char   * rtc_system_getCreatedModbusValueTime( void );
void     rtc_system_SetCreatedModbusValueDate( time_t server_time );
char   * rtc_system_getCreatedModbusValueDate( void );
int8_t   rtc_system_localUTCTime( uint32_t *hour );
int8_t   rtc_system_UTCTime( uint32_t *hour );
uint32_t rtc_system_getReadInitAlarmFromBKUP( void );
uint32_t rtc_system_getReadInitAlarm( void );
uint32_t rtc_system_getReadEndAlarmFromBKUP( void );
uint32_t rtc_system_getReadEndAlarm( void );
uint32_t rtc_system_getSendAlarmFromBKUP( void );
uint32_t rtc_system_getSendAlarm( void );
uint32_t rtc_system_getCurrentAlarmFromBKUP( void );
void     rtc_system_setCurrentSensorAlarm( uint32_t _current_alarm );
uint32_t rtc_system_getCurrentSensorAlarm( void );
void     rtc_system_setCurrentModbusAlarm( uint32_t _current_alarm );
uint32_t rtc_system_getCurrentModbusAlarm( void );
uint32_t rtc_system_getCurrentAlarm( void );
void     rtc_system_setCurrentAlarm( uint32_t _current_alarm );
uint32_t rtc_system_getCurrentParamsAlarm( void );
void     rtc_system_setCurrentParamsAlarm( uint32_t _current_alarm );
int32_t  rtc_system_SetReadTimeInitTimeAlarm( void );
int32_t  rtc_system_SetSendTimeInitTimeAlarm( void );
void 	 rtc_System_InitRemainingTime( void );
int32_t  rtc_system_SetAlarm( time_t sys_time_ms );
void     rtc_system_reset_remaining_time( void );
int32_t  rtc_system_SetPulseTimer(void);
int32_t  rtc_system_SetPulseAlarm(void);
void     rtc_system_reset_remaining_time_last_pulse( void );
uint32_t rtc_system_last_pulse( void );
int32_t  rtc_system_SetQuarterAlarm();
int32_t  rtc_system_SetSyncAlarm( uint32_t read_time );
int32_t  rtc_system_SetPressureOnSyncAlarm( uint32_t read_time );
int32_t  rtc_system_SetResetAlarm( void );
int32_t  rtc_system_SetDelaySendAlarm( void );
uint32_t rtc_system_set_timeout_waiting_pulse( void );
uint32_t rtc_system_reset_timeout_waiting_pulse( void );
uint32_t rtc_system_get_timeout_waiting_pulse( void );
void     rtc_system_set_wake_up_time( void );
void     rtc_system_EnterSTANDBYMode(void);
void     rtc_system_Set( uint32_t t_seconds );
void 	 rtc_system_Set_Stop_Mode( uint32_t t_seconds );
void     rtc_system_Set_Stop_Mode_RTCCLK_DIV16(uint32_t wake_up_time_ms);


#endif /* APPLICATION_USER_COMMON_INC_RTC_SYSTEM_H_ */
