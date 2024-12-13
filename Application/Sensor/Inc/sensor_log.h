/**
  ******************************************************************************
  * @file           sensor_log.h
  * @author 		Datakorum Development Team
  * @brief          Header file for sensor_log.c
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
 * sensor_log.h
 *
 *  Created on: 14 oct. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_SENSOR_INC_SENSOR_LOG_H_
#define APPLICATION_SENSOR_INC_SENSOR_LOG_H_

#include <math.h>

#include "stm32u5xx_hal.h"
#include "adc.h"
#include "params.h"

#ifdef EXT_SENSOR
#define SENSOR_ID                    (1)
#define MAX_PRESSURE_TELEGRAM_VALUES (1500)//(352)

void     sensor_log_set_save_programword( uint32_t __save );
uint32_t sensor_log_check_set_programword( uint32_t _diff_time_in_secs );
void     sensor_log_init(void);
uint32_t sensor_log_get_write_pulse_acc(void);
void     sensor_log_set_write_pulse_acc(uint32_t _acc);
float_t *sensor_log_GetPressureValues( void );
float_t *sensor_log_GetPressureIndexValue( uint32_t index);
uint8_t  sensor_log_GetPressureValuesCurrentIndex( void );
char    *sensor_log_GetDatalogger_Info(void);
uint8_t  sensor_log_WritePressureLogIndex( uint32_t index );
uint32_t sensor_log_is_pressure_log_init( void );
uint8_t  sensor_log_CheckDataInMem( void );
void     sensor_log_recover_rd_address_backup( void );
uint32_t sensor_log_GetOverPressureAlarm( void );
uint32_t sensor_log_GetLowPressureAlarm( void );
uint32_t sensor_log_GetOperatingAlarm( void );
void 	 sensor_log_SetOperatingAlarm( uint32_t _operating_alarm );
char   * sensor_log_GetPressureAvg( void );
char   * sensor_log_GetPressureMin( void );
char   * sensor_log_GetPressureMax( void );
char   * sensor_log_get_pressure( void );
void     sensor_log_Log( void );
float_t  AD_CheckAlarms( void );
char   * sensor_log_message_value_pressure_sensor( void );
void     sensor_log_pressure_store_telegram_value( void );
char   * sensor_log_pressure_instant_pulses_telegram( uint32_t * measure_count, uint32_t pulses_on, uint32_t pressure_on );
char   * sensor_log_pressure_telegram( uint32_t *measure_count, uint32_t *sensor_id );
char   * sensor_log_pressure_params( uint32_t *measure_count );
void     sensor_log_pressure_reset_acc_values( void );
void     sensor_log_pressure_reset_telegram_values( void );
void     sensor_log_write_lock( uint8_t _write_lock );
uint32_t sensor_log_pressure_register( void );
#endif

#endif /* APPLICATION_SENSOR_INC_SENSOR_LOG_H_ */
