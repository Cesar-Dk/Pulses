/*
 * generic_sensor_log.h
 *
 *  Created on: 26 oct. 2023
 *      Author: Sergio Millán López
 */

#ifndef SENSOR_INC_GENERIC_SENSOR_LOG_H_
#define SENSOR_INC_GENERIC_SENSOR_LOG_H_

#include <math.h>

#include "stm32u5xx_hal.h"
#include "params.h"

#define MAX_GENERIC_SENSOR_TELEGRAM_VALUES (1500)

void    generic_sensor_log_set_save_programword( uint32_t __save );
uint32_t generic_sensor_log_check_set_programword( uint32_t _diff_time_in_secs );
void    generic_sensor_log_init( void );
void    generic_sensor_log_read_sensor( uint32_t num );
void    generic_sensor_log_recover_rd_address_backup( uint32_t num );
uint8_t generic_sensor_log_CheckDataInMem( uint32_t num );

uint32_t generic_sensor_log_GetOverGenericSensorAlarm( uint32_t num );
uint32_t generic_sensor_log_GetLowGenericSensorAlarm( uint32_t num );
uint32_t generic_sensor_log_GetOperatingSensorAlarm( uint32_t num );
void 	 generic_sensor_log_SetOperatingSensorAlarm( uint32_t num, uint32_t _operating_alarm );
char   * generic_sensor_log_GetGenericSensorAvg( uint32_t num );
char   * generic_sensor_log_GetGenericSensorMin( uint32_t num );
char   * generic_sensor_log_GetGenericSensorMax( uint32_t num );

void  generic_sensor_log_Log(uint32_t num);
char *generic_sensor_log_GetDatalogger_Info( void );
char *generic_sensor_log_GetPressureAvg( uint32_t num );
char *generic_sensor_log_GetPressureMin( uint32_t num );
char *generic_sensor_log_GetPressureMax( uint32_t num );
char *generic_sensor_log_message_value_sensor( uint32_t num );
void  generic_sensor_log_pressure_store_telegram_value(uint32_t num);
char *generic_sensor_log_sensor_num_telegram( uint32_t *measure_count, uint32_t *sensor_id, uint32_t num );
void  generic_sensor_log_pressure_reset_telegram_values( uint32_t num );

#endif /* SENSOR_INC_GENERIC_SENSOR_LOG_H_ */
