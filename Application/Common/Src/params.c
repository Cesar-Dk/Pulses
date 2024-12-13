/**
  ******************************************************************************
  * @file           params.c
  * @author 		Datakorum Development Team
  * @brief          General system parameters driver. This file provides APIs to
  * handle internal FLASH operations for general system parameters management.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution SL.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * params.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Sergio Mill�n L�pez
 */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>

#include "stm32u5xx_hal.h"
#include "main.h"
#include "params.h"
#include "crc32.h"
#include "tick.h"
#include "ME910.h"
#include "generic_modbus.h"

///* !!! Be careful the user area should be in another bank than the code !!! */
//#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_0   						/* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_255 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */


/** @defgroup System_Parameters System Parameters
  * @brief Parameters are stored in a 8k partition of internal uC FLASH memory
  * @note
  * @{
  */

struct params param;
/* To adjust parameters size to 2048 */
ct_assert( sizeof(param) == (uint32_t) ( FLASH_PAGE_SIZE ) );

uint8_t pages_n, page_i;
static uint8_t params_get_values( void );

uint32_t pulse_acc_backup = 0, pulse_write = 0;
static char device_battery[24];
extern char _params_size[], _params_start_address[];

/** @defgroup System_Parameters_version System Parameters Firmware Version
  * @{
  */

/**
 * @fn uint32_t params_version_major(void)
 * @brief Returns the major firmware version.
 *
 * @return Returns the major firmware version.
 */
uint32_t params_version_major( void )
{
	return param.version.major;
}

/**
  * @brief  Returns the minor firmware version.
  * @retval Returns the minor firmware version.
  */
uint32_t params_version_minor( void )
{
	return param.version.minor;
}

/**
  * @}
  */


/** @defgroup System_Parameters_configuration System Parameters Configuration
 *  @brief Parameter to configure the system
 * @{
 *  */

/**
 * @fn uint32_t params_config_send_time(void)
 * @brief Gets the time elapsed programmed between transmissions.
 *
 * @return sending time period.
 */
uint32_t params_config_send_time(void)
{
    return param.config.st;
}

/**
 * @fn void params_config_set_send_time(uint32_t)
 * @brief Sets the time elapsed programmed between transmissions.
 *
 * @param _send_time time period between transmissions.
 */
void params_config_set_send_time(uint32_t _send_time)
{
    param.config.st = _send_time;
}

/**
 * @fn uint32_t params_config_read_time(void)
 * @brief Gets the time elapsed programmed between sensor reading samples.
 *
 * @return sensor reading time period.
 */
uint32_t params_config_read_time(void)
{
    if ( 1 == params_config_get_disable_meter() )
    {
    	return 0;
    }
    else
    {
    	return param.config.rt;
    }
}

/**
 * @fn void params_config_set_read_time(uint32_t)
 * @brief Sets the time elapsed programmed between sensor reading samples.
 *
 * @param _read_time  sensor reading time period.
 */
void params_config_set_read_time(uint32_t _read_time)
{
    param.config.rt = _read_time;
}

/**
 * @fn uint32_t params_config_send_hour(uint8_t)
 * @brief
 *
 * @param num
 * @return
 *
 * @note NOT USED
 */
uint32_t params_config_send_hour(uint8_t num)
{
    return param.config.sh[num];
}

/**
 * @fn uint32_t params_config_alarms_num(void)
 * @brief
 *
 * @return
 *
 * @note NOT USED
 */
uint32_t params_config_alarms_num(void)
{
    return sizeof(param.config.sh);
}


/**
  * @brief  Send count is the number of meter readings before a transmission.
  * @retval Returns Send Messages Counter.
  */
uint32_t params_config_send_count(void)
{
    return param.config.send_count;
}

/**
  * @brief  Updates the number of meter readings before a transmission.
  * @param _send_count value to update.
  */
void params_config_set_send_count(uint32_t _send_count)
{
    param.config.send_count = _send_count;
}

/**
 * @fn void params_config_set_sensor_log(uint8_t)
 * @brief Set the frame type used for transmission.
 *
 * @param _sensor_log_on @arg 0 - Reduced format.
 * 						 @arg 1 - Extended format.
 *
 * @note The reduced format only sends 1 P+F frame with sensor values every 15minutes included of the P frame.
 * The Gateway only waits for a single ACK.
 * In the extended format the gateway send P, S and F frames independently. In this mode the sensor values taken
 * every 30 seconds are included on the S frame. The Gateway expects 3 ACK, one for frame.
 *
 */
void params_config_set_sensor_log(uint8_t _sensor_log_on)
{
    param.config.ti1 = _sensor_log_on;
}

/**
 * @fn uint32_t params_config_get_sensor_log(void)
 * @brief Get the frame type used for transmission.
 *
 * @return @arg 0 - Reduced format.
 * 			@arg 1 - Extended format.
 *
 * @note The reduced format only sends 1 P+F frame with sensor values every 15minutes included of the P frame.
 * The Gateway only waits for a single ACK.
 * In the extended format the gateway send P, S and F frames independently. In this mode the sensor values taken
 * every 30 seconds are included on the S frame. The Gateway expects 3 ACK, one for frame.
 *
 */
uint32_t params_config_get_sensor_log(void)
{
    return param.config.ti1;
}

/**
 * @fn void params_config_set_period(uint8_t)
 * @brief Sets the sensing and transmission mode, whether to use window based or continuous data transmissions.
 *
 * @param _period @arg 0 - Window based sensor readings and data transmissions.
 * 				  @arg 1 - Periodic sensor read and data transmissions given by parameters param.config.rt and
 * 				  param.config.st respectively.
 */
void params_config_set_period(uint8_t _period)
{
    param.config.ti2 = _period;
}

/**
 * @fn uint32_t params_config_get_period(void)
 * @brief returns the sensing and transmission mode.
 *
 * @return _period @arg 0 - Window based sensor readings and data transmissions.
 * 				  @arg 1 - Periodic sensor read and data transmissions given by parameters param.config.rt and
 * 				  param.config.st respectively.
 */

uint32_t params_config_get_period(void)
{
    return param.config.ti2;
}

/**
 * @fn void params_config_set_adc_on(uint8_t)
 * @brief Enables the use of the ADC to read an external analog sensor.
 *
 * @param _adc_on @arg 0 - ADC disabled, external sensor enabled.
 * 				  @arg 1 - ADC enabled, external sensor enabled.
 */
void params_config_set_adc_on(uint8_t _adc_on)
{
    param.config.idd = _adc_on;
}

/**
 * @fn uint32_t params_config_get_adc_on(void)
 * @brief Checks whether the external sensor is enabled or disabled.
 *
 * @return _adc_on @arg 0 - ADC disabled, external sensor enabled.
 * 				  @arg 1 - ADC enabled, external sensor enabled.
 */
uint32_t params_config_get_adc_on(void)
{
    return param.config.idd;
}

/**
 * @fn void params_config_set_disable_meter(uint8_t)
 * @brief Disables/Enables the water meter reading.
 *
 * @param _disable_meter @arg 0 - Water meter is enabled.
 * 						 @arg 1 - Water meter is disabled.
 */
void params_config_set_disable_meter(uint8_t _disable_meter)
{
    param.config.idg = _disable_meter;
}

/**
 * @fn uint32_t params_config_get_disable_meter(void)
 * @brief
 *
 * @return _disable_meter @arg 0 - Water meter is enabled.
 * 						 @arg 1 - Water meter is disabled.
 */
uint32_t params_config_get_disable_meter(void)
{
    return param.config.idg;
}

/**
 * @fn void params_config_set_sync_read(uint8_t)
 * @brief Enables/Disables the time synchronization sensor reading. When enabled the device performs synchronized reading with minitues 0, 15
 * 30 and/or 45 of each hour depending on the sensor reading period assigned.
 *
 * @param _sync_read @arg 0 - Disabled.
 * 					 @arg 1 - Enabled.
 */
void params_config_set_sync_read(uint8_t _sync_read)
{
    param.config.idp = _sync_read;
}

/**
 * @fn uint32_t params_config_get_sync_read(void)
 * @brief Reads the synchronization sensor reading configuration. When enabled the device performs synchronized reading with minitues 0, 15
 * 30 and/or 45 of each hour depending on the sensor reading period assigned.
 *
 * @return _sync_read @arg 0 - Disabled.
 * 					 @arg 1 - Enabled.
 */
uint32_t params_config_get_sync_read( void )
{
    return param.config.idp;
}

#if 0
void params_config_set_generic_modbus_send_time( uint8_t _generic_modbus_send_time )
{
    param.config.idp = _generic_modbus_send_time;
}

uint32_t params_config_get_generic_modbus_send_time( void )
{
    return param.config.idp;
}
#endif

/**
 * @fn void params_set_telegram_mode(uint32_t)
 * @brief Sets the type of telegram to transmit.
 *
 * @param mode @arg TELEGRAM_MODE_RAW
 * 			   @arg TELEGRAM_MODE_DK
 */
void params_set_telegram_mode( uint32_t mode )
{
	param.telegram_mode = mode;
}

/**
 * @fn uint32_t params_telegram_mode(void)
 * @brief Gets the type of telegram to transmit.
 *
 * @return mode @arg TELEGRAM_MODE_RAW
 * 			   @arg TELEGRAM_MODE_DK
 */
uint32_t params_telegram_mode( void )
{
	return param.telegram_mode;
}

/**
 * @fn void params_set_protocol_mode(uint32_t)
 * @brief Sets the connection communication protocol.
 *
 * @param mode @arg PROTOCOL_TCP
 * 	           @arg PROTOCOL_UDP
 */
void params_set_protocol_mode( uint32_t mode )
{
	param.protocol_mode = mode;
}

/**
 * @fn uint32_t params_protocol_mode(void)
 * @brief Gets the connection communication protocol.
 *
 * @return mode @arg PROTOCOL_TCP
 * 	           @arg PROTOCOL_UDP
 */
uint32_t params_protocol_mode( void )
{
	return param.protocol_mode;
}

/**
 * @fn void params_set_nbiotpwr_mode(uint32_t)
 * @brief Sets the power mode configuration for the radio communications.
 *
 * @param mode @arg NBIOT_PWR_ONOFF
 * 	           @arg NBIOT_PWR_PSM
 */
void params_set_nbiotpwr_mode( uint32_t mode )
{
	param.nbiotpwr_mode = mode;
}

/**
 * @fn uint32_t params_nbiotpwr_mode(void)
 * @brief Gets the power mode configuration for the radio communications.
 *
 * @return mode @arg NBIOT_PWR_ONOFF
 * 	           @arg NBIOT_PWR_PSM
 */
uint32_t params_nbiotpwr_mode( void )
{
	return param.nbiotpwr_mode;
}

/**
 * @}
 *  */ //End defgroup System_Parameters_configuration

/**
 * @defgroup System_Parameters_Pulses System Parameters Pulses
 * @brief
 * @{
 */

/**
  * @brief  Set whether pulses feature is enable or not.
  * @param _on @arg 0 - Pulses disabled.
  * 		   @arg 1 - Pulses enabled.
  *
  * @retval Returns pulses enable flag.
  */
void params_pulses_set_on( uint32_t _on )
{
	param.pulses.on = _on;
}

/**
  * @brief  Returns whether pulses feature is enabled or not.
  * @retval _on @arg 0 - Pulses disabled.
  * 		   @arg 1 - Pulses enabled.
  *
  */
uint32_t params_pulses_on( void )
{
	return param.pulses.on;
}

/**
 * @fn uint32_t params_pulses_pulse_factor(void)
 * @brief
 *
 * @return
 */
uint32_t params_pulses_pulse_factor( void )
{
	return param.pulses.pulse_factor;
}

/**
 * @fn uint32_t params_pulses_pulse_unit_k_factor_out_1(void)
 * @brief
 *
 * @return
 */
uint32_t params_pulses_pulse_unit_k_factor_out_1( void )
{
	return param.pulses.unit_k_factor_out_1;
}

/**
 * @fn uint32_t params_pulses_pulse_unit_k_factor_out_2(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t params_pulses_pulse_unit_k_factor_out_2( void )
{
	return param.pulses.unit_k_factor_out_2;
}

/**
 * @fn void params_pulses_set_pulse_factor(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _pulse_factor
 */
void params_pulses_set_pulse_factor( uint32_t _pulse_factor )
{
	param.pulses.pulse_factor = _pulse_factor;
}

/**
 * @fn void params_pulses_set_pulse_unit_k_factor_out_1(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _unit_k_factor
 */
void params_pulses_set_pulse_unit_k_factor_out_1( uint32_t _unit_k_factor )
{
	param.pulses.unit_k_factor_out_1 = _unit_k_factor;
}

/**
 * @fn void params_pulses_set_pulse_unit_k_factor_out_2(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _unit_k_factor
 */
void params_pulses_set_pulse_unit_k_factor_out_2( uint32_t _unit_k_factor )
{
	param.pulses.unit_k_factor_out_2 = _unit_k_factor;
}

/**
 * @}
 */ // End defgroup System_Parameters_Pulses


/**
 * @defgroup System_Parameters_Water_Quality System Parameters Water Quality
 * @brief
 * @{
 */

/**
  * @brief  Set whether water quality is enabled or not.
  * @param _on
  * 	@arg 0 - Water Quality disabled.
  *     @arg 1 - Water Quality enabled.
  *
  * @retval Returns water quality enable flag.
  */
void params_wq_set_on( uint32_t _on )
{
	param.wq = _on;
}

/**
  * @brief  Returns whether water quality is enabled or not.
  * @retval  _on @arg 0 - Water Quality disabled.
  *            @arg 1 - Water Quality enabled.
  */
uint32_t params_wq_on( void )
{
	return param.wq;
}

/**
 *  @}
 */

/**
 * @defgroup System_Parameters_Modbus System Parameters Modbus
 * @brief
 * @{
 */

/**
 * @defgroup System_Parameters_Modbus_LocalTool System Parameters Modbus Local Tool
 * @brief
 * @{
 */

/**
 * @fn uint8_t params_modbus_log_enabled(void)
 * @brief Checks whether local tool or Modbus is enabled or disabled.
 *
 * @return enable
 * 		@arg 0 - Local Port or Modbus is Disabled.
 * 		@arg 1 - Local Port or Modbus is Enabled.
 */
uint8_t params_modbus_log_enabled( void )
{
	return param.modbus_log.enabled;
}

/**
 * @fn void params_modbus_log_set_enabled(uint8_t)
 * @brief Enables local port or Modbus.
 *
 * @param enable
 * 		@arg 0 - Local Port or Modbus is Disabled.
 * 		@arg 1 - Local Port or Modbus is Enabled.
 */
void params_modbus_log_set_enabled( uint8_t enable )
{
	param.modbus_log.enabled = enable;
}

/**
 * @fn uint8_t params_modbus_log_prev_enabled(void)
 * @brief Indicates whether the localTool is activated or not
 *
 * @return enable
 * 		@arg 0 - Local Tool is Disable.
 * 		@arg 1 - Local Tool is Enable.
 */
uint8_t params_modbus_log_prev_enabled( void )
{
	return param.modbus_log.prev_enabled;
}

/**
 * @fn void params_modbus_log_set_prev_enabled(uint8_t)
 * @brief
 *
 * @param enable
 * 		@arg 0 - Local Tool is Disable.
 * 		@arg 1 - Local Tool is Enable.
 *
 * @note NOT USED.
 * @note Local Tool can only be activated after a harware reset.
 *
 */
void params_modbus_log_set_prev_enabled( uint8_t enable )
{
	param.modbus_log.prev_enabled = enable;
}

/**
 * @fn unsigned char params_modbus_get_tool_cert*(void)
 * @brief Returns a pointer to the local tool password certificate.
 *
 * @return pointer to the local tool certificate
 */
unsigned char *params_modbus_get_tool_cert( void )
{
    return param.tool_cert;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup System_Parameters_Low_Power_Configuration System Parameters Low Power Configuration
 * @brief
 * @{
 */

/**
 * @fn void params_psm_edrx_set_tt3412(uint8_t)
 * @brief Set the value for the timer T3412.
 *
 * @param _tt3412
 */
void params_psm_edrx_set_tt3412( uint32_t _tt3412 )
{
	param.psm_edrx.tt3412 = _tt3412;
	param.psm_tt3412      = _tt3412;
}

/**
 * @fn uint8_t params_psm_edrx_tt3412(void)
 * @brief Returns the value for the timer T3412.
 *
 * @return T3412 value
 */
uint32_t params_psm_edrx_tt3412( void )
{
//	return param.psm_edrx.tt3412;
	return param.psm_tt3412;
}

/**
 * @fn void params_psm_edrx_set_tt3324(uint8_t)
 * @brief Sets the value for timer T3324
 *
 * @param _tt3324
 */
void params_psm_edrx_set_tt3324( uint32_t _tt3324 )
{
	param.psm_edrx.tt3324 = _tt3324;
	param.psm_tt3324      = _tt3324;
}

/**
 * @fn uint8_t params_psm_edrx_tt3324(void)
 * @brief Returns the value for the timer T3324.
 *
 * @return T3324 value
 */
uint32_t params_psm_edrx_tt3324( void )
{
//	return param.psm_edrx.tt3324;
	return param.psm_tt3324;
}

/**
 * @fn void params_psm_edrx_set_edrx(uint8_t)
 * @brief Sets the value for the edrx timer.
 *
 * @param _edrx
 */
void params_psm_edrx_set_edrx( uint8_t _edrx )
{
	param.psm_edrx.edrx = _edrx;
}

/**
 * @fn uint8_t params_psm_edrx_edrx(void)
 * @brief Returns the configuration value for the edr timer.
 *
 * @return EDRX time.
 */
uint8_t params_psm_edrx_edrx( void )
{
	return param.psm_edrx.edrx;
}

/**
 * @}
 */

/**
 * @defgroup System_Parameters_Maintenance System Parameters Maintenance
 * @brief
 * @{
 */

/**
 * @fn uint8_t params_manteinance_release_assistance(void)
 * @brief
 *
 * @return
 * @note NOT USED.
 */
uint8_t params_manteinance_release_assistance( void )
{
	return param.manteinance.release_assistance;
}

/**
 * @fn void params_manteinance_set_release_assistance(uint8_t)
 * @brief
 *
 * @param _rai
 * @note NOT USED.
 */
void params_manteinance_set_release_assistance( uint8_t _rai )
{
	param.manteinance.release_assistance = _rai;
}

/**
 * @fn uint8_t params_manteinance_delete_memory(void)
 * @brief
 *
 * @return  _delete_memory
 * 			@arg 0 - Disabled, datalogger is used.
 * 			@arg 1 - Enabled, clears memory only once.
 * 			@arg 2 - Disabled, but datalogger is not sent.
 * 			@arg 4 - CLEAR_MEMORY_DISABLE_HARD_RESET
 */
uint8_t params_manteinance_delete_memory( void )
{
	return param.manteinance.delete_memory;
}

/**
 * @fn void params_manteinance_set_delete_memory(uint8_t)
 * @brief Enables the erase of the external FLASH memory (Datalogger).
 *
 * @param _delete_memory
 * 			@arg 0 - Disabled, datalogger is used.
 * 			@arg 1 - Enabled, clears memory only once.
 * 			@arg 2 - Disabled, but datalogger is not sent.
 *
 *  @note If memory is erased to erase again the memory, the change needs to be forced from the Middleware.
 */
void params_manteinance_set_delete_memory(uint8_t _delete_memory)
{
	param.manteinance.delete_memory = _delete_memory;
}

/**
 * @fn uint8_t params_manteinance_reset_meters(void)
 * @brief Set to erase statistics parameters:
 * 		@arg number of sendings
 * 		@arg number of readings
 * 		@arg attach errors
 * 		@arg attach errors log
 *
 * @return  _reset_meters
 * 		@arg 0 - Do not erase maintenance parameters.
 * 		@arg 1 - Erase maintenance parameters.
 */
uint8_t params_manteinance_reset_meters(void)
{
	return param.manteinance.reset_meters;
}

/**
 * @fn void params_manteinance_set_reset_meters(uint8_t)
 * @brief Check whether it needs to erased statistics parameters:
 * 		@arg number of transmissions.
 * 		@arg number of readings.
 * 		@arg attach errors.
 * 		@arg attach errors log.
 *
 * @param _reset_meters
 * 		@arg 0 - Do not erase maintenance parameters.
 * 		@arg 1 - Erase maintenance parameters.
 */
void params_manteinance_set_reset_meters(uint8_t _reset_meters)
{
	param.manteinance.reset_meters = _reset_meters;
}

/**
 * @fn uint8_t params_manteinance_reed_activation(void)
 * @brief Returns whether the Reed magnetic sensor parameter is enabled.
 *
 * @return Reed activation:
 * 		@arg 0 - Disabled.
 * 		@arg 1 - Enabled.
 */
uint8_t params_manteinance_reed_activation(void)
{
	return param.manteinance.reed_activation;
}

/**
 * @fn void params_manteinance_set_reed_activation(uint8_t)
 * @brief Enables or Disables the Reed magnetic sensor parameter.
 *
 * @param _reed_activation:
 * 		@arg 0 - Disabled.
 * 		@arg 1 - Enabled.
 */
void params_manteinance_set_reed_activation(uint8_t _reed_activation)
{
	param.manteinance.reed_activation = _reed_activation;
}

/**
 * @fn uint32_t params_maintenance_number_readings(void)
 * @brief Gets the total number of readings.
 *
 * @return total number of readings since last memory clear.
 */
uint32_t params_maintenance_number_readings(void)
{
    return param.manteinance.number_readings;
}

/**
 * @fn void params_maintenance_set_number_readings(uint32_t)
 * @brief Sets the total number of readings.
 *
 * @param _number total number or readings.
 */
void params_maintenance_set_number_readings(uint32_t _number)
{
	param.manteinance.number_readings = _number;
}

/**
 * @fn void params_maintenance_inc_number_readings(void)
 * @brief Increments the total number of readings.
 *
 */
void params_maintenance_inc_number_readings(void)
{
	param.manteinance.number_readings++;
}

/**
 * @fn uint32_t params_maintenance_number_ok_sendings(void)
 * @brief Gets the total number of successful transmissions.
 *
 * @return total number of successful transmissions.
 */
uint32_t params_maintenance_number_ok_sendings(void)
{
    return param.manteinance.number_ok_sendings;
}

/**
 * @fn void params_maintenance_set_number_ok_sendings(uint32_t)
 * @brief Sets the total number of successful transmissions.
 *
 * @param _number
 */
void params_maintenance_set_number_ok_sendings(uint32_t _number)
{
	param.manteinance.number_ok_sendings = _number;
}

/**
 * @fn void params_maintenance_inc_number_ok_sendings(void)
 * @brief Increments the total number of successful transmissions.
 *
 */
void params_maintenance_inc_number_ok_sendings(void)
{
	param.manteinance.number_ok_sendings++;
}

/**
 * @fn uint32_t params_maintenance_number_nok_sendings(void)
 * @brief Gets the number of failed transmissions.
 *
 * @return failed transmissions.
 */
uint32_t params_maintenance_number_nok_sendings(void)
{
    return param.manteinance.number_nok_sendings;
}

/**
 * @fn void params_maintenance_set_number_nok_sendings(uint32_t)
 * @brief Set the number of failed transmissions.
 *
 * @param _number failed transmissions.
 */
void params_maintenance_set_number_nok_sendings(uint32_t _number)
{
	param.manteinance.number_nok_sendings = _number;
}

/**
 * @fn void params_maintenance_inc_number_nok_sendings(void)
 * @brief Increments the total number of failed transmissions.
 */
void params_maintenance_inc_number_nok_sendings(void)
{
	param.manteinance.number_nok_sendings++;
}

/**
 * @fn void params_maintenance_set_enable_alarm_sending(uint32_t)
 * @brief Enables/Disables the transmission driven by an alarm.
 *
 * @param _onoff
 * 		@arg 0 - Disabled.
 *      @arg 1 - Enabled.
 *
 * @note If enable an alarm will trigger and immediate transmission.
 *
 */
void params_maintenance_set_enable_alarm_sending(uint32_t _onoff)
{
	param.manteinance.enable_alarm_sending = _onoff;
}

/**
 * @fn uint32_t params_maintenance_enable_alarm_sending(void)
 * @brief Gets the status of the transmission driven by an alarm.
 *
 * @return _onoff
 */
uint32_t params_maintenance_enable_alarm_sending(void)
{
    return param.manteinance.enable_alarm_sending;
}

/**
 * @fn void params_maintenance_set_max_allowed_number_of_sendings(uint32_t)
 * @brief Sets the maximum allowed transmissions before a reset.
 *
 * @param _maxnum
 */
void params_maintenance_set_max_allowed_number_of_sendings(uint32_t _maxnum)
{
	param.manteinance.max_allowed_num_of_sendings = _maxnum;
}

/**
 * @fn uint32_t params_maintenance_max_allowed_number_of_sendings(void)
 * @brief Gets the maximum a.lowed transmission before a reset.
 *
 * @return _maximum number of maximum transmissions.
 */
uint32_t params_maintenance_max_allowed_number_of_sendings(void)
{
    return param.manteinance.max_allowed_num_of_sendings;
}
/**
 * @}
 */

/**
 * @defgroup System_parameters_programming_windows System Parameters Programming Windows
 * @brief
 *
 * @{
 */

uint8_t params_read_time_init_time(uint8_t num)
{
	return param.read_time[num].init_time;
}

void params_read_time_set_init_time(uint8_t num, uint8_t _read_init_time)
{
	param.read_time[num].init_time = _read_init_time;
}

uint8_t params_read_time_end_time(uint8_t num)
{
	return param.read_time[num].end_time;
}

void params_read_time_set_end_time(uint8_t num, uint8_t _read_end_time)
{
	param.read_time[num].end_time = _read_end_time;
}

uint8_t params_read_time_cycle(uint8_t num)
{
	return param.read_time[num].cycle;
}

void params_read_time_set_cycle(uint8_t num, uint8_t _cycle)
{
	param.read_time[num].cycle = _cycle;
}

uint8_t params_send_time_send_time(uint8_t num)
{
	return param.send_time[num].send_time;
}

void params_send_time_set_send_time(uint8_t num, uint8_t _send_init_time)
{
	param.send_time[num].send_time = _send_init_time;
}

uint8_t params_sensor_read_time_init_time(uint8_t num)
{
	return param.sensor_read_time[num].init_time;
}

void params_sensor_read_time_set_init_time(uint8_t num, uint8_t _read_init_time)
{
	param.sensor_read_time[num].init_time = _read_init_time;
}

uint8_t params_sensor_read_time_end_time(uint8_t num)
{
	return param.sensor_read_time[num].end_time;
}

void params_sensor_read_time_set_end_time(uint8_t num, uint8_t _read_end_time)
{
	param.sensor_read_time[num].end_time = _read_end_time;
}

uint16_t params_sensor_read_time_cycle(uint8_t num)
{
	return param.sensor_read_time[num].cycle;
}

void params_sensor_read_time_set_cycle(uint8_t num, uint16_t _cycle)
{
	param.sensor_read_time[num].cycle = _cycle;
}

uint8_t params_sensor_send_time_send_time(uint8_t num)
{
	return param.sensor_send_time[num].send_time;
}

void params_sensor_read_time_set_date_day(uint8_t num, uint32_t _date_day)
{
	param.sensor_read_date_day[num] = _date_day;
}

uint8_t params_sensor_read_time_date_day(uint8_t num)
{
	return param.sensor_read_date_day[num];
}

void params_sensor_read_time_set_date_month(uint8_t num, uint32_t _date_month)
{
	param.sensor_read_date_month[num] = _date_month;
}

uint8_t params_sensor_read_time_date_month(uint8_t num)
{
	return param.sensor_read_date_month[num];
}

void params_sensor_period_date_month_set_period_date_month(uint8_t num, uint32_t _period_date_month)
{
	param.sensor_periodic_send_date_month[num] = _period_date_month;
}

uint8_t params_sensor_period_date_month(uint8_t num)
{
	return param.sensor_periodic_send_date_month[num];
}

void params_sensor_send_time_set_send_time(uint8_t num, uint8_t _send_init_time)
{
	param.sensor_send_time[num].send_time = _send_init_time;
}

void params_input_pulse_as_sensor_set_num(uint8_t _num)
{
	param.input_pulse_as_sensor_num = _num;
}

uint32_t params_input_pulse_as_sensor_get_num(void)
{
	return param.input_pulse_as_sensor_num;
}

uint8_t params_modbus_read_time_init_time(uint8_t num)
{
	return param.modbus_read_time[num].init_time;
}

void params_modbus_read_time_set_init_time(uint8_t num, uint8_t _read_init_time)
{
	param.modbus_read_time[num].init_time = _read_init_time;
}

uint8_t params_modbus_read_time_end_time(uint8_t num)
{
	return param.modbus_read_time[num].end_time;
}

void params_modbus_read_time_set_end_time(uint8_t num, uint8_t _read_end_time)
{
	param.modbus_read_time[num].end_time = _read_end_time;
}

uint16_t params_modbus_read_time_cycle(uint8_t num)
{
	return param.modbus_read_time[num].cycle;
}

void params_modbus_read_time_set_cycle(uint8_t num, uint16_t _cycle)
{
	param.modbus_read_time[num].cycle = _cycle;
}

uint8_t params_modbus_send_time_send_time(uint8_t num)
{
	return param.modbus_send_time[num].send_time;
}

void params_modbus_send_time_set_send_time(uint8_t num, uint8_t _send_init_time)
{
	param.modbus_send_time[num].send_time = _send_init_time;
}

uint8_t params_send_network_params_time_send_time(uint8_t num)
{
	return param.send_params_time[num].send_params_time;
}

void params_send_network_params_time_set_send_time(uint8_t num, uint8_t _send_init_time)
{
	param.send_params_time[num].send_params_time = _send_init_time;
}

float_t params_get_over_sensor_alarm(uint8_t num)
{
	return param.over_sensor_alarm[num];
}

void params_set_over_sensor_alarm(uint8_t num, uint32_t _over_sensor)
{
	param.over_sensor_alarm[num] = _over_sensor;
}

float_t params_get_low_sensor_alarm(uint8_t num)
{
	return param.low_sensor_alarm[num];
}

void params_set_low_sensor_alarm(uint8_t num, uint32_t _over_sensor)
{
	param.low_sensor_alarm[num] = _over_sensor;
}

uint32_t params_get_low_sensor_condition( void )
{
	return param.lowsensor_condition;
}

void params_set_low_sensor_condition(uint32_t _low_sensor)
{
	param.lowsensor_condition = _low_sensor;
}

uint32_t params_get_over_sensor_condition( void )
{
	return param.oversensor_condition;
}

void params_set_over_sensor_condition(uint32_t _over_sensor)
{
	param.oversensor_condition = _over_sensor;
}

uint32_t params_get_relay_status( void )
{
	return param.relay_status;
}

void params_set_relay_status(uint32_t _relay_status)
{
	param.relay_status = _relay_status;
}
/**
 * @}
 */

/**
 * @defgroup System_Parameters_Error System Parameters Error Logs
 * @brief
 * @{
 *  */

/**
 * @fn void params_attach_error_inc_SIM_error(void)
 * @brief Increments SIM error.
 *
 */
void params_attach_error_inc_SIM_error(void)
{
	param.attach_errors.sim_err++;
}

/**
 * @fn void params_attach_error_inc_CSQ_error(void)
 * @brief Increments CSQ errors.
 *
 * @pre
 * @post
 */
void params_attach_error_inc_CSQ_error(void)
{
	param.attach_errors.csq_err++;
}

/**
 * @fn void params_attach_error_inc_CEREG_error(void)
 * @brief Increments Registration errors.
 *
 */
void params_attach_error_inc_CEREG_error(void)
{
	param.attach_errors.cereg_err++;
}

/**
 * @fn void params_attach_error_inc_TSO_error(void)
 * @brief Increments Timeout in Opening Socket
 *
 */
void params_attach_error_inc_TSO_error(void)
{

	param.attach_errors.tso_err++;
}

/**
 * @fn void params_attach_error_inc_TSE_error(void)
 * @brief Increments Server Timeout Error
 *
 */
void params_attach_error_inc_TSE_error(void)
{
	param.attach_errors.tse_err++;
}

/**
 * @fn void params_attach_error_inc_IP_error(void)
 * @brief Increments Server IP error
 *
 */
void params_attach_error_inc_IP_error(void)
{
	param.attach_errors.ip_err++;
}

/**
 * @fn uint32_t params_attach_error_get_SIM_error(void)
 * @brief Returns the number of SIM errors.
 *
 * @return SIM errors.
 */
uint32_t params_attach_error_get_SIM_error(void)
{
	return param.attach_errors.sim_err;
}

/**
 * @fn uint32_t params_attach_error_get_CSQ_error(void)
 * @brief Returns the number of CSQ errors.
 *
 * @return CSQ errors.
 */
uint32_t params_attach_error_get_CSQ_error(void)
{
	return param.attach_errors.csq_err;
}

/**
 * @fn uint32_t params_attach_error_get_CEREG_error(void)
 * @brief Returns the number of network registration errors
 *
 * @return Network registration CEREG errors.
 */
uint32_t params_attach_error_get_CEREG_error(void)
{
	return param.attach_errors.cereg_err;
}

/**
 * @fn uint32_t params_attach_error_get_TSO_error(void)
 * @brief Returns the number of Opening Socket error.
 *
 * @return Opening Socket error.
 */
uint32_t params_attach_error_get_TSO_error(void)
{

	return param.attach_errors.tso_err;
}

/**
 * @fn uint32_t params_attach_error_get_TSE_error(void)
 * @brief Returns the Time Server Errors.
 *
 * @return Time Server Errors.
 */
uint32_t params_attach_error_get_TSE_error(void)
{
	return param.attach_errors.tse_err;
}

/**
 * @fn uint32_t params_attach_error_get_IP_error(void)
 * @brief Returns the number of IP errors.
 *
 * @return IP Errors.
 */
uint32_t params_attach_error_get_IP_error(void)
{
	return param.attach_errors.ip_err;
}

/**
 * @fn void params_attach_insert_log(uint32_t)
 * @brief Adds the corresponding error defined in _log along with the time stamp in the error log list.
 *
 * @param _log type of error to register
 * 			@arg @ref SIM_ERR
 * 			@arg @ref CSQ_ERR
 * 			@arg @ref CREG_ERR
 * 			@arg @ref IP_ERR
 * 			@arg @ref TSO_ERR
 * 			@arg @ref TSE_ERR
 */
void params_attach_insert_log(uint32_t _log)
{
	static uint32_t index = 0;

	param.attach_errors.err_log[index] = _log;
	param.attach_errors.err_log_timestamp[index] = Tick_Get(SECONDS);

	if ( index >= 23 )
	{
		index = 0;
	}
	else
	{
		index++;
	}
}

/**
 * @fn uint32_t params_attach_get_log_index(uint32_t)
 * @brief Returns the error log register stored in position defined by index
 *
 * @param index position in the array error log.
 * @return Error log
 *
 * @note index needs to be smaller than 24.
 */
uint32_t params_attach_get_log_index(uint32_t index)
{
	return param.attach_errors.err_log[index];
}

/**
 * @fn uint32_t params_attach_get_log_timestamp_index(uint32_t)
 * @brief Returns the time stamp of the error log stored in position defined by index.
 *
 * @param index position in the array error log time stamp.
 * @return
 */
uint32_t params_attach_get_log_timestamp_index(uint32_t index)
{
	return param.attach_errors.err_log_timestamp[index];
}

/**
 * @}
 */

/**
 * @fn void params_set_time_reset(time_t)
 * @brief Sets main system reset time given by _time_reset.
 *
 * @param _time_reset in epoch
 *
 * @note System Reset will be performed every day at _time_reset in epoch
 */
void params_set_time_reset(time_t _time_reset)
{
	param.reset_time = _time_reset;
}

/**
 * @fn time_t params_get_time_reset(void)
 * @brief Gets the main system reset time.
 *
 * @return system time reset in epach.
 */
time_t params_get_time_reset(void)
{
	return param.reset_time;
}

/**
 * @fn void params_set_timeout_connection(uint32_t)
 * @brief Sets the maximum timeout connection before the device finishes the communication attempts.
 *
 * @param _timeout_connection
 */
void params_set_timeout_connection(uint32_t _timeout_connection)
{
	param.timeout_connection = _timeout_connection;
}

/**
 * @fn uint32_t params_get_timeout_connection(void)
 * @brief Gets the maximum timeout connection time.
 *
 * @return Maximum connection time.
 *
 * @note
 */
uint32_t params_get_timeout_connection(void)
{
	if ( 0 == param.timeout_connection )
	{
		param.timeout_connection = 90;
	}
	return param.timeout_connection;
}

/**
 * @fn void params_set_retries_socket(uint32_t)
 * @brief Sets the maximum allowed number of attempts in case of error in opening socket.
 *
 * @param _retries_socket
 */
void params_set_retries_socket(uint32_t _retries_socket)
{
	param.retries_socket = _retries_socket;
}

/**
 * @fn uint32_t params_get_retries_socket(void)
 * @brief Gets the maximum number of attempts in case of error in opening socket.
 *
 * @return _retries_socket attempts.
 */
uint32_t params_get_retries_socket(void)
{
//	/* Number of retries cannot be 0 */
//	if ( 0 == param.retries_socket )
//	{
//		param.retries_socket = 1;
//	}
	return param.retries_socket;
}

/**
 * @fn void params_set_timeout_server(uint32_t)
 * @brief Sets the maximum timeout to connect to the remote server.
 *
 * @param _timeout_server time in seconds.
 */
void params_set_timeout_server(uint32_t _timeout_server)
{
	param.timeout_server = _timeout_server;
}

/**
 * @fn uint32_t params_get_timeout_server(void)
 * @brief Gets the maximum timeout connection with the remote server
 *
 * @return _timout_server in seconds.
 *
 * @note Minimum time cannot be 0.
 */
uint32_t params_get_timeout_server(void)
{
	if ( 0 == param.timeout_server )
	{
		param.timeout_server = 10;
	}
	return param.timeout_server;
}


/**
 * @fn void params_set_retries_server(uint32_t)
 * @brief Sets the maximum number of attempts to connect to the server.
 *
 * @param _retries_server attempts.
 */
void params_set_retries_server(uint32_t _retries_server)
{
	param.retries_server = _retries_server;
}

void params_set_pulse_acc( uint32_t _pulse_acc )
{
	param.pulse_acc = _pulse_acc;
}

uint32_t params_get_pulse_acc( void )
{
	return param.pulse_acc;
}

uint32_t params_get_pulse_acc_backup( void )
{
	return pulse_acc_backup;
}

void params_set_pulse_acc_backup( uint32_t _pulse_acc )
{
	pulse_acc_backup = _pulse_acc;
}

uint32_t params_get_pulse_write( void )
{
	return pulse_write;
}

void params_set_pulse_write( uint32_t _pulse_write )
{
	pulse_write = _pulse_write;
}

void params_set_mqtt_dc_on( uint32_t _mqtt_dc_on )
{
	param.mqtt_dc_on = _mqtt_dc_on;
}

uint32_t params_get_mqtt_dc_on( void )
{
	return param.mqtt_dc_on;
}

void params_set_modbus_type( uint32_t _modbus_type )
{
	param.generic_modbus_type = _modbus_type;
}

time_t params_get_modbus_type( void )
{
	return param.generic_modbus_type;
}

void params_set_modbus_read_time( uint32_t _modbus_read_time )
{
	param.generic_modbus_read_time = _modbus_read_time;
}

time_t params_get_modbus_read_time( void )
{
	return param.generic_modbus_read_time;
}

void params_set_modbus_send_time( uint32_t _modbus_send_time )
{
	param.generic_modbus_send_time = _modbus_send_time;
}

time_t params_get_modbus_send_time( void )
{
	return param.generic_modbus_send_time;
}

void params_set_modbus_quantity( uint32_t _quantity )
{
	param.generic_modbus_quantity = _quantity;
}

uint32_t params_get_modbus_quantity( void )
{
	return param.generic_modbus_quantity;
}

void params_set_modbus_function( uint32_t _function )
{
	param.generic_modbus_function = _function;
}

uint32_t params_get_modbus_function( void )
{
	return param.generic_modbus_function;
}

void params_set_modbus_address( uint32_t _address )
{
	param.generic_modbus_address = _address;
}

uint32_t params_get_modbus_address( void )
{
	return param.generic_modbus_address;
}

void params_set_modbus_slave_id( uint32_t _slave_id )
{
	param.generic_modbus_slave_id = _slave_id;
}

uint32_t params_get_modbus_slave_id( void )
{
	return param.generic_modbus_slave_id;
}

void params_set_modbus_slave_num( uint32_t _function )
{
	param.generic_modbus_num_slaves = _function;
}

uint32_t params_get_modbus_slave_num( void )
{
	return param.generic_modbus_num_slaves;
}

void params_set_modbus_slave_id_table( uint32_t slave, uint32_t slave_id )
{
	param.generic_modbus_slaves_id[slave] = slave_id;
}

uint32_t params_get_modbus_slave_id_table( uint32_t slave )
{
	return param.generic_modbus_slaves_id[slave];
}

void params_set_modbus_parity( uint32_t _parity )
{
	param.generic_modbus_parity = _parity;
}

uint32_t params_get_modbus_parity( void )
{
	return param.generic_modbus_parity;
}

void params_set_modbus_baudrate( uint32_t _baudrate )
{
	param.generic_modbus_baudrate = _baudrate;
}

uint32_t params_get_modbus_baudrate( void )
{
	return param.generic_modbus_baudrate;
}

void params_set_modbus_stopbits( uint32_t _stopbits )
{
	param.generic_modbus_stop_bits = _stopbits;
}

uint32_t params_get_modbus_stopbits( void )
{
	return param.generic_modbus_stop_bits;
}

void params_set_modbus_warm_time( uint32_t _warm_time )
{
	param.generic_modbus_warm_time = _warm_time;
}

uint32_t params_get_modbus_warm_time( void )
{
	return param.generic_modbus_warm_time;
}

void params_set_generic_sensor_warm_time( uint32_t _warm_time )
{
	param.generic_sensor_warm_time = _warm_time;
}

uint32_t params_get_generic_sensor_warm_time( void )
{
	return param.generic_sensor_warm_time;
}

void params_set_input_alarm_sensor( uint32_t _enable )
{
	param.input_alarm_sensor = _enable;
}

uint32_t params_get_input_alarm_sensor( void )
{
	return param.input_alarm_sensor;
}
/**
 * @fn uint32_t params_get_retries_server(void)
 * @brief Get the maximum number of attempts to connecto to the server.
 *
 * @return _retries_server attempts.
 */
uint32_t params_get_retries_server(void)
{
	/* Starts the counter from 1*/
	if ( 0 == param.retries_server )
	{
		param.retries_server = 1;
	}
	return param.retries_server;
}

/**
 * @fn void params_set_mbus_baudrate(uint32_t)
 * @brief Sets the baud rate for the MBUS communication interface.
  *
 * @param _baud_rate
 *
 * @note Allowed values are
 * 		@arg 2400bps
 * 		@arg 9600bps
 */
void params_set_mbus_baudrate(uint32_t _baud_rate)
{
	param.mbus_baudrate = _baud_rate;
}

/**
 * @fn uint32_t params_get_mbus_baudrate(void)
 * @brief Gets the baud rate for the MBUS communication interface.
 *
 * @return _baud_rate
 * 		@arg 2400bps
 * 		@arg 9600bps
 */
uint32_t params_get_mbus_baudrate(void)
{
	return param.mbus_baudrate;
}

void params_set_uart_meter_type( uint32_t _type )
{
	param.uart_meter_type = _type;
}

uint32_t params_get_uart_meter_type( void )
{
	return param.uart_meter_type;
}

/**
 * @fn void params_set_max_datalogger_size(uint32_t)
 * @brief Sets the maximum datalogger size.
 *
 * @param _max_datalogger_size
 */
void params_set_max_datalogger_size(uint32_t _max_datalogger_size)
{
	param.max_datalogger_size = _max_datalogger_size;
}

/**
 * @fn uint32_t params_get_max_datalogger_size(void)
 * @brief Gets the maximum datalogger size.
 *
 * @return _max_datalogger_size
 */
uint32_t params_get_max_datalogger_size(void)
{
#if 0
	if ( 0 == param.max_datalogger_size )
	{
		param.max_datalogger_size = 3;
	}
#endif
	return param.max_datalogger_size;
}

/**
 * @fn void params_set_max_datalogger_msgs(uint32_t)
 * @brief Sets maximum datalogger messages.
 *
 * @param _max_datalogger_msgs
 */
void params_set_max_datalogger_msgs(uint32_t max_datalogger_msgs)
{
	param.max_datalogger_msgs = max_datalogger_msgs;
}

/**
 * @fn uint32_t params_get_max_datalogger_msgs(void)
 * @brief Gets maximum datalogger messages.
 *
 * @return max_datalogger_msgs
 */
uint32_t params_get_max_datalogger_msgs(void)
{
#if 0
	if ( 0 == param.max_datalogger_msgs ) {
		param.max_datalogger_msgs = 3;
	}
#endif
	return param.max_datalogger_msgs;
}

/**
 * @fn void params_set_poweroff_time(uint32_t)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_poweroff_time(uint32_t _poweroff_time)
{
	param.poweroff_time = _poweroff_time;
}

/**
 * @fn uint32_t params_get_poweroff_time(void)
 * @brief
 *
 * @return
 */
uint32_t params_get_poweroff_time(void)
{
	return param.poweroff_time;
}

/**
 * @fn void params_set_type_comm(uint32_t)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_type_comm(uint32_t _type_comm)
{
	param.type_comm = _type_comm;
}

/**
 * @fn uint32_t params_get_type_comm(void)
 * @brief
 *
 * @return
 */
uint32_t params_get_type_comm(void)
{
	return param.type_comm;
}

/**
 * @fn void params_set_synch_meters(uint32_t _synch_meters)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_synch_meters(uint32_t _synch_meters)
{
	param.synch_meters = _synch_meters;
}

/**
 * @fn uint32_t params_get_synch_meters(void)
 * @brief
 *
 * @return
 */
uint32_t params_get_synch_meters(void)
{
	return param.synch_meters;
}

/**
 * @fn void params_set_slow_meter(uint32_t _synch_meters)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_slow_meter(uint32_t _slow_meter)
{
	param.slow_meter = _slow_meter;
}

/**
 * @fn uint32_t params_get_slow_meter(void)
 * @brief
 *
 * @return
 */
uint32_t params_get_slow_meter(void)
{
	return param.slow_meter;
}

/**
 * @fn uint32_t params_slow_meter_delay(void)
 * @brief
 *
 */
uint32_t params_slow_meter_delay(void)
{
	if ( 1 == params_get_slow_meter() )
	{
		HAL_Delay(1000);
		return 1;
	}
	return 0;
}

/**
 * @fn void params_set_mbus_frame(uint32_t _synch_meters)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_mbus_frame(uint32_t _mbus_frame)
{
	param.mbus_frame = _mbus_frame;
}

/**
 * @fn uint32_t params_get_mbus_frame(void)
 * @brief
 *
 * @return
 */
uint32_t params_get_mbus_frame(void)
{
	return param.mbus_frame;
}

/**
 * @fn void params_set_inc_pulses_totalizer( void)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_inc_pulses_totalizer(void)
{
	param.pulses_totalizer += 1;
}

/**
 * @fn void params_set_inc_pulses_totalizer( void)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_reset_pulses_totalizer(void)
{
	param.pulses_totalizer = 0;
}

/**
 * @fn uint32_t params_get_pulses_totalizer(void)
 * @brief
 *
 * @return
 */
uint64_t params_get_pulses_totalizer(void)
{
	return param.pulses_totalizer;
}

/**
 * @fn void params_manteinance_reset_statistics(void)
 * @brief Reset all statistical parameters
 *
 */
void params_manteinance_reset_statistics( void )
{
	param.manteinance.number_nok_sendings = 0;
	param.manteinance.number_ok_sendings  = 0;
	param.manteinance.number_readings     = 0;
	param.attach_errors.csq_err           = 0;
	param.attach_errors.sim_err           = 0;
	param.attach_errors.cereg_err         = 0;
	param.attach_errors.tso_err           = 0;
	param.attach_errors.tse_err           = 0;
	param.attach_errors.ip_err            = 0;
	memset(param.attach_errors.err_log, 0, sizeof(param.attach_errors.err_log));
	memset(param.attach_errors.err_log_timestamp, 0, sizeof(param.attach_errors.err_log_timestamp));
}

/**
 * @fn void params_set_middleware_id(char*)
 * @brief Set middleware ID
 *
 * @param middleware_id pointer to a string with the middleware ID finished with '\0'.
 */
void params_set_middleware_id(char *middleware_id)
{
	strncpy((char *) param.middleware_id, middleware_id, 12);
}

/**
 * @fn char params_get_middleware_id*(void)
 * @brief Gets the configured middleware ID.
 *
 * @return pointer to the configured middleware ID.
 *
 * @note if middleware is not configured then returns the IMEI.
 */
char * params_get_middleware_id(void)
{
	if (param.middleware_id[0] == '\0')
	{
		return param.imei;
	}
	else
	{
		return (char *) param.middleware_id;
	}
}

/**
 * @fn char params_get_meter_id*(void)
 * @brief Gets the meter id.
 *
 * @return pointer to a string containing the meter id finished with '\0'
 */
char * params_get_meter_id(void)
{
	if (param.meter_id[0] == '\0')
	{
		return param.imei;
	}
	else
	{
		return (char *) param.meter_id;
	}
}


/**
 * @fn char params_get_APN*(void)
 * @brief Gets the Access Point Name.
 *
 * @return pointer to a string containing the APN[0] name.
 */
char * params_get_APN(void)
{
	return param.APN[ME910_APN_get_current()].name;
}

/**
 * @fn char params_get_server*(void)
 * @brief Gets the Server Name
 *
 * @return pointer to a string containing the Server[0] name.
 */
char * params_get_server(void)
{
	return param.server[0].name;
}

uint32_t params_get_server_keepalive( uint32_t _index )
{
	return param.server[_index].keepalive;
}

uint32_t params_set_server_keepalive( uint32_t _index, uint32_t _keepalive )
{
	param.server[_index].keepalive = _keepalive;

	return _keepalive;
}

/**
 * @fn void params_set_meter_id(char*)
 * @brief Writes the meter identification to system parameters in RAM.
 *
 * @param meter_id pointer to a string containing the meter id name.
 */
void params_set_meter_id(char *meter_id)
{
	strncpy((char *) param.meter_id, meter_id, 12);
}

/**
 * @fn uint32_t params_imei_ok(void)
 * @brief Checks whether the IMEI has been written to RAM system parameters.
 *
 * @return @arg 0 - IMEI is not written to RAM memory.
 * 		   @arg 1 - IMEI is already written to RAM memory.
 */
uint32_t params_imei_ok(void)
{
	uint32_t ret = 0;
	if (param.imei[0] != '\0')
	{
		ret = 1;
	}
	return ret;
}

/**
 * @fn char params_get_imei*(void)
 * @brief Gets the system IMEI.
 *
 * @return pointer to a string containing the IMEI of the sytem.
 */
char * params_get_imei(void)
{
	return param.imei;
}

/**
 * @fn void params_config_set_overpressure_alarm(float_t)
 * @brief Sets the pressure value to configure the overpressure alarm.
 *
 * @param _overpressure pressure value.
 */
void params_config_set_overpressure_alarm(float_t _overpressure)
{
    param.config.ti3 = _overpressure;
}

/**
 * @fn float_t params_config_get_overpressure_alarm(void)
 * @brief Gets the over pressure alarm value configured.
 *
 * @return _overpressure pressure value.
 */
float_t params_config_get_overpressure_alarm(void)
{
    return param.config.ti3;
}

/**
 * @fn void params_config_set_lowpressure_alarm(float_t)
 * @brief Sets the pressure value to configure the lower pressure alarm.
 *
 * @param _lowpressure pressure value.
 */
void params_config_set_lowpressure_alarm(float_t _lowpressure)
{
    param.config.ti4 = _lowpressure;
}

/**
 * @fn float_t params_config_get_lowpressure_alarm(void)
 * @brief Gets the lower pressure alarm value configured.
 *
 * @return _lowpressure pressure value.
 */
float_t params_config_get_lowpressure_alarm(void)
{
    return param.config.ti4;
}

/**
 * @fn void params_set_i2c_sensor(uint32_t)
 * @brief
  *
 * @param
 *
 * @note
 */
void params_set_i2c_sensor(uint32_t _i2c_sensor)
{
	param.i2c_sensor = _i2c_sensor;
}

/**
 * @fn uint32_t params_get_i2c_sensor(void)
 * @brief
 *
 * @return
 */
uint32_t params_get_i2c_sensor(void)
{
	return param.i2c_sensor;
}

/**
 * @fn char params_device_get_battery_var*(void)
 * @brief Estimates the battery level based on a 87600 sending.
 *
 * @return pointer to a string containing the battery level in an array.
 */
char * params_device_get_battery_var(void)
{
	float num_sends = params_maintenance_number_ok_sendings() + params_maintenance_number_nok_sendings();
	float rem = (num_sends * 100) / 87600;
	float battery = 100 - rem;

#define DECIMAL(a)	((int) ((fabsf((a - (float) ((int) a)))) * 10))

	sprintf( device_battery, "%2d.%d", (int) battery, DECIMAL(battery));

	return device_battery;
}

/**
  * @brief  System parameters initialization.
  * @retval None
  */
void params_init()
{
	/** There is 8K in internal FLASH reserved for parameters. The parameters size is 2k
	 * therefore there are 4 different pages page_i to store new parameters -> pages_n = 4 for 2048 bytes
	 * params_size = 8K and FLASH_PAGE_SIZE = 2K*/
	pages_n = (uint32_t) _params_size / FLASH_PAGE_SIZE;
	page_i  = params_get_values();
}

/**
  * @brief  Loads to structure param in RAM the parameters stored in internal UC FLASH memory.
  * @retval Returns the number of the page inside the reserved 8k memory in which parameters are stored.
  *
  * @note The selected page is changed each time parameters are written to avoid to use allways the same
  * section of FLASH memory.
  *
  @dot
  digraph G {
  rankdir = LR;
  size = "14";
  page_0 [shape=box];
  page_1 [shape=box];
  page_2 [shape=box];
  page_3 [shape=box];

  page_0 -> page_1 -> page_2 -> page_3 -> page_0;
  }
  @enddot
  */
static unsigned char params_get_values()
{
	crc32_init_table();

    /** Copies the 2K FLASH parameters to param structure and compares CRC, the returns the number of
     * the page_i in params FLASH space memory.
     * */

	for (unsigned char i = 0; i < pages_n; i++)
    {
    	memcpy((void *) &param, (uint32_t *) ((uint32_t) _params_start_address + FLASH_PAGE_SIZE * i), sizeof(param));

    	if(param.crc == crc32((char *) &param, sizeof(param) - sizeof(param.crc) - sizeof(param.align_crc), 0xFFFFFFFF))
        {
            return i;
        }
    }

	/** If no page matches with the CRC then copy the default parameters to page_i = 0 and selects it.
	 * */
	memset((void *) &param, '\0', sizeof(param));
#ifdef ZONOS
#	if 0
    strcpy(param.server[0].name, "10.241.12.34");					//field test dewa zonos server ip
    strcpy(param.server[1].name, "10.241.12.34");
#	endif
    strcpy(param.server[0].name, "10.241.111.42");				//prod MORO server ip
    strcpy(param.server[1].name, "10.241.111.42");
#	if 0
    strcpy(param.server[0].name, "qa-watr.smartgridfan.local");	//dewa zonos server fqdn
    strcpy(param.server[1].name, "qa-watr.smartgridfan.local");
#	endif
    strcpy(param.APN[0].name, "dewa-sg");							//dewa etisalat sim cards apn.
    strcpy(param.APN[1].name, "dewa-sg");
#else
#if defined(PRIVATE_SERVER)
    strcpy(param.server[0].name, "172.31.27.44");			   //private ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "172.31.27.44");
#elif defined(PUBLIC_SERVER)
    strcpy(param.server[0].name, "34.248.252.126");		   //public ip gateways.trango.app
    strcpy(param.server[1].name, "34.248.252.126");
//    strcpy(param.server[0].name, "gateways.trango.app");		   //public ip gateways.trango.app
//    strcpy(param.server[1].name, "gateways.trango.app");
#	if 0
    strcpy(param.server[0].name, "217.165.206.110");	   //public ip thinworx
    strcpy(param.server[1].name, "217.165.206.110");
#	endif
#elif defined(PUBLIC_SERVER_QA_TRANGO)
    strcpy(param.server[0].name, "46.51.135.242");		   //public ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "46.51.135.242");
#elif defined(PUBLIC_SERVER_ADDC)
//    strcpy(param.server[0].name, "46.51.135.242");		   //public ip dewa.e3tmqtt.com
//    strcpy(param.server[1].name, "46.51.135.242");
    strcpy(param.server[0].name, "10.170.4.13");		   //public ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "10.170.4.13");
//    strcpy(param.server[0].name, "10.170.3.4");		   //public ip dewa.e3tmqtt.com
//    strcpy(param.server[1].name, "10.170.3.4");
#elif defined(PUBLIC_SERVER_ADDC_QA)
    strcpy(param.server[0].name, "10.170.4.13");		   //public ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "10.170.4.13");
#elif defined(PUBLIC_SERVER_ADDC_PROD)
    strcpy(param.server[0].name, "10.170.3.4");		   //public ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "10.170.3.4");
#elif defined(PUBLIC_DMCC_SERVER)
    strcpy(param.server[0].name, "192.168.130.146");		   //public ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "192.168.130.146");
#elif defined(PUBLIC_SERVER_SEWA_PROD)
    strcpy(param.server[0].name, "192.166.100.20");		   //public ip dewa.e3tmqtt.com
    strcpy(param.server[1].name, "192.166.100.20");
#elif defined(PUBLIC_SERVER_DEWA_PROD)
    strcpy( param.server[0].name, "10.241.111.42" );				//prod MORO server ip
    strcpy( param.server[1].name, "10.241.111.42" );
#elif defined(BOMBAI_SERVER)
    strcpy(param.server[0].name, "13.233.107.37");		   //etisalat.e3tmqtt.com
    strcpy(param.server[1].name, "13.233.107.37");
#elif defined(ETISALAT_LOCAL_SERVER)
    strcpy(param.server[0].name, "151.253.222.142");		   //etisalat.datakorum.com
    strcpy(param.server[1].name, "151.253.222.142");
#elif defined(PUBLIC_RECKENBERG_SERVER)
    strcpy(param.server[0].name, "87.138.70.142");		   //etisalat.datakorum.com
    strcpy(param.server[1].name, "87.138.70.142");
#endif
#if defined(MOVISTAR_SIM)
    strcpy(param.APN[0].name, "e3tcity.movistar.es");		   //datakorum apn
    strcpy(param.APN[1].name, "e3tcity.movistar.es");
#elif defined(ETISALAT_SIM)
    strcpy(param.APN[0].name, "nbiot");					   //etisalat apn
    strcpy(param.APN[1].name, "nbiot");
#	if 0
    strcpy(param.APN[0].name, "test");					   //etisalat rhode&schwarz apn
    strcpy(param.APN[1].name, "test");
#	endif
#elif defined(ETISALAT_AADC_SIM)
    strcpy(param.APN[0].name, "addc-nbiot");					   //etisalat apn
    strcpy(param.APN[1].name, "aadc-nbiot");
#elif defined(ETISALAT_AADC_SIM_CATM1)
    strcpy(param.APN[0].name, "addc-p2p");					   //etisalat apn
    strcpy(param.APN[1].name, "aadc-p2p");
#	if 0
    strcpy(param.APN[0].name, "test");					   //etisalat rhode&schwarz apn
    strcpy(param.APN[1].name, "test");
#	endif
#elif defined(ETISALAT_SEWA_SIM_CATM1)
    strcpy(param.APN[0].name, "sewa.gov.ae");					   //sewa apn
    strcpy(param.APN[1].name, "sewa.gov.ae");
#	if 0
    strcpy(param.APN[0].name, "test");					   //etisalat rhode&schwarz apn
    strcpy(param.APN[1].name, "test");
#	endif
#elif defined(VODAFONE_SIM)
    strcpy(param.APN[0].name, "Vodafone");
    strcpy(param.APN[1].name, "Vodafone");
#elif defined(BOMBAI_SIM)
    strcpy(param.APN[0].name, "m2m.movistar.es");
    strcpy(param.APN[1].name, "m2m.movistar.es");
#elif defined(THETHINX_SIM)
    strcpy(param.APN[0].name, "sm2m-apple.movistar.es");	  //THETHINX APN
    strcpy(param.APN[1].name, "sm2m-apple.movistar.es");
#elif defined(ONCE_SIM)
    strcpy(param.APN[0].name, "iot.1nce.net");			  //1NCE APN
    strcpy(param.APN[1].name, "iot.1nce.net");
#elif defined(ONCE_GLOBAL_SIM)
    strcpy(param.APN[0].name, "iot.1nce.net");			  //1NCE APN
    strcpy(param.APN[1].name, "iot.1nce.net");
#elif defined(ALLIOT_SIM)
    strcpy(param.APN[0].name, "m2m.nbiot.t-mobile.at");	  //ALLIOT APN //alliot.nbiot.at
    strcpy(param.APN[1].name, "m2m.nbiot.t-mobile.at");
#elif defined(CHUNGHWA)
    strcpy(param.APN[0].name, "internet.iot");			  //CHUNGHWA
    strcpy(param.APN[1].name, "internet.iot");
#elif defined(CMNBIOT)
    strcpy( param.APN[0].name, "cmnbiot");			  	  //CMNBIOT
    strcpy( param.APN[1].name, "cmnbiot" );
#elif defined(VODAFONE_GERMANY_SIM)
    strcpy(param.APN[0].name, "lpwa.vodafone.com");		 //VODAFONE GERMANY
    strcpy(param.APN[1].name, "lpwa.vodafone.com");
#elif defined(VODAFONE_NETHERLANDS_SIM)
    strcpy(param.APN[0].name, "lpwa.vodafone.iot");		 //VODAFONE GERMANY
    strcpy(param.APN[1].name, "lpwa.vodafone.iot");
#elif defined(VODAFONE_LOGIC_SIM)
    strcpy(param.APN[0].name, "nb.wlapn.com");				 //VODAFONE LOGIC
    strcpy(param.APN[1].name, "nb.wlapn.com");
#elif defined(VODAFONE_LOGIC_NETHERLANDS_SIM)
    strcpy(param.APN[0].name, "nb.wlapn.com");				//VODAFONE LOGIC
    strcpy(param.APN[1].name, "nb.wlapn.com");
#elif defined(ORANGE_BELGIUM_SIM)
    strcpy(param.APN[0].name, "standard.m2mmobi.be");		 //ORANGE BELGIUM
    strcpy(param.APN[1].name, "standard.m2mmobi.be");
#elif defined(ORANGE_GLOBAL_SIM)
    strcpy(param.APN[0].name, "internet");		 //ORANGE GLOBAL
    strcpy(param.APN[1].name, "internet");
#elif defined(ORANGE_SIM)
    strcpy(param.APN[0].name, "iot.nat.es");		 //ORANGE
    strcpy(param.APN[1].name, "iot.nat.es");
#elif defined(DMCC_SIM)
    strcpy(param.APN[0].name, "DMM2M.GOV.AE");
    strcpy(param.APN[1].name, "DMM2M.GOV.AE");
#elif defined(ETISALAT_DEWA_SIM_CATM1)
    strcpy( param.APN[0].name, "dewa-sg" );							//dewa etisalat sim cards apn.
    strcpy( param.APN[1].name, "dewa-sg" );
#elif defined(CYTA_SIM)
    strcpy( param.APN[0].name, "iot" );							//dewa etisalat sim cards apn.
    strcpy( param.APN[1].name, "iot" );
#endif
#endif

    strcpy(param.APN[0].user, "\0");
    strcpy(param.APN[1].user, "\0");
    strcpy(param.APN[0].password, "\0");
    strcpy(param.APN[1].password, "\0");

#if 0
    strcpy(param.APN[0].title, "ETISALAT");
    strcpy(param.APN[1].title, "ETISALAT");
#endif
#if 0
    strcpy(param.APN[0].title, "Vodafone");
    strcpy(param.APN[1].title, "Vodafone");
#endif
#if 0
    strcpy(param.APN[0].title, "MOVISTAR");
    strcpy(param.APN[1].title, "MOVISTAR");
#endif
    uint8_t i;
    for (i = 0; i < 24; i++)
    {
    	param.config.sh[i] = i;
    }

    for (i = 0; i < 8; i++) {
    	param.read_time[i].init_time = -1;
    	param.read_time[i].end_time  = -1;
    	param.read_time[i].cycle     = -1;
    }
    param.sensor_read_time[0].cycle  = 30;

    param.manteinance.delete_memory      = 0;
    param.manteinance.reset_meters       = 0;
    param.manteinance.release_assistance = 0;
    param.manteinance.reed_activation    = 1;

    param.pulses.on                      = 0;
    param.pulses.pulse_factor            = 1;
    param.pulses.unit_k_factor_out_1     = 1;
    param.pulses.unit_k_factor_out_2     = 10;
    param.pulse_acc                      = 0;

    param.wq                             = 0;

    param.config.rt                      = 900;
    param.config.st                      = 900;
    param.config.ti2                     = 1;
    param.telegram_mode                  = TELEGRAM_MODE_RAW;
    param.protocol_mode                  = PROTOCOL_TCP;
    param.nbiotpwr_mode                  = NBIOT_PWR_ONOFF;
    param.config.ti1                     = 1;

#ifdef MODBUS
    param.modbus_log.enabled             = 1;
#endif
#ifdef ZONOS
	param.server[0].port                 = param.server[1].port      = 443;
    param.server[0].keepalive            = param.server[1].keepalive = 60;
#else
	param.server[0].port                 = param.server[1].port      = 80;
    param.server[0].keepalive            = param.server[1].keepalive = 60;
#endif

    param.timeout_connection             = 220;//90;
    param.timeout_server                 = 20;//10;
    param.retries_socket                 = 3;
    param.retries_server                 = 3;//1;
    param.max_datalogger_size            = 96;
    param.max_datalogger_msgs            = 4;

    param.version.major                  = VERSION_MAJOR;
    param.version.minor                  = VERSION_MINOR;

    param.mem_cycle                      = 0;

    param.reset_time                     = 1601856335;

    param.generic_modbus_send_time       = 0;
    param.generic_modbus_read_time       = 0;
    param.generic_modbus_type            = LAST_GENERIC_485_TYPE;
    param.generic_modbus_warm_time       = 0;
    param.generic_sensor_warm_time       = 500;

    param.mqtt_dc_on                     = 0;
    
    param.uart_meter_type                = MBUS_METER_TYPE;
#if defined(ETISALAT_AADC_SIM_CATM1)
    param.type_comm                      = TYPE_COMM_CATM1;
#elif defined(ETISALAT_SEWA_SIM_CATM1)
    param.type_comm                      = TYPE_COMM_CATM1;
#elif defined(ETISALAT_DEWA_SIM_CATM1)
    param.type_comm                      = TYPE_COMM_CATM1;
#else
    param.type_comm                      = TYPE_COMM_NBIOT;
#endif
    param.synch_meters                   = METER_NOT_SYNCH;
    param.slow_meter                     = 0;
    param.mbus_frame                     = 0xF0;

    param.psm_tt3324                     = 200;
    param.psm_tt3412                     = 25;

    param.i2c_sensor                     = 0;

//    param.id.device                      = 109113087;//109113438;//109113594;//IMEI Hardcoded
//    param.id.group                       = 1;//IMEI Hardcoded
//    param.id.platform                    = 7;//IMEI Hardcoded
    param.registered                     = 0;//1;//IMEI Hardcoded


    return 0;
}

/**
 * @fn uint32_t params_get_mem_cycle(void)
 * @brief Returns whether the dedicated FLASH memory reserved space has reached
 * the end.
 *
 * @note when any of the pointers reaches the end it starts from the beginning again
 * setting this flag to 1.
 *
 * @return @arg 0 - Memory has not reached the end.
 * 		   @arg 1 - Memory has reached the end.
 */
uint32_t params_get_mem_cycle( void )
{
    return param.mem_cycle;
}

/**
 * @fn void params_set_mem_cycle(uint32_t)
 * @brief Sets the memory overflow state of a dedicated FLASH memory reserved space.
 *
 * @param mem_cycle @arg 0 - Memory has not reached the end.
 * 				    @arg 1 - Memory has reached the end.
 *
 * @note when any of the pointers reaches the end it starts from the beginning again
 * setting this flag to 1.
 */
void params_set_mem_cycle( uint32_t mem_cycle )
{
    param.mem_cycle = mem_cycle;
}

/**
  * @brief  Gets the corresponding internal FLASH page given an address.
  * @param  Addr Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the internal FLASH memory bank of a given address.
  * @param  Addr Address of the FLASH Memory.
  * @retval @ref FLASH_BANK_1.
  */
static uint32_t GetBank(uint32_t Addr)
{
	uint32_t bank = 0;
	if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
	{
		/* Bank 1 */
		bank = FLASH_BANK_1;
	}
	else
	{
		/* Bank 2 */
		bank = FLASH_BANK_2;
	}
	return bank;
}

/**
 * @fn void params_check_for_changes()
 * @brief Checks whether the parameters stored in RAM are the same as the stored in internal FLASH.
 * If they are different copies from RAM to internal FLASH.
 *
 */
void params_check_for_changes()
{
	uint32_t flash_error, flash_page, flash_bank;

	/** Reset the system if overflow in parameter page
	 * */
	if (page_i > 2) {
		asm("nop");
		HAL_NVIC_SystemReset();
	}

	/** Compares param structure in RAM with FLASH, if equals exits.
	 *  If not equal then...
	 *  */
	if(memcmp((void *) &param,
			   (uint32_t *) ((uint32_t) _params_start_address + FLASH_PAGE_SIZE * page_i),
			   sizeof(param)))
	{
        unsigned char i, s;
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t PAGEError = 0;

        /* Unlock the Flash to enable the flash control register access *************/
		HAL_FLASH_Unlock();
		/* Clear OPTVERR bit set on virgin samples */
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);

//        /* Get the 1st page to erase */
//        flash_page = GetPage((uint32_t) _params_start_address + FLASH_PAGE_SIZE);
//        /* Fill EraseInit structure*/
//        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//        EraseInitStruct.Banks     = GetBank(flash_page);
//        EraseInitStruct.Page      = flash_page;
//        EraseInitStruct.NbPages   = 1;
//        HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
        /** Get the 1st page to erase
         * */
        flash_page = GetPage((uint32_t) _params_start_address + FLASH_PAGE_SIZE * 0);
        flash_bank = GetBank((uint32_t) _params_start_address + FLASH_PAGE_SIZE * 0);
        /* Fill EraseInit structure*/
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.Banks     = flash_bank;
        EraseInitStruct.Page      = flash_page;
        EraseInitStruct.NbPages   = 1;
        HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

		param.counter++;
		param.crc = crc32( (char *) &param, sizeof(param) - sizeof(param.crc) - sizeof(param.align_crc), 0xFFFFFFFF );

        for(i = 0, s = page_i; i < pages_n; i++)
        {
            if(s < pages_n - 1)
            {
            	s++;
            }
            else
            {
            	s = 0;
            }

            /** Get the 1st page to erase
             * */
            flash_page = GetPage((uint32_t) _params_start_address + FLASH_PAGE_SIZE * s);
            flash_bank = GetBank((uint32_t) _params_start_address + FLASH_PAGE_SIZE * s);
            /* Fill EraseInit structure*/
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.Banks     = flash_bank;
            EraseInitStruct.Page      = flash_page;
            EraseInitStruct.NbPages   = 1;

            /** Erase the FLASH page
             * */
            if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) == HAL_OK)
            {
            	uint8_t *data = (uint8_t *) &param;
            	uint32_t add;
            	HAL_StatusTypeDef r;
            	/** Program the entire FLASH page in double word mode
            	 * */
            	for (uint32_t j = 0; j < FLASH_PAGE_SIZE; j+=16)
            	{
            		add = (uint32_t) _params_start_address + FLASH_PAGE_SIZE * s + j;
            		r = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, add, (uint32_t)((data + j)));
            		if (r != HAL_OK)
            		{
            			flash_error = HAL_FLASH_GetError();
            			if (HAL_FLASH_ERROR_WRP == flash_error)
            			{
            				UNUSED(flash_error);
            			}
            			break;
            		}
            	}

            	if (r != HAL_OK)
            	{
            		continue;
            	}

            	/** Checks FLASH data write integrity comparing CRC and writes 0x00000000 on the CRC field of param
            	 *  structure on the current FLASH PAGE page_i so next time the FLASH is read this page will not be
            	 *  selected to avoid to use the same portion of internal FLASH all the time.
            	 *
            	 * */
			    struct params *paramF;
				paramF = (struct params *) ((uint32_t) _params_start_address + FLASH_PAGE_SIZE * s );
				if(paramF->crc == crc32((char *) paramF, sizeof(param) - sizeof(param.crc) - sizeof(param.align_crc), 0xFFFFFFFF))
				{
					uint8_t crc[16]= {0x0};
					paramF = (struct params *) ((uint32_t) _params_start_address + FLASH_PAGE_SIZE * page_i);
					r = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,	(uint32_t) &(paramF->crc), (uint32_t )(crc));

					if (r == HAL_OK)
					{
						page_i = s;
						break;
					}
				}
			}
            else
            {
				flash_error = HAL_FLASH_GetError();
    			if (HAL_FLASH_ERROR_WRP == flash_error)
    			{
    				UNUSED(flash_error);
    			}
			}
        }

        HAL_FLASH_Lock();
    }
}

uint8_t loglive_level = 1;
uint8_t params_get_loglive_level( void )
{
	return loglive_level;
}

void params_set_loglive_level( uint8_t _level )
{
	loglive_level = _level;
}

#if defined(LOGLIVE_MODE)
void params_loglive(uint32_t level, const char *f,  ...)
{
	va_list ap;
	va_start(ap, f);
	if ((level >= LOGLIVE_LEVEL) || ( 99 == LOGLIVE_LEVEL ))
	{
		if ( /*( modbus_sensors_get_comm_state() != MODBUS_COMM_STARTED )*/( 0 == modbus_sensors_get_get_data() ) && (( CHECK_TAMPER_PIN == 0 ) || ( 99 == LOGLIVE_LEVEL )) && ( con_dlms_in_process() == 0 ) && ( ( modbus_get_end_session() == MODBUS_SESSION_END ) || ( 0 == modbus_get_local_tool_started() ) ) ) {
			MX_USART2_UART_Init();
			ENABLE_RS485();
			HAL_Delay(1);
			DE_TX_RS485();
			vsnprintf(loglive_str, sizeof(loglive_str),f,ap);
			serial_write_loglive(loglive_str, strlen(loglive_str));
			DISABLE_RS485();
			if ( modbus_get_end_session() == MODBUS_SESSION_INIT ) {MX_USART2_USART_DeInit();modbus_init();}
		}
	}
	va_end(ap);
}

void params_loglive_tamp_en(uint32_t level, const char *f,  ...)
{
	va_list ap;
	va_start(ap, f);
	if (level >= LOGLIVE_LEVEL)
	{
		if ( ( modbus_sensors_get_comm_state() != MODBUS_COMM_STARTED ) && ( con_dlms_in_process() == 0 ) && ( ( modbus_get_end_session() == MODBUS_SESSION_END ) || ( 0 == modbus_get_local_tool_started() ) ) ) {
			MX_USART2_UART_Init();
			ENABLE_RS485();
			HAL_Delay(1);
			DE_TX_RS485();
			vsnprintf(loglive_str, sizeof(loglive_str),f,ap);
			serial_write_loglive(loglive_str, strlen(loglive_str));
			DISABLE_RS485();
			if ( modbus_get_end_session() == MODBUS_SESSION_INIT ) {MX_USART2_USART_DeInit();modbus_init();}
		}
	}
	va_end(ap);
}

void params_loglive_serial(uint32_t level, const char *f,  ...)
{
	va_list ap;
	va_start(ap, f);
	if (level >= LOGLIVE_LEVEL)
	{
		if ( ( modbus_sensors_get_comm_state() != MODBUS_COMM_STARTED ) /*&& ( CHECK_TAMPER_PIN == 0 )*/ && ( con_dlms_in_process() == 0 ) && ( ( modbus_get_end_session() == MODBUS_SESSION_END ) || ( 0 == modbus_get_local_tool_started() ) ) ) {
			DISABLE_RS485();
			MX_USART2_UART_Init();
//			ENABLE_RS485();
//			HAL_Delay(1);
//			DE_TX_RS485();
			vsnprintf(loglive_str, sizeof(loglive_str),f,ap);
			serial_write_loglive(loglive_str, strlen(loglive_str));
//			DISABLE_RS485();
			if ( modbus_get_end_session() == MODBUS_SESSION_INIT ) {MX_USART2_USART_DeInit();modbus_init();}
		}
	}
	va_end(ap);
}
#endif

/**
  * @}
  */
