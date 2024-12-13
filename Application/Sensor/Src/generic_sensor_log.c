/*
 * generic_sensor_log.c
 *
 *  Created on: 26 oct. 2023
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "common_lib.h"

#include "generic_sensor_log.h"
#include "generic_sensor.h"
#include "ventosa.h"
#include "spi_flash.h"
#include "tick.h"
#include "leds.h"
#include "spi_flash.h"

#define NUM_INPUTS            (5U)
#define SENSOR_LOG_BLOCK_SIZE (0x70000)//(0x80000)

static char message_value_sensor[NUM_INPUTS][7];

extern char _pressure_start_address[], _pressure_meas_size[];

uint32_t generic_sensor_telegram_values_address           = (uint32_t) _pressure_start_address + 0x1000;

uint32_t generic_sensor_one_telegram_values_address       = (uint32_t) _pressure_start_address + 0x1000;
uint32_t generic_sensor_one_telegram_values_max_address   = (uint32_t) _pressure_start_address + SENSOR_LOG_BLOCK_SIZE - 1;

uint32_t generic_sensor_two_telegram_values_address       = (uint32_t) _pressure_start_address + 0x1000 + SENSOR_LOG_BLOCK_SIZE;
uint32_t generic_sensor_two_telegram_values_max_address   = (uint32_t) _pressure_start_address + 2*SENSOR_LOG_BLOCK_SIZE - 1;

uint32_t generic_sensor_three_telegram_values_address     = (uint32_t) _pressure_start_address + 0x1000 + 2*SENSOR_LOG_BLOCK_SIZE;
uint32_t generic_sensor_three_telegram_values_max_address = (uint32_t) _pressure_start_address + 3*SENSOR_LOG_BLOCK_SIZE - 1;

uint32_t generic_sensor_four_telegram_values_address      = (uint32_t) _pressure_start_address + 0x1000 + 3*SENSOR_LOG_BLOCK_SIZE;
uint32_t generic_sensor_four_telegram_values_max_address  = (uint32_t) _pressure_start_address + 4*SENSOR_LOG_BLOCK_SIZE;

uint32_t generic_sensor_five_telegram_values_address      = (uint32_t) _pressure_start_address + 0x1000 + 4*SENSOR_LOG_BLOCK_SIZE;
uint32_t generic_sensor_five_telegram_values_max_address  = (uint32_t) _pressure_start_address + 5*SENSOR_LOG_BLOCK_SIZE;

uint32_t generic_sensor_telegram_values_max_address       = (uint32_t) _pressure_start_address + NUM_INPUTS*SENSOR_LOG_BLOCK_SIZE;

uint32_t generic_sensor_log_addresses[NUM_INPUTS] = {(uint32_t) _pressure_start_address,
										    (uint32_t) _pressure_start_address + 1*SENSOR_LOG_BLOCK_SIZE,
										    (uint32_t) _pressure_start_address + 2*SENSOR_LOG_BLOCK_SIZE,
										    (uint32_t) _pressure_start_address + 3*SENSOR_LOG_BLOCK_SIZE,
											(uint32_t) _pressure_start_address + 4*SENSOR_LOG_BLOCK_SIZE};

uint32_t generic_sensor_telegram_values_addresses[NUM_INPUTS] = {(uint32_t) _pressure_start_address + 0x1000,
											            (uint32_t) _pressure_start_address + 0x1000 + 1*SENSOR_LOG_BLOCK_SIZE,
											            (uint32_t) _pressure_start_address + 0x1000 + 2*SENSOR_LOG_BLOCK_SIZE,
											            (uint32_t) _pressure_start_address + 0x1000 + 3*SENSOR_LOG_BLOCK_SIZE,
														(uint32_t) _pressure_start_address + 0x1000 + 4*SENSOR_LOG_BLOCK_SIZE};

uint32_t generic_sensor_telegram_values_max_addresses[NUM_INPUTS] = {(uint32_t) _pressure_start_address + 1*SENSOR_LOG_BLOCK_SIZE - 1,
											                (uint32_t) _pressure_start_address + 2*SENSOR_LOG_BLOCK_SIZE - 1,
											                (uint32_t) _pressure_start_address + 3*SENSOR_LOG_BLOCK_SIZE - 1,
											                (uint32_t) _pressure_start_address + 4*SENSOR_LOG_BLOCK_SIZE - 1,
															(uint32_t) _pressure_start_address + 5*SENSOR_LOG_BLOCK_SIZE - 1};

static uint32_t  sensor_telegram_log_size = 0;

#define DECIMAL_DIGITS  (2)
#define MAX_DIGITS      (8)//(9)
/** @brief Structure that holds generic sensor values to save the geneirc sensor data in flash */
typedef struct {
	char    generic_sensor_val[MAX_DIGITS];		/*!< Instantaneous value in string */
	char    generic_sensor_avg[MAX_DIGITS];		/*!< Average value in string */
	char    generic_sensor_max[MAX_DIGITS];		/*!< Maximum value in string */
	char    generic_sensor_min[MAX_DIGITS];		/*!< Minimum value string */
	float_t generic_sensor_val_dec;		/*!< Current value in float */
	float_t generic_sensor_acc_dec;		/*!< Accumulated value in float */
	float_t generic_sensor_avg_dec;		/*!< Average value in float */
	float_t generic_sensor_max_dec;		/*!< Maximum in float */
	float_t generic_sensor_min_dec;		/*!< Minimum pressure value in float */
} generic_sensor_values_t;

/** @brief Structure to save the data, pointers and counters in flash
 * this is saved to _pressure_start_address = 0x700000
 * */
typedef struct {
	generic_sensor_values_t generic_sensor_val;					/*!< Generic sensor data */
	uint32_t       			samples;					/*!< Number of pressure data samples */
	uint32_t        		init;						/*!< Token to check that structure is initialized */
	uint32_t        		msg_num;					/*!< Number of telegrams sent */
	uint32_t        		sensor_id;				    /*!<  Sensor identifier */
	uint32_t        		msg_num_backup;				/*!<  */
	uint32_t        		msg_wr_address;				/*!<  */
	uint32_t        		msg_rd_address;				/*!<  */
	uint32_t        		msg_rd_address_backup;		/*!<  */
    uint32_t        		msg_rd_ini_addr;			/*!<  */
    uint32_t 				msg_rd_end_addr;			/*!<  */
    uint32_t 				msg_rd_end_addr_backup;		/*!<  */
} generic_sensor_log_t;

generic_sensor_values_t sensor_values[NUM_INPUTS];
generic_sensor_log_t    sensor_log[NUM_INPUTS];

/** @brief Structure to save the pressure data telegram in flash */
typedef struct {
	char     sensor_val[MAX_DIGITS];				/*!<  Instantaneous pressure value in string */
	char     sensor_avg[MAX_DIGITS];				/*!<  Average pressure value in string */
	char     sensor_max[MAX_DIGITS];				/*!<  Maximum pressure value in string */
	char     sensor_min[MAX_DIGITS];				/*!<  Minimum pressure value in string */
	char     created_sensor_value[7];		/*!<  Pressure value time stamp */
	char     created_sensor_value_date[9];	/*!<  Pressure value date */
	uint32_t sensor_id;
	uint32_t msg_num;						/*!<  Message counter */
	uint32_t wr_addr;						/*!<  Pointer to address to write to memory */
	uint32_t rd_addr;						/*!<  Pointer to address to read from memory*/
} sensor_telegram_values_t;//#define NITEMS(x) (sizeof(x)/sizeof(x[0]))

/** Structure pressure telegram initialization */
static sensor_telegram_values_t sensor_telegram_value[NUM_INPUTS] = {
		{
		.sensor_val = {0},
		.sensor_avg = {0},
		.sensor_max = {0},
		.sensor_min = {0},
		.msg_num      = 0,
		.wr_addr      = 0,
		.rd_addr      = 0
		},
		{
		.sensor_val = {0},
		.sensor_avg = {0},
		.sensor_max = {0},
		.sensor_min = {0},
		.msg_num      = 0,
		.wr_addr      = 0,
		.rd_addr      = 0
		},
		{
		.sensor_val = {0},
		.sensor_avg = {0},
		.sensor_max = {0},
		.sensor_min = {0},
		.msg_num      = 0,
		.wr_addr      = 0,
		.rd_addr      = 0
		},
		{
		.sensor_val = {0},
		.sensor_avg = {0},
		.sensor_max = {0},
		.sensor_min = {0},
		.msg_num      = 0,
		.wr_addr      = 0,
		.rd_addr      = 0
		}
};

char sensor_telegram_string[NUM_INPUTS][MAX_GENERIC_SENSOR_TELEGRAM_VALUES];

/**  Structure for sensor alarms */
static struct {
	uint32_t oversensor_alarm;	/*!<  */
	uint32_t lowsensor_alarm;		/*!<  */
	uint32_t operatingsensor_alarm;		/*!<  */
} generic_sensor_alarms[NUM_INPUTS];

uint32_t generic_sensor_log_save_programword = 0;
static uint8_t __readGenericSensorLog(uint32_t num);

void generic_sensor_log_set_save_programword( uint32_t __save )
{
	generic_sensor_log_save_programword = __save;
}

uint32_t generic_sensor_log_check_set_programword( uint32_t _diff_time_in_secs )
{
	if ( 1 == rtc_system_time_to_reset(_diff_time_in_secs) )
	{
		generic_sensor_log_save_programword = 1;
	}
	else
	{
		generic_sensor_log_save_programword = 0;
	}
	return 0;
}

/**
  * @brief Writes structure pressure_log in sector of address _pressure_start_address (0x700000)
  * @retval 0
  */
static uint8_t __programWord(uint32_t num)
{
	uint32_t  size, ret = 0, check_write = 0;
	uint8_t  *ptr;
	generic_sensor_log_t sensor_check;

	if (( 1 == generic_sensor_log_save_programword )||( 2 == generic_sensor_log_save_programword ))
	{
		size = sizeof(sensor_log[num]);
		ptr = (uint8_t *) &sensor_log[num];
		generic_sensor_log_save_programword = 2;
		LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> EXECUTE __programWord()  \r\n", (int)Tick_Get( SECONDS ));
		if (sensor_log[num].init == 0)
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> WARNING!! init:%d \r\n", (int)Tick_Get( SECONDS ), (int)sensor_log[num].init);
		}
		uint32_t Flash_ID = sFLASH_ReadID();
		sFLASH_WaitBusy();

		if (0==Flash_ID)
		{
			//		sFLASH_Init();
			//		sFLASH_initSST26V();
			sFLASH_Init_Test();
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> Sensor Log Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)generic_sensor_log_addresses[num]);
		sFLASH_EraseSector((uint32_t) generic_sensor_log_addresses[num]);
		HAL_Delay(10);
		sFLASH_WriteBuffer(ptr, (uint32_t) generic_sensor_log_addresses[num], size);
		HAL_Delay(10);
		sFLASH_ReadBuffer( (uint8_t *)&sensor_check, (uint32_t) generic_sensor_log_addresses[num], size );

		if ( 0 == memcmp(ptr, (uint8_t *)&sensor_check, size)) {
			__NOP();
			check_write = 0;
		} else {
			__NOP();
			check_write = 1;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> WARNING!! CHECK WRITE in __programWord init:%d init_check:%d\r\n", (int)Tick_Get( SECONDS ), (int)sensor_log[num].init, (int)sensor_check.init);
		}

		/**WARNING: In order to write to FLASH the sector needs to be empty first */
		uint32_t times = 0;
		if ( 1 == check_write ) {
			do {
				HAL_Delay(100);
				sFLASH_WriteBuffer(ptr,(uint32_t) generic_sensor_log_addresses[num], size);
				HAL_Delay(100);
				sFLASH_ReadBuffer( (uint8_t *)&sensor_check, (uint32_t)generic_sensor_log_addresses[num], size );
				times++;
				if ( 0 == memcmp(ptr, (uint8_t *)&sensor_check, size)) {
					__NOP();
					check_write = 0;
					LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> RECOVERED!! CHECK WRITE in __programWord Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)_pressure_start_address);
				} else {
					__NOP();
					check_write = 1;
				}
			} while ( ( 1 == check_write ) && ( times < 5 ) );
		}
	}
    return ret;
}

void generic_sensor_log_init(void)
{
	uint32_t i;

	HAL_Delay(10);
	generic_sensor_log_set_save_programword(2);
	for (i=0; i < NUM_INPUTS; i++ )
	{
		__readGenericSensorLog(i);
		if (sensor_log[i].init != 0xAAAA)
		{
			sensor_log[i].init               = 0xAAAA;
			sensor_log[i].msg_num            = 0;
			sensor_log[i].sensor_id          = i;
			sensor_log[i].msg_wr_address     = generic_sensor_telegram_values_addresses[i];
			sensor_log[i].msg_rd_address     = generic_sensor_telegram_values_addresses[i];
			sensor_log[i].msg_rd_ini_addr    = generic_sensor_telegram_values_addresses[i];
			sensor_log[i].msg_rd_end_addr    = generic_sensor_telegram_values_addresses[i];
			sensor_telegram_value[i].wr_addr = generic_sensor_telegram_values_addresses[i];
			sensor_telegram_value[i].rd_addr = generic_sensor_telegram_values_addresses[i];
			sensor_log[i].samples            = 1;
			sensor_log[i].generic_sensor_val.generic_sensor_val_dec = 0;
			sensor_log[i].generic_sensor_val.generic_sensor_min_dec = 0;
			sensor_log[i].generic_sensor_val.generic_sensor_max_dec = 0;
			sensor_log[i].generic_sensor_val.generic_sensor_acc_dec = 0;
			sensor_log[i].generic_sensor_val.generic_sensor_avg_dec = sensor_log[i].generic_sensor_val.generic_sensor_acc_dec / sensor_log[i].samples;
			/** Writes pressure_log in sector _pressure_start_address */
			__programWord(i);
		}
	}
}

uint8_t generic_sensor_log_CheckDataInMem( uint32_t num )
{
	uint8_t ret = ( sensor_log[num].msg_rd_ini_addr != sensor_log[num].msg_rd_end_addr )?1:0;

	return ret;
}

void generic_sensor_log_recover_rd_address_backup( uint32_t num )
{
	sensor_log[num].msg_rd_address  = sensor_log[num].msg_rd_address_backup;
	sensor_log[num].msg_wr_address  = sensor_log[num].msg_rd_address;
	sensor_log[num].msg_rd_ini_addr = sensor_log[num].msg_rd_address_backup;
	sensor_log[num].msg_num         = sensor_log[num].msg_num_backup;

	HAL_Delay(100);
	__programWord(num);
}

/**
  * @brief Updates pressure_log max, min and avg and writes pressure_log to FLASH.
  * @retval 0
  */
static uint8_t __writeGenericSensorLog( uint32_t num )
{
	float_t m;

	/** Increments samples counter and reads pressure value */
	sensor_log[num].samples++;
	m = sensor_log[num].generic_sensor_val.generic_sensor_val_dec;

	/* First time pressure_log.init = 0 -> after that pressure_log.init = 0xAAAA
	 * initializes pressure_log to the default values */
	if (sensor_log[num].init != 0xAAAA)
	{
		sensor_log[num].init               = 0xAAAA;
		sensor_log[num].msg_num            = 0;
		sensor_log[num].msg_wr_address     = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_address     = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_ini_addr    = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_end_addr    = generic_sensor_telegram_values_addresses[num];
		sensor_telegram_value[num].wr_addr = generic_sensor_telegram_values_addresses[num];
		sensor_telegram_value[num].rd_addr = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].generic_sensor_val.generic_sensor_min_dec = m;
		sensor_log[num].generic_sensor_val.generic_sensor_max_dec = m;
		sensor_log[num].generic_sensor_val.generic_sensor_acc_dec = 0;
		sensor_log[num].samples = 1;
		sensor_log[num].generic_sensor_val.generic_sensor_avg_dec = sensor_log[num].generic_sensor_val.generic_sensor_acc_dec / sensor_log[num].samples;
	}

	/** Updates average, maximum and minimum values */
	sensor_log[num].generic_sensor_val.generic_sensor_acc_dec += m;
	sensor_log[num].generic_sensor_val.generic_sensor_avg_dec = sensor_log[num].generic_sensor_val.generic_sensor_acc_dec / sensor_log[num].samples;

    if (m > sensor_log[num].generic_sensor_val.generic_sensor_max_dec)
    {
    	sensor_log[num].generic_sensor_val.generic_sensor_max_dec = m;
    }

    if (m < sensor_log[num].generic_sensor_val.generic_sensor_min_dec)
    {
    	sensor_log[num].generic_sensor_val.generic_sensor_min_dec = m;
    }

    /** Converts float data to a string */
    common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_avg_dec, sensor_log[num].generic_sensor_val.generic_sensor_avg, DECIMAL_DIGITS);
    common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_min_dec, sensor_log[num].generic_sensor_val.generic_sensor_min, DECIMAL_DIGITS);
    common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_max_dec, sensor_log[num].generic_sensor_val.generic_sensor_max, DECIMAL_DIGITS);

    /** Writes pressure_log in sector _pressure_start_address 0x700000*/
	__programWord(num);

    return 0;
}

/**
  * @brief Reads the pressure Log from FLASH memory and stores it to pressure_log structure.
  * @retval 0
  */
static uint8_t __readGenericSensorLog(uint32_t num)
{
	if ( 2 == generic_sensor_log_save_programword )
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> EXECUTE __readGenericSensorLog().\r\n", (int)Tick_Get( SECONDS ));
		HAL_Delay(100);
		sFLASH_ReadBuffer((uint8_t *) &sensor_log[num], (uint32_t) generic_sensor_log_addresses[num], sizeof(sensor_log[num]));
	}
	return 0;
}

/**
 * @fn uint32_t __checkLowGenericSensorAlarm(uint32_t num)
 * @brief low pressure alarm
 *
 * @return low_pressure_alarm
 * 			@arg 0 - Alarm has not occurred.
 * 			@arg 1 - Alarm has occurred.
 */
static uint32_t __checkLowGenericSensorAlarm(uint32_t num)
{
	uint32_t ret = 0;
	static uint32_t and_condition_break = 0;

	if ( 0 == num )
	{
		and_condition_break = 0;
	}
	if (0xFFFFFFFF != params_get_low_sensor_alarm(num))
	{
		if ( (sensor_log[num].generic_sensor_val.generic_sensor_val_dec < params_get_low_sensor_alarm(num)))
		{
			if (((AND_CONDITION == params_get_low_sensor_condition()) && (num == params_input_pulse_as_sensor_get_num() - 1) && ( 0 == and_condition_break ) )
					||(OR_CONDITION  == params_get_low_sensor_condition()) )
			{
				generic_sensor_alarms[num].lowsensor_alarm  = 1;
				generic_sensor_alarms[num].oversensor_alarm = 0;
				ret = 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> LOW sensor %d ALARM!!!!!!!!!!!!!!!!!!!.\r\n", (int)Tick_Get( SECONDS ), (int)num);
			}
			else
			{
				generic_sensor_alarms[num].oversensor_alarm = 0;
				generic_sensor_alarms[num].lowsensor_alarm  = 0;
			}
		}
		else
		{
			generic_sensor_alarms[num].oversensor_alarm = 0;
			generic_sensor_alarms[num].lowsensor_alarm  = 0;
			if (AND_CONDITION == params_get_over_sensor_condition())
			{
				and_condition_break = 1;
			}
		}
	}
	else
	{
		generic_sensor_alarms[num].oversensor_alarm = 0;
		generic_sensor_alarms[num].lowsensor_alarm  = 0;
	}
	return ret;
}

/**
 * @fn __checkOverGenericSensorAlarm( uint32_t num )
 * @brief
 *
 * @return Over_pressure_alarm
 * 			@arg 0 - Alarm has not occurred.
 * 			@arg 1 - Alarm has occurred.
 */
static uint32_t __checkOverGenericSensorAlarm( uint32_t num )
{
	uint32_t ret = 0;
	static uint32_t and_condition_break = 0;

	if ( 0 == num )
	{
		and_condition_break = 0;
	}
	if (0xFFFFFFFF != params_get_over_sensor_alarm(num))
	{
		if ( (sensor_log[num].generic_sensor_val.generic_sensor_val_dec > params_get_over_sensor_alarm(num)))
		{
			if (((AND_CONDITION == params_get_over_sensor_condition()) && (num == params_input_pulse_as_sensor_get_num() - 1) && ( 0 == and_condition_break ) )
					||(OR_CONDITION  == params_get_over_sensor_condition()) )
			{
				generic_sensor_alarms[num].oversensor_alarm = 1;
				generic_sensor_alarms[num].lowsensor_alarm  = 0;
				ret = 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> OVER sensor %d ALARM!!!!!!!!!!!!!!!!!!!.\r\n", (int)Tick_Get( SECONDS ), (int)num);
			}
			else
			{
				generic_sensor_alarms[num].oversensor_alarm = 0;
				generic_sensor_alarms[num].lowsensor_alarm  = 0;
			}
		}
		else
		{
			generic_sensor_alarms[num].oversensor_alarm = 0;
			generic_sensor_alarms[num].lowsensor_alarm  = 0;
			if (AND_CONDITION == params_get_over_sensor_condition())
			{
				and_condition_break = 1;
			}
		}
	}
	else
	{
		generic_sensor_alarms[num].oversensor_alarm = 0;
		generic_sensor_alarms[num].lowsensor_alarm  = 0;
	}
	return ret;
}

/**
 * @fn generic_sensor_log_GetOverGenericSensorAlarm( uint32_t num )
 * @brief Checks for alarm.
 *
 * @return overpressure_alarm
 * 			@arg 0 - alarm has not occurred.
 * 			@arg 1 - alarm has occurred.
 */
uint32_t generic_sensor_log_GetOverGenericSensorAlarm( uint32_t num )
{
	/** Reads pressure */
	generic_sensor_log_read_sensor(num);
	/** Compares with overpressure alarm value */
	__checkOverGenericSensorAlarm(num);

	return generic_sensor_alarms[num].oversensor_alarm;
}

/**
 * @fn generic_sensor_log_GetLowGenericSensorAlarm( uint32_t num )
 * @brief Checks for lowerpressure alarm.
 *
 * @return overpressure_alarm
 * 			@arg 0 - Lowerpressure alarm has not occurred.
 * 			@arg 1 - Lowerpressure alarm has occurred.
 */
uint32_t generic_sensor_log_GetLowGenericSensorAlarm( uint32_t num )
{
	/** Reads pressure */
	generic_sensor_log_read_sensor(num);
	/** Compares with lowerpressure alarm value */
	__checkLowGenericSensorAlarm(num);

	return generic_sensor_alarms[num].lowsensor_alarm;
}

uint32_t generic_sensor_log_GetOperatingSensorAlarm( uint32_t num )
{
	return generic_sensor_alarms[num].operatingsensor_alarm;
}

void generic_sensor_log_SetOperatingSensorAlarm( uint32_t num, uint32_t _operating_alarm )
{
	generic_sensor_alarms[num].operatingsensor_alarm = _operating_alarm;
}

char * generic_sensor_log_GetGenericSensorAvg( uint32_t num )
{
	__readGenericSensorLog(num);

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_avg_dec, sensor_log[num].generic_sensor_val.generic_sensor_avg, DECIMAL_DIGITS);

	return sensor_log[num].generic_sensor_val.generic_sensor_avg;
}

char * generic_sensor_log_GetGenericSensorMin( uint32_t num )
{
	__readGenericSensorLog(num);

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_min_dec, sensor_log[num].generic_sensor_val.generic_sensor_min, DECIMAL_DIGITS);

	return sensor_log[num].generic_sensor_val.generic_sensor_min;
}

char * generic_sensor_log_GetGenericSensorMax( uint32_t num )
{
	__readGenericSensorLog(num);

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_max_dec, sensor_log[num].generic_sensor_val.generic_sensor_max, DECIMAL_DIGITS);

	return sensor_log[num].generic_sensor_val.generic_sensor_max;
}


void generic_sensor_log_read_sensor(uint32_t num)
{
	if (0==num)
	{
		sensor_log[num].generic_sensor_val.generic_sensor_val_dec = (float_t)generic_sensor_get_input_read(num)/1000;
	}
	else
	{
	sensor_log[num].generic_sensor_val.generic_sensor_val_dec = (float_t)generic_sensor_get_input_read(num);
	}

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_val_dec, sensor_log[num].generic_sensor_val.generic_sensor_val, DECIMAL_DIGITS);
	//	printf("%.*s%s", n >= 10 ? 0 : (int)(10-n), "0000000000", s);
	int n = strlen(sensor_log[num].generic_sensor_val.generic_sensor_val);
	LOGLIVE(LEVEL_2, "LOGLIVE> %d GENERIC_SENSOR_LOG> generic sensor log%d: %.*s%s.\r\n",
			(int)Tick_Get( SECONDS ),
			(int)num,
			n >= 6 ? 0 : (int)(6-n), "000000",sensor_log[num].generic_sensor_val.generic_sensor_val);
}

char generic_sensor_info[400];
char *generic_sensor_log_GetDatalogger_Info(void)
{
	uint32_t i, strlen = 0;
	memset(generic_sensor_info,0,sizeof(generic_sensor_info));
	for (i=0; i<params_input_pulse_as_sensor_get_num(); i++)
	{
//		if ( generic_sensor_get_input_read(i) != NULL )
		{
			sprintf(generic_sensor_info + strlen,
					"%d,0x%X,0x%X;",
					(int)sensor_log[i].msg_num,
					(int)sensor_log[i].msg_rd_address_backup,
					(int)sensor_log[i].msg_wr_address
			);
			strlen = strnlen(generic_sensor_info, sizeof(generic_sensor_info));
		}
	}

	if ( strlen > 0 )
	{
		generic_sensor_info[strlen - 1] = '\0';
	}

	return generic_sensor_info;
}

/**
  * @brief
  * @retval None
  */
void generic_sensor_log_Log(uint32_t num)
{
	/** Reads pressure_log from FLASH, updates pressure_log register with new values
	 * and writes back to FLASH */
	sFLASH_Init_Test();
	__readGenericSensorLog(num);
	generic_sensor_log_read_sensor(num);
	__writeGenericSensorLog(num);
}
/**
  * @brief
  * @retval Returns the next address to write
  */
static uint32_t __nextAddress(uint32_t num, uint32_t n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr   = sensor_log[num].msg_wr_address  + n;
	next_sector = (sensor_log[num].msg_wr_address + 0x1000) & 0xFFFFF000;

    if ((sensor_log[num].msg_wr_address & 0x00000FFF) == 0)
    {
    	sFLASH_EraseSector(sensor_log[num].msg_wr_address);
    }
    else if (next_addr > next_sector)
    {
    	sFLASH_EraseSector(next_sector);
    }

    addr                            = sensor_log[num].msg_wr_address;
    sensor_log[num].msg_wr_address += n;
    if (sensor_log[num].msg_wr_address > generic_sensor_telegram_values_max_addresses[num])
    {
    	sensor_log[num].msg_wr_address = generic_sensor_telegram_values_addresses[num];
    }

    if ((sensor_log[num].msg_wr_address == sensor_log[num].msg_rd_end_addr))
    {
    	sensor_log[num].msg_rd_end_addr += sensor_telegram_log_size;
    }
    sensor_log[num].msg_rd_ini_addr = sensor_log[num].msg_wr_address;
    sensor_log[num].msg_rd_address = sensor_log[num].msg_wr_address;

    return addr;
}

/**
  * @brief
  * @retval 0
  */
static uint8_t __programTelegramValueWord( uint32_t num )
{
    uint32_t  size, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  *ptr;
	uint32_t  next_addr;
	sensor_telegram_values_t record_check;

    size = sizeof(sensor_telegram_value[num]);
    ptr  = (uint8_t *) sensor_telegram_value[num].sensor_val;

    /** Gets the number of FLASH pages required to write the whole telegram
     *  sFLASH_SPI_PAGESIZE = 256 bytes and pressure_telegram_value size = 60 bytes */
    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 )
    {
    	sensor_telegram_log_size = (num_pages + 1) * sFLASH_SPI_PAGESIZE;
    }
    else
    {
    	sensor_telegram_log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    /** Gets the next address in memory to write the following chunk of data, since the
     * size is smaller than a page, this will be next page */
    next_addr = __nextAddress(num, sensor_telegram_log_size);
    LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);

    uint32_t Flash_IDent = sFLASH_ReadID();
	sFLASH_WaitBusy();

	if (0==Flash_IDent)
	{
//		sFLASH_Init();
//		sFLASH_initSST26V();
		sFLASH_Init_Test();
	}
    /** Writes a page to FLASH in next_address and reads back to check write integrity */
    sFLASH_WriteBuffer(ptr, next_addr, size);
    HAL_Delay(10);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	__NOP();
    	check_write = 0;
    } else {
    	__NOP();
    	check_write = 1;
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> WARNING!! CHECK WRITE in Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    }

    /**WARNING: In order to write to FLASH the sector needs to be empty first */
    uint32_t times = 0;
    if ( 1 == check_write ) {
    	do {
    		sFLASH_EraseSector(next_addr&0xFFFFF000);
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		times++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> RECOVERED!! CHECK WRITE in Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)_pressure_start_address);
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( times < 5 ) );
    }

    return ret;
}

char * generic_sensor_log_GetPressureAvg( uint32_t num )
{
	__readGenericSensorLog(num);

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_avg_dec, sensor_log[num].generic_sensor_val.generic_sensor_avg, DECIMAL_DIGITS);

	return sensor_log[num].generic_sensor_val.generic_sensor_avg;
}

char * generic_sensor_log_GetPressureMin( uint32_t num )
{
	__readGenericSensorLog(num);

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_min_dec, sensor_log[num].generic_sensor_val.generic_sensor_min, DECIMAL_DIGITS);

	return sensor_log[num].generic_sensor_val.generic_sensor_min;
}

char * generic_sensor_log_GetPressureMax( uint32_t num )
{
	__readGenericSensorLog(num);

	common_lib_ftoa(sensor_log[num].generic_sensor_val.generic_sensor_max_dec, sensor_log[num].generic_sensor_val.generic_sensor_max, DECIMAL_DIGITS);

	return sensor_log[num].generic_sensor_val.generic_sensor_max;
}

char *generic_sensor_log_message_value_sensor( uint32_t num )
{
	float_t sensor = 0.0;

	sensor = (float_t)generic_sensor_get_input_read(num)/1000;

	common_lib_ftoa( sensor, message_value_sensor[num], DECIMAL_DIGITS );

	if ( 0 == sensor ) {
//		generic_sensor_log_SetOperatingAlarm(1);
	}

	return message_value_sensor[num];
}

/**
  * @brief  Stores data from structure pressure_log to pressure_log_telegram_value to get it ready to send.
  * @retval None
  */
time_t generic_sensor_first_value_time = 0;
uint32_t generic_sensor_first_msg = 0xFFFFFFFF;
void generic_sensor_log_pressure_store_telegram_value(uint32_t num)
{
#define  CHECK_TAMPER	(HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ))
	if (sensor_log[num].init == 0xAAAA)
	{
		if ( 0xFFFFFFFF == generic_sensor_first_msg )
		{
			generic_sensor_first_msg        = sensor_log[num].msg_num;
			generic_sensor_first_value_time = Tick_Get(SECONDS);
		}
		if ( ( sensor_log[num].msg_num < 1 ) || ( generic_sensor_first_msg > sensor_log[num].msg_num ) )
		{
			generic_sensor_first_msg        = sensor_log[num].msg_num;
			generic_sensor_first_value_time = Tick_Get(SECONDS);
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> first_msg:%d first_value_time:%d\r\n", (int)Tick_Get( SECONDS ), (int)generic_sensor_first_msg, (int)generic_sensor_first_value_time);
		sensor_log[num].msg_num++;

		strncpy(sensor_telegram_value[num].sensor_val, sensor_log[num].generic_sensor_val.generic_sensor_val, 7);
		strncpy(sensor_telegram_value[num].sensor_avg, sensor_log[num].generic_sensor_val.generic_sensor_avg, 7);
		strncpy(sensor_telegram_value[num].sensor_max, sensor_log[num].generic_sensor_val.generic_sensor_max, 7);
		strncpy(sensor_telegram_value[num].sensor_min, sensor_log[num].generic_sensor_val.generic_sensor_min, 7);

		sensor_telegram_value[num].sensor_id = num + 1;
		sensor_telegram_value[num].msg_num++;

		memcpy(sensor_telegram_value[num].created_sensor_value_date, (char * ) (rtc_system_getCreatedSensorValueDate()), \
				sizeof(sensor_telegram_value[num].created_sensor_value_date));
		memcpy(sensor_telegram_value[num].created_sensor_value, (char *) (rtc_system_getCreatedSensorValueTime()), \
				sizeof(sensor_telegram_value[num].created_sensor_value));

		LOGLIVE(LEVEL_2, "\r\nLOGLIVE> %d GENERIC SENSOR LOG> %s%s Pavg:%s. Pinst:%s. Msg.num:%d.\r\n", (int)Tick_Get( SECONDS ),
				sensor_telegram_value[num].created_sensor_value_date,
				sensor_telegram_value[num].created_sensor_value,
				sensor_telegram_value[num].sensor_avg, sensor_telegram_value[num].sensor_val,
				(int)sensor_log[num].msg_num);
		if (CHECK_TAMPER == 0)
		{
			leds_LED_On(LED_BLUE);
		}
		__programTelegramValueWord(num);
		HAL_Delay(10);
		/**TOASK programWord is required because msg_num++?? it could be added before in sensor_log_Log*/
		__programWord(num);
		leds_LED_Off(LED_GREEN);
	}
}

static void __incReadGenericSensorLog( uint32_t num )
{
    uint32_t size = sizeof(sensor_telegram_value[num]);

    /** Gets the number of FLASH pages required to write the whole telegram
     *  sFLASH_SPI_PAGESIZE = 256 bytes and pressure_telegram_value size = 60 bytes */
    uint32_t num_pages     = size / sFLASH_SPI_PAGESIZE;
    uint32_t num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 )
    {
    	sensor_telegram_log_size = (num_pages + 1) * sFLASH_SPI_PAGESIZE;
    }
    else
    {
    	sensor_telegram_log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }
    if ( sensor_log[num].msg_rd_ini_addr - sensor_telegram_log_size >= (uint32_t) generic_sensor_telegram_values_addresses[num] ) {
    	sensor_log[num].msg_rd_ini_addr -= sensor_telegram_log_size;
    } else {//TODO: CHECK THIS CASE!!!!!!( + 1??)
    	if ( generic_sensor_telegram_values_max_addresses[num] - sensor_telegram_log_size > sensor_log[num].msg_rd_end_addr ) {
    		sensor_log[num].msg_rd_ini_addr = generic_sensor_telegram_values_max_addresses[num] - sensor_telegram_log_size + 1;//+1??
    	} else {
    		sensor_log[num].msg_rd_ini_addr = sensor_log[num].msg_rd_end_addr;
    	}
    }

    if ( ( sensor_log[num].msg_rd_ini_addr <= sensor_log[num].msg_rd_end_addr ) ) {
//        ret = 0;
    }

    sensor_log[num].msg_wr_address = sensor_log[num].msg_rd_ini_addr;
    sensor_log[num].msg_rd_address = sensor_log[num].msg_rd_ini_addr;
}

char * generic_sensor_log_sensor_num_telegram( uint32_t *measure_count, uint32_t *sensor_id, uint32_t num )
{
	uint32_t i, str_len = 0, num_it = 0, rd_ini_backup = 0, wr_backup = 0, rd_backup = 0;
	char date[10];

	__readGenericSensorLog(num);

	num_it = sensor_log[num].msg_num;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> msg num:%d \r\n",(int)Tick_Get( SECONDS ), (int)num_it);
	if ( 0 == num_it ) {
		return NULL;
	}
	memset(sensor_telegram_string[num], 0, sizeof(sensor_telegram_string[num]));
	rd_ini_backup                      = sensor_log[num].msg_rd_ini_addr;
	wr_backup                          = sensor_log[num].msg_wr_address;
	rd_backup                          = sensor_log[num].msg_rd_address;
	sensor_log[num].msg_rd_address_backup = sensor_log[num].msg_rd_ini_addr;//pressure_log.msg_rd_address;
	sensor_log[num].msg_num_backup        = sensor_log[num].msg_num;
	*measure_count                     = *measure_count + 1;
	*sensor_id                         = sensor_log[num].sensor_id;
	__incReadGenericSensorLog(num);
	sensor_log[num].msg_num--;
	if ( sensor_log[num].msg_num > 0 ) {
//		num_it = pressure_log.msg_num;
	}

	for ( i = 0; i < num_it; i++ ) {
		sFLASH_ReadBuffer( (uint8_t *)&sensor_telegram_value[num],
				sensor_log[num].msg_rd_ini_addr,
				sizeof( sensor_telegram_value[num] ) );
		LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> rd_ini:0x%X len:%d\r\n",(int)Tick_Get( SECONDS ), (int)sensor_log[num].msg_rd_ini_addr, (int)str_len);
		*sensor_id = sensor_telegram_value[num].sensor_id;
		if ( (  str_len
				+ sizeof(sensor_telegram_value[num])
		     ) >= ( MAX_GENERIC_SENSOR_TELEGRAM_VALUES - 250 ) ) {
			sensor_log[num].msg_rd_ini_addr = rd_ini_backup;
			sensor_log[num].msg_wr_address  = wr_backup;
			sensor_log[num].msg_rd_address  = rd_backup;
			sensor_log[num].msg_num         = sensor_log[num].msg_num + 1;
			*measure_count               = *measure_count - 1;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> Frame End:%d Count:%d\r\n", (int)Tick_Get( SECONDS ), (int)sensor_log[num].msg_rd_ini_addr, (int)(*measure_count));
			break;
		}
		if ( 0xFF == sensor_telegram_value[num].sensor_val[0] ) {
			LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> SENSOR MEMORY ERROR!!!\r\n", (int)Tick_Get( SECONDS ));
			continue;
		}
		if ( ( 0 == i )
		  || ( sensor_telegram_value[num].created_sensor_value_date[6] != date[6] )
		  || ( sensor_telegram_value[num].created_sensor_value_date[7] != date[7] )
		  ) {
			if ( 0 == i ) {
				snprintf(sensor_telegram_string[num],
						sizeof( sensor_telegram_value[num].created_sensor_value_date ) + 1,
						"%s|", sensor_telegram_value[num].created_sensor_value_date);
				str_len = strlen(sensor_telegram_string[num]);
			} else if (( sensor_telegram_value[num].created_sensor_value_date[6] != date[6] )
				    || ( sensor_telegram_value[num].created_sensor_value_date[7] != date[7] )) {
				sensor_log[num].msg_rd_ini_addr = rd_ini_backup;
				sensor_log[num].msg_wr_address  = wr_backup;
				sensor_log[num].msg_rd_address  = rd_backup;
				sensor_log[num].msg_num         = sensor_log[num].msg_num + 1;
				*measure_count                  = *measure_count - 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d GENERIC SENSOR LOG> Frame End:%d Count:%d\r\n",(int)Tick_Get( SECONDS ), (int)sensor_log[num].msg_rd_ini_addr, (int)(*measure_count));
				break;
			}
		}
//	    printf("%.*s%s", n >= 10 ? 0 : (int)(10-n), "0000000000", s);
		sensor_telegram_value[num].sensor_val[6] = 0;
		sensor_telegram_value[num].sensor_avg[6] = 0;
		sensor_telegram_value[num].sensor_min[6] = 0;
		sensor_telegram_value[num].sensor_max[6] = 0;
		int n_ins = strlen(sensor_telegram_value[num].sensor_val);
		int n_avg = strlen(sensor_telegram_value[num].sensor_avg);
		int n_min = strlen(sensor_telegram_value[num].sensor_min);
		int n_max = strlen(sensor_telegram_value[num].sensor_max);
		sprintf(sensor_telegram_string[num] + str_len,
				"%s%.*s%s%.*s%s%.*s%s%.*s%s",
				sensor_telegram_value[num].created_sensor_value,
				n_ins >= 6 ? 0 : (int)(6-n_ins),"000000",sensor_telegram_value[num].sensor_val,
				n_avg >= 6 ? 0 : (int)(6-n_avg),"000000",sensor_telegram_value[num].sensor_avg,
				n_min >= 6 ? 0 : (int)(6-n_min),"000000",sensor_telegram_value[num].sensor_min,
				n_max >= 6 ? 0 : (int)(6-n_max),"000000",sensor_telegram_value[num].sensor_max);
		str_len = strlen(sensor_telegram_string[num]);
		*measure_count = *measure_count + 1;
		if ( sensor_log[num].msg_num > 0 ) {
			rd_ini_backup = sensor_log[num].msg_rd_ini_addr;
			wr_backup     = sensor_log[num].msg_wr_address;
			rd_backup     = sensor_log[num].msg_rd_address;
			__incReadGenericSensorLog(num);
			sensor_log[num].msg_num--;
		} else {
			break;
		}
		memcpy( date,
				sensor_telegram_value[num].created_sensor_value_date,
				sizeof(sensor_telegram_value[num].created_sensor_value_date) + 1 );
	}

	if (sensor_telegram_string[num][0] == '\0') {
		sprintf(sensor_telegram_string[num],
				"%s;%s;%s;%s|",
				generic_sensor_log_message_value_sensor(num),
				generic_sensor_log_GetPressureAvg(num),
				generic_sensor_log_GetPressureMax(num),
				generic_sensor_log_GetPressureMin(num));
	}

	sensor_log[num].samples                                   = 0;
	sensor_log[num].generic_sensor_val.generic_sensor_min_dec = sensor_log[num].generic_sensor_val.generic_sensor_val_dec;
	sensor_log[num].generic_sensor_val.generic_sensor_max_dec = sensor_log[num].generic_sensor_val.generic_sensor_val_dec;
	sensor_log[num].generic_sensor_val.generic_sensor_acc_dec = 0;

	HAL_Delay(10);
	__programWord(num);

	return sensor_telegram_string[num];
}

void generic_sensor_log_pressure_reset_telegram_values( uint32_t num )
{
#if 0
	if ( sensor_log[num].init == 0xAAAA ) {
		LOGLIVE(LEVEL_1, "LOGLIVE> %d sensor log> Reset telegram values\r\n", (int)Tick_Get( SECONDS ));
		sensor_log[num].msg_num               = 0;
		sensor_log[num].msg_num_backup        = 0;
		sensor_log[num].msg_wr_address        = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_address        = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_address_backup = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_ini_addr       = generic_sensor_telegram_values_addresses[num];
		sensor_log[num].msg_rd_end_addr       = generic_sensor_telegram_values_addresses[num];
		sensor_telegram_value[num].wr_addr    = generic_sensor_telegram_values_addresses[num];
		sensor_telegram_value[num].rd_addr    = generic_sensor_telegram_values_addresses[num];
		__programWord(num);
	}
#endif
	if ( sensor_log[num].init == 0xAAAA )
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d sensor log> Reset telegram values to next sector.\r\n", (int)Tick_Get( SECONDS ));

		uint32_t next_sector = (sensor_log[num].msg_rd_address_backup + 0x1000) & 0xFFFFF000;
		if ( next_sector >= generic_sensor_telegram_values_max_addresses[num] )
		{
			next_sector = generic_sensor_telegram_values_addresses[num];
		}
		sensor_log[num].msg_num               = 0;
		sensor_log[num].msg_num_backup        = 0;
		sensor_log[num].msg_wr_address        = next_sector;
		sensor_log[num].msg_rd_address        = next_sector;
		sensor_log[num].msg_rd_address_backup = next_sector;
		sensor_log[num].msg_rd_ini_addr       = next_sector;
		sensor_log[num].msg_rd_end_addr       = next_sector;
		sensor_telegram_value[num].wr_addr    = next_sector;
		sensor_telegram_value[num].rd_addr    = next_sector;
		__programWord(num);

		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> pressure_log.msg_wr_address:0x%X.\r\n",
				(int)Tick_Get( SECONDS ), (int)sensor_log[num].msg_wr_address);
	}
}
