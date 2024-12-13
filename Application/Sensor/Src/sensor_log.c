/**
  ******************************************************************************
  * @file           sensor_log.c
  * @author 		Datakorum Development Team
  * @brief          Driver to store pressure sensor data to FLASH memory.
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
 * sensor_log.c
 *
 *  Created on: 14 oct. 2019
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "common_lib.h"

#include "udp_protocol.h"
#include "sensor_log.h"
#include "rtc_system.h"
#include "modbus.h"
#include "generic_modbus.h"
#include "ad.h"
#include "une82326.h"
#include "mbus.h"
#include "pulses.h"
#include "gpio.h"
#include "spi_flash.h"
#include "leds.h"
#include "tick.h"
#include "shutdown.h"
#include "message_queue.h"
#include "params.h"

#ifdef EXT_SENSOR
/**
 * @def SENSOR_INTERVAL_WRITE
 * @brief Analog sensor Sampling period
 *
 */
#define SENSOR_INTERVAL_WRITE (30U)


/** @brief Structure that holds pressure values to save the pressure data in flash */
typedef struct _pressure_vals {
	char    pressure_val[7];		/*!< Instantaneous pressure value in string */
	char    pressure_avg[7];		/*!< Average pressure value in string */
	char    pressure_max[7];		/*!< Maximum pressure value in string */
	char    pressure_min[7];		/*!< Minimum pressure value string */
	float_t pressure_val_dec;		/*!< Current pressure value in float */
	float_t pressure_acc_dec;		/*!< Accumulated pressure value in float */
	float_t pressure_avg_dec;		/*!< Average pressure value in float */
	float_t pressure_max_dec;		/*!< Maximum pressure value in float */
	float_t pressure_min_dec;		/*!< Minimum pressure value in float */
} pressure_values;

/** @brief Structure to save the pressure data, pointers and counters in flash
 * this is saved to _pressure_start_address = 0x700000
 * */
typedef struct {
	pressure_values pressure;					/*!< Pressure data */
	uint32_t        samples;					/*!< Number of pressure data samples */
	uint32_t        init;						/*!< Token to check that structure is initialized */
	uint32_t        msg_num;					/*!< Number of telegrams sent */
	uint32_t        sensor_id;				    /*!<  Sensor identifier */
	uint32_t        msg_num_backup;				/*!<  */
	uint32_t        msg_wr_address;				/*!<  */
	uint32_t        msg_rd_address;				/*!<  */
	uint32_t        msg_rd_address_backup;		/*!<  */
    uint32_t        msg_rd_ini_addr;			/*!<  */
    uint32_t 		msg_rd_end_addr;			/*!<  */
    uint32_t 		msg_rd_end_addr_backup;		/*!<  */
} st_pressure_log;

static st_pressure_log pressure_log;

/** @brief Structure to save the pressure data telegram in flash */
typedef struct {
	char     pressure_val[7];				/*!<  Instantaneous pressure value in string */
	char     pressure_avg[7];				/*!<  Average pressure value in string */
	char     pressure_max[7];				/*!<  Maximum pressure value in string */
	char     pressure_min[7];				/*!<  Minimum pressure value in string */
	uint32_t pulses_count[4];				/*!<  Pulses counter */
	char     created_sensor_value[7];		/*!<  Pressure value time stamp */
	char     created_sensor_value_date[9];	/*!<  Pressure value date */
	uint32_t msg_num;						/*!<  Message counter */
	uint32_t wr_addr;						/*!<  Pointer to address to write to memory */
	uint32_t rd_addr;						/*!<  Pointer to address to read from memory*/
	uint32_t pulse_while_sending;			/*!<  Read pulses while sending frames*/
} pressure_telegram_values;//#define NITEMS(x) (sizeof(x)/sizeof(x[0]))

/** Structure pressure telegram initialization */
static pressure_telegram_values pressure_telegram_value = {
		.pressure_val = {0},
		.pressure_avg = {0},
		.pressure_max = {0},
		.pressure_min = {0},
		.pulses_count = {0},
		.msg_num      = 0,
		.wr_addr      = 0,
		.rd_addr      = 0
};


char pressure_telegram_string[MAX_PRESSURE_TELEGRAM_VALUES];

/**  Structure for pressure alarms */
static struct {
	uint32_t overpressure_alarm;	/*!<  */
	uint32_t lowpressure_alarm;		/*!<  */
	uint32_t operating_alarm;		/*!<  */
} pressure_alarms;

/**
 *	variables defined in linker script.
 *  _pressure_start_address  = 0x700000;
 * _pressure_meas_size 		 = 64K;
 *
 * */
extern char _pressure_start_address[], _pressure_meas_size[];

#define INIT  (0)
#define INDEX (1)

#define PRESSURE_PAGE_SIZE sFLASH_SPI_PAGESIZE

static char message_value_pressure[7];

uint32_t pressure_page_address = (uint32_t) _pressure_start_address + 0x1000;
float_t  pressure_page[PRESSURE_PAGE_SIZE];

uint32_t pressure_telegram_values_address = (uint32_t) _pressure_start_address + 0x2000;
uint32_t pressure_telegram_values_max_address = (uint32_t) _pressure_start_address + 0x200000 - 1;//(uint32_t)_pressure_start_address + 0x80000 - 1;
static uint32_t  pressure_telegram_log_size = 0;

uint32_t sensor_log_write_pulse_acc = 0;

uint32_t sensor_log_save_programword = 0;
static uint8_t __readPressureLog(void);

// reverses a string 'str' of length 'len'
static void __reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[]. d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
static int __intToStr(int x, char str[], int d)
{
	int i = 0;
	if (x == 0) {
		str[i++] = '0';
	}
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	__reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
static void __ftoa(float n, char *res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	int i = __intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		__intToStr((int)fpart, res + i + 1, afterpoint);
	}
}

uint32_t sensor_log_get_write_pulse_acc(void)
{
	return sensor_log_write_pulse_acc;
}

void sensor_log_set_write_pulse_acc(uint32_t _acc)
{
	sensor_log_write_pulse_acc = _acc;
}

/**
  * @brief Reads pressure_page block memory and stores it in pressure_page array
  * @retval 0
  */
uint8_t __readPressureValuePage(void)
{
	sFLASH_ReadBuffer((uint8_t * ) &pressure_page, pressure_page_address, sizeof(pressure_page));

	return 0;
}

/**
  * @brief Updates new pressure value in pressure_page_block
  * @retval None
  */
static void __programPressureValuePage(void)
{
	uint8_t index;
	uint32_t size_page = sizeof(pressure_page);
	uint32_t token;

	/** Reads pressure_page_address _pressure_start_address + 0x1000 */
	sFLASH_ReadBuffer((uint8_t *) &pressure_page, pressure_page_address, size_page);

	/* First time ever. Reads token and if not correct 0xA5 token deletes pressure_page structure
	 * and initializes index -> first element */
	token = (uint32_t) pressure_page[0];
	if (token != 0xA5)
	{
		memset(pressure_page, 0, sizeof(pressure_page));
		pressure_page[INIT] = 0xA5;
		pressure_page[INDEX] = 1;
	}

	/* increments index */
	index = (uint8_t) pressure_page[INDEX] + 1;

	/* Index overflow */
	if (index == PRESSURE_PAGE_SIZE - 14)
	{
		index = INDEX + 1;
	}

	/** Copy new value from sensor_log to array page pressure_page */
	pressure_page[INDEX] = index;
	pressure_page[index] = pressure_log.pressure.pressure_val_dec;

	/** Clears the sector (_pressure_start_address = 0x700000 + 0x1000) and updates the sector with new updated values */
    sFLASH_EraseSector((uint32_t) pressure_page_address);
    sFLASH_WriteBuffer((uint8_t * ) &pressure_page, (uint32_t) pressure_page_address, size_page);
}

float_t * sensor_log_GetPressureValues(void)
{
	uint8_t current_index, index;

	__readPressureValuePage();

	current_index = pressure_page[ INDEX ];

	if ( current_index > 16 ) {
		index = current_index - 15 + 1;
	} else {
		index = INDEX + 1;
	}

	return &pressure_page[ index ];
}

float_t *sensor_log_GetPressureIndexValue( uint32_t index )
{
	return &pressure_page[ index ];
}

uint8_t sensor_log_GetPressureValuesCurrentIndex( void )
{
	uint8_t current_index;

	__readPressureValuePage();

	current_index = pressure_page[ INDEX ];

	return current_index;
}

char datalogger_pulse_info[100];
char *sensor_log_GetDatalogger_Info(void)
{
	memset(datalogger_pulse_info,0,sizeof(datalogger_pulse_info));
	if ( 1 == AD_GetADOn() ) {
		sprintf(datalogger_pulse_info,
				"%d,0x%X,0x%X",
				(int)pressure_log.msg_num,
				(int)pressure_log.msg_rd_address_backup,
				(int)pressure_log.msg_wr_address
		);
	}
	return datalogger_pulse_info;
}

void sensor_log_set_save_programword( uint32_t __save )
{
	sensor_log_save_programword = __save;
}

uint32_t sensor_log_check_set_programword( uint32_t _diff_time_in_secs )
{
	if ( 1 == rtc_system_time_to_reset(_diff_time_in_secs) )
	{
		sensor_log_save_programword = 1;
	}
	else
	{
		sensor_log_save_programword = 0;
	}
	return 0;
}

/**
  * @brief Writes structure pressure_log in sector of address _pressure_start_address (0x700000)
  * @retval 0
  */
static uint8_t __programWord(void)
{
	uint32_t  size, ret = 0, check_write = 0;
	uint8_t  *ptr;
	st_pressure_log pressure_check;

	if (( 1 == sensor_log_save_programword ) || ( 2 == sensor_log_save_programword ) )
	{
		size = sizeof(pressure_log);
		ptr = (uint8_t *) &pressure_log;
		sensor_log_save_programword = 2;
		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> EXECUTE __programWord()  \r\n", (int)Tick_Get( SECONDS ));
		if (pressure_log.init == 0)
		{
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> WARNING!! init:%d \r\n", (int)Tick_Get( SECONDS ), (int)pressure_log.init);
		}
		sFLASH_EraseSector((uint32_t) _pressure_start_address);
		HAL_Delay(10);//HAL_Delay(100);
		sFLASH_WriteBuffer(ptr, (uint32_t) _pressure_start_address, size);
		HAL_Delay(10);//HAL_Delay(100);
		sFLASH_ReadBuffer( (uint8_t *)&pressure_check, (uint32_t) _pressure_start_address, size );

		if ( 0 == memcmp(ptr, (uint8_t *)&pressure_check, size)) {
			__NOP();
			check_write = 0;
		} else {
			__NOP();
			check_write = 1;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> WARNING!! CHECK WRITE in __programWord init:%d init_check:%d\r\n", (int)Tick_Get( SECONDS ), (int)pressure_log.init, (int)pressure_check.init);
		}

		/**WARNING: In order to write to FLASH the sector needs to be empty first */
		uint32_t num = 0;
		if ( 1 == check_write ) {
			do {
				HAL_Delay(100);
				sFLASH_WriteBuffer(ptr,(uint32_t) _pressure_start_address, size);
				HAL_Delay(100);
				sFLASH_ReadBuffer( (uint8_t *)&pressure_check, (uint32_t)_pressure_start_address, size );
				num++;
				if ( 0 == memcmp(ptr, (uint8_t *)&pressure_check, size)) {
					__NOP();
					check_write = 0;
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> RECOVERED!! CHECK WRITE in __programWord Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)_pressure_start_address);
				} else {
					__NOP();
					check_write = 1;
				}
			} while ( ( 1 == check_write ) && ( num < 5 ) );
		}
	}
    return ret;
}

void sensor_log_init(void)
{
	HAL_Delay(10);
	sensor_log_set_save_programword(2);
	__readPressureLog();
	if (pressure_log.init != 0xAAAA)
	{
		pressure_log.init               = 0xAAAA;
		pressure_log.msg_num            = 0;
		pressure_log.sensor_id          = SENSOR_ID;
		pressure_log.msg_wr_address     = pressure_telegram_values_address;
		pressure_log.msg_rd_address     = pressure_telegram_values_address;
		pressure_log.msg_rd_ini_addr    = pressure_telegram_values_address;
		pressure_log.msg_rd_end_addr    = pressure_telegram_values_address;
		pressure_log.msg_rd_address_backup = pressure_telegram_values_address;
		pressure_log.msg_rd_end_addr_backup = pressure_telegram_values_address;
		pressure_telegram_value.wr_addr = pressure_telegram_values_address;
		pressure_telegram_value.rd_addr = pressure_telegram_values_address;
		pressure_log.pressure.pressure_min_dec = 0;
		pressure_log.pressure.pressure_max_dec = 0;
		pressure_log.pressure.pressure_acc_dec = 0;
		pressure_log.samples                   = 1;
		pressure_log.pressure.pressure_avg_dec = pressure_log.pressure.pressure_acc_dec / pressure_log.samples;
	    /** Writes pressure_log in sector _pressure_start_address 0x700000*/
		__programWord();
	}
}

uint8_t sensor_log_WritePressureLogIndex( uint32_t index )
{
	uint32_t i;
	float_t  pressure_float, pressure_acc_float;
	float_t *pressure_index;

	pressure_index = sensor_log_GetPressureValues();//sensor_log_GetPressureIndexValue( index );
	pressure_log.pressure.pressure_max_dec = *pressure_index;
	pressure_log.pressure.pressure_min_dec = *pressure_index;
	pressure_acc_float = 0;
	for (i = 0; i < 15; i++) {
		pressure_float = *(pressure_index + i);
		pressure_acc_float += pressure_float;
		pressure_log.pressure.pressure_avg_dec  = pressure_acc_float/(i + 1);
	    if( pressure_float > pressure_log.pressure.pressure_max_dec ) {
	    	pressure_log.pressure.pressure_max_dec = pressure_float;
	    }
	    if( pressure_float < pressure_log.pressure.pressure_min_dec ) {
	     	pressure_log.pressure.pressure_min_dec = pressure_float;
	     }
	}

    __ftoa(pressure_log.pressure.pressure_avg_dec, pressure_log.pressure.pressure_avg, 3);
    __ftoa(pressure_log.pressure.pressure_min_dec, pressure_log.pressure.pressure_min, 3);
    __ftoa(pressure_log.pressure.pressure_max_dec, pressure_log.pressure.pressure_max, 3);

	__programWord();

	return 0;
}

uint32_t sensor_log_is_pressure_log_init( void )
{
	uint32_t ret = 0;
	if ( 0xAAAA == pressure_log.init ) {
		ret = 1;
	} else {
		ret = 0;
	}
	return ret;
}

uint8_t sensor_log_CheckDataInMem( void )
{
	uint8_t ret = ( pressure_log.msg_rd_ini_addr != pressure_log.msg_rd_end_addr )?1:0;

	return ret;
}

void sensor_log_recover_rd_address_backup( void )
{
	pressure_log.msg_rd_address = pressure_log.msg_rd_address_backup;
	pressure_log.msg_wr_address = pressure_log.msg_rd_address;
	pressure_log.msg_rd_ini_addr = pressure_log.msg_rd_address_backup;
	pressure_log.msg_num = pressure_log.msg_num_backup;

	HAL_Delay(100);
	__programWord();
}

/**
  * @brief Updates pressure_log max, min and avg and writes pressure_log to FLASH.
  * @retval 0
  */
static uint8_t __writePressureLog( void )
{
	float_t m;

	/** Increments samples counter and reads pressure value */
	pressure_log.samples++;
	m = pressure_log.pressure.pressure_val_dec;

	/* First time pressure_log.init = 0 -> after that pressure_log.init = 0xAAAA
	 * initializes pressure_log to the default values */
	if (pressure_log.init != 0xAAAA)
	{
		pressure_log.init = 0xAAAA;
		pressure_log.msg_num = 0;
		pressure_log.msg_wr_address = pressure_telegram_values_address;
		pressure_log.msg_rd_address = pressure_telegram_values_address;
		pressure_log.msg_rd_ini_addr = pressure_telegram_values_address;
		pressure_log.msg_rd_end_addr = pressure_telegram_values_address;
		pressure_telegram_value.wr_addr = pressure_telegram_values_address;
		pressure_telegram_value.rd_addr = pressure_telegram_values_address;
		pressure_log.pressure.pressure_min_dec = m;
		pressure_log.pressure.pressure_max_dec = m;
		pressure_log.pressure.pressure_acc_dec = 0;
		pressure_log.samples = 1;
		pressure_log.pressure.pressure_avg_dec = pressure_log.pressure.pressure_acc_dec / pressure_log.samples;
	}

	/** Updates average, maximum and minimum values */
	pressure_log.pressure.pressure_acc_dec += m;
	pressure_log.pressure.pressure_avg_dec = pressure_log.pressure.pressure_acc_dec / pressure_log.samples;

    if (m > pressure_log.pressure.pressure_max_dec)
    {
    	pressure_log.pressure.pressure_max_dec = m;
    }

    if (m < pressure_log.pressure.pressure_min_dec)
    {
    	pressure_log.pressure.pressure_min_dec = m;
    }

    /** Converts float data to a string */
    __ftoa(pressure_log.pressure.pressure_avg_dec, pressure_log.pressure.pressure_avg, 3);
    __ftoa(pressure_log.pressure.pressure_min_dec, pressure_log.pressure.pressure_min, 3);
    __ftoa(pressure_log.pressure.pressure_max_dec, pressure_log.pressure.pressure_max, 3);

    /** Writes pressure_log in sector _pressure_start_address 0x700000*/
	__programWord();

    return 0;
}

/**
  * @brief Reads the pressure Log from FLASH memory and stores it to pressure_log structure.
  * @retval 0
  */
static uint8_t __readPressureLog(void)
{
	if ( 2 == sensor_log_save_programword )
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> EXECUTE __readPressureLog().\r\n", (int)Tick_Get( SECONDS ));
		sFLASH_ReadBuffer((uint8_t *) &pressure_log, (uint32_t) _pressure_start_address, sizeof(pressure_log));
	}
	return 0;
}

/**
 * @fn uint32_t __checkLowPressureAlarm(void)
 * @brief low pressure alarm
 *
 * @return low_pressure_alarm
 * 			@arg 0 - Alarm has not occurred.
 * 			@arg 1 - Alarm has occurred.
 */
static uint32_t __checkLowPressureAlarm(void)
{
	uint32_t ret = 0;

	if ((0 != params_config_get_lowpressure_alarm()) && (pressure_log.pressure.pressure_val_dec < params_config_get_lowpressure_alarm()))
	{
		pressure_alarms.lowpressure_alarm  = 1;
		pressure_alarms.overpressure_alarm = 0;
		ret = 1;
	}
	else
	{
		pressure_alarms.overpressure_alarm = 0;
		pressure_alarms.lowpressure_alarm  = 0;
	}

	return ret;
}

/**
 * @fn uint32_t __checkOverPressureAlarm(void)
 * @brief
 *
 * @return Over_pressure_alarm
 * 			@arg 0 - Alarm has not occurred.
 * 			@arg 1 - Alarm has occurred.
 */
static uint32_t __checkOverPressureAlarm( void )
{
	uint32_t ret = 0;

	if ((0 != params_config_get_overpressure_alarm()) && (pressure_log.pressure.pressure_val_dec > params_config_get_overpressure_alarm()))
	{
		pressure_alarms.overpressure_alarm = 1;
		pressure_alarms.lowpressure_alarm  = 0;
		ret = 1;
	}
	else
	{
		pressure_alarms.overpressure_alarm = 0;
		pressure_alarms.lowpressure_alarm  = 0;
	}

	return ret;
}

/**
 * @fn uint32_t sensor_log_GetOverPressureAlarm(void)
 * @brief Checks for overpressure alarm.
 *
 * @return overpressure_alarm
 * 			@arg 0 - Overpressure alarm has not occurred.
 * 			@arg 1 - Overpressure alarm has occurred.
 */
uint32_t sensor_log_GetOverPressureAlarm(void)
{
	/** Reads pressure */
	sensor_log_get_pressure();
	/** Compares with overpressure alarm value */
	__checkOverPressureAlarm();

	return pressure_alarms.overpressure_alarm;
}

/**
 * @fn uint32_t sensor_log_GetLowPressureAlarm(void)
 * @brief Checks for lowerpressure alarm.
 *
 * @return overpressure_alarm
 * 			@arg 0 - Lowerpressure alarm has not occurred.
 * 			@arg 1 - Lowerpressure alarm has occurred.
 */
uint32_t sensor_log_GetLowPressureAlarm( void )
{
	/** Reads pressure */
	sensor_log_get_pressure();
	/** Compares with lowerpressure alarm value */
	__checkLowPressureAlarm();

	return pressure_alarms.lowpressure_alarm;
}

uint32_t sensor_log_GetOperatingAlarm( void )
{
	return pressure_alarms.operating_alarm;
}

void sensor_log_SetOperatingAlarm( uint32_t _operating_alarm )
{
	pressure_alarms.operating_alarm = _operating_alarm;
}

char * sensor_log_GetPressureAvg( void )
{
	__readPressureLog();

	__ftoa(pressure_log.pressure.pressure_avg_dec, pressure_log.pressure.pressure_avg, 3);

	return pressure_log.pressure.pressure_avg;
}

char * sensor_log_GetPressureMin( void )
{
	__readPressureLog();

	__ftoa(pressure_log.pressure.pressure_min_dec, pressure_log.pressure.pressure_min, 3);

	return pressure_log.pressure.pressure_min;
}

char * sensor_log_GetPressureMax( void )
{
	__readPressureLog();

	__ftoa(pressure_log.pressure.pressure_max_dec, pressure_log.pressure.pressure_max, 3);

	return pressure_log.pressure.pressure_max;
}

/**
  * @brief Converts ADC average to pressure average and updates pressure_log structure
  * @retval returns a pointer to the member pressure_val of pressure_log structure
  */
char * sensor_log_get_pressure(void)
{
	float_t data_average = 0.0;

//	data_average = (AD_GetAverage() / 1000 + 0.2176) / 0.2943;//(AD_GetAverage()/1000 + 0.1657)/0.3252;
	data_average = AD_GetAverage()/1000;
	pressure_log.pressure.pressure_val_dec = data_average;

	__ftoa(pressure_log.pressure.pressure_val_dec, pressure_log.pressure.pressure_val, 3);

	return pressure_log.pressure.pressure_val;
}

/**
  * @brief
  * @retval None
  */
void sensor_log_Log(void)
{
	/** Reads pressure_log from FLASH, updates pressure_log register with new values
	 * and writes back to FLASH */
	__readPressureLog();
	sensor_log_get_pressure();
	__writePressureLog();
	/** Stores the current pressure value in float to pressure_page array and stores it in
	 * a dedicated sector in FLASH */
	__programPressureValuePage();
	__readPressureValuePage();
}

/**
  * @brief
  * @retval Returns the next address to write
  */
static uint32_t __nextAddress(unsigned int n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr = pressure_log.msg_wr_address + n;
	next_sector = (pressure_log.msg_wr_address + 0x1000) & 0xFFFFF000;

    if ((pressure_log.msg_wr_address & 0x00000FFF) == 0)
    {
    	sFLASH_EraseSector(pressure_log.msg_wr_address);
    }
    else if (next_addr > next_sector)
    {
    	sFLASH_EraseSector(next_sector);
    }

    addr = pressure_log.msg_wr_address;
    pressure_log.msg_wr_address += n;
    if (pressure_log.msg_wr_address > pressure_telegram_values_max_address)
    {
    	pressure_log.msg_wr_address = pressure_telegram_values_address;
    }

    if ((pressure_log.msg_wr_address == pressure_log.msg_rd_end_addr))
    {
    	pressure_log.msg_rd_end_addr += pressure_telegram_log_size;
    }
    pressure_log.msg_rd_ini_addr = pressure_log.msg_wr_address;
    pressure_log.msg_rd_address = pressure_log.msg_wr_address;

    return addr;
}

/**
  * @brief
  * @retval 0
  */
static uint8_t __programTelegramValueWord( void )
{
    uint32_t  size, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  *ptr;
	uint32_t  next_addr;
	pressure_telegram_values record_check;

    size = sizeof(pressure_telegram_value);
    ptr  = (uint8_t *) pressure_telegram_value.pressure_val;

    /** Gets the number of FLASH pages required to write the whole telegram
     *  sFLASH_SPI_PAGESIZE = 256 bytes and pressure_telegram_value size = 60 bytes */
    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 )
    {
    	pressure_telegram_log_size = (num_pages + 1) * sFLASH_SPI_PAGESIZE;
    }
    else
    {
    	pressure_telegram_log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    /** Gets the next address in memory to write the following chunk of data, since the
     * size is smaller than a page, this will be next page */
    next_addr = __nextAddress(pressure_telegram_log_size);
    LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
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
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> WARNING!! CHECK WRITE in Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)next_addr);
    }

    /**WARNING: In order to write to FLASH the sector needs to be empty first */
    uint32_t num = 0;
    if ( 1 == check_write ) {
    	do {
//    		sFLASH_EraseSector(next_addr&0xFFFFF000);
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> RECOVERED!! CHECK WRITE in Telegram Value Write address:0x%X\r\n", (int)Tick_Get( SECONDS ), (int)(next_addr&0xFFFFF000));
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }

    return ret;
}

char *sensor_log_message_value_pressure_sensor( void )
{
	float_t pressure;

//	pressure = (AD_GetAverage()/1000 + 0.2176)/0.2943;//(AD_GetAverage()/1000 + 0.1657)/0.3252;
	pressure = AD_GetAverage()/1000;

	__ftoa( pressure, message_value_pressure, 3 );

	if ( 0 == pressure ) {
		sensor_log_SetOperatingAlarm(1);
	}

	return message_value_pressure;
}

/**
  * @brief  Stores data from structure pressure_log to pressure_log_telegram_value to get it ready to send.
  * @retval None
  */
time_t first_value_time = 0;
uint32_t first_msg = 0xFFFFFFFF;
void sensor_log_pressure_store_telegram_value(void)
{
#define  CHECK_TAMPER	(HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ))
	if (pressure_log.init == 0xAAAA)
	{
		if ( 0xFFFFFFFF == first_msg )
		{
			first_msg        = pressure_log.msg_num;
			first_value_time = Tick_Get(SECONDS);
		}
		if ( ( pressure_log.msg_num < 1 ) || ( first_msg > pressure_log.msg_num ) )
		{
			first_msg        = pressure_log.msg_num;
			first_value_time = Tick_Get(SECONDS);
		}
		LOGLIVE(LEVEL_1, "LOGLIVE> %d RTC> first_msg:%d first_value_time:%d\r\n", (int)Tick_Get( SECONDS ), (int)first_msg, (int)first_value_time);
		pressure_log.msg_num++;

		strncpy(pressure_telegram_value.pressure_val, pressure_log.pressure.pressure_val, 7);
		strncpy(pressure_telegram_value.pressure_avg, pressure_log.pressure.pressure_avg, 7);
		strncpy(pressure_telegram_value.pressure_max, pressure_log.pressure.pressure_max, 7);
		strncpy(pressure_telegram_value.pressure_min, pressure_log.pressure.pressure_min, 7);

		if (1 == params_get_pulse_acc())
		{
			pressure_telegram_value.pulses_count[0]  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR22);
			pressure_telegram_value.pulses_count[1]  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR23);
			pressure_telegram_value.pulses_count[2]  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR24);
			pressure_telegram_value.pulses_count[0] += pulses_get_ch1p();//pulses_get_backflow_compensated_2p();
			pressure_telegram_value.pulses_count[1] += pulses_get_ch1d();
			pressure_telegram_value.pulses_count[2] += pulses_get_backflow_compensated_2p();//pulses_get_ch1p();
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR22, pressure_telegram_value.pulses_count[0] );
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR23, pressure_telegram_value.pulses_count[1] );
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR24, pressure_telegram_value.pulses_count[2] );
		}
		else
		{
			pressure_telegram_value.pulses_count[0] = pulses_get_ch1p();//pulses_get_backflow_compensated_2p();
			pressure_telegram_value.pulses_count[1] = pulses_get_ch1d();
			pressure_telegram_value.pulses_count[2] = pulses_get_backflow_compensated_2p();//pulses_get_ch1p();
		}
		pressure_telegram_value.msg_num++;

//		if ( ( 1 == params_pulses_on() ) /*&& ( 0 == params_get_pulse_acc() )*/ )
//		{
//			memcpy(pressure_telegram_value.created_sensor_value, (char *) (rtc_system_getCreatedSensorValueTimeFromFirstMeas(first_value_time, pressure_log.msg_num - first_msg)),
//					sizeof(pressure_telegram_value.created_sensor_value));
//			memcpy(pressure_telegram_value.created_sensor_value_date, (char * ) (rtc_system_getCreatedSensorValueDate()),
//					sizeof(pressure_telegram_value.created_sensor_value_date));
//		}
//		else
		{
			memcpy(pressure_telegram_value.created_sensor_value_date, (char * ) (rtc_system_getCreatedSensorValueDate()), \
					sizeof(pressure_telegram_value.created_sensor_value_date));
			memcpy(pressure_telegram_value.created_sensor_value, (char *) (rtc_system_getCreatedSensorValueTime()), \
					sizeof(pressure_telegram_value.created_sensor_value));
		}
		params_set_pulse_write(1);
		LOGLIVE(LEVEL_2, "\r\nLOGLIVE> %d SENSOR LOG> %s%s BUS read count: %d. Pulses values:%d %d %d. Pavg:%s. Pinst:%s. Msg.num:%d. While Sending:%d\r\n", (int)Tick_Get( SECONDS ),
				pressure_telegram_value.created_sensor_value_date,
				pressure_telegram_value.created_sensor_value,
				(int)params_maintenance_number_readings(),
				(int)pressure_telegram_value.pulses_count[0],(int)pressure_telegram_value.pulses_count[1], (int)pressure_telegram_value.pulses_count[2],
				pressure_telegram_value.pressure_avg, pressure_telegram_value.pressure_val,
				(int)pressure_log.msg_num,
				(int)pressure_telegram_value.pulse_while_sending);
		if (CHECK_TAMPER == 0)
		{
			leds_LED_On(LED_BLUE);
		}
		__programTelegramValueWord();
		HAL_Delay(10);//HAL_Delay(100);
		/**TOASK programWord is required because msg_num++?? it could be added before in sensor_log_Log*/
		__programWord();
		leds_LED_Off(LED_GREEN);
		if (1 == params_get_pulse_acc())
		{
			pressure_telegram_value.pulses_count[0] = 0;
			pressure_telegram_value.pulses_count[1] = 0;
			pressure_telegram_value.pulses_count[2] = 0;
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR22, pressure_telegram_value.pulses_count[0] );
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR23, pressure_telegram_value.pulses_count[1] );
			HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR24, pressure_telegram_value.pulses_count[2] );
		}
	}
}

static void __incReadPressureLog( void )
{
    uint32_t size = sizeof(pressure_telegram_value);

    /** Gets the number of FLASH pages required to write the whole telegram
     *  sFLASH_SPI_PAGESIZE = 256 bytes and pressure_telegram_value size = 60 bytes */
    uint32_t num_pages     = size / sFLASH_SPI_PAGESIZE;
    uint32_t num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 )
    {
    	pressure_telegram_log_size = (num_pages + 1) * sFLASH_SPI_PAGESIZE;
    }
    else
    {
    	pressure_telegram_log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }
    if ( pressure_log.msg_rd_ini_addr - pressure_telegram_log_size >= (uint32_t) pressure_telegram_values_address ) {
    	pressure_log.msg_rd_ini_addr -= pressure_telegram_log_size;
    } else {
    	if ( pressure_telegram_values_max_address - pressure_telegram_log_size > pressure_log.msg_rd_end_addr ) {
    		pressure_log.msg_rd_ini_addr = pressure_telegram_values_max_address - pressure_telegram_log_size + 1;
    	} else {
    		pressure_log.msg_rd_ini_addr = pressure_log.msg_rd_end_addr;
    	}
    }

    if ( ( pressure_log.msg_rd_ini_addr <= pressure_log.msg_rd_end_addr ) ) {
//        ret = 0;
    }

    pressure_log.msg_wr_address = pressure_log.msg_rd_ini_addr;
    pressure_log.msg_rd_address = pressure_log.msg_rd_ini_addr;
}

char * sensor_log_pressure_instant_pulses_telegram( uint32_t * measure_count, uint32_t pulses_on, uint32_t pressure_on )
{
	uint32_t i = 0, j = 0, str_len = 0, num_it = 0, pressure_coded_val = 0, pressure = 0, rd_ini_backup = 0, wr_backup = 0, rd_backup = 0;
	float_t  pressure_inst = 0.0;
	char     pressure_hex_str[9], pressure_coded[3][2], pressure_coded_acc[4][8], datetime[15], pressure_pulse_hex_str[3][9];
	char    *ptr_header_curr = pressure_telegram_string + 15;

	num_it = pressure_log.msg_num;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> msg num:%d \r\n",(int)Tick_Get( SECONDS ), (int)num_it);
	if ( 0 == num_it )
	{
		return NULL;
	}
	memset(pressure_hex_str, 0, sizeof(pressure_hex_str));
	rd_ini_backup                      = pressure_log.msg_rd_ini_addr;
	wr_backup                          = pressure_log.msg_wr_address;
	rd_backup                          = pressure_log.msg_rd_address;
	pressure_log.msg_rd_address_backup = pressure_log.msg_rd_ini_addr;//pressure_log.msg_rd_address;
	pressure_log.msg_num_backup        = pressure_log.msg_num;
	*measure_count = *measure_count + 1;
	__incReadPressureLog();
	pressure_log.msg_num--;
	if ( pressure_log.msg_num > 0 )
	{
//		num_it = pressure_log.msg_num;
	}
	else
	{
		asm("nop");
	}
	for ( i = 0; i < num_it; i++ )
	{
		sFLASH_ReadBuffer( (uint8_t *)&pressure_telegram_value,
				pressure_log.msg_rd_ini_addr,
				sizeof( pressure_telegram_value )
		);
		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> rd_ini:0x%X len:%d\r\n",(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)str_len);
		if ( ( str_len + sizeof(pressure_telegram_value) )
				>= ( MAX_PRESSURE_TELEGRAM_VALUES - 500 ) )//250
		{
			pressure_log.msg_rd_ini_addr = rd_ini_backup;
			pressure_log.msg_wr_address  = wr_backup;
			pressure_log.msg_rd_address  = rd_backup;
			pressure_log.msg_num         = pressure_log.msg_num + 1;
			*measure_count               = *measure_count - 1;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Frame End:%d Count:%d\r\n",(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)(*measure_count));
			break;
		}

		if (1 == params_get_pulse_acc())
		{
			pressure_inst = atof(pressure_telegram_value.pressure_avg);
		}
		else
		{
			pressure_inst = atof(pressure_telegram_value.pressure_val);
		}

		if ( pulses_on != 0 )
		{
			if ( 1 == pressure_on )
			{
				if (1 == params_get_pulse_acc())
				{
					pressure_coded_val       = 0;
					pressure_coded_acc[0][0] = 0;
					pressure_coded_acc[0][1] = 0;
					pressure_coded_acc[0][2] = 0;
					pressure_coded_acc[0][3] = 0;
					for ( j = 0; j < 3; j++ )
					{
						pressure = pressure_telegram_value.pulses_count[j];
						if ( 0 == j )
						{
							pressure_coded_val       = (uint32_t)(pressure_inst * 100);
							pressure_coded_acc[0][0] = (pressure_coded_val&0x0000FF00)>>8;
							pressure_coded_acc[0][1] = (pressure_coded_val&0x000000FF);
							pressure_coded_acc[0][2] = (pressure&0x0000FF00)>>8;
							pressure_coded_acc[0][3] = (pressure&0x000000FF);
							if ( 1 == pulses_on )
							{
								j = 3;
							}
						}
						else if ( 1 == j )
						{
							pressure_coded_acc[1][0] = (pressure&0x0000FF00)>>8;
							pressure_coded_acc[1][1] = (pressure&0x000000FF);
						}
						else if ( 2 == j )
						{
							pressure_coded_acc[2][0] = (pressure&0x0000FF00)>>8;
							pressure_coded_acc[2][1] = (pressure&0x000000FF);
						}
					}
				}
				else
				{
					pressure_coded_val   = 0;
					pressure_coded[0][1] = 0;
					pressure_coded[0][0] = 0;
					for ( j = 0; j < 3; j++ )
					{
						pressure = pressure_telegram_value.pulses_count[j] & 0x3F;
						if ( 0 == j )
						{
							pressure_coded_val   = (((uint32_t)(pressure_inst * 100) & 0x7FF) << 6)|pressure;
							pressure_coded[0][1] = pressure_coded_val&0xFF;
							pressure_coded[0][0] = (pressure_coded_val&0xFF00)>>8;
							if ( 1 == pulses_on )
							{
								j = 3;
							}
						}
						else if ( 1 == j )
						{
							pressure_coded_val = (pressure<<6)&0xFC0;
						}
						else if ( 2 == j )
						{
							pressure_coded_val   = ((pressure_coded_val&0xFC0)|pressure)<<4;
							pressure_coded[1][1] = pressure_coded_val&0xFF;
							pressure_coded[1][0] = (pressure_coded_val&0xFF00)>>8;
						}
					}
				}
			}
			else if ( 0 == pressure_on )
			{
				for ( j = 0; j < 3; j++ )
				{
					pressure = 0;
					pressure_coded_val   = (((uint32_t)(pressure_inst * 100) & 0x7FF) << 6)|pressure;
					pressure_coded[i][1] = pressure_coded_val&0xFF;
					pressure_coded[i][0] = (pressure_coded_val&0xFF00)>>8;
				}
			}
		}
		else if ( 0 == pulses_on )
		{
			if ( 1 == pressure_on)
			{
				pressure_coded_val = (uint32_t)(pressure_inst * 100);
			}
			else
			{
				pressure_coded_val = 0;
			}
			pressure_coded[0][1] = pressure_coded_val&0xFF;
			pressure_coded[0][0] = (pressure_coded_val&0xFF00)>>8;
		}

		if ( 0 == i )
		{
			snprintf( datetime,
					( sizeof( pressure_telegram_value.created_sensor_value_date ) + sizeof( pressure_telegram_value.created_sensor_value ) ),
					"%s%s|",
					pressure_telegram_value.created_sensor_value_date, pressure_telegram_value.created_sensor_value
			);
		}

		if ( 0 == pulses_on )
		{
			common_lib_string2hexString(pressure_coded[0], pressure_hex_str, 2);
			snprintf( ptr_header_curr,
					  9 * sizeof(char),
					  "%s",
					  pressure_hex_str);
			ptr_header_curr = ptr_header_curr + 4;
			str_len = strlen(pressure_telegram_string + sizeof(datetime));//First "sizeof(datetime)" number of bytes reserved for date.
		}
		else
		{
			if ( 1 == pulses_on )
			{
				if (1 == params_get_pulse_acc())
				{
					common_lib_string2hexString(pressure_coded_acc[0], pressure_pulse_hex_str[0], 4);
					snprintf( ptr_header_curr,
							9 * sizeof(char),
							"%s",
							pressure_pulse_hex_str[0]);
					ptr_header_curr = ptr_header_curr + 8;
				}
				else
				{
					common_lib_string2hexString(pressure_coded[0], pressure_pulse_hex_str[0], 2);
					volatile size_t n = 5 * sizeof(char);
					snprintf( ptr_header_curr,
							n,//5 * sizeof(char),
							"%s",
							pressure_pulse_hex_str[0]);
					ptr_header_curr = ptr_header_curr + 4;
				}
				str_len = strlen(pressure_telegram_string + sizeof(datetime));//First "sizeof(datetime)" number of bytes reserved for date.
			}
			else
			{
				if (1 == params_get_pulse_acc())
				{
					common_lib_string2hexString(pressure_coded_acc[0], pressure_pulse_hex_str[0], 4);
					common_lib_string2hexString(pressure_coded_acc[1], pressure_pulse_hex_str[1], 2);
					common_lib_string2hexString(pressure_coded_acc[2], pressure_pulse_hex_str[2], 2);
					pressure_pulse_hex_str[2][4] = '\0';
					for ( j = 0; j < 3; j++ )
					{
						volatile size_t n = 9 * sizeof(char);
						snprintf( ptr_header_curr,
								n,//9 * sizeof(char),
								"%s",
								pressure_pulse_hex_str[j]);
						if ( 0 == j )
						{
							ptr_header_curr = ptr_header_curr + 8;
						}
						else if ( 1 == j )
						{
							ptr_header_curr = ptr_header_curr + 4;
						}
						else if ( 2 == j )
						{
							ptr_header_curr = ptr_header_curr + 4;
						}
					}
				}
				else
				{
					common_lib_string2hexString(pressure_coded[0], pressure_pulse_hex_str[0], 2);
					common_lib_string2hexString(pressure_coded[1], pressure_pulse_hex_str[1], 2);
					pressure_pulse_hex_str[1][3] = '\0';
					for ( j = 0; j < 2; j++ )
					{
						snprintf( ptr_header_curr,
								9 * sizeof(char),
								"%s",
								pressure_pulse_hex_str[j]);
						if ( 0 == j )
						{
							ptr_header_curr = ptr_header_curr + 4;
						}
						else if ( 1 == j )
						{
							ptr_header_curr = ptr_header_curr + 3;
						}
					}
				}
				str_len = strlen(pressure_telegram_string + sizeof(datetime));//First "sizeof(datetime)" number of bytes reserved for date.
			}
		}

		*measure_count = *measure_count + 1;
		if ( pressure_log.msg_num > 0 )
		{
			rd_ini_backup = pressure_log.msg_rd_ini_addr;
			wr_backup     = pressure_log.msg_wr_address;
			rd_backup     = pressure_log.msg_rd_address;
			__incReadPressureLog();
			pressure_log.msg_num--;
			if ( 1 == pressure_telegram_value.pulse_while_sending )
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Pulse while sending:0x%X Count:%d\r\n",
						(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)(*measure_count));
				break;
			}
		}
		else
		{
			break;
		}
	}

	if ( pressure_log.msg_rd_ini_addr < (uint32_t)pressure_telegram_values_address )
	{
		asm("nop");
	}
	memcpy(pressure_telegram_string, datetime, 15);
	HAL_Delay(100);
	__programWord();

	if ( 0 == params_get_pulse_acc() )
	{
		pressure_telegram_value.pulses_count[0] = 0;
		pressure_telegram_value.pulses_count[1] = 0;
		pressure_telegram_value.pulses_count[2] = 0;
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR22, pressure_telegram_value.pulses_count[0] );
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR23, pressure_telegram_value.pulses_count[1] );
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR24, pressure_telegram_value.pulses_count[2] );
	}

	return pressure_telegram_string;
}

char * sensor_log_pressure_telegram( uint32_t *measure_count, uint32_t *sensor_id )
{
	uint32_t i, str_len = 0, num_it = 0, rd_ini_backup = 0, wr_backup = 0, rd_backup = 0;
	char date[10];

	num_it = pressure_log.msg_num;
	LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> msg num:%d \r\n",(int)Tick_Get( SECONDS ), (int)num_it);
	if ( 0 == num_it ) {
		return NULL;
	}
	memset(pressure_telegram_string, 0, sizeof(pressure_telegram_string));
	rd_ini_backup                      = pressure_log.msg_rd_ini_addr;
	wr_backup                          = pressure_log.msg_wr_address;
	rd_backup                          = pressure_log.msg_rd_address;
	pressure_log.msg_rd_address_backup = pressure_log.msg_rd_ini_addr;//pressure_log.msg_rd_address;
	pressure_log.msg_num_backup        = pressure_log.msg_num;
	*measure_count                     = *measure_count + 1;
	*sensor_id                         = pressure_log.sensor_id;
	__incReadPressureLog();
	pressure_log.msg_num--;
	if ( pressure_log.msg_num > 0 ) {
//		num_it = pressure_log.msg_num;
	}

	for ( i = 0; i < num_it; i++ ) {
		sFLASH_ReadBuffer( (uint8_t *)&pressure_telegram_value,
				pressure_log.msg_rd_ini_addr,
				sizeof( pressure_telegram_value ) );
		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> rd_ini:0x%X len:%d\r\n",(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)str_len);
		if ( (  str_len
				+ sizeof(pressure_telegram_value)
		     ) >= ( MAX_PRESSURE_TELEGRAM_VALUES - 250 ) ) {
			pressure_log.msg_rd_ini_addr = rd_ini_backup;
			pressure_log.msg_wr_address  = wr_backup;
			pressure_log.msg_rd_address  = rd_backup;
			pressure_log.msg_num         = pressure_log.msg_num + 1;
			*measure_count               = *measure_count - 1;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Frame End:%d Count:%d\r\n",(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)(*measure_count));
			break;
		}
		if ( 0xFF == pressure_telegram_value.pressure_val[0] ) {
			LOGLIVE(LEVEL_1, "Sensor Memory Error,time:%d",(int)Tick_Get( SECONDS ));
			continue;
		}
		if ( ( 0 == i )
		  || ( pressure_telegram_value.created_sensor_value_date[6] != date[6] )
		  || ( pressure_telegram_value.created_sensor_value_date[7] != date[7] )
		  ) {
			if ( 0 == i ) {
				snprintf(pressure_telegram_string,
						sizeof( pressure_telegram_value.created_sensor_value_date ) + 1,
						"%s|", pressure_telegram_value.created_sensor_value_date);
				str_len = strlen(pressure_telegram_string);
			} else if (( pressure_telegram_value.created_sensor_value_date[6] != date[6] )
				    || ( pressure_telegram_value.created_sensor_value_date[7] != date[7] )) {
				pressure_log.msg_rd_ini_addr = rd_ini_backup;
				pressure_log.msg_wr_address  = wr_backup;
				pressure_log.msg_rd_address  = rd_backup;
				pressure_log.msg_num         = pressure_log.msg_num + 1;
				*measure_count               = *measure_count - 1;
				LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Frame End:%d Count:%d\r\n",(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)(*measure_count));
				break;
			}
		}

		sprintf(pressure_telegram_string + str_len,
				"%s%.*s%.*s%.*s%.*s",
				pressure_telegram_value.created_sensor_value,
				(int)5,pressure_telegram_value.pressure_val,
				(int)5,pressure_telegram_value.pressure_avg,
				(int)5,pressure_telegram_value.pressure_min,
				(int)5,pressure_telegram_value.pressure_max);
		str_len = strlen(pressure_telegram_string);
		*measure_count = *measure_count + 1;
		if ( pressure_log.msg_num > 0 ) {
			rd_ini_backup = pressure_log.msg_rd_ini_addr;
			wr_backup     = pressure_log.msg_wr_address;
			rd_backup     = pressure_log.msg_rd_address;
			__incReadPressureLog();
			pressure_log.msg_num--;
		} else {
			break;
		}
		memcpy( date,
				pressure_telegram_value.created_sensor_value_date,
				sizeof(pressure_telegram_value.created_sensor_value_date) + 1 );
	}

	if (pressure_telegram_string[0] == '\0') {
		sprintf(pressure_telegram_string,
				"%s;%s;%s;%s|",
				sensor_log_message_value_pressure_sensor(),
				sensor_log_GetPressureAvg(),
				sensor_log_GetPressureMax(),
				sensor_log_GetPressureMin());
	}

	pressure_log.samples                   = 0;
	pressure_log.pressure.pressure_min_dec = pressure_log.pressure.pressure_val_dec;
	pressure_log.pressure.pressure_max_dec = pressure_log.pressure.pressure_val_dec;
	pressure_log.pressure.pressure_acc_dec = 0;

	HAL_Delay(10);//HAL_Delay(100);
	__programWord();

	return pressure_telegram_string;
}

char * sensor_log_pressure_params( uint32_t *measure_count )
{
	uint32_t i, num = 0, str_len = 0, num_it = 0, date_quarter_num;
	char date[10];
	char date_quarter[3];

	num_it = pressure_log.msg_num;

	memset(pressure_telegram_string, 0, sizeof(pressure_telegram_string));
	pressure_log.msg_rd_address_backup = pressure_log.msg_rd_ini_addr;//pressure_log.msg_rd_address;
	pressure_log.msg_num_backup        = pressure_log.msg_num;

	if ( ( num_it != 0 ) && ( num_it != 0xFFFFFFFF) && ( 0 == params_config_get_sensor_log() ) ) {
		*measure_count = *measure_count + 1;
		__incReadPressureLog();
		pressure_log.msg_num--;
		if ( pressure_log.msg_num > 0 ) {
			num_it = pressure_log.msg_num;
		}
		for ( i = 0; i < num_it; i++ ) {
			sFLASH_ReadBuffer( (uint8_t *)&pressure_telegram_value,
					pressure_log.msg_rd_ini_addr,
					sizeof( pressure_telegram_value ) );
			if ( (  str_len
					+ sizeof(pressure_telegram_value)
			) >= 1000  ) {
				break;
			}
			memset(date_quarter, 0, sizeof(date_quarter));
			date_quarter[0]  = pressure_telegram_value.created_sensor_value[2];
			date_quarter[1]  = pressure_telegram_value.created_sensor_value[3];
			date_quarter[2]  = '\0';
			if ( 0xFF == date_quarter[0]) {
				 continue;
			}
			date_quarter_num = atoi(date_quarter);
			if ( ( 0 == date_quarter_num) || ( 15 == date_quarter_num ) || ( 30 == date_quarter_num ) || ( 45 == date_quarter_num ) ) {
				if ( ( 0 == i )
						|| ( pressure_telegram_value.created_sensor_value_date[6] != date[6] )
						|| ( pressure_telegram_value.created_sensor_value_date[7] != date[7] )
				) {
					snprintf(pressure_telegram_string,
							sizeof( pressure_telegram_value.created_sensor_value_date ) + 1,
							"%s|", pressure_telegram_value.created_sensor_value_date);
					str_len = strlen(pressure_telegram_string);
				}
				sprintf(pressure_telegram_string + str_len,
						"P%d|%s;%s;%s;%s;%s;",
						(int)num,
						pressure_telegram_value.created_sensor_value,
						pressure_telegram_value.pressure_val,
						pressure_telegram_value.pressure_avg,
						pressure_telegram_value.pressure_min,
						pressure_telegram_value.pressure_max);
				str_len        = strlen(pressure_telegram_string);
				*measure_count = *measure_count + 1;
				num            = num + 1;
			}
			memcpy( date,
					pressure_telegram_value.created_sensor_value_date,
					sizeof(pressure_telegram_value.created_sensor_value_date) + 1 );
			if ( pressure_log.msg_num > 0 ) {
				__incReadPressureLog();
				pressure_log.msg_num--;
			} else {
				break;
			}
		}
	}

	if (pressure_telegram_string[0] == '\0') {
		char     param_pressure_val[7];
		char     param_pressure_avg[7];
		char     param_pressure_max[7];
		char     param_pressure_min[7];
		memcpy(param_pressure_val, sensor_log_get_pressure(),   7);
		memcpy(param_pressure_avg, sensor_log_GetPressureAvg(), 7);
		memcpy(param_pressure_min, sensor_log_GetPressureMin(), 7);
		memcpy(param_pressure_max, sensor_log_GetPressureMax(), 7);
		sprintf(pressure_telegram_string,
				"%s;%s;%s;%s",
				param_pressure_val,
				param_pressure_avg,
				param_pressure_min,
				param_pressure_max);
	}

	pressure_log.samples                   = 0;
	pressure_log.pressure.pressure_min_dec = pressure_log.pressure.pressure_val_dec;
	pressure_log.pressure.pressure_max_dec = pressure_log.pressure.pressure_val_dec;
	pressure_log.pressure.pressure_acc_dec = 0;

	HAL_Delay(10);
	__programWord();

	return pressure_telegram_string;
}

void sensor_log_pressure_reset_acc_values( void )
{
	pressure_log.samples                   = 0;
	pressure_log.pressure.pressure_min_dec = pressure_log.pressure.pressure_val_dec;
	pressure_log.pressure.pressure_max_dec = pressure_log.pressure.pressure_val_dec;
	pressure_log.pressure.pressure_acc_dec = 0;

	HAL_Delay(100);
	__programWord();
}

void sensor_log_pressure_reset_telegram_values( void )
{
#if 0
	if ( pressure_log.init == 0xAAAA ) {
		LOGLIVE(LEVEL_1, "LOGLIVE> %d sensor log> Reset telegram values\r\n", (int)Tick_Get( SECONDS ));
		pressure_log.msg_num               = 0;
		pressure_log.msg_num_backup        = 0;
		pressure_log.msg_wr_address        = pressure_telegram_values_address;
		pressure_log.msg_rd_address        = pressure_telegram_values_address;
		pressure_log.msg_rd_address_backup = pressure_telegram_values_address;
		pressure_log.msg_rd_ini_addr       = pressure_telegram_values_address;
		pressure_log.msg_rd_end_addr       = pressure_telegram_values_address;
		pressure_telegram_value.wr_addr    = pressure_telegram_values_address;
		pressure_telegram_value.rd_addr    = pressure_telegram_values_address;
		pressure_telegram_value.pulse_while_sending = 0;
		__programWord();
	}
#endif
	if ( pressure_log.init == 0xAAAA )
	{
		uint32_t next_sector = (pressure_log.msg_rd_address_backup + 0x1000) & 0xFFFFF000;
		if ( next_sector >= pressure_telegram_values_max_address )
		{
			next_sector = pressure_telegram_values_address;
		}
		pressure_log.msg_num               = 0;
		pressure_log.msg_num_backup        = 0;
		pressure_log.msg_wr_address        = next_sector;
		pressure_log.msg_rd_address        = next_sector;
		pressure_log.msg_rd_address_backup = next_sector;
		pressure_log.msg_rd_ini_addr       = next_sector;
		pressure_log.msg_rd_end_addr       = next_sector;
		pressure_telegram_value.wr_addr    = next_sector;
		pressure_telegram_value.rd_addr    = next_sector;
		pressure_telegram_value.pulse_while_sending = 0;
		__programWord();
		LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> pressure_log.msg_wr_address:0x%X.\r\n",
				(int)Tick_Get( SECONDS ), (int)pressure_log.msg_wr_address);
	}
}

uint8_t write_lock = 0;	/*!< lock key for */

/**
 * @fn void sensor_log_write_lock(uint8_t)
 * @brief
 *
 * @param _write_lock
 */
void sensor_log_write_lock( uint8_t _write_lock )
{
	write_lock = _write_lock;
}

/**
  * @brief Writes sensor data log in SPI FLASH.
  * @retval 1
  */
#define SENSOR_SERVER_RESPONSE   (2)
#define WAITING_SENSOR_MSG_RESP ((udp_protocol_get_status() == (UDP_PROT_WAITING_FOR_RESPONSE)) && (udp_get_server_resp_type() == (SENSOR_SERVER_RESPONSE)))
#define WAITING_SENSOR_MSG_SEND ((SEND_SENSOR) == message_queue_read()) && (udp_protocol_get_status() == (UDP_PROT_CONNECTED))
uint32_t sensor_log_pressure_register(void)
{
	static uint32_t tick_ini = 0, tick_end = 0;

	/* If pulses feature is enabled pointer can be modified while in use */
	if ( (1 == params_pulses_on())
	&& ( ( (1 == params_get_pulse_acc()) && (1 == sensor_log_get_write_pulse_acc()) ) || (0 == params_get_pulse_acc()) )
		)
	{
		if (0 == write_lock)
		{
			tick_ini = HAL_GetTick();
		}
		else if (0 == params_get_pulse_acc())
		{
			tick_end = HAL_GetTick();
			/* If there is a timer server or some communication delay, pointers are shifted so it will not write into
			 * the right position. */
			if (((tick_end - tick_ini) > SENSOR_INTERVAL_WRITE*1000) /*&& (udp_protocol_get_status() != UDP_PROT_WAITING_FOR_RESPONSE)*/)
			{
				/* While sending or ready to send, pulses are counted */
				if ((WAITING_SENSOR_MSG_RESP)//waiting sensor response.
				 || (WAITING_SENSOR_MSG_SEND)//waiting sensor send.
				  )
				{
					if (WAITING_SENSOR_MSG_RESP && (0 == udp_protocol_get_end_sensor_messages()))//Still sending frames...
					{
						write_lock = 1;
//						LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> WAIT WRITING MESSAGE!!!. rd_ini:0x%X msg_num:%d.\r\n",
//								(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)pressure_log.msg_num);
					}
					else if ( 0 == sensor_log_CheckDataInMem() )
					{
						tick_ini   = HAL_GetTick();
						write_lock = 0;
						LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Last Write in rd_ini:0x%X msg_num:%d.\r\n",
								(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)pressure_log.msg_num);
						rtc_system_reset_remaining_time();
					}
					else
					{
						write_lock = 1;
//						LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> WAIT WRITING MESSAGE!!!. rd_ini:0x%X msg_num:%d.\r\n",
//								(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)pressure_log.msg_num);
					}
				}
				else
				{
					tick_ini   = HAL_GetTick();
					write_lock = 0;
					rtc_system_reset_remaining_time();
//					rtc_system_GetServerTime();
					LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Last Write in rd_ini:0x%X msg_num:%d.\r\n",
							(int)Tick_Get( SECONDS ), (int)pressure_log.msg_rd_ini_addr, (int)pressure_log.msg_num);
				}
			}
		}
	}

	if ( (1 == params_pulses_on()) && (0 == sensor_log_get_write_pulse_acc()) && (1 == params_get_pulse_acc()) && (0 == write_lock) )
	{
		write_lock = 1;
		/** Processes pressure sensor measures and writes to FLASH */
		sensor_log_Log();
		pressure_telegram_value.pulses_count[0]  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR22);
		pressure_telegram_value.pulses_count[1]  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR23);
		pressure_telegram_value.pulses_count[2]  = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR24);
		pressure_telegram_value.pulses_count[0] += pulses_get_ch1p();
		pressure_telegram_value.pulses_count[1] += pulses_get_ch1d();
		pressure_telegram_value.pulses_count[2] += pulses_get_backflow_compensated_2p();
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR22, pressure_telegram_value.pulses_count[0] );
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR23, pressure_telegram_value.pulses_count[1] );
		HAL_RTCEx_BKUPWrite( &hrtc, RTC_BKP_DR24, pressure_telegram_value.pulses_count[2] );
		pulses_set_backflow_compensated_2p(0);
		pulses_set_ch1d(0);
		pulses_set_ch1p(0);
		LOGLIVE(LEVEL_2, "LOGLIVE> %d SENSOR LOG> Pulse acc.:Pulse 0:%d; Pulse 1:%d; Pulse 2:%d - Pavg:%s; Pinst:%s.\r\n",
				(int)Tick_Get( SECONDS ),
				(int)pressure_telegram_value.pulses_count[0],
				(int)pressure_telegram_value.pulses_count[1],
				(int)pressure_telegram_value.pulses_count[2],
				     pressure_log.pressure.pressure_avg,
				     pressure_log.pressure.pressure_val
				);
#if defined(UNE82326)
		if ((0 == shutdown_initTelitModule()) && (0 == une82326_get_start_comm()))
		{
#elif defined (MBUS)
		if ((0 == shutdown_initTelitModule()) && (0 == mbus_get_start_comm()))
		{
#endif
#ifdef MODBUS
			if ( (MODBUS_SESSION_END == modbus_get_end_session()) && ( 0 == modbus_sensors_get_get_data() ) )
			{
#endif
				Tick_Force_1Second_Task();/**Forces one second task execution in order to save battery.*/
				shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
#ifdef MODBUS
			}
#endif
		}
	}
	/* First write ever or Local Tool wants pressure measure. Also, time should be initialized or in process of taking it from server. */
	else if ( ( ( ( (0 == write_lock) || (1 == AD_GetRegisterADCData()) ) && (0 == params_get_pulse_acc()) )
			 || ( ( (0 == write_lock) && (1 == params_get_pulse_acc()) && (1 == sensor_log_get_write_pulse_acc()) ) ) )
	         && (1 == Tick_cloud_time_init())
	   )
	{
		write_lock = 1;
		AD_SetRegisterADCData(0);
		uint32_t pulse_before_2     = pulses_get_ch1d();
		uint32_t pulsech1p_before_2 = pulses_get_ch1p();
		uint32_t pulsech2p_before_2 = pulses_get_backflow_compensated_2p();
		/** Processes pressure sensor measures and writes to FLASH */
		sensor_log_Log();
		/* copy sensorg_lo struture to pressure_telegram_value*/
		uint32_t pulse_before     = pulses_get_ch1d();
		uint32_t pulsech1p_before = pulses_get_ch1p();
		uint32_t pulsech2p_before = pulses_get_backflow_compensated_2p();
		/** Copies sensor_log strurture to pressure_telegram_value structure and writes to FLASH */
		sensor_log_pressure_store_telegram_value();
		sensor_log_set_write_pulse_acc(0);
		if (pulses_get_backflow_compensated_2p() > pulsech2p_before)
		{
			pulses_set_backflow_compensated_2p(pulses_get_backflow_compensated_2p() - pulsech2p_before);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Pulse before:%d. Pulse before2:%d. Pulse now:%d\r\n",
										(int)Tick_Get( SECONDS ), (int)pulsech2p_before, (int)pulsech2p_before_2, (int)pulses_get_backflow_compensated_2p());
		}
		else
		{
			pulses_set_backflow_compensated_2p(0);
		}
		if (pulses_get_ch1d() > pulse_before)
		{
			pulses_set_ch1d(pulses_get_ch1d() - pulse_before);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Pulse before:%d. Pulse before2:%d. Pulse now:%d\r\n",
										(int)Tick_Get( SECONDS ), (int)pulse_before, (int)pulse_before_2, (int)pulses_get_ch1d());
		}
		else
		{
			pulses_set_ch1d(0);
		}
		if (pulses_get_ch1p() > pulsech1p_before)
		{
			pulses_set_ch1p(pulses_get_ch1p() - pulsech1p_before);
			LOGLIVE(LEVEL_1, "LOGLIVE> %d SENSOR LOG> Pulse before:%d. Pulse before2:%d. Pulse now:%d\r\n",
										(int)Tick_Get( SECONDS ), (int)pulsech1p_before, (int)pulsech1p_before_2 , (int)pulses_get_ch1p());
		}
		else
		{
			pulses_set_ch1p(0);
		}
#if defined(UNE82326)
		if ((0 == shutdown_initTelitModule()) && (0 == une82326_get_start_comm()))
		{
#elif defined (MBUS)
		if ((0 == shutdown_initTelitModule())
				&& ( (( 0 == mbus_get_start_comm() )    && ( 0 == generic_485_get_enable() ))
				||   (( 1 == generic_485_get_enable() ) && ( MODBUS_COMM_END == modbus_sensors_get_comm_state() )))
				   )
		{
#endif
#ifdef MODBUS
			if (MODBUS_SESSION_END == modbus_get_end_session())
			{
#endif
				Tick_Force_1Second_Task();/**Forces one second task execution in order to save battery.*/
				shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
#ifdef MODBUS
			}
#endif
		}
	}

	return 1;
}

#endif
