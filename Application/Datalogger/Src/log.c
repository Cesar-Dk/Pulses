/**
  ******************************************************************************
  * @file           log.c
  * @author 		Datakorum Development Team
  * @brief			Driver to handle the read and write sensor data from external
  *					FLASH memory
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
 * log.c
 *
 *  Created on: 17 ago. 2019
 *      Author: Sergio Millán López
 */
#include "log.h"
#include "datalogger_buffer.h"
#include "udp_protocol.h"
#include "une82326.h"
#include "une82326_protocol.h"
#include "ad.h"
#include "sensor_log.h"
#include "spi_flash.h"
#include "rtc.h"
#include "shutdown.h"
#include "params.h"
#include "mbus.h"
#include "mbus_protocol.h"
#include "pulses.h"
#include "modbus.h"
#include "modbus_sensors.h"
#include "modbus_sensors_log.h"
#include "generic_modbus.h"
#include "dlms_client.h"
#include "dlms_log.h"
#include "leds.h"

/**
 * @enum
 * @brief Structure that holds period time
 *
 */
typedef enum
{
    REALTIME = 0,	/**< REALTIME */
    MIN,         	/**< MIN */
    QUARTER,     	/**< QUARTER */
    HOUR,        	/**< HOUR */
    DAY,         	/**< DAY */
    WEEK,        	/**< WEEK */
    MONTH,       	/**< MONTH */
    YEAR,        	/**< YEAR */
    NEVER = 0xFF,	/**< NEVER */
} log_period;

/**
 * @struct __loggerData
 * @brief
 *
 */
typedef struct __loggerData
{
    uint32_t     time_in_seconds;
    uint32_t     switching_n;
    uint32_t     time_on_in_secs;
    uint32_t     time_connected;
    uint32_t     num_of_disconn;
    uint32_t     battery;
} dataLoggerData;

/**
 * @struct __log_params
 * @brief Parameter to log for the datalogger module
 *
 */
typedef struct __log_params
{
    uint8_t  enable;			/**< Datalogger module status
     	 	 	 	 	 	 	 	 	 @arg 0 - Disable
     	 	 	 	 	 	 	 	 	 @arg 1 - Enable */
    uint32_t wr_addr;			/**< Datalogger Write address */
    uint32_t rd_addr;			/**< Datalogger Read address */
    uint32_t rd_addr_backup;	/**< Datalogger read address backup */
    uint32_t rd_ini_addr;		/**< Datalogger read initial address */
    uint32_t rd_end_addr;		/**< Datalogger read end address */
    uint32_t rd_end_addr_backup;	/**< Datalogger read end address backup */
    uint8_t  sending;				/**< */
    uint8_t  write_period;			/**< meter reading time (param.config.rt)*/
    uint8_t  send_period;			/**< sending time (param.config.st)*/
    uint8_t  send_type;				/**< */

    log_mode log_mode;		/**< Datalogger log mode @ref log_mode
     	 	 	 	 	 	 	 	 @arg MEASURES_LOG
     	 	 	 	 	 	 	 	 @arg EVENTS_LOG */
} dataLoggerParams;

/**
 * @struct __dataloggerModule
 * @brief Structure that holds all datalogger module information.
 *
 */
static struct __dataloggerModule
{
    dataLoggerParams dl_params;

    dataLoggerData   data;

    uint32_t         sample_n;
    uint32_t         pages_n;  	/**< Number of FLASH pages the datalogger module takes */
} dataloggerModule;

static uint8_t          init_time = 0;
static dataLoggerRecord log_data_wr;
static dataLoggerRecordPulses log_data_wr_pulses;
static uint32_t         datalogger_meas_max_address;
static uint8_t          refresh = 0;
static uint8_t          mem_cycle = 0;
/**
 * @brief Flag to indicate that read and write pointer are correctly initialized
 */
static uint8_t          init_rd_wr_pointers = 0;
static uint32_t         log_size = 0;

extern struct params param;

extern char _logger_start_address[], _logger_meas_size[];

#define LOG_GET_ADDRESS (uint32_t)_logger_start_address
#define LOG_WR_ADDRESS  (uint32_t)_logger_start_address + 4
#define LOG_RD_ADDRESS  (uint32_t)_logger_start_address + 8

/**
 * @fn void __initAddresses(void)
 * @brief Initializes the pointers for datalogger
 *
 */
static void __initAddresses( void )
{
	/** Set all pointers to the origin logger_start_address location */
	dataloggerModule.dl_params.wr_addr     = (uint32_t) _logger_start_address;
    dataloggerModule.dl_params.rd_addr     = (uint32_t) _logger_start_address;
	dataloggerModule.dl_params.rd_ini_addr = (uint32_t) _logger_start_address;
	dataloggerModule.dl_params.rd_end_addr = (uint32_t) _logger_start_address;
    params_set_mem_cycle(0);
    /** Sets RTC_BKP_DR1 backup register to 1 indicating that pointers are initializes */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 1);
    /** Saves all pointer: wr_addr to DR2, rd_addr to DR3, rd_ini_addr to DR9 and rd_end_addr to DR10*/
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, dataloggerModule.dl_params.wr_addr);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, dataloggerModule.dl_params.rd_addr);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9, dataloggerModule.dl_params.rd_ini_addr);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10, dataloggerModule.dl_params.rd_end_addr);
}

#if 0
static uint8_t __eraseTelegramsBlock( void )
{
	uint32_t i;

    for (i = 0x2000; i < 0x7FFF; i = i + 0x2000) {
    	sFLASH_EraseBlock( i );
    	sFLASH_WaitBusy();
    }

    for ( i = 0x8000; i < 0xFFFF; i = i + 0x8000) {
    	sFLASH_EraseBlock( i );
    	sFLASH_WaitBusy();
    }

    for (i = 0x10000; i < 0x6FFFFF; i = i + 0x10000) {
    	sFLASH_EraseBlock( i );
    	sFLASH_WaitBusy();
    }

    return 0;
}
#endif

/**
 * @fn void __checkWrRdPointers(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __checkWrRdPointers( void )
{
    uint32_t dst;
    static uint32_t erased = 0;
    static uint32_t erased_statistics = 0;

    /** Reads whether the pointer are initialized in back up domain */
    dst = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR1);

    /** Updates pointers or delete the entire external FLASH */
    if (dst == 1)
    {
        dataloggerModule.dl_params.wr_addr     = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR2);
        dataloggerModule.dl_params.rd_addr     = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR3);
        dataloggerModule.dl_params.rd_ini_addr = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR9);
        dataloggerModule.dl_params.rd_end_addr = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR10);
        dataloggerModule.dl_params.rd_end_addr_backup = dataloggerModule.dl_params.rd_end_addr;

        /* Memory empty */
        if ((dataloggerModule.dl_params.wr_addr == 0xFFFFFFFF) || (dataloggerModule.dl_params.rd_addr == 0xFFFFFFFF))
        {
            __initAddresses();
        }

        if (dataloggerModule.dl_params.rd_ini_addr != dataloggerModule.dl_params.rd_end_addr)
        {
            init_rd_wr_pointers = 1;
        }
    }
    /* pointers are not updated then erase FLASH */
    else
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> Delete Memory Start.\r\n", (int)Tick_Get( SECONDS ));
    	sFLASH_EraseBulk();
//    	__eraseTelegramsBlock();
    	sFLASH_WaitBusy();
        __initAddresses();
        LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> Delete Memory End.\r\n", (int)Tick_Get( SECONDS ));
    }

    /* Delete memory parameter is enabled erase FLASH */
    if (1 == params_manteinance_delete_memory())
    {
    	if (0 == erased)
    	{
    		erased = 1;
    		LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> Delete Memory Start.\r\n", (int)Tick_Get( SECONDS ));
    		sFLASH_EraseBulk();
//    	    __eraseTelegramsBlock();
    		sFLASH_WaitBusy();
    		__initAddresses();
    		dlms_client_init();
    		LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> Delete Memory End.\r\n", (int)Tick_Get( SECONDS ));
    	}
    }
    else
    {
    	erased = 0;
    }

    /* Reset meter statistics parameter is enabled */
    if (1 == params_manteinance_reset_meters())
    {
    	if (0 == erased_statistics)
    	{
    		erased_statistics = 1;
    		params_manteinance_reset_statistics();
    	}
    }
    else
    {
    	erased_statistics = 0;
    }
}

/**
  * @brief Saves datalogger pointers to backup domain.
  * @retval None
  */
void DataLogger_SaveLogWrRdAdress( void )
{
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 1);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, dataloggerModule.dl_params.wr_addr);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, dataloggerModule.dl_params.rd_addr);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR9, dataloggerModule.dl_params.rd_ini_addr);
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR10, dataloggerModule.dl_params.rd_end_addr);
}

/**
 * @fn void __initSST26V(void)
 * @brief Low level external FLASH memory initialization
 */
static void __initSST26V( void )
{
	uint16_t block_protection_18[1];

	block_protection_18[0] = 0x00;

	sFLASH_ResetEnable();
	sFLASH_WaitBusy();
	sFLASH_Reset();
	sFLASH_WaitBusy();

	sFLASH_GlobalBlockProtectionUnlock();
	sFLASH_WaitBusy();

	sFLASH_WriteBlockProtection(&block_protection_18[0]);
	sFLASH_WaitBusy();

	sFLASH_ReadID();
	sFLASH_WaitBusy();
}

/**
 * @fn void DataLogger_Init(void)
 * @brief
 *
 * @pre
 * @post
 */
void DataLogger_Init(void)
{
	/** External FLASH memory initialization */
	__initSST26V();
	/** Clears datalogger Module */
    memset(&dataloggerModule, 0, sizeof(dataloggerModule));

    datalogger_meas_max_address = (uint32_t) _logger_start_address + (uint32_t) _logger_meas_size - 1;

    dataloggerModule.dl_params.enable  = 1;

    /** Initialize datalogger read and write pointers */
    __checkWrRdPointers();

    /* Calculates the number of FLASH pages of the datalogger module */
    dataloggerModule.pages_n           = (uint32_t) _logger_meas_size / SPI_FLASH_PAGE_SIZE;

    /* Writes send and write perido to the datalogger module */
    dataloggerModule.dl_params.send_period  = param.config.st;
    dataloggerModule.dl_params.write_period = param.config.rt;
//    dataloggerModule.dl_params.send_type    = param.log.send_type;

    dataloggerModule.dl_params.sending  = READING_FRAMES;
    dataloggerModule.dl_params.log_mode = MEASURES_LOG;
    dataloggerModule.sample_n           = 0;
}

void DataLogger_ResetPointers( void )
{
	dataloggerModule.dl_params.wr_addr     = (uint32_t) _logger_start_address;
	dataloggerModule.dl_params.rd_addr     = (uint32_t) _logger_start_address;
	dataloggerModule.dl_params.rd_ini_addr = (uint32_t) _logger_start_address;
	dataloggerModule.dl_params.rd_end_addr = (uint32_t) _logger_start_address;
	DataLogger_SaveLogWrRdAdress();
}

uint32_t datalogger_send_disable = 0;
void Datalogger_Set_Send_Disable( uint32_t _disable )
{
	datalogger_send_disable = _disable;
}

uint32_t Datalogger_Get_Send_Disable( void )
{
	return datalogger_send_disable;
}

void Datalogger_NextFramesMemoryBlock( void )
{
	dataloggerModule.dl_params.wr_addr     = dataloggerModule.dl_params.rd_end_addr_backup;
	dataloggerModule.dl_params.rd_addr     = dataloggerModule.dl_params.rd_end_addr_backup;
	dataloggerModule.dl_params.rd_ini_addr = dataloggerModule.dl_params.rd_end_addr_backup;
	DataLogger_SaveLogWrRdAdress();
}

uint8_t Datalogger_CheckLIFOPointers( void )
{
	uint8_t ret = ( dataloggerModule.dl_params.rd_ini_addr != dataloggerModule.dl_params.rd_end_addr )?1:0;

	return ret;
}

uint8_t Datalogger_CheckPointers( void )
{
	uint8_t ret = ( dataloggerModule.dl_params.wr_addr != dataloggerModule.dl_params.rd_addr )?1:0;

	return ret;
}

static void __increaseNumOfSamplesByOne(void)
{
    dataloggerModule.sample_n++;
}

static void __getMeasures(void)
{
    __increaseNumOfSamplesByOne();
}

void __getTime( uint32_t *time )
{
    *time = Tick_Get(SECONDS);
}

void __initTime( uint32_t *time )
{
    *time = Tick_Get(SECONDS);
}

void __measuresRegister(uint32_t *before)
{
	uint32_t time;

    __getTime(&time);

    if ( 1 == DataLogger_GetRefresh() ) {
        DataLogger_Register(MEASURES_LOG);
        return;
    }

#if defined (UNE82326)
    if ( 1 == une82326_get_write_record() ) {
    	*before = time;
    	DataLogger_Register(MEASURES_LOG);
    	une82326_set_write_record(0);
    }
	if ( ( 0 == shutdown_initTelitModule() )
			) {
		if ( ( 1 == une82326_get_last_device() ) && ( 1 == une82326_get_end_comm() ) ) {
			if ( MODBUS_SESSION_END == modbus_get_end_session() ) {
				shutdown_set( 1, params_config_read_time() );
			}
    	}
    }
#elif defined(MBUS)
    if ( 1 == mbus_get_write_record() ) {
    	*before = time;
    	DataLogger_Register(MEASURES_LOG);
    	mbus_set_write_record(0);
    }
#endif
//    if ( 1 == params_pulses_on() ) {
//    	if ( 1 == pulses_get_write_record() ) {
//    		DataLogger_RegisterPulses();
//    		pulses_set_write_record(0);
//    		pulses_set_pulse_rx( 0 );
////    		udp_protocol_set_send_pending(1);
//    	}
//    }
}

void __measuresSend(uint32_t *before)
{
	uint32_t time;

    __getTime(&time);

    if ( ( 0 == dataloggerModule.dl_params.sending ) && ( dataloggerModule.dl_params.rd_ini_addr != dataloggerModule.dl_params.rd_end_addr ) ) {
		*before = time;
		dataloggerModule.dl_params.sending = 2;
	}

    if ( 2 == dataloggerModule.dl_params.sending ) {
    	dataloggerModule.dl_params.sending = 1;
        datalog_buffer_write();
    }
}

#ifdef UNE82326
uint32_t __framesSend( void )
{
	uint32_t msgs_size = 0, ret = 0;

	if ( ( READING_FRAMES                         == dataloggerModule.dl_params.sending     )
	  && ( dataloggerModule.dl_params.rd_ini_addr != dataloggerModule.dl_params.rd_end_addr )
	  && ( 0                                      == datalog_check_frame_tx_pointers() )    ) {
		msgs_size = datalog_buffer_read_device_frames(rest_get_modbus_sensor_tx());//rest_get_str_tx()
		if (msgs_size > 0) {
			dataloggerModule.dl_params.sending = PARSING_NEW_FRAME;
		} else {
			ret = 1;
		}
	}

	if ( PARSING_NEW_FRAME == dataloggerModule.dl_params.sending ) {
		if ( 1 == datalog_check_frame_tx_pointers() )  {
			dataloggerModule.dl_params.sending = WAITING_PARSE_NEXT_FRAME;
			if ( datalog_buffer_write_device_frames(rest_get_modbus_sensor_tx()) != 0 ) {//rest_get_str_tx()
				ret = 1;
			}
		}
	}

	return ret;
}
#endif

uint32_t Datalogger_GetLogSize( void )
{
	return log_size;
}

static unsigned int __nextAddress(unsigned int n)
{
	uint32_t addr;
	uint32_t next_addr, next_sector;

	next_addr   = dataloggerModule.dl_params.wr_addr + n;
	next_sector = ( dataloggerModule.dl_params.wr_addr + 0x1000 ) & 0xFFFFF000;

    if ( ( dataloggerModule.dl_params.wr_addr & 0x00000FFF ) == 0 ) {
    	sFLASH_EraseSector(dataloggerModule.dl_params.wr_addr );
    } else if (  next_addr > next_sector ) {
    	sFLASH_EraseSector( next_sector );
    }

    addr = dataloggerModule.dl_params.wr_addr;
    dataloggerModule.dl_params.wr_addr += n;
    if ( dataloggerModule.dl_params.wr_addr > datalogger_meas_max_address ) {
        dataloggerModule.dl_params.wr_addr = (uint32_t) _logger_start_address;
        mem_cycle = 1;
        params_set_mem_cycle(1);
    }

    if ( ( dataloggerModule.dl_params.wr_addr == dataloggerModule.dl_params.rd_end_addr )
      && ( 1 == params_get_mem_cycle() ) ) {
    	dataloggerModule.dl_params.rd_end_addr += log_size;
    }
    dataloggerModule.dl_params.rd_ini_addr = dataloggerModule.dl_params.wr_addr;
    dataloggerModule.dl_params.rd_addr     = dataloggerModule.dl_params.wr_addr;

    return addr;
}

static uint8_t __programWord(dataLoggerRecord log_data_wr)
{
    uint32_t  size, num_pages, num_pages_mod, check_write, ret = 0;
	uint8_t  *ptr;
	dataLoggerRecord record_check;

    size = sizeof(log_data_wr);
    ptr  = (uint8_t *) &log_data_wr;

    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 ) {
    	log_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    uint32_t addr = __nextAddress( log_size );
    sFLASH_WriteBuffer(ptr, addr, size);
    HAL_Delay(100);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, addr, sizeof( dataLoggerRecord ) );
    LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> MBUS Telegram Value Write address: 0x%X.--------------------\r\n", (int)Tick_Get( SECONDS ), (int)addr);

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, sizeof( dataLoggerRecord ))) {
    	__NOP();
    	check_write = 0;
    } else {
    	__NOP();
    	check_write = 1;
    }
    uint32_t num = 0;
//    if ( record_check.token != CHECK_TOKEN ) {
    if ( 1 == check_write ) {
    	do {
//    		sFLASH_EraseSector( addr & 0xFFFFF000 );
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, addr, sizeof( dataLoggerRecord ) );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, sizeof( dataLoggerRecord ))) {
    	    	__NOP();
    	    	check_write = 0;
    	    	LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> MBUS ERROR RECOVERED:Telegram Value Write address: 0x%X\r\n", (int)Tick_Get( SECONDS ), (int)addr);
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    }
//    	} while ((record_check.token != CHECK_TOKEN) && (num<5));
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }

    if ( record_check.token != CHECK_TOKEN ) {
    	asm("nop");
    }
    if ( num >= 5 ) {
    	asm("nop");
//    	printf("MBus Memory Error 0x%X,time:%d",(int)addr,(int)Tick_Get( SECONDS ));
    }
    return ret;
}

static uint8_t __programWordPulses(dataLoggerRecordPulses log_data_wr)
{
    uint32_t  size, num_pages, num_pages_mod, ret = 0;
	uint8_t  *ptr;

    size = sizeof(log_data_wr);
    ptr  = (uint8_t *) &log_data_wr;

    num_pages     = size / sFLASH_SPI_PAGESIZE;
    num_pages_mod = size % sFLASH_SPI_PAGESIZE;
    if ( num_pages_mod != 0 ) {
    	log_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }

    sFLASH_WriteBuffer(ptr, __nextAddress( log_size ), size);

    return ret;
}

#if defined(UNE82326)
static uint8_t __writeToDatalogger(void)
{
	uint32_t frame_type = une82326_get_frame_type();

    if (!dataloggerModule.sample_n) {
        return 0;
    }

    log_data_wr.time_in_seconds = Tick_Get( SECONDS );
    memcpy( log_data_wr.device_id , (char *)( une82326_s_field() )                        , sizeof( log_data_wr.device_id ) );

    if ( A_PLUS_FRAME == frame_type ) {
    	une82326_build_a_plus_information_message();
    	memcpy( log_data_wr.frame , (char *)( une82326_get_a_plus_information_message() ) , sizeof( log_data_wr.frame ) );
    	memcpy( log_data_wr.crc   , (char *)( une82326_crc_field() )                      , sizeof( log_data_wr.crc   ) );
    	memcpy( log_data_wr.date  , (char *)( rtc_system_getCreatedTimeFileName() )       , sizeof( log_data_wr.date  ) );
    } else if (  ( WATER_METER_GATHERING_VALUES == frame_type ) || ( WATER_METER_FRAME == frame_type ) ) {
    	memcpy( log_data_wr.crc   , (char *)( une82326_get_crc_field_value() )            , sizeof( log_data_wr.crc   ) );
//    	memcpy( log_data_wr.date  , (char *)( rtc_system_getCreatedMeterValueDate() )     , sizeof( log_data_wr.date  ) );
    	une82326_add_r_field_time_value();
    	memcpy( log_data_wr.frame , (char *)( une82326_get_r_field_values() )             , sizeof( log_data_wr.frame ) );
    	memset( log_data_wr.date  , 0                                                     , sizeof( log_data_wr.date  ) );
    	memcpy( log_data_wr.date  , (char *)( rtc_system_getCreatedTimeFileName() )       , 8 * sizeof( char  ) );
    	frame_type = WATER_METER_FRAME;
    }

    log_data_wr.frame_size    = strlen(log_data_wr.frame);
    log_data_wr.send_type     = frame_type;
    log_data_wr.token         = CHECK_TOKEN;

    __programWord(log_data_wr);

    dataloggerModule.sample_n = 0;
    LOGLIVE(LEVEL_2, "LOGLIVE> %d LOG> UNE Telegram: %s.\r\n",(int)rtc_system_GetServerTime(), log_data_wr.frame);
    return 1;
}

uint32_t Datalogger_get_frame_size( void )
{
	return log_data_wr.frame_size;
}

uint8_t Datalogger_writeToDatalogger(void)
{
	uint32_t frame_type = une82326_get_frame_type();

	__getMeasures();
    if (!dataloggerModule.sample_n) {
        return 0;
    }

    log_data_wr.time_in_seconds = Tick_Get( SECONDS );
    memcpy( log_data_wr.device_id , (char *)( une82326_s_field() )                        , sizeof( log_data_wr.device_id ) );

    if ( A_PLUS_FRAME == frame_type ) {
    	une82326_build_a_plus_information_message();
    	memcpy( log_data_wr.frame , (char *)( une82326_get_a_plus_information_message() ) , sizeof( log_data_wr.frame ) );
    	memcpy( log_data_wr.crc   , (char *)( une82326_crc_field() )                      , sizeof( log_data_wr.crc   ) );
    	memcpy( log_data_wr.date  , (char *)( rtc_system_getCreatedTimeFileName() )       , sizeof( log_data_wr.date  ) );
    } else if (  ( WATER_METER_GATHERING_VALUES == frame_type ) || ( WATER_METER_FRAME == frame_type ) ) {
    	memcpy( log_data_wr.crc   , (char *)( une82326_get_crc_field_value() )            , sizeof( log_data_wr.crc   ) );
//    	memcpy( log_data_wr.date  , (char *)( rtc_system_getCreatedMeterValueDate() )     , sizeof( log_data_wr.date  ) );
    	une82326_add_r_field_time_value();
    	memcpy( log_data_wr.frame , (char *)( une82326_get_r_field_values() )             , sizeof( log_data_wr.frame ) );
    	memset( log_data_wr.date  , 0                                                     , sizeof( log_data_wr.date  ) );
    	memcpy( log_data_wr.date  , (char *)( rtc_system_getCreatedTimeFileName() )       , 8 * sizeof( char  ) );
    	frame_type = WATER_METER_FRAME;
    }

    log_data_wr.frame_size    = strlen(log_data_wr.frame);
    log_data_wr.send_type     = frame_type;
    log_data_wr.token         = CHECK_TOKEN;

    __programWord(log_data_wr);

    dataloggerModule.sample_n = 0;

    return 1;
}

char datalogger_info[100];
char *Datalogger_Get_Info( void )
{
	memset(datalogger_info,0,sizeof(datalogger_info));
	if ( 0 == params_config_get_disable_meter() ) {
		if ( 0 == log_size )
		{
			log_size = 1;
		}
		sprintf(datalogger_info,
				"%d,0x%X,0x%X",
				(int)(((dataloggerModule.dl_params.wr_addr - dataloggerModule.dl_params.rd_end_addr)/ log_size ) + datalog_buffer_n()),
				(int)dataloggerModule.dl_params.rd_end_addr,
				(int)dataloggerModule.dl_params.wr_addr
		);
	}

	return datalogger_info;
}
#elif defined(MBUS)
#if 0
static void __buildMFrameTelegram( char *crc )
{
	memset( log_data_wr.telegram_1, 0, sizeof(log_data_wr.telegram_1) );
	mbus_build_typeM_telegram(log_data_wr.telegram_1, &log_data_wr.num_chars);
	memcpy( log_data_wr.date,       (char *)( rtc_system_getCreatedMeterValueDate() ), sizeof( log_data_wr.date)        );
	memcpy( log_data_wr.crc,        (char *) crc,                                      sizeof( log_data_wr.crc )        );
	log_data_wr.telegram_size    = strlen(log_data_wr.telegram_1);
	log_data_wr.frame_type       = mbus_get_frame_type();
	log_data_wr.token            = CHECK_TOKEN;
}

static void __buildFFrameTelegram( char *crc )
{
	memcpy( log_data_wr.date,       (char *)( rtc_system_getCreatedTimeFileName() ), sizeof( log_data_wr.date) );
	memcpy( log_data_wr.telegram_1, (char *)( mbus_get_telegram_one() ),             sizeof( log_data_wr.telegram_1 ) );
	memcpy( log_data_wr.crc,        (char *) crc,                                    sizeof( log_data_wr.crc )        );
	log_data_wr.telegram_size    = strlen(log_data_wr.telegram_1);
	log_data_wr.frame_type       = mbus_get_frame_type();
	log_data_wr.token            = CHECK_TOKEN;
}
#endif
//function to convert ascii char[] to hex-string (char[])
static inline void _string2hexString(char* input, char* output, uint32_t len)
{
    int loop;
    int i,j;

    i=0;
    loop=0;

    for (j=0; j<2*len; j++)
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
    }
    //insert NULL at the end of the output string
//    output[i++] = '\0';
}

static void __buildRawFrameTelegram( void )
{
	char data[2048];

	memset(data, 0 , sizeof(data));

	mbus_set_frame_type(F_FRAME);
//	_string2hexString((char *)mbus_get_raw_telegram(), data, mbus_get_raw_telegram_length());

	memset( log_data_wr.telegram_1, 0,                                                                    sizeof(log_data_wr.telegram_1) );
	memcpy( log_data_wr.telegram_1, (char *)mbus_get_raw_telegram(),                 					  mbus_get_raw_telegram_length() );
//	memcpy( log_data_wr.date,       (char *)( rtc_system_getCreatedTimeFileName() ),                      sizeof( log_data_wr.date)      );
	memcpy( log_data_wr.date,       (char *)( rtc_system_getCreatedMeterValueDate() ),                    8      );
	memcpy( log_data_wr.date+8,     (char *)( rtc_system_getCreatedMeterValueTime() ),                    6      );
	if ((MBUS_METER_TYPE == params_get_uart_meter_type()) || (MBUS_SLAVES_METER_TYPE == params_get_uart_meter_type()))
	{
		memcpy( log_data_wr.crc,        (char *) &log_data_wr.telegram_1[mbus_get_raw_telegram_length() - 2], sizeof( char )                 );
	}
	else
	{
		memcpy( log_data_wr.crc,        (char *) &log_data_wr.telegram_1[mbus_get_raw_telegram_length() - 3], 2 * sizeof( char )                 );
	}

//	log_data_wr.telegram_1[ mbus_get_raw_telegram_length() ] = '|';
	log_data_wr.telegram_size    								 = mbus_get_raw_telegram_length();//strlen(log_data_wr.telegram_1);
	log_data_wr.frame_type       								 = mbus_get_frame_type();
	log_data_wr.token            								 = CHECK_TOKEN;

	if ( 0 == modbus_get_local_tool_started() )
	{
		memset((char *)mbus_get_raw_telegram(),  0, mbus_get_raw_telegram_length());
	}
//	char data[2048];
//
//	memset(data, 0 , sizeof(data));
//
//	mbus_set_frame_type(F_FRAME);
//	_string2hexString((char *)mbus_get_raw_telegram(), data, mbus_get_raw_telegram_length());
//
//	memset( log_data_wr.telegram_1, 0,                                                                        sizeof(log_data_wr.telegram_1)     );
//	memcpy( log_data_wr.telegram_1, data,                 							                          2 * mbus_get_raw_telegram_length() );
//	memcpy( log_data_wr.date,       (char *)( rtc_system_getCreatedTimeFileName() ),                          sizeof( log_data_wr.date)          );
//	memcpy( log_data_wr.crc,        (char *) &log_data_wr.telegram_1[2 * mbus_get_raw_telegram_length() - 4], 2 * sizeof( char )                 );
//
//	log_data_wr.telegram_1[ 2 * mbus_get_raw_telegram_length() ] = '|';
//	log_data_wr.telegram_size    								 = strlen(log_data_wr.telegram_1);
//	log_data_wr.frame_type       								 = mbus_get_frame_type();
//	log_data_wr.token            								 = CHECK_TOKEN;
//	LOGLIVE(LEVEL_2, "LOGLIVE> %d LOG> M-BUS Telegram: %s.\r\n",(int) rtc_system_GetServerTime(), log_data_wr.telegram_1);
}

static uint8_t __writeToDatalogger(void)
{
//	char crc[5];

    if (!dataloggerModule.sample_n) {
        return 0;
    }

    log_data_wr.time_in_seconds  = Tick_Get( SECONDS );

	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

    if ( TELEGRAM_MODE_RAW == params_telegram_mode() ) {
		memcpy( log_data_wr.device_id,  (char *)( Telit_dev_identifier() ), sizeof( log_data_wr.device_id )  );
    	__buildRawFrameTelegram();
    } else if ( TELEGRAM_MODE_DTK == params_telegram_mode() ) {
//    	if ( 2 == mbus_get_manufacturer() ) {
//    		mbus_build_hydrus_telegram();
//    		memcpy(crc, (char *)( mbus_get_hydrus_crc() ), sizeof( crc ) );
//    		memcpy( log_data_wr.device_id,  (char *)( mbus_get_hydrus_serial_num() ), sizeof( log_data_wr.device_id )  );//Telit_dev_identifier() )
//    	} else if ( 1 == mbus_get_manufacturer() ) {
//    		mbus_build_badger_telegram();
//    		memcpy(crc, (char *)( mbus_get_badger_crc() ), sizeof( crc ) );
//    		memcpy( log_data_wr.device_id,  (char *)( mbus_get_badger_serial_num() ), sizeof( log_data_wr.device_id )  );//Telit_dev_identifier() )
//    	}
//    	else {
//    		mbus_build_falconpr6_telegram();
//    		memcpy(crc, (char *)( mbus_get_pr6_crc() ), sizeof( crc ) );
//    		memcpy( log_data_wr.device_id,  (char *)( mbus_get_pr6_serial_num() ), sizeof( log_data_wr.device_id )  );//Telit_dev_identifier() )
//    	}
//    	if ( M_FRAME == mbus_get_frame_type() ) {
//    		__buildMFrameTelegram(crc);
//    	} else if ( F_FRAME == mbus_get_frame_type() ) {
//    		__buildFFrameTelegram(crc);
//    	}
    }

    __programWord(log_data_wr);

    dataloggerModule.sample_n = 0;

    return 1;
}

char datalogger_info[100];
char *Datalogger_Get_Info( void )
{
	memset(datalogger_info,0,sizeof(datalogger_info));
	if ( 0 == params_config_get_disable_meter() ) {
		if (0 == log_size)
		{
			sprintf(datalogger_info,
					"%d,0x%X,0x%X",
					(int)(0),
					(int)0,
					(int)0
			);
		}
		else
		{
			sprintf(datalogger_info,
					"%d,0x%X,0x%X",
					//				(int)log_data_wr.telegram_size,
					(int)(((dataloggerModule.dl_params.wr_addr - dataloggerModule.dl_params.rd_end_addr)/ log_size ) + datalog_buffer_n()),//(int)datalog_buffer_n(),
					(int)dataloggerModule.dl_params.rd_end_addr,//(int)dataloggerModule.dl_params.rd_addr_backup,
					(int)dataloggerModule.dl_params.wr_addr
			);
		}
	}

	return datalogger_info;
}

uint8_t Datalogger_writeToDatalogger(void)
{
//	char crc[5];

//    if (!dataloggerModule.sample_n) {
//        return 0;
//    }

    log_data_wr.time_in_seconds  = Tick_Get( SECONDS );

    if ( TELEGRAM_MODE_RAW == params_telegram_mode() ) {
		memcpy( log_data_wr.device_id,  (char *)( Telit_dev_identifier() ), sizeof( log_data_wr.device_id )  );
    	__buildRawFrameTelegram();
    } else if ( TELEGRAM_MODE_DTK == params_telegram_mode() ) {
//    	if ( 2 == mbus_get_manufacturer() ) {
//    		mbus_build_hydrus_telegram();
//    		memcpy(crc, (char *)( mbus_get_hydrus_crc() ), sizeof( crc ) );
//    		memcpy( log_data_wr.device_id,  (char *)( mbus_get_hydrus_serial_num() ), sizeof( log_data_wr.device_id )  );//Telit_dev_identifier() )
//    	} else if ( 1 == mbus_get_manufacturer() ) {
//    		mbus_build_badger_telegram();
//    		memcpy(crc, (char *)( mbus_get_badger_crc() ), sizeof( crc ) );
//    		memcpy( log_data_wr.device_id,  (char *)( mbus_get_badger_serial_num() ), sizeof( log_data_wr.device_id )  );//Telit_dev_identifier() )
//    	}
//    	else {
//    		mbus_build_falconpr6_telegram();
//    		memcpy(crc, (char *)( mbus_get_pr6_crc() ), sizeof( crc ) );
//    		memcpy( log_data_wr.device_id,  (char *)( mbus_get_pr6_serial_num() ), sizeof( log_data_wr.device_id )  );//Telit_dev_identifier() )
//    	}
//    	if ( M_FRAME == mbus_get_frame_type() ) {
//    		__buildMFrameTelegram(crc);
//    	} else if ( F_FRAME == mbus_get_frame_type() ) {
//    		__buildFFrameTelegram(crc);
//    	}
    }

    __programWord(log_data_wr);

    dataloggerModule.sample_n = 0;

    return 1;
}
#endif

static void __writeMeasLog(uint32_t *before)
{
    __getMeasures();

    if (!dataloggerModule.dl_params.enable) {
        dataloggerModule.dl_params.wr_addr = (uint32_t) _logger_start_address;
        *before = 0;
        return;
    }
    if (!Tick_Get(SECONDS)) {
        return;
    }

    if (dataloggerModule.dl_params.log_mode == MEASURES_LOG) {
        __measuresRegister(before);
    }
}

void DataLogger_Register(log_mode log_mode)
{
	if (log_mode == MEASURES_LOG) {
        dataloggerModule.dl_params.log_mode = MEASURES_LOG;
		if ( ( 0 == shutdown_initTelitModule() )
				) {
#if defined (UNE82326)
			if ( ( 1 == __writeToDatalogger() ) && ( 1 == une82326_get_last_device() ) ) {
				if ( MODBUS_SESSION_END == modbus_get_end_session() ) {
					shutdown_set( 1, params_config_read_time() );
				}
        	}
        }
#elif defined (MBUS)
		if ( ( 1 == __writeToDatalogger() ) && ( 1 == mbus_get_last_device() ) ) {
			if (( MODBUS_SESSION_END == modbus_get_end_session() )
				 && ( 0 == modbus_sensors_get_get_data() )
				 && ( dlms_client_get_dlms_comm_state() != DLMS_COMM_STARTED )
				 && ( 1 == CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue()) )
			 )
				{
					shutdown_set( 1, params_config_read_time() );
				}
    		}
		}
#endif
		else {
			__writeToDatalogger();
		}
    }
}

static uint8_t __writeToDataloggerPulses(void)
{
	log_data_wr_pulses.time_in_seconds  = Tick_Get( SECONDS );

    memset( log_data_wr_pulses.telegram_1, 0, sizeof( log_data_wr_pulses.telegram_1 ) );
    pulses_build_typeM_telegram( log_data_wr_pulses.telegram_1, &log_data_wr_pulses.num_dig );

	memcpy( log_data_wr_pulses.date,       (char *)( rtc_system_getCreatedMeterValueDate() ), sizeof( log_data_wr_pulses.date      ) );
	memcpy( log_data_wr_pulses.device_id,  (char *)( params_get_meter_id()  ),                sizeof( log_data_wr_pulses.device_id ) );//Telit_dev_identifier()
	memcpy( log_data_wr_pulses.crc,        (char *)( pulses_get_crc()       ),                sizeof( log_data_wr_pulses.crc )       );
	log_data_wr_pulses.telegram_size    = strlen( log_data_wr_pulses.telegram_1 );
	log_data_wr_pulses.frame_type       = pulses_get_frame_type();
	log_data_wr_pulses.token            = CHECK_TOKEN;

    __programWordPulses(log_data_wr_pulses);

    dataloggerModule.sample_n = 0;

    return 1;
}

void DataLogger_RegisterPulses( void )
{
	if ( 0 == shutdown_initTelitModule() ) {
		if ( 1 == __writeToDataloggerPulses() ) {
			if ( MODBUS_SESSION_END == modbus_get_end_session() ) {
				shutdown_set( 1, params_config_read_time() );
			}
		}
	} else {
		__writeToDataloggerPulses();
	}
}

/**
 * @defgroup System_Threads System Threads
 * @brief Main system threads
 *
\dot
digraph main {
size="14";
SystemInit [shape=box];
DataloggerSensorWrite [shape=box];
DataloggerModbusWrite [shape=box];
MBUSTask [shape=box];
NarrowBandTask [shape=box];

SystemInit -> DataloggerSensorWrite -> DataloggerModbusWrite;
DataloggerModbusWrite -> MBUSTask -> NarrowBandTask -> DataloggerTask -> ScheduledTask;
ScheduledTask -> Task100ms -> Task1s -> DataloggerManagement;
ScheduledTask -> DataloggerManagement -> DataloggerSensorWrite;

inc_systick [shape=box];
IRQ_1ms -> inc_systick;
}

digraph task100ms {
size="14";

led_fsm [shape=box];
elapsed_100ms [shape=diamond];
Task100ms -> elapsed_100ms -> led_fsm [label = yes];
elapsed_100ms -> Exit [label = no]


}
\enddot

 *
 *
 * @{
 * */

/**
  * @brief Datalogger for pressure sensor thread. Writes data from ADC to Datalogger in SPI memory.
  * @retval None
  */
void Datalogger_SensorWrite(void)
{
    if (1 == Tick_cloud_time_init())
    {
    	if ((1 == AD_getEnd()) /*|| ( 1 == pulses_get_interruption() )*/ )
    	{
    		/** Writes Time and Date stamp in Sys_time.created_sensor_value_time/date */
    		rtc_system_SetCreatedSensorValueTime(Tick_Get(SECONDS));
    		rtc_system_SetCreatedSensorValueDate(Tick_Get(SECONDS));
    		/** Writes sensor data log in SPI FLASH */
    		if ( ( params_input_pulse_as_sensor_get_num() >= 1 ) && ( params_input_pulse_as_sensor_get_num() <= NUM_INPUTS_SENSOR ) )
    		{
    			if ( ( (0 == shutdown_initTelitModule())
    			   || ((1 == shutdown_initTelitModule())
    			    && (1 == params_get_mqtt_dc_on())
					&& (0 == message_queue_get_elements())
					  )
					 )
    					&& ( ( 0 == mbus_get_start_comm() )
    					||   (( 1 == generic_485_get_enable() ) && ( MODBUS_COMM_END != modbus_sensors_get_comm_state() )))
    					   )
    			{
    				if ((MODBUS_SESSION_END == modbus_get_end_session())
    						&& ( 0 == modbus_sensors_get_get_data() )
    						&& ( dlms_client_get_dlms_comm_state() != DLMS_COMM_STARTED )
    						&& ( 1 == CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue())) )
    				{
    					Tick_Force_1Second_Task();/**Forces one second task execution in order to save battery.*/
    					shutdown_set(1, rtc_system_getReadSensorAlarmCycleNextInSeconds());
    				}
    			}
    		}
    		else
    		{
    			sensor_log_pressure_register();
    		}
    	}
    }
}

/**
  * @brief Datalogger for MODBUS sensor thread. Writes data from MODUB Sensor to Datalogger in SPI memory.
  * @retval None
  */
void Datalogger_ModbusSensorsWrite(void)
{
    if (1 == Tick_cloud_time_init())
    {
    	/* If comms are undergoing do not go to sleep... */
    	if (MODBUS_COMM_STARTED == modbus_sensors_get_comm_state())
    	{
    		shutdown_set(0, rtc_system_getReadSensorAlarmCycleNextInSeconds());
    	}
    	/* If comms have finished get time/date stamp and process data */
    	else if (MODBUS_COMM_END == modbus_sensors_get_comm_state())
    	{
    		/** Writes Time and Date stamp in Sys_time.created_sensor_value_time/date */
    		/* TOASK ...CreatedModbusValueTime...??*/
    		rtc_system_SetCreatedSensorValueTime(Tick_Get(SECONDS));
    		rtc_system_SetCreatedSensorValueDate(Tick_Get(SECONDS));
    		if (1 == params_wq_on())
    		{
//    			modbus_sensors_log_params_register();
    			modbus_sensors_raw_frame_register();
    		}
    		else if (MODBUS_RAW == generic_485_get_type())
    		{
    			modbus_sensors_raw_frame_register();
    		}
    	}
    }
}

/**
  * @brief Datalogger for DLMS thread. Writes data from DLMS to Datalogger in SPI memory.
  * @retval None
  */
void Datalogger_DLMSWrite(void)
{
	if (1 == Tick_cloud_time_init())
	{
    	/* If comms are undergoing do not go to sleep... */
    	if (DLMS_COMM_STARTED == dlms_client_get_dlms_comm_state())
    	{
    		shutdown_set(0, rtc_system_getReadSensorAlarmCycleNextInSeconds());
    	}
    	else
    	{
    		if ((DLMS_COMM_END == dlms_client_get_dlms_comm_state()) || (DLMS_COMM_WRITE == dlms_client_get_dlms_comm_state()))
    		{
        		rtc_system_SetCreatedModbusValueTime(Tick_Get(SECONDS));
        		rtc_system_SetCreatedModbusValueDate(Tick_Get(SECONDS));
    			dlms_raw_frame_register();
    		}
    	}
	}
}

void DataLogger_Task(void)
{
	static uint32_t time_before_wr = 0;
	static uint32_t time_before_snd = 0;

    if (0 == init_time)
    {
        __initTime(&time_before_wr);
        __initTime(&time_before_snd);
        init_time = 1;
    }

    if ((1 == Tick_cloud_time_init())
#ifdef MODBUS
//      && ( ( MODBUS_SESSION_END == modbus_get_end_session() ) || ( MODBUS_SESSION_SEND_TELEGRAM == modbus_get_end_session() ))
#endif
		)
    {
    	__writeMeasLog(&time_before_wr);
    }

    if ((1 == rtc_system_InitByServer()) && (1 == udp_protocol_get_send_pending()) && (CONNECTED == Telit_is_socket_available()))
    {
#if defined(MBUS)
    	if (1 == shutdown_get_meter_send())
    	{
    	__measuresSend(&time_before_snd);
    	}
#elif defined (UNE82326)
    	if (1 == params_pulses_on())
    	{
    		__measuresSend(&time_before_snd);
    	}
    	else
    	if (1 == __framesSend())
    	{
//    		Telit_socket_quick_close_and_shutdown();//Close socket and shutdown.
    	}
#endif
    }
}

/**
 * @}
 *
 * */

void Datalogger_GetRdAddrBackUp( void )
{
	dataloggerModule.dl_params.rd_addr_backup = dataloggerModule.dl_params.rd_ini_addr;//dataloggerModule.dl_params.rd_addr;
}

void Datalogger_SetRdEndAddrBackUp( uint32_t _rd_end_addr_backup )
{
	dataloggerModule.dl_params.rd_end_addr_backup = _rd_end_addr_backup;
}

uint8_t Datalogger_CheckLogMemIsEmpty( void )
{
	uint8_t ret = ( dataloggerModule.dl_params.rd_end_addr == dataloggerModule.dl_params.rd_end_addr_backup )?1:0;
//	if ( 1 == params_pulses_on() ) {
//		ret = 0;
//	}
	return ret;
}

uint32_t Datalogger_GetRdIniAddr( void )
{
	return dataloggerModule.dl_params.rd_ini_addr;
}

uint32_t Datalogger_GetRdEndAddr( void )
{
	return dataloggerModule.dl_params.rd_end_addr;
}

uint32_t DataLogger_NextRdIniAddress( uint32_t src_addr )
{
	uint32_t ret = src_addr;
	if ( 0 == log_size )
	{
		log_size = 3 * sFLASH_SPI_PAGESIZE;//num_pages = 3
	}

	if ( src_addr - log_size >= (uint32_t) _logger_start_address ) {
		ret = src_addr - log_size;
	} else {
		ret = 0;
	}

	return ret;
}

uint8_t Datalogger_UpdateReadBufferPointerInLIFO( void )
{
	uint8_t  ret  = 1;
    uint32_t size = sizeof(log_data_wr);

    uint32_t num_pages     = size / sFLASH_SPI_PAGESIZE;
    uint32_t num_pages_mod = size % sFLASH_SPI_PAGESIZE;

    if ( num_pages_mod != 0 ) {
    	log_size = ( num_pages + 1 ) * sFLASH_SPI_PAGESIZE;
    }
    else {
    	log_size = num_pages * sFLASH_SPI_PAGESIZE;
    }


    if ( dataloggerModule.dl_params.rd_ini_addr - log_size >= (uint32_t) _logger_start_address ) {
        dataloggerModule.dl_params.rd_ini_addr -= log_size;
    } else {
    	if ( datalogger_meas_max_address - log_size > dataloggerModule.dl_params.rd_end_addr ) {
    		dataloggerModule.dl_params.rd_ini_addr = datalogger_meas_max_address - log_size + 1;
    	} else {
    		dataloggerModule.dl_params.rd_ini_addr = dataloggerModule.dl_params.rd_end_addr;
    	}
		mem_cycle = 0;
		params_set_mem_cycle(0);
    }

    if ( ( dataloggerModule.dl_params.rd_ini_addr <= dataloggerModule.dl_params.rd_end_addr ) && ( params_get_mem_cycle() == 0 ) ) {
        ret = 0;
    }

    dataloggerModule.dl_params.wr_addr = dataloggerModule.dl_params.rd_ini_addr;
    dataloggerModule.dl_params.rd_addr = dataloggerModule.dl_params.rd_ini_addr;

    return ret;
}

uint8_t Datalogger_UpdateReadBufferPointer( void )
{
    uint8_t ret = 1;

    if (dataloggerModule.dl_params.rd_addr + log_size <= datalogger_meas_max_address ) {
        dataloggerModule.dl_params.rd_addr += log_size;
    }
    else {
        dataloggerModule.dl_params.rd_addr = (uint32_t) _logger_start_address;
        mem_cycle = 0;
        params_set_mem_cycle(0);
    }

    if ((dataloggerModule.dl_params.rd_addr >= dataloggerModule.dl_params.wr_addr) && (params_get_mem_cycle() == 0)) {
        ret = 0;
    }

    if (init_rd_wr_pointers == 1) {
        if (dataloggerModule.dl_params.wr_addr != dataloggerModule.dl_params.rd_addr ) {
            DataLogger_SaveLogWrRdAdress();
        }
        else {
            DataLogger_SaveLogWrRdAdress();
            init_rd_wr_pointers = 0;
        }
    }

    return ret;
}

void DataLogger_ReadAddress(uint32_t *rd_address)
{
    *rd_address = (uint32_t )( dataloggerModule.dl_params.rd_addr );
}

void DataLogger_ReadIniAddress(uint32_t *rd_address)
{
    *rd_address = (uint32_t )( dataloggerModule.dl_params.rd_ini_addr );
}

/**
 * @fn uint8_t DataLogger_ReadFromDataloggerBuffer(dataLoggerRecord*, uint32_t)
 * @brief Copies the content of external FLASH memory to a buffer.
 *
 * @param dst pointer to a buffer to copy elements to.
 * @param src_addr address of the first element of external FLASH memory to start the copy.
 * @return next_addr Next address to read from for the next copy.
 * 			@arg 0 -
 * 			@arg 1 -
 */
uint8_t DataLogger_ReadFromDataloggerBuffer(dataLoggerRecord *dst, uint32_t src_addr)
{
    uint8_t next_addr = 0;

    /* Reads from datalogger */
    sFLASH_ReadBuffer((uint8_t *) dst, src_addr, sizeof(dataLoggerRecord));
    uint32_t time = dst->time_in_seconds;
    LOGLIVE(LEVEL_1, "LOGLIVE> %d LOG> rd_ini:0x%X len:%d\r\n",(int)Tick_Get( SECONDS ), (int)src_addr, (int)(sizeof(dataLoggerRecord)));
    /* If there is data calculates the following address to read from and exits */
    if (time != 0) //TOASK when FLASH memory is empty value is 0xFFFFFFFF
    {
    	next_addr = datalog_buffer_next_record();
    }
    return next_addr;
}

uint8_t DataLogger_ReadFromDataloggerBufferPulses(dataLoggerRecordPulses *dst, uint32_t src_addr)
{
    uint8_t next_addr = 0;

    sFLASH_ReadBuffer( (uint8_t *)dst, src_addr, sizeof( dataLoggerRecordPulses ) );
    uint32_t time = dst->time_in_seconds;
    if ( time != 0 ) {
    	next_addr = datalog_buffer_next_record();
    }
    return next_addr;
}

void Datalogger_RecoverSendFailRecord( void )
{
//	dataloggerModule.dl_params.rd_addr = dataloggerModule.dl_params.rd_addr_backup;
	dataloggerModule.dl_params.rd_ini_addr = dataloggerModule.dl_params.rd_addr_backup;
}

inline void DataLogger_Enable(void)
{
    dataloggerModule.dl_params.enable = 1;
}

inline void DataLogger_Disable(void)
{
    dataloggerModule.dl_params.enable = 0;
}

uint8_t Datalogger_GetType(void)
{
    return dataloggerModule.dl_params.write_period;
}

void Datalogger_SetType( uint8_t send_type )
{
    dataloggerModule.dl_params.send_type = send_type;
}

#if defined (UNE82326)
void DataLogger_SendEnd(void)
{
    dataloggerModule.dl_params.sending = READING_FRAMES;
}
#elif defined(MBUS)
void DataLogger_SendEnd(void)
{
    dataloggerModule.dl_params.sending = 0;
}
#endif

void DataLogger_SetSend( uint8_t _sending )
{
    dataloggerModule.dl_params.sending = _sending;
}

uint8_t DataLogger_GetRefresh( void )
{
    return refresh;
}

void DataLogger_SetRefresh( uint8_t refresh_val)
{
    refresh = refresh_val;
}

void DataLogger_SetWritePeriod(uint8_t write_period)
{
    init_time = 0;
    dataloggerModule.dl_params.write_period = write_period;
}

void DataLogger_SetSendPeriod(uint8_t send_period)
{
    init_time = 0;
    dataloggerModule.dl_params.send_period = send_period;
}
