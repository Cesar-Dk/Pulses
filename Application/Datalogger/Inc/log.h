/**
  ******************************************************************************
  * @file           log.h
  * @author 		Datakorum Development Team
  * @brief          Header file for log.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * log.h
 *
 *  Created on: 17 ago. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_DATALOGGER_INC_LOG_H_
#define APPLICATION_DATALOGGER_INC_LOG_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "params.h"
#include "tick.h"

#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))

#define CHECK_TOKEN (0x5A5A5A5A)

#define DATALOGGER_SEND_ENABLED  (0)
#define DATALOGGER_SEND_DISABLED (1)


/**
 * @enum
 * @brief log mode
 *
 */
typedef enum
{
    MEASURES_LOG,/**< MEASURES_LOG */
    EVENTS_LOG,  /**< EVENTS_LOG */
} log_mode;

#if defined(UNE82326)
/**
 * @struct __log_data
 * @brief
 *
 */
typedef struct __log_data
{
    uint32_t time_in_seconds;	/**< */
	char     frame[512];		/**< */
	char     r_field[19];		/**< */
	char     crc[5];			/**< */
	char     device_id[18];		/**< */
	char     date[20];			/**< */
	uint32_t frame_size;		/**< */
	uint32_t send_type;			/**< */
	uint32_t token;				/**< */
} dataLoggerRecord;
#elif defined (MBUS)
/**
 * @struct __log_data
 * @brief
 *
 */
typedef struct __log_data
{
    uint32_t time_in_seconds;	/**< */
	char     telegram_1[512];	/**< */
	char     crc[5];			/**< */
	char     device_id[20];		/**< */
	char     date[20];			/**< */
	uint32_t frame_type;		/**< */
	uint32_t num_chars;			/**< */
	uint32_t telegram_size;		/**< */
	uint32_t token;				/**< */
} dataLoggerRecord;
#endif

/**
 * @struct __pulses_log_data
 * @brief
 *
 */
typedef struct __pulses_log_data
{
    uint32_t time_in_seconds;
	char     telegram_1[512];
	char     crc[5];
	char     device_id[20];
	char     date[20];
	uint32_t num_dig;
	uint32_t frame_type;
	uint32_t telegram_size;
	uint32_t token;
} dataLoggerRecordPulses;

/**
 * @enum
 * @brief
 *
 */
typedef enum
{
    READING_FRAMES = 0,   		/**< READING_FRAMES */
	WAITING_PARSE_NEXT_FRAME,	/**< WAITING_PARSE_NEXT_FRAME */
    PARSING_NEW_FRAME,    		/**< PARSING_NEW_FRAME */
} sending_mode;

void     DataLogger_SaveLogWrRdAdress( void );
void     DataLogger_Init( void );
void     DataLogger_ResetPointers( void );
void     Datalogger_Set_Send_Disable( uint32_t _disable );
uint32_t Datalogger_Get_Send_Disable( void );
void     Datalogger_NextFramesMemoryBlock( void );
uint8_t  Datalogger_CheckLIFOPointers( void );
uint8_t  Datalogger_CheckPointers( void );
void     Datalogger_SetFirstWrite( uint8_t _first_write );
uint32_t Datalogger_GetLogSize( void );
char 	*Datalogger_Get_Info( void );
uint32_t Datalogger_get_frame_size( void );
uint8_t  Datalogger_writeToDatalogger(void);
void     DataLogger_Register( log_mode log_mode );
void 	 DataLogger_RegisterPulses( void );
uint8_t  DataLogger_Read( dataLoggerRecord *dst );
void     DataLogger_ReadAddress( uint32_t *rd_address );
void     DataLogger_ReadIniAddress(uint32_t *rd_address);
uint8_t  DataLogger_ReadFromDataloggerBuffer( dataLoggerRecord *dst, uint32_t src_address );
uint8_t  DataLogger_ReadFromDataloggerBufferPulses(dataLoggerRecordPulses *dst, uint32_t src_addr);

void     Datalogger_SensorWrite( void );
void     Datalogger_ModbusSensorsWrite( void );
void     Datalogger_DLMSWrite(void);
void     DataLogger_Task( void );
void     Datalogger_GetRdAddrBackUp( void );
void     Datalogger_SetRdEndAddrBackUp( uint32_t _rd_end_addr_backup );
uint8_t  Datalogger_CheckLogMemIsEmpty( void );
uint32_t Datalogger_GetRdIniAddr( void );
uint32_t Datalogger_GetRdEndAddr( void );
uint32_t DataLogger_NextRdIniAddress(  uint32_t src_addr  );
uint8_t  Datalogger_UpdateReadBufferPointerInLIFO( void );
uint8_t  Datalogger_UpdateReadBufferPointer( void );
void     Datalogger_RecoverSendFailRecord( void );
void     DataLogger_Enable( void );
void     DataLogger_Disable( void );
uint8_t  Datalogger_GetType( void );
void     Datalogger_SetType( uint8_t send_type );
void     DataLogger_SendEnd( void );
void     DataLogger_SetSend( uint8_t _sending );
uint8_t  DataLogger_GetRefresh( void );
void     DataLogger_SetRefresh( uint8_t refresh_val);
void     DataLogger_SetWritePeriod( uint8_t write_period );
void     DataLogger_SetSendPeriod( uint8_t send_period );

#endif /* APPLICATION_DATALOGGER_INC_LOG_H_ */
