/*
 * datalogger_buffer.c
 *
 *  Created on: 17 ago. 2019
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "common_lib.h"
#include "datalogger_buffer.h"
#include "message_queue.h"
#include "params.h"
#include "log.h"
#include "rtc.h"
#include "ME910.h"
#include "une82326.h"
#include "une82326_device_table.h"
#include "spi_flash.h"
#include "udp_protocol.h"
#include "shutdown.h"
#ifdef MBUS
#include "mbus.h"
#else
#define F_FRAME 0
#define M_FRAME 1
#endif

#include "dlms_client.h"
#define DATALOGGER_BUFFER_SIZE ( 60 )
#define SENSOR_1               ( 0 )
#define SENSOR_2               ( 1 )
#define SENSOR_3               ( 2 )
#define SENSOR_4               ( 3 )

#define EVENT_TYPE_3           ( 3 )

#define TELEGRAMS_IN_ONE_CSV   ( 4 )

typedef enum {
    CURRENT_MEAS,
    AVG_MEAS,
    MIN_MEAS,
    MAX_MEAS,
} measure_type;

/**
 * @struct
 * @brief buffer to store
 *
 */
static struct
{
    uint32_t	data_record[DATALOGGER_BUFFER_SIZE];	/**< Array of addresses to store the address of the datalogger */
    uint8_t     n;				/**< Number of elements in the datalogger buffer */
    uint8_t     n_backup;		/**< backup for number of elements in the buffer */
    uint8_t     start;			/**< start element of the buffer */
    uint8_t     start_backup;	/**< back up of the start element of the buffer */
    uint8_t     end;			/**< last element of the buffer */
    uint8_t     end_backup;		/**< backup for the last element of the buffer */
} datalog_buffer;

uint32_t start_meter_datalogger = 0, retry_meter_msg = 0, msg_meter_num_total = 0;

char idRequest[40];

char * datalog_get_idRequest( void )
{
	return idRequest;
}

uint32_t datalog_set_idRequest( char * __idRequest)
{
	strncpy((char *) idRequest, __idRequest, sizeof(idRequest));

	return 0;
}

uint32_t datalog_get_start_meter_datalogger( void )
{
	return start_meter_datalogger;
}

uint32_t datalog_get_retry_meter_msg( void )
{
	return retry_meter_msg;
}

void datalog_set_start_meter_datalogger( uint32_t _start_meter_datalogger )
{
	start_meter_datalogger = _start_meter_datalogger;
}

void datalog_set_retry_meter_msg( uint32_t _retry_meter_msg )
{
	retry_meter_msg = _retry_meter_msg;
}

void datalog_set_meter_msg_num_total( uint32_t _meter_msg_num_total )
{
	msg_meter_num_total = _meter_msg_num_total;
}

#if defined(UNE82326)
typedef struct
{
	char      frame_tx_buffer[4096];
	uint32_t  n;
	uint32_t  frame_num;
	uint32_t  rd_addr;
	uint32_t  rd_addr_backup;
	uint32_t  wr_addr;
	uint32_t  total_frames_size;
	char     *send_addr;
} frame_tx_buffer_st;

frame_tx_buffer_st frame_tx_buffer;

typedef struct
{
	uint32_t rd_addr;
	uint32_t wr_addr;
	uint32_t rd_addr_backup;
} device_frames_flash_pointers_st;

device_frames_flash_pointers_st device_frames_flash_pointers;

uint32_t msg_num;

extern struct params param;

void datalog_reset_frame_tx_buffer( void )
{
	memset( frame_tx_buffer.frame_tx_buffer, 0, sizeof( frame_tx_buffer.frame_tx_buffer ) );
	frame_tx_buffer.n         = 0;
	frame_tx_buffer.frame_num = 0;
	frame_tx_buffer.rd_addr   = 0;
	frame_tx_buffer.rd_addr_backup = 0;
	frame_tx_buffer.total_frames_size = 0;
//	frame_tx_buffer.send_addr = 0;
	frame_tx_buffer.wr_addr = 0;
}

char * datalog_get_frame_tx_buffer( void )
{
	return frame_tx_buffer.frame_tx_buffer;
}

uint32_t datalog_get_frame_tx_frame_num( void )
{
	return frame_tx_buffer.frame_num;
}

uint32_t datalog_get_frame_n( void )
{
	return frame_tx_buffer.n;
}

uint32_t datalog_frame_tx_get_rd_pointer( void )
{
	return frame_tx_buffer.rd_addr;
}

uint32_t datalog_frame_tx_get_wr_pointer( void )
{
	return frame_tx_buffer.wr_addr;
}

char * datalog_frame_tx_get_send_addr( void )
{
	return frame_tx_buffer.send_addr;
}

void datalog_frame_tx_set_wr_pointer( uint32_t _wr_addr )
{
	frame_tx_buffer.wr_addr = _wr_addr;
}

uint8_t datalog_check_frame_tx_pointers( void )
{
	uint8_t ret = ( frame_tx_buffer.rd_addr != frame_tx_buffer.wr_addr )?1:0;

	return ret;
}

uint8_t datalog_check_all_mesgs_sent( void )
{
	uint8_t ret = ( frame_tx_buffer.frame_num == msg_num )?1:0;

	return ret;
}

void datalog_recover_sent_fail_frame( void )
{
	frame_tx_buffer.rd_addr = frame_tx_buffer.rd_addr_backup;
}

void datalog_frame_tx_buffer_init( void )
{
	frame_tx_buffer.frame_num = 0;
	frame_tx_buffer.n         = 0;
	frame_tx_buffer.rd_addr   = 0;
	frame_tx_buffer.wr_addr   = 0;
	memset( frame_tx_buffer.frame_tx_buffer, 0, sizeof(frame_tx_buffer.frame_tx_buffer) );
}

void datalog_reset_buffer( void )
{
    memset( &datalog_buffer, 0, sizeof(datalog_buffer) );
}

unsigned char datalog_buffer_n( void )
{
    return datalog_buffer.n;
}

void datalog_buffer_recover_backup_end( void )
{
	datalog_buffer.n     = datalog_buffer.n_backup;
	datalog_buffer.start = datalog_buffer.start_backup;
	datalog_buffer.end   = datalog_buffer.end_backup;
}

static void __buffer_fill_data_rd_address( uint32_t *dst )
{
    DataLogger_ReadAddress(dst);
}

static void __updateCircBufferPointers( void )
{
    if( datalog_buffer.end < ( DATALOGGER_BUFFER_SIZE - 1 ) ) {
        datalog_buffer.end++;
    }
    else {
        datalog_buffer.end = 0;
    }

    if( datalog_buffer.end == datalog_buffer.start ) {
        if( datalog_buffer.start < ( DATALOGGER_BUFFER_SIZE - 1 ) ) {
            datalog_buffer.start++;
        }
        else {
            datalog_buffer.start = 0;
        }
    }
    else {
        datalog_buffer.n++;
    }
}

//We add one message to datalog_buffer send queue.
//Gets current read pointer, reads memory address and increments datalog_buffer queue number of elements to read.
uint8_t datalog_buffer_write( void )
{
    uint8_t  next_pos= 0, num = 0, ret = 0;
    uint32_t initial_tick = 0, elapsed = 0;

    initial_tick = Tick_Get(SECONDS);
    Datalogger_GetRdAddrBackUp();

    do {
        next_pos = Datalogger_UpdateReadBufferPointerInLIFO();
        __buffer_fill_data_rd_address( &datalog_buffer.data_record[datalog_buffer.end] );// copy dataloggerModule.dl_params.rd_addr in datalog_buffer.data_record queue the next address to send.
        __updateCircBufferPointers();													 // adding one element to datalog_buffer struct. datalog_buffer.n++.
        num++;
        elapsed  = Tick_Get(SECONDS) - initial_tick;
    } while ( next_pos && (elapsed < 5) && (num < 1) );

    if ( elapsed >= 5 ) {
        DataLogger_Init();
        DataLogger_ResetPointers();
        datalog_reset_buffer();
    }

    return ret;
}

static uint32_t num_sends = 0;
uint32_t datalog_get_num_sends( void )
{
	num_sends = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR4);
	return num_sends;
}

uint8_t datalog_buffer_write_device_frames( char * ptr )
{
	uint8_t  ret = 0;
	uint32_t frame_size = 0;

	frame_size = datalog_buffer_parse_messages( ptr );
	if ( frame_size > 1 ) {
		__updateCircBufferPointers();
	} else {
		ret = 1;
	}
	return ret;
}

#define TIME_VALUE_VALID   ((time_check != -1) && (1 == Tick_cloud_time_init()) && (1 == rtc_system_InitByServer())) || (1 == rtc_system_getWaitingTimeServer())
dataLoggerRecord record;
uint32_t datalog_buffer_get_message_num( void )
{
	return msg_num;
}

uint32_t datalog_buffer_read_device_frames( char *ptr )
{
	uint8_t          total_devices = 0, first_m_frame_register = 0, waiting_end_of_message = 0, last_frame_type;
	uint32_t         src_addr_next = 0, frame_num = 0, curr_device = 0, device_in_loop = 0;
	dataLoggerRecord curr_record;
	char             serial_num[ 18 ], date[20];
	uint32_t         rd_ini    = Datalogger_GetRdIniAddr();
	uint32_t         rd_end    = Datalogger_GetRdEndAddr();
	uint32_t         src_addr  = DataLogger_NextRdIniAddress(rd_ini);
	static char      SOMx[]    = "*SOM:";
	static char      EOMx[]    = "*EOM:";
	size_t           str_len   = 0;
	uint32_t         diff 	   = rd_ini - rd_end;
    size_t 			 str_len_ret = 0;//str_len_header = 0,
//    char             header[256];

	if ( ( diff ) > (uint32_t)UDP_LENGTH ) {
		rd_end = rd_ini - Datalogger_GetLogSize()*(UDP_LENGTH/768);//768*21=16128 (UDP_LENGTH = 16384)
	}
	Datalogger_SetRdEndAddrBackUp(rd_end);

	memset( &curr_record, 0, sizeof(dataLoggerRecord) );
	une82326_device_table_manager_read_table();
	total_devices = une82326_device_table_get_num_of_devices();

	// Messages to send for device: curr_device (checks its serial number).
	for (curr_device = 0; curr_device < total_devices; curr_device++) {
		memcpy( serial_num , (char *)( une82326_device_table_manager_get_device_serial_num(curr_device) ) , sizeof(serial_num) );
		if ( NULL != serial_num ) {
			for (src_addr = rd_ini; src_addr >= rd_end; src_addr = src_addr_next ) { // Let´s look after this device messages stored in Flash SPI memory.
				sFLASH_ReadBuffer( (uint8_t *)&curr_record, src_addr, sizeof( dataLoggerRecord ) );
				if ( 0 == memcmp( curr_record.device_id, serial_num, strlen(serial_num) ) ) {
//					memcpy(date, curr_record.date, sizeof(date));
					if ( curr_record.send_type == A_PLUS_FRAME ) {
						if ( ( 1               == first_m_frame_register )
						  && ( 1               == waiting_end_of_message )
						  && ( last_frame_type == WATER_METER_FRAME      )
						  ) {
							snprintf(ptr, UDP_LENGTH,
									"|%s%d*",//EOMx
									EOMx, (int)frame_num
							);
							frame_num++;
							str_len                = strlen( ptr );
							ptr                   += str_len;
							str_len_ret           += str_len;
							waiting_end_of_message = 0;
							first_m_frame_register = 0;
						}
						snprintf(ptr, UDP_LENGTH,
								"|%s%d|" //SOMx
								"%s|F|"  //GatewayID|M|
								"%s|"    //idBUSDevice
								"%s|"    //Checksum
								"%s|"    //Datetime
								"%s|"    //A+Frame
								"%s%d*", //EOMx
								SOMx,(int)frame_num,
								ME910_IMEI(),
								curr_record.device_id,
								curr_record.crc,
								curr_record.date,
								curr_record.frame,
								EOMx, (int)frame_num
						);
						frame_num++;
						str_len         = strlen( ptr );
						ptr            += str_len;
						str_len_ret    += str_len;
						last_frame_type = A_PLUS_FRAME;
						src_addr_next = DataLogger_NextRdIniAddress(src_addr);
					} else if (curr_record.send_type == WATER_METER_FRAME) {
						if ( ( 1               == first_m_frame_register )
						  && ( 1               == waiting_end_of_message )
						  && ( last_frame_type == WATER_METER_FRAME      )
						  && ( device_in_loop  != curr_device            )
						  ) {
							snprintf(ptr, UDP_LENGTH,
									"|%s%d*",//EOMx
									EOMx, (int)frame_num
							);
							frame_num++;
							str_len                = strlen( ptr );
							ptr                   += str_len;
							str_len_ret           += str_len;
							waiting_end_of_message = 0;
							first_m_frame_register = 0;
						}
						if ( ( 0 == first_m_frame_register )
						  || ( ( curr_record.date[6] != date[6] ) || ( curr_record.date[7] != date[7] ) ) ) { /*|| ( memcmp(date, curr_record.date, sizeof(date) ) != 0 )*/
							if ( ( last_frame_type == WATER_METER_FRAME      )
							  && ( 1               == waiting_end_of_message ) ) {
								snprintf(ptr, UDP_LENGTH,
										"|%s%d*",//EOMx
										EOMx, (int)frame_num
								);
								frame_num++;
								str_len                = strlen( ptr );
								ptr                   += str_len;
								str_len_ret           += str_len;
								waiting_end_of_message = 0;
								first_m_frame_register = 0;
							}
							snprintf(ptr, UDP_LENGTH,
									"|%s%d|" //SOMx
									"%s|M|"  //GatewayID|M|
									"%s|"    //idBUSDevice
									"%s|"    //Checksum
									"%s|"    //Datetime
									"%s",    //timexvaluex
									SOMx,(int)frame_num,
									ME910_IMEI(),
									curr_record.device_id,
									curr_record.crc,
									curr_record.date,
									curr_record.frame
							);
							device_in_loop         = curr_device;
							str_len                = strlen( ptr );
							ptr                   += str_len;
							str_len_ret           += str_len;
							first_m_frame_register = 1;
							waiting_end_of_message = 1;
							memcpy(date, curr_record.date, sizeof(date));
						} else {
							snprintf(ptr, UDP_LENGTH,
									"%s",  //timexvaluex
									curr_record.frame
							);
							str_len      = strlen( ptr );
							ptr         += str_len;
							str_len_ret += str_len;
						}
						last_frame_type = WATER_METER_FRAME;
						src_addr_next   = DataLogger_NextRdIniAddress(src_addr);
					}
				} else if ( ( last_frame_type == WATER_METER_FRAME      )
					     && ( 1               == waiting_end_of_message )
						 && ( device_in_loop  != curr_device            )
						 ) {
					snprintf(ptr, UDP_LENGTH,
							"|%s%d*",//EOMx
							EOMx, (int)frame_num
					);
					frame_num++;
					str_len                = strlen( ptr );
					ptr                   += str_len;
					str_len_ret           += str_len;
					waiting_end_of_message = 0;
					first_m_frame_register = 0;
					src_addr_next          = DataLogger_NextRdIniAddress(src_addr);
				} else {
					src_addr_next = DataLogger_NextRdIniAddress(src_addr);
				}
			}
		}
	}
	if ( ( last_frame_type == WATER_METER_FRAME      )
	  && ( 1               == waiting_end_of_message ) ) {
		snprintf(ptr, UDP_LENGTH,
				"|%s%d*",//EOMx
				EOMx, (int)frame_num
		);
		frame_num++;
		str_len                = strlen( ptr );
		ptr                   += str_len;
		str_len_ret           += str_len;
		waiting_end_of_message = 0;
		first_m_frame_register = 0;
	}
	msg_num = frame_num;
	frame_tx_buffer.total_frames_size = str_len_ret;
	frame_tx_buffer.wr_addr           = str_len_ret;

//	if ((PROTOCOL_TCP == params_protocol_mode()) && (str_len_ret != 0))
//	{
//		sprintf(header,
//				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
//				"Content-Type: text/plain\r\n"
//				"Host:172.31.27.44\r\n"
//				"Content-Length: %d\r\n"
//				"\r\n",
//				Telit_dev_identifier(),
//				str_len_ret
//		);
//
//		str_len_header = strlen( header );
//		ptr           -= (str_len_ret );
//		memmove(ptr + str_len_header, ptr, str_len_ret );
//		memcpy(ptr, header, str_len_header);
//		str_len_ret += str_len_header;
//		str_len      = str_len_ret;
//	}
//	else
//	{
//		str_len        = str_len_ret;
//	}

	return str_len_ret;
}

uint32_t datalog_buffer_parse_messages( char * ptr  )
{
	char   SOM_str[20],EOM_str[20];
	char   header[256];
	size_t str_len_header = 0;
	size_t str_len   = 0;

	sprintf(SOM_str,"|*SOM:%d|" , (int)frame_tx_buffer.frame_num );
	sprintf(EOM_str,"|*EOM:%d*|", (int)frame_tx_buffer.frame_num );
	// Getting first frame to send.
	ptr += frame_tx_buffer.rd_addr;
	ptr  = memmem( ptr, frame_tx_buffer.total_frames_size, SOM_str, 8 );
	ptr  = strtok( ptr, "|" );  //SOM
	ptr  = strtok( NULL, "*" ); //MSG

	if ( ( NULL != ptr ) && ( frame_tx_buffer.frame_num < datalog_buffer_get_message_num() ) ) {
		frame_tx_buffer.n = strlen(ptr) + 1;
		snprintf(frame_tx_buffer.frame_tx_buffer + frame_tx_buffer.rd_addr, frame_tx_buffer.n, "%s", ptr);
		frame_tx_buffer.send_addr = frame_tx_buffer.frame_tx_buffer + frame_tx_buffer.rd_addr;
		frame_tx_buffer.rd_addr  += frame_tx_buffer.n;
		frame_tx_buffer.frame_num++;
	} else {
		frame_tx_buffer.n = 0;
	}

	if ((PROTOCOL_TCP == params_protocol_mode()) && (frame_tx_buffer.n != 0))
	{
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
				"Host:172.31.27.44\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				(int)(frame_tx_buffer.n - 1)
		);

		str_len_header = strlen( header );
		memmove(frame_tx_buffer.frame_tx_buffer + str_len_header, frame_tx_buffer.send_addr, frame_tx_buffer.n );
		memcpy(frame_tx_buffer.frame_tx_buffer, header, str_len_header);
//		frame_tx_buffer.n += str_len_header;
		str_len      = frame_tx_buffer.n + str_len_header;
	}
	else
	{
		str_len      = frame_tx_buffer.n;
	}


	return str_len;//frame_tx_buffer.n;
}

uint32_t datalog_buffer_read( char *ptr, uint32_t msg_size )
{
    uint8_t          next_addr  = 0;
    uint8_t          num        = 0;
    size_t           str_len    = 0;
    int              time_check = 0;
    size_t 			 total_str_len = 0;
    size_t 			 str_len_header = 0;
    char             header[256];

    do {
    	memset( &record, 0, sizeof(dataLoggerRecord) );

        next_addr  = DataLogger_ReadFromDataloggerBuffer( &record, datalog_buffer.data_record[datalog_buffer.start] );
        time_check = (int) record.time_in_seconds;

        if ( (
        	   ( TIME_VALUE_VALID             )
			&& ( record.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN == record.token  )
			) ) {

        	if (record.send_type == A_PLUS_FRAME) {
        		snprintf(ptr,
        				sizeof(record.frame) + 6,
						"%s|F|%d|%s",
						ME910_IMEI(),
						(int)Tick_Get( SECONDS ),
						record.frame
        		);
        	} else if (record.send_type == WATER_METER_FRAME) {
        		snprintf(ptr,
        				sizeof(record.frame) + 6,
						"%s|M|%d|%s",
						ME910_IMEI(),
						(int)Tick_Get( SECONDS ),
						record.frame
						);
        	}

            str_len   = strlen( ptr );
            ptr      += str_len;
            num_sends = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR4);
            num_sends++;
            HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, num_sends);
            num++;
        }
        else {
        	if ( CHECK_TOKEN != record.token ) {
        		DataLogger_Init();
        		DataLogger_ResetPointers();
        		datalog_reset_buffer();
        	} else {
        		next_addr = 1;
        		num       = 0;
        	}
        }
    } while( next_addr && num < 1 );

	if ((PROTOCOL_TCP == params_protocol_mode()) && (total_str_len != 0))
	{
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
				"Host:172.31.27.44\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				total_str_len
		);

		str_len_header = strlen( header );
		ptr           -= (total_str_len );
		memmove(ptr + str_len_header, ptr, total_str_len );
		memcpy(ptr, header, str_len_header);
		total_str_len += str_len_header;
		str_len        = total_str_len;
	}
	else
	{
		str_len        = total_str_len;
	}

    return str_len;
}

void datalog_buffer_delete( )
{
    if( ! datalog_buffer.n ) {
        return;
    }

    if( datalog_buffer.start < (DATALOGGER_BUFFER_SIZE - 1) ) {
        datalog_buffer.start++;
    }
    else {
        datalog_buffer.start = 0;
    }

    datalog_buffer.n--;
}

uint8_t datalog_buffer_next_record(void)
{
    uint8_t ret = 1;

    if( ! datalog_buffer.n ) {
        return 0;
    }

    if ( datalog_buffer.start < ( DATALOGGER_BUFFER_SIZE - 1 ) ) {
        datalog_buffer.start++;
    }
    else {
        datalog_buffer.start = 0;
    }

    if ( datalog_buffer.start >= datalog_buffer.end ) {
        ret = 0;
    }

    datalog_buffer.n--;

    return ret;
}

uint8_t datalog_buffer_poll( void )
{
    if( datalog_buffer_n() && ! message_queue_get_elements() ) {
    	message_queue_write( SEND_MESSAGE );
    	return 1;
    }

    return 0;
}

dataLoggerRecordPulses record_pulses;
uint32_t datalog_buffer_read_pulses( char *ptr )
{
    uint8_t          next_addr  = 0, num = 0;
    size_t           str_len    = 0, str_len_header = 0, total_str_len = 0;
    int              time_check = 0;
    uint32_t         telegram_size = 0;
    static uint32_t  set_header = 0;
    char             header[ 256 ];

    do {
    	memset( &record_pulses, 0, sizeof(dataLoggerRecordPulses) );

        next_addr  = DataLogger_ReadFromDataloggerBufferPulses( &record_pulses, datalog_buffer.data_record[datalog_buffer.start] );
        time_check = (int) record_pulses.time_in_seconds;

        if ( (
        	   ( TIME_VALUE_VALID             )
        	&& ( record_pulses.telegram_1[0] != '\0' )
			&& ( record_pulses.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN == record_pulses.token  )
			) ) {
        	if ( 0 == set_header) {
        		set_header = 1;
        		if ( F_FRAME == record_pulses.frame_type) {
        			sprintf(ptr,
        					"%s|F|"
        					"%s|%s|%s|",
							ME910_IMEI(),
							record_pulses.device_id,
							record_pulses.crc,
							record_pulses.date
					);
        		} else if ( M_FRAME == record_pulses.frame_type) {
            		sprintf(ptr,
            				"%s|M|"
            				"%s|%s|%s|",
    						ME910_IMEI(),
							record_pulses.device_id,
							record_pulses.crc,
							record_pulses.date
					);
            	}
                str_len = strlen( ptr );
                ptr    += str_len;
        	}
        	if ( ( next_addr == 0 ) || ( num >= ( TELEGRAMS_IN_ONE_CSV - 1 ) ) ) {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 4 * sizeof(char),
						"%s\r\n\r\n",
						record_pulses.telegram_1
				);
				telegram_size += (record_pulses.telegram_size + 2);
        	}
        	else {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 2 * sizeof(char),
						"%s\r\n",
						record_pulses.telegram_1
				);
				telegram_size += (record_pulses.telegram_size + 2);
        	}
            str_len += strlen( ptr );
            ptr     += str_len;
            num++;
        }
        else if ( ( record_pulses.telegram_1[0] == '\0' ) || ( CHECK_TOKEN != record_pulses.token ) ) {
        	DataLogger_Init();
        	DataLogger_ResetPointers();
        	datalog_reset_buffer();
        }
    } while( next_addr && num < TELEGRAMS_IN_ONE_CSV );

	if ( PROTOCOL_TCP == params_protocol_mode() ) {
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
				"Host:172.31.27.44\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				str_len
		);
		str_len_header = strlen( header );
		ptr -= total_str_len;
		memmove(ptr + str_len_header, ptr, total_str_len );
		memcpy(ptr, header, str_len_header);
		total_str_len += str_len_header;
		str_len        = total_str_len;
	}

    set_header = 0;
    return str_len;
}

uint32_t datalog_buffer_read_modbus( char *ptr )
{
    uint8_t          next_addr     = 0;
    uint8_t          num           = 0;
    size_t           str_len       = 0;
#ifndef MODBUS
    int              time_check    = 0;
#endif
    uint32_t         telegram_size = 0;

    do {
    	memset( &record, 0, sizeof(dataLoggerRecord) );

        next_addr  = DataLogger_ReadFromDataloggerBuffer( &record, datalog_buffer.data_record[datalog_buffer.start] );
#ifndef MODBUS
        time_check = (int) record.time_in_seconds;
#endif

        if ( CHECK_TOKEN != record.token  ) {
        	Datalogger_writeToDatalogger();
        	datalog_buffer_write();
        	next_addr  = DataLogger_ReadFromDataloggerBuffer( &record, datalog_buffer.data_record[datalog_buffer.start] );
        }

        if ( (
#ifndef MODBUS
        	   ( TIME_VALUE_VALID             )
        	&&
#endif
			( record.frame[0]         != '\0' )
			&& ( record.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN          == record.token  )
			) ) {
				if ( ( next_addr == 0 ) || ( num >= ( TELEGRAMS_IN_ONE_CSV - 1 ) ) ) {
					snprintf(ptr,
							sizeof(record.frame) + 4 * sizeof(char),
							"%s\r\n\r\n",
							record.frame
					);
					telegram_size += (record.frame_size + 2);
				}
				else {
					snprintf(ptr,
							sizeof(record.frame) + 2 * sizeof(char),
							"%s\r\n",
							record.frame
					);
					telegram_size += (record.frame_size + 2);
				}
				str_len  = strlen( ptr );
				ptr     += str_len;
				num++;
//				if ( ( next_addr == 0 ) || ( num >= TELEGRAMS_IN_ONE_CSV ) ) {
//					http_post_content_length = telegram_size;
//				}
        	}
			else if ( ( record.frame[0] == '\0' ) || ( CHECK_TOKEN != record.token ) ) {
				DataLogger_Init();
				DataLogger_ResetPointers();
				datalog_reset_buffer();
			}
    } while( next_addr && num < TELEGRAMS_IN_ONE_CSV );

    return telegram_size;
}

uint32_t datalog_buffer_read_pulses_modbus( char *ptr )
{
    uint8_t          next_addr  = 0, num = 0;
    size_t           str_len    = 0, total_str_len = 0;
//    int              time_check = 0;
    uint32_t         telegram_size = 0;

    do {
    	memset( &record_pulses, 0, sizeof(dataLoggerRecordPulses) );

        next_addr  = DataLogger_ReadFromDataloggerBuffer( (dataLoggerRecord *)&record_pulses, datalog_buffer.data_record[datalog_buffer.start] );
#ifndef MODBUS
        time_check = (int) record_pulses.time_in_seconds;
#endif

        if ( CHECK_TOKEN != record.token  ) {
        	DataLogger_RegisterPulses();
        	datalog_buffer_write();
        	next_addr  = DataLogger_ReadFromDataloggerBuffer( (dataLoggerRecord *)&record_pulses, datalog_buffer.data_record[datalog_buffer.start] );
        }


        if ( (
#ifndef MODBUS
        	   ( TIME_VALUE_VALID             )
        	&&
#endif
			( record_pulses.telegram_1[0] != '\0' )
			&& ( record_pulses.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN == record_pulses.token  )
			) ) {
        	if ( ( next_addr == 0 ) || ( num >= ( TELEGRAMS_IN_ONE_CSV - 1 ) ) ) {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 4 * sizeof(char),
						"%s\r\n",
						record_pulses.telegram_1
				);
				telegram_size += (record_pulses.telegram_size + 2);
        	} else {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 2 * sizeof(char),
						"%s\r\n",
						record_pulses.telegram_1
				);
				telegram_size += (record_pulses.telegram_size + 2 + sizeof(record_pulses.telegram_size));
        	}
            str_len       += strlen( ptr );
            total_str_len += str_len;
            ptr           += str_len;
            num++;
        } else if ( ( record_pulses.telegram_1[0] == '\0' ) || ( CHECK_TOKEN != record_pulses.token ) ) {
        	DataLogger_Init();
        	DataLogger_ResetPointers();
        	datalog_reset_buffer();
        }
    } while( next_addr && num < TELEGRAMS_IN_ONE_CSV );

    return str_len;
}

#elif defined(MBUS)
uint8_t datalog_check_all_mesgs_sent( void )
{
	return 1;
}

void datalog_reset_buffer( void )
{
    memset( &datalog_buffer, 0, sizeof(datalog_buffer) );
}

unsigned char datalog_buffer_n( void )
{
    return datalog_buffer.n;
}

void datalog_buffer_recover_backup_end( void )
{
	datalog_buffer.n     = datalog_buffer.n_backup;
	datalog_buffer.start = datalog_buffer.start_backup;
	datalog_buffer.end   = datalog_buffer.end_backup;
}

#ifdef UNE82326
static void __buffer_fill_data_rd_address( uint32_t *dst )
{
    DataLogger_ReadAddress(dst);
}
#endif

static void __buffer_fill_data_rd_ini_address( uint32_t *dst )
{
    DataLogger_ReadIniAddress(dst);
}

static void __updateCircBufferPointers( void )
{
    if( datalog_buffer.end < ( DATALOGGER_BUFFER_SIZE - 1 ) ) {
        datalog_buffer.end++;
    }
    else {
        datalog_buffer.end = 0;
    }

    if( datalog_buffer.end == datalog_buffer.start ) {
        if( datalog_buffer.start < ( DATALOGGER_BUFFER_SIZE - 1 ) ) {
            datalog_buffer.start++;
        }
        else {
            datalog_buffer.start = 0;
        }
    }
    else {
        datalog_buffer.n++;
    }
}

void datalog_buffer_delete( )
{
    if( ! datalog_buffer.n ) {
        return;
    }

    if( datalog_buffer.start < (DATALOGGER_BUFFER_SIZE - 1) ) {
        datalog_buffer.start++;
    }
    else {
        datalog_buffer.start = 0;
    }

    datalog_buffer.n--;
}

/**
 * @fn uint8_t datalog_buffer_next_record(void)
 * @brief Sets the pointer of datalog_buffer to the next position to allow the next datalogger record.
 *
 * @return ret
 * 				@arg 0 -
 * 				@arg 1 -
 */
uint8_t datalog_buffer_next_record(void)
{
    uint8_t ret = 1;

    /* no elements */
    if(!datalog_buffer.n)
    {
        return 0;
    }
    /* Increments the position of the index */
    if (datalog_buffer.start < (DATALOGGER_BUFFER_SIZE - 1))
    {
        datalog_buffer.start++;
    }
    else
    {
        datalog_buffer.start = 0; //buffer overflow
    }

    /* Error? */
//    if (datalog_buffer.start >= datalog_buffer.end)
//    {
//        ret = 0;
//    }

    datalog_buffer.n--;

    return ret;
}

//We add one message to datalog_buffer send queue.
//Gets current read pointer, reads memory address and increments datalog_buffer queue number of elements to read.
uint8_t datalog_buffer_write( void )
{
    uint8_t  next_pos= 0, num = 0, ret = 0;
    uint32_t initial_tick = 0, elapsed = 0;

    initial_tick = Tick_Get(SECONDS);
    Datalogger_GetRdAddrBackUp();

    do {
        next_pos = Datalogger_UpdateReadBufferPointerInLIFO();
        __buffer_fill_data_rd_ini_address( &datalog_buffer.data_record[datalog_buffer.end] );// copy dataloggerModule.dl_params.rd_addr in datalog_buffer.data_record queue the next address to send.
        __updateCircBufferPointers();													 // adding one element to datalog_buffer struct. datalog_buffer.n++.
        num++;
        elapsed  = Tick_Get(SECONDS) - initial_tick;
    } while ( next_pos && (elapsed < 5) && (num < TELEGRAMS_IN_ONE_CSV) );

    if ( elapsed >= 5 ) {
        DataLogger_Init();
        DataLogger_ResetPointers();
        datalog_reset_buffer();
    }

    return ret;
}

/**
 * @def TIME_VALUE_VALID
 * @brief
 *
 */
#define TIME_VALUE_VALID   ((time_check != -1) && (1 == Tick_cloud_time_init()) )
/**
 * @brief record used to store the data read from FLASH memory
 */
dataLoggerRecord record;

/**
 * @fn uint32_t datalog_buffer_read(char*, uint32_t)
 * @brief
 *
 * @param ptr poiter to the array to transmit.
 * @param msg_size
 * @return
 */
uint32_t datalog_buffer_read( char *ptr, uint32_t msg_size )
{
    uint8_t          next_addr  = 0;  /**< 0 or 1 -  */
    uint8_t 		 num 		= 0;
    size_t           str_len    = 0;
    size_t 			 str_len_header = 0;
    size_t 			 total_str_len = 0;
    int              time_check = 0;
    uint32_t         telegram_size = 0;
    uint32_t 		 len = 0;
//    uint32_t		 datalogger_meter_threshold = 0;
    static uint32_t  set_header = 0;
    char             header[256];
    char             telegram[2048];
    char             crc[5];
    char            *tag;

    /* Reduced format frame type */
    if (0 == params_config_get_sensor_log())
    {
    	str_len = msg_size;
    }

    /** */
    do {
    	memset(&record, 0, sizeof(dataLoggerRecord));
        next_addr  = DataLogger_ReadFromDataloggerBuffer(&record, datalog_buffer.data_record[datalog_buffer.start]);
        time_check = (int) record.time_in_seconds;

        start_meter_datalogger = 0;
        if  (Tick_Get(SECONDS) > time_check )
        {
        	if ( ( Tick_Get(SECONDS) - time_check ) > params_config_send_time() )
        	{
        		start_meter_datalogger = 1;
        	}
        }
        /* There is data stored in datalogger memory space */
        if (((TIME_VALUE_VALID)	&& (record.telegram_1[0] != '\0') && (record.device_id[0]  != '\0')	&& (CHECK_TOKEN == record.token)))
        {
        	if (0 == set_header)
        	{
        		set_header = 1;
        		/* F frame*/
        		if (F_FRAME == record.frame_type)
        		{
        			memset(crc, 0 , sizeof(crc));
        			if ( SEVERN_TRENT_METER_TYPE == params_get_uart_meter_type() )
        			{
        				tag = "SF";
        				common_lib_string2hexString((char *)record.crc, (char *)crc, 4);
        			}
        			else
        			{
        				tag = "F";
        				common_lib_string2hexString((char *)record.crc, (char *)crc, 1);
        			}
        			crc[2] ='\0';
        			sprintf(ptr,
        					"%s|%s|"
        					"%s|%s|%s|",
							ME910_IMEI(),
							tag,
							record.device_id,
							crc,
							record.date
					);
        		}
        		/* M frame */
        		else if ( M_FRAME == record.frame_type ) {
        			memset(crc, 0 , sizeof(crc));
        			common_lib_string2hexString((char *)record.crc, (char *)crc, 1);
        			crc[2] ='\0';
        			sprintf(ptr,
            				"%s|M|"
            				"%s|%s|%s|",
    						ME910_IMEI(),
    						record.device_id,
							crc,
							record.date
					);
            	}

                str_len        = strlen(ptr);
                total_str_len += str_len + msg_size;
                ptr           += str_len;

                if (M_FRAME == record.frame_type)
                {
                	len = record.num_chars;
                }
                else if (F_FRAME == record.frame_type)
                {
                	len = str_len;
                }
        	}

        	/**/
        	if ((next_addr == 0) || (num >= (TELEGRAMS_IN_ONE_CSV - 1)))
        	{
        		if (TELEGRAM_MODE_DTK == params_telegram_mode())
        		{
        			snprintf(ptr,
        					sizeof(record.telegram_1),
							"%*s",
							(int)len,
							record.telegram_1
        			);
        			str_len        = strlen( ptr );
        			total_str_len += str_len;
        		}
        		else if (TELEGRAM_MODE_RAW == params_telegram_mode())
        		{
        			memset(telegram, 0 , sizeof(telegram));
        			mbus_set_raw_telegram_length(record.telegram_size);
        			common_lib_string2hexString((char *)record.telegram_1, telegram, 2 * mbus_get_raw_telegram_length());
        			telegram[ 2 * mbus_get_raw_telegram_length() ] = '|';
        			memcpy(telegram + 2 * mbus_get_raw_telegram_length() + 1, datalog_get_idRequest(), strlen(datalog_get_idRequest()));
        			telegram[ 2 * mbus_get_raw_telegram_length() + strlen(datalog_get_idRequest()) + 1 ] = '\r';
        			telegram[ 2 * mbus_get_raw_telegram_length() + strlen(datalog_get_idRequest()) + 2 ] = '\n';
        			memcpy(ptr, telegram, 2 * mbus_get_raw_telegram_length() + strlen(datalog_get_idRequest()) + 3);//memcpy(ptr, record.telegram_1, 2 * mbus_get_raw_telegram_length() + 1);
//        			memset(datalog_get_idRequest(), 0 , 40*sizeof(char));
        			if ( 1 == start_meter_datalogger ){
        				strcat(ptr,"DL|");
        			}
        			str_len       = strlen(ptr);
        			total_str_len += str_len;
//        			LOGLIVE(LEVEL_2, "LOGLIVE> %d LOG> CRC Frame: %s. M-BUS Telegram: %s.\r\n", (int)Tick_Get( SECONDS ), crc, telegram);
        		}

				telegram_size += record.telegram_size;
        	}
        	/**/
        	else
        	{
        		if (TELEGRAM_MODE_DTK == params_telegram_mode())
        		{
        			snprintf(ptr,
        					sizeof(record.telegram_1),
							"%*s",
							(int)len,
							record.telegram_1
        			);
        			str_len        = strlen( ptr );
        			total_str_len += str_len;
        		}
        		else if (TELEGRAM_MODE_RAW == params_telegram_mode())
        		{
        			memset(telegram, 0 , sizeof(telegram));
        			mbus_set_raw_telegram_length(record.telegram_size);
        			common_lib_string2hexString((char *)record.telegram_1, telegram, 2 * mbus_get_raw_telegram_length());
        			telegram[ 2 * mbus_get_raw_telegram_length() ] = '|';
        			memcpy(telegram + 2 * mbus_get_raw_telegram_length() + 1, datalog_get_idRequest(), strlen(datalog_get_idRequest()));
        			telegram[ 2 * mbus_get_raw_telegram_length() + strlen(datalog_get_idRequest()) + 1 ] = '\r';
        			telegram[ 2 * mbus_get_raw_telegram_length() + strlen(datalog_get_idRequest()) + 2 ] = '\n';
        			memcpy(ptr, telegram, 2 * mbus_get_raw_telegram_length() + strlen(datalog_get_idRequest()) + 3);//memcpy(ptr, record.telegram_1, 2 * mbus_get_raw_telegram_length() + 1);
//        			memset(datalog_get_idRequest(), 0 , 40*sizeof(char));
        			if ( 1 == start_meter_datalogger ){
        				strcat(ptr,"DL|");
        			}
        			str_len        = strlen(ptr);
        			total_str_len += str_len;
//        			LOGLIVE(LEVEL_2, "LOGLIVE> %d LOG> CRC Frame: %s. M-BUS Telegram: %s.\r\n",(int)Tick_Get( SECONDS ), crc, telegram);
        		}

				telegram_size += record.telegram_size;
        	}

            ptr += str_len;

            if (M_FRAME == record.frame_type)
            {
				snprintf(ptr,
						sizeof(record.num_chars) + 2 * sizeof(char),
						"|%d|",
						(int)record.num_chars
				);
            }

            num++;
            set_header = 0;
            msg_size   = 0;
#if 0
            if (0 == retry_meter_msg)
            {
        		msg_meter_num_total++;
        	}
//        	if ( params_config_send_time() >= 3600 ) {
        		datalogger_meter_threshold = (uint32_t) ((msg_meter_num_total * params_config_read_time()) / (params_config_send_time()));
        		if (datalogger_meter_threshold >= 1)
        		{
        			start_meter_datalogger = 1;
        			msg_meter_num_total    = 0;
        		}
//        	}
#endif
        }
        /* There is no data stored in datalogger memory space */
        else if ((record.telegram_1[0] == '\0') || (CHECK_TOKEN != record.token))
        {
        	DataLogger_Init();
        	DataLogger_ResetPointers();
        	datalog_reset_buffer();
        }
    } while(next_addr && num < TELEGRAMS_IN_ONE_CSV);

    retry_meter_msg = 0;

	if ((PROTOCOL_TCP == params_protocol_mode()) && (total_str_len != 0))
	{
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				total_str_len
		);

		str_len_header = strlen( header );
		ptr           -= (total_str_len );
		memmove(ptr + str_len_header, ptr, total_str_len );
		memcpy(ptr, header, str_len_header);
		total_str_len += str_len_header;
		str_len        = total_str_len;
	}
	else
	{
		str_len        = total_str_len;
	}

    set_header = 0;
    return str_len;
}

dataLoggerRecordPulses record_pulses;
uint32_t datalog_buffer_read_pulses( char *ptr )
{
    uint8_t          next_addr  = 0, num = 0;
    size_t           str_len    = 0, str_len_header = 0, total_str_len = 0;
    int              time_check = 0;
    uint32_t         telegram_size = 0;
    static uint32_t  set_header = 0;
    char             header[ 256 ];

    do {
    	memset( &record_pulses, 0, sizeof(dataLoggerRecordPulses) );

        next_addr  = DataLogger_ReadFromDataloggerBuffer( (dataLoggerRecord *)&record_pulses, datalog_buffer.data_record[datalog_buffer.start] );
        time_check = (int) record_pulses.time_in_seconds;

        if ( (
        	   ( TIME_VALUE_VALID             )
        	&& ( record_pulses.telegram_1[0] != '\0' )
			&& ( record_pulses.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN == record_pulses.token  )
			) ) {
        	if ( 0 == set_header) {
        		set_header = 1;
        		if ( F_FRAME == record_pulses.frame_type) {
        			sprintf(ptr,
        					"%s|F|"
        					"%s|%s|%s|",
							ME910_IMEI(),
							record_pulses.device_id,
							record_pulses.crc,
							record_pulses.date
					);
        		} else if ( M_FRAME == record_pulses.frame_type) {
            		sprintf(ptr,
            				"%s|M|"
            				"%s|%s|%s|",
    						ME910_IMEI(),
							record_pulses.device_id,
							record_pulses.crc,
							record_pulses.date
					);
            	}
                str_len       = strlen( ptr );
                total_str_len = str_len;
                ptr          += str_len;
        	}
        	if ( ( next_addr == 0 ) || ( num >= ( TELEGRAMS_IN_ONE_CSV - 1 ) ) ) {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 4 * sizeof(char) + 12,
						"%s|%d\r\n\r\n",
						record_pulses.telegram_1,
						(int)record_pulses.num_dig
				);
				telegram_size += (record_pulses.telegram_size + 2);
        	} else {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 2 * sizeof(char) + 12,
						"%s|%d\r\n",
						record_pulses.telegram_1,
						(int)record_pulses.num_dig
				);
				telegram_size += (record_pulses.telegram_size + 2 + sizeof(record_pulses.telegram_size));
        	}
            str_len       += strlen( ptr );
            total_str_len += str_len;
            ptr           += str_len;
            num++;
        } else if ( ( record_pulses.telegram_1[0] == '\0' ) || ( CHECK_TOKEN != record_pulses.token ) ) {
        	DataLogger_Init();
        	DataLogger_ResetPointers();
        	datalog_reset_buffer();
        }
    } while( next_addr && num < TELEGRAMS_IN_ONE_CSV );

	if ( PROTOCOL_TCP == params_protocol_mode() ) {
		sprintf(header,
				"POST /file3_restws/TCP_server/%s HTTP/1.1\r\n"
				"Content-Type: text/plain\r\n"
				"Host:%s\r\n"
				"Content-Length: %d\r\n"
				"\r\n",
				Telit_dev_identifier(),
				param.server[ME910_server_get_index()].name,
				str_len
		);
		str_len_header = strlen( header );
		ptr -= total_str_len;
		memmove(ptr + str_len_header, ptr, total_str_len );
		memcpy(ptr, header, str_len_header);
		total_str_len += str_len_header;
		str_len        = total_str_len;
	}

    set_header = 0;
    return str_len;
}

uint32_t datalog_buffer_read_pulses_modbus( char *ptr )
{
    uint8_t          next_addr  = 0, num = 0;
    size_t           str_len    = 0, total_str_len = 0;
//    int              time_check = 0;
    uint32_t         telegram_size = 0;

    do {
    	memset( &record_pulses, 0, sizeof(dataLoggerRecordPulses) );

        next_addr  = DataLogger_ReadFromDataloggerBuffer( (dataLoggerRecord *)&record_pulses, datalog_buffer.data_record[datalog_buffer.start] );
#ifndef MODBUS
        time_check = (int) record_pulses.time_in_seconds;
#endif

        if ( CHECK_TOKEN != record.token  ) {
        	DataLogger_RegisterPulses();
        	datalog_buffer_write();
        	next_addr  = DataLogger_ReadFromDataloggerBuffer( (dataLoggerRecord *)&record_pulses, datalog_buffer.data_record[datalog_buffer.start] );
        }


        if ( (
#ifndef MODBUS
        	   ( TIME_VALUE_VALID             )
        	&&
#endif
			( record_pulses.telegram_1[0] != '\0' )
			&& ( record_pulses.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN == record_pulses.token  )
			) ) {
        	if ( ( next_addr == 0 ) || ( num >= ( TELEGRAMS_IN_ONE_CSV - 1 ) ) ) {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 4 * sizeof(char),
						"%s\r\n",
						record_pulses.telegram_1
				);
				telegram_size += (record_pulses.telegram_size + 2);
        	} else {
				snprintf(ptr,
						sizeof(record_pulses.telegram_1) + 2 * sizeof(char),
						"%s\r\n",
						record_pulses.telegram_1
				);
				telegram_size += (record_pulses.telegram_size + 2 + sizeof(record_pulses.telegram_size));
        	}
            str_len       += strlen( ptr );
            total_str_len += str_len;
            ptr           += str_len;
            num++;
        } else if ( ( record_pulses.telegram_1[0] == '\0' ) || ( CHECK_TOKEN != record_pulses.token ) ) {
        	DataLogger_Init();
        	DataLogger_ResetPointers();
        	datalog_reset_buffer();
        }
    } while( next_addr && num < TELEGRAMS_IN_ONE_CSV );

    return str_len;
}

uint32_t datalog_buffer_read_modbus( char *ptr )
{
    uint8_t          next_addr     = 0;
    uint8_t          num           = 0;
    size_t           str_len       = 0;
#ifndef MODBUS
    int              time_check    = 0;
#endif
    uint32_t         telegram_size = 0;
    char             telegram[2048];

    do {
    	memset( &record, 0, sizeof(dataLoggerRecord) );

		DataLogger_Init();
		DataLogger_ResetPointers();
		datalog_reset_buffer();

        next_addr  = DataLogger_ReadFromDataloggerBuffer( &record, datalog_buffer.data_record[datalog_buffer.start] );
#ifndef MODBUS
        time_check = (int) record.time_in_seconds;
#endif

        if ( CHECK_TOKEN != record.token  ) {
        	Datalogger_writeToDatalogger();
        	datalog_buffer_write();
        	next_addr  = DataLogger_ReadFromDataloggerBuffer( &record, datalog_buffer.data_record[datalog_buffer.start] );
        }

        if ( (
#ifndef MODBUS
        	   ( TIME_VALUE_VALID             )
        	&&
#endif
			( record.telegram_1[0]    != '\0' )
			&& ( record.device_id[0]  != '\0' )
			&& ( CHECK_TOKEN          == record.token  )
			) ) {
				if ( ( next_addr == 0 ) || ( num >= ( TELEGRAMS_IN_ONE_CSV - 1 ) ) ) {
        			memset(telegram, 0 , sizeof(telegram));
        			mbus_set_raw_telegram_length(record.telegram_size);
        			common_lib_string2hexString((char *)record.telegram_1, telegram, 2 * mbus_get_raw_telegram_length());
//        			telegram[ 2 * mbus_get_raw_telegram_length() ] = '|';
        			telegram[ 2 * mbus_get_raw_telegram_length() ] = '\0';
        			strcat(telegram,"|\r\n");
        			memcpy(ptr, telegram, 2 * mbus_get_raw_telegram_length() + 3);
//					snprintf(ptr,
//							sizeof(telegram) + 4 * sizeof(char),
//							"%s\r\n\r\n",
//							telegram
//					);
//					telegram_size += (record.telegram_size + 2);
        			str_len       = strlen(ptr);
        			telegram_size += str_len;
				}
				else {
        			memset(telegram, 0 , sizeof(telegram));
        			common_lib_string2hexString((char *)record.telegram_1, telegram, 2 * mbus_get_raw_telegram_length());
//        			telegram[ 2 * mbus_get_raw_telegram_length() ] = '|';
        			telegram[ 2 * mbus_get_raw_telegram_length() ] = '\0';
        			strcat(telegram,"|\r\n");
        			memcpy(ptr, telegram, 2 * mbus_get_raw_telegram_length() + 3);
//					snprintf(ptr,
//							sizeof(telegram) + 2 * sizeof(char),
//							"%s\r\n",
//							telegram
//					);
//					telegram_size += (record.telegram_size + 2);
        			str_len       = strlen(ptr);
        			telegram_size += str_len;
				}
				str_len  = strlen( ptr );
				ptr     += str_len;
				num++;
//				if ( ( next_addr == 0 ) || ( num >= TELEGRAMS_IN_ONE_CSV ) ) {
//					http_post_content_length = telegram_size;
//				}
        	}
			else if ( ( record.telegram_1[0] == '\0' ) || ( CHECK_TOKEN != record.token ) ) {
				DataLogger_Init();
				DataLogger_ResetPointers();
				datalog_reset_buffer();
			}
    } while( next_addr && num < TELEGRAMS_IN_ONE_CSV );

    return telegram_size;
}

/**
 * @fn uint8_t datalog_buffer_poll(void)
 * @brief  detecta cuando tiene que enviar mbus
 *
 * @pre
 * @post
 * @return
 */
uint8_t datalog_buffer_poll( void )
{
    if (datalog_buffer_n() && ! message_queue_get_elements())
    {
    	if (1 == shutdown_get_meter_send())
    	{
    		message_queue_write(SEND_MESSAGE);
    		return 1;
    	}
    }

    return 0;
}
#endif
