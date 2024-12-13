/*
 * datalogger_buffer.h
 *
 *  Created on: 17 ago. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_DATALOGGER_INC_DATALOGGER_BUFFER_H_
#define APPLICATION_DATALOGGER_INC_DATALOGGER_BUFFER_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "params.h"

char   * datalog_get_idRequest( void );
uint32_t datalog_set_idRequest( char * __idRequest);
uint32_t datalog_get_start_meter_datalogger( void );
uint32_t datalog_get_retry_meter_msg( void );
void     datalog_set_start_meter_datalogger( uint32_t _start_meter_datalogger );
void     datalog_set_retry_meter_msg( uint32_t _retry_meter_msg );
void     datalog_set_meter_msg_num_total( uint32_t _meter_msg_num_total );
#ifdef UNE82326
void        datalog_reset_frame_tx_buffer( void );
char      * datalog_get_frame_tx_buffer( void );
uint32_t    datalog_get_frame_tx_frame_num( void );
uint32_t    datalog_get_frame_n( void );
uint32_t 	datalog_frame_tx_get_rd_pointer( void );
uint32_t 	datalog_frame_tx_get_wr_pointer( void );
void        datalog_frame_tx_set_wr_pointer( uint32_t _wr_addr );
char 	  * datalog_frame_tx_get_send_addr( void );
uint8_t     datalog_check_frame_tx_pointers( void );
void        datalog_frame_tx_buffer_init( void );
uint8_t     datalog_buffer_write_device_frames( char * ptr );
uint32_t    datalog_buffer_read_device_frames( char *ptr );
#endif
uint8_t     datalog_check_all_mesgs_sent( void );
void        datalog_recover_sent_fail_frame( void );
void 		datalog_reset_buffer( void );
uint8_t		datalog_buffer_n( void );
void 		datalog_buffer_recover_backup_end( void );
uint8_t     datalog_buffer_write( void );
uint32_t    datalog_get_num_sends( void );
uint32_t    datalog_buffer_get_message_num( void );
uint32_t    datalog_buffer_parse_messages( char * ptr );
uint32_t    datalog_buffer_read( char *ptr, uint32_t msg_size );
uint32_t    datalog_buffer_read_pulses( char *ptr );
void        datalog_buffer_delete( void );
uint8_t     datalog_buffer_next_record( void );
uint32_t    datalog_buffer_read_pulses_modbus( char *ptr );
uint32_t    datalog_buffer_read_modbus( char *ptr );
uint8_t     datalog_buffer_poll( void );

#endif /* APPLICATION_DATALOGGER_INC_DATALOGGER_BUFFER_H_ */
