/*
 * une82326.c
 *
 *  Created on: 18 jul. 2019
 *      Author: smill
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "une82326.h"
#include "une82326_protocol.h"
#include "une82326_device_table.h"
#include "serial_une82326.h"
#include "udp_protocol.h"
#include "tick.h"
#include "usart.h"
#include "shutdown.h"
#include "leds.h"

#ifdef UNE82326
//#define DEBUG_15_CONTADORES

typedef enum {
	UNE82326_IDLE,
	UNE82326_ENABLE,
	UNE82326_A_AND_A_PLUS_FRAMES,
	UNE82326_END,
	UNE82326_LAST = 0xFF
}Une82326_Status;

static Une82326_Status une82326_status;
static Une82326        une82326;
static uint32_t        time_wait;

extern uint32_t reed_sending;

void une82326_init( void )
{
	une82326.end_comm   = 0;
	une82326.start_comm = 0;
	une82326_device_table_manager_init();
}

void une82326_reset( void )
{
	une82326.end_comm     = 0;
	une82326.start_comm   = 0;
	une82326.last_device  = 0;
	une82326.write_record = 0;
	une82326.type_frame   = 0;

	une82326_status       = UNE82326_IDLE;
}

uint8_t une82326_get_start_comm( void )
{
	return une82326.start_comm;
}

void une82326_set_start_comm( uint8_t _start_comm )
{
	une82326.start_comm = _start_comm;
}

uint8_t une82326_get_end_comm( void )
{
	return une82326.end_comm;
}

void une82326_set_end_comm( uint8_t _end_comm )
{
	une82326.end_comm = _end_comm;
}

uint8_t une82326_get_last_device( void )//No more devices as per UNE82326 specs (delay between devices communications).
{
	return une82326.last_device;
}

void une82326_set_last_device( uint8_t _last_device )
{
	une82326.last_device = _last_device;
	if (1 == _last_device)
	{
//		LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Last device.\r\n", (int)Tick_Get( SECONDS ));
	}
}

uint8_t une82326_get_write_record( void )
{
	return une82326.write_record;
}

void une82326_set_write_record( uint8_t _write_record )
{
	une82326.write_record = _write_record;
}

uint8_t une82326_get_frame_type( void )
{
	return une82326.type_frame;
}

void une82326_set_frame_type( uint8_t _frame_type )
{
	une82326.type_frame = _frame_type;
}

static void __goToNextState( uint32_t time, Une82326_Status next_status )
{
	Tick_update_tick( TICK_UNE82326 );
	time_wait       = time;
	une82326_status = next_status;
}

void une82326_task( void )
{
	static uint32_t   first_a_frame = 0, init_send = 0, num_dev = 0;
//	static uint32_t   num_dev = 0;
	error_frame_parse error_parse;
	uint8_t           no_comm = 0;

//	if ( 0 == Tick_cloud_time_init() ) {
//		if ( 0 == shutdown_initTelitModule()) {
//			init_send = 1;
//			une82326_set_start_comm(1);
//			shutdown_setInitTelitModule(1);
//		}
//		return;
//	}

	switch (une82326_status)
	{
	case UNE82326_IDLE:
		if ( ( 1 == une82326_get_start_comm() ) && ( 0 == params_pulses_on() )  ) {
//			if ( ( ( 1 == shutdown_initTelitModule() ) && ( 1 == Telit_is_socket_available() ) )
//			    || ( 0 == shutdown_initTelitModule() )
//			   ) {
				une82326_set_last_device(0);
				une82326_set_end_comm(0);
#ifdef DEBUG_15_CONTADORES
				if ( 0 == shutdown_initTelitModule()) {//DEBUG:
					shutdown_set_wchdg_time(SHUTDOWN_STAND_BY_MAX_TIME);//DEBUG:
				}
				__goToNextState(15, UNE82326_ENABLE);//DEBUG:15
#else
				__goToNextState(1, UNE82326_ENABLE);
#endif
//			}
		}
		break;
	case UNE82326_ENABLE:
		if ( CHECK_ELAPSED_TIME( time_wait, TICK_UNE82326 ) ) {
			if ( 0 == shutdown_initTelitModule()) {
				shutdown_set_wchdg_time(SHUTDOWN_STAND_BY_MAX_TIME);
			}
			une82326_device_table_manager_read_table();
			MX_UART_EMUL_DEINIT();
			MX_UART_EMUL_INIT();
			serial_une82326_unsel();
			serial_une82326_pwr_enable();
			first_a_frame = 1;
			__goToNextState( 3, UNE82326_A_AND_A_PLUS_FRAMES );
			num_dev = 0;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Started.\r\n", (int)Tick_Get( SECONDS ));
		}
		break;
	case UNE82326_A_AND_A_PLUS_FRAMES:
		if ( CHECK_ELAPSED_TIME( time_wait, TICK_UNE82326 ) ) {
			if ( ( 0 == une82326_get_last_device() ) && ( 0 == une82326_get_write_record() ) ) {
				if( 1 == first_a_frame ) {
					first_a_frame = 0;
					serial_une82326_sel();
//					leds_set_METER_status(METER_RD_WR);
					leds_LED_On(LED_GREEN);
					no_comm = serial_une2326_wait_rx(&UartEmulHandle);
//					memcpy(serial_une82326_rx_data_pointer(),"VSEL18012856\tRC000000.124480\tX3051\tA0\tQo0\tF1281\tBJ18YA012856K\tC408<\r",68);
//					serial_une82326_set_rcx_bytes_n(68);
					error_parse = une82326_protocol_parse((UNE82326_Frame *)serial_une82326_rx_data_pointer(), serial_une82326_rx_data_pointer(), serial_une82326_rx_num());
					serial_une82326_delete(serial_une82326_rx_num());
					if ( ( ERROR_A_FRAME_PARSE_OK == error_parse ) || ( ERROR_A_PLUS_FRAME_PARSE_OK == error_parse ) ) {
						une82326_device_table_manager_save_table();
						num_dev++;
						LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Read device 1st.Device num:%d Data:%s.\r\n", (int)Tick_Get( SECONDS ), (int)num_dev, une82326_device_table_manager_get_device_serial_num(num_dev - 1));
					}  else if ( ( ERROR_PARSE_UNKNOWN == error_parse ) || ( ERROR_A_PLUS_FRAME_PARSE_TX_CRC == error_parse ) ) {
						leds_set_METER_status(METER_MODULE_ERROR);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Read device 1st timeout.\r\n", (int)Tick_Get( SECONDS ));
					}
				} else {
//					char AUX_str[90];
//					num_dev++;
					serial_une82326_receive_buffer(&UartEmulHandle);
				    UART_Emul_Restart(&UartEmulHandle);
					serial_une82326_sel();
//					leds_set_METER_status(METER_RD_WR);
					leds_LED_On(LED_GREEN);
					no_comm = serial_une2326_wait_rx(&UartEmulHandle);
//					serial_une82326_rx_vars_init();
//					if (num_dev == 1) {
//						sprintf(AUX_str, "VSEL18012857\tRC000000.121110\tX3051\tA0\tQo0\tF1281\tBJ18YA012857P\tC<<;>\r");
//					} else if (num_dev == 2) {
//						sprintf(AUX_str, "VSEL1801286%d\tRC000000.121110\tX3051\tA0\tQo0\tF1281\tBJ18YA012857P\tC04>0\r", (int)num_dev);
//					} else if (num_dev == 3) {
//						sprintf(AUX_str, "VSEL1801286%d\tRC000000.121110\tX3051\tA0\tQo0\tF1281\tBJ18YA012857P\tC4093\r", (int)num_dev);
//					}
//					memcpy(serial_une82326_rx_data_pointer(), AUX_str, 68);
//					serial_une82326_set_rcx_bytes_n(68);
//					if (num_dev == 3) {
//					    num_dev = 0;
//						une82326_set_last_device(1);
//					}
					error_parse = une82326_protocol_parse((UNE82326_Frame *)serial_une82326_rx_data_pointer(), serial_une82326_rx_data_pointer(), serial_une82326_rx_num());
					serial_une82326_delete(serial_une82326_rx_num());
					if ( ( ERROR_A_FRAME_PARSE_OK == error_parse ) || ( ERROR_A_PLUS_FRAME_PARSE_OK == error_parse ) ) {
						une82326_device_table_manager_save_table();
						num_dev++;
						LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Read device.Device num:%d Data:%s.\r\n", (int)Tick_Get( SECONDS ), (int)num_dev, une82326_device_table_manager_get_device_serial_num(num_dev - 1));
					}  else if ( ( ERROR_PARSE_UNKNOWN == error_parse ) || ( ERROR_A_PLUS_FRAME_PARSE_TX_CRC == error_parse ) ) {
						leds_set_METER_status(METER_MODULE_ERROR);
						LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Read device timeout.\r\n", (int)Tick_Get( SECONDS ));
					}
				}
				if ( 1 == no_comm ) {
					if ( 0 == shutdown_initTelitModule() ) {
						shutdown_set( 1, params_config_read_time() );
					} else {
						__goToNextState( 2, UNE82326_END );
					}
				}
			} else {
				if ( WATER_METER_GATHERING_VALUES == une82326_get_frame_type() ) {
					une82326_set_frame_type(WATER_METER_FRAME);
//					une82326_set_write_record(1);
				} else if ( A_PLUS_FRAME == une82326_get_frame_type()) {
//    				une82326_set_write_record(1);
    			}
				if ( ( ALARM_SEND == rtc_system_getCurrentAlarmFromBKUP() ) || ( 1 == init_send ) || ( 1 == reed_sending ) ) {
					init_send    = 0;
					reed_sending = 0;
					udp_protocol_set_send_pending(1);
				}
				__goToNextState( 2, UNE82326_END );
			}
//			leds_set_METER_status(METER_LOW_POWER);
		}
		break;
	case UNE82326_END:
		if ( CHECK_ELAPSED_MILISEC( time_wait, TICK_UNE82326 ) ) {
//			leds_set_METER_status(METER_LOW_POWER);
			leds_LED_Off(LED_GREEN);
			MX_UART_EMUL_DEINIT();
			serial_une82326_pwr_disable();
			une82326_set_end_comm(1);
			une82326_set_start_comm(0);
			une82326_status = UNE82326_IDLE;
			LOGLIVE(LEVEL_1, "LOGLIVE> %d UNE82326> Read End. Device num:%d. Data:%s.\r\n", (int)Tick_Get( SECONDS ), (int)num_dev, serial_une82326_rx_data());
//			shutdown_setInitTelitModule(1);//DEBUG: Check...remove??
		}
		break;
	default:
		break;
	}
}
#endif
