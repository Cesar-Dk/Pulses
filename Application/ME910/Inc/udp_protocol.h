/**
  ******************************************************************************
  * @file           udp_protocol.h
  * @author 		Datakorum Development Team
  * @brief          Header file for udp_protocol.c
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
 * udp_protocol.h
 *
 *  Created on: 31 mar. 2019
 *      Author: smill
 */

#ifndef UDP_PROTOCOL_H_
#define UDP_PROTOCOL_H_

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

#include "params.h"
#include "comm_udp.h"

#ifdef UDP

#define UDP_LENGTH (3*4096 + 2*512)//(6*4096)

typedef enum {
	UDP_PROT_IDLE,
	UDP_PROT_CONNECTED,
	UD_PROT_GETTING_FRAMES,
	UDP_PROT_SEND,
	UDP_PROT_WAITING_FOR_RESPONSE,
	UDP_PROT_CLOSE,
	UDP_PROT_GET_FW_UPGRADE,
	UDP_PROT_FW_UPDATING,
	UDP_PROT_GET_TIME,
} udp_protocol_st;

void __processTimeoutWaitingResponse( void );
void            udp_protocol_reset_vars(void);
udp_protocol_st udp_protocol_get_status( void );
void            udp_protocol_set_no_ack( uint32_t  _no_ack );
uint32_t        udp_protocol_get_no_ack(void);
void            udp_protocol_set_waiting_answer( uint32_t  _waiting_answer );
uint32_t        udp_protocol_get_waiting_answer(void);
uint32_t        udp_get_server_resp_type( void );
void            udp_protocol_set_status( udp_protocol_st  _status );
void            udp_set_server_resp_type( uint32_t  _udp_server_resp_type );
uint32_t 		udp_protocol_get_send_ok( void );
void 			udp_protocol_set_send_ok( uint32_t ok );
void            udp_protocol_inc_send_ok( void );
uint32_t        udp_protocol_get_send_pending( void );
void            udp_protocol_set_send_pending( uint32_t  pend );
void 			rest_get_upgrading_data( void );
uint32_t 		rest_check_upgrading_data( void );
uint32_t 		rest_upgrading_firmware( void );
void 			rest_set_upgrading_firmware( uint32_t _upgrading_firmware );
void 			rest_reset_upgrading_data( void );
char 		  * rest_get_str_rx( void );
char          * rest_get_str_tx( void );
char          * rest_get_modbus_sensor_tx( void );
void 			udp_reset_buffers( void );
void            udp_protocol_set_on_demand_command( udp_protocol_st _on_demand_command );
uint32_t        udp_protocol_get_on_demand_command( void );
void            udp_protocol_set_on_periodic_profile( udp_protocol_st _on_demand_command );
uint32_t        udp_protocol_get_on_periodic_profile( void );
void            udp_protocol_set_on_billing_profile( udp_protocol_st _on_billing_profile );
uint32_t        udp_protocol_get_on_billing_profile(void);
void            udp_protocol_set_on_event_profile( udp_protocol_st _on_event_profile );
uint32_t        udp_protocol_get_on_event_profile(void);
uint32_t	    udp_protocol_get_send_network_params( void );
//void            udp_protocol_set_send_network_params( uint32_t  _network_params );
uint32_t        udp_protocol_send_modbus_sensor_param_frame( char *ptr_header, uint32_t modbus_sensor_param );
uint32_t        udp_protocol_send_network_params_msg( char *ptr_header, uint8_t queue );
void            udp_protocol_reconnect(  uint8_t sending  );
void            rest_get_fw_version( char * _fw_version );
uint32_t        rest_send_get_fw_update(char *ptr_header );
uint32_t        rest_send_head(char *ptr_header );
void            udp_protocol_restore_pointers( void );
void            udp_protocol_reset_profile_sending(void);
void            udp_protocol_reset_datalogger_max_msgs_params( void );
void 			udp_protocol_task( void );
time_t          udp_protocol_get_date( char *date );
uint8_t 		udp_protocol_process_response( char *ptr_header, uint32_t recv_bytes );
uint8_t         udp_protocol_rest_process_response( char *ptr );
uint32_t        udp_protocol_get_end_sensor_messages(void);
void            udp_protocol_set_end_sensor_messages( uint32_t _end_sensor_messages );
uint8_t         rest_get_fw_update_response( char *ptr_body, uint32_t *body_len );
time_t 			rest_get_date( char *date );
uint8_t		    rest_time_response( char *ptr_body);
#endif
#endif /* UDP_PROTOCOL_H_ */
