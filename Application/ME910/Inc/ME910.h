/**
  ******************************************************************************
  * @file           ME910.h
  * @author 		Datakorum Development Team
  * @brief          Header file for ME910.c.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * ME910.h
 *
 *  Created on: 15 jul. 2019
 *      Author: Sergio Mill�n L�pez
 */

#ifndef APPLICATION_ME910_INC_ME910_H_
#define APPLICATION_ME910_INC_ME910_H_

#include "stm32u5xx_hal.h"
#include "params.h"

typedef enum {
	NOT_CONNECTED = 0,
	CONNECTED,
} conn_status;

typedef enum {
	ACTIVE = 0,
	SHUTDOWN,
	QUICK_SHUTDOWN,
	RECONNECT,
	ENTERS_PSM,
	HTTP_CONNECTION
} shtdwn_status;

typedef enum {
	ERROR_INIT = 0,
	ERROR_REGISTERING,
	ERROR_TRYING_TO_CONNECT,
	ERROR_CONNECTED
}sME910_error;

typedef struct {
	conn_status   connectedStatus;
	shtdwn_status shutdownStatus;
	uint32_t      timeConnected;
	uint32_t      attemptsConnection;
	uint32_t      tcpSocketAvailable;
	uint32_t      udpSocketAvailable;
	uint32_t      CSQ;
	uint32_t      PSMMode;
	char          IP[ 17 ];
	char          ICCID[ 20 ];
	char          IMSI[ 16 ];
	char          IMEI[ 16 ];
	char          dev_identifier[ 10 ];
	sME910_error  error;
}ME910_t;

typedef struct {
	int32_t  rssi;
	int32_t  rsrq;
	int32_t  rsrp;
	uint32_t csq;
	uint32_t pci;
	uint32_t sinr;
	uint32_t ecl;
	uint32_t tx_power;
	int32_t  total_power;
	uint32_t battery_live;
	char     cell_id[10];
	uint32_t rtt_mean;
	uint32_t creg;
	uint32_t abnd;
}NetworkParameters_t;

#define  ME910_CHECK_PSM	(HAL_GPIO_ReadPin( UC_NBIOT_PWRMON_GPIO_Port, UC_NBIOT_PWRMON_Pin ))

int32_t  ME910_network_params_rssi( void );
int32_t  ME910_network_params_rsrq( void );
int32_t  ME910_network_params_rsrp( void );
uint32_t ME910_network_params_csq( void );
uint32_t ME910_network_params_pci( void );
uint32_t ME910_network_params_sinr( void );
uint32_t ME910_network_params_ecl( void );
uint32_t ME910_network_params_tx_power( void );
uint32_t ME910_network_params_total_power( void );
uint32_t ME910_network_params_battery_live( void );
char *   ME910_network_params_cell_id( void );
uint32_t ME910_network_params_rtt_mean( void );
void     ME910_network_params_set_rtt_mean( uint32_t rtt_mean );
uint32_t ME910_network_params_creg( void );
uint32_t ME910_network_params_abnd( void );

void          ME910_init( void );
void 		  ME910_reinit_device( void );
void          ME910_inc_conn_attempts( void );
void          ME910_reset_conn_attempts( void );
uint32_t      ME910_get_conn_attempts( void );
conn_status   ME910_ModuleConnected( void );
void          ME910_ModuleSetConnection( conn_status connected );
int8_t        ME910_test_ok( void );
char 		 *ME910_getIP(void);
void          ME910_set_network_params_get( uint8_t _network_params_get );
uint8_t       ME910_get_network_params_get( void );
void          ME910_set_dk_network_params_get( uint8_t _dk_network_params_get );
uint8_t       ME910_get_dk_network_params_get( void );
void          ME910_EraseBuffer( void );

uint8_t ME910_checkPWRMON( void );
uint8_t ME910_setPowerDown( void );
void    ME910_setTimeToWait( uint32_t _time_to_wait_cmd );
void    ME910_task( void );
uint8_t Telit_wake_up_from_PSM( void );
void    ME910_enable( void );
void    Telit_modem_disable( void );
void    Telit_modem_off_prc( void );
void    Telit_disable( void );
uint8_t Telit_in_PSM( void );
void    Telit_save_status( void );
uint8_t Telit_read_status( void );
uint8_t Telit_read_error_status(void);
uint8_t Telit_wait_exit_stop_mode( void );
uint8_t Telit_write_data( char * str );
uint8_t Telit_write_data_length( char * str, uint32_t length );
uint8_t Telit_write_header_data( char * str, char * str_hdr );

uint32_t     ME910_get_connection_time( void );
void         ME910_reset_status( void );
void         ME910_set_error( sME910_error error );
sME910_error ME910_get_error( void );

char *  ME910_ICCID( void );
char *  ME910_IMSI( void );
char *  ME910_IMEI( void );
char *  Telit_dev_identifier( void );
int8_t  ME910_StringEqual(char * string, size_t lenstring);
uint8_t ME910_StringEnds(char * string, size_t lenstring, uint32_t end_len);

uint32_t ME910_getCSQ( void );
void     ME910_setPSMMode( uint8_t mode );
uint32_t ME910_getPSMMode( void );
char     *Telit_getSignalStrengthCondition( void );

uint8_t ME910_APN_get_new_index( void );
uint8_t ME910_APN_get_index( void );
uint8_t ME910_APN_get_current( void );
void    ME910_APN_set_current( uint8_t _apn );
void    ME910_APN_reset_index( void );
uint8_t ME910_server_get_new_index( void );
uint8_t ME910_server_get_index( void );
void    ME910_server_reset_index( void );

#ifndef UDP
conn_status Telit_is_socket_available( void );
void        Telit_socket_set_available(conn_status available);
void        Telit_socket_close( void );
void        Telit_socket_close_and_shutdown( void );
void        Telit_socket_quick_close_and_shutdown( void );
void        Telit_socket_quick_close_and_shutdown( void );
void 		Telit_socket_reconnect( void );
void 		Telit_socket_enters_PSM( void );
#else
uint8_t 	Telit_is_socket_available( void );
void    	Telit_socket_set_available(uint8_t available);
void   	 	Telit_socket_close( void );
void    	Telit_socket_close_and_shutdown( void );
void 		Telit_socket_quick_close_and_shutdown( void );
void 		Telit_socket_reconnect( void );
void 		Telit_socket_enters_PSM( void );
void        Telit_socket_HTTP_connection( void );
#endif

#endif /* APPLICATION_ME910_INC_ME910_H_ */
