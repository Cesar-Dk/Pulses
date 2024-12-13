/*
 * comm_udp.c
 *
 *  Created on: 17 jul. 2019
 *      Author: Sergio Millán López
 */

#include <comm_udp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_stdint.h>
#include <tick.h>
#include <usart.h>

#ifdef UDP
typedef enum
{
	UDP_IDLE,
	UDP_WAITING_SSENDEXT, UDP_WAITING_SSENDEXT_OK,
	UDP_WAITING_SRECV,
	UDP_WAITING_DATA
} udp_status_st;

udp_status_st udp_status;

static struct
{
	uint8_t  data[ UDP_MAX_TXRX_SIZE ];
	uint32_t len;
} comm_udp_tx, comm_udp_rx;

static void     __checkTimeout( uint32_t *t );

////////////////////////////////////////////////////////////////////////////////

void udp_comm_reset_buffers( void )
{
	memset( comm_udp_rx.data, 0, sizeof( comm_udp_rx.data ) );
	memset( comm_udp_tx.data, 0, sizeof( comm_udp_tx.data) );
}

void udp_task()
{
    static uint32_t t, len_rx;
	static char     AT_str[ 32 ];
	char *s;

	switch( udp_status )
	{
		case UDP_IDLE:
			Tick_update_tick( TICK_UDP );
			if( udp_is_socket_open() ) {
				// \r\nSRING: 1,3\r\n
				if(( comm_udp_rx.len == 0 )
				  && ME910_StringEqual( "\r\nSRING: ", 9 )
				  && ME910_StringEnds( "\r\n", 2, comm_serial_rcx_bytes_n() )) {
					strcpy( AT_str, "AT#SRECV=1,1500,0\r" );
					ME910_EraseBuffer();
					comm_serial_trx(&huart3, ( unsigned char *) AT_str, strlen( AT_str ));
					comm_serial_delete( comm_serial_rcx_bytes_n() );
					Tick_update_tick( TICK_UDP );
					t          = 30;
					udp_status = UDP_WAITING_SRECV;
				}
				else if( comm_udp_tx.len
					&& ( comm_serial_trx_bytes_n() == 0 )) {
					if ( 1 == params_manteinance_release_assistance() ) {
						sprintf( AT_str, "AT#SSENDEXT=1,%d,2\r", ( int ) comm_udp_tx.len );// MAX NUM BYTES: 512.
					} else if ( 0 == params_manteinance_release_assistance() ) {
						sprintf( AT_str, "AT#SSENDEXT=1,%d\r", ( int ) comm_udp_tx.len );// MAX NUM BYTES: 512.
					}
					ME910_EraseBuffer();
					comm_serial_trx(&huart3, ( unsigned char *) AT_str, strlen( AT_str ));
					comm_serial_delete( comm_serial_rcx_bytes_n() );
					Tick_update_tick( TICK_UDP );
					t          = 10;
					udp_status = UDP_WAITING_SSENDEXT;
				}
			}
			break;
		case UDP_WAITING_SSENDEXT:
			if( ME910_StringEqual( "\r\n> ", 4 )) {
				ME910_EraseBuffer();
				comm_serial_trx(&huart3, ( unsigned char *) comm_udp_tx.data, comm_udp_tx.len );
//				comm_serial_receive_buffer(&huart3);
				comm_serial_delete( comm_serial_rcx_bytes_n() );
				Tick_update_tick( TICK_UDP );
				t          = 10;
				udp_status = UDP_WAITING_SSENDEXT_OK;
			}
			break;
		case UDP_WAITING_SSENDEXT_OK:
			if( ME910_StringEqual( "\r\nOK\r\n", 6 )) {
				comm_udp_tx.len = 0;
//				comm_serial_delete( 6 );
				comm_serial_delete( comm_serial_rcx_bytes_n() );
				ME910_EraseBuffer();
				Tick_update_tick( TICK_UDP );
				t          = 2;
				udp_status = UDP_IDLE;
			}
			break;
		case UDP_WAITING_SRECV:
			// \r\n#SRECV: 1,3\r\n\003\005\0\r\nOK\r\n
			if( ME910_StringEqual( "\r\n#SRECV: ", 10 )
			 && ME910_StringEnds( "\r\n", 2, comm_serial_rcx_bytes_n() )) {
				comm_serial_rcx_n(( unsigned char * ) AT_str, 32 );
				s      = strstr(AT_str, ": ");
				len_rx = atoi(s + 4);
				Tick_update_tick( TICK_UDP );
				t          = 10;
				udp_status = UDP_WAITING_DATA;
        	}
			break;
		case UDP_WAITING_DATA:
			if (comm_serial_rcx_bytes_n() >= len_rx + 18) {
				uint8_t i;
				for( i = 13; i < 32; i++ ) {
					if( AT_str[ i ] == '\n' ) {
						i++;
						comm_serial_delete( i );
						memset( comm_udp_rx.data, 0, sizeof( comm_udp_rx.data ));
						comm_udp_rx.len = comm_serial_rcx_n( comm_udp_rx.data, len_rx );
						break;
					}
				}
				comm_serial_delete( comm_serial_rcx_bytes_n() );
				ME910_EraseBuffer();
				Tick_update_tick( TICK_UDP );
				t          = 3;
				udp_status = UDP_IDLE;
			}
			break;
		default:
			udp_status = UDP_IDLE;
			break;
	}

	__checkTimeout( &t );
}

////////////////////////////////////////////////////////////////////////////////

uint32_t udp_is_put_ready()
{
	if ( comm_udp_tx.len == 0 ) {
		return UDP_MAX_TXRX_SIZE;
	}
	return 0;
}

void udp_put_array( uint8_t *data, uint32_t len )
{
	if ( 0 == len ) {
		asm("nop");
	}
	comm_udp_tx.len = min( len, UDP_MAX_TXRX_SIZE );
	memcpy( comm_udp_tx.data, data, comm_udp_tx.len );
}

////////////////////////////////////////////////////////////////////////////////

uint32_t udp_is_get_ready()
{
	return comm_udp_rx.len;
}

uint8_t udp_get()
{
	return comm_udp_rx.data[ 0 ];
}

int udp_get_array( uint8_t *data, int len )
{
	len = min( len, comm_udp_rx.len );
	memcpy( data, comm_udp_rx.data, len );
	udp_delete( len );
	return len;
}

// borra N datos y desplaza los que queden al principio del buffer
void udp_delete( uint32_t len )
{
	uint32_t i, rem_len;

	len = min( len, comm_udp_rx.len );
	if( comm_udp_rx.len != len ) {
		rem_len = comm_udp_rx.len - len;
		for( i = 0; i < rem_len; i++ ) {
			comm_udp_rx.data[ i ] = comm_udp_rx.data[ i + len ];
		}
	}
	comm_udp_rx.len -= len;
}

////////////////////////////////////////////////////////////////////////////////

static void __checkTimeout( uint32_t *t )
{
	uint32_t time = *t + 1;
	if( CHECK_ELAPSED_TIME( time, TICK_UDP )) {
		memset( &comm_udp_rx, 0, sizeof( comm_udp_rx ));
		memset( &comm_udp_tx, 0, sizeof( comm_udp_tx ));
		Tick_update_tick( TICK_UDP );
		*t = 1;
		udp_status = UDP_IDLE;
	}
}
#endif
