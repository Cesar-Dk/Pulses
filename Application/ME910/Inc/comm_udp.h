#ifndef UDP_H
#define UDP_H

#include "main.h"
#include "ME910.h"
#include "comm_serial.h"

#define  UDP_MAX_TXRX_SIZE ( 1024 )//( 2048 )

void     udp_comm_reset_buffers( void );
void     udp_task( void );

#if defined(UDP)
#define  udp_is_socket_open()               Telit_is_socket_available()
#define  udp_socket_close()                 Telit_socket_close()
#define  udp_socket_close_and_shutdown()    Telit_socket_close_and_shutdown()

uint32_t udp_is_put_ready( void );
void     udp_put_array( uint8_t *data, uint32_t len );

uint32_t udp_is_get_ready( void );
uint8_t  udp_get( void );
int      udp_get_array( uint8_t *data, int len );
void     udp_delete( uint32_t len );
#endif

#endif
