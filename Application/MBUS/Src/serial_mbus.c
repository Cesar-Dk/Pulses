/*
 * serial_mbus.c
 *
 *  Created on: 15 oct. 2019
 *      Author: Sergio Millán López
 */

#include <main.h>
#include <serial_mbus.h>
#include <stm32u5xx_hal_def.h>

#include "usart.h"
#include "tick.h"
#include "mbus.h"

#ifdef MBUS
#define	min( a, b )	(((a) < (b)) ? (a) : (b))
#define	max( a, b )	(((a) > (b)) ? (a) : (b))

unsigned char buff[PACKET_BUFF_SIZE];

static struct
{
    uint8_t data[ MBUS_TXSIZE ];
    uint32_t n;
} serial_mbus_tx;

static struct
{
	uint8_t data[ MBUS_RXSIZE ];
	uint32_t n, start, end;
} serial_mbus_rx;

static uint32_t start_bit;

extern uint32_t unselected;

static uint32_t mbus_RC_FLAG;
uint32_t serial_mbus_get_RC_FLAG( void )
{
	return mbus_RC_FLAG;
}

void serial_mbus_set_RC_FLAG( uint32_t  _mbus_RC_FLAG )
{
	mbus_RC_FLAG = _mbus_RC_FLAG;
}

void serial_mbus_pwr_enable( void )
{
	MBUS_TX_DISABLE();
	MBUS_PWR_ENABLE();
}

void serial_mbus_pwr_disable( void )
{
	MBUS_PWR_DISABLE();
}

void serial_mbus_set_start_bit( uint32_t _start_bit )
{
	start_bit = _start_bit;
}

uint32_t serial_mbus_get_start_bit( void )
{
	return start_bit;
}

uint8_t *serial_mbus_rx_data_pointer( void )
{
	return serial_mbus_rx.data + serial_mbus_rx.start;
}

uint8_t *serial_une82326_tx_data_pointer( void )
{
	return serial_mbus_tx.data;
}

uint32_t serial_mbus_rx_num( void )
{
	return serial_mbus_rx.n;
}

uint32_t serial_mbus_tx_num( void )
{
	return serial_mbus_tx.n;
}

/**
 * @fn uint32_t serial_mbus_rcx(uint8_t*)
 * @brief Copies the data existing in serial_mbus_rx to the string pointed by *data
 * @param data pointer to the string to copy the serial_mbus_rx buffer to.
 * @return n number of elements copied.
 */
uint32_t serial_mbus_rcx(uint8_t *data)
{
	uint32_t i, j, n;

	j = serial_mbus_rx.start;
	n = serial_mbus_rx.n;

	for (i = 0; i < n; i++)
	{
		data[i] = serial_mbus_rx.data[j];
		if (j < (MBUS_RXSIZE - 1))
		{
			j++;
		}
		else
		{
			j = 0;
		}
	}
    return n;
}

uint32_t serial_mbus_rcx_n( uint8_t *data, uint8_t n )
{
    uint32_t i, j;

    j = serial_mbus_rx.start;
    n = min( n, serial_mbus_rx.n );

    for( i = 0; i < n; i++ ) {
    	data[ i ] = serial_mbus_rx.data[ j ];
		if( j < ( MBUS_RXSIZE - 1 ) ) {
			j++;
		}
		else {
			j = 0;
		}
    }

    return n;
}

uint32_t serial_mbus_rcx_bytes_n( void )
{
    return serial_mbus_rx.n;
}

void serial_mbus_set_rcx_bytes_n( uint32_t n )
{
    serial_mbus_rx.n = n;
}

void serial_mbus_delete( uint32_t n )
{
	uint32_t x;

	n = min( n, serial_mbus_rx.n );
	x = serial_mbus_rx.start + n;

	if( x > ( MBUS_RXSIZE - 1 )) {
		serial_mbus_rx.start = x - MBUS_RXSIZE;
	}
    else {
    	serial_mbus_rx.start = x;
    }

	serial_mbus_rx.n -= n;

}

void serial_mbus_rx_vars_init( void )
{
	serial_mbus_rx.end = 0;
	serial_mbus_rx.start = 0;
	serial_mbus_rx.n = 0;
	memset( serial_mbus_rx.data, 0 , sizeof(serial_mbus_rx.data) );
}

uint32_t serial_mbus_receive_one_byte(UART_HandleTypeDef *uart)
{
	HAL_UART_Receive_IT(uart, (uint8_t *) &serial_mbus_rx.data[serial_mbus_rx.end], 1);

	return 0;
}

uint32_t serial_mbus_receive_buffer(UART_HandleTypeDef *uart)
{
	/*## Put UART Emulation in reception process ###########################*/
	if ( HAL_UART_Receive_IT( uart, (uint8_t *)&serial_mbus_rx.data[serial_mbus_rx.end], MBUS_RXSIZE ) != HAL_OK ) {
		Error_Handler();
	}
	return 0;
}

uint32_t serial_mbus_int_rx(UART_HandleTypeDef *uart)
{
	if (serial_mbus_rx.end < ( MBUS_RXSIZE - 1)) {
		serial_mbus_rx.end++;
	} else {
		serial_mbus_rx.end = 0;
	}
	if (serial_mbus_rx.end == serial_mbus_rx.start) {
		if (serial_mbus_rx.start < ( MBUS_RXSIZE - 1)) {
			serial_mbus_rx.start++;
		} else {
			serial_mbus_rx.start = 0;
		}
	} else {
		serial_mbus_rx.n++;
	}

	/*## Put UART in reception process ###########################*/
	serial_mbus_set_RC_FLAG(0);
	if (HAL_UART_Receive_IT(uart, (uint8_t *) &serial_mbus_rx.data[serial_mbus_rx.end], MBUS_RXSIZE) == HAL_ERROR) {
		Error_Handler();
	}
	return 0;
}

void serial_mbus_wait_rx(UART_HandleTypeDef *uart)
{
	  while ( serial_mbus_get_RC_FLAG() != 1 )
	  {}
}

/**
 * @fn uint32_t serial_mbus_trx(UART_HandleTypeDef*, uint8_t*, uint32_t)
 * @brief Sends a string of data pointed by *data with a length of data_len through MBUS UART in interruption mode.
 * @param uart Handle to the used for MBUS peripheral UART
 * @param data pointer to the string of data to send.
 * @param data_len length of the string of data to send.
 * @return HAL status
 * 			@arg HAL_ERROR
 * 			@arg HAL_OK
 */
uint32_t serial_mbus_trx( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len )
{
	uint32_t i, j;

	serial_mbus_tx.n = min(data_len, MBUS_TXSIZE);

	for (i = 0, j = serial_mbus_tx.n; j; i++, j--)
	{
		serial_mbus_tx.data[j - 1] = data[i];
	}

	serial_mbus_tx.n--;
	HAL_UART_SetUartReady(RESET);

	if (HAL_UART_Transmit_IT(uart, &serial_mbus_tx.data[serial_mbus_tx.n], 1) != HAL_OK)
	{
		//Error_Handler();
		return HAL_ERROR;
	}

	while( HAL_UART_GetUartReady() != SET){}
	HAL_UART_SetUartReady(RESET);
	return HAL_OK;
}

void serial_mbus_trx_header_msg( UART_HandleTypeDef *uart, uint8_t *data, uint32_t data_len, uint8_t *hdr, uint32_t hdr_len )
{
	uint32_t i, j;

	serial_mbus_tx.n = min( data_len + hdr_len, MBUS_TXSIZE );

	for( i = 0, j = data_len; j; i++, j-- ) {
		serial_mbus_tx.data[ j - 1 ] = data[ i ];
	}

	for( i = 0, j = hdr_len; j; i++, j-- ) {
		serial_mbus_tx.data[ j - 1 + data_len ] = hdr[ i ];
	}

	if (HAL_UART_Transmit_IT(uart, &serial_mbus_tx.data[serial_mbus_tx.n], serial_mbus_tx.n)!= HAL_OK) {
		Error_Handler();
	}
}

uint32_t serial_mbus_trx_bytes_n( void )
{
    return serial_mbus_tx.n;
}

uint32_t serial_mbus_int_tx(UART_HandleTypeDef *uart)
{
	if( serial_mbus_tx.n ) {
		serial_mbus_tx.n--;
		if (HAL_UART_Transmit_IT(uart, &serial_mbus_tx.data[serial_mbus_tx.n], 1)!= HAL_OK) {
			Error_Handler();
		}
	} else {
		return 1;
	}

	return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
int32_t
serial_mbus_recv_frame(mbus_frame *frame)
{
    int remaining, timeouts;
    uint32_t len, nread = 0, nread_prev = 0;

    uint32_t initial_tick = 0, elapsed = 0, elapsed_2 = 0;

    initial_tick = Tick_Get(SECONDS);

    if (frame == NULL)
    {
        return MBUS_RECV_RESULT_ERROR;
    }

    memset((void *)buff, 0, sizeof(buff));

    //
    // read data until a packet is received
    //
    remaining = 1; // start by reading 1 byte
    len       = 0;
    timeouts  = 0;

    do
    {
    	elapsed_2 = Tick_Get(SECONDS) - initial_tick;
    	if ( elapsed_2 > 20 )
    	{
    		break;
    	}

        if (len + remaining > PACKET_BUFF_SIZE)
        {
            // avoid out of bounds access
            return MBUS_RECV_RESULT_ERROR;
        }
        HAL_Delay(200);
        do {
        	elapsed    = Tick_Get(SECONDS) - initial_tick;
        	nread_prev = serial_mbus_rcx_n(&buff[len], remaining);
        	serial_mbus_delete(nread_prev);
        	nread     += nread_prev;
            if (len > (MBUS_RXSIZE-nread))
            {
                // avoid overflow
                return MBUS_RECV_RESULT_ERROR;
            }
            len += nread_prev;
        } while( ( nread_prev != 0 ) && ( elapsed < 5 ) );

        if (nread == 0)
        {
            timeouts++;

            if (timeouts >= 3)
            {
                // abort to avoid endless loop
                break;
            }
        }
    } while ((remaining = mbus_parse(frame, buff, len)) > 0);

    mbus_set_raw_telegram_length( len );
    memcpy(mbus_get_raw_telegram(), buff, len);
    if (len > 100)
    {
    	LOGLIVE(LEVEL_2, "LOGLIVE> %d SERIAL_MBUS> RX: %s. Length:%d.\r\n", (int)Tick_Get( SECONDS ),(char *) mbus_get_raw_telegram(), (int)mbus_get_raw_telegram_length());
    }
    if (len == 0)
    {
        // No data received
        return MBUS_RECV_RESULT_TIMEOUT;
    }

    if (remaining != 0)
    {
        // Would be OK when e.g. scanning the bus, otherwise it is a failure.
    	LOGLIVE(LEVEL_2,"LOGLIVE> %s: M-Bus layer failed to receive complete data.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_INVALID;
    }

    if (len == -1)
    {
        return MBUS_RECV_RESULT_ERROR;
    }

    return MBUS_RECV_RESULT_OK;
}

int32_t serial_mbus_send_frame(mbus_frame *frame)
{
//    unsigned char buff[PACKET_BUFF_SIZE];
    int len;
    uint32_t initial_tick = 0, elapsed = 0;

    initial_tick = Tick_Get(SECONDS);

    if (frame == NULL)
    {
        return -1;
    }

    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        return -1;
    }

    serial_mbus_trx(&huart3, buff, len);
    //
    // wait until complete frame has been transmitted
    //
    do
    {
    	elapsed  = Tick_Get(SECONDS) - initial_tick;
    }while( ( serial_mbus_trx_bytes_n() != 0 ) && ( elapsed < 5 ) );

    return 0;
}

#endif
