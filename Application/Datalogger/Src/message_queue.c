/*
 * message_queue.c
 *
 *  Created on: 17 ago. 2019
 *      Author: Sergio Millán López
 */
#include "message_queue.h"
#include "tick.h"

// PUBLISH BUFFER //////////////////////////////////////////////////////////////
#define MESSAGE_PUB_BUFFER_SIZE (900)//(10)

/**
 * @struct
 * @brief
 *
 */
struct
{
	uint8_t  data[MESSAGE_PUB_BUFFER_SIZE];
	uint32_t n;
	uint32_t start;
	uint32_t end;
} message_queue;

/**
 * @fn void message_queue_init(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
void message_queue_init( void )
{
    memset( &message_queue, 0, sizeof( message_queue ));
}

/**
 * @fn uint8_t message_queue_get_elements(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint32_t message_queue_get_elements(void)
{
    return message_queue.n;
}

/**
 * @fn void message_queue_write(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param x
 */
void message_queue_write(uint8_t x)
{
	message_queue.data[message_queue.end] = x;

    if (message_queue.end < (MESSAGE_PUB_BUFFER_SIZE - 1))
    {
    	message_queue.end++;
    }
    else
    {
    	message_queue.end = 0;
    }

    if (message_queue.end == message_queue.start)
    {
        if (message_queue.start < (MESSAGE_PUB_BUFFER_SIZE - 1))
        {
        	message_queue.start++;
        }
        else
        {
        	message_queue.start = 0;
        }
    }
    else
    {
    	message_queue.n++;
    }
    LOGLIVE(LEVEL_1, "LOGLIVE> %d MESSAGE_QUEUE> Message Write:%d. Message num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)x, (int)message_queue.n);
}

void message_queue_replace(uint8_t x, uint32_t replace_on)
{
	if (0==replace_on)
	{
		message_queue_write( x );
	}
	else
	{
		message_queue.data[message_queue.start] = x;
	}
}
/**
 * @fn uint8_t message_queue_read(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t message_queue_read(void)
{
    return message_queue.data[message_queue.start];
}

/**
 * @fn void message_queue_delete(uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param n
 */
void message_queue_delete( uint32_t n )
{
    uint32_t x;

    if(! message_queue.n)
    {
    	return;
    }

    x = message_queue.start + n;

    if(x > (MESSAGE_PUB_BUFFER_SIZE - 1))
    {
    	message_queue.start = x - MESSAGE_PUB_BUFFER_SIZE;
    }
    else
    {
    	message_queue.start = x;
    }

    message_queue.n -= n;
    LOGLIVE(LEVEL_1, "LOGLIVE> %d MESSAGE_QUEUE> Delete Message:%d. Message num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)x, (int)message_queue.n);
}
