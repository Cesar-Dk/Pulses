/*
 * message_queue.h
 *
 *  Created on: 17 ago. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_DATALOGGER_INC_MESSAGE_QUEUE_H_
#define APPLICATION_DATALOGGER_INC_MESSAGE_QUEUE_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "params.h"

/**
 * @enum
 * @brief
 *
 */
typedef enum
{
	SEND_MESSAGE,       /**< MBUS Message */
	SEND_NETWORK_PARAMS,/**< SEND_NETWORK_PARAMS */
	SEND_SENSOR,        /**< SEND_SENSOR */
	SEND_MODBUS_SENSOR, /**< SEND_MODBUS_SENSOR */
	SEND_NETWORK_PARAMS_CMD,/**< SEND_NETWORK_PARAMS_CMD */
	FINAL = 0xFF,       /**< FINAL */
} send_type;

void      message_queue_init( void );
uint32_t  message_queue_get_elements( void );
void      message_queue_write( uint8_t x );
void      message_queue_replace(uint8_t x, uint32_t replace_on);
uint8_t   message_queue_read( void );
void      message_queue_delete( uint32_t n );

#endif /* APPLICATION_DATALOGGER_INC_MESSAGE_QUEUE_H_ */
