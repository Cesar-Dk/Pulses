/*
 * circular_buffer.c
 *
 *  Created on: 30 abr. 2022
 *      Author: Sergio Millán López
 */

#include "circular_buffer.h"
#include "params.h"
#include "tick.h"
#include <stdlib.h>
#include <string.h>


//typedef struct CircularBufferStruct
//{
//	uint32_t   count;
//	uint32_t   index;
//	uint32_t   outdex;
//	uint32_t   capacity;
//	uint8_t  * values;
//} CircularBufferStruct;

enum {BUFFER_GUARD = 255};

void CircularBuffer_Create(CircularBuffer circular_buffer, uint32_t capacity, uint8_t *data)
{
	memset( data, 0, sizeof(uint8_t) * capacity);
	circular_buffer->count  			= 0;
    circular_buffer->index  			= 0;
    circular_buffer->outdex  			= 0;
    circular_buffer->capacity 			= capacity;
    circular_buffer->values 			= data;
    circular_buffer->values[capacity - 1] 	= BUFFER_GUARD;
}

void CircularBuffer_Destroy(CircularBuffer self)
{
//    free(self->values);
//    free(self);
	self->capacity = 0;
}

void CircularBuffer_Reset(CircularBuffer circular_buffer)
{
	circular_buffer->count  	= 0;
    circular_buffer->index  	= 0;
    circular_buffer->outdex  	= 0;
    circular_buffer->values[0] 	= BUFFER_GUARD;
}

uint32_t CircularBuffer_VerifyIntegrity(CircularBuffer self)
{
	return self->values[self->capacity] == BUFFER_GUARD;
}

uint32_t CircularBuffer_IsEmpty(CircularBuffer self)
{
    return self->count == 0;
}

uint32_t CircularBuffer_IsFull(CircularBuffer self)
{
    return self->count == self->capacity;
}

uint32_t CircularBuffer_Count(CircularBuffer self)
{
    return self->count;
}

uint32_t CircularBuffer_Put(CircularBuffer self, uint32_t value)
{
    if (self->count >= self->capacity)
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d CIRCULAR_BUFFER> BUFFER FULL!!!! Mess num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)self->count);
    	return 0;
    }

    self->count++;
    self->values[self->index++] = value;
    if (self->index >= self->capacity)
    {
        self->index = 0;
    }
    LOGLIVE(LEVEL_1, "LOGLIVE> %d CIRCULAR_BUFFER> PUT Mess:%d.Mess num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)value, (int)self->count);
    return 1;
}

uint32_t CircularBuffer_Replace(CircularBuffer self, uint32_t value, uint32_t replace_on)
{
    if (0 == replace_on)
    {
    	CircularBuffer_Put(self, value);
    }
    else
    {
	if (self->count >= self->capacity)
    {
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d CIRCULAR_BUFFER> BUFFER FULL!!!! Mess num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)self->count);
    	return 0;
    }

    self->values[self->outdex] = value;

    LOGLIVE(LEVEL_1, "LOGLIVE> %d CIRCULAR_BUFFER> REPLACE Mess:%d.Mess num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)value, (int)self->count);
    }
    return 1;
}

uint32_t CircularBuffer_Get(CircularBuffer self)
{
	uint32_t value;
    if (self->count <= 0)
    {
        return -1;
    }

    value = self->values[self->outdex++];
    self->count--;
    if (self->outdex >= self->capacity)
    {
        self->outdex = 0;
    }
    LOGLIVE(LEVEL_1, "LOGLIVE> %d CIRCULAR_BUFFER> GET Mess:%d.Mess num:%d.\r\n", (int)Tick_Get( SECONDS ), (int)value, (int)self->count);
    return value;
}

uint32_t CircularBuffer_Read(CircularBuffer self)
{
	uint32_t value;
    if (self->count <= 0)
    {
        return -1;
    }

    value = self->values[self->outdex];

    return value;
}

uint32_t CircularBuffer_Find(CircularBuffer self, uint32_t value_min, uint32_t value_max)
{
	uint32_t value = 0, outdex = 0;//, index = 0;
    if (self->count <= 0)
    {
        return -1;
    }

    if ( self->outdex <= self->index )
    {
    for (outdex = self->outdex; outdex < self->index; outdex++)
    {
    	if ((self->values[outdex] >= value_min) && (self->values[outdex] <= value_max))
    	{
    		value = 1;
    	}
    }
    }
    else
    {
        for (outdex = self->outdex; outdex < self->capacity; outdex++)
        {
        	if ((self->values[outdex] >= value_min) && (self->values[outdex] <= value_max))
        	{
        		value = 1;
        	}
        }
        for (outdex = 0; outdex < self->index; outdex++)
        {
        	if ((self->values[outdex] >= value_min) && (self->values[outdex] <= value_max))
        	{
        		value = 1;
        	}
        }
    }

    return value;
}

uint32_t CircularBuffer_Capacity(CircularBuffer self)
{
    return self->capacity;
}
