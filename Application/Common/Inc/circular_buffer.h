/*
 * circular_buffer.h
 *
 *  Created on: 30 abr. 2022
 *      Author: Sergio Millán López
 */

#ifndef COMMON_INC_CIRCULAR_BUFFER_H_
#define COMMON_INC_CIRCULAR_BUFFER_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "params.h"

typedef struct CircularBufferStruct
{
	uint32_t   count;
	uint32_t   index;
	uint32_t   outdex;
	uint32_t   capacity;
	uint8_t  * values;
} CircularBufferStruct;

typedef struct CircularBufferStruct * CircularBuffer;

void 		   CircularBuffer_Create(CircularBuffer self, uint32_t capacity, uint8_t *data);
void           CircularBuffer_Destroy(CircularBuffer self);
void 		   CircularBuffer_Reset(CircularBuffer circular_buffer);
uint32_t 	   CircularBuffer_IsEmpty(CircularBuffer self);
uint32_t       CircularBuffer_IsFull(CircularBuffer self);
uint32_t       CircularBuffer_Count(CircularBuffer self);
uint32_t       CircularBuffer_Put(CircularBuffer self, uint32_t);
uint32_t 	   CircularBuffer_Replace(CircularBuffer self, uint32_t value, uint32_t replace_on);
uint32_t       CircularBuffer_Get(CircularBuffer self);
uint32_t 	   CircularBuffer_Read(CircularBuffer self);
uint32_t       CircularBuffer_Find(CircularBuffer self, uint32_t value_max, uint32_t value_min);
uint32_t       CircularBuffer_Capacity(CircularBuffer self);
uint32_t       CircularBuffer_VerifyIntegrity(CircularBuffer self);

#endif /* COMMON_INC_CIRCULAR_BUFFER_H_ */
