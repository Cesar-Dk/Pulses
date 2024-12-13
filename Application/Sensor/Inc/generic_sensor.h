/*
 * generic_sensor.h
 *
 *  Created on: 26 oct. 2023
 *      Author: Sergio Millán López
 */

#ifndef SENSOR_INC_GENERIC_SENSOR_H_
#define SENSOR_INC_GENERIC_SENSOR_H_

#include "stm32u5xx_hal.h"
#include "params.h"
#include "generic_sensor_log.h"

#define NUM_INPUTS (5U)//(4U)

/*
A pointer to an incomplete type (hides the implementation details).
*/
typedef struct generic_sensor_t* generic_sensor_Ptr;

typedef uint32_t (*generic_sensor_input_read)(void);
typedef void (*generic_sensor_input_exec)(void);

void     generic_sensor_set_on_off(uint32_t num, uint32_t _on_off);
uint32_t generic_sensor_get_on_off(uint32_t num);
void     generic_sensor_set_end_conv(uint32_t num, uint32_t _end_conv);
uint32_t generic_sensor_get_end_conv(uint32_t num);
void     generic_sensor_set_burst_mode(uint32_t num, uint32_t _burst_mode);
uint32_t generic_sensor_get_burst_mode(uint32_t num);
uint32_t generic_sensor_get_input_read(uint32_t num);
uint32_t generic_sensor_set_sensors_to_send(void);
uint32_t generic_sensor_get_next_sensor_to_send(void);
void     generic_sensor_init(void);
void     generic_sensor_task(void);

#endif /* SENSOR_INC_GENERIC_SENSOR_H_ */
