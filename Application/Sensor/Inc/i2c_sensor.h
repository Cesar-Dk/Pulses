/*
 * i2c_sensor.h
 *
 *  Created on: 1 jul. 2024
 *      Author: Sergio Millán López
 */

#ifndef SENSOR_INC_I2C_SENSOR_H_
#define SENSOR_INC_I2C_SENSOR_H_

#include "stm32u5xx_hal.h"

#include "i2c.h"
#include "generic_sensor.h"

/*
A pointer to an incomplete type (hides the implementation details).
*/
typedef struct i2c_sensor_t* i2c_sensor_Ptr;
typedef uint32_t (*i2c_sensor_input_exec)(i2c_sensor_Ptr i2c_sensor);

void 		i2c_sensor_task(void);

uint32_t 	i2c_sensor_low_power_mode(uint32_t _sensor_num);
uint32_t    i2c_sensor_get_curr_sensor(void);
void        i2c_sensor_set_curr_sensor(uint32_t i);
uint32_t    i2c_sensor_get_num_sensors(void);
void        i2c_sensor_set_num_sensors(uint32_t num);
uint32_t    i2c_sensor_get_address(uint32_t _address);
void        i2c_sensor_set_address(uint32_t i, uint32_t address);

uint32_t	i2c_sensor_get_data( void );
void       *i2c_sensor_init(void);

#endif /* SENSOR_INC_I2C_SENSOR_H_ */
