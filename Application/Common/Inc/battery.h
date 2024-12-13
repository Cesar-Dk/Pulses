/*
 * battery.h
 *
 *  Created on: 29 ene. 2021
 *      Author: smill
 */

#ifndef COMMON_INC_BATTERY_H_
#define COMMON_INC_BATTERY_H_


#include "main.h"

#define MAX_VOLTAGE (3600)

uint32_t battery_get_Vdda( void );
float_t  battery_get_Vinst( void );
void     battery_set_Vdda( uint32_t _Vdda );
uint32_t battery_get_Vbat(void);
void     battery_voltage_meas( uint32_t no_printf );

#endif /* COMMON_INC_BATTERY_H_ */
