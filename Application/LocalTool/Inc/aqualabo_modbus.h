/*
 * aqualabo_modbus.h
 *
 *  Created on: 23 ene. 2020
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_LOCALTOOL_INC_AQUALABO_MODBUS_H_
#define APPLICATION_LOCALTOOL_INC_AQUALABO_MODBUS_H_

#include <stdint.h>
#include <math.h>

#include "params.h"

#ifdef UNE82326
#define ENABLE_AQUALABO_PWR_SUPPLY  	{ serial_une82326_pwr_enable();  }
#define ENABLE_AQUALABO_SENSOR      	{ serial_une82326_sel();         }
#define DISABLE_AQUALABO_PWR_SUPPLY 	{ serial_une82326_pwr_disable(); }
#define DISABLE_AQUALABO_SENSOR     	{ serial_une82326_unsel();       }
#endif
#ifdef MBUS
#define PORT_UNE_SEL					( GPIOA )
#define PIN_UNE_SEL        				( GPIO_PIN_9 )
#define PORT_UNE_PWR_ENABLE				( GPIOA )
#define PIN_UNE_PWR_ENABLE      		( GPIO_PIN_15 )

#define UNE_SEL()  						HAL_GPIO_WritePin( PORT_UNE_SEL, PIN_UNE_SEL, GPIO_PIN_SET )
#define UNE_UNSEL()   					HAL_GPIO_WritePin( PORT_UNE_SEL, PIN_UNE_SEL, GPIO_PIN_RESET )
#define UNE_PWR_ENABLE()  				HAL_GPIO_WritePin( PORT_UNE_PWR_ENABLE, PIN_UNE_PWR_ENABLE, GPIO_PIN_SET )
#define UNE_PWR_DISABLE()   			HAL_GPIO_WritePin( PORT_UNE_PWR_ENABLE, PIN_UNE_PWR_ENABLE, GPIO_PIN_RESET )

#define ENABLE_AQUALABO_PWR_SUPPLY  	{ UNE_PWR_ENABLE();  }
#define ENABLE_AQUALABO_SENSOR      	{ UNE_SEL();         }
#define DISABLE_AQUALABO_PWR_SUPPLY 	{ UNE_PWR_DISABLE(); }
#define DISABLE_AQUALABO_SENSOR     	{ UNE_UNSEL();       }
#endif

void     aqualabo_set_comm_state( uint8_t _comm_state );
uint8_t  aqualabo_get_comm_state( void );
uint8_t  aqualabo_modbus_available( void );
void     aqualabo_modbus_sensor_disable( void );
uint8_t  aqualabo_modbus_send( uint8_t *data );
void     aqualabo_modbus_receive( uint8_t *data, uint32_t n );
uint8_t  aqualabo_get_aqualabo_offset( void );
int32_t  aqualabo_modbus_get( uint8_t i, float_t * value );
float_t *aqualabo_modbus_get_raw_data( void );
void     aqualabo_modbus_set_get_data( uint8_t _get_data );
uint32_t aqualabo_modbus_enable( uint8_t _enable );
uint32_t aqualabo_modbus_get_enable( void );

#endif /* APPLICATION_LOCALTOOL_INC_AQUALABO_MODBUS_H_ */
