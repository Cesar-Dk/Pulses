/**
  ******************************************************************************
  * @file           modbus.c
  * @author 		Datakorum Development Team
  * @brief          Header file for modbus.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * modbus.h
 *
 *  Created on: 19 oct. 2019
 *      Author: smill
 */

#ifndef APPLICATION_LOCALTOOL_INC_MODBUS_H_
#define APPLICATION_LOCALTOOL_INC_MODBUS_H_

/*
 * modbus.h
 *
 *  Created on: 2 jun. 2019
 *      Author: smill
 */

#ifndef MODBUS_H_
#define MODBUS_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32u5xx_hal.h"
#include "params.h"

#include "serial_modbus.h"

/**
 * @addtogroup System_Modbus System Modbus
 * @{
 *
 */

#define MODBUS_SESSION_TIME (180)//(60)//(900)

#define MODBUS_SESSION_INIT 		 (0)	/*!< init */
#define MODBUS_SESSION_END  		 (1)	/*!< */
#define MODBUS_SESSION_PARAM_CHANGE  (2)	/*!< */
#define MODBUS_SESSION_SEND_TELEGRAM (3)	/*!< */
/**
 * @}
 * */
void     modbus_vars_init( void );
void     modbus_init( void );
void     modbus_deInit( void );
char   * modbus_get_tool_cert( void );
void     modbus_set_tool_cert( char *tool_cert );
uint8_t  modbus_get_end_session( void );
uint32_t modbus_get_timeout_modbus( void );
uint8_t  modbus_get_send( void );
void     modbus_set_send( uint8_t _modbus_send );
uint8_t  modbus_get_send_lock( void );
void     modbus_set_send_lock( uint8_t _modbus_send_lock );
uint32_t modbus_get_local_tool_started( void );
void     modbus_set_local_tool_started( uint32_t _modbus_local_tool_started );
uint32_t modbus_rcx( void );
void     modbus_trx( void );
uint32_t modbus_local_tool_task( void );
unsigned short modbus_test_as11_get( void );
void 	 modbus_test_trx( void );
void 	 modbus_test_rcx( void );

#endif /* MODBUS_H_ */


#endif /* APPLICATION_LOCALTOOL_INC_MODBUS_H_ */
