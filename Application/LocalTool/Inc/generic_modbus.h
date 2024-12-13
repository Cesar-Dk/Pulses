/**
  ******************************************************************************
  * @file           generic_modbus.h
  * @author 		Datakorum Development Team
  * @brief          Header file for generic_modbus.c file.
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
 * generic_modbus.h
 *
 *  Created on: 9 mar. 2020
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_LOCALTOOL_INC_GENERIC_MODBUS_H_
#define APPLICATION_LOCALTOOL_INC_GENERIC_MODBUS_H_

#include <stdint.h>
#include <math.h>

#include "params.h"

/**
 * @addtogroup System_Modbus_Generic
 * @{
 * */

#define MAX_SLAVES   (32)//(11)//(20)//
#define MAX_COMMANDS (30)//(5)//(4)
#define MAX_VALUES   (50)//(64)//(123)

/**
 * @enum
 * @brief Generic RS485 Status.
 *
 */
typedef enum
{
	DISABLED = 0,	/**< DISABLED */
	ENABLED  = 1 	/**< ENABLED */
} generic_485_status;

/**
 * @enum Generic RS485 communication state.
 * @brief
 *
 */
typedef enum
{
	COMMUNICATON_DISABLED = 0,	/**< COMMUNICATON_DISABLED */
	START_COMMUNICATION   = 1,	/**< START_COMMUNICATION */
	END_COMMUNICATION     = 2 	/**< END_COMMUNICATION */
} generic_485_comm_state;

/**
 * @enum
 * @brief Generic RS485 output type
 *
 */
typedef enum
{
    RAW = 0,              		/**< RAW */
	MODBUS_DECODED = 1,      	/**< MODBUS_DECODED */
	MODBUS_RAW = 2,          	/**< MODBUS_RAW */
	MODBUS_RAW_DLT645 = 3,      /**< MODBUS_RAW_DLT645 */
	LAST_GENERIC_485_TYPE = 4	/**< LAST_GENERIC_485_TYPE */
} generic_485_type;

/**
 * @struct
 * @brief Structure definition of Generic 485 object.
 *
 */
typedef struct _generic_485_
{
    uint8_t  type;		/*!< MBUS device type
    							@arg 1 - krone.
    							@arg 2 - Aqualabo.
    							@arg 3 - 4/20mA.
    							*/
    uint8_t  dtl645_on;
    uint32_t device; 	/*!< sensor id field, see @ref Modbus for more information */
    union
    {
        struct
        {
            struct
            {
                uint32_t speed; /*!< serial port baud rate configuration */
                uint8_t  mode; 	/*!< @arg 8N1
                 	 	 	 	 	 @arg 9N1
                 	 	 	 	 */
            } serial;

            uint8_t  enable;		/*!< */
            uint8_t  comm_state;	/*!< */
            uint8_t  slave;		    /*!< */
            uint8_t  address_645[12];/*!< */
            uint8_t  attempt;		/*!< */
            uint8_t  tx_on;			/*!< */
            uint32_t read_time;		/*!< */
            uint32_t send_time;		/*!< */

            struct
            {
                uint8_t  function;				/*!< */
                uint16_t addr;					/*!< */
                uint16_t quantity;				/*!< */
                uint16_t values[MAX_VALUES];	/*!< */
                uint32_t data_id_645; 		    /*!< Period to send
                									@arg 1 - only send once
                								 */
                struct
                {
                    uint8_t available;				/*!< */
                    uint8_t error_code; 			/*!< */
                    uint8_t on_demand;              /*!< */
                    uint16_t values[MAX_VALUES];	/*!< */
                } response;
            } command[MAX_COMMANDS];
        } modbus;
    };
}generic485_st;

void      generic_485_init( void );
generic485_st *generic_485_get_generic_485(void);
size_t    generic_485_get_size_of_generic_485(void);
uint8_t   generic_485_get_enable( void );
void      generic_485_set_enable( uint8_t _enable );
void      generic_485_set_tx_on(uint8_t _tx_on);
uint8_t   generic_485_get_comm_state( void );
void      generic_485_set_comm_state( uint8_t _comm_state );
void      generic_485_set_device( uint32_t _device );
void      generic_485_set_slave_id_reg( uint8_t _slave_id, uint32_t reg );
void      generic_485_set_address_645( uint8_t _bcd, uint32_t pos, uint8_t slave );
void      generic_485_set_data_id_645(uint32_t slave, uint32_t _data_id, uint32_t reg);
void      generic_485_set_function_reg( uint32_t slave, uint8_t _function, uint32_t reg );
uint8_t   generic_485_get_function_slave_reg( uint32_t slave, uint32_t reg );
void      generic_485_set_addr_reg( uint32_t slave, uint16_t _addr, uint32_t reg );
uint16_t  generic_485_get_address_slave_reg( uint32_t slave, uint32_t reg );
void      generic_485_set_quantity_reg( uint32_t slave, uint16_t _quantity, uint32_t reg );
void 	  generic_485_set_value_reg( uint32_t slave, uint16_t _value, uint32_t reg, uint32_t value_num );
uint8_t   generic_485_get_quantity_slave_reg( uint32_t slave, uint32_t reg );
uint8_t   generic_485_get_slave_id(void);
uint8_t   generic_485_get_function(void);
uint16_t  generic_485_get_addr(void);
uint16_t  generic_485_get_quantity(void);
uint32_t  generic_485_get_send_time( void );
void      generic_485_set_send_time( uint32_t _send_time );
uint32_t  generic_485_get_read_time( void );
void      generic_485_set_read_time( uint32_t _read_time );
uint32_t  generic_485_get_type( void );
uint32_t  generic_485_get_dtl645( void );
void      generic_485_set_type( uint32_t _type );
void      generic_485_set_type_and_disable( uint32_t _type );
void      generic_485_set_dtl645( uint8_t _on );
void      generic_485_set_slave_id_type( uint8_t slave_id, uint32_t _type );
uint8_t   generic_485_get_is_on_demand(uint8_t __command);
uint8_t   generic_485_set_is_on_demand(uint8_t __command, uint8_t __on_demand);
uint8_t   generic_485_available( void );
uint32_t  generic_485_send( uint8_t *data );
uint32_t  generic_485_set_alarm_message( uint8_t *data, uint32_t n ) ;
void      generic_485_receive( uint8_t *data, uint32_t n );

//uint16_t *generic_485_get_raw_modbus( uint16_t *quantity );
uint16_t *generic_485_get_raw_modbus( uint16_t *quantity, uint16_t *buff, uint32_t slave_num, uint32_t *slave_id, uint8_t *on_demand );
uint8_t  *generic_485_get_raw_dlt645( uint16_t *quantity, uint8_t *buff, uint32_t slave_num, uint8_t *slave_id );

void      generic_485_reset_on_demand( void );
#endif /* APPLICATION_LOCALTOOL_INC_GENERIC_MODBUS_H_ */

/**
 * @}
 * */
