/*
 * test_prod.h
 *
 *  Created on: 8 april 2020
 *      Author: Sergio Millán López
 */

#ifndef TEST_PROD_H_
#define TEST_PROD_H_

#include "stm32u5xx_hal.h"
#include "main.h"
#include "params.h"
#include "serial_mbus.h"

#define PORT_TEST_LED     (UC_LED_BLUE_GPIO_Port)
#define PIN_TEST_LED      (UC_LED_BLUE_Pin)

#define PORT_TEST_SPI  	  (UC_SPI_MEMORY_ENABLE_GPIO_Port)
#define PIN_TEST_SPI      (UC_SPI_MEMORY_ENABLE_Pin)

#define PORT_TEST_MODBUS  (UC_MODBUS_ENABLE_GPIO_Port)
#define PIN_TEST_MODBUS   (UC_MODBUS_ENABLE_Pin)

#define PORT_TEST_MBUS    (UC_SMPS_ENABLE_GPIO_Port)
#define PIN_TEST_MBUS     (UC_SMPS_ENABLE_Pin)

#define PORT_TEST_SENSOR  (UC_SENSOR_ENABLE_GPIO_Port)
#define PIN_TEST_SENSOR   (UC_SENSOR_ENABLE_Pin)

void    test_prod_task( void );
void    test_prod_run( uint8_t _run );
uint8_t test_prod_run_on( void );

#endif
