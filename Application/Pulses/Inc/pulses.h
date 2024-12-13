/**
  ******************************************************************************
  * @file           pulses.h
  * @author 		Datakorum Development Team
  * @brief          Header file for pulses.c.
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
 * pulses.h
 *
 *  Created on: 3 dic. 2019
 *      Author: Sergio Millán López
 */

#ifndef APPLICATION_PULSES_INC_PULSES_H_
#define APPLICATION_PULSES_INC_PULSES_H_

#include "stm32u5xx_hal.h"
#include "params.h"

/**
 * @ingroup System_Pulses
 * @{
 *
 */

#define  PULSE_CHECK_DIRECTION	(HAL_GPIO_ReadPin( (GPIOB), (GPIO_PIN_5)  ))
#define  CH2C_CHECK_DIRECTION	(HAL_GPIO_ReadPin( (GPIOC), (GPIO_PIN_15) ))

#define REVERSE 		 (0)
#define FORWARD 		 (1)

#define COMPENSATION_OFF (0)
#define COMPENSATION_ON  (1)

#define PULSE_RX_OFF 	 (0)
#define PULSE_RX_ON 	 (1)

typedef void (*int_handler_func)(void);

/**
 * @struct Pulses structure
 * @brief Holds all pulses readings information
 *
 */
typedef struct {
	uint32_t init;
	uint32_t forward_ch1p;
	uint32_t reverse_ch1p;
	uint32_t direction_ch1d;
	uint32_t forward_ch1d;
	uint32_t backflow_compensated_ch2p;
	uint32_t compensation_ch2c;
	uint32_t litres_forward;
	uint32_t litres_reverse;
	uint32_t litres_compensated;
	uint32_t c_flag;
	uint32_t last_device;
	uint32_t write_record;
	uint32_t pulse_rx;
	uint32_t ch1p_pulse_rx;
	uint32_t ch2p_pulse_rx;
	uint32_t ch1d_pulse_rx;
	uint32_t ch2c_pulse_rx;
	uint32_t frame_type;
	uint32_t interruption;
	uint32_t mutex;
	uint32_t input_num;
	char     crc[5];
} pulses;


void     pulses_init( void );
void     pulses_deinit( void );
uint8_t  pulses_get_init( void );
uint32_t pulses_get_input_num( void );
void     pulses_set_input_num( uint32_t _input_num );
uint32_t pulses_get_pulse_ch2p_rx( void );
uint32_t pulses_get_pulse_ch1p_rx( void );
uint32_t pulses_get_interruption( void );
void     pulses_set_interruption( uint32_t _interruption );
uint32_t pulses_get_mutex( void );
void     pulses_set_mutex( uint32_t _mutex );
uint8_t  pulses_get_last_device( void );
void     pulses_set_last_device( uint8_t _last_device );
uint8_t  pulses_get_write_record( void );
void     pulses_set_write_record( uint8_t _write_record );
uint32_t pulses_get_litres_forward( void );
uint32_t pulses_get_litres_reverse( void );
uint32_t pulses_get_litres_compensated( void );
uint32_t pulses_get_backflow_compensated_2p( void );
void     pulses_set_backflow_compensated_2p( uint32_t _num_pulses );
uint32_t pulses_get_ch1p( void );
void     pulses_set_ch1p( uint32_t _num_pulses );
uint32_t pulses_get_ch1d( void );
void     pulses_set_ch1d( uint32_t _num_pulses );
uint32_t pulses_get_pulse_rx( void );
void     pulses_set_pulse_rx( uint32_t _pulse_on );
uint32_t pulses_get_frame_type( void );
void     pulses_set_frame_type( uint32_t _frame_type );
char    *pulses_get_crc( void );
void     pulses_set_crc( char * crc);
void     pulses_ch1p_handler( void );
void     pulses_ch1d_handler( void );
void     pulses_ch2p_handler( void );
void     pulses_ch2c_handler( void );
void     pulses_tamper_handler( void );
uint32_t pulses_int_handler( GPIO_TypeDef *int_port, uint16_t num_pin, int_handler_func int_handler );
uint32_t pulses_int_2_handler( GPIO_TypeDef *int_port, uint16_t num_pin, int_handler_func int_handler );
void     pulses_build_typeM_telegram( char *telegram_1, uint32_t *num_dig );

#endif /* APPLICATION_PULSES_INC_PULSES_H_ */

/**
 * @}
 */
