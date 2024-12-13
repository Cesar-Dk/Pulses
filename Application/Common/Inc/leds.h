/**
  ******************************************************************************
  * @file           leds.h
  * @author 		Datakorum Development Team
  * @brief          Header file for leds.c.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * leds.h
 *
 *  Created on: 16 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_USER_COMMON_INC_LEDS_H_
#define APPLICATION_USER_COMMON_INC_LEDS_H_

#include "main.h"

#define  CHECK_VBACKUP	(HAL_GPIO_ReadPin( UC_AC_DETECTION_GPIO_Port, UC_AC_DETECTION_Pin ))
#define  CHECK_ALARM	(HAL_GPIO_ReadPin( UC_PULSES_TAMP_GPIO_Port,  UC_PULSES_TAMP_Pin ))

typedef enum
{
  LED_BLUE = 0,
  LED_WHITE,
  LED_GREEN,
  LED_LAST = 0xFF
} Led_TypeDef;

#define LEDn	(3)

#define LEDBLUE_PIN                           GPIO_PIN_0
#define LEDBLUE_GPIO_PORT                     GPIOB
#define LEDWHITE_PIN                          //GPIO_PIN_0
#define LEDWHITE_GPIO_PORT                    //GPIOH
#define LEDGREEN_PIN                          //GPIO_PIN_1
#define LEDGREEN_GPIO_PORT                    //GPIOH

#define LEDBLUE_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOB_CLK_ENABLE()
#define LEDBLUE_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOB_CLK_DISABLE()
#define LEDWHITE_GPIO_CLK_ENABLE()            //__HAL_RCC_GPIOA_CLK_ENABLE()
#define LEDWHITE_GPIO_CLK_DISABLE()           //__HAL_RCC_GPIOA_CLK_DISABLE()
#define LEDGREEN_GPIO_CLK_ENABLE()            //__HAL_RCC_GPIOA_CLK_ENABLE()
#define LEDGREEN_GPIO_CLK_DISABLE()           //__HAL_RCC_GPIOA_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__)     	  do { if((__LED__) == LED_BLUE)  { LEDBLUE_GPIO_CLK_ENABLE();  } else \
                                                   if((__LED__) == LED_WHITE) { LEDWHITE_GPIO_CLK_ENABLE(); } else \
											       if((__LED__) == LED_GREEN) { LEDGREEN_GPIO_CLK_ENABLE(); } } while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)    	  do { if((__LED__) == LED_BLUE)  { LEDBLUE_GPIO_CLK_DISABLE();  } else \
        									       if((__LED__) == LED_WHITE) { LEDWHITE_GPIO_CLK_DISABLE(); } else \
											       if((__LED__) == LED_GREEN) { LEDGREEN_GPIO_CLK_DISABLE(); } } while(0)

typedef enum {
	NB_Y,
	NB_N,
	SSL_Y,
	SSL_N,
	NB_MODULE_ERROR,
	LOW_POWER,
	TEST_MODULE,
	TEST_MODULE_ERROR_1,
	TEST_MODULE_ERROR_2,
	TEST_MODULE_ERROR_3,
	TEST_MODULE_ERROR_4,
	TEST_MODULE_ERROR_5,
	TEST_MODULE_ERROR_6,
	NB_LOCAL_TOOL
}
my_nb_event;

typedef enum {
	METER_RD_WR,
	METER_LOCAL_TOOL,
	METER_ERROR,
	METER_MODULE_ERROR,
	METER_LOW_POWER,
	METER_TEST_MODULE,
	METER_TEST_MODULE_ERROR_1,
	METER_TEST_MODULE_ERROR_2,
	METER_TEST_MODULE_ERROR_3,
	METER_TEST_MODULE_ERROR_4,
	METER_TEST_MODULE_ERROR_5,
	METER_TEST_MODULE_ERROR_6
}
my_meter_event;

uint8_t leds_device_switch_on( void );
void    leds_set_device_switch_on( uint8_t _device_switch_on);
void    leds_LED_Init(Led_TypeDef Led);
void    leds_LED_Init_Read(Led_TypeDef Led);
void    leds_LED_DeInit(Led_TypeDef Led);
void    leds_LED_On(Led_TypeDef Led);
void    leds_LED_Off(Led_TypeDef Led);
void    leds_LED_Toggle(Led_TypeDef Led);
void    leds_init( void );
void    leds_set_NET_status( my_nb_event event );
void    leds_set_METER_status( my_meter_event event );
uint32_t leds_get_leds_enabled( void );
void     leds_FSM( void );

#endif /* APPLICATION_USER_COMMON_INC_LEDS_H_ */
