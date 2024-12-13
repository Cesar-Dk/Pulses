/**
  ******************************************************************************
  * @file           leds.c
  * @author 		Datakorum Development Team
  * @brief          LED driver. This file provides APIs to handle internal use
  * of LEDs.
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
 * leds.c
 *
 *  Created on: 16 jul. 2019
 *      Author: smill
 */
#include "leds.h"
#include "tick.h"
#include "shutdown.h"
#include "params.h"
#include "test_prod.h"

//#define  CHECK_LEDS		(HAL_GPIO_ReadPin( GPIOH, UC_LED_GREEN_Pin ))
#define  CHECK_TAMPER	(HAL_GPIO_ReadPin( GPIOA, GPIO_PIN_0 ))
#define  CHECK_REED  	(HAL_GPIO_ReadPin( GPIOC, GPIO_PIN_13 ))

#define LED_BLUE_STATUS  (0)
#define LED_WHITE_STATUS (1)
#define LED_GREEN_STATUS (2)

GPIO_TypeDef  *GPIO_PORT[LEDn] = {LEDBLUE_GPIO_PORT, LEDBLUE_GPIO_PORT, LEDBLUE_GPIO_PORT};//{LEDBLUE_GPIO_PORT, LEDWHITE_GPIO_PORT, LEDGREEN_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn]  = {LEDBLUE_PIN,       LEDBLUE_PIN,       LEDBLUE_PIN      };//{LEDBLUE_PIN, LEDWHITE_PIN, LEDGREEN_PIN};

static enum {
	OFF,
	BLINK,
	ON,
	ERROR_BLINK,
	LOW_POWER_OFF,
	TEST_MODE,
	TEST_MODE_ERROR_1,
	TEST_MODE_ERROR_2,
	TEST_MODE_ERROR_3,
	TEST_MODE_ERROR_4,
	TEST_MODE_ERROR_5,
	TEST_MODE_ERROR_6
} status[3] = { LOW_POWER_OFF, LOW_POWER_OFF, LOW_POWER_OFF };

uint8_t device_switch_on = 0;

extern uint32_t tamper_on, tamper_sending, vbackup_on, vbackup_sending, vbackup_int;
extern uint32_t alarm_on,  alarm_sending,  alarm_on,   alarm_sending,   alarm_int;

/**
 * @defgroup System_leds System Leds
 *
 * @{
 */


uint8_t leds_device_switch_on( void )
{
	return device_switch_on;
}

/**
  * @brief  Flag to
  * @param _device_switch_on:
  * 		@arg 0 ->
  * 		@arg 1 ->
  * @retval None
  */
void leds_set_device_switch_on( uint8_t _device_switch_on)
{
	device_switch_on = _device_switch_on;
}

/**
  * @brief  Configure LED GPIO.
  * @param  Led: LED to be configured.
  *         This parameter can be one of the following values:
  *            @arg  @ref Led_TypeDef
  * @retval None
  */
void leds_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  if ((Led == LED_GREEN) || (Led == LED_WHITE))
  {
	  return;
  }

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin   = GPIO_PIN[Led];
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}

void leds_LED_Init_Read(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct = {0};

  if ((Led == LED_GREEN) || (Led == LED_WHITE))
  {
	  return;
  }

  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin   = GPIO_PIN[Led];
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}
/**
  * @brief  DeInitialize LED GPIO.
  * @param  Led: LED to be deinitialized.
  *         This parameter can be one of the following values:
  *            @arg  @ref Led_TypeDef
  * @note BSP_LED_DeInit() does not disable the GPIO clock
  * @retval None
  */
void leds_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if ((Led == LED_GREEN) || (Led == LED_WHITE))
  {
	  return;
  }

  /* Turn off LED */
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  HAL_GPIO_DeInit(GPIO_PORT[Led], GPIO_InitStruct.Pin);
}

/**
  * @brief  Initialize LEDs.
  * @retval None
  */
void leds_init( void )
{
	leds_LED_Init(LED_BLUE);
#if 0
	leds_LED_Init(LED_WHITE);
	leds_LED_Init(LED_GREEN);
#endif
#if defined(PWR_STDBY)
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
	{ // Led on one second only when switch on the device.
		leds_set_device_switch_on(1);
		leds_LED_On(LED_BLUE);
	}
	else
	{
		leds_LED_Off(LED_BLUE);
	}
#elif defined(PWR_STOP)
	static uint8_t init = 0;
	if ((0 == init)
	/* Standby flag and software reset flag are 0 */
#ifndef DEBUG_MODE
	&& (( __HAL_PWR_GET_FLAG(PWR_FLAG_SBF) == RESET ) && (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) == RESET )))
#else
	&& (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)))
#endif
	{
		init = 1;
		leds_set_device_switch_on(1);
		leds_LED_On(LED_BLUE);
	    HAL_Delay( 200 );
		leds_LED_Off(LED_BLUE);
	}
	else if (__HAL_PWR_GET_FLAG(PWR_FLAG_SBF) != RESET)
	{
		init = 1;
	}
#endif

	leds_set_NET_status(LOW_POWER);
	leds_set_METER_status(METER_LOW_POWER);
}

/**
  * @brief  Turn selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *            @arg  @ref Led_TypeDef
  * @retval None
  */
void leds_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
  * @brief  Turn selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *            @arg  @ref Led_TypeDef
  * @retval None
  */
void leds_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  Toggle the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *            @arg  @ref Led_TypeDef
  * @retval None
  */
void leds_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

static uint8_t test_error_led;
uint32_t leds_enabled = 0;
uint32_t leds_get_leds_enabled( void )
{
	return leds_enabled;
}

void leds_FSM( void )
{
    static unsigned char toggle = 0, slow_toggle = 0, boot_t = 21;
    static uint32_t elapsed, elapsed_tamper;
    static uint32_t elapsed_ini, elapsed_tamper_ini;
#ifndef DEBUG_MODE
    static uint32_t elapsed_vbackup_ini, vbackup_bounce = 0;
#endif
//    static uint32_t elapsed_alarm_ini, alarm_bounce = 0;

    (void)elapsed_tamper;
    if ( CHECK_TAMPER == 0 ) {
    	tamper_on      = 1;
    	elapsed_tamper = 0;
    	elapsed_tamper_ini = Tick_Get( SECONDS );
    } else if ( CHECK_TAMPER == 1 ) {
    	tamper_on   = 0;
    	elapsed     = 600;
    	elapsed_ini = Tick_Get( SECONDS ) - 61;
//    	if (elapsed_tamper++>600) {
    	if ( Tick_Get( SECONDS ) > elapsed_tamper_ini) {
    		if ( ( Tick_Get( SECONDS ) - elapsed_tamper_ini) > 60 ) {
    			shutdown_set_tamper_sending(0);
    		}
    	}
    }

#ifndef DEBUG_MODE
    if (params_manteinance_reed_activation() != 2)
    {
    	if ( CHECK_VBACKUP == 0 )
    	{
    		if (0 == vbackup_on)
    		{
    			if (vbackup_bounce++>20)
    			{
    				vbackup_bounce = 0;
    				vbackup_on  = 1;
    				vbackup_int = 1;
    				if ( shutdown_get_vbackup_sending() != 2 ) {
    					vbackup_sending = 1;
    				}
    			}
    		}
    		elapsed_vbackup_ini = Tick_Get( SECONDS );
    	}
    	else if ( CHECK_VBACKUP == 1 )
    	{
    		if (1 == vbackup_on)
    		{
    			vbackup_int = 1;
    			if ( shutdown_get_vbackup_sending() != 4 )
    			{
    				vbackup_sending = 3;
    			}
    			if (1 == params_get_mqtt_dc_on())
    			{
    				shutdown_set_start_count(0);
    			}
    		}
    		vbackup_on   			  = 0;
    		if ( Tick_Get( SECONDS ) > elapsed_vbackup_ini)
    		{
    			if ( ( Tick_Get( SECONDS ) - elapsed_vbackup_ini) > 60 )
    			{
    				shutdown_set_vbackup_sending(0);
    			}
    			if (1 == params_get_mqtt_dc_on())
    			{
    				shutdown_set_start_count(0);
    			}
    		}
    	}
    }
#endif
#if 0
#define ALARM_ON  (0)
#define ALARM_OFF (1)
    if ( 1 == params_get_input_alarm_sensor() )
    {
    	if ( CHECK_ALARM == ALARM_ON )
    	{
    		if (0 == alarm_on)
    		{
    			if (alarm_bounce++>20)
    			{
    				alarm_bounce = 0;
    				alarm_on  = 1;
    				alarm_int = 1;
    				if ( shutdown_get_alarm_sending() != 2 )
    				{
    					alarm_sending = 1;
    				}
    			}
    		}
    		elapsed_alarm_ini = Tick_Get( SECONDS );
    	}
    	else if ( CHECK_ALARM == ALARM_OFF )
    	{
    		if (1 == alarm_on)
    		{
    			alarm_int = 1;
    			if ( shutdown_get_alarm_sending() == 3 )
    			{
    				alarm_sending = 4;
    				alarm_on      = 0;
    			}
//    			if (1 == params_get_mqtt_dc_on())
//    			{
//    				shutdown_set_start_count(0);
//    			}
    		}
//    		alarm_on   			  = 0;
//    		if ( Tick_Get( SECONDS ) > elapsed_alarm_ini)
//    		{
//    			if ( ( Tick_Get( SECONDS ) - elapsed_alarm_ini) > 60 )
//    			{
//    				shutdown_set_alarm_sending(0);
//    			}
//    			if (1 == params_get_mqtt_dc_on())
//    			{
//    				shutdown_set_start_count(0);
//    			}
//    		}
    	}
    }
#endif

    if ( CHECK_REED == 1 ) {
    	asm("nop");
    }

    if ( 0 == leds_enabled ) {
    	if ( tamper_on == 1 ) {
    		if ( CHECK_TAMPER == 0 ) {
    			elapsed      = 0;
    			elapsed_ini  = Tick_Get( SECONDS );
    			leds_enabled = 1;
    		} else {
    			tamper_on = 0;
    		}
    	}
    } else if ( 1 == leds_enabled ) {
    	elapsed++;
    	leds_LED_Init(LED_GREEN);
    	leds_LED_Init(LED_BLUE);
//    	leds_LED_Init(LED_WHITE);
//    	if ( elapsed > 600 ) {//TODO: Instead of elapsed, with diff TickGet.
    	if ( Tick_Get( SECONDS ) > elapsed_ini) {
    		if ( ( Tick_Get( SECONDS ) - elapsed_ini ) > 60 ) {
    			leds_enabled = 0;
    			tamper_on    = 0;
//    		    shutdown_set_tamper_sending(0);
    			if ( 0 == test_prod_run_on() ) {
    				leds_LED_Init_Read(LED_GREEN);
    				leds_LED_Init_Read(LED_BLUE);
//    			    leds_LED_Init_Read(LED_WHITE);
    			}
    		}
    	}
    } else {
    	if ( 0 == test_prod_run_on() ) {
    		leds_LED_Off(LED_BLUE);
    		leds_LED_Init_Read(LED_GREEN);
    		leds_LED_Init_Read(LED_BLUE);
    	}
    	return;
    }

    if( boot_t ) {
        if( boot_t == 1 ) {
        	leds_LED_Off(LED_BLUE);
//        	leds_LED_Off(LED_WHITE);
        }
        boot_t--;
        return;
    }

    if ( 0 == test_prod_run_on() ) {
    if( status[1] == ERROR_BLINK ) {
    	toggle = toggle ? 0 : 1;
    	if( toggle ) {
//        	leds_LED_Off(LED_WHITE);
        	leds_LED_On(LED_BLUE);
    	}
    	else {
//        	leds_LED_On(LED_WHITE);
        	leds_LED_Off(LED_BLUE);
    	}
    	return;
    }
    }

    if( toggle < 5 ) {
    	toggle++;
    }
    else {
    	toggle = 0;
    }

    if( slow_toggle < 19 ) {
    	slow_toggle++;
    }
	else {
		slow_toggle = 0;
	}

    switch( status[ 0 ])
    {
        case OFF:
            if( slow_toggle < 3 ) {
//            	leds_LED_On(LED_WHITE);
            }
            else {
//            	leds_LED_Off(LED_WHITE);
            }
            break;
        case BLINK:
            if( toggle < 3 ) {
//            	leds_LED_On(LED_WHITE);
            }
            else {
//            	leds_LED_Off(LED_WHITE);
            }
            break;
        case ON:
//        	leds_LED_On(LED_WHITE);
            break;
        case TEST_MODE:
            break;
        case ERROR_BLINK:
        	break;
        case LOW_POWER_OFF:
//        	leds_LED_Off(LED_WHITE);
        	break;
        default:
            break;
    }

    switch( status[ 1 ])
    {
        case OFF:
            if( slow_toggle < 3 ) {
            	leds_LED_On(LED_BLUE);
            }
            else {
            	leds_LED_Off(LED_BLUE);
            }
            break;
        case BLINK:
            if( toggle < 3 ) {
            	leds_LED_On(LED_BLUE);
            }
            else {
            	leds_LED_Off(LED_BLUE);
            }
            break;
        case ON:
        	leds_LED_On(LED_BLUE);
            break;
        case LOW_POWER_OFF:
        	leds_LED_Off(LED_BLUE);
            break;
        case TEST_MODE:
            break;
        case TEST_MODE_ERROR_1:
        case TEST_MODE_ERROR_2:
        case TEST_MODE_ERROR_3:
        case TEST_MODE_ERROR_4:
        case TEST_MODE_ERROR_5:
        case TEST_MODE_ERROR_6:
        	if ( test_error_led & 1 ) {
				if( slow_toggle < 3 ) {
					HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_SET  );
				} else {
					HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_RESET  );
				}
        	}
        	if ( test_error_led & 2 ) {
				if( slow_toggle < 3 ) {
					HAL_GPIO_WritePin( PORT_TEST_SPI, PIN_TEST_SPI, GPIO_PIN_SET );
				} else {
					HAL_GPIO_WritePin( PORT_TEST_SPI, PIN_TEST_SPI, GPIO_PIN_RESET );
				}
        	}
        	if ( test_error_led & 4 ) {
				if( slow_toggle < 3 ) {
					HAL_GPIO_WritePin( PORT_TEST_MODBUS, PIN_TEST_MODBUS, GPIO_PIN_SET  );
				} else {
					HAL_GPIO_WritePin( PORT_TEST_MODBUS, PIN_TEST_MODBUS, GPIO_PIN_RESET  );
				}
        	}
        	if ( test_error_led & 8 ) {
				if( slow_toggle < 3 ) {
					HAL_GPIO_WritePin( PORT_TEST_MBUS, PIN_TEST_MBUS, GPIO_PIN_SET  );
				} else {
					HAL_GPIO_WritePin( PORT_TEST_MBUS, PIN_TEST_MBUS, GPIO_PIN_RESET  );
				}
        	}
        	if ( test_error_led & 16 ) {
				if( slow_toggle < 3 ) {
//					HAL_GPIO_WritePin( PORT_TEST_EURIDIS, PIN_TEST_EURIDIS, GPIO_PIN_SET  );
					HAL_GPIO_WritePin( PORT_TEST_MBUS, PIN_TEST_MBUS, GPIO_PIN_SET  );
				} else {
//					HAL_GPIO_WritePin( PORT_TEST_EURIDIS, PIN_TEST_EURIDIS, GPIO_PIN_RESET  );
					HAL_GPIO_WritePin( PORT_TEST_MBUS, PIN_TEST_MBUS, GPIO_PIN_RESET  );
				}
        	}
        	if ( test_error_led & 32 ) {
				if( slow_toggle < 3 ) {
					HAL_GPIO_WritePin( PORT_TEST_SPI, PIN_TEST_SPI, GPIO_PIN_SET  );
					HAL_GPIO_WritePin( PORT_TEST_MODBUS, PIN_TEST_MODBUS, GPIO_PIN_SET  );
				} else {
					HAL_GPIO_WritePin( PORT_TEST_SPI, PIN_TEST_SPI, GPIO_PIN_RESET  );
					HAL_GPIO_WritePin( PORT_TEST_MODBUS, PIN_TEST_MODBUS, GPIO_PIN_RESET  );
				}
        	}
            break;
        default:
            break;
    }

    switch( status[ LED_GREEN_STATUS ])
    {
        case OFF:
            if( slow_toggle < 3 ) {
//            	leds_LED_On(LED_GREEN);
            }
            else {
//            	leds_LED_Off(LED_GREEN);
            }
            break;
        case BLINK:
            if( toggle < 3 ) {
//            	leds_LED_On(LED_GREEN);
            }
            else {
//            	leds_LED_Off(LED_GREEN);
            }
            break;
        case ON:
//        	leds_LED_On(LED_GREEN);
            break;
        case TEST_MODE:
            break;
        case ERROR_BLINK:
        	break;
        case LOW_POWER_OFF:
//        	leds_LED_Off(LED_GREEN);
        	break;
        default:
            break;
    }
}

/*  void LEDs_set_net_change( my_wifi_event event )
 *          OFF    BLINK   ON
 * WIFI_Y   blink   -       -
 * WIFI_N   -       off     off
 * MQTT_Y   on      on      -
 * MQTT_N   -       -       blink
 */

void leds_set_NET_status( my_nb_event event )
{
    if( event == NB_MODULE_ERROR ) {
    	status[ 1 ] = ERROR_BLINK;
    	return;
    }

    if( event == TEST_MODULE ) {
    	status[1] = TEST_MODE;
    } else if( event == TEST_MODULE_ERROR_1 ) {
    	status[1] = TEST_MODE_ERROR_1;
    } else if( event == TEST_MODULE_ERROR_2 ) {
    	status[1] = TEST_MODE_ERROR_2;
    } else if( event == TEST_MODULE_ERROR_3 ) {
    	status[1] = TEST_MODE_ERROR_3;
    } else if( event == TEST_MODULE_ERROR_4 ) {
    	status[1] = TEST_MODE_ERROR_4;
    } else if( event == TEST_MODULE_ERROR_5 ) {
    	status[1] = TEST_MODE_ERROR_5;
    } else if( event == TEST_MODULE_ERROR_6 ) {
    	status[1] = TEST_MODE_ERROR_6;
    } else if( event == NB_LOCAL_TOOL ) {
    	status[1] = BLINK;
    }

	switch( status[ 1 ])
    {
		case LOW_POWER_OFF:
        case OFF:
        case ERROR_BLINK:
            if( event == NB_Y ) {
            	status[1] = BLINK;
            } else if( event == NB_N ) {
            	status[1] = OFF;
            } else if( event == SSL_Y ) {
            	status[1] = ON;
            } else if( event == LOW_POWER ) {
            	status[1] = LOW_POWER_OFF;
            } else if( event == TEST_MODULE ) {
            	status[1] = TEST_MODE;
            } else if( event == TEST_MODULE_ERROR_1 ) {
            	status[1] = TEST_MODE_ERROR_1;
            } else if( event == TEST_MODULE_ERROR_2 ) {
            	status[1] = TEST_MODE_ERROR_2;
            } else if( event == TEST_MODULE_ERROR_3 ) {
            	status[1] = TEST_MODE_ERROR_3;
            } else if( event == TEST_MODULE_ERROR_4 ) {
            	status[1] = TEST_MODE_ERROR_4;
            } else if( event == TEST_MODULE_ERROR_5 ) {
            	status[1] = TEST_MODE_ERROR_5;
            } else if( event == TEST_MODULE_ERROR_6 ) {
            	status[1] = TEST_MODE_ERROR_6;
            } else if( event == NB_LOCAL_TOOL ) {
            	status[1] = BLINK;
            }
            break;
        case BLINK:
            if( event == NB_N ) {
            	status[1] = OFF;
            } else if( event == SSL_Y ) {
            	status[1] = ON;
            } else if( event == LOW_POWER ) {
            	status[1] = LOW_POWER_OFF;
            } else if( event == TEST_MODULE ) {
            	status[1] = TEST_MODE;
            } else if( event == TEST_MODULE_ERROR_1 ) {
            	status[1] = TEST_MODE_ERROR_1;
            } else if( event == TEST_MODULE_ERROR_2 ) {
            	status[1] = TEST_MODE_ERROR_2;
            } else if( event == TEST_MODULE_ERROR_3 ) {
            	status[1] = TEST_MODE_ERROR_3;
            } else if( event == TEST_MODULE_ERROR_4 ) {
            	status[1] = TEST_MODE_ERROR_4;
            } else if( event == TEST_MODULE_ERROR_5 ) {
            	status[1] = TEST_MODE_ERROR_5;
            } else if( event == TEST_MODULE_ERROR_6 ) {
            	status[1] = TEST_MODE_ERROR_6;
            } else if( event == NB_LOCAL_TOOL ) {
            	status[1] = BLINK;
            }
            break;
        case ON:
            if( event == NB_N ) {
            	status[1] = OFF;
            } else if( event == SSL_N ) {
            	status[1] = BLINK;
            } else if( event == LOW_POWER ) {
            	status[1] = LOW_POWER_OFF;
            } else if( event == TEST_MODULE ) {
            	status[1] = TEST_MODE;
            } else if( event == TEST_MODULE_ERROR_1 ) {
            	status[1] = TEST_MODE_ERROR_1;
            } else if( event == TEST_MODULE_ERROR_2 ) {
            	status[1] = TEST_MODE_ERROR_2;
            } else if( event == TEST_MODULE_ERROR_3 ) {
            	status[1] = TEST_MODE_ERROR_3;
            } else if( event == TEST_MODULE_ERROR_4 ) {
            	status[1] = TEST_MODE_ERROR_4;
            } else if( event == TEST_MODULE_ERROR_5 ) {
            	status[1] = TEST_MODE_ERROR_5;
            } else if( event == TEST_MODULE_ERROR_6 ) {
            	status[1] = TEST_MODE_ERROR_6;
            } else if( event == NB_LOCAL_TOOL ) {
            	status[1] = BLINK;
            }
            break;
        case TEST_MODE:
        case TEST_MODE_ERROR_1:
        case TEST_MODE_ERROR_2:
        case TEST_MODE_ERROR_3:
        case TEST_MODE_ERROR_4:
        case TEST_MODE_ERROR_5:
        case TEST_MODE_ERROR_6:
			if (event == TEST_MODULE) {
				status[1] = TEST_MODE;
			} else if (event == TEST_MODULE_ERROR_1) {
				status[1] = TEST_MODE_ERROR_1;
				test_error_led |= 1;
			} else if (event == TEST_MODULE_ERROR_2) {
				status[1] = TEST_MODE_ERROR_2;
				test_error_led |= 2;
			} else if (event == TEST_MODULE_ERROR_3) {
				status[1] = TEST_MODE_ERROR_3;
				test_error_led |= 4;
			} else if (event == TEST_MODULE_ERROR_4) {
				status[1] = TEST_MODE_ERROR_4;
				test_error_led |= 8;
			} else if (event == TEST_MODULE_ERROR_5) {
				status[1] = TEST_MODE_ERROR_5;
				test_error_led |= 16;
			} else if (event == TEST_MODULE_ERROR_6) {
				status[1] = TEST_MODE_ERROR_6;
				test_error_led |= 32;
			} else if( event == NB_LOCAL_TOOL ) {
            	status[1] = BLINK;
            }

        	break;
        default:
            break;
    }
}

/*  void LEDs_set_net_change( my_wifi_event event )
 *          OFF    BLINK   ON
 * WIFI_Y   blink   -       -
 * WIFI_N   -       off     off
 * MQTT_Y   on      on      -
 * MQTT_N   -       -       blink
 */
void leds_set_METER_status( my_meter_event event )
{
    if( event == METER_MODULE_ERROR ) {
    	status[ LED_GREEN_STATUS ] = ERROR_BLINK;
    	return;
    }

	switch( status[ LED_GREEN_STATUS ] )
    {
		case LOW_POWER_OFF:
        case OFF:
        case ERROR_BLINK:
            if( event == METER_RD_WR ) {
            	status[LED_GREEN_STATUS] = ON;
            } else if( event == METER_LOW_POWER ) {
            	status[LED_GREEN_STATUS] = LOW_POWER_OFF;
            } else if( event == METER_TEST_MODULE ) {
            	status[LED_GREEN_STATUS] = TEST_MODE;
            } else if( event == METER_TEST_MODULE_ERROR_1 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_1;
            } else if( event == METER_TEST_MODULE_ERROR_2 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_2;
            } else if( event == METER_TEST_MODULE_ERROR_3 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_3;
            } else if( event == METER_TEST_MODULE_ERROR_4 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_4;
            } else if( event == METER_TEST_MODULE_ERROR_5 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_5;
            } else if( event == METER_TEST_MODULE_ERROR_6 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_6;
            } else if( event == METER_LOCAL_TOOL ) {
            	status[LED_GREEN_STATUS] = BLINK;
            }
            break;
        case BLINK:
        	if( event == METER_RD_WR ) {
            	status[LED_GREEN_STATUS] = ON;
            } else if( event == METER_LOW_POWER ) {
            	status[LED_GREEN_STATUS] = LOW_POWER_OFF;
            } else if( event == METER_TEST_MODULE ) {
            	status[LED_GREEN_STATUS] = TEST_MODE;
            } else if( event == METER_TEST_MODULE_ERROR_1 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_1;
            } else if( event == METER_TEST_MODULE_ERROR_2 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_2;
            } else if( event == METER_TEST_MODULE_ERROR_3 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_3;
            } else if( event == METER_TEST_MODULE_ERROR_4 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_4;
            }  else if( event == METER_TEST_MODULE_ERROR_5 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_5;
            } else if( event == METER_TEST_MODULE_ERROR_6 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_6;
            } else if( event == METER_LOCAL_TOOL ) {
            	status[LED_GREEN_STATUS] = BLINK;
            }
            break;
        case ON:
        	if( event == METER_RD_WR ) {
            	status[LED_GREEN_STATUS] = ON;
            } else if( event == METER_LOW_POWER ) {
            	status[LED_GREEN_STATUS] = LOW_POWER_OFF;
            } else if( event == METER_TEST_MODULE ) {
            	status[LED_GREEN_STATUS] = TEST_MODE;
            } else if( event == METER_TEST_MODULE_ERROR_1 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_1;
            } else if( event == METER_TEST_MODULE_ERROR_2 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_2;
            } else if( event == METER_TEST_MODULE_ERROR_3 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_3;
            } else if( event == METER_TEST_MODULE_ERROR_4 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_4;
            } else if( event == METER_TEST_MODULE_ERROR_5 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_5;
            } else if( event == METER_TEST_MODULE_ERROR_5 ) {
            	status[LED_GREEN_STATUS] = TEST_MODE_ERROR_5;
            } else if( event == METER_LOCAL_TOOL ) {
            	status[LED_GREEN_STATUS] = BLINK;
            }
            break;
        case TEST_MODE:
        	break;
        default:
            break;
    }
}

/**
 *
 * @}
 *
 */
