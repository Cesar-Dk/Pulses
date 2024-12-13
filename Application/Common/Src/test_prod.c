/*
 * test_prod.c
 *
 *  Created on: 8 april 2020
 *      Author: Sergio Millán López
 */
#include "test_prod.h"
#include "ME910.h"
#include "tick.h"
#include "leds.h"
#if defined(MBUS)
#include "mbus.h"
#elif defined(UNE82326)
#include "une82326.h"
#endif
#include "shutdown.h"
#include "ad.h"
#include "modbus.h"
#include "modbus_sensors.h"
#include "pulses.h"
#include "spi_flash.h"
#include "gpio.h"
#include "usart.h"
#include "spi.h"
#include "iwdg.h"

#define OK  (1)
#define NOK (0)

#define PORT_TEST_IN     (UC_PULSES_TAMP_GPIO_Port)
#define PIN_TEST_IN      (UC_PULSES_TAMP_Pin)
#define PORT_TEST_IN_2   (UC_PULSES_CH1D_GPIO_Port)
#define PIN_TEST_IN_2    (UC_PULSES_CH1D_Pin)
#define PORT_TEST_IN_3   (UC_PULSES_CH1P_GPIO_Port)
#define PIN_TEST_IN_3    (UC_PULSES_CH1P_Pin)

#define TIMEOUT_REED   (500)
#define TIMEOUT_PRE    (2000)
#define TIMEOUT_PRE_EU (4000)
#define TIMEOUT_CHECK  (10000)

typedef enum {
	ERROR_LEDS,
	ERROR_TAMPER,
	ERROR_REED,
	ERROR_NB,
	ERROR_SPI,
	ERROR_MODBUS,
	ERROR_MBUS,
	ERROR_SENSOR,
	ERROR_PULSE,
	ERROR_TO_MODULE,
	ERROR_FINAL,
	ERROR_LAST
} error_index ;

static uint8_t  run = 0;
uint8_t         error[ ERROR_LAST ];

extern uint32_t reed_on, tamper_on, tamper_int;

#if 0
static void __initPinStructure(GPIO_InitTypeDef GPIO_InitStructure, GPIO_TypeDef *pin_port,  uint32_t in_out, uint32_t GPIO_PuPd, uint32_t num_pin)
{
    GPIO_InitStructure.Pin   = num_pin;
    GPIO_InitStructure.Mode  = in_out;
    GPIO_InitStructure.Pull  = GPIO_PuPd;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_Init( pin_port, &GPIO_InitStructure );
}

static int8_t __testDetection( void )
{
    static enum
    {
       INIT, READ_PIN, DEBOUNCE, RUN_TEST, EXIT,
    } status = INIT;

    GPIO_InitTypeDef GPIOLED_1_InitStructure;
    static uint32_t init_time = 0, tick = 0, debounce = 0;
    int8_t ret = 0;

    switch (status) {
    case INIT:
        init_time = Tick_Get(SECONDS);
//        HAL_GPIO_WritePin(GPIOH, UC_LED_BLUE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PORT_TEST_IN, PIN_TEST_IN, GPIO_PIN_SET);
        __initPinStructure(GPIOLED_1_InitStructure, PORT_TEST_IN, GPIO_MODE_INPUT, GPIO_PULLUP, PIN_TEST_IN);

        HAL_GPIO_WritePin(PORT_TEST_IN_2, PIN_TEST_IN_2, GPIO_PIN_SET);
        __initPinStructure(GPIOLED_1_InitStructure, PORT_TEST_IN_2, GPIO_MODE_INPUT, GPIO_PULLUP, PIN_TEST_IN_2);

        HAL_GPIO_WritePin(PORT_TEST_IN_3, PIN_TEST_IN_3, GPIO_PIN_SET);
        __initPinStructure(GPIOLED_1_InitStructure, PORT_TEST_IN_3, GPIO_MODE_INPUT, GPIO_PULLUP, PIN_TEST_IN_3);
        status++;
        break;
    case READ_PIN:
        if ((Tick_Get(SECONDS) - init_time) < 5) {
            if (( GPIO_PIN_RESET == HAL_GPIO_ReadPin( PORT_TEST_IN,   PIN_TEST_IN ) )
             && ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( PORT_TEST_IN_2, PIN_TEST_IN_2 ) )
			 && ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( PORT_TEST_IN_3, PIN_TEST_IN_3 ) )
			 )
			{
                tick = Tick_Get(SECONDS);
                status++;
            }
        } else {
            status = EXIT;
        }
        break;
    case DEBOUNCE:
        if ((Tick_Get(SECONDS) - tick) < 2) {
            if (( GPIO_PIN_RESET == HAL_GPIO_ReadPin( PORT_TEST_IN,   PIN_TEST_IN ) )
             && ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( PORT_TEST_IN_2, PIN_TEST_IN_2 ) )
			 && ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( PORT_TEST_IN_3, PIN_TEST_IN_3 ) )
            		){
                debounce++;
            } else {
                debounce = 0;
            }
        }
        else {
            if (debounce >= 0) {
                tick = Tick_Get(SECONDS);
                status++;
            } else {
                status = EXIT;
            }
        }
        break;
    case RUN_TEST:
        ret = 1;
        break;
    case EXIT:
        ret = -1;
        break;
    }
    return ret;
}
#endif

__STATIC_INLINE void __ledsON( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_LED,     PIN_TEST_LED,     GPIO_PIN_SET );
    HAL_GPIO_WritePin( PORT_TEST_MBUS,    PIN_TEST_MBUS,    GPIO_PIN_SET );
    HAL_GPIO_WritePin( PORT_TEST_MODBUS,  PIN_TEST_MODBUS,  GPIO_PIN_SET );
    HAL_GPIO_WritePin( PORT_TEST_SPI,     PIN_TEST_SPI,     GPIO_PIN_SET );
    HAL_GPIO_WritePin( PORT_TEST_SENSOR,  PIN_TEST_SENSOR,  GPIO_PIN_SET );
}

__STATIC_INLINE void __ledsOff( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_LED,     PIN_TEST_LED,     GPIO_PIN_RESET );
    HAL_GPIO_WritePin( PORT_TEST_MBUS,    PIN_TEST_MBUS,    GPIO_PIN_RESET );
    HAL_GPIO_WritePin( PORT_TEST_MODBUS,  PIN_TEST_MODBUS,  GPIO_PIN_RESET );
    HAL_GPIO_WritePin( PORT_TEST_SPI,     PIN_TEST_SPI,     GPIO_PIN_RESET );
    HAL_GPIO_WritePin( PORT_TEST_SENSOR,  PIN_TEST_SENSOR,  GPIO_PIN_RESET );
}

__STATIC_INLINE void __ledCloudOff( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_RESET  );
}

__STATIC_INLINE void __ledMBEnOn( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_MBUS, PIN_TEST_MBUS, GPIO_PIN_SET );
}

__STATIC_INLINE void __ledMBEnOff( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_MBUS, PIN_TEST_MBUS, GPIO_PIN_RESET );
}

__STATIC_INLINE void __ledModBusEnOn( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_MODBUS, PIN_TEST_MODBUS, GPIO_PIN_SET );
}

__STATIC_INLINE void __ledModBusEnOff( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_MODBUS, PIN_TEST_MODBUS, GPIO_PIN_RESET );
}

__STATIC_INLINE void __ledSPIEnOn( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_SPI, PIN_TEST_SPI, GPIO_PIN_SET  );
}

__STATIC_INLINE void __ledSPIEnOff( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_SPI, PIN_TEST_SPI, GPIO_PIN_RESET  );
}

__STATIC_INLINE void __ledSEnOn( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_SENSOR, PIN_TEST_SENSOR, GPIO_PIN_SET  );
}

__STATIC_INLINE void __ledSEnOff( void )
{
	leds_set_NET_status(TEST_MODULE);
	leds_set_METER_status(METER_TEST_MODULE);

    HAL_GPIO_WritePin( PORT_TEST_SENSOR, PIN_TEST_SENSOR, GPIO_PIN_RESET  );
}

__STATIC_INLINE void __ledsERROR_1( void )
{
	leds_set_NET_status(TEST_MODULE_ERROR_1);
}

__STATIC_INLINE void __ledsERROR_2( void )
{
	leds_set_NET_status(TEST_MODULE_ERROR_2);
}

__STATIC_INLINE void __ledsERROR_3( void )
{
	leds_set_NET_status(TEST_MODULE_ERROR_3);
}

__STATIC_INLINE void __ledsERROR_4( void )
{
	leds_set_NET_status(TEST_MODULE_ERROR_4);
}

__STATIC_INLINE void __ledsERROR_5( void )
{
	leds_set_NET_status(TEST_MODULE_ERROR_5);
}

__STATIC_INLINE void __ledsERROR_6( void )
{
	leds_set_NET_status(TEST_MODULE_ERROR_6);
}

__STATIC_INLINE void __ledsERROR( void )
{
//	error[ERROR_FINAL] = error[ERROR_NB] & error[ERROR_SPI] & error[ERROR_MODBUS] & error[ERROR_MBUS] & error[ERROR_EURIDIS] & error[ERROR_PULSE];

	if (error[ERROR_NB] == NOK)
	{
		__ledCloudOff();
		__ledSPIEnOff();
	}
	if (error[ERROR_SPI] == NOK)
	{
		__ledSPIEnOff();
	}
	if (error[ERROR_MODBUS] == NOK)
	{
		__ledModBusEnOff();
	}
	if (error[ERROR_MBUS] == NOK)
	{
		__ledMBEnOff();
	}
//	if (error[ERROR_EURIDIS] == NOK)
//	{
//		__ledEuridisEnOff();
//	}
}

__STATIC_INLINE uint8_t __programTelegramValueWord( void )
{
    uint32_t  size, check_write, ret = 0;
	uint8_t  *ptr;
	uint32_t next_addr;
	char     dlms_telegram_value[17], record_check[17];

	MX_SPI2_Init();
	sFLASH_Init();

	snprintf(dlms_telegram_value, 17*sizeof(uint8_t), "TEST SPI FLASH\r\n");
	size = sizeof(dlms_telegram_value);
    ptr  = (uint8_t *) &dlms_telegram_value;

    next_addr = 0x2000;
	sFLASH_EraseSector(next_addr&0xFFFFF000);
	HAL_Delay(100);
    sFLASH_WriteBuffer(ptr, next_addr, size);
    HAL_Delay(100);
    sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );

    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	__NOP();
    	check_write = 0;
    	ret         = 0;
    } else {
    	__NOP();
    	check_write = 1;
    	ret         = 1;
    }
    uint32_t num = 0;
    if ( 1 == check_write ) {
    	do {
    		sFLASH_EraseSector(next_addr&0xFFFFF000);
    		HAL_Delay(100);
    		sFLASH_WriteBuffer(ptr, next_addr, size);
    		HAL_Delay(100);
    		sFLASH_ReadBuffer( (uint8_t *)&record_check, next_addr, size );
    		num++;
    	    if ( 0 == memcmp(ptr, (uint8_t *)&record_check, size)) {
    	    	__NOP();
    	    	check_write = 0;
    	    	ret         = 0;
    	    } else {
    	    	__NOP();
    	    	check_write = 1;
    	    	ret         = 1;
    	    }
    	} while ( ( 1 == check_write ) && ( num < 5 ) );
    }

    return ret;
}

__STATIC_INLINE void __modbusTask(void)
{
    static uint8_t tx_rx = 0;

    if ((tx_rx == 3) || (tx_rx == 6)) {
    	modbus_test_rcx();
//    	if (tx_rx == 6) {
//    		tx_rx = 0;
//    	}
//    	else
    	{
    		tx_rx++;
    	}
    }
    else {
    	if (tx_rx == 0) {
//    		MX_USART2_UART_Init();
//    		HAL_GPIO_WritePin( PORT_TEST_MODBUS,  PIN_TEST_MODBUS,  GPIO_PIN_SET );
    		modbus_test_trx();
    	}
        tx_rx++;
    }
}

void test_prod_task( void )
{
	static enum
	{
		INIT, WAITING_LIGHT, REED, LTE_NB, SPI_TEST, MODBUS_TEST,
		SENSOR_TEST, MBUS_TEST, PULSE_TEST, ERROR_TO_MODULE, ERROR, LAST
	} status = INIT;

	static uint32_t tick, tick_ini;
	static uint32_t tries = 3;
	static uint32_t modbus_step = 0;
	static uint32_t mbus_euridis_step = 0;
	static uint32_t pulses_step = 0;
	static uint32_t severn_trent_err = 0, mbus_err = 0, mem_err = 0, clk_err = 0, nb_status = 0;;
	int8_t run_test = 0;

	if ( status == INIT ) {
//		run_test = __testDetection();
//		run_test = 1;//DEBUG.
	    if ( -1 == run_test ) {
	    	return;
		} else if ( 1 == run_test ) {
			run = 1;
			modbus_test_as11_get();
		}
	}

	if (run == 0) {
		return;
	}

	if ( MODBUS_TEST == status ) {
		__modbusTask();
	}

	switch(status) {
	case INIT:
//		params_set_loglive_level(LEVEL_0);
//		if ( 1 == mbus_end() ) {
//			__ledsOff();
//			LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> Waiting TAMPER.\r\n",
//				(int)Tick_Get( SECONDS ));
			LOGLIVE_TAMP_EN(LEVEL_1, "AT+TEST=FOTOTRANSISTOR\r\n");
			LOGLIVE_TAMP_EN(LEVEL_1, "AT+TEST=MBUS\r\n");
			mbus_set_start_comm(1);
			tick_ini = HAL_GetTick();
			tick     = HAL_GetTick();
			status++;
//		}
		break;
	case WAITING_LIGHT:
    	if (run) {
			if ( ( 1 == tamper_on ) || ( 1 == tamper_int ) )  {
//				LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> Waiting REED.\r\n",
//								(int)Tick_Get( SECONDS ));
				LOGLIVE_TAMP_EN(LEVEL_1, "AT+TEST=REED\r\n");
				__ledsON();
				error[ERROR_TAMPER] = OK;
				tick_ini = HAL_GetTick();
				tick     = HAL_GetTick();
				status++;

		    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_FOTOTRANSISTOR:%d.\r\n",
		    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_TAMPER]);

		    	HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_SET );
			}
			else if ( (HAL_GetTick() - tick) > (TIMEOUT_CHECK+5000)) {
				__ledsON();
				error[ERROR_TAMPER] = NOK;
				tick_ini = HAL_GetTick();
				tick     = HAL_GetTick();
				status++;

		    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d \r\nRESULT_FOTOTRANSISTOR:%d.\r\n",
		    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_TAMPER]);

		    	HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_SET );
			}
    	}
		break;
	case REED:
		if ( (HAL_GetTick() - tick) > TIMEOUT_REED ) {
    		if ( 1 == reed_on ) {
    			LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST STARTED.\r\n",
    							(int)Tick_Get( SECONDS ));
    			__ledsOff();
//    			HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_SET );
    			HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_RESET );
    			reed_on  = 0;
    			error[ERROR_REED] = OK;
    			tick_ini          = HAL_GetTick();
    			tick              = HAL_GetTick();
    			status++;
//    			params_set_loglive_level(LEVEL_0);

		    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_REED:%d.\r\n",
		    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_REED]);
    		}
			else if ( (HAL_GetTick() - tick) > (TIMEOUT_CHECK + 5000 - TIMEOUT_PRE) ) {
    			LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST STARTED.\r\n",
    							(int)Tick_Get( SECONDS ));
				__ledsOff();
				HAL_GPIO_WritePin( PORT_TEST_LED, PIN_TEST_LED, GPIO_PIN_SET );
				error[ERROR_REED] = NOK;
    			tick_ini  		  = HAL_GetTick();
    			tick    		  = HAL_GetTick();
				status++;

		    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_REED:%d.\r\n",
		    	  		(int)Tick_Get( SECONDS ),  (int)error[ERROR_REED]);
			}
		}
		break;
    case LTE_NB:
//    	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( PORT_TEST_IN, PIN_TEST_IN ) )
    	{
    		if ( (HAL_GetTick() - tick) > TIMEOUT_PRE ) {
				if ( 1 == ME910_test_ok() ) {//1
					error[ERROR_NB] = OK;
					tick_ini        = HAL_GetTick();
					tick            = HAL_GetTick();
					status          = SPI_TEST;
					mbus_set_start_comm(1);

	    			LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> \r\nRESULT_IMEI:%s.\r\n", ME910_IMEI());
	    			LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> \r\nRESULT_APN:%s.\r\n", params_get_APN());
			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d \r\nRESULT_TELIT:%d.\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_NB]);
				} else if ( -1 == ME910_test_ok() ) {
//					__ledsOff();
//					__ledsERROR_1();
					error[ERROR_NB] = NOK;
	    			tick_ini  		= HAL_GetTick();
	    			tick    	    = HAL_GetTick();
					status++;
					shutdown_setInitTelitModule(0);
					mbus_set_start_comm(1);
					nb_status = Telit_read_error_status();
					LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> \r\nRESULT_IMEI:%s.\r\n", ME910_IMEI());
			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d \r\nRESULT_TELIT:%d.%d\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_NB], (int)nb_status);
				}
    		}
    	}
    break;
    case SPI_TEST:
//    	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( PORT_TEST_IN, PIN_TEST_IN ) )
    	{
    		if ( (HAL_GetTick() - tick) > TIMEOUT_PRE) {
    			uint32_t wr_mem_err = __programTelegramValueWord();
    			if ((0 == wr_mem_err) && ( rtc_system_get_FreqLSI() > 31000) && ( rtc_system_get_FreqLSI() < 35000)) {
//    				modbus_sensors_set_serial_config_baud_rate(115200);
//					serialRS485_deInit();
//					serialRS485_module_init(115200);
//    				modbus_deInit();
//    				modbus_init();
					error[ERROR_SPI] = OK;
					tick_ini         = HAL_GetTick();
					tick             = HAL_GetTick();
					status           = SENSOR_TEST;//MODBUS_TEST;
					mem_err          = OK;
					clk_err          = OK;
					AD_SetGetADCData(1);

			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD>\r\nRESULT_CLK:1.%d. \r\nRESULT_MEM:%d.\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)rtc_system_get_FreqLSI(), (int)error[ERROR_SPI]);
    			} else {
//					__ledsERROR_2();
//					modbus_sensors_set_serial_config_baud_rate(115200);
//					serialRS485_deInit();
//					serialRS485_module_init(115200);
					error[ERROR_SPI] = NOK;
					tick_ini         = HAL_GetTick();
					tick             = HAL_GetTick();
					if (( rtc_system_get_FreqLSI() > 31000) && ( rtc_system_get_FreqLSI() < 35000))
					{
						clk_err = OK;
					}
					else
					{
						clk_err = NOK;
					}
					if ( 0 == wr_mem_err)
					{
						mem_err = OK;
					}
					else
					{
						mem_err = NOK;
					}
//					status++;
					status = SENSOR_TEST;//MODBUS_TEST;
					AD_SetGetADCData(1);

			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_CLK:%d.%d. \r\nRESULT_MEM:%d.\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)rtc_system_get_FreqLSI(), (int)clk_err, (int)mem_err);
				}
    		}
    	}
    	break;
    case MODBUS_TEST:
//    	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( PORT_TEST_IN, PIN_TEST_IN ) )
    	{
    		if ( (HAL_GetTick() - tick) > (2*TIMEOUT_PRE)) {
    			if ( 0 == modbus_step ) {
    				float sound = ( float ) modbus_test_as11_get() / 10.0;
    				if ( sound > 30 ) {
    					error[ERROR_MODBUS] = OK;
    					tick_ini            = HAL_GetTick();
    					tick                = HAL_GetTick();
    					modbus_step         = 1;
//    					params_set_loglive_level(LEVEL_1);
    					LOGLIVE_SERIAL(LEVEL_1, "\r PIPE20 SMART WATER GATEWAY TEST\r");
//    					params_set_loglive_level(LEVEL_0);
//        				modbus_deInit();
//        				modbus_init();
    					modbus_sensors_set_serial_config_baud_rate(115200);
    					serialRS485_deInit();
    					serialRS485_module_init(115200);
    				} else if (sound <= 30) {
    					if (tries-- == 0) {
//    						__ledsERROR_3();
    						error[ERROR_MODBUS] = NOK;
        					tick_ini            = HAL_GetTick();
        					tick                = HAL_GetTick();
    						status              = SENSOR_TEST;
    						AD_SetGetADCData(1);

    				    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_MODBUS:%d.\r\n",
    				    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_MODBUS]);
    					}
    				}
    			} else if ( 1 == modbus_step ) {
    				float sound = ( float ) modbus_test_as11_get() / 10.0;
    				if ( sound > 30 ) {
    					__ledModBusEnOff();
    					error[ERROR_MODBUS] = OK;
    					tick_ini            = HAL_GetTick();
    					tick                = HAL_GetTick();
    					modbus_step         = 0;
    					status              = SENSOR_TEST;
    					AD_SetGetADCData(1);

				    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nESULT_MODBUS:%d.\r\n",
				    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_MODBUS]);
    				} else if (sound <= 30) {
    					if (tries-- == 0) {
//    						__ledsERROR_3();
    						__ledModBusEnOff();
    						error[ERROR_MODBUS] = NOK;
        					tick_ini            = HAL_GetTick();
        					tick                = HAL_GetTick();
    						status              = SENSOR_TEST;
    						AD_SetGetADCData(1);

    				    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_MODBUS:%d.\r\n",
    				    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_MODBUS]);
    					}
    				}
    			}
    		}
    	}
    	break;
    case SENSOR_TEST:
    	if ( (HAL_GetTick() - tick) > TIMEOUT_PRE) {
    		if ( 1 == AD_getEnd() ) {
    			float_t pressure = (AD_GetAverage()/1000 + 0.2176)/0.2943;
    			if ( pressure > 2.00 ) {
    				__ledSEnOn();
    				error[ERROR_SENSOR] = OK;
    				tick_ini            = HAL_GetTick();
    				tick                = Tick_Get(SECONDS);
    				status 				= MBUS_TEST;

			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_SENSOR:%d.\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_SENSOR]);
    			} else {
    				__ledsERROR_3();
    				error[ERROR_SENSOR] = NOK;
    				status = MBUS_TEST;

			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_SENSOR:%d.\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_SENSOR]);
    			}
    		}
    	}
    	break;
    case MBUS_TEST:
//    	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( PORT_TEST_IN, PIN_TEST_IN ) )
    	{
    		if ( (HAL_GetTick() - tick) > TIMEOUT_PRE) {
    			if (mbus_euridis_step == 0) {
    				if ( 1 == mbus_end() ) {
    					if ( 1 == mbus_check_mbus_frame_ok() ) {
    						error[ERROR_MBUS] = OK;
    						mbus_err = OK;
    					} else {
    						error[ERROR_MBUS] = NOK;
    						mbus_err = NOK;
    					}
						mbus_euridis_step = 1;
						tick_ini          = HAL_GetTick();
						tick              = HAL_GetTick();
						status            = MBUS_TEST;

						params_set_uart_meter_type(SEVERN_TRENT_METER_TYPE);
						mbus_set_start_comm(1);

						LOGLIVE_TAMP_EN(LEVEL_1, "AT+TEST=SEVERN TRENT\r\n");
    				}
    			} else if (mbus_euridis_step == 1) {
    				if ( 1 == mbus_end() ) {
    					if (1 == mbus_check_severn_trent_ok()) {
    						if ( OK == error[ERROR_MBUS] )
    						{
    							error[ERROR_MBUS] = OK;
    							severn_trent_err  = OK;
    						}
    					}
    					else if (2 == mbus_check_severn_trent_ok()) {
    						error[ERROR_MBUS] = NOK;
    						severn_trent_err  = NOK;
    					} else if ( (HAL_GetTick() - tick) > TIMEOUT_CHECK) {
    						error[ERROR_MBUS] = NOK;
    						severn_trent_err  = NOK;
    	    			}
						mbus_euridis_step = 0;
						tick_ini          = HAL_GetTick();
						tick              = HAL_GetTick();
						status            = PULSE_TEST;

						pulses_step          = 1;
						MX_GPIO_Enable_Pulses_CH1P(1);
						MX_GPIO_Enable_Pulses_CH1D(1);
						MX_GPIO_Enable_Tamper(1);
						__ledsOff();
						__ledSPIEnOn();
						__ledModBusEnOn();

				    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_ST:%d. \r\nRESULT_MBUS:%d.\r\n",
				    	  		(int)Tick_Get( SECONDS ), (int)severn_trent_err, (int)mbus_err);

						LOGLIVE_TAMP_EN(LEVEL_1, "AT+TEST=PULSE\r\n");

    				}
    			} else if (( (HAL_GetTick() - tick) > (2*TIMEOUT_CHECK)) || (mbus_euridis_step == 2)) {
					error[ERROR_MBUS] = NOK;
					severn_trent_err  = NOK;
					mbus_euridis_step = 0;
					tick_ini          = HAL_GetTick();
					tick              = HAL_GetTick();
					status            = PULSE_TEST;

					pulses_step       = 1;
					MX_GPIO_Enable_Pulses_CH1P(1);
					MX_GPIO_Enable_Pulses_CH1D(1);
					MX_GPIO_Enable_Tamper(1);
					__ledsOff();
					__ledSPIEnOn();
					__ledModBusEnOn();

			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_ST:%d. \r\nRESULT_MBUS:%d.\r\n",
			    	  		(int)Tick_Get( SECONDS ), (int)severn_trent_err, (int)mbus_err);
					LOGLIVE_TAMP_EN(LEVEL_1, "AT+TEST=PULSE\r\n");
    			}
    		}
    	}
    	break;
    case PULSE_TEST:
//    	if ( GPIO_PIN_SET == HAL_GPIO_ReadPin( PORT_TEST_IN, PIN_TEST_IN ) )
    	{
    		if ( (HAL_GetTick() - tick) > TIMEOUT_PRE)
    		{
    			if ( 1 == pulses_step )
    			{
    				if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( UC_PULSES_CH1P_GPIO_Port, UC_PULSES_CH1P_Pin ) ) {
//    					__ledSPIEnOff();
    					__ledsON();
    					error[ERROR_PULSE] = OK;
    					tick_ini           = HAL_GetTick();
    					tick               = HAL_GetTick();
    					pulses_step        = 2;
    				}
    				else if ( ( HAL_GetTick() - tick ) > TIMEOUT_CHECK )
    				{
//    					__ledsOff();
//    					__ledsERROR_6();
    					__ledsON();
    					error[ERROR_PULSE] = NOK;
    					tick_ini           = HAL_GetTick();
    					tick               = HAL_GetTick();
    					status++;

    			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_PULSE:%d.\r\n",
    			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_PULSE]);
    				}
    			}
    			else if ( 2 == pulses_step )
    			{
    				if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( UC_PULSES_CH1D_GPIO_Port, UC_PULSES_CH1D_Pin ) )
    				{
//    					__ledModBusEnOff();
    					__ledsON();
    					error[ERROR_PULSE] = OK;
    					tick_ini           = HAL_GetTick();
    					tick               = HAL_GetTick();
    					pulses_step        = 3;
//    					status++;
    				}
    				else if ( ( HAL_GetTick() - tick ) > TIMEOUT_CHECK )
    				{
//    					__ledsOff();
//    					__ledsERROR_6();
    					__ledsON();
    					error[ERROR_PULSE] = NOK;
    					tick_ini           = HAL_GetTick();
    					tick               = HAL_GetTick();
    					status++;

    			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_PULSE:%d.\r\n",
    			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_PULSE]);
    				}
    			}
    			else if ( 3 == pulses_step )
    			{
    				if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin ) )
    				{
//    					__ledModBusEnOff();
    					__ledsON();
    					error[ERROR_PULSE] = OK;
    					tick_ini           = HAL_GetTick();
    					tick               = HAL_GetTick();
    					status++;

    			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_PULSE:%d.\r\n",
    			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_PULSE]);
    				}
    				else if ( ( HAL_GetTick() - tick ) > TIMEOUT_CHECK )
    				{
//    					__ledsOff();
//    					__ledsERROR_6();
    					__ledsON();
    					error[ERROR_PULSE] = NOK;
    					tick_ini           = HAL_GetTick();
    					tick               = HAL_GetTick();
    					status++;

    			    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRESULT_PULSE:%d.\r\n",
    			    	  		(int)Tick_Get( SECONDS ), (int)error[ERROR_PULSE]);
    				}
    			}
    		}
    	}
    	break;
    case ERROR_TO_MODULE:
    	params_set_loglive_level(LEVEL_1);
    	error[ERROR_TO_MODULE] = error[ERROR_TAMPER] & error[ERROR_REED] & error[ERROR_PULSE];
    	HAL_Delay(4000);
    	if (error[ERROR_TO_MODULE] == OK)
    	{
    		LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> TAMPER:%d. REED:%d. PULSE:%d.\r\n",
    				(int)Tick_Get( SECONDS ), (int)error[ERROR_TAMPER], (int)error[ERROR_REED], (int)error[ERROR_PULSE]);
    	}
    	else
    	{
    		LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> TAMPER:%d. REED:%d. PULSE:%d.\r\n",
    	  			(int)Tick_Get( SECONDS ), (int)error[ERROR_TAMPER], (int)error[ERROR_REED], (int)error[ERROR_PULSE]);
    	}
		tick_ini           = HAL_GetTick();
		tick               = HAL_GetTick();
		status++;
    	break;
    case ERROR:
//    	if ( (HAL_GetTick() - tick) > TIMEOUT_PRE)
    	{
    	params_set_uart_meter_type(MBUS_METER_TYPE);//REMOVE!!!
    	params_check_for_changes();//REMOVE!!
    	run = 0;//REMOVE!!!
    	error[ERROR_FINAL] = error[ERROR_TAMPER] & error[ERROR_REED] & error[ERROR_PULSE] & error[ERROR_NB] & error[ERROR_SPI] & error[ERROR_MBUS] & error[ERROR_SENSOR];
    	LOGLIVE_TAMP_EN(LEVEL_1, "LOGLIVE> %d TEST_PROD> \r\nRES_VERSION:%d.%d \r\nRES_FOTOTRANSISTOR:%d. \r\nRES_REED:%d. \r\nRES_PULSE:%d. \r\nRES_TELIT:%d.%d. \r\nRES_MEM:%d. \r\nRES_MBUS:%d. \r\nRES_ST:%d. \r\nRES_SENSOR:%d. \r\nRES_CLK:%d.%d. \r\nRES_IMEI:%s. \r\nRESULT_TEST:%d.\r\n",
    	  		(int)Tick_Get( SECONDS ), (int)params_version_major(), (int)params_version_minor(), (int)error[ERROR_TAMPER], (int)error[ERROR_REED], (int)error[ERROR_PULSE], (int)error[ERROR_NB], (int)nb_status, (int)mem_err, (int)mbus_err, (int)severn_trent_err, (int)error[ERROR_SENSOR], (int)clk_err, (int)rtc_system_get_FreqLSI(), ME910_IMEI(), (int)error[ERROR_FINAL] );
//    	if ( error[ERROR_FINAL] != OK ) {
    		HAL_IWDG_Refresh(&hiwdg);
    		__ledsON();
    		__ledsERROR();
    		HAL_Delay(25000);
//    	}
    	status++;
//    	if ( (HAL_GetTick() - tick) > 30000) {
//			params_set_uart_meter_type(MBUS_METER_TYPE);
//			run = 0;
//    	}
    	}
    	break;
    case LAST:
    default:
    	break;
	}

	if ( ( HAL_GetTick() - tick_ini ) > 120000 ) {
		status = LAST;
		test_prod_run(0);
	}
}

void test_prod_run( uint8_t _run )
{
	run = _run;
}

uint8_t test_prod_run_on( void )
{
	return run;
}
