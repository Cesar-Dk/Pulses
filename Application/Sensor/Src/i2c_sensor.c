/*
 * i2c_sensor.c
 *
 *  Created on: 1 jul. 2024
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "tick.h"
#include "i2c_sensor.h"
#include "gpdma.h"
#include "params.h"
#include "ad.h"
#include "pulses.h"

#define NUM_I2C_INPUTS (4)

typedef enum {
	I2C_SENSOR_NONE = 0,
	I2C_SENSOR_PTE7300,
	I2C_SENSOR_DCT532,
	I2C_SENSOR_LAST_SENSOR
} i2c_sensor_model_t;

struct i2c_sensor_t{
	uint32_t i2c_sensor_type;
	uint32_t i2c_sensor_data[NUM_I2C_INPUTS];
	uint32_t i2c_sensor_curr_sensor;
	uint32_t i2c_sensor_num_sensors;
	uint32_t i2c_sensor_address[NUM_I2C_INPUTS];
	uint32_t warmup_time;
};

/* Size of Transmission buffer */
#define TXBUFFERSIZE                    (COUNTOF(aTxBuffer) - 1)
#define TX_CONSTBUFF8_SIZE              (COUNTOF(ConstBuffer8)-1)

/* Size of Reception buffer */
#define RXBUFFERSIZE                    TXBUFFERSIZE
/* Buffer used for transmission */
uint8_t aTxBuffer[] = "*******";

/* Buffer used for reception */
uint8_t aRxBuffer[256];

/* Buffers used for DMA transfer */
static uint8_t ConstBuffer8[] = "********";
static uint8_t Buffer8[256];
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
//PTE7300 Definitions.
void i2c_sensor_pte7300( void );

// default nodeaddress
#define DEFAULT_NODE_ADDRESS	0xD8//0x6C//

// command register address
#define RAM_ADDR_CMD       0x22
// serial register
#define RAM_ADDR_SERIAL    0x50
// result registers
#define RAM_ADDR_DSP_T     0x2E
#define RAM_ADDR_DSP_S     0x30
#define RAM_ADDR_STATUS	   0x36
#define RAM_ADDR_ADC_TC    0x26

void i2c_sensor_set_data( uint8_t *pData, uint32_t number, uint32_t bytesRead, uint16_t *buffer );

uint32_t __pte7300_write_read_register(uint8_t *reg_address, uint16_t number, uint8_t *buffer, uint32_t read)
{
#define WAIT_TIME (1500)
	uint32_t bytes_read = 0;
	uint32_t tick_while = HAL_GetTick();
	do
	{
		if((bytes_read = HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)(i2c_sensor_get_address(i2c_sensor_get_curr_sensor())*2)/*(uint16_t)DEFAULT_NODE_ADDRESS*/, reg_address, number))!= HAL_OK)
		{
			/* Error_Handler() function is called when error occurs. */
			Error_Handler();
		}

		/* Start the GPDMA Channel 0 transfer */
		HAL_DMA_Start_IT(&handle_GPDMA1_Channel0, (uint32_t) &ConstBuffer8, (uint32_t) &Buffer8, TX_CONSTBUFF8_SIZE);

		/*##-3- Wait for the end of the transfer #################################*/
		/*  Before starting a new communication transfer, you need to check the current
	        state of the peripheral; if it is busy you need to wait for the end of current
	        transfer before starting a new one.
	        For simplicity reasons, this example is just waiting till the end of the
	        transfer, but application may perform other tasks while transfer operation
	        is ongoing. */
		uint32_t tickstart = HAL_GetTick();

		do {
			asm("nop");
		} while((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) && ((HAL_GetTick() - tickstart) < WAIT_TIME));

		/* When Acknowledge failure occurs (Slave don't acknowledge it is address)
	       Master restarts communication */
	}
	while((HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) && ((HAL_GetTick() - tick_while) < WAIT_TIME));

	if (1 == read)
	{
		/*##-4- Put I2C peripheral in reception process ###########################*/
		do
		{
			if((bytes_read = HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)(i2c_sensor_get_address(i2c_sensor_get_curr_sensor())*2)/*(uint16_t)DEFAULT_NODE_ADDRESS*/, buffer, number*2)) != HAL_OK)
			{
				/* Error_Handler() function is called when error occurs. */
				Error_Handler();
			}

			/* Start the GPDMA Channel 3 transfer */
			HAL_DMA_Start_IT(&handle_GPDMA1_Channel0, (uint32_t) &ConstBuffer8, (uint32_t) &Buffer8, TX_CONSTBUFF8_SIZE);

			/*##-5- Wait for the end of the transfer #################################*/
			/*  Before starting a new communication transfer, you need to check the current
	        state of the peripheral; if it is busy you need to wait for the end of current
	        transfer before starting a new one.
	        For simplicity reasons, this example is just waiting till the end of the
	        transfer, but application may perform other tasks while transfer operation
	        is ongoing. */
			uint32_t tickstart_ = HAL_GetTick();

			do {
				asm("nop");
			} while((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) && ((HAL_GetTick() - tickstart_) < WAIT_TIME));

			/* When Acknowledge failure occurs (Slave don't acknowledge it is address)
	       Master restarts communication */
		}
		while((HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) && ((HAL_GetTick() - tick_while) < WAIT_TIME));
	}
	return bytes_read;
}

uint32_t __pte7300_write_read_register_i2c4(uint8_t *reg_address, uint16_t number, uint8_t *buffer, uint32_t read)
{
#define WAIT_TIME (1500)
	uint32_t bytes_read = 0;
	uint32_t tick_while = HAL_GetTick();
	/* While the I2C in reception process, user can transmit data through
	     "aTxBuffer" buffer */
	/* Timeout is set to WAIT_TIMEmS */
	do
	{
		/* Error_Handler() function is called when Timeout error occurs.
	       When Acknowledge failure occurs (Slave don't acknowledge its address)
	       Master restarts communication */
		if ((bytes_read = HAL_I2C_Master_Transmit(&hi2c4,
				(uint16_t)(i2c_sensor_get_address(i2c_sensor_get_curr_sensor())*2),
				reg_address, number, WAIT_TIME)) != HAL_OK)
		{
			Error_Handler();
		}

		uint32_t tickstart = HAL_GetTick();
		do {
			__NOP();
		} while((HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) && ((HAL_GetTick() - tickstart) < WAIT_TIME));

	} while ((HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) && ((HAL_GetTick() - tick_while) < WAIT_TIME));

	if (1 == read)
	{
		  /*##- Put I2C peripheral in reception process ############################*/
		  /* Timeout is set to 10S */
		  do
		  {
		    /* Error_Handler() function is called when Timeout error occurs.
		       When Acknowledge failure occurs (Slave don't acknowledge it's address)
		       Master restarts communication */
		    if ((bytes_read=HAL_I2C_Master_Receive(&hi2c4,
					  (uint16_t)(i2c_sensor_get_address(i2c_sensor_get_curr_sensor())*2),
					  buffer, number*2, WAIT_TIME)) != HAL_OK)
		    {
		    	Error_Handler();
		    }

			uint32_t tickstart_ = HAL_GetTick();
			do {
				__NOP();
			} while((HAL_I2C_GetState(&hi2c4) != HAL_I2C_STATE_READY) && ((HAL_GetTick() - tickstart_) < WAIT_TIME));

		  } while ((HAL_I2C_GetError(&hi2c4) == HAL_I2C_ERROR_AF) && ((HAL_GetTick() - tick_while) < WAIT_TIME));
	}
	return bytes_read;
}

static void (*i2c_sensor_model[ I2C_SENSOR_LAST_SENSOR ])( void ) =
{
		(void *)I2C_SENSOR_NONE,
		i2c_sensor_pte7300,
		(void *)I2C_SENSOR_NONE
};

struct i2c_sensor_t i2c_sensor;

void __reset_i2c(void)
{
	MX_I2C1_DeInit();
	MX_GPDMAI2C1_DeInit();
	MX_GPDMA1_Init_I2C();
	MX_I2C1_Init();
}

void __reset_i2c_next(void)
{
	MX_I2C4_DeInit();
	MX_I2C4_Init();
}

void i2c_sensor_task(void)
{
	static uint32_t disable_adc = 0;

	if ((1 == AD_GetADCData())
	 && (0 == pulses_get_pulse_rx())
	 && (1 == params_get_i2c_sensor())
	   )
	{
		if (0==disable_adc)
		{
			AD_read_sensor_pressure_sensor();
			disable_adc = 1;
		}
		i2c_sensor_pte7300();
		AD_setEnd(1);
		AD_SetGetADCData(0);
		pulses_set_mutex(0);
//		i2c_sensor_low_power_mode(1);
	}
}
void i2c_sensor_pte7300(void)
{
//	uint8_t idle_power_mode[3] = {0x22,0xBA,0x7B};
	uint8_t high_power_mode[3] = {0x22,0x93,0x8B};
	uint8_t deep_sleep_mode[3] = {0x22,0x32,0x6C};
	uint8_t address = RAM_ADDR_DSP_S;
	uint32_t error;

	memset(aRxBuffer,0,sizeof(aRxBuffer));

	if ( 0 == i2c_sensor_get_curr_sensor() )
	{
		__reset_i2c();

		__pte7300_write_read_register(high_power_mode, 3, aRxBuffer,0);

		__reset_i2c();

		error = __pte7300_write_read_register(&address, 1, aRxBuffer,1);

		__reset_i2c();
	}
	else if ( 1 == i2c_sensor_get_curr_sensor() )
	{
		__reset_i2c_next();

		__pte7300_write_read_register_i2c4(high_power_mode, 3, aRxBuffer,0);

		__reset_i2c_next();

		error = __pte7300_write_read_register_i2c4(&address, 1, aRxBuffer,1);

		__reset_i2c_next();
	}

	if (error == HAL_OK)
	{
		i2c_sensor_set_data(aRxBuffer, 1, 2, (uint16_t *)&i2c_sensor.i2c_sensor_data[i2c_sensor_get_curr_sensor()]);
		if (1 == params_pulses_on())
		{
			int16_t val_int = (int16_t)(i2c_sensor.i2c_sensor_data[i2c_sensor_get_curr_sensor()]);
			float_t val_dec = (float_t)((val_int*5.00/1600 + 50.00) * 1000);
			AD_SetAverage(val_dec);
		}
	}
	else
	{
		i2c_sensor.i2c_sensor_data[i2c_sensor_get_curr_sensor()] = 49536;
		LOGLIVE(LEVEL_2, "LOGLIVE> %d I2C_SENSOR> Error reading sensor:%d address:%d.\r\n",
				(int)Tick_Get( SECONDS ), (int)i2c_sensor.i2c_sensor_curr_sensor,
				(int)i2c_sensor.i2c_sensor_address[i2c_sensor.i2c_sensor_curr_sensor]);
		if (1 == params_pulses_on())
		{
			int16_t val_int = (int16_t)(i2c_sensor.i2c_sensor_data[i2c_sensor_get_curr_sensor()]);
			float_t val_dec = (float_t)((val_int*5.00/1600 + 50.00) * 1000);
			AD_SetAverage(val_dec);
		}
	}

	if ( 0 == i2c_sensor_get_curr_sensor() )
	{
		__pte7300_write_read_register(deep_sleep_mode, 3, aRxBuffer, 0);

		MX_I2C1_DeInit();
		MX_GPDMAI2C1_DeInit();
	}
	else if ( 1 == i2c_sensor_get_curr_sensor() )
	{
		__pte7300_write_read_register_i2c4(deep_sleep_mode, 3, aRxBuffer, 0);

		MX_I2C4_DeInit();
	}
}

uint32_t i2c_sensor_low_power_mode(uint32_t _sensor_num)
{
	uint8_t deep_sleep_mode[3] = {0x22,0x32,0x6C};

	memset(aRxBuffer,0,sizeof(aRxBuffer));

	if ( 0 == _sensor_num )
	{
		__reset_i2c();

		__pte7300_write_read_register(deep_sleep_mode, 3, aRxBuffer, 0);

		MX_I2C1_DeInit();
		MX_GPDMAI2C1_DeInit();
	}
	else if (  1 == _sensor_num )
	{
		__reset_i2c_next();

		__pte7300_write_read_register_i2c4(deep_sleep_mode, 3, aRxBuffer, 0);

		MX_I2C4_DeInit();
	}
	return 0;
}

uint32_t i2c_sensor_get_curr_sensor(void)
{
	return i2c_sensor.i2c_sensor_curr_sensor;
}

void i2c_sensor_set_curr_sensor(uint32_t i)
{
	i2c_sensor.i2c_sensor_curr_sensor = i;
}

uint32_t i2c_sensor_get_num_sensors(void)
{
	return i2c_sensor.i2c_sensor_num_sensors;
}

void i2c_sensor_set_num_sensors(uint32_t num)
{
	i2c_sensor.i2c_sensor_num_sensors = num;
}

uint32_t i2c_sensor_get_address(uint32_t _address)
{
	return i2c_sensor.i2c_sensor_address[_address];
}

void i2c_sensor_set_address(uint32_t i, uint32_t address)
{
	i2c_sensor.i2c_sensor_address[i] = address;
}

uint32_t i2c_sensor_get_data( void )
{
	//Multiplica por 1000 porque luego en generic_sensor_log_read_sensor divide por 1000...Pero sólo el primer sensor!!!!
	float_t pct = 0;
	if ( 0 == i2c_sensor_get_curr_sensor() )
	{
		pct = (float_t)(((int16_t)(i2c_sensor.i2c_sensor_data[i2c_sensor_get_curr_sensor()])*5.00/1600 + 50.00) * 1000);
	}
	else
	{
		pct = (float_t)((int16_t)(i2c_sensor.i2c_sensor_data[i2c_sensor_get_curr_sensor()])*5.00/1600 + 50.00);
	}

	return pct;
}

void i2c_sensor_set_data( uint8_t *pData, uint32_t number, uint32_t bytesRead, uint16_t *buffer )
{
	if ( bytesRead >= number * 2 )
	{
		for (uint32_t i = 0; i < number; i++)
		{
			uint8_t lowByte  = pData[0]; // read low byte
			uint8_t highByte = pData[1]; // read high byte
			buffer[i]        = highByte << 8 | lowByte; // join two bytes into word (uint16)
		}
	}
}

void *i2c_sensor_init(void)
{
	if ( params_get_i2c_sensor() != 0 )
	{
		return i2c_sensor_model[params_get_i2c_sensor()];
	}
	else
	{
		return NULL;
	}

}
