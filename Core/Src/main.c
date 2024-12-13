/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpdma.h"
#include "icache.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "i2c.h"
#include "params.h"
#include "ME910.h"
#include "tick.h"
#include "rtc_system.h"
#include "leds.h"
#include "datalogger_buffer.h"
#include "log.h"
#include "shutdown.h"
#include "leds.h"
#include "spi_flash.h"
#include "serial_une82326.h"
#include "une82326_protocol.h"
#include "une82326.h"
#include "mbus.h"
#include "udp_protocol.h"
#include "sensor_log.h"
#include "modbus.h"
#include "modbus_sensors.h"
#include "modbus_sensors_log.h"
#include "generic_modbus.h"
#include "pulses.h"
#include "ad.h"
#include "ventosa.h"
#include "generic_sensor.h"
#include "i2c_sensor.h"
#include "test_prod.h"
#ifdef DLMS
#include "connection.h"
#include "communication.h"
#include "dlms_client.h"
#endif
#ifdef MQTT
#include "mqtt_task.h"
#include "mqtt_frames.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Interrupt Vector Table RAM Mapping */
#define APPLICATION_ADDRESS     (uint32_t)0x08008000 //_main_app_start_address
#if   (defined ( __CC_ARM ))
__IO uint32_t VectorTable[142] __attribute__((at(0x20000000)));
#elif (defined (__ICCARM__))
#pragma location = 0x20000000
__no_init __IO uint32_t VectorTable[142];
#elif defined   (  __GNUC__  )
__IO uint32_t VectorTable[142] __attribute__((section(".RAMVectorTable")));
#elif defined ( __TASKING__ )
__IO uint32_t VectorTable[142] __at(0x20000000);
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if UINT32_MAX == UINTPTR_MAX
#define STACK_CHK_GUARD 0xe2dee396
#else
#define STACK_CHK_GUARD 0x595e9fbd94fda766
#endif

uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

__attribute__((noreturn))
void __stack_chk_fail(void)
{
#if __STDC_HOSTED__
	abort();
#elif __is_myos_kernel
	panic("Stack smashing detected");
#endif
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
/* USER CODE BEGIN PFP */

__STATIC_FORCEINLINE void __LowLevelSystemInit(void);
__STATIC_FORCEINLINE void __RemapTable(void);
__STATIC_FORCEINLINE void __SystemInit(void);
__STATIC_FORCEINLINE void __DataloggerManagement(void);
__STATIC_FORCEINLINE void __NarrowBandTask(void);
__STATIC_FORCEINLINE void __ScheduledTasks(void);
__STATIC_FORCEINLINE void __Scheduled1s(void);
__STATIC_FORCEINLINE void __Scheduled100ms(void);
__STATIC_FORCEINLINE void __Scheduled10ms(void);
__STATIC_FORCEINLINE void __Scheduled1ms(void);

/**
  * @brief  Maps UART to _write -> to be used with printf.
  *
  * @param  file pointer to the source file name
  * @param  ptr pointer to the byte to write
  * @param  len length of the string to write
  * @retval length of then string to write
  */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */

	for (int i=0 ; i<len ; i++)
	{
#ifdef ITM_MODE
		/* With the debugger connected, if ITM mode is enable printf can be used thru ITM port 0*/
		ITM_SendChar((*ptr++));
#endif
#ifdef LOGLIVE_MODE
		/* To use printf with huart3 for LOGLIVE */
//		HAL_UART_Transmit(&huart2, (uint8_t *)ptr++, 1, 0xFF);
#endif
	}
  return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	__LowLevelSystemInit();
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* Reset the RTC peripheral and the RTC clock source selection */
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
	{
		HAL_PWR_EnableBkUpAccess();
		__HAL_RCC_BACKUPRESET_FORCE();
		__HAL_RCC_BACKUPRESET_RELEASE();
	}
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the System Power */
	SystemPower_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_GPDMA1_Init();
//	MX_ADC4_Init();
//	MX_ADC1_Init();
	MX_ICACHE_Init();
	MX_IWDG_Custom_Init();//MX_IWDG_Init();
//  MX_RTC_Init();
	MX_SPI2_Init();
//	MX_TIM2_Init();
//	MX_USART3_UART_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Custom_Init();//MX_TIM3_Init();
	MX_TIM16_Init();
	/* USER CODE BEGIN 2 */
	/* TIM3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
#ifdef DEBUG_MODE
	HAL_DBGMCU_DisableDBGIWDG();
#endif
	__SystemInit();
	__HAL_RCC_CLEAR_RESET_FLAGS();

	LOGLIVE(LEVEL_1, "\n\r****************************************************");
	LOGLIVE(LEVEL_1, "\r PIPE20 SMART WATER GATEWAY\r");
	LOGLIVE(LEVEL_1, "\n****************************************************\r\n");
	LOGLIVE(LEVEL_1, "Firmware version: PIPE20_V%i.%i.%i\n",(int)param.version.major, (int)param.version.minor, (int) REVISION);
	LOGLIVE(LEVEL_1, "TEST_PROD> RESULT_VERSION:%i.%i\r\n",(int)param.version.major, (int)param.version.minor);
	LOGLIVE(LEVEL_1, "\r\nAT+TEST=AUTOTEST\r\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
	#ifdef EXT_SENSOR
		  Datalogger_SensorWrite();
	#endif
		  Datalogger_ModbusSensorsWrite();
	#if defined(DLMS)
		  Datalogger_DLMSWrite();
	#endif
	#if defined(UNE82326)
		  une82326_task();
	#elif defined(MBUS)
		  mbus_manager();
	#endif
	#if defined(DLMS)
		  con_dlms_Task(con_dlms_get_con_singleton());
	#endif
		  __NarrowBandTask();
		  if ((params_config_read_time() != 0) && (params_config_send_time() != 0))
		  {
			  DataLogger_Task();
		  }
		  __ScheduledTasks();
		  __DataloggerManagement(); // MBUS
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI|RCC_OSCILLATORTYPE_MSIK;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
  RCC_OscInitStruct.MSIKClockRange = RCC_MSIKRANGE_0;
  RCC_OscInitStruct.MSIKState = RCC_MSIK_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCCEx_EnableLSECSS();
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  PWR_PVDTypeDef sConfigPVD = {0};

  /*
   * PVD Configuration
   */
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_3;
  sConfigPVD.Mode = PWR_PVD_MODE_EVENT_FALLING;
  HAL_PWR_ConfigPVD(&sConfigPVD);

  /*
   * Enable the PVD Output
   */
  HAL_PWR_EnablePVD();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();
}

/* USER CODE BEGIN 4 */
void HAL_PWR_PVDCallback(void)
{
	LOGLIVE(LEVEL_1, "LOGLIVE> CPU> PVD\r\n");
}

/**
 * @fn void __LowLevelSystemInit(void)
 * @brief Low Level System Initialization.
 *
 */
__STATIC_FORCEINLINE void __LowLevelSystemInit(void)
{
	 /** Remaps Vector Table Interrupt
	  * */
    __RemapTable();
    /** Enables global interrupts
     * */
    __enable_irq();
}

#define VECT_TAB_OFFSET  0x00000000UL /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */
 /**
  * @fn void __RemapTable(void)
  * @brief Remaps Vector Interrupt Table in RAM.
  *
  */
__STATIC_FORCEINLINE void __RemapTable(void)
{
     // Copy interrupt vector table to the RAM.
     uint32_t ui32_VectorIndex = 0;

     for (ui32_VectorIndex = 0; ui32_VectorIndex < 142; ui32_VectorIndex++)
     {
         VectorTable[ui32_VectorIndex] = * (__IO uint32_t *) (APPLICATION_ADDRESS + (ui32_VectorIndex << 2));
     }

     __HAL_RCC_APB2_FORCE_RESET();
     /* Enable SYSCFG peripheral clock */
     __HAL_RCC_SYSCFG_CLK_ENABLE();
     __HAL_RCC_APB2_RELEASE_RESET();

     /* Remap SRAM at 0x00000000 */
//     __HAL_SYSCFG_REMAPMEMORY_SRAM();
     SCB->VTOR = SRAM1_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
}

/**
 * @fn void __SystemInit(void)
 * @brief System initialization
 */
__STATIC_INLINE void __SystemInit(void)
{
	params_config_set_send_count(0);
	params_init();
    leds_init();
#if defined(PWR_STDBY)
   rtc_system_config();
#elif defined(PWR_STOP)
   /** RTC configuration
    * */
	rtc_system_configStopMode();
#endif
	Tick_init();
	shutdown_check_period_from_reset();
	sFLASH_Init();
//	modbus_vars_init();
	DataLogger_Init();
	mbus_task_init();
//	rest_init();
	ME910_init();
#if defined (UNE82326)
	une82326_init();
#elif defined(MBUS)
	mbus_reset();
//	mbus_set_start_comm(1);
#endif
#ifdef EXT_SENSOR
//	sensor_log_Log();
	if ( ( params_input_pulse_as_sensor_get_num() >= 1 ) && ( params_input_pulse_as_sensor_get_num() <=4 ) )
	{
		generic_sensor_log_init();
	}
	else
	{
		sensor_log_init();
	}
#endif
	if (1 == params_pulses_on())
	{
		pulses_init();
	}
	else
	{
		pulses_deinit();
	}
	MX_GPIO_Enable_Power_Mon(0);
#ifdef MODBUS
	modbus_vars_init();
// 	modbus_sensors_log_init(3);
	generic_485_init();
#endif
	dlms_client_init();
	if (rtc_system_checkAlarmOccured())
	{
		asm("nop");
	}
	MX_GPIO_Relay_Activation(params_get_relay_status());
}

/**
 * @fn void __NarrowBandTask(void)
 * @brief Narrow Band Task.
 */
__STATIC_FORCEINLINE void __NarrowBandTask(void)
{
	ME910_task();
	/* serial port control for data handling -> udp_task is not needed */
#if 0
	udp_task();
#endif
	udp_protocol_task();
#ifdef MQTT
	mqtt_task();
#endif
}

/**
 * @fn void __DataloggerManagement(void)
 * @brief Datalogger Management Thread.
 */
__STATIC_FORCEINLINE void __DataloggerManagement(void)
{
	if ((params_config_read_time() != 0) && (params_config_send_time() != 0))
	{
		datalog_buffer_poll();
	}
}

/**
 * @fn void __ScheduledTasks(void)
 * @brief Periodic Threads.
 *
 */
__STATIC_FORCEINLINE void __ScheduledTasks(void)
{
	__Scheduled1ms();
    __Scheduled10ms();
    __Scheduled100ms();
    __Scheduled1s();
}

/**
 * @fn void __Scheduled1s(void)
 * @brief Threads to run every second.
 */
__STATIC_FORCEINLINE void __Scheduled1s(void)
{
	if (1 == Tick_Second())
	{
		if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
		{
			/* Refresh Error */
			Error_Handler();
		}
		shutdown_task();
#if 0
		encrypt_encryption();
		encrypt_decryption();
#endif
#ifdef MODBUS
		/* Local thread */
		if ( ( 1 == modbus_sensors_get_get_data() ) || ( modbus_local_tool_task() != 2 ) )
		{
			modbus_sensors_task();
		}
#endif
		/* Test production */
        if ((0 == Tick_cloud_time_init()) || (1 == test_prod_run_on()))
        {
        	test_prod_task();
        }

		params_check_for_changes();

		static uint8_t last_param_reed = 0;
		/* Sensor Reed parameter is activated  */
		if (params_manteinance_reed_activation() != 0)
		{
			if (0 == last_param_reed)
			{
				last_param_reed = 1;
				MX_GPIO_Enable_Reed_Sensor(1);
				MX_GPIO_Enable_Tamper(1);
			 	if ((1 == params_get_mqtt_dc_on()) && (params_manteinance_reed_activation() != 2))
			 	{
			 		MX_GPIO_Enable_VBackup_Detection(1);
			 	}
			 	else
			 	{
			 		MX_GPIO_Enable_VBackup_Detection(0);
			 	}
			}
		}
		else
		{
			if (1 == last_param_reed)
			{
				last_param_reed = 0;
				MX_GPIO_Enable_Reed_Sensor(0);
				MX_GPIO_Enable_Tamper(0);
				MX_GPIO_Enable_VBackup_Detection(0);
			}
		}

		static uint8_t last_param_alarm = 0;
		if (1 == params_get_input_alarm_sensor())
		{
			if (0 == last_param_alarm)
			{
				last_param_alarm = 1;
				MX_GPIO_Enable_Alarm_Detection(1);
			}
		}
		else
		{
			if (1 == last_param_alarm)
			{
				last_param_alarm = 0;
				MX_GPIO_Enable_Alarm_Detection(0);
			}
		}

		static uint8_t init_pulses = 0;
		if (1 == params_pulses_on())
		{
			if (0 == init_pulses)
			{
				pulses_init();
				init_pulses = 1;
			}
//			i2c_sensor_task();
		}
		else if (1 == init_pulses)
		{
			pulses_deinit();
			init_pulses = 0;
		}
		static uint8_t init_generic_sensor = 0;
		if (params_input_pulse_as_sensor_get_num() != 0)
		{
			if (0 == init_generic_sensor)
			{
				generic_sensor_init();
				init_generic_sensor = 1;
			}
		}
		if (params_input_pulse_as_sensor_get_num() != 0)
		{
			generic_sensor_task();
		}
	}
}

/**
 * @fn void __Scheduled100ms(void)
 * @brief Threads to run every 100ms.
 *
 * @pre
 * @post
 */
__STATIC_FORCEINLINE void __Scheduled100ms(void)
{
	if (1 == Tick_100Miliseconds())
	{
		leds_FSM();
//		static uint8_t init_generic_sensor = 0;
//		if (params_input_pulse_as_sensor_get_num() != 0)
//		{
//			if (0 == init_generic_sensor)
//			{
//				generic_sensor_init();
//				init_generic_sensor = 1;
//			}
//		}
//		if (params_input_pulse_as_sensor_get_num() != 0)
//		{
//			generic_sensor_task();
//		}
	}
}

/**
 * @fn void __Scheduled10ms(void)
 * @brief Threads to run every 10ms.
 *
 * @pre
 * @post
 */
__STATIC_FORCEINLINE void __Scheduled10ms(void)
{
	if (1 == Tick_10Miliseconds())
	{
#if 0
		if ( PULSE_RX_ON == pulses_get_pulse_ch1p_rx() ) {
			if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( GPIOB, GPIO_PIN_4 ) ) {
				pulses_set_write_record(1);
			}
		}
		if ( PULSE_RX_ON == pulses_get_pulse_ch2p_rx() ) {
			if ( GPIO_PIN_RESET == HAL_GPIO_ReadPin( GPIOC, GPIO_PIN_14 ) ) {
				pulses_set_write_record(1);
			}
		}
#ifdef EXT_SENSOR
		AD_FSM();
#endif
		__testMain();
#endif
	}
}

/**
 * @fn void __Scheduled1ms(void)
 * @brief Threads to run every ms.
 *
 * @pre
 * @post
 */
__STATIC_FORCEINLINE void __Scheduled1ms(void)
{
	if (1 == Tick_1Miliseconds())
	{
#if 0
#ifdef EXT_SENSOR
		AD_FSM();
#endif
#endif
		if (1 == params_pulses_on())
		{
			i2c_sensor_task();
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
	{
#ifdef DEBUG_MODE
		LOGLIVE(LEVEL_1, "LOGLIVE> CPU> ERROR HANDLER!!\r\n");
#endif
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
#ifdef DEBUG_MODE
	LOGLIVE(LEVEL_1, "Wrong parameters value: file %s on line %d\r\n", file, (int) line);
#endif
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
