/**
  ******************************************************************************
  * @file            ventosa.c
  * @brief           Source file for the HAWLE VENTOSA device. This driver
  * contains the necessary functions to retrieve the client specified data
  * from the VENTOSA device sensors:
  * 	@arg **PT5414** -> 4.20 pressure sensor.
  * 	@arg **KF5013** -> Open collector air-water capacitive sensor.
  * 	@arg **PICO15** -> 4.20 ultrasound distance sensor.
  *
  * @author			 Jorge Benedicto Centeno.
  * @date            20/10/23
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Datakorum.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ventosa.h"
#include "ad.h"
#include "battery.h"

/* External global variables -------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

/* Private variables ---------------------------------------------------------*/
ventosa_t ventosa;

/** @defgroup VENTOSA_Driver VENTOSA_Driver
 *  @brief HAWLE VENTOSA sensors driver.
  * @{
  */

/** @defgroup VENTOSA_driver_Functions VENTOSA_driver_Functions
  * @brief    Driver operation functions.
  * @{
  */

/** @defgroup VENTOSA_SetGet_functions VENTOSA_SetGet_functions
  * @brief    Ventosa_t structure get/set functions.
  * @{
  */

/**
 * @fn void ventosa_set_pt5414_data(ventosa_t* _ventosa, uint32_t _value)
 * @brief Sets the data collected from the PT5414 sensor (4-20mA sensor) connected to ADC4 CH19, PULSES.TAMP.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post The PT5414 data in the ventosa_t structure will be updated with the specified '_value'.
 * @param _ventosa A pointer to a ventosa_t structure to store the data.
 * @param _value The uint32_t value to set as PT5414 data.
 */
void ventosa_set_pt5414_data(uint32_t _value)
{
	ventosa.pt5414.data = _value;
}

/**
 * @fn uint32_t ventosa_get_pt5414_data(ventosa_t* _ventosa)
 * @brief Retrieves the data collected from the PT5414 sensor (4-20mA sensor) connected to ADC CH19, PULSES.TAMP.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post None.
 * @param _ventosa A pointer to a ventosa_t structure from which to retrieve the PT5414 data.
 * @return A uint32_t value representing the PT5414 sensor data.
 */
uint32_t ventosa_get_pt5414_data(void)
{
	return ventosa.pt5414.data;
}

/**
 * @fn void ventosa_set_kf5013_data(ventosa_t* _ventosa, kf5013_status_t _status)
 * @brief Sets the data collected from the KF5013 sensor (water or air status) in a ventosa_t structure. KF5013 sensor
 * connected to GPIO input pin PULSES.CH1D with a internal pull-up.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post The KF5013 data in the ventosa_t structure will be updated with the specified '_status'.
 * @param _ventosa A pointer to a ventosa_t structure to store the KF5013 sensor data.
 * @param _status The status (KF5013_WATER or KF5013_AIR) to set in the KF5013 data.
 */
void ventosa_set_kf5013_data(kf5013_status_t _status)
{
	ventosa.kf5013.data = _status;
}

/**
 * @fn kf5013_status_t ventosa_get_kf5013_data(ventosa_t* _ventosa)
 * @brief Retrieves the data collected from the KF5013 sensor (water or air status) in a ventosa_t structure. KF5013 sensor
 * connected to GPIO input pin PULSES.CH1D with a internal pull-up.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post None.
 * @param _ventosa A pointer to a ventosa_t structure from which to retrieve the KF5013 sensor data.
 * @return A kf5013_status_t value representing the KF5013 sensor status (KF5013_WATER or KF5013_AIR).
 */
uint32_t ventosa_get_kf5013_data(void)
{
	return ventosa.kf5013.data;
}

/**
 * @fn void ventosa_set_pico15_data(ventosa_t* _ventosa, uint32_t _value)
 * @brief Sets the data collected from the PICO15 sensor in a ventosa_t structure. PICO15 sensor connected to ADC4 CH11 PULSES.CH1P.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post The PICO15 data in the ventosa_t structure will be updated with the specified '_value'.
 * @param _ventosa A pointer to a ventosa_t structure to store the PICO15 sensor data.
 * @param _value The uint32_t value to set as PICO15 data.
 */
void ventosa_set_pico15_data(uint32_t _value)
{
	ventosa.pico15.data = _value;
}

/**
 * @fn uint32_t ventosa_get_pico15_data(ventosa_t* _ventosa)
 * @brief Retrieves the data collected from the PICO15 sensor in a ventosa_t structure. PICO15 sensor connected to ADC4 CH11 PULSES.CH1P.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post None.
 * @param _ventosa A pointer to a ventosa_t structure from which to retrieve the PICO15 sensor data.
 * @return A uint32_t value representing the PICO15 sensor data.
 */
uint32_t ventosa_get_pico15_data(void)
{
	return ventosa.pico15.data;
}

/**
 * @fn void ventosa_set_warmup_time(ventosa_t* _ventosa, uint32_t _value)
 * @brief Sets the warm-up time for a ventosa_t structure.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post The warm-up time in the ventosa_t structure will be updated with the specified '_value'.
 * @param _ventosa A pointer to a ventosa_t structure to store the warm-up time.
 * @param _value The uint32_t value to set as the warm-up time.
 */
void ventosa_set_warmup_time(uint32_t _value)
{
	ventosa.warmup_time = _value;
}

/**
 * @fn uint32_t ventosa_get_warmup_time(ventosa_t* _ventosa)
 * @brief Retrieves the warm-up time from a ventosa_t structure.
 * @pre The '_ventosa' pointer must be a valid pointer to a ventosa_t structure.
 * @post None.
 * @param _ventosa A pointer to a ventosa_t structure from which to retrieve the warm-up time.
 * @return A uint32_t value representing the warm-up time.
 */
uint32_t ventosa_get_warmup_time(void)
{
	return ventosa.warmup_time;
}

/**
  * @}
  */

/** @defgroup VENTOSA_peripherals_InitDeInit_functions VENTOSA_peripherals_InitDeInit_functions
  * @brief    Peripherals Init and DeInit functions
  *
@verbatim
 ===============================================================================
                      	  	  ##### Peripherals #####
 ===============================================================================
    [..]  This functions are used to Init and deInit the following PINS:

      (+) **PULSES.CH1P**. Configured as the ADC4 CH19.
      (+) **PULSES.CH1D**. Configured as a GPIO Input pin with a internal pull-up.
      (+) **PULSES.TAMP**. Configured as the ADC4 CH11.

@endverbatim
  * @{
  */

/**
 * @fn void ventosa_GPIO_Init(void)
 * @brief GPIO configuration for the Ventosa device.
 *
 * This function configures the GPIO pins for the Ventosa device, including enabling the clock for
 * the required GPIO port, setting the pin modes.
 *
 * @verbatim
 ===============================================================================
                      	  ##### GPIO Configuration #####
 ===============================================================================
      (+) **PULSES.CH1P - PA6** Configured as GPIO analog.
      (+) **PULSES.CH1D - PA7** Configured as GPIO Input with a pull-up.
      (+) **PULSES.TAMP - PB1** Configured as GPIO analog.
      (+) **MBUS_ENANLE - PH0** Configured as GPIO Output PP.
 * @endverbatim
 *
 * @pre None.
 * @post GPIO pins are configured for the Ventosa device as specified in this function.
 */
void ventosa_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Enable clocks.
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	// Set the MBUS enable pin to GPIO_PIN_RESET
	HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_RESET);

	// Configure the UC_MBUS_ENABLE pin
	GPIO_InitStruct.Pin = UC_SMPS_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(UC_SMPS_ENABLE_GPIO_Port, &GPIO_InitStruct);

	// Configure the PULSES_CH1D pin for input
	GPIO_InitStruct.Pin = UC_PULSES_CH1D_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(UC_PULSES_CH1D_GPIO_Port, &GPIO_InitStruct);

	// Configure the PULSES_CH1P pin for analog mode
	GPIO_InitStruct.Pin = UC_PULSES_CH1P_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(UC_PULSES_CH1P_GPIO_Port, &GPIO_InitStruct);

	// Configure the PULSES_TAMP pin for analog mode
	GPIO_InitStruct.Pin = UC_PULSES_TAMP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(UC_PULSES_TAMP_GPIO_Port, &GPIO_InitStruct);

}

/**
 * @fn void ventosa_GPIO_DeInit(void)
 * @brief Deinitializes the GPIO configuration for the VENTOSA device.
 *
 * This function deinitializes the GPIO pins previously configured for the Ventosa device. It is
 * typically used to release resources and reset the GPIO pins to their default state.
 *
 * @pre The GPIO pins must have been previously initialized using ventosa_GPIO_Init or a similar
 * configuration function.
 * @post GPIO pins are deinitialized and reset to their default state.
 */
void ventosa_GPIO_DeInit(void)
{
	// Deinitialize the specified GPIO pins
	HAL_GPIO_DeInit(UC_PULSES_CH1P_GPIO_Port, UC_PULSES_CH1P_Pin);
	HAL_GPIO_DeInit(UC_PULSES_CH1D_GPIO_Port, UC_PULSES_CH1D_Pin);
	HAL_GPIO_DeInit(UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin);
}

/**
 * @fn void ADC_PT5414_Init(void)
 * @brief Initializes the ADC (Analog-to-Digital Converter) for the PT5414 sensor.
 *
 * This function configures and initializes the ADC settings for the PT5414 sensor. It sets up
 * parameters like resolution, sampling time, and channel to ensure accurate data conversion.
 *
 * @pre The necessary HAL configuration and resources must be set up before calling this function.
 * ventosa_GPIO_Init() needs to be called before launching this function.
 * @post The ADC for the PT5414 sensor is configured and ready for data conversion.
 */
void ADC_PT5414_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};

	  // Enable clock
	  __HAL_RCC_PWR_CLK_ENABLE();
	  // Enable power and voltage domains
	  HAL_PWREx_EnableVddA();
	  HAL_PWREx_EnableVddIO2();

	  /** Common config
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.GainCompensation = 0;
	  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	  hadc1.Init.OversamplingMode = DISABLE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  // Initialize the ADC
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  // Configure the ADC channel for PT5414 sensor
	  sConfig.Channel = ADC_CHANNEL_16;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_814CYCLES;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

/**
 * @fn void ADC_PICO15_Init(void)
 * @brief Initializes the ADC (Analog-to-Digital Converter) for the PICO15 sensor.
 *
 * This function configures and initializes the ADC settings for the PICO15 sensor. It sets up
 * parameters like resolution, sampling time, and channel to enable accurate data conversion.
 *
 * @pre The necessary HAL configuration and resources must be set up before calling this function.
 * ventosa_GPIO_Init() needs to be called before launching this function.
 * @post The ADC for the PICO15 sensor is configured and ready for data conversion.
 */
void ADC_PICO15_Init(void)
{
	  ADC_ChannelConfTypeDef sConfig = {0};

	  // Enable clock
	  __HAL_RCC_PWR_CLK_ENABLE();
	  // Enable power and voltage domains
	  HAL_PWREx_EnableVddA();
	  HAL_PWREx_EnableVddIO2();

	  // Initialize ADC settings
	  /** Common config
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.GainCompensation = 0;
	  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
	  hadc1.Init.OversamplingMode = DISABLE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  // Initialize the ADC
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  // Configure the ADC channel for PICO15 sensor
	  sConfig.Channel = ADC_CHANNEL_11;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_814CYCLES;
	  sConfig.SingleDiff = ADC_SINGLE_ENDED;
	  sConfig.OffsetNumber = ADC_OFFSET_NONE;
	  sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/**
 * @fn void ADC_PT5414_DeInit(void)
 * @brief Deinitializes the ADC (Analog-to-Digital Converter) for the PT5414 sensor.
 *
 * This function deinitializes the ADC settings for the PT5414 sensor.
 *
 * @pre The ADC for the PT5414 sensor must have been previously initialized using ADC_PT5414_Init
 * or a similar configuration function.
 * @post The ADC for the PT5414 sensor is deinitialized and reset to its default state.
 */
void ADC_PT5414_DeInit(void)
{
	ADC_HandleTypeDef* adcHandle = &hadc1;
	if(adcHandle->Instance == ADC1) {
		HAL_ADC_DeInit(&hadc1);
	}
}

/**
 * @fn void ADC_PICO15_DeInit(void)
 * @brief Deinitializes the ADC (Analog-to-Digital Converter) for the PICO15 sensor.
 *
 * This function deinitializes the ADC settings for the PICO15 sensor.
 *
 * @pre The ADC for the PICO15 sensor must have been previously initialized using ADC_PICO15_Init
 * or a similar configuration function.
 * @post The ADC for the PICO15 sensor is deinitialized and reset to its default state.
 */
void ADC_PICO15_DeInit(void)
{
	ADC_HandleTypeDef* adcHandle = &hadc1;
	if(adcHandle->Instance == ADC1) {
		HAL_ADC_DeInit(&hadc1);
	}
}

/**
  * @}
  */

/** @defgroup VENTOSA_control_functions VENTOSA_control_functions
  * @brief This group of functions is used to simplify the use use HAL libraries
  *  for the VENTOSA application.
  * @{
  */

/**
 * @fn uint32_t ventosa_read_pt5414(void)
 * @brief Reads data from the PT5414 sensor using the ADC.
 *
 * This function performs the necessary steps to read data from the PT5414 sensor using the ADC.
 * It starts with calibration, then starts the conversion, and waits for the conversion to complete.
 * Finally, it retrieves and returns the ADC value representing the PT5414 sensor data.
 *
 * @pre The ADC for the PT5414 sensor must have been properly initialized using ADC_PT5414_Init
 * or a similar configuration function.
 * @post The PT5414 sensor data is read, and the ADC is in a state ready for the next conversion.
 *
 * @return A uint32_t value representing the data collected from the PT5414 sensor.
 */
uint32_t ventosa_read_pt5414(void)
{

	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		/* Start Conversation Error */
		Error_Handler();
	}

	if (HAL_ADC_PollForConversion(&hadc1, 5000) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		Error_Handler();
	}
	__IO uint32_t Vdda = __LL_ADC_CALC_DATA_TO_VOLTAGE(ADC1, battery_get_Vinst(), HAL_ADC_GetValue(&hadc1),  LL_ADC_RESOLUTION_14B);
	return (Vdda);
//	return (AD_LSB * HAL_ADC_GetValue(&hadc1));
}

/**
 * @fn uint32_t ventosa_read_kf5013(void)
 * @brief Reads data from the KF5013 sensor using GPIO input.
 *
 * This function reads the state of a GPIO pin connected to the KF5013 sensor and determines
 * the sensor's status (Air or Water) based on the pin's state. It returns the corresponding status.
 * If the pin's state is undefined, it returns -1 to indicate an error.
 *
 * @pre The GPIO pin for the KF5013 sensor must have been properly configured using ventosa_GPIO_Init
 * or similar configuration functions.
 * @post The KF5013 sensor status is read, and the GPIO remains in its current state.
 *
 * @return A value representing the status of the KF5013 sensor: KF5013_AIR, KF5013_WATER, or -1 for an error.
 */
uint32_t ventosa_read_kf5013(void)
{
	switch (HAL_GPIO_ReadPin(UC_PULSES_CH1D_GPIO_Port, UC_PULSES_CH1D_Pin))
	{
	case GPIO_PIN_SET:
		return KF5013_AIR;
		break;

	case GPIO_PIN_RESET:
		return KF5013_WATER;
		break;

	default:
		return -1;
		break;
	}
}

/**
 * @fn uint32_t ventosa_read_pico15(void)
 * @brief Reads data from the PICO15 sensor using the ADC.
 *
 * This function performs the necessary steps to read data from the PICO15 sensor using the ADC.
 * It starts with calibration, then starts the conversion, and waits for the conversion to complete.
 * Finally, it retrieves and returns the ADC value representing the PICO15 sensor data.
 *
 * @pre The ADC for the PICO15 sensor must have been properly initialized using ADC_PICO15_Init
 * or a similar configuration function.
 * @post The PICO15 sensor data is read, and the ADC is in a state ready for the next conversion.
 *
 * @return A uint32_t value representing the data collected from the PICO15 sensor.
 */
uint32_t ventosa_read_pico15(void)
{
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
	{
		/* Calibration Error */
		Error_Handler();
	}
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		/* Start Conversation Error */
		Error_Handler();
	}

	if (HAL_ADC_PollForConversion(&hadc1, 5000) != HAL_OK)
	{
		/* End Of Conversion flag not set on time */
		Error_Handler();
	}
	__IO uint32_t Vdda = __LL_ADC_CALC_DATA_TO_VOLTAGE(ADC1, battery_get_Vinst(),HAL_ADC_GetValue(&hadc1),  LL_ADC_RESOLUTION_14B);
	return (Vdda);
//	return (AD_LSB * HAL_ADC_GetValue(&hadc1));
}

/**
 * @fn void ventosa_enable(void)
 * @brief Enables the Ventosa device by setting the enable pin to HIGH.
 *
 * This function enables the Ventosa device by setting the enable pin to HIGH using GPIO.
 * GPIO Pin used to enable is MBUS_ENABLE.
 * When MBUS_ENABLE is high it activates a step-up DC/DC converter from 3.3V to 14 V. This voltage
 * is used to source the Ventosa sensors.
 *
 * @pre The GPIO pin for device enable must have been properly configured using ventosa_GPIO_Init
 * or similar configuration functions.
 * @post The Ventosa device is enabled, allowing it to perform its functions.
 */
void ventosa_enable(void)
{
	HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_SET);
}

/**
 * @fn void ventosa_disable(void)
 * @brief Disables the Ventosa device by setting the enable pin to LOW.
 *
 * This function disables the Ventosa device by setting the enable pin to LOW using GPIO.
 * GPIO Pin used to disable is MBUS_ENABLE.
 * When MBUS_ENABLE is high it deactivates a step-up DC/DC converter from 3.3V to 14 V. This voltage
 * is used to source the Ventosa sensors.
 *
 * @pre The GPIO pin for device enable must have been properly configured using ventosa_GPIO_Init
 * or similar configuration functions.
 * @post The Ventosa device is disabled, preventing it from performing its functions.
 */
void ventosa_disable(void)
{
	HAL_GPIO_WritePin(UC_SMPS_ENABLE_GPIO_Port, UC_SMPS_ENABLE_Pin, GPIO_PIN_RESET);
}

/**
  * @}
  */

/** @defgroup VENTOSA_main_functions VENTOSA_main_functions
  * @brief High level driver functions.
  * @{
  */

/**
 * @fn void ventosa_read(ventosa_t *_ventosa)
 * @brief Reads data from Ventosa sensors, performs warm-up, and updates the Ventosa structure.
 *
 * This function is responsible for reading data from various sensors connected to the Ventosa device.
 * It performs sensor-specific initialization, warm-up, and data collection, and then updates the
 * Ventosa data structure with the acquired data.
 *
 * @pre The Ventosa device and its associated resources must be properly configured and initialized
 * before calling this function. The Ventosa structure must also be properly initialized.
 * @post The Ventosa structure is updated with sensor data.
 *
 * @param _ventosa ventosa_t* pointer to the Ventosa structure that will be updated with sensor data.
 */
void ventosa_read(void)
{
	// Deinitialize and reinitialize GPIO settings
	ventosa_GPIO_DeInit();
	ventosa_GPIO_Init();
	ventosa_enable();

	// Check and set a valid warm-up time
	if ( (ventosa_get_warmup_time() <= 0) || ventosa_get_warmup_time() > 4000)
	{
		ventosa_set_warmup_time(500);
	}

	// Perform sensors warm-up
	HAL_Delay(ventosa_get_warmup_time());

	// Initialize and read data from PT5414 sensor
	ADC_PT5414_Init();
	ventosa_set_pt5414_data(ventosa_read_pt5414());
	ADC_PT5414_DeInit();

	// Initialize and read data from PICO15 sensor
	ADC_PICO15_Init();
	ventosa_set_pico15_data(ventosa_read_pico15());
	ADC_PICO15_DeInit();

	// Read data from KF5013 sensor
	ventosa_set_kf5013_data(ventosa_read_kf5013());

	// Disable the Ventosa device and deinitialize GPIO settings
	ventosa_disable();
	ventosa_GPIO_DeInit();
	// Restore the GPIO settings
//	MX_GPIO_Init();
}

/**
 * @fn
 * @brief
 * @pre
 * @post
 * @param
 * @param
 */
void ventosa_read_sensor_pt5414(void)
{
#if 0
	// Deinitialize and reinitialize GPIO settings
	ventosa_GPIO_DeInit();
	ventosa_GPIO_Init();
	ventosa_enable();

	// Check and set a valid warm-up time
	if ( (ventosa_get_warmup_time() <= 0) || ventosa_get_warmup_time() > 4000)
	{
		ventosa_set_warmup_time(500);
	}

	// Perform sensors warm-up
	HAL_Delay(ventosa_get_warmup_time());
#endif
	// Initialize and read data from PT5414 sensor
	ADC_PT5414_Init();
	ventosa_set_pt5414_data(ventosa_read_pt5414());
	ADC_PT5414_DeInit();
#if 0
	// Disable the Ventosa device and deinitialize GPIO settings
	ventosa_disable();
	ventosa_GPIO_DeInit();
	// Restore the GPIO settings
//	MX_GPIO_Init();
#endif
}

void ventosa_read_sensor_pico15(void)
{
#if 0
	// Deinitialize and reinitialize GPIO settings
	ventosa_GPIO_DeInit();
	ventosa_GPIO_Init();
	ventosa_enable();

	// Check and set a valid warm-up time
	if ( (ventosa_get_warmup_time() <= 0) || ventosa_get_warmup_time() > 4000)
	{
		ventosa_set_warmup_time(500);
	}

	// Perform sensors warm-up
	HAL_Delay(ventosa_get_warmup_time());
#endif
	// Initialize and read data from PICO15 sensor
	ADC_PICO15_Init();
	ventosa_set_pico15_data(ventosa_read_pico15());
	ADC_PICO15_DeInit();
#if 0
	// Disable the Ventosa device and deinitialize GPIO settings
	ventosa_disable();
	ventosa_GPIO_DeInit();
	// Restore the GPIO settings
//	MX_GPIO_Init();
#endif
}

void ventosa_read_sensor_kf5013(void)
{
#if 0
	// Deinitialize and reinitialize GPIO settings
	ventosa_GPIO_DeInit();
	ventosa_GPIO_Init();
	ventosa_enable();

	// Check and set a valid warm-up time
	if ( (ventosa_get_warmup_time() <= 0) || ventosa_get_warmup_time() > 4000)
	{
		ventosa_set_warmup_time(500);
	}

	// Perform sensors warm-up
	HAL_Delay(ventosa_get_warmup_time());
#endif
	ventosa_set_kf5013_data(ventosa_read_kf5013());
#if 0
	// Disable the Ventosa device and deinitialize GPIO settings
	ventosa_disable();
	ventosa_GPIO_DeInit();
	// Restore the GPIO settings
//	MX_GPIO_Init();
#endif
}

void ventosa_enable_sensors(void)
{
	// Deinitialize and reinitialize GPIO settings
	ventosa_GPIO_DeInit();
	ventosa_GPIO_Init();
	ventosa_enable();

	// Check and set a valid warm-up time
	if ( (ventosa_get_warmup_time() <= 0) || ventosa_get_warmup_time() > 4000)
	{
		ventosa_set_warmup_time(params_get_generic_sensor_warm_time());
	}

	// Perform sensors warm-up
	HAL_Delay(ventosa_get_warmup_time());
}

void ventosa_disable_sensors(void)
{
	// Disable the Ventosa device and deinitialize GPIO settings
	ventosa_disable();
	ventosa_GPIO_DeInit();
	// Restore the GPIO settings
//	MX_GPIO_Init();
}

/**
  * @}
  */

/**
  * @}
  */
/**
  * @}
  */


