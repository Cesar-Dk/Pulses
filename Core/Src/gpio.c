/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "params.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC14-OSC32_IN (PC14)   ------> RCC_OSC32_IN
     PC15-OSC32_OUT (PC15)   ------> RCC_OSC32_OUT
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PB3 (JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, UC_SMPS_ENABLE_Pin|UC_MODBUS_ENABLE_Pin|UC_SPI_MEMORY_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UC_MODBUS_DRV_Pin|UC_NBIOT_UART_CTS_Pin|UC_NBIOT_UART_RTS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UC_LED_BLUE_Pin|UC_SECURE_ENABLER_Pin|UC_SPI_SS_Pin
                          |UC_SENSOR_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = UC_SENSOR_REED_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(UC_SENSOR_REED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UC_SMPS_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(UC_SMPS_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UC_MODBUS_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UC_MODBUS_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = UC_TAMPER_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(UC_TAMPER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UC_MODBUS_DRV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UC_MODBUS_DRV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = UC_PULSES_CH1P_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(UC_PULSES_CH1P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = UC_PULSES_CH1D_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(UC_PULSES_CH1D_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin */
  GPIO_InitStruct.Pin = UC_LED_BLUE_Pin|UC_SECURE_ENABLER_Pin|UC_SPI_SS_Pin|UC_NBIOT_ENABLE_Pin
                          |UC_SENSOR_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
//  GPIO_InitStruct.Pin = UC_PULSES_TAMP_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  HAL_GPIO_Init(UC_PULSES_TAMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UC_NBIOT_PWRMON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UC_NBIOT_PWRMON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = UC_NBIOT_UART_CTS_Pin|UC_NBIOT_UART_RTS_Pin|UC_NBIOT_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UC_SPI_MEMORY_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UC_SPI_MEMORY_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

//  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

//  HAL_NVIC_SetPriority(EXTI6_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI6_IRQn);

//  HAL_NVIC_SetPriority(EXTI7_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI7_IRQn);

//  HAL_NVIC_SetPriority(EXTI8_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI8_IRQn);

//  HAL_NVIC_SetPriority(EXTI13_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

}

/* USER CODE BEGIN 2 */
/**
  * @brief Configures PowerMon pin to work as interrupt or general input.
  * @param enable: Specifies the operation mode.
  * 			0 -> general input.
  * 			1 -> falling edge interrupt.
  * @retval None
  */
void MX_GPIO_Enable_Power_Mon( uint8_t enable )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = UC_NBIOT_PWRMON_Pin;
	if ( 1 == enable )
	{
		__disable_irq();
		HAL_NVIC_SetPriority(EXTI8_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI8_IRQn);
		HAL_GPIO_DeInit(UC_NBIOT_PWRMON_GPIO_Port, UC_NBIOT_PWRMON_Pin);
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(UC_NBIOT_PWRMON_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		HAL_GPIO_DeInit(UC_NBIOT_PWRMON_GPIO_Port, UC_NBIOT_PWRMON_Pin);
		GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull  = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(UC_NBIOT_PWRMON_GPIO_Port, &GPIO_InitStruct);
	}
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__enable_irq();
}

/**
  * @brief Enable/Disable Reed Sensor pin to work as interrupt or as analog input.
  * @param enable: Specifies the operation.
  * 			0 -> DISABLE, analog input.
  * 			1 -> ENABLE, falling edge interrupt with pull up.
  * @retval None
  */
void MX_GPIO_Enable_Reed_Sensor( uint8_t enable )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = UC_SENSOR_REED_Pin;
	if ( 1 == enable )
	{
		__disable_irq();
		HAL_NVIC_SetPriority(EXTI13_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI13_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(UC_SENSOR_REED_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI13_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(UC_SENSOR_REED_GPIO_Port, &GPIO_InitStruct);
	}
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__enable_irq();
}

/**
  * @brief Enable/Disable Tamper input pin to work as interrupt or as analog input.
  * @param enable: Specifies the operation mode.
  * 			0 -> DISABLE, analog input.
  * 			1 -> ENABLE, falling edge interrupt.
  * @retval None
  */
void MX_GPIO_Enable_Tamper( uint8_t enable )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = UC_TAMPER_Pin;
	if ( 1 == enable )
	{
		__disable_irq();
		HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(UC_TAMPER_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(UC_TAMPER_GPIO_Port, &GPIO_InitStruct);
	}
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__enable_irq();
}

/**
  * @brief Enable/Disable Tamper input pin to work as interrupt or as analog input.
  * @param enable: Specifies the operation mode.
  * 			0 -> DISABLE, analog input.
  * 			1 -> ENABLE, falling edge interrupt.
  * @retval None
  */
void MX_GPIO_Enable_VBackup_Detection( uint8_t enable )
{
#ifndef DEBUG_MODE
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (1 == enable)
	{
		/*Configure GPIO pin : PtPin */
		GPIO_InitStruct.Pin   = UC_AC_DETECTION_Pin;
		GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull  = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(UC_AC_DETECTION_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		/*Configure GPIO pin : PtPin */
		GPIO_InitStruct.Pin   = UC_AC_DETECTION_Pin;
		GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull  = GPIO_PULLUP;
		HAL_GPIO_Init(UC_AC_DETECTION_GPIO_Port, &GPIO_InitStruct);
	}
#endif
}

/**
  * @brief Enable/Disable Alarm Sensor pin to work as interrupt or as analog input.
  * @param enable: Specifies the operation.
  * 			0 -> DISABLE, analog input.
  * 			1 -> ENABLE, falling edge interrupt with pull up.
  * @retval None
  */
void MX_GPIO_Enable_Alarm_Detection( uint8_t enable )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = UC_PULSES_TAMP_Pin;
	if ( 1 == enable )
	{
		__disable_irq();
		HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;//GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(UC_PULSES_TAMP_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(UC_PULSES_TAMP_GPIO_Port, &GPIO_InitStruct);
	}
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__enable_irq();
}

/**
  * @brief Enable/Disable PULSES CH1P input pin to work as interrupt or as analog input.
  * @param enable: Specifies the operation mode.
  * 			0 -> DISABLE, analog input.
  * 			1 -> ENABLE, falling edge interrupt with pull up.
  * @retval None
  */
void MX_GPIO_Enable_Pulses_CH1P( uint8_t enable )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = UC_PULSES_CH1P_Pin;
	if ( 1 == enable )
	{
		HAL_NVIC_SetPriority(EXTI6_IRQn, 2, 2);
		HAL_NVIC_EnableIRQ(EXTI6_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;//GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		__HAL_RCC_GPIOA_CLK_ENABLE();
		HAL_GPIO_Init(UC_PULSES_CH1P_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI6_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		__HAL_RCC_GPIOA_CLK_ENABLE();
		HAL_GPIO_Init(UC_PULSES_CH1P_GPIO_Port, &GPIO_InitStruct);
	}
	__HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief Enable/Disable PULSES CH1D input pin to work as interrupt or as analog input.
  * @param enable: Specifies the operation mode.
  * 			0 -> DISABLE, analog input.
  * 			1 -> ENABLE, general purpose input with pull up.
  * @retval None
  */
void MX_GPIO_Enable_Pulses_CH1D( uint8_t enable )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin  = UC_PULSES_CH1D_Pin;
	if ( 1 == enable )
	{
		HAL_NVIC_SetPriority(EXTI7_IRQn, 3, 3);
		HAL_NVIC_EnableIRQ(EXTI7_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		__HAL_RCC_GPIOA_CLK_ENABLE();
		HAL_GPIO_Init(UC_PULSES_CH1D_GPIO_Port, &GPIO_InitStruct);
	}
	else
	{
		HAL_NVIC_DisableIRQ(EXTI7_IRQn);
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		__HAL_RCC_GPIOA_CLK_ENABLE();
		HAL_GPIO_Init(UC_PULSES_CH1D_GPIO_Port, &GPIO_InitStruct);
	}
	__HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief Enable Analog sensor power supply
  * @retval None
  */
void MX_GPIO_Enable_Sensor( void )
{
	HAL_GPIO_WritePin(UC_SENSOR_ENABLE_GPIO_Port, UC_SENSOR_ENABLE_Pin, GPIO_PIN_SET);
}

/**
  * @brief Disable Analog sensor power rail supply
  * @retval None
  */
void MX_GPIO_Disable_Sensor(void)
{
	HAL_GPIO_WritePin(UC_SENSOR_ENABLE_GPIO_Port, UC_SENSOR_ENABLE_Pin, GPIO_PIN_RESET);
}

void MX_GPIO_Relay_Activation( uint32_t _act )
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_DeInit(UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin);

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin   = UC_PULSES_TAMP_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(UC_PULSES_TAMP_GPIO_Port, &GPIO_InitStruct);

	( 0 == _act )?HAL_GPIO_WritePin(UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin, GPIO_PIN_RESET):HAL_GPIO_WritePin(UC_PULSES_TAMP_GPIO_Port, UC_PULSES_TAMP_Pin, GPIO_PIN_SET);

	if (1 == params_get_input_alarm_sensor())
	{
		MX_GPIO_Enable_Alarm_Detection(1);
	}
}
/* USER CODE END 2 */
