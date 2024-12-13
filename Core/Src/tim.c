/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "serial_une82326.h"
//#include "stm32l4xx_hal_uart_emul.h"
#include "une82326.h"
#include "params.h"
/* Prescaler declaration */
__IO uint32_t uwPrescalerValue = 0;
__IO uint32_t time_counter     = 0;
__IO uint32_t unselected       = 0;
__IO uint8_t unselections      = 0;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 6;//24;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
/* TIM16 init function */
void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV8;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  if (HAL_TIMEx_TISelection(&htim16, TIM_TIM16_TI1_LSE, TIM_CHANNEL_1) != HAL_OK)
  {
	/* Initialization Error */
	Error_Handler();
  }
  /* USER CODE END TIM16_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

//  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspInit 0 */

  /* USER CODE END TIM16_MspInit 0 */
    /* TIM16 clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();

//    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM16 GPIO Configuration
    PB8     ------> TIM16_CH1
    */
//    GPIO_InitStruct.Pin = GPIO_PIN_8;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF14_TIM16;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM16 interrupt Init */
    HAL_NVIC_SetPriority(TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE BEGIN TIM16_MspInit 1 */

  /* USER CODE END TIM16_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM16)
  {
  /* USER CODE BEGIN TIM16_MspDeInit 0 */

  /* USER CODE END TIM16_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM16_CLK_DISABLE();

    /**TIM16 GPIO Configuration
    PB8     ------> TIM16_CH1
    */
//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    /* TIM16 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM16_IRQn);
  /* USER CODE BEGIN TIM16_MspDeInit 1 */

  /* USER CODE END TIM16_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* TIM3 init function */
void MX_TIM3_Custom_Init(void)
{
	/*##-1- Configure the TIM peripheral #######################################*/
	/* -----------------------------------------------------------------------
	    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1),
	    since APB1 prescaler is equal to 1.
	      TIM3CLK = PCLK1
	      PCLK1 = HCLK
	      => TIM3CLK = HCLK = SystemCoreClock
	    To get TIM3 counter clock at 1 MHz, the Prescaler is computed as following:
	    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	    Prescaler = (SystemCoreClock /1 MHz) - 1

	    Note:
	     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32l4xx.c file.
	     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
	     variable value. Otherwise, any configuration based on this variable will be incorrect.
	     This variable is updated in three ways:
	      1) by calling CMSIS function SystemCoreClockUpdate()
	      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
	      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
	  ----------------------------------------------------------------------- */

	/* Compute the prescaler value to have TIMx counter clock equal to 1000000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 320000) - 1;

	/* Set TIMx instance */
	htim3.Instance = TIM3;

	/* Initialize TIMx peripheral as follows:
	       + Period = 25 - 1
	       + Prescaler = (SystemCoreClock/1000000) - 1
	       + ClockDivision = 0
	       + Counter direction = Up

	       (Experimental value: 64,2 us.)
	 */
	htim3.Init.Period            = 4 - 1;
	htim3.Init.Prescaler         = uwPrescalerValue;
	htim3.Init.ClockDivision     = 0;
	htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim3.Init.RepetitionCounter = 0;

	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

//	/*##-2- Start the TIM Base generation in interrupt mode ####################*/
//	/* Start Channel1 */
//	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
//	{
//		/* Starting Error */
//		Error_Handler();
//	}
}

#ifdef UNE82326
static void __timerStateMachine( void )
{
#define TIME_1_25_SEC (100000) //1,25s/12,5us
#define TIME_1_5_SEC  (120000) //1,5s/12,5us

	static enum {
		WAIT_INTERRUPT,
		INTERRUPT_DETECTED,
		SEL_OFF,
		SEL_ON
	} timer_status = WAIT_INTERRUPT;

	switch (timer_status)
	{
	case WAIT_INTERRUPT:
		if ( 1 == serial_une82326_get_start_bit() ) {
			timer_status = INTERRUPT_DETECTED;
		}
		break;
	case INTERRUPT_DETECTED:
		if ( 1 == serial_une82326_get_start_bit() ) {
			if ( time_counter++ == 1 ) {
				// Disable UNE_SEL line. Bus arbitration.
				serial_une82326_unsel();
				unselections++;
				timer_status = SEL_OFF;
			}
		}
		break;
	case SEL_OFF:
		if ( 1 == serial_une82326_get_start_bit() ) {
			if ( time_counter++ >= TIME_1_25_SEC ) {
				serial_une82326_sel();
				time_counter = 0;
				timer_status = WAIT_INTERRUPT;
			}
		} else {
			time_counter = 0;
			timer_status = SEL_ON;
		}
		break;
	case SEL_ON:
		if ( 1 == serial_une82326_get_start_bit() ) {
			time_counter = 0;
			timer_status = INTERRUPT_DETECTED;
		} else if ( time_counter++ >= TIME_1_5_SEC ) {
			time_counter = 0;
			serial_une82326_unsel();
			une82326_set_last_device(1);
			UART_Emul_DMADisableRcpt();
			if ( HAL_TIM_Base_Stop(&htim3) != HAL_OK )
			{
				Error_Handler();
			}
		}
		break;
	default:
		break;
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3) {
		__timerStateMachine();
#if 0
		if ( 1 == serial_une82326_get_start_bit() ) {
			if ( 0 == unselected ) {
				if ( time_counter++ == 1 ) {
					// Disable UNE_SEL line. Bus arbitration.
					serial_une82326_unsel();
					unselections++;
				}
			}
		} else {
			if ( time_counter++ >= 120000 ) {
				time_counter = 0;
//				unselected   = 0;
				serial_une82326_unsel();
//#ifdef UART_EMUL
//				UART_Emul_DMADisableRcpt();
//#else
//				serial_une82326_set_RC_FLAG(1);
//#endif
				une82326_set_last_device(1);
				if ( HAL_TIM_Base_Stop(&htim3) != HAL_OK )
				{
					Error_Handler();
				}
			}
		}
#endif
	}
}
#endif

/* USER CODE END 1 */
