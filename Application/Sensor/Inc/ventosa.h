/**
  ******************************************************************************
  * @file            ventosa.h
  * @brief           Header file for the HAWLE VENTOSA device. This driver
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VENTOSA_H_
#define __VENTOSA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include <stdio.h>

/** @addtogroup VENTOSA_Driver
  * @{
  */

/** @defgroup VENTOSA_types VENTOSA_types
  * @{
  */

/**
 * @enum kf5013_status_t
 * @brief Enumeration representing the status of the KF5013 sensor.
 */
typedef enum
{
	KF5013_WATER, /**< KF5013 sensor status: Water */
	KF5013_AIR    /**< KF5013 sensor status: Air */
} kf5013_status_t;

/**
 * @struct ventosa_t
 * @brief Structure for storing data from various sensors.
 */
typedef struct
{
    struct
    {
        uint32_t data; /**< Data collected from the PT5414 sensor (4-20mA sensor). */
    } pt5414; /**< PT5414 sensor data. */

    struct
    {
        kf5013_status_t data; /**< Data collected from the KF5013 sensor (Water or Air status). */
    } kf5013; /**< KF5013 sensor data. */

    struct
    {
        uint32_t data; /**< Data collected from the PICO15 sensor. */
    } pico15; /**< PICO15 sensor data. */

    uint32_t warmup_time; /**< Time in milliseconds used to source and read sensors. */
} ventosa_t;

/**
  * @}
  */

/** @addtogroup VENTOSA_driver_Functions
  * @{
  */

/** @addtogroup VENTOSA_SetGet_functions
  * @{
  */
void					ventosa_set_pt5414_data(uint32_t _value);
uint32_t				ventosa_get_pt5414_data(void);
void					ventosa_set_kf5013_data(kf5013_status_t _status);
uint32_t     			ventosa_get_kf5013_data(void);
void					ventosa_set_pico15_data(uint32_t _value);
uint32_t				ventosa_get_pico15_data(void);
void 					ventosa_set_warmup_time(uint32_t _value);
uint32_t				ventosa_get_warmup_time(void);
/**
  * @}
  */

/** @addtogroup VENTOSA_peripherals_InitDeInit_functions
  * @{
  */
void ventosa_GPIO_Init(void);
void ventosa_GPIO_DeInit(void);
void ADC_PT5414_Init(void);
void ADC_PICO15_Init(void);
void ADC_PT5414_DeInit(void);
void ADC_PICO15_DeInit(void);
/**
  * @}
  */

/** @addtogroup VENTOSA_control_functions
  * @{
  */
uint32_t ventosa_read_pt5414(void);
uint32_t ventosa_read_kf5013(void);
uint32_t ventosa_read_pico15(void);
void ventosa_enable(void);
void ventosa_disable(void);
/**
  * @}
  */
/**
  * @}
  */

/** @addtogroup VENTOSA_main_functions
  * @{
  */
void ventosa_read(void);
void ventosa_read_sensor_pt5414(void);
void ventosa_read_sensor_pico15(void);
void ventosa_read_sensor_kf5013(void);
void ventosa_enable_sensors(void);
void ventosa_disable_sensors(void);
/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __VENTOSA_H_ */
