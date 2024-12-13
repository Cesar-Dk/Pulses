/**
  ******************************************************************************
  * @file           params.h
  * @author 		Datakorum Development Team
  * @brief          Header file for params.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solution SL.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * params.h
 *
 *  Created on: 15 jul. 2019
 *      Author: Sergio Mill�n L�pez
 */
#ifndef _PARAMS_H
#define _PARAMS_H

#include <time.h>
#include <math.h>

#include "serial_modbus.h"
#include "modbus_sensors.h"
#include "modbus.h"
#include "connection.h"

extern void MX_USART2_UART_Init(void);
extern void MX_USART2_USART_DeInit(void);
uint8_t params_get_loglive_level( void );
void 	params_set_loglive_level( uint8_t _level );
void 	params_loglive(uint32_t level, const char *f,  ...);
void    params_loglive_tamp_en(uint32_t level, const char *f,  ...);
void 	params_loglive_serial(uint32_t level, const char *f,  ...);
/* Firmware version */
#define VERSION_MAJOR (10)
#define VERSION_MINOR (2)

/* VERSION 2 - FOTA in external SPI Memory.
 * */

/* Internal I+D revision:
 * Every time a new version is compiled to be ready to validate by QC the
 * REVISION must be increased and code must be committed to the repository
 * with the following title MAJOR.MINOR.REVISION"R" (Release)
 *
 * Example: commit >> 5.21.02R -> Release revision 2 version 5.21 handled
 * to QC.
 *
 * Once a particular revision is validated a new commit to the repository
 * must be done with the following title MAJOR.MINOR"R" (Release)
 *
 * Example: commit >> 5.21R -> Final Release validated by QC and released
 * to customers.
 *
 * REVISION 1 - First revision.
 * */
#define REVISION	(6)

//#define DEBUG_MODE
//#define COMPILER

/* Comment for disable LOGLIVE */
#define LEVEL_0 	(9)  	/**< All disabled */
#define LEVEL_1		(1)		/**< All enabled */
#define LEVEL_2		(2)		/**< Medium LOGLIVE */
#define LEVEL_3		(3)		/**< Low  */

#define LOGLIVE_MODE
#define LOGLIVE_LEVEL	(params_get_loglive_level())//LEVEL_1
/* Comment for disable ITM_MODE */
//#define ITM_MODE

#if defined(LOGLIVE_MODE)
char loglive_str[8246];
#define  CHECK_TAMPER_PIN	            (HAL_GPIO_ReadPin( UC_TAMPER_GPIO_Port, UC_TAMPER_Pin ))
#define LOGLIVE(level, f, ...)          params_loglive(level, f,  ##__VA_ARGS__);
#define LOGLIVE_TAMP_EN(level, f, ...)  params_loglive_tamp_en(level, f,  ##__VA_ARGS__);
#define LOGLIVE_SERIAL(level, f, ...)   params_loglive_serial(level, f,  ##__VA_ARGS__);
#define LOGLIVE_(level, f, ...) (level < LOGLIVE_LEVEL) ? (void)(0) : \
	({ \
	     if ( ( modbus_sensors_get_comm_state() != MODBUS_COMM_STARTED ) && ( CHECK_TAMPER_PIN == 0 ) && ( con_dlms_in_process() == 0 ) && ( ( modbus_get_end_session() == MODBUS_SESSION_END ) || ( 0 == modbus_get_local_tool_started() ) ) ) { \
				MX_USART2_UART_Init(); \
				ENABLE_RS485(); \
				HAL_Delay(1);\
				DE_TX_RS485(); \
				snprintf(loglive_str,sizeof(loglive_str),f,##__VA_ARGS__); \
				serial_write_loglive(loglive_str, strlen(loglive_str)); \
				DISABLE_RS485(); \
				if ( modbus_get_end_session() == MODBUS_SESSION_INIT ) {MX_USART2_USART_DeInit();modbus_init();} \
	     }\
	}) \

#elif  defined(ITM_MODE)
	#define LOGLIVE(level, f,...)			printf(f,##__VA_ARGS__)
	#define LOGLIVE_TAMP_EN(level, f, ...)  printf(f,##__VA_ARGS__)
	#define LOGLIVE_SERIAL(level, f, ...)   printf(f,##__VA_ARGS__)
#else
	#define LOGLIVE(level, f,...)
#endif


//#define UNE82326
#define MBUS
#define EXT_SENSOR
#define MODBUS
#define DLMS
#define MQTT

//#define GL865_MODULE
#define ME910_MODULE

//#define CAT_M1
#define DATALOGGER_SST26

//#define ZONOS

#ifndef ZONOS
	#define MOVISTAR_SIM
//	#define ETISALAT_SIM
//	#define ETISALAT_AADC_SIM
//	#define ETISALAT_AADC_SIM_CATM1
//	#define ETISALAT_SEWA_SIM_CATM1
//	#define VODAFONE_SIM
//	#define BOMBAI_SIM
//	#define THETHINX_SIM
//	#define ONCE_SIM
//  #define ONCE_GLOBAL_SIM
//	#define ALLIOT_SIM
//	#define CHUNGHWA
//  #define CMNBIOT
//	#define VODAFONE_GERMANY_SIM
//  #define VODAFONE_NETHERLANDS_SIM
//	#define VODAFONE_LOGIC_SIM
//  #define VODAFONE_LOGIC_NETHERLANDS_SIM
//  #define ORANGE_BELGIUM_SIM
//	#define ORANGE_GLOBAL_SIM
//  #define ORANGE_SIM
//	#define DMCC_SIM
//  #define ETISALAT_DEWA_SIM_CATM1
//	#define CYTA_SIM


	#define PRIVATE_SERVER
//	#define PUBLIC_SERVER
//	#define PUBLIC_SERVER_QA_TRANGO
//	#define PUBLIC_SERVER_ADDC
//	#define PUBLIC_SERVER_ADDC_QA
//	#define PUBLIC_SERVER_ADDC_PROD
//	#define PUBLIC_SERVER_SEWA_PROD
//  #define PUBLIC_SERVER_DEWA_PROD
//	#define BOMBAI_SERVER
//	#define ETISALAT_LOCAL_SERVER
//  #define PUBLIC_DMCC_SERVER
//  #define PUBLIC_RECKENBERG_SERVER

//	#define HTTP
	#define UDP
#else
	#define ETISALAT_SIM
	#define UDP
#endif

#define PSM_MODE

//#define PWR_STDBY
#define PWR_STOP

#define ENCRYPTION

#ifdef DATALOGGER_SST26
#define SPI_FLASH_PAGE_SIZE		(uint32_t)( 256 )
#endif

#define TELEGRAM_MODE_RAW       (1)
#define TELEGRAM_MODE_DTK       (2)
#define PROTOCOL_TCP            (1)
#define PROTOCOL_UDP            (2)
#define NBIOT_PWR_ONOFF			(1)
#define NBIOT_PWR_PSM           (2)

#define SERVER_NAME_LEN         (96)
#define IMEI_NAME_LEN           (16)
#define READ_WINDOWS            (8)
#define SEND_WINDOWS			(8)

#define SIM_ERR                 (1)
#define CSQ_ERR                 (2)
#define CEREG_ERR               (3)
#define TSO_ERR                 (4)
#define TSE_ERR                 (5)
#define IP_ERR                  (6)

#define DC_MQTT_OFF             (0)
#define DC_MQTT_ON              (1)

#define MBUS_METER_TYPE			(1)
#define SEVERN_TRENT_METER_TYPE (2)
#define MBUS_SLAVES_METER_TYPE  (3)

#define TYPE_COMM_NBIOT         (1)
#define TYPE_COMM_CATM1         (0)

#define METER_NOT_SYNCH 		(0)
#define METER_GST_SYNCH 		(1)
#define METER_UTC_SYNCH 		(2)

#define NUM_INPUTS_SENSOR       (5U)//(4U)

#define OR_CONDITION			(0)
#define AND_CONDITION			(1)
/**
 * @struct params
 * @brief Structure that holds all the parameters of the system needs to be of 2048 bytes long.
 */
struct params
{
	/**
	 * @struct
	 * @brief Substructure for firmware version information
	 */
    struct
    {
    	uint32_t major;		/*!< Major firmware version */
    	uint32_t minor;		/*!< Minor firmware version */
    	uint32_t to_major;	/*!< Major firmware for firmware update */
    	uint32_t to_minor;	/*!< Minor firmware for firmware update */
    	uint32_t size;		/*!< Firmware update file size */
    	uint32_t crc;		/*!< Firmware update CRC */
    } version;

    uint32_t counter;	/*!< Increments each time parameters are written to FLASH */

    /**
     * @struct
     * @brief Substructure for APN configuration and credentials
     */
    struct
    {
      char name[100];		/*!< APN name */
      char user[32];		/*!< APN user */
      char password[64];	/*!< APN password */
      char title[30];		/*!<  */
    } APN[2];

     /**
     * @struct
     * @brief Substructure for server configuration.
     */
    struct
    {
        char     name[SERVER_NAME_LEN];	/*!< Server name  */
        uint32_t port;					/*!< Sever port */
        uint32_t keepalive;				/*!<  */
        uint32_t duration;				/*!<  */
    } server[2];

    /**
     * @struct
     * @brief Substructure for Low power modes PSM and EDRX configuration.
     */
    struct
	{
    	uint8_t tt3412;	/*!< timer t3412 */
    	uint8_t tt3324;	/*!< timer t3324 */
    	uint8_t edrx;	/*!< edrx timer */
    	uint8_t dummy;	/*!< Not used */
	} psm_edrx;

	/**
	 * @struct
	 * @brief
	 *
	 */
    struct
	{
    	uint32_t idp;		/*!< Synch read */
    	uint32_t idg;
    	uint32_t idd;
    	uint32_t rt;			/*!< time elapsed between sensor reading samples */
    	uint32_t st;			/*!< time elapsed between transmissions */
    	uint32_t sh[24];
    	uint32_t send_count;	/*!< Meter readings before a transmission */ //TOASK Why this counter is placed in struct config?
    	uint32_t ti1;			/*!< Sensor log */
    	uint32_t ti2;			/*!< Period */
    	float_t  ti3;			/*!< Values for Over pressure alarm */
    	float_t  ti4;			/*!< Value for Below pressure alarm */
	} config;

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint8_t init_time;
		uint8_t end_time;
		uint8_t cycle;
		uint8_t dummy;
	} read_time[READ_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint32_t send_time;
	} send_time[SEND_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint32_t send_params_time;
	} send_params_time[SEND_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint8_t  init_time;
		uint8_t  end_time;
		uint16_t cycle;
	} sensor_read_time[SEND_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint32_t send_time;
	} sensor_send_time[SEND_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint8_t  init_time;
		uint8_t  end_time;
		uint16_t cycle;
	} modbus_read_time[SEND_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint32_t send_time;
	} modbus_send_time[SEND_WINDOWS];

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
	{
		uint8_t  delete_memory; /* CLEAR_MEMORY_DELETE_DATALOGGER   (1) CLEAR_MEMORY_DISABLE_DATALOGGER  (2)
									CLEAR_MEMORY_FORCE_HARD_RESET    (3) CLEAR_MEMORY_DISABLE_HARD_RESET  (4) */
		uint8_t  reset_meters;
		uint8_t  release_assistance;
		uint8_t  reed_activation;
		uint32_t number_readings;
		uint32_t number_ok_sendings;
		uint32_t number_nok_sendings;
		uint32_t enable_alarm_sending;
		uint32_t max_allowed_num_of_readings;
		uint32_t max_allowed_num_of_sendings;
	} manteinance;

	/**
	 * @struct
	 * @brief
	 *
	 */
	struct
    {
        uint8_t write_period;
        uint8_t send_period;
        uint8_t prev_enabled;
        uint8_t enabled;
    } modbus_log;

    /**
     * @struct
     * @brief
     *
     */
    struct
	{
    	uint32_t on;
    	uint32_t pulse_factor;
		uint32_t unit_k_factor_out_1;
		uint32_t unit_k_factor_out_2;
	} pulses;

	uint32_t mem_cycle;

    char 	 imei[ IMEI_NAME_LEN ];
    char 	 middleware_id[12];
#ifdef MBUS
    unsigned char manufacturer_serial_num[12];
    unsigned char dewa_serial_num[20];
#endif
    unsigned char tool_cert[12];
    char 	 meter_id[12];

    uint32_t wq;		/*!< Water quality sensor enabled */

    uint32_t telegram_mode;
    uint32_t protocol_mode;
    uint32_t nbiotpwr_mode;

    /**
     * @struct
     * @brief
     *
     */
    struct{
		uint32_t sim_err;
		uint32_t csq_err;
		uint32_t cereg_err;
		uint32_t tso_err;
		uint32_t tse_err;
		uint32_t ip_err;
		uint32_t err_log[24];
		uint32_t err_log_timestamp[24];
    } attach_errors;

    uint32_t reset_time;
    uint32_t timeout_connection;
    uint32_t retries_socket;
    uint32_t timeout_server;
    uint32_t mbus_baudrate;
    uint32_t retries_server;
    uint32_t max_datalogger_size;
    uint32_t max_datalogger_msgs;
    uint32_t generic_modbus_type;
    uint32_t generic_modbus_read_time;
    uint32_t generic_modbus_send_time;
    uint32_t generic_modbus_quantity;
    uint32_t generic_modbus_function;
    uint32_t generic_modbus_address;
    uint32_t generic_modbus_slave_id;
    uint32_t generic_modbus_baudrate;
    uint32_t generic_modbus_parity;
    uint32_t generic_modbus_stop_bits;
    uint32_t generic_modbus_num_slaves;
    uint32_t generic_modbus_slaves_id[20];
    uint32_t pulse_acc;
    uint32_t mqtt_dc_on;
    struct {
         uint32_t device;
         uint32_t group;
         uint32_t platform;
    } id;
    uint32_t registered;

    uint32_t poweroff_time;
    uint32_t uart_meter_type;
    uint32_t type_comm;
    uint32_t synch_meters;
    uint32_t slow_meter;
    uint32_t mbus_frame;
    uint8_t  sensor_read_date_day[READ_WINDOWS];
    uint8_t  sensor_read_date_month[READ_WINDOWS];
    uint32_t input_pulse_as_sensor_num;
    uint8_t  sensor_periodic_send_date_month[READ_WINDOWS];
    float_t  over_sensor_alarm[NUM_INPUTS_SENSOR];
    float_t  low_sensor_alarm[NUM_INPUTS_SENSOR];
    uint32_t oversensor_condition;
    uint32_t lowsensor_condition;
    uint32_t relay_status;

    uint32_t psm_tt3412;
    uint32_t psm_tt3324;

    uint32_t i2c_sensor;

    uint32_t generic_modbus_warm_time;
    uint32_t input_alarm_sensor;
    uint32_t generic_sensor_warm_time;

    uint64_t pulses_totalizer;

#if defined(UNE82326)
    uint32_t gap[1828 - 3 - 12/4 - 4/4 - 4/4*READ_WINDOWS - 1*SEND_WINDOWS - 1 - 1*SEND_WINDOWS - 1*SEND_WINDOWS - 1*SEND_WINDOWS - 1 - 3 - 4 - 1*SEND_WINDOWS - 1*SEND_WINDOWS - 12/4 - 1 - 3 - 54 - 1 - 4 - 3 - 1 - 2 - 3 - 1 - 3 - 1 - 20 - 1 - 5 - 1 - 1 - 1 - 2 - 1 - 2 - 2 - 1 - 2 - 4 - 4 - 1 - 1 - 1 - 2 - 1 - 1 - 1 - 1 - 2];/*2048/4 - 1 - 7 - 452/4 - (96/4 + 3)*2 - 34 - 6 - 1 - IMEI_NAME_LEN/4*/
#elif defined(MBUS)
    uint32_t gap[1828 - 3 - 12/4 - 4/4 - 4/4*READ_WINDOWS - 1*SEND_WINDOWS - 1 - 1*SEND_WINDOWS - 1*SEND_WINDOWS - 1*SEND_WINDOWS - 3 - 5 - 1 - 3 - 4 - 1*SEND_WINDOWS - 1*SEND_WINDOWS - 12/4 - 1 - 3 - 54 - 1 - 4 - 3 - 1 - 2 - 3 - 1 - 3 - 1 - 20 - 1 - 5 - 1 - 1 - 1 - 2 - 1 - 2 - 2 - 1 - 2 - 5 - 5 - 1 - 1 - 1 - 2 - 1 - 1 - 1 - 1 - 2];/*2048/4 - 1 - 7 - 452/4 - (96/4 + 3)*2 - 34 - 6 - 1 - IMEI_NAME_LEN/4*/
#endif
    uint32_t crc;
    uint32_t align_crc[3];
};

extern struct params param;

uint32_t 	 params_version_major( void );
uint32_t 	 params_version_minor( void );
uint32_t 	 params_pulses_on( void );
uint32_t     params_wq_on( void );
uint32_t     params_telegram_mode( void );
void         params_set_telegram_mode( uint32_t mode );
void         params_set_protocol_mode( uint32_t mode );
void         params_set_nbiotpwr_mode( uint32_t mode );
uint32_t     params_protocol_mode( void );
uint32_t     params_nbiotpwr_mode( void );
uint32_t 	 params_pulses_pulse_factor( void );
uint32_t 	 params_pulses_pulse_unit_k_factor_out_1( void );
uint32_t 	 params_pulses_pulse_unit_k_factor_out_2( void );
void 		 params_pulses_set_on( uint32_t _on );
void         params_wq_set_on( uint32_t _on );
void 		 params_pulses_set_pulse_factor( uint32_t _pulse_factor );
void 		 params_pulses_set_pulse_unit_k_factor_out_1( uint32_t _unit_k_factor );
void 		 params_pulses_set_pulse_unit_k_factor_out_2( uint32_t _unit_k_factor );
uint8_t      params_modbus_log_enabled( void );
void         params_modbus_log_set_enabled( uint8_t enable );
uint8_t      params_modbus_log_prev_enabled( void );
void         params_modbus_log_set_prev_enabled( uint8_t enable );
unsigned char *params_modbus_get_tool_cert( void );

uint32_t 	 params_psm_edrx_tt3412( void );
uint32_t 	 params_psm_edrx_tt3324( void );
uint8_t 	 params_psm_edrx_edrx( void );
void         params_psm_edrx_set_tt3412( uint32_t _tt3412 );
void 		 params_psm_edrx_set_tt3324( uint32_t _tt3324 );
void 		 params_psm_edrx_set_edrx( uint8_t _edrx );
uint8_t      params_manteinance_release_assistance( void );
void         params_manteinance_set_release_assistance( uint8_t _rai );
uint8_t      params_manteinance_delete_memory( void );
void         params_manteinance_set_delete_memory( uint8_t _delete_memory );
uint8_t      params_manteinance_reset_meters( void );
void         params_manteinance_set_reset_meters( uint8_t _reset_meters );
uint8_t      params_manteinance_reed_activation( void );
void         params_manteinance_set_reed_activation( uint8_t _reed_activation );
uint32_t 	 params_config_send_time( void );
void 		 params_config_set_send_time( uint32_t _send_time );
uint8_t 	 params_read_time_init_time( uint8_t num );
void    	 params_read_time_set_init_time( uint8_t num, uint8_t _read_init_time );
uint8_t 	 params_read_time_end_time( uint8_t num );
void     	 params_read_time_set_end_time( uint8_t num, uint8_t _read_end_time );
uint8_t 	 params_read_time_cycle( uint8_t num );
void      	 params_read_time_set_cycle( uint8_t num, uint8_t _cycle );
uint8_t      params_send_time_send_time( uint8_t num );
void         params_send_time_set_send_time( uint8_t num, uint8_t _send_init_time );
uint8_t      params_sensor_read_time_init_time( uint8_t num );
void         params_sensor_read_time_set_init_time( uint8_t num, uint8_t _read_init_time );
uint8_t      params_sensor_read_time_end_time( uint8_t num );
void         params_sensor_read_time_set_end_time( uint8_t num, uint8_t _read_end_time );
uint16_t     params_sensor_read_time_cycle( uint8_t num );
void         params_sensor_read_time_set_cycle( uint8_t num, uint16_t _cycle );
uint8_t      params_sensor_send_time_send_time( uint8_t num );
void         params_sensor_send_time_set_send_time( uint8_t num, uint8_t _send_init_time );
void         params_sensor_read_time_set_date_day(uint8_t num, uint32_t _date_day);
uint8_t      params_sensor_read_time_date_day(uint8_t num);
void         params_sensor_read_time_set_date_month(uint8_t num, uint32_t _date_month);
uint8_t      params_sensor_read_time_date_month(uint8_t num);
void 		 params_sensor_period_date_month_set_period_date_month(uint8_t num, uint32_t _period_date_month);
uint8_t 	 params_sensor_period_date_month(uint8_t num);
void         params_input_pulse_as_sensor_set_num(uint8_t _num);
uint32_t     params_input_pulse_as_sensor_get_num(void);
uint8_t 	 params_modbus_read_time_init_time( uint8_t num );
void 		 params_modbus_read_time_set_init_time( uint8_t num, uint8_t _read_init_time );
uint8_t 	 params_modbus_read_time_end_time( uint8_t num );
void 		 params_modbus_read_time_set_end_time( uint8_t num, uint8_t _read_end_time );
uint16_t 	 params_modbus_read_time_cycle( uint8_t num );
void		 params_modbus_read_time_set_cycle( uint8_t num, uint16_t _cycle );
uint8_t 	 params_modbus_send_time_send_time( uint8_t num );
void 		 params_modbus_send_time_set_send_time( uint8_t num, uint8_t _send_init_time );
uint8_t      params_send_network_params_time_send_time( uint8_t num );
void         params_send_network_params_time_set_send_time( uint8_t num, uint8_t _send_init_time );
float_t      params_get_over_sensor_alarm(uint8_t num);
void         params_set_over_sensor_alarm(uint8_t num, uint32_t _over_sensor);
float_t      params_get_low_sensor_alarm(uint8_t num);
void         params_set_low_sensor_alarm(uint8_t num, uint32_t _over_sensor);
uint32_t     params_get_low_sensor_condition( void );
void 		 params_set_low_sensor_condition(uint32_t _low_sensor);
uint32_t     params_get_over_sensor_condition( void );
void         params_set_over_sensor_condition(uint32_t _over_sensor);
uint32_t 	 params_get_relay_status( void );
void 		 params_set_relay_status(uint32_t _relay_status);
uint32_t 	 params_config_read_time( void );
void         params_config_set_read_time( uint32_t _read_time );
uint32_t 	 params_config_send_hour( uint8_t num );
uint32_t 	 params_config_send_count( void );
void         params_config_set_send_count( uint32_t _send_count );
uint32_t 	 params_config_alarms_num( void );
void         params_config_set_sensor_log( uint8_t _sensor_log_on );
uint32_t 	 params_config_get_sensor_log( void );
uint32_t     params_maintenance_number_readings( void );
void         params_config_set_period( uint8_t _period );
uint32_t     params_config_get_period( void );
void         params_config_set_adc_on( uint8_t _adc_on );
uint32_t     params_config_get_adc_on( void );
void         params_config_set_disable_meter( uint8_t _disable_meter );
uint32_t     params_config_get_disable_meter( void );
void         params_config_set_sync_read( uint8_t _sync_read );
uint32_t     params_config_get_sync_read( void );
//void         params_config_set_generic_modbus_send_time( uint8_t _generic_modbus_send_time );
//uint32_t     params_config_get_generic_modbus_send_time( void );
void 		 params_maintenance_set_number_readings( uint32_t _number );
void 		 params_maintenance_inc_number_readings( void );
uint32_t     params_maintenance_number_ok_sendings( void );
void         params_maintenance_set_number_ok_sendings( uint32_t _number );
void 		 params_maintenance_inc_number_ok_sendings( void );
uint32_t     params_maintenance_number_nok_sendings( void );
void         params_maintenance_set_number_nok_sendings( uint32_t _number );
void 		 params_maintenance_inc_number_nok_sendings( void );
uint32_t 	 params_maintenance_enable_alarm_sending( void );
void 		 params_maintenance_set_enable_alarm_sending( uint32_t _onoff );
uint32_t     params_maintenance_max_allowed_number_of_sendings( void );
void         params_maintenance_set_max_allowed_number_of_sendings( uint32_t _maxnum );
void 		 params_attach_error_inc_SIM_error( void );
void		 params_attach_error_inc_CSQ_error( void );
void 		 params_attach_error_inc_CEREG_error( void );
void 		 params_attach_error_inc_TSO_error( void );
void 		 params_attach_error_inc_TSE_error( void );
void 		 params_attach_error_inc_IP_error( void );
uint32_t     params_attach_error_get_SIM_error( void );
uint32_t     params_attach_error_get_CSQ_error( void );
uint32_t     params_attach_error_get_CEREG_error( void );
uint32_t     params_attach_error_get_TSO_error( void );
uint32_t     params_attach_error_get_TSE_error( void );
uint32_t     params_attach_error_get_IP_error( void );
void         params_attach_insert_log( uint32_t _log );
uint32_t     params_attach_get_log_index( uint32_t index );
uint32_t     params_attach_get_log_timestamp_index( uint32_t index );
void         params_set_time_reset( time_t _time_reset );
time_t       params_get_time_reset( void );
void         params_set_timeout_connection( uint32_t _timeout_connection );
uint32_t     params_get_timeout_connection( void );
void         params_set_retries_socket( uint32_t _retries_socket );
uint32_t     params_get_retries_socket( void );
void         params_set_timeout_server( uint32_t _timeout_server );
uint32_t     params_get_timeout_server( void );
void         params_set_retries_server( uint32_t _retries_server );
uint32_t     params_get_retries_server( void );
void         params_set_pulse_acc( uint32_t _pulse_acc );
uint32_t     params_get_pulse_acc( void );
uint32_t     params_get_pulse_acc_backup( void );
void         params_set_pulse_acc_backup( uint32_t _pulse_acc );
uint32_t     params_get_pulse_write( void );
void         params_set_pulse_write( uint32_t _pulse_write );
void         params_set_mqtt_dc_on( uint32_t _mqtt_dc_on );
uint32_t     params_get_mqtt_dc_on( void );
void         params_set_modbus_type( uint32_t _modbus_type );
time_t       params_get_modbus_type( void );
void         params_set_modbus_read_time( uint32_t _modbus_read_time );
time_t       params_get_modbus_read_time( void );
void         params_set_modbus_send_time( uint32_t _modbus_send_time );
time_t       params_get_modbus_send_time( void );
void         params_set_modbus_quantity( uint32_t _quantity );
uint32_t     params_get_modbus_quantity( void );
void         params_set_modbus_function( uint32_t _quantity );
uint32_t     params_get_modbus_function( void );
void         params_set_modbus_address( uint32_t _quantity );
uint32_t     params_get_modbus_address( void );
void         params_set_modbus_slave_id( uint32_t _slave_id );
uint32_t     params_get_modbus_slave_id( void );
void         params_set_modbus_slave_num( uint32_t _function );
uint32_t     params_get_modbus_slave_num( void );
void         params_set_modbus_slave_id_table( uint32_t slave, uint32_t slave_id );
uint32_t     params_get_modbus_slave_id_table( uint32_t slave );
void         params_set_modbus_parity( uint32_t _parity );
uint32_t     params_get_modbus_parity( void );
void         params_set_modbus_baudrate( uint32_t _baudrate );
uint32_t     params_get_modbus_baudrate( void );
void         params_set_modbus_stopbits( uint32_t _stopbits );
uint32_t     params_get_modbus_stopbits( void );
void         params_set_modbus_warm_time( uint32_t _warm_time );
uint32_t     params_get_modbus_warm_time( void );
void         params_set_generic_sensor_warm_time( uint32_t _warm_time );
uint32_t     params_get_generic_sensor_warm_time( void );
void         params_set_input_alarm_sensor( uint32_t _enable );
uint32_t     params_get_input_alarm_sensor( void );
void         params_set_mbus_baudrate( uint32_t _baud_rate );
uint32_t     params_get_mbus_baudrate( void );
void         params_set_uart_meter_type( uint32_t _type );
uint32_t     params_get_uart_meter_type( void );
void         params_set_max_datalogger_size( uint32_t _max_datalogger_size );
uint32_t     params_get_max_datalogger_size( void );
void         params_set_max_datalogger_msgs( uint32_t max_datalogger_msgs );
uint32_t     params_get_max_datalogger_msgs( void );
void         params_set_poweroff_time(uint32_t _poweroff_time);
uint32_t     params_get_poweroff_time( void );
void 		 params_set_type_comm(uint32_t _type_comm);
uint32_t 	 params_get_type_comm(void);
void 		 params_set_synch_meters(uint32_t _synch_meters);
uint32_t     params_get_synch_meters(void);
void 		 params_set_slow_meter(uint32_t _synch_meters);
uint32_t     params_get_slow_meter(void);
uint32_t     params_slow_meter_delay(void);
void 		 params_set_mbus_frame(uint32_t _mbus_frame);
uint32_t     params_get_mbus_frame(void);
void         params_set_inc_pulses_totalizer(void);
void         params_reset_pulses_totalizer(void);
uint64_t     params_get_pulses_totalizer(void);

void         params_manteinance_reset_statistics( void );
char       * params_get_middleware_id( void );
void         params_set_middleware_id( char *middleware_id );
char       * params_get_meter_id( void );
void         params_set_meter_id( char *meter_id );
char 	   * params_get_APN( void );
char 	   * params_get_server( void );
uint32_t     params_get_server_keepalive( uint32_t _index );
uint32_t     params_set_server_keepalive( uint32_t _index, uint32_t _keepalive );
uint32_t     params_imei_ok( void );
char       * params_get_imei( void );
void         params_config_set_overpressure_alarm( float_t _overpressure );
float_t      params_config_get_overpressure_alarm( void );
void         params_config_set_lowpressure_alarm( float_t _lowpressure );
float_t      params_config_get_lowpressure_alarm( void );
void         params_set_i2c_sensor(uint32_t _i2c_sensor);
uint32_t     params_get_i2c_sensor(void);
char       * params_device_get_battery_var( void );

void         params_init( void );
uint32_t     params_get_mem_cycle( void );
void         params_set_mem_cycle( uint32_t mem_cycle );
void         params_check_for_changes( void );

#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
#define ct_assert(e) enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(!!(e)) }

#endif
