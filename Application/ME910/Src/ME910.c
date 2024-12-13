/**
  ******************************************************************************
  * @file           ME910.c
  * @author 		Datakorum Development Team
  * @brief          Telit ME910 Communication driver. This file provides APIs to
  * handle internal communication between the host microcontroller and the LTE
  * module.
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
 * ME910.c
 *
 *  Created on: 15 jul. 2019
 *      Author: Sergio Mill�n L�pez
 */
#define _GNU_SOURCE
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "main.h"
#include "usart.h"
#include "params.h"
#include "comm_serial.h"
#include "tick.h"
#include "leds.h"
#include "shutdown.h"
#include "une82326.h"
#include "comm_udp.h"
#include "udp_protocol.h"
#include "mbus.h"
#include "modbus.h"
#include "pulses.h"
#include "aqualabo_modbus.h"
#include "generic_modbus.h"
#include "ad.h"
#include "test_prod.h"
#include "modbus_sensors.h"
#include "message_queue.h"

#include "ME910.h"

#ifdef MQTT
#include "mqtt_timer.h"
#endif

#include "dlms_client.h"

//#define HTTPS

/**
 * @defgroup System_low_level_comms System low level communications
 * @brief
 * @{
 */

#define ON_DELAY 	(7)//(5)
#define OFF_DELAY	(1)//(3)

/**
 * @enum
 * @brief
 *
 */
typedef enum {
    M_BOOT0, M_BOOT_SW_OFF, M_BOOT_SW_ON, M_BOOT1, M_BOOT2, M_BOOT3,   																	// Power ON
    M_ECHO_OFF, M_BAND, M_CGMR, M_CPU_MODE, M_CMEE, M_CEMODE, M_CCIOTOPT, M_IMEI, M_SIMDET, M_SAVE_SIMDET, M_PIN, M_CCID, M_CIMI, M_CFUN, M_WS46IOT, M_WS46,  M_GPIO_1, M_GPIO, M_SLED, M_SKIPESC, M_CGATT_0, M_CGATT_1, M_CGDCONT_1, M_COPS, M_CSQ, M_CEREG,	// Conexion 4G
	M_eDRX, M_DNS, M_CPSMS, M_MONI, M_MONI_PARSE, M_RFSTS, M_CGDCONT, M_SGACT, M_SSLCFG, M_SSLSECCFG, M_SSLDIS, M_SSLEN, M_SSLSECCA, M_SSLSECDATAKEY_CMD, M_SSLSECDATAKEY, M_SSLSECDATA_CMD, M_SSLSECDATA, M_CGCONTRDP, M_SENDSSLSECDATA, M_CCLKMODE, M_CALA, M_CLOCK, M_SD, M_CONNECTED, 				// Conexion datos
	M_WAITING_FOR_SHUTDOWN,M_SHUTDOWN,
    M_DELAY,
	M_RECONNECT,
	M_WAIT_EXIT_STOP_MODE,
	M_DISABLE_PSM,
	M_DISABLE_PSM_RESP,
	M_EXIT_WITHOUT_CONNECTION,
	M_RECONNECT_HTTP,
} ME910_status;

/**
 * @enum
 * @brief
 *
 */
typedef enum {
	AT,                /**< AT */
	ATE0,              /**< ATE0 */
	BAND,	  
	CGMR,              /**< CGMR */
	CMEE,              /**< CMEE */
	CEMODE,            /**< CEMODE */
	CCIOTOPT,          /**< CCIOTOPT */
	CGSN,              /**< CGSN */
	SIMDET, 		   /**< SIMDET */
	SAVESIMDET, 	   /**< SAVESIMDET */
	CPIN,              /**< CPIN */
	CCID,              /**< CCID */
	CIMI,              /**< CIMI */
	CFUN,              /**< CFUN */
	WS46IOT,           /**< WS46IOT */
	WS46,              /**< WS46 */
	GPIO1,             /**< GPIO1 */
	GPIO,              /**< GPIO */
	SLED,              /**< SLED */
	SKIPESC,           /**< SKIPESC */
	CGATT0,            /**< CGATT0 */
	CGATT1,            /**< CGATT1 */
	CSQ,               /**< CSQ */
	COPS,              /**< COPS */
	CEREG,             /**< CEREG */
	CeDRX,             /**< CeDRX */
	CPSM,              /**< CPSM */
	ATMONI,            /**< ATMONI */
	ATRFSTS,           /**< ATRFSTS */
	CGDCONT,           /**< CGDCONT */
	SGACT,             /**< SGACT */
	SSLCFG,            /**< SSLCFG */
	SSLCFG_2,          /**< SSLCFG_2 */
	SSLSECCFG,         /**< SSLSECCFG */
	SSLDIS,            /**< SSLEN */
	SSLEN,             /**< SSLEN */
	CCLKMODE,          /**< CCLKMODE */
	CCLK,              /**< CCLK */
	SSLD,              /**< SSLD */
	SD,                /**< SD */
	CPSMS_DISABLE,     /**< CPSMS_DISABLE */
	CALA,              /**< CALA */
	COPS_TABLE, 	   /**< COPS_TABLE */
	CFUN5,
	LAST_ME910_COMMAND,/**< LAST_ME910_COMMAND */
} ME910_command;

/**
 * @brief
 */
static char *ME910_strings[ LAST_ME910_COMMAND ] =
{
	"AT\r", 				/**< AT - 0 */
	"ATE0\r",				/**< ATEO - 1 */
	"AT#BND=5,0,168695967\r",//"AT#BND=5,0,168695967\r",//"AT#SWPKGV\r",			/**< CGMR - 2 *///"AT+CGMR\r",//Bands:3,8,20
	"AT#SWPKGV\r",//"AT+CGMR\r",
	"AT+CMEE=2\r",			/**< CMEE - 3 */
	"AT+CEMODE=0\r",		/**< CEMODE - 4 */
	"AT#CCIOTOPT=0001\r",	/**< CCIOTOPT - 5 */
	"AT+CGSN\r",			/**< CGSN - 6 */
	"AT#SIMDET=",		/**< SIMDET - 16 */
	"AT&P&W\r",		        /**< P&W - 16 */
	"AT+CPIN?\r",			/**< CPIN - 7 */
	"AT#CCID\r",			/**< CCID - 8 */
	"AT+CIMI\r",			/**< CIMI - 9 */
	"AT+CFUN=1\r",			/**< CFUN - 10 */
//#ifndef CAT_M1
//	"AT#WS46=1\r",			/**< WS46IOT - 11 */
//#else
//	"AT#WS46=0\r",			/**< WS46IOT - 11 */
//#endif
//	"AT#WS46=",				/**< WS46IOT - 11 */
	"AT#WS46=2\r",			/**< WS46IOT - 11 */
	"AT+WS46=28\r",			/**< WS46 - 12 */
	"AT#GPIO=2,0,1\r",		/**< GPIO1 - 13 */
	"AT#GPIO=1,0,2\r",		/**< GPIO - 14 */
	"AT#SLED=4\r",			/**< SLED - 15 */
	"AT#SKIPESC=1\r",		/**< SKIPESC - 16 */
	"AT+CGATT=0\r",			/**< CGATT0 - 17 */
	"AT+CGATT=1\r",			/**< CGATT1 - 18 */
	"AT+CSQ\r",				/**< CSQ - 19 */
#if defined (ETISALAT_SIM)
#ifndef CAT_M1
	"AT+COPS=1,2,\"42402\",9\r",	/**< COPS - 20 */
//	"AT+COPS=1,2,\"00101\",9\r",
#else
	"AT+COPS=1,2,\"42402\",8\r",	/**< COPS - 20 */
#endif
#elif defined (ETISALAT_AADC_SIM_CATM1)
	"AT+COPS=1,2,\"42402\",8\r",	/**< COPS - 20 */
#elif defined (ETISALAT_AADC_SIM)
#ifndef CAT_M1
	"AT+COPS=1,2,\"42402\",9\r",	/**< COPS - 20 */
//	"AT+COPS=1,2,\"00101\",9\r",
#else
	"AT+COPS=1,2,\"42402\",8\r",	/**< COPS - 20 */
#endif
#elif defined(MOVISTAR_SIM)
	"AT+COPS=1,2,\"21407\",9\r",	/**< COPS - 20 */
#elif defined(VODAFONE_SIM)
	"AT+COPS=1,2,\"21401\",9\r",	/**< COPS - 20 */
#elif defined(BOMBAI_SIM)
	"AT+COPS=1,2,\"21407\",9\r",	/**< COPS - 20 */
#elif defined(THETHINX_SIM)
	"AT+COPS=1,2,\"21407\",9\r",	/**< COPS - 20 */
#elif defined(ONCE_SIM)
	"AT+COPS=1,2,\"26201\",9\r",	/**< COPS - 20 */
#elif defined(ONCE_GLOBAL_SIM)
#ifndef CAT_M1
//	"AT+COPS=1,2,\"90140\",9\r",
//	"AT+COPS=4,2,\"90140\",9\r",
	"AT+COPS=1,2,\"20416\",9\r",	/**< COPS - 20 */
#else
//	"AT+COPS=1,2,\"90140\",8\r",
	"AT+COPS=1,2,\"20416\",8\r",	/**< COPS - 20 */
#endif
#elif defined(ORANGE_BELGIUM_SIM)
#ifndef CAT_M1
	"AT+COPS=4,2,\"20610\",9\r",	/**< COPS - 20 */
#else
	"AT+COPS=4,2,\"20610\",8\r",	/**< COPS - 20 */
#endif
#elif defined(VODAFONE_NETHERLANDS_SIM)
#ifndef CAT_M1
	"AT+COPS=1,2,\"20404\",9\r",	/**< COPS - 20 */
#else
	"AT+COPS=1,2,\"20404\",8\r",	/**< COPS - 20 */
#endif
#elif defined(ALLIOT_SIM)
	"AT+COPS=1,2,\"23203\",9\r",	/**< COPS - 20 */
#elif defined(CHUNGHWA)
//	"AT+COPS=1,2,\"46692\",9\r",	/**< COPS - 20 */
	"AT+COPS=1,2,\"46000\",9\r",
#elif defined(VODAFONE_GERMANY_SIM)
	"AT+COPS=1,2,\"26202\",9\r",	/**< COPS - 20 */
#elif defined(VODAFONE_LOGIC_SIM)
	"AT+COPS=1,2,\"21401\",9\r",	/**< COPS - 20 */
#elif defined(VODAFONE_LOGIC_NETHERLANDS_SIM)
	"AT+COPS=4,2,\"20404\",9\r",	/**< COPS - 20 */
#elif defined(ORANGE_SIM)
	"AT+COPS=1,2,\"21403\",9\r",	/**< COPS - 20 */
#elif defined (DMCC_SIM)
	"AT+COPS=1,2,\"42402\",9\r",
#elif defined (ETISALAT_SEWA_SIM_CATM1)
	"AT+COPS=1,2,\"42402\",8\r",
#elif defined (ETISALAT_DEWA_SIM_CATM1)
	"AT+COPS=1,2,\"42402\",8\r",
#elif defined(CYTA_SIM)
	"AT+COPS=1,2,\"28001\",9\r",	/**< COPS - 20 */
#endif
	"AT+CEREG?\r",		/**< CEREG - 21 */
	"AT+CEDRXS=1,5,\"0001\"\r",		/**< CeDRX - 22 */  	//"AT+CEDRXS=0\r", //eDRX value 20'48 s, Page Time Window 5'12 s.
	"AT+CPSMS=1,,,\"01000001\",\"00001111\"\r",		/**< CPSM - 23 */	//"AT+CPSMS=1,,,\"10100100\",\"00001000\"\r", //"AT+CPSMS=1,,,\"00100001\",\"01100001\"\r",
	"AT#MONI\r",				/**< ATMONI - 24 */
	"AT#RFSTS\r",				/**< ATRFSTS - 25 */
	"AT+CGDCONT=1,\"IP\",",		/**< CGDCONT - 26 */
	"AT#SGACT=1,1",				/**< SGACT - 27 */
#ifdef HTTPS
	"AT#SSLCFG=1,1,1500,0,100,10,0,0,1,0,1200,0\r",//"AT#SSLCFG=1,1,1500,0,100,10,1,0,0,0\r",//
	"AT#SSLCFG=1,1,1500,0,100,10,0,0,1,0,1200,0\r",//"AT#SSLCFG=1,1,1500,0,100,10,1,0,0,0\r",//
#else
	"AT#SCFG=1,1,1500,0,100,10\r",//	/**< SSLCFG - 28 */		//"AT#SCFG=1,1,1500,0,300,0\r",//"AT#SCFG=1,1,300,10,10,0\r",//"AT#SCFG=1,1,300,0,300,10\r",//"AT#SCFG=1,1,1500,0,100,0\r",
	"AT#SCFG=1,1,1500,0,100,10\r",//	/**< SSLCFG_2 - 29 */		//"AT#SCFG=1,1,1500,10,10,0\r",
#endif
	"AT#SSLSECCFG=1,0,2\r",//"AT#SSLSECCFG=1,0x9F,0\r",			/**< SSLSECCFG - 30 */		//"AT#SSLSECCFG=1,0,0\r",
	"AT#SSLEN=1,0\r",				/**< SSLDIS - 31 */
	"AT#SSLEN=1,1\r",				/**< SSLEN - 31 */
#ifdef HTTPS
	"AT#SSLSECCFG2=1,4,1,9,0\r",//"AT#CCLKMODE=1\r",//
#else
	"AT#CCLKMODE=1\r",			/**< CCLKMODE - 32 */
#endif
	"AT+CCLK?\r",				/**< CCLK - 33 *///"AT+CCLK=\"18/10/01,08:38:00+00\"\r",
	"AT#SSLD=1,443,",			/**< SSLD - 34 */
	"AT#SD=1,0,",				/**< SD - 35 */
	"AT+CPSMS=0\r",				/**< CPSM_DISABLE - 36 */
	"AT+CALA=\"20/11/18,11:23:12+00\"\r",	/**< CALA - 37 */
	"AT+COPS=1,2,",	/**< COPS - 38 */
	"AT+CFUN=5\r",
											/**< LAST_ME910_COMMAND - 39 */
};

/**
 * @enum
 * @brief
 *
 */
typedef enum {
	MOVISTAR,
	CHUNGWA,
	VODAFONE_GERMANY,
	LOGIC_SPAIN,
	CMNBIOT,
	ETISALAT,
	ORANGE,
	CYTA,
	NOPLMN,
	LAST_PLMN_INDEX
}plmn_index;

#if 1
/**
 * @brief
 */
static char *PLMN_table[ LAST_PLMN_INDEX ] = {
		"21407",
		"46692",
		"26202",
		"21401",
		"46000",
		"42402",
		"21403",
		"28001",
		"NOPLMN",
};
#endif
/**
 * @brief
 */
ME910_t sME910 = {
		.connectedStatus    = 0,
		.IP                 = { 0 },
		.ICCID              = { 0 },
		.IMSI               = { 0 },
		.IMEI               = { 0 },
		.dev_identifier     = { 0 },
		.error              = ERROR_INIT,
		.timeConnected      = 0,
		.tcpSocketAvailable = 0,
		.udpSocketAvailable = 0,
		.shutdownStatus     = 0,
		.attemptsConnection = 0,
		.PSMMode            = 0,
};

static NetworkParameters_t Network_parameters;
static ME910_status status = M_BOOT0, error_status = M_BOOT0;
static char         ME910_data[ 1024 ];
static uint32_t     time_to_wait_cmd;
static int8_t       csq_ok = 0;
static uint8_t      network_params_get = 0, dk_network_params_get = 0;

uint32_t connect_time_ini = 0, connect_time_end = 0;

extern uint32_t reed_on;
extern uint32_t rtc_refresh_time;

static uint8_t  __write_cmd( ME910_command cmd, uint8_t *params );
static void     __checkTimeout( uint32_t *t );
static uint8_t  __getIMSI( void );
static char     __getIMEI( void );
static uint8_t  __getICCID( void );
static char 	__getIP(void);
static char *   __searchPLMNbyAPN( uint32_t apn );
static char *   __searchPLMNbyAPNInSIM( uint32_t *is_global );
////////////////////////////////////////////////////////////////////////////////
// Enable 4V_NB

/**
 * @fn void __modemEnable(void)
 * @brief Enables ME910 power rail.
 */
static void __modemEnable(void)
{
	HAL_GPIO_WritePin(UC_NBIOT_ENABLE_GPIO_Port, UC_NBIOT_ENABLE_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

/**
 * @fn void __startModemOnOffProc(void)
 * @brief Enables ME910 ON/OFF pin.
 */
static void __startModemOnOffProc(void)
{
	HAL_GPIO_WritePin(UC_NBIOT_ON_OFF_GPIO_Port, UC_NBIOT_ON_OFF_Pin, GPIO_PIN_SET);
}

/**
 * @fn void __endModemOnOffProc(void)
 * @brief Disables ME910 ON/OFF pin.
 */
static void __endModemOnOffProc(void)
{
	HAL_GPIO_WritePin(UC_NBIOT_ON_OFF_GPIO_Port, UC_NBIOT_ON_OFF_Pin, GPIO_PIN_RESET);
}

/**
 * @fn uint8_t Telit_wake_up_from_PSM(void)
 * @brief Sets the ME910 ON/OFF pin low for 5s for asynchronous PSM woken up.
 * @return ME910 status.
 * 			@arg 0 - ME910 not enabled.
 * 			@arg 1 - ME910 enabled.
 */
uint8_t Telit_wake_up_from_PSM(void)
{
	HAL_GPIO_WritePin(UC_NBIOT_ON_OFF_GPIO_Port, UC_NBIOT_ON_OFF_Pin, GPIO_PIN_SET);
    HAL_Delay(5000);//>=5s
    HAL_GPIO_WritePin(UC_NBIOT_ON_OFF_GPIO_Port, UC_NBIOT_ON_OFF_Pin, GPIO_PIN_RESET);

    if (1 == ME910_checkPWRMON())
    {
    	return 1;
    }

    return 0;
}

/**
 * @fn void ME910_enable(void)
 * @brief ME910 ENABLE and ON/OFF I/O pin configuration.
 */
void ME910_enable(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/** Pin configuration */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStructure.Pin   = UC_NBIOT_ENABLE_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(UC_NBIOT_ENABLE_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin   = UC_NBIOT_ON_OFF_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(UC_NBIOT_ON_OFF_GPIO_Port, &GPIO_InitStructure);

    /** Set power rail to ON */
	__modemEnable();
}

/**
 * @fn void Telit_modem_disable(void)
 * @brief Disables ME910 power rail
 */
void Telit_modem_disable(void)
{
	HAL_GPIO_WritePin(UC_NBIOT_ENABLE_GPIO_Port, UC_NBIOT_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
}

/**
 * @fn void Telit_modem_off_prc(void)
 * @brief Generates a ME910 hardware OFF procedure issuing a Network de-attach
 */
void Telit_modem_off_prc(void)
{
	HAL_GPIO_WritePin(UC_NBIOT_ON_OFF_GPIO_Port, UC_NBIOT_ON_OFF_Pin, GPIO_PIN_SET);
	HAL_Delay(3200);//>=3s
    HAL_GPIO_WritePin(UC_NBIOT_ON_OFF_GPIO_Port, UC_NBIOT_ON_OFF_Pin, GPIO_PIN_RESET);
}

/**
 * @fn void Telit_disable(void)
 * @brief ME910 DIABLE and ON/OFF I/O pin configuration.
 */
void Telit_disable(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/** Pin configuration */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStructure.Pin   = UC_NBIOT_ENABLE_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(UC_NBIOT_ENABLE_GPIO_Port, &GPIO_InitStructure);

    GPIO_InitStructure.Pin   = UC_NBIOT_ON_OFF_Pin;
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(UC_NBIOT_ON_OFF_GPIO_Port, &GPIO_InitStructure);

    /** Set power rail to OFF */
    Telit_modem_disable();
}

/**
 * @fn uint8_t Telit_in_PSM(void)
 * @brief Returns PSMMode field of the structure sME910
 * @return PSM mode
 * 			@arg 0 - PSM not enabled.
 * 			@arg 1 - PSM enabled.
 */
uint8_t Telit_in_PSM(void)
{
	return sME910.PSMMode;
}

/**
 * @fn void Telit_save_status(void)
 * @brief Saves ME910 FSM status to register DR19 of the backup domain
 * @note Backup domain is not erased after a system reset.
 */
void Telit_save_status(void)
{
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR19, status);
}

/**
 * @fn uint8_t Telit_read_status(void)
 * @brief returns the current status of the ME910 FSM.
 * @note
 * @return status current status of the ME910 FSM.
 */
uint8_t Telit_read_status(void)
{
	static uint32_t lock = 0;
	uint8_t status_backup;

	status_backup = HAL_RTCEx_BKUPRead(&hrtc,RTC_BKP_DR19);
	if ((M_WAIT_EXIT_STOP_MODE == status_backup) && (1 == rtc_system_getGetTime()) && (0 == lock)
	  && (1 == shutdown_initTelitModule()))
	{ //Device daily reset to sync server time...We want to recover PSM attached state.
		lock   = 1;
		status = status_backup;
	}
	return status;
}

/**
 * @fn uint8_t Telit_read_status(void)
 * @brief returns the current status of the ME910 FSM.
 * @note
 * @return status current status of the ME910 FSM.
 */
uint8_t Telit_read_error_status(void)
{

	return error_status;
}

/**
 * @fn uint8_t Telit_wait_exit_stop_mode(void)
 * @brief Checks whether the current status of the ME910 is WAIT_EXIT_STOP_MODE
 * @return status
 * 			@arg 0 - status is not WAIT_EXIT_STOP_MODE
 * 			@arg 1 - status is WAIT_EXIT_STOP_MODE
 */
uint8_t Telit_wait_exit_stop_mode(void)
{
//	Telit_read_status();
	return (status == M_WAIT_EXIT_STOP_MODE ? 1 : 0);
}

////////////////////////////////////////////////////////////////////////////////
int32_t ME910_network_params_rssi( void )
{
	return Network_parameters.rssi;
}

int32_t ME910_network_params_rsrq( void )
{
	return Network_parameters.rsrq;
}

int32_t ME910_network_params_rsrp( void )
{
	return Network_parameters.rsrp;
}

uint32_t ME910_network_params_csq( void )
{
	return Network_parameters.csq;
}

uint32_t ME910_network_params_pci( void )
{
	return Network_parameters.pci;
}

uint32_t ME910_network_params_sinr( void )
{
	return Network_parameters.sinr;
}

uint32_t ME910_network_params_ecl( void )
{
	return Network_parameters.ecl;
}

uint32_t ME910_network_params_tx_power( void )
{
	return Network_parameters.tx_power;
}

uint32_t ME910_network_params_total_power( void )
{
	return Network_parameters.total_power;
}

uint32_t ME910_network_params_battery_live( void )
{
	return Network_parameters.battery_live;
}

char *ME910_network_params_cell_id( void )
{
	return Network_parameters.cell_id;
}

uint32_t ME910_network_params_rtt_mean( void )
{
	return Network_parameters.rtt_mean;
}

void ME910_network_params_set_rtt_mean( uint32_t rtt_mean )
{
	Network_parameters.rtt_mean = rtt_mean;
}

uint32_t ME910_network_params_creg( void )
{
	return Network_parameters.creg;
}

uint32_t ME910_network_params_abnd( void )
{
	return Network_parameters.abnd;
}

uint32_t ME910_get_connection_time( void )
{
	uint32_t ret = (connect_time_end - connect_time_ini);
	return ret;
}
////////////////////////////////////////////////////////////////////////////////
void ME910_reset_status( void )
{
	if ( status != M_WAIT_EXIT_STOP_MODE ) {
		status = M_BOOT0;
	}
}

void ME910_init( void )
{
	sME910.timeConnected      = 0;
	sME910.connectedStatus    = NOT_CONNECTED;
	sME910.shutdownStatus     = 0;
	sME910.attemptsConnection = 0;
	sME910.error 	          = 0;
	sME910.IMEI[0]            = '\0';
	sME910.IP[0]              = '\0';
	if ( M_WAIT_EXIT_STOP_MODE == rtc_system_readBackUpRegister(RTC_BKP_DR11) ) {
		status = M_WAIT_EXIT_STOP_MODE;
	}
}

void ME910_reinit_device( void )
{
	ME910_init();
	status           = M_BOOT0;
	time_to_wait_cmd = 2;
	Tick_update_tick( TICK_TELIT );
}

void ME910_inc_conn_attempts( void )
{
	sME910.attemptsConnection++;
}

void ME910_reset_conn_attempts( void )
{
	sME910.attemptsConnection = 0;
}

uint32_t ME910_get_conn_attempts( void )
{
    return sME910.attemptsConnection;
}

conn_status ME910_ModuleConnected( void )
{
   return sME910.connectedStatus && sME910.IMEI[0];
}

void ME910_ModuleSetConnection( conn_status connected )
{
	sME910.connectedStatus = connected;
}

char *ME910_getIP(void)
{
	return sME910.IP;
}

int8_t ME910_test_ok( void )
{
    return csq_ok;
}

void ME910_set_network_params_get( uint8_t _network_params_get )
{
	network_params_get = _network_params_get;
}

uint8_t ME910_get_network_params_get( void )
{
	return network_params_get;
}

void ME910_set_dk_network_params_get( uint8_t _dk_network_params_get )
{
	dk_network_params_get = _dk_network_params_get;
}

uint8_t ME910_get_dk_network_params_get( void )
{
	return dk_network_params_get;
}

void ME910_EraseBuffer( void )
{
	memset( ME910_data, 0, sizeof( ME910_data ));
}

uint8_t ME910_checkPWRMON( void )
{
	uint32_t initial_tick, elapsed, at_ok;

	initial_tick = HAL_GetTick();

	do {
		at_ok = 0;
		comm_serial_delete(comm_serial_rcx_bytes_n());
		__write_cmd(AT, NULL);
		HAL_Delay( 100 );
		elapsed  = HAL_GetTick() - initial_tick;

		if (ME910_StringEqual("AT\r\r\nOK\r", 8) || ME910_StringEqual("OK", 2))
		{
			comm_serial_delete(comm_serial_rcx_bytes_n());
			__write_cmd(ATE0, NULL);
			HAL_Delay( 100 );

			if (ME910_StringEqual("OK", 2))
			{
				at_ok = 1;
				comm_serial_delete(comm_serial_rcx_bytes_n());
			}
		}
	} while((at_ok == 0) && (elapsed < 15000) && (GPIO_PIN_RESET == ME910_CHECK_PSM));

	if (at_ok == 1)
	{
		return 1;
	}
	return 0;
}

uint8_t ME910_setPowerDown( void )
{
	uint32_t initial_tick, elapsed, at_ok;

	initial_tick = HAL_GetTick();

	do {
		at_ok = 0;
		comm_serial_delete(comm_serial_rcx_bytes_n());
		__write_cmd(CFUN5, NULL);
		HAL_Delay( 100 );
		elapsed  = HAL_GetTick() - initial_tick;

		if (ME910_StringEqual("AT\r\r\nOK\r", 8) || ME910_StringEqual("OK", 2))
		{
			comm_serial_delete(comm_serial_rcx_bytes_n());
			at_ok = 1;
		}
	} while((at_ok == 0) && (elapsed < 15000) && (GPIO_PIN_RESET == ME910_CHECK_PSM));

	if (at_ok == 1)
	{
		return 1;
	}
	return 0;
}

void ME910_setTimeToWait( uint32_t _time_to_wait_cmd )
{
	time_to_wait_cmd = _time_to_wait_cmd;
}

// APN                    COMPANY           ID LENGTH
//=====					  =======           =  ======
//"e3tcity.movistar.es"   MOVISTAR          0 (19)
//"m2m.movistar.es"       MOVISTAR          0 (15)
//"sensorm2m.movistar.es" MOVISTAR          0 (21)
//"internet.iot"          CHUNGWA           1 (12)
//"lpwa.vodafone.com"     VODAFONE_GERMANY  2 (17)
//"nb.wlapn.com"          LOGIC SPAIN       3 (12)
//"cmnbiot"               CMNBIOT           4 (7)
//"nbiot"                 ETISALAT          5 (5)
//"aadc-nbiot"            ETISALAT          5 (10)
//"addc-p2p"              ETISALAT          5 (8)
//"sewa.gov.ae"           ETISALAT          5 (11)
//"DMM2M.GOV.AE"		  ETISALAT          5 (12)
//"dewa-sg"				  ETISALAT          5 (7)
//"iot.nat.es"            ORANGE            6 (10)
//"iot"                   CYTA              7 (3)
static char *__searchPLMNbyAPN( uint32_t apn_index )
{
	uint32_t index = 0;
	switch ( strlen(param.APN[apn_index].name)) {
	case 19:
	case 15:
	case 21:
		index = 0;
		break;
	case 12:
		if ('i' == param.APN[apn_index].name[0])
		{
			index = 1;
		}
		else if ('n' == param.APN[apn_index].name[0])
		{
			index = 3;
		}
		else if (('D' == param.APN[apn_index].name[0])||('d' == param.APN[apn_index].name[0]))
		{
			index = 5;
		}
		break;
	case 17:
		index = 2;
		break;
	case 7:
		if ('d' == param.APN[apn_index].name[0])
		{
			index = 5;
		}
		else
		{
			index = 4;
		}
		break;
	case 5:
	case 11:
		index = 5;
		break;
	case 8:
	case 10:
		if ('i' == param.APN[apn_index].name[0])
		{
			index = 6;
		}
		else if ('a' == param.APN[apn_index].name[0])
		{
			index = 5;
		}
		break;
	case 3:
		index = 7;
		break;
	default:
		index = 6;
		break;
	}
	return PLMN_table[index];
}

static char PLMN_value[6];
static char *__searchPLMNbyAPNInSIM( uint32_t *is_global )
{
	*is_global    = 0;

	PLMN_value[0] = sME910.IMSI[0];
	PLMN_value[1] = sME910.IMSI[1];
	PLMN_value[2] = sME910.IMSI[2];
	PLMN_value[3] = sME910.IMSI[3];
	PLMN_value[4] = sME910.IMSI[4];
	PLMN_value[5] = '\0';

	if ((PLMN_value[0] == '9') && (PLMN_value[1]=='0') && (PLMN_value[2] == '1'))
	{
		*is_global = 1;
	}
	return PLMN_value;
}

/**
 * @fn uint32_t __processOKCommand(ME910_command, uint8_t*, uint32_t, ME910_status)
 * @brief
 *
 * @pre
 * @post
 * @param cmd
 * @param params
 * @param time
 * @param _status
 * @return
 */
static uint32_t __processOKCommand( ME910_command cmd, uint8_t *params, uint32_t time, ME910_status _status )
{
	uint32_t ret = HAL_OK;

	if (cmd != LAST_ME910_COMMAND) {
		comm_serial_delete(comm_serial_rcx_bytes_n());
		if ( __write_cmd(cmd, params) != HAL_OK ) {
			ret = HAL_ERROR;
		}
	}

    status           = _status;
    time_to_wait_cmd = time;
    Tick_update_tick( TICK_TELIT );

    return ret;
}

/**
 * @fn uint32_t __getCSQValue(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
static uint32_t __getCSQValue( void )
{
	char        un,dec;
	char        *s, *s2;
	unsigned int x;

	s2  = s = (char *) ME910_data;
    s   = strstr(s2, ": ");
    s   = s + 2;
    dec = *s - '0';
    s2  = s + 1;

    if (*s2 != ',')
    {
    	un = *s2 - '0';
    	x  = un + dec * 10;
    }
    else
    {
    	x = dec;
    }

    sME910.CSQ = x;
    return x;
}

/**
 * @fn time_t __getDate(char*)
 * @brief Parses +CCLK command date and time returned by the ME910 module.
 * @param date pointer to the returned string given by ME910 module.
 * @return Epoch UTC time in uint32_t format
 * @note The output format of the ME910 is +CCLK: \"19/04/01,11:58:24+08\"
 */
static time_t __getDate(char *date)
{
    struct tm fecha = {0};
    char *str;

    // www.epochconverter.com
    //+CCLK: \"19/04/01,11:58:24+08\"\r\n\r\nOK\r\n
    str = strtok( date, "/ :" );
    fecha.tm_year = atoi( str ) + 2000 - 1900;
    str = strtok( NULL, "/ , :" );
    fecha.tm_mon = atoi( str ) - 1;
    str = strtok( NULL, "/ , :" );
    fecha.tm_mday = atoi( str );

    str = strtok( NULL, "/ , :" );
    fecha.tm_hour = atoi( str );
    str = strtok( NULL, "/ , :" );
    fecha.tm_min = atoi( str );
    str = strtok( NULL, "/ :" );
    fecha.tm_sec = atoi( str );

    char buffer[80];
    strftime(buffer,80,"Network Date and Time: %m/%d/%y %H:%M:%S", &fecha);
    LOGLIVE(LEVEL_2,"LOGLIVE> %d ME910> %s\r\n", (int)Tick_Get( SECONDS ), buffer);

    return mktime(&fecha);
}

/**
 * @fn void __parseMONI(uint32_t)
 * @brief Parses #MONI command returned by the ME910 module.
 * @param frame_len
 * @note The output format of the ME910 is #MONI: <netmame> RSRP:<rsrp> RSRQ:<rsrq> TAC:<tac> Id:<id>
 * 	EARFCN:<earfcn> PWR:<dBm> DRX:<drx> pci:<physicalCellId> QRxLevMin:<QRxLevMin> Cc:<cc> Nc:<nc> RSRP:<rsrp>
 * 	RSRQ:<rsrq> TAC:<tac> Id:<id> EARFCN:<earfcn> PWR:<dBm> DRX:<drx> pci:<physicalCellId> QRxLevMin:<QRxLevMin>
 *
 */
static void __parseMONI( uint32_t frame_len )
{
	char *s, *str;

	s = (char *) ME910_data;

	//RSRP:<rsrp> RSRQ:<rsrq> TAC:<tac> Id:<id> EARFCN:<earfcn> PWR:<dBm> DRX:<drx> pci:<physicalCellId> QRxLevMin:<QRxLevMin>
	str = memmem( (char *)s, frame_len, "RSRP:", 5 );
	str = strtok( str, ": " );//RSRP
	str = strtok( NULL, ": " );//<rsrp>
	Network_parameters.rsrp = atoi(str);

	str = memmem( (char *)str, frame_len, "RSRQ:", 5 );
	str = strtok( str, ": " );
	str = strtok( NULL, ": " );
	Network_parameters.rsrq = atoi(str);

	str = memmem( (char *)str, frame_len, "Id:", 3 );
	str = strtok( str, ": " );
	str = strtok( NULL, ": " );
    snprintf(Network_parameters.cell_id, sizeof(Network_parameters.cell_id), "%s", str);

	str = memmem( (char *)str, frame_len, "pci:", 4 );
	str = strtok( str, ": " );
	str = strtok( NULL, ": " );
	Network_parameters.pci = atoi(str);

	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> RSRP:%i; RSRQ:%i; Cell ID:%s; PCI:%u.\r\n",
			(int)Tick_Get( SECONDS ),
			(int) Network_parameters.rsrp,
			(int) Network_parameters.rsrq,
			Network_parameters.cell_id,
			(unsigned int) Network_parameters.pci);

}

/**
 * @fn void __parseRFSTS(uint32_t)
 * @brief Parses #RFSTS command returned by the ME910 module.
 * @param frame_len
 * @note #RFSTS:<PLMN>,<EARFCN>,<RSRP>,<RSSI>,<RSRQ>,<TAC>,<RAC>,[<TXPWR>],<DRX>, <MM>,<RRC>,
 * 		<CID>,<IMSI>,[<NetNameAsc>],<SD>,<ABND>,<T3402>,<T3412>,<SINR>
 */
static void __parseRFSTS( uint32_t frame_len )
{
	char  *str;
	uint8_t pwr_void = 0;

	str = (char *) ME910_data;

	str = strtok( str, "," ); //PLMN
	str = strtok( NULL, "," );//EARFCN
	str = strtok( NULL, "," );//RSRP
	str = strtok( NULL, "," );//RSSI
	Network_parameters.rssi        = atoi(str);
	Network_parameters.total_power = Network_parameters.rssi;
	str = strtok( NULL, "," );//RSRQ
	str = strtok( NULL, "," );//TAC
	str = strtok( NULL, "," );//RAC
	if ( ( *(str + 3) == ',' ) || ( *(str + 4) == ',' )) {
		pwr_void = 1;
	}
	str = strtok( NULL, "," );//TX_POWER
	Network_parameters.tx_power = atoi(str);
	if ( /*( 1024 == Network_parameters.tx_power ) ||*/ ( 1 == pwr_void ) ) {
		Network_parameters.tx_power = 0;
	} else {
		str = strtok( NULL, "," );//DRX
	}
	str = strtok( NULL, "," );//MM
	str = strtok( NULL, "," );//RRC
	str = strtok( NULL, "," );//CID
	str = strtok( NULL, "," );//IMSI
	str = strtok( NULL, "," );//NetNameAsc
	str = strtok( NULL, "," );//SD
	str = strtok( NULL, "," );//ABND
	Network_parameters.abnd = atoi(str);
	str = strtok( NULL, "," );//T3402
	str = strtok( NULL, "," );//T3412
	str = strtok( NULL, "," );//SINR
	Network_parameters.sinr = atoi(str);

	Network_parameters.csq  = sME910.CSQ;

	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> RSSI:%i; PWR:%u; BAND:%i SINR:%u; CSQ:%u.\r\n",
			(int)Tick_Get( SECONDS ),
			(int) Network_parameters.rssi,
			(unsigned int) Network_parameters.tx_power,
			(int) Network_parameters.abnd,
			(unsigned int) Network_parameters.sinr,
			(unsigned int) Network_parameters.csq);
}

/**
 * @def BYTE_TO_BINARY_PATTERN
 * @brief
 *
 */
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

/**
 * @def BYTE_TO_BINARY
 * @brief
 *
 */
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

/**
 * @def BYTE_TO_4_BINARY
 * @brief
 *
 */
#define BYTE_TO_4_BINARY(byte)  \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

/**
 * @fn void __configPSM(void)
 * @brief
 *
 * @pre
 * @post
 */
static void __configPSM( void )
{
	char AUX_str[96];
	int tt3324_time[4];
	uint32_t tt3324_val;
	if (NBIOT_PWR_PSM == params_nbiotpwr_mode())
	{
		if (params_psm_edrx_tt3324() > 62)
		{
			tt3324_val     = (params_psm_edrx_tt3324()/60);
			if (tt3324_val > 31)
			{
				tt3324_val     = tt3324_val/6;
				if (tt3324_val > 31)
				{
					tt3324_val = 31;
				}
				tt3324_time[0] = 0x30;
				tt3324_time[1] = 0x31;
				tt3324_time[2] = 0x30;
			}
			else
			{
				tt3324_time[0] = 0x30;
				tt3324_time[1] = 0x30;
				tt3324_time[2] = 0x31;
			}
		}
		else
		{
			tt3324_val     = params_psm_edrx_tt3324()/2;
			if (0 == tt3324_val)
			{
				tt3324_val = 40;
			}
			tt3324_time[0] = 0x30;
			tt3324_time[1] = 0x30;
			tt3324_time[2] = 0x30;
		}

		comm_serial_delete(comm_serial_rcx_bytes_n());
		//"AT+CPSMS=1,,,\"01000001\",\"00000111\"\r"
		sprintf(AUX_str, "AT+CPSMS=1,,,\"001%c%c%c%c%c\",\"%c%c%c%c%c%c%c%c\"\r",
//		sprintf( AUX_str, "AT+CPSMS=1,,,\"101%c%c%c%c%c\",\"%c%c%c%c%c%c%c%c\"\r",
				BYTE_TO_BINARY(params_psm_edrx_tt3412()),
				tt3324_time[0], tt3324_time[1], tt3324_time[2],
				BYTE_TO_BINARY(tt3324_val));
	}
	else if (NBIOT_PWR_ONOFF == params_nbiotpwr_mode())
	{
		comm_serial_delete(comm_serial_rcx_bytes_n());
		sprintf( AUX_str, "AT+CPSMS=0\r");
	}

	comm_serial_trx(&huart1, ( unsigned char * )AUX_str, strlen(AUX_str));
}

/**
 * @fn void __updateStatus(ME910_status, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param _status
 * @param _time_to_wait_cmd
 */
static void __updateStatus( ME910_status _status, uint32_t _time_to_wait_cmd )
{
	Tick_update_tick( TICK_TELIT );
	time_to_wait_cmd = _time_to_wait_cmd;
    status           = _status;
}

/**
  * @brief  Checks the state of the measurement.
  * @retval @arg 0 ->
  * 		@arg 1 ->
  */
static uint32_t __checkMeasurementsState( void )
{
#ifdef MQTT
	if ( ( 1 == dlms_client_get_dlms_enable() ) && (( 0 == CircularBuffer_IsEmpty(dlms_client_get_read_msg_queue()) ) || ( 1 == con_dlms_in_process() ) ) )
	{
    	Tick_update_tick( TICK_TELIT );
    	return 1;
	}
//	if ( ( 0 == mbus_task_end() ) && ( MBUS_SLAVES_METER_TYPE == params_get_uart_meter_type() ) )
//	{
//    	Tick_update_tick( TICK_TELIT );
//    	return 1;
//	}
	if (( 1 == params_get_mqtt_dc_on() ) && ( 1 == mqtt_timer_get_idle_mode() )  && ( 0 == mbus_get_start_comm() ) && (MODBUS_COMM_DISABLED == modbus_sensors_get_comm_state()))
	{
		return 0;
	}
#endif
#if defined(UNE82326)
    if ( ( ( 1 == une82326_get_start_comm() ) && ( 0 == une82326_get_end_comm() ) && ( 0 == params_pulses_on() ) )
      || ( ( 1 == une82326_get_start_comm() ) && ( 1 == params_pulses_on()      ) && ( 0 == pulses_get_init()  ) )
	  ) {
#elif defined(MBUS)
    if ( ( ( 1 == mbus_get_start_comm() ) && ( 0 == mbus_get_end_comm() ) && ( 0 == params_pulses_on() ) )
      || ( ( 1 == mbus_get_start_comm() ) && ( 1 == params_pulses_on()  ) && ( 0 == pulses_get_init()  ) )
	  ) {
#endif
    	Tick_update_tick( TICK_TELIT );
    	return 1;
    }
    if ( ( ( 1 == aqualabo_modbus_get_enable() ) || ( 1 == generic_485_get_enable() ) ) && ( MODBUS_COMM_END != modbus_sensors_get_comm_state() ) ) {
    	Tick_update_tick( TICK_TELIT );
    	return 1;
    }
	if ( ( 1 == AD_GetADOn() ) && ( 1 == AD_GetADCData() ) && ( 0 == AD_getEnd() ) ) {
    	Tick_update_tick( TICK_TELIT );
    	return 1;
	}

	return 0;
}


/**
  * @brief  Narrow Band system thread.
  * @retval None
  */
void ME910_task(void)
{
    uint32_t frame_len, x;
    char *s, *s2;
    static uint32_t csq_try, creg_try, sd_try;
    static time_t   telit_time;//, get_server_time = 0;
    static uint32_t simdet = 0;

#ifdef ME910_LOG
    ME910_log( status );
#endif

    if (1 == reed_on)
    {
     	if ( 0 == test_prod_run_on())
    	{
//    		shutdown_set( 1, 2 );
    		if ( 0 == dlms_client_get_resend() )
    		{
    			dlms_client_set_resend(1);
    		}
    		shutdown_set_start_count(0);
    		rtc_system_Set_Stop_Mode(0);
    	}
    }

    if (0 == shutdown_initTelitModule())
    {
    	return;
    }

//    if (1 == Tick_cloud_time_init()) // TODO:REVIEW
    {
    	if (1 == __checkMeasurementsState())
    	{
    		return;
    	}
    }

    switch (status)
    {
        case M_BOOT0: // MODEM ON PROCEDURE
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Started.\r\n", (int)Tick_Get( SECONDS ));
        	csq_ok = 0;
			sME910.IMEI[0] = '\0';
			csq_try  = 500;
			creg_try = 500;
			connect_time_ini = HAL_GetTick();
//			ME910_set_error(ERROR_TRYING_TO_CONNECT);
			ME910_EraseBuffer();
//			shutdown_set_wchdg_time(SHUTDOWN_STAND_BY_MAX_TIME);
			Telit_socket_set_available(NOT_CONNECTED);
//			Telit_disable();
//			comm_serial_disable();
			__modemEnable();
			if (GPIO_PIN_SET == ME910_CHECK_PSM)
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Switched OFF: Start Switch ON Process.\r\n", (int)Tick_Get( SECONDS ));
				ME910_enable();
				comm_serial_enable();
				__startModemOnOffProc();
				ME910_setPSMMode(0);
				__processOKCommand(LAST_ME910_COMMAND, NULL, ON_DELAY, M_BOOT_SW_ON);
			}
			else
			{
				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Switched ON: Start Switch OFF Process.\r\n", (int)Tick_Get( SECONDS ));
				__startModemOnOffProc();
				ME910_setPSMMode(0);
				__processOKCommand( LAST_ME910_COMMAND, NULL, OFF_DELAY + 2, M_BOOT_SW_OFF );
			}
        break;

        case M_BOOT_SW_OFF:
    		if (CHECK_ELAPSED_TIME( time_to_wait_cmd, TICK_TELIT))
    		{
//    			HAL_Delay(200);
    			__endModemOnOffProc();
    			LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Stop Switch Off Process.\r\n", (int)Tick_Get( SECONDS ));
    			uint32_t tickstart = HAL_GetTick();
    			do {
    				asm("nop");
    			} while((GPIO_PIN_RESET == ME910_CHECK_PSM) && ((HAL_GetTick() - tickstart) < 5000));

    			if (GPIO_PIN_RESET == ME910_CHECK_PSM)
    			{
    				status = M_BOOT0;
    				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Cannot Switch Off!!!\r\n", (int)Tick_Get( SECONDS ));
    				Telit_modem_off_prc();
    				uint32_t tickstart = HAL_GetTick();
    				do {
        				asm("nop");
        			} while((GPIO_PIN_RESET == ME910_CHECK_PSM) && ((HAL_GetTick() - tickstart) < 5000));
    				if (GPIO_PIN_SET == ME910_CHECK_PSM)
    				{
        	        	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Switch Off.\r\n", (int)Tick_Get( SECONDS ));
        				ME910_enable();
        				comm_serial_enable();
        				__startModemOnOffProc();
        				__processOKCommand(LAST_ME910_COMMAND, NULL, ON_DELAY, M_BOOT_SW_ON);
    				}
    			}
    			else
    			{
    	        	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Switch Off.\r\n", (int)Tick_Get( SECONDS ));
    				ME910_enable();
    				comm_serial_enable();
    				__startModemOnOffProc();
    				__processOKCommand(LAST_ME910_COMMAND, NULL, ON_DELAY, M_BOOT_SW_ON);
    			}
    		}
        break;

        case M_BOOT_SW_ON:
    		if(CHECK_ELAPSED_TIME( time_to_wait_cmd, TICK_TELIT))
    		{
    			__endModemOnOffProc();
    			uint32_t counter = 0,tickstart_1 = 0;
    			uint32_t tickstart = HAL_GetTick();

    			do {
    				counter++;
    			} while((GPIO_PIN_SET == ME910_CHECK_PSM) && ((HAL_GetTick() - tickstart) < 5000));

    			tickstart_1 = HAL_GetTick();
    			(void)tickstart_1;
//    			uint32_t counter = 0,tickstart_1 = 0;
//    			uint32_t tickstart = HAL_GetTick();
//    			do {
//    				counter++;
//    			} while( ( GPIO_PIN_SET == ME910_CHECK_PSM ) && ( (HAL_GetTick() - tickstart) < 5000 ) );
//    			tickstart_1 = HAL_GetTick();
//    			(void)tickstart_1;
    			if (GPIO_PIN_SET == ME910_CHECK_PSM)
    			{
    				status = M_BOOT0;
    			}
    			else
    			{
    				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Switch On.\r\n", (int)Tick_Get( SECONDS ));
    				__processOKCommand( LAST_ME910_COMMAND, NULL, 5, M_BOOT1 );
    			}
    		}
        break;

        case M_BOOT1:
        	if (CHECK_ELAPSED_MILISEC( time_to_wait_cmd, TICK_TELIT))
        	{
        		__processOKCommand(LAST_ME910_COMMAND, NULL, 2, M_BOOT2);//if (1 == une82326_get_end_comm) DEBUG: Check...Add??
    		}
        break;

        case M_BOOT2: // START AT CMD
            if (CHECK_ELAPSED_TIME(time_to_wait_cmd, TICK_TELIT))
            {
            	ME910_set_error(ERROR_TRYING_TO_CONNECT);
            	if (TYPE_COMM_NBIOT == params_get_type_comm())
            	{
            		leds_set_NET_status(NB_N);
            	}
            	else
            	{
            		leds_set_NET_status(NB_MODULE_ERROR);
            	}
                __processOKCommand(AT, NULL, 5, M_BOOT3);
            }
        break;

        case M_BOOT3:
            if (ME910_StringEqual("AT\r\r\nOK\r\n", 9) || ME910_StringEqual("OK", 2))
            {
                __processOKCommand(ATE0, NULL, 5, M_ECHO_OFF);
            	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> AT OK.\r\n", (int)Tick_Get( SECONDS ));
            }
        break;

        case M_ECHO_OFF:
            if (ME910_StringEqual("\r\nOK\r\n", 6))
            {
                __processOKCommand(CMEE, NULL, 5, M_CGMR);
            }
        break;

        case M_BAND:
            if (ME910_StringEqual("\r\nOK\r\n", 6))
            {
                __processOKCommand(BAND, NULL, 5, M_CGMR);
            }
        break;	
        case M_CGMR:
            if (ME910_StringEqual("\r\nOK\r\n", 6))
            {
                __processOKCommand(CGMR, NULL, 5, M_CMEE);
            }
        break;

        case M_CMEE:
            if (ME910_StringEqual("\r\nOK\r\n", 6))
            {
            	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> \r\nRESULT_SPKG:%s.\r\n", (int)Tick_Get( SECONDS ), ME910_data+2);
            	if ( (ME910_data[9] == '1') && ( (ME910_data[10] == '1') || (ME910_data[10] == '3')) )
            	{
            		simdet = 2;
            	}
            	else
            	{
            		simdet = 1;
            	}
            	__processOKCommand(CEMODE, NULL, 5, M_CEMODE);
            }
        break;

        case M_CEMODE:
        	if (ME910_StringEqual("\r\nOK\r\n", 6))
        	{
                __processOKCommand(CCIOTOPT, NULL, 5, M_CCIOTOPT);
            }
        break;

        case M_CCIOTOPT:
        	if (ME910_StringEqual("\r\nOK\r\n", 6))
        	{
                __processOKCommand(CGSN, NULL, 25, M_IMEI);
            }
        break;

        case M_IMEI:
        	if (CHECK_ELAPSED_MILISEC(time_to_wait_cmd * 100, TICK_TELIT))
        	{
				frame_len = comm_serial_rcx((unsigned char *) ME910_data);
				if (frame_len)
				{
					s = (char *) ME910_data;

					if (ME910_StringEqual("\r\nOK\r\n", 6))
					{
						__getIMEI();
	                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> IMEI: %s.\r\n",(int)Tick_Get( SECONDS ), param.imei);
						comm_serial_delete(frame_len);
						char _str1[10];
						sprintf(_str1, "%d\r", (int)simdet);
						__write_cmd(SIMDET, (uint8_t *)_str1);
						__processOKCommand(LAST_ME910_COMMAND, NULL, 5, M_SIMDET);
					}
				}
        	}
        break;

        case M_SIMDET:
			if (ME910_StringEqual("\r\nOK\r\n", 6))
			{
				__processOKCommand(SAVESIMDET, NULL, 5, M_SAVE_SIMDET);
			}
        break;

        case M_SAVE_SIMDET:
			if (ME910_StringEqual("\r\nOK\r\n", 6))
			{
				__processOKCommand(CPIN, NULL, 5, M_PIN);
			}
        break;

        case M_PIN:
            if (ME910_StringEqual("\r\nOK\r\n", 6))
            {
                if (ME910_StringEqual("+CPIN: READY", 12))
                {
                	csq_ok = 1;
                	__processOKCommand(CCID, NULL, 5, M_CCID);
                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> SIM Ready.\r\n", (int)Tick_Get( SECONDS ));
                }
            }
            else if (ME910_StringEqual("\r\n+CME ERROR: SIM", 17))
            {
//            	shutdown_set(1, params_config_read_time());
            	params_attach_error_inc_SIM_error();
            	params_attach_insert_log(SIM_ERR);
            	comm_serial_delete(comm_serial_rcx_bytes_n());
            	__write_cmd(CPIN, NULL);
				__processOKCommand(LAST_ME910_COMMAND, NULL, 5, M_PIN);
            	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> SIM ERROR!.\r\n", (int)Tick_Get( SECONDS ));
            }
        break;

        case M_CCID:
        	if (CHECK_ELAPSED_MILISEC( time_to_wait_cmd * 100, TICK_TELIT))
        	{
				frame_len = comm_serial_rcx((unsigned char *) ME910_data);

				if (frame_len)
				{
					s = (char *) ME910_data;

					if ((ME910_StringEqual("#CCID: ", 7)) && (ME910_StringEnds("\r\n", 2, comm_serial_rcx_bytes_n())))
					{
						__getICCID();
						__processOKCommand(CIMI, NULL, 5, M_CIMI);
	                	LOGLIVE(LEVEL_3, "LOGLIVE> %d ME910> CCID: %s.\r\n", (int)Tick_Get( SECONDS ), sME910.ICCID);
					}
				}
        	}
        break;

        case M_CIMI:
        	if (CHECK_ELAPSED_MILISEC(time_to_wait_cmd * 100, TICK_TELIT))
        	{
				frame_len = comm_serial_rcx(( unsigned char *) ME910_data);

				if (frame_len)
				{
					s = (char *) ME910_data;
					if (ME910_StringEqual("\r\nOK\r\n", 6))
					{
						__getIMSI();
						__processOKCommand( CFUN, NULL, 5, M_CFUN );
	                	LOGLIVE(LEVEL_3, "LOGLIVE> %d ME910> IMSI: %s.\r\n", (int)Tick_Get( SECONDS ), sME910.IMSI);
					}
				}
        	}
        break;

        case M_CFUN:
        	if (ME910_StringEqual("\r\nOK\r\n", 6))
        	{
//                uint8_t type_comm = params_get_type_comm();
//                char _str[3];
//                sprintf(_str, "%d\r", (int)type_comm);
//        		__processOKCommand(WS46IOT, (uint8_t *)_str, 5, M_WS46IOT);
        		__processOKCommand(WS46IOT, NULL, 5, M_WS46IOT);
        	}
        	else if (ME910_StringEqual("\r\nERROR\r\n", 9))
        	{
        		comm_serial_delete(comm_serial_rcx_bytes_n());
        		Tick_update_tick(TICK_TELIT);
        	}
        break;

        case M_WS46IOT:
        	if ((ME910_StringEqual("\r\nOK\r\n", 6)) || (ME910_StringEqual("CME ERROR: unknown", 18)))
        	{
        		__processOKCommand(WS46, NULL, 5, M_WS46);
        	}
        break;

        case M_WS46:
			if (ME910_StringEqual("\r\nOK\r\n", 6))
			{
				__processOKCommand(GPIO1, NULL, 5, M_GPIO_1);
			}
        break;

        case M_GPIO_1:
			if (ME910_StringEqual("\r\nOK\r\n", 6))
			{
				__processOKCommand(GPIO, NULL, 5, M_GPIO);
			}
        break;

        case M_GPIO:
        	if (ME910_StringEqual("\r\nOK\r\n", 6))
        	{
        		__processOKCommand(SLED, NULL, 5, M_SLED);
        	}
        break;

        case M_SLED:
        	if (ME910_StringEqual("\r\nOK\r\n", 6))
        	{
        		__processOKCommand(SKIPESC, NULL, 5, M_SKIPESC);
        	}
        break;

        case M_SKIPESC:
        	if (ME910_StringEqual("\r\nOK\r\n", 6))
        	{
        		char _str[120];
        		comm_serial_delete(comm_serial_rcx_bytes_n());
        		/* Testing process */
        		if ((1 == ME910_get_network_params_get()) && (55 == ME910_get_dk_network_params_get()))
        		{
        			sprintf(_str, "AT+CGDCONT=1,\"IP\",\"e3tcity.movistar.es\"\r");
        			comm_serial_trx(&huart1, (unsigned char *)_str, strlen(_str));
        			__updateStatus(/*M_CGATT_0*/M_CGDCONT_1, 20);
        		}
        		/* Regular operation */
        		else
        		{
        			sprintf(_str, "\"%s\"\r", param.APN[ME910_APN_get_new_index()].name);
        			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> APN[%i]: %s.\r\n", (int)Tick_Get( SECONDS ), (int) ME910_APN_get_index(), param.APN[ME910_APN_get_index()].name);
        			__write_cmd(CGDCONT, (uint8_t *) _str);
        			__updateStatus(/*M_CGATT_0*/M_CGDCONT_1, 20);
        		}
        	}
        break;

        case M_CGATT_0:
        	if (CHECK_ELAPSED_MILISEC( time_to_wait_cmd * 100, TICK_TELIT))
        	{
				if (ME910_StringEqual("OK", 2))
				{
					__processOKCommand(CGATT0, NULL, 5, M_CGATT_1);
				}
        	}
        break;

        case M_CGATT_1:
        	if (ME910_StringEqual("OK", 2))
        	{
        		__processOKCommand(CGATT1, NULL, 5, M_CGDCONT_1);
        	}
        break;

        case M_CGDCONT_1:
        	if (CHECK_ELAPSED_MILISEC(1 * 100, TICK_TELIT))
        	{
				if (ME910_StringEqual("\r\nOK\r\n", 6))
				{
	        		/* Testing process */
					if ((1 == ME910_get_network_params_get()) && (55 == ME910_get_dk_network_params_get()))
					{
						char AUX_str[96];
//#ifndef CAT_M1
						if (TYPE_COMM_CATM1 == params_get_type_comm())
						{
							sprintf(AUX_str, "AT+COPS=1,2,\"21407\",8\r");
						}
//#else
						else
						{
							sprintf(AUX_str, "AT+COPS=1,2,\"21407\",9\r");
						}
//#endif
		        		comm_serial_trx(&huart1, ( unsigned char * )AUX_str, strlen(AUX_str));
		        		__updateStatus( M_COPS, 50 );
					}
					/* Regular operation */
					else
					{
						char _str[96];
//						__processOKCommand( COPS, NULL, 50, M_COPS );
						uint32_t is_global = 0;
						__searchPLMNbyAPNInSIM(&is_global);
//#ifndef CAT_M1
						if (TYPE_COMM_CATM1 == params_get_type_comm())
						{
							if (0 == is_global)
							{
								sprintf(_str, "\"%s\",8\r", __searchPLMNbyAPNInSIM(&is_global));
							}
							else
							{
								sprintf(_str, "\"%s\",8\r", __searchPLMNbyAPN(ME910_APN_get_index()));
							}
//#else
						}
						else
						{
							if (0 == is_global)
							{
								sprintf(_str, "\"%s\",9\r", __searchPLMNbyAPNInSIM(&is_global));
							}
							else
							{
								sprintf(_str, "\"%s\",9\r", __searchPLMNbyAPN(ME910_APN_get_index()));
							}
//#endif
						}
	        			if ( 0 == memcmp(_str,"\"NOPLMN\"",8) )
	        			{
	        				__processOKCommand( COPS, NULL, 50, M_COPS );
	        				LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> PLMN: %s.\r\n", (int)Tick_Get( SECONDS ), ME910_strings[COPS] );
	        			}
	        			else
	        			{
		        			LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> PLMN: %s.\r\n", (int)Tick_Get( SECONDS ), _str);
		        			__write_cmd(COPS_TABLE, (uint8_t *) _str);
		        			__updateStatus(M_COPS, 20);
	        			}
	                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> APN[%i]: %s.\r\n", (int)Tick_Get( SECONDS ), (int) ME910_APN_get_index(), param.APN[ME910_APN_get_index()].name);
	                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> COPS OK.\r\n", (int)Tick_Get( SECONDS ));
					}
				}
        	}
        break;

        case M_COPS:
        	if (CHECK_ELAPSED_MILISEC(500, TICK_TELIT))
        	{
				if (ME910_StringEqual("\r\nOK\r\n", 6))
				{
					ME910_EraseBuffer();
					__processOKCommand( CSQ, NULL, 20/*60*/, M_CSQ );
				}
        	}
        break;

        case M_CSQ:
        	if (CHECK_ELAPSED_MILISEC(time_to_wait_cmd * 100, TICK_TELIT))
        	{
				frame_len = comm_serial_rcx(( unsigned char *)ME910_data);
				if (frame_len)
				{
					if (ME910_StringEqual("\r\nOK\r\n", 6))
					{
						if (ME910_StringEqual("+CSQ:", 5))
						{
							x = __getCSQValue();
							Tick_update_tick(TICK_TELIT);

							if ((x >= 99) || (x < 2))
							{
								if (csq_try)
								{
									csq_try--;
									time_to_wait_cmd = 2;
									__write_cmd( CSQ, NULL );
								}
								else
								{
									params_attach_error_inc_CSQ_error();
									params_attach_insert_log(CSQ_ERR);
									csq_ok = -1;
									sME910.CSQ = x;
									HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, sME910.CSQ);
									Network_parameters.csq = sME910.CSQ;
									ME910_EraseBuffer();
									__write_cmd( CEREG, NULL );
									__updateStatus( M_CEREG, 10 );
				                	LOGLIVE(LEVEL_3, "LOGLIVE> %d ME910> CSQ: %u.\r\n", (int)Tick_Get( SECONDS ), (unsigned int) sME910.CSQ);
								}
							}
							else
							{
								csq_ok     = 1;
								sME910.CSQ = x;
								HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, sME910.CSQ);
								Network_parameters.csq = sME910.CSQ;
								ME910_EraseBuffer();
								__write_cmd( CEREG, NULL );
								__updateStatus( M_CEREG, 10 );
			                	LOGLIVE(LEVEL_3, "LOGLIVE> %d ME910> CSQ: %u.\r\n", (int)Tick_Get( SECONDS ), (unsigned int) sME910.CSQ);
							}
							comm_serial_delete(frame_len);
						}
					}
				}
        	}
        break;

        case M_CEREG:
        	if ( CHECK_ELAPSED_MILISEC(time_to_wait_cmd * 100, TICK_TELIT))
        	{
				frame_len = comm_serial_rcx((unsigned char *) ME910_data);

				if (frame_len)
				{
					if (ME910_StringEqual("\r\nOK\r\n", 6))
					{
						if (ME910_StringEqual("+CEREG:", 6))
						{
							char AUX_str[103];
							s2 = s = (char *) ME910_data;
							s  = strstr(s2, ",");
							s = s + 1;
							x = atoi(s);
							Network_parameters.creg = x;
							Tick_update_tick(TICK_TELIT);
							switch (x)
							{
								case 1:
									/* FLUSHTHRU */
								case 5:
									ME910_set_error(ERROR_INIT);

									/* PSM*/
					                if (ME910_getPSMMode() == 1)
					                {
					        			char AUX_str[103];
					                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> REGISTERED in PSM Mode.\r\n", (int)Tick_Get( SECONDS ));

					    				if ( 0 == params_psm_edrx_edrx() )
					    				{
					    					if (TYPE_COMM_CATM1 == params_get_type_comm())
					    					{
					    						sprintf( AUX_str, "AT+CEDRXS=0,4\r");
					    					}
					    					else
					    					{
					    						sprintf( AUX_str, "AT+CEDRXS=0,5\r");
					    					}
					    				}
					    				else
					    				{
					    					if (TYPE_COMM_CATM1 == params_get_type_comm())
					    					{
					    						sprintf( AUX_str, "AT+CEDRXS=1,4,\"%c%c%c%c\"\r",
					    								BYTE_TO_4_BINARY((params_psm_edrx_edrx()-(1))));
					    					}
					    					else
					    					{
					    						sprintf( AUX_str, "AT+CEDRXS=1,5,\"%c%c%c%c\"\r",
					    								BYTE_TO_4_BINARY((params_psm_edrx_edrx()-(1))));
					    					}
					    				}
					    				comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
					    				__updateStatus(M_CPSMS, 5);
					                }
					                /* ON OFF*/
					                else
					                {
					                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> REGISTERED in ON/OFF Mode.\r\n", (int)Tick_Get( SECONDS ));
					                	/* TEST procedure */
					                	if ((1 == ME910_get_network_params_get()) && (55 == ME910_get_dk_network_params_get()))
					               		{
					                			sprintf(AUX_str, "AT+CGDCONT=1,\"IP\",\"e3tcity.movistar.es\"\r");
					                			comm_serial_trx(&huart1, ( unsigned char * )AUX_str, strlen(AUX_str));
					                			__updateStatus( M_eDRX, 5 );
					                	}
					                	/* Regular operation */
					                	else
					                	{
					                		ME910_APN_set_current(ME910_APN_get_index());
					                		sprintf(AUX_str, "\"%s\"\r", param.APN[ME910_APN_get_index()].name);
					                    	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> APN[%i]: %s.\r\n", (int)Tick_Get( SECONDS ), (int) ME910_APN_get_index(), param.APN[ME910_APN_get_index()].name);
					                    	ME910_APN_get_new_index();
					                		__write_cmd(CGDCONT, (uint8_t *) AUX_str);
					                		__updateStatus(M_eDRX, 5);
					                	}
					                }
								break;

								default:
									if (creg_try)
									{
										creg_try--;
										__write_cmd(CEREG, NULL);
										time_to_wait_cmd = 2;
									}
									else
									{
										params_attach_error_inc_CEREG_error();
										params_attach_insert_log(CEREG_ERR);
										LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> CEREG ERROR: %d.\r\n", (int)Tick_Get( SECONDS ), (int)x);
										if (0 == ME910_get_network_params_get())
										{
											__updateStatus(M_DELAY, 1);
										}
										else
										{
											ME910_set_network_params_get(2);
											shutdown_setInitTelitModule(0);
											ME910_reset_status();
										}
									}
								break;
							}

							comm_serial_delete(frame_len);
						}
					}
				}
        	}
        break;

        case M_DNS:
        	if (ME910_StringEqual("OK", 2))
        	{
        		comm_serial_trx(&huart1, (unsigned char *) "AT#DNS=1,\"10.241.120.204\",\"10.241.20.204\"\r", 42);
                __updateStatus(M_CGDCONT, 5);
        	}
        break;

        case M_eDRX:
        	if(CHECK_ELAPSED_MILISEC(time_to_wait_cmd * 100, TICK_TELIT))
        	{
        		if ( ME910_StringEqual("\r\nOK\r\n", 6))
        		{
        			char AUX_str[96];
//        			"AT+CEDRXS=1,5,\"0001\"\r"
    				comm_serial_delete(comm_serial_rcx_bytes_n());
                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> COPS OK.\r\n", (int)Tick_Get( SECONDS ));

    				if ( 0 == params_psm_edrx_edrx() )
    				{
    					if (TYPE_COMM_CATM1 == params_get_type_comm())
    					{
    						sprintf( AUX_str, "AT+CEDRXS=0,4\r");
    					}
    					else
    					{
    						sprintf( AUX_str, "AT+CEDRXS=0,5\r");
    					}
    				}
    				else
    				{
    					if (TYPE_COMM_CATM1 == params_get_type_comm())
    					{
    						sprintf( AUX_str, "AT+CEDRXS=1,4,\"%c%c%c%c\"\r",
    								BYTE_TO_4_BINARY((params_psm_edrx_edrx()-(1))));
    					}
    					else
    					{
    						sprintf( AUX_str, "AT+CEDRXS=1,5,\"%c%c%c%c\"\r",
    								BYTE_TO_4_BINARY((params_psm_edrx_edrx()-(1))));
    					}
    				}

    				comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
    				__updateStatus(M_CPSMS, 5);
				}
        		else if (ME910_StringEqual("CME ERROR: context already activated", 36))
        		{
					asm("nop");
                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> COPS ERROR!.\r\n", (int)Tick_Get( SECONDS ));
                	ME910_set_error(ERROR_TRYING_TO_CONNECT);
                    __updateStatus(M_DELAY, 10);
				}
        	}
        break;

        case M_CPSMS:
			if (ME910_StringEqual("OK", 2) || ME910_StringEqual("CME ERROR: context already activated", 36))
			{
				__configPSM();
				__updateStatus(M_MONI, 5);
			}
        break;

        case M_MONI:
        	if (ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()))
        	{
        		__processOKCommand(ATMONI, NULL, 5, M_MONI_PARSE);
        	}
        	else if (CHECK_ELAPSED_TIME(time_to_wait_cmd, TICK_TELIT))
        	{
        		asm("nop");
        	}
        break;

        case M_MONI_PARSE:
        	if (ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()))
        	{
				frame_len = comm_serial_rcx((unsigned char *) ME910_data);
				if (frame_len != 0)
				{
					__parseMONI(frame_len);
					__processOKCommand(ATRFSTS, NULL, 5, M_RFSTS);
				}
        	}
        break;

        case M_RFSTS:
        	if (ME910_StringEqual("#RFSTS:", 7) && ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()))
        	{
        		frame_len = comm_serial_rcx((unsigned char *) ME910_data);
        		if (frame_len != 0)
        		{
        			__parseRFSTS( frame_len);

        			if (0 == ME910_get_network_params_get())
        			{
        				__processOKCommand(AT, NULL, 2, M_CGDCONT);
        			}
        			else
        			{
        				ME910_set_error(ERROR_CONNECTED);
        				connect_time_end = HAL_GetTick();
        				if (GPIO_PIN_RESET == ME910_CHECK_PSM)
        				{
        					Telit_modem_off_prc();
        					ME910_checkPWRMON();
        				}
        				comm_serial_disable();
        				Telit_disable();
        				Telit_modem_disable();

        				if ((1 == ME910_get_network_params_get()) && (55 == ME910_get_dk_network_params_get()))
        				{
        					Tick_system_time_init(1533278899);
        					shutdown_reset_watchdog();
        				}

        				ME910_set_network_params_get(2);
        				shutdown_setInitTelitModule(0);
        				status = M_BOOT0;
        			}
        		}
        	}
        break;

        case M_CGDCONT:
        	if (ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()))
        	{
                char AUX_str[96];
                comm_serial_delete( comm_serial_rcx_bytes_n());
                sprintf(AUX_str, "AT#SGACT=1,1\r");
                comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
                __updateStatus(M_SGACT, 50);
            }
        break;

        case M_SGACT:
        	if (ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()))
        	{
                if (ME910_StringEqual("#SGACT:", 7))
                {
                	char AUX_str[96];
                	comm_serial_delete(comm_serial_rcx_bytes_n());
    				__getIP();
                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> IP: %s.\r\n", (int)Tick_Get( SECONDS ), sME910.IP);
                    sprintf(AUX_str, "AT+CGCONTRDP\r");
                    comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
                    __updateStatus(M_CGCONTRDP, 5);
                }
                else if ((ME910_StringEqual("CME ERROR: context already activated", 36)) || (ME910_StringEqual("\r\nERROR\r\n", 9)))
                {
                	params_attach_error_inc_IP_error();
                	params_attach_insert_log(IP_ERR);
                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> ERROR-context already activated-unable to get IP!\r\n", (int)Tick_Get( SECONDS ));
                	/* TOASK Error and goes to SENDSSLSECDATA */
                	if (ME910_getPSMMode() == 1)
                	{
                		comm_serial_delete(comm_serial_rcx_bytes_n());
                		__write_cmd(SSLCFG, NULL);
                		__updateStatus(M_SENDSSLSECDATA, 5);
                	}
                }
                else if (ME910_StringEqual("CME ERROR: ", 11))
                {
                	params_attach_error_inc_IP_error();
                	params_attach_insert_log(IP_ERR);
                	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> ERROR unable to get IP!\r\n", (int)Tick_Get( SECONDS ));
                	ME910_set_error(ERROR_TRYING_TO_CONNECT);
                	__updateStatus(M_DELAY, 10);
                }
            }
        	else if ((ME910_StringEqual("CME ERROR: context already activated", 36)) || (ME910_StringEqual("\r\nERROR\r\n", 9)))
        	{
//            	params_attach_error_inc_IP_error();
//            	params_attach_insert_log(IP_ERR);
            	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> ERROR-context already activated-unable to get IP!\r\n", (int)Tick_Get( SECONDS ));
//            	if (ME910_getPSMMode() == 1)
            	{
            		comm_serial_delete(comm_serial_rcx_bytes_n());
            		//at#sslcfg=1,1,1500,2000,1000,100,1,2,0,0
            		__write_cmd(SSLCFG, NULL);
            		__updateStatus(M_SENDSSLSECDATA, 5);
            	}
            }
        	else if (ME910_StringEqual("CME ERROR: activation failed", 28))
        	{
        		HAL_Delay(10);
        		comm_serial_rcx((unsigned char *) ME910_data);
//        		params_attach_error_inc_IP_error();
//            	params_attach_insert_log(IP_ERR);
            	LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> PSM Mode:%d.ERROR unable to get IP!:%s\r\n",
            			(int)Tick_Get( SECONDS ), (int)ME910_getPSMMode(), ME910_data);
            	ME910_set_error(ERROR_TRYING_TO_CONNECT);
//                __updateStatus(M_DELAY, 10);
            	if (ME910_getPSMMode() == 1)
            	{
            		comm_serial_delete(comm_serial_rcx_bytes_n());
            		//at#sslcfg=1,1,1500,2000,1000,100,1,2,0,0
            		__write_cmd(SSLCFG, NULL);
            		__updateStatus(M_SENDSSLSECDATA, 5);
            	}
            	else
            	{
            		params_attach_error_inc_IP_error();
                	params_attach_insert_log(IP_ERR);
            		__updateStatus(M_DELAY, 10);
            	}
            }
        break;

        case M_CGCONTRDP:
            if (ME910_StringEqual("CGCONTRDP:", 10) && ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());

            	if (ME910_getPSMMode() == 1)
            	{
            		__write_cmd(SSLCFG, NULL);
            	}
            	else
            	{
            		__write_cmd(SSLCFG_2, NULL);
            	}

#ifdef HTTPS
            	__updateStatus(M_SSLCFG, 5);
#else
            	__updateStatus(M_SENDSSLSECDATA, 5);
#endif
            }
        break;

        case M_SSLCFG:
            if (ME910_StringEqual("OK", 2))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());
            	__write_cmd(SSLSECCFG, NULL);
            	__updateStatus(M_SSLSECCFG, 10);
            }
            else if (ME910_StringEqual("CME ERROR: SSL not activated", 28))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());
               	__write_cmd(SSLDIS, NULL);
                __updateStatus(M_SSLDIS, 10);
            }
        break;

        case M_SSLSECCFG:
            if (ME910_StringEqual("OK", 2) || ME910_StringEqual("CME ERROR: SSL not activated", 28))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());
               	__write_cmd(SSLDIS, NULL);
                __updateStatus(M_SSLDIS, 10);
            }
        break;

        case M_SSLDIS:
            if (ME910_StringEqual("OK", 2) || ME910_StringEqual("CME ERROR: SSL already activated", 28))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());
            	__write_cmd(SSLEN, NULL);
                __updateStatus(M_SSLEN, 10);
            }
        break;

        case M_SSLEN:
            if (ME910_StringEqual("OK", 2) || ME910_StringEqual("CME ERROR: SSL already activated", 32))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());
#ifndef HTTPS
            	if (ME910_getPSMMode() == 1)
            	{
                	__write_cmd(CCLK, NULL);
                    __updateStatus(M_CLOCK, 10);
                }
            	else
#endif
            	{
                	comm_serial_trx(&huart1, (unsigned char *) "AT#SSLSECDATA=4,1,1,1939\r", 25);//"AT#SSLSECDATA=2,1,1,1944\r"//"AT#SSLSECDATA=4,1,1,1939\r"
                	__updateStatus(M_SSLSECCA, 10);
                }
            }
        break;

        case M_SSLSECCA:
            if (ME910_StringEqual(">", 1))
            {
#ifdef HTTPS
            	char AUX_str[1950];
				#define CTRL_Z 26
            	comm_serial_delete(comm_serial_rcx_bytes_n());
#if 1
//Root
            	sprintf(AUX_str,
            			"-----BEGIN CERTIFICATE-----\n"
            			"MIIFazCCA1OgAwIBAgIIK0RQTrBwBHswDQYJKoZIhvcNAQELBQAwQzETMBEGA1UE\n"
            			"AwwKQ0EgUm9vdCBRQTEQMA4GA1UECwwHQUREQyBDQTENMAsGA1UECgwEQUREQzEL\n"
            			"MAkGA1UEBhMCQUUwHhcNMjIxMTI4MTMwNjA0WhcNNDcxMTIyMTMwNjA0WjBDMRMw\n"
            			"EQYDVQQDDApDQSBSb290IFFBMRAwDgYDVQQLDAdBRERDIENBMQ0wCwYDVQQKDARB\n"
            			"RERDMQswCQYDVQQGEwJBRTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIB\n"
            			"AKkGd0WEmYokYf5saDUXPxJLYm29DVlJ7vdP3/nL3//QlMmJ1HjyuT4IH4y/hePF\n"
            			"Iy5ke+22ZzPumU3Van9jqy/e0Ht/zfQ5kHrofwsIQOlf3vagGXG/ezjMTCEuzFSv\n"
            			"6vXKxF73jeFyfMV19ybfZTRHVfQcqN79RBrCS/diFcvocGxmIpSjcesVWWaqgPG6\n"
            			"uB9yHY3ed8uWgZve7/SIYHBxUUP6imemUhoxrOJuUTNrfydHfDu4nE7pzx1teTfQ\n"
            			"YO7srlI8wN6Ky4f/ZvkfMbb2jGgiqK1WywwFdsYG+nhKcWzsltkUqpumVkcE6lGu\n"
            			"dwpmXN0vY1k4C8BLRxeLUXMC/Yhu25NA3iHOFYES6gH6kc4EBd/D0Ir+4ZgieWKF\n"
            			"sVKBBrC0nuRjtG7+ZXqfhI+JJIyyBKhloy2Ms7hGnncfsqhfp6EeXMKrahgtLwbT\n"
            			"xdgpn4XzdjKaqCjfD6sHLjr+xenmikSgL/6PRg7jL4R+6bYQQ7hJIOlaRbjwRPoL\n"
            			"Jvu53nMegbxgjZ16gLK2V7rOzkfA5hAizFuF7TkOPDNff0S1Jk08qf+eDsdpIPu4\n"
            			"yBlrWl61Pod9ls1kWogEjhyNPyK3oAB0Bo0UL3/vyt9pnRWgnQy1fCQK2t/pcTOd\n"
            			"//LhUpV3+qL/9slz8dXk4WMvkeJS8OyfiqCEJJyL+X69AgMBAAGjYzBhMA8GA1Ud\n"
            			"EwEB/wQFMAMBAf8wHwYDVR0jBBgwFoAUYvnOEIpLUiuWLFjR0E35cn9lAcwwHQYD\n"
            			"VR0OBBYEFGL5zhCKS1IrlixY0dBN+XJ/ZQHMMA4GA1UdDwEB/wQEAwIBhjANBgkq\n"
            			"hkiG9w0BAQsFAAOCAgEAacdGD3YotOSsBPvD3JCoGAU7fZ3lZb2Nmh9MGy8RToua\n"
            			"cFxLzIVflU5fx/zmJ0LFFaAAlfSH3qYfXog8MLLiZTfqnRf7SkJ/xJ4bSq9ghfHs\n"
            			"Kzv8kEMLaHBGqVBxo611QMCl4ldNt2Fyp1J4+dhDHAzkkQ3BPtMOtWjZFvc7rSme\n"
            			"XSMFvNVrnPx5aNmH/X6qM0TjOUsbs74GAI6kThGN2mGqXSj+DPxsR0RqAsWYIrH8\n"
            			"TJ/Fu4z8posJwWZtUeE0vVU5kUVqSA1N1f34REvVMWKKQV1LmQ8gH0qOLBAtzltx\n"
            			"jOly6pY2rnjwUEIiD2vdT3YGJdB1+oNqoN+rx2cyuZBOhJihf5CKqHn+4215l14W\n"
            			"nFSUluaVlQCt9LlSptF7bgg+YGuK2itMKml51AsaEmFwTkaQBXmbNrlReWauUDzs\n"
            			"j6r+0qz/WicnkZOqMSADIW/V4NJjiWA0CnHeoTKqKVof9+msQ05CdS5FLZwoRQLK\n"
            			"bIVGjPNdtZ3JN9ElPAxiV3CW0RgxsolHxbyV5nKmFfG5EROd5vQHc2hS580wLU0r\n"
            			"knRQXtNP2jcS57l43f04VsPR7gPc5+XJ9s1V5eO7Vk/O+vsacQpuuWN7aigmbXk/\n"
            			"SX8i3EvpxqiRg327DvBbfmAhGzGNdNUdK5Ubmw0zS5mXUQmW7HmR9DeflqGvrAw=\n"
            			"-----END CERTIFICATE-----\n%c", CTRL_Z);
#endif
#if 0
//SubRoot
            	sprintf(AUX_str,
            			"-----BEGIN CERTIFICATE-----\n"
            			"MIIFbjCCA1agAwIBAgIIXf/w+UafhjUwDQYJKoZIhvcNAQELBQAwQzETMBEGA1UE\n"
            			"AwwKQ0EgUm9vdCBRQTEQMA4GA1UECwwHQUREQyBDQTENMAsGA1UECgwEQUREQzEL\n"
            			"MAkGA1UEBhMCQUUwHhcNMjIxMTI4MTcyNDI3WhcNNDcxMTIyMTMwNjA0WjBGMRIw\n"
            			"EAYDVQQDDAlDQSBTdWIgUUExFDASBgNVBAsMC0FEREMgQ0EgU3ViMQ0wCwYDVQQK\n"
            			"DARBRERDMQswCQYDVQQGEwJBRTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoC\n"
            			"ggIBAPSDbye/rfNCosxfuTXslqF42XRw4kOWuZeTEJsBBVUmsWTiAQqwgP2Qmgew\n"
            			"8qKeAfdp67Ni3ExP+rZoWNClMWliH3clD8Xa1R/yf8bzYhsiME3gquhmsBQqCkf7\n"
            			"N/LTGL63kHZ8GEPS/I8qTw1P5MA8mA3Xvin5LKpOC//LRD3wrrHo19XzL3Oe0lQr\n"
            			"cGs6gqa1kLQQvJdaK1LjjDsxi6PIWsm/v3W1911p2HsBqQKQ3RXOMEWBhSdZApEY\n"
            			"SFY3gvj68nWt145fVYgTp3c4ZBhFCBzxwe17nQcmqsTNO1TxBlecXKQ3dR4VR9dp\n"
            			"kWb4wvJ0/d8oTTVNFEuZCY8KH5eQu/7LMG2iCj/ar9rtwQNq2V0OO0RjkQ1oPz5R\n"
            			"6tkZPuXl9DoUisdJdiD3kO0XtQ5fLsTaMkSdg5FhBM/TpdDR70NbTUOAUBDK36y3\n"
            			"cQ7cWVG2ztCczj202FiaONfu7pndCIu0lHt+sbUhgALlOBL1mU2V9Pmgl04Hw6JF\n"
            			"r7fXfwsHj3xIl0UZnrXSOzveyiKfAqEaKEVndAqY/1KBZODHmt6uOY+bcSwGqUfS\n"
            			"RbQYoZwcHMGrUN/kg0Hs6C8WxqoHihiR9FagOvjaJNbJ6/jsleUUF9v2OYGpwyLv\n"
            			"j20oi5qCDbUEjRd1Np8bN2TG9OZlhnze3uxJ4GCV+kjAxFzrAgMBAAGjYzBhMA8G\n"
            			"A1UdEwEB/wQFMAMBAf8wHwYDVR0jBBgwFoAUYvnOEIpLUiuWLFjR0E35cn9lAcww\n"
            			"HQYDVR0OBBYEFJQF7e2fTto3q2As5aRXoTxexw1KMA4GA1UdDwEB/wQEAwIBhjAN\n"
            			"BgkqhkiG9w0BAQsFAAOCAgEAI8spJVrGE1KnGNtw8+eq/mdsPN2+SMul7z4VpAG7\n"
            			"N83YZToSRSLhQowa+V50O6L2lIPB/ZXK+0V/cMbt4DEj6dYLf4x/IWf//qAqncYl\n"
            			"P2H3UYgkW8dC3IEF5P37AD+cVb9acdYX6kmgAKucBKjw1nv2feo2IRC5qbFpnVKe\n"
            			"xr3UKint/s3HacY2cvk8qAXGWgugZ/OyQGACsUbETftz7ko3WMzfn12D7Vb9qejz\n"
            			"EEmBg5NhsLNpK1FSqiDV/8Zo2cnaC4xlh5DE0WXqB7U1Y3qqNmKiU+Ao6jWpI1Wk\n"
            			"Xtx/ZnTwFk+UhVAoEBpg/pC7Qz3T9HndTcemj+eY48xcqfSVsdjJf0qcpItuQKT/\n"
            			"IZ9eIuWdIwkN300coDrNEc0d9xu1U43SnE8fi0Ngtdg+4tvrCvDcvNzIFVf6Oc72\n"
            			"PS8R92XnMDG3ej8Cjio3+fJHh4k6Qd8i1LQJPWFk+gVyjOH4/upMBtdlmByiJf4f\n"
            			"6OE6QkzMyKTWB/ZPXZvysFEeVVeBih2hyO2rgB1yrN0rZ7Go8uXdQa9vth9JSG9z\n"
            			"QqZpaK+xu41z7/2hjZk66MuG0XdU7doIDXRWtQ5IO5G64UjQNRpvKQ4y9IV9lfL2\n"
            			"+iC36nhioagOgGsfgU2HLG+tUhTCB6h9BjO1g6/NHO5ULl1FvysyjkVwlgnvE1Iq\n"
            			"gv4=\n"
            			"-----END CERTIFICATE-----\n%c", CTRL_Z);
#endif
#if 0
//SubRoot SCEP
            	sprintf(AUX_str,
            			"-----BEGIN CERTIFICATE-----\n"
            			"MIIEkzCCAnugAwIBAgIINS/ZQNSTg6EwDQYJKoZIhvcNAQELBQAwRjESMBAGA1UE\n"
            			"AwwJQ0EgU3ViIFFBMRQwEgYDVQQLDAtBRERDIENBIFN1YjENMAsGA1UECgwEQURE\n"
            			"QzELMAkGA1UEBhMCQUUwHhcNMjMxMTA2MDkwNzAwWhcNMjYxMTA1MDkwNzAwWjBM\n"
            			"MRgwFgYDVQQDDA9zY2VwLnFhLmFkZGMuZWExFDASBgNVBAsMC0FEREMgU1VCIFFB\n"
            			"MQ0wCwYDVQQKDARBRERDMQswCQYDVQQGEwJBRTCCASIwDQYJKoZIhvcNAQEBBQAD\n"
            			"ggEPADCCAQoCggEBAMHWdxxNhXqJVswE2dUuf+erHxRFAXd7Vnemnkzk8aNpdE3a\n"
            			"kMUBZiLWnEvmhkLBIMUn+RhM09+tBieTBwKkP1dmW3W560pVffopPjpmSosArwEN\n"
            			"lPcwQnqVlHmqTWN/2brp2tCX9zKxDJIm2yLDBPmu12YvqVF6th4AULBZMslSFw3F\n"
            			"xFF1ijpwzkv1aSj5ITemygo2Z7L/+gui64MRxsCkkgxEuw12Pxlt43yPjSK8WxnD\n"
            			"KxTUiF4mmijI259jV5TUvF2v3QcbPaMd0cRleF7YlSdWSI58ivA+He7ef1J0CEz9\n"
            			"Mhau1qTRxZgCWNWwWig/u5D5oYmJrv10RJqoCO8CAwEAAaN/MH0wDAYDVR0TAQH/\n"
            			"BAIwADAfBgNVHSMEGDAWgBSUBe3tn07aN6tgLOWkV6E8XscNSjAdBgNVHSUEFjAU\n"
            			"BggrBgEFBQcDAgYIKwYBBQUHAwQwHQYDVR0OBBYEFDK1c0VBwalBAONRGgJ8cayS\n"
            			"LurbMA4GA1UdDwEB/wQEAwIFoDANBgkqhkiG9w0BAQsFAAOCAgEAjbBjt/4UWKIo\n"
            			"e1+v2JVkbaRecdNa2Wl6mnVtEvWanRLi/5l4/vOr1eH4kwJZF5EhtIosEj+vWlMc\n"
            			"vfyQLN1pWtmK6XvzJNGjyquQLszVy4BlUiapjyCX2HUm0Fsnme1f3lTs8ud5k9ZA\n"
            			"jM1TTfDdJtz5a3QGqe50rucvNGB/fTQwqPdn6POeuYR8b5VzpoqN57Z0IiIj4R3q\n"
            			"mvyTLQREjYMoIqa2YIpfUik8nLYlhu6hJn3ANHMyp8pSJSA8qcd/SiYINvtl7qIS\n"
            			"5qpp1MV6ItQBMbPp1y7AKxiT9h5N6YL6fRbMn3o3aXhEmyo0IwU2/c62mmjLGHUC\n"
            			"/BQcqBO1DAsPK1Ubz5NhZjS1RLLEi6pZ7FmSGyhwLJLW9TofjNvC+wOABcM3RgOW\n"
            			"8ULVvRj8hkUl0NNmhbtLU0CE3BPddGg/jXIUq2rbvjhQzlnsQfgC6B7BaSpmNLYD\n"
            			"zmmAps0cO6umoRk4Fc1+b4nLIG2hiy4wTFFFOGjRW2XwNDVOb0aWPvxCfR3oUHg5\n"
            			"jm05eSa6g8sjlFj2+P80Lb19DnwYmT50EXt2/qmcKTEvwrglhctHzMIUs4iy4uUY\n"
            			"kFmwNJKE41Dpncl0QHSXQ4x9ZlJcPTTquDOroHsb/sU3VF2E/PIc4lbNxXpJkldn\n"
            			"taEbWjjg3HV1hIyGLvp+RvmODZZQXhk=\n"
            			"-----END CERTIFICATE-----\n%c", CTRL_Z);
#endif
//            	Telit_write_data(AUX_str);
            	comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
//                __updateStatus(M_SENDSSLSECDATA, 10);
#endif
            	__updateStatus(M_SSLSECDATAKEY_CMD, 10);//__updateStatus(M_SENDSSLSECDATA, 10);//
            }
        break;

        case M_SSLSECDATAKEY_CMD:
        	if (CHECK_ELAPSED_MILISEC(2 * 10, TICK_TELIT))
        	{
        		if (ME910_StringEqual("\r\nOK\r\n", 6))
        		{
        			comm_serial_trx(&huart1, (unsigned char *) "AT#SSLSECDATA=3,1,2,1704\r", 25);
        			__updateStatus(M_SSLSECDATAKEY, 10);
        		}
        	}
        	break;

        case M_SSLSECDATAKEY://gateway1.key
        	if (ME910_StringEqual(">", 1))
        	{
#ifdef HTTPS
        		char AUX_str[1800];
#define CTRL_Z 26
        		comm_serial_delete(comm_serial_rcx_bytes_n());
        		sprintf(AUX_str,
        				"-----BEGIN PRIVATE KEY-----\n"
        				"MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCtG1qXo+xwToCP\n"
        				"+euc5RMoWpa/f6dYMUXg05AY90wx2vU5vEHatuiq+oCim0edAvwscJDqsdXRLpfE\n"
        				"uImRtWbLEzSatX7kIXMyhPYJJ8qFccQ+La84DbJHFmhwcS4Q0lDwpX6o6mhkuAKP\n"
        				"HMxeh7DSEW9F3hSNAFzZkJiKpNrEmmmwd2tyC2EY+RMWe7VojW7H+VTw5sVmQtFw\n"
        				"Ireh8OrOI4Ct0op58htt6mPXpUDYWpBA9pSQRjj0/Sd0xhycPOXt2h1IKvC7vdVM\n"
        				"5yrcuEXuP/CIaCWPtGKRXtyVEJBBwMm1rZLvN1rH8kb7p3LKCJ7mGhajHRq4AVDh\n"
        				"kSdKkVV1AgMBAAECggEAJkalGmCRmCeTf8JYGaIBQWv+Zyt3uiVy+Qpuk0ajH2b1\n"
        				"0CVTPmQxQBURumKxTANr9PuS34Ig8BboUgQnzwMYTpDZkdfNze2jhikmKdIVkroc\n"
        				"FU+OrctBfzxyLWKpN+j5IqzFO95Q5OxXDIPmoIRKUbivLPi0/JR281hokkdvZAcP\n"
        				"H6ItgHl5kekrL3pORnd4z0xxf5t+K7MAY84lG1rFl1WE8yn3wNfK8BYW21WnxrI9\n"
        				"FKGB9qyeepQFHO9VeFkk1sMYohKNrvH0YzN58S1iQFQBY30Z+IVIBeBeWfVUpor0\n"
        				"OHUoVPqOQoPPODmh/jztY60j6sz7Y7oG+31CFt6nBQKBgQDmBG26c0u4KDWP8+O8\n"
        				"eQKAdJSlO+bjOb0h5zn3NkLrhtULVhxNrN1Utvdn0FXnMNeMlMyer8Uvm8m6LoXF\n"
        				"afDCXYwHCfGMX7SsDn7b8vU06noAEGdorUjqvIA2hHRZPmWUWUdiFDYzBnaFm/Td\n"
        				"mpwwpjLZTHRnmbr089JE5sUvwwKBgQDAqTS5RN2xCtx5CU4CNuaT968VAPbOmLBd\n"
        				"kTz3uXdAMyE6YO9fCppIeXz5xwgQvW9orBCv+AZpxEjBXbQ9sTM58sb4IxFq+brp\n"
        				"ctfL9c31enMkpVNWKZwg8lj7kC7ZyogDkgtohOJS3CPxEmAhi4S+8BiSFYIPe2K8\n"
        				"bf9fiNSKZwKBgBxiGYQza/mKhKmBx+1e5U8ik6EcFi6d1lfTp5R4zqMvnSRr+Mwg\n"
        				"86LbfQYDcEY6Aa8oQWOnfuaUC718ticZHottbbUEphjTSoEcvMy5W0avVGSjoY8K\n"
        				"KwBESHHxwEWu7Gv05FkW82Uye/eaugmKZZGfQJEN3SNXlflaCO1VqKlvAoGBAKD+\n"
        				"hIVxS8Q/LD+ki+CD2ii56D+n/5Zit7ZowMbN4B3w3Ap5qG4GYSjBBLOBI3dBpdCV\n"
        				"QJibrVhC4v1a/Vw0MHdzLdt7CnoXV57vDe9tZ3+DpKZx1PMCcGqDueH1+YCbow9A\n"
        				"wnHD4ZCWTx6LWXOOsA4SeqXQfU/MrpSBYs6Fe/5vAoGATbg8hj4415AErp5mXP+6\n"
        				"to5NQm6cPOqdhUa9ymfj2V2C2T0kzG2okjGM+S109I4RJIzV51TOHiGMAyZyXkdt\n"
        				"jSWb0NF/iSow4gf23mx/9hoznFt3QjvLm/2UgHZVQv3FvH6HnfhuoTAO9yyeiJpk\n"
        				"KSD7rkYWoYHCisOuPYxOKG4=\n"
        				"-----END PRIVATE KEY-----\n%c", CTRL_Z);
//            	Telit_write_data(AUX_str);
        		comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
#endif
        		__updateStatus(M_SSLSECDATA_CMD, 10);
        	}
        	break;

        case M_SSLSECDATA_CMD:
        	if (CHECK_ELAPSED_MILISEC(2 * 10, TICK_TELIT))
        	{
        		if (ME910_StringEqual("\r\nOK\r\n", 6))
        		{
        			comm_serial_trx(&huart1, (unsigned char *) "AT#SSLSECDATA=1,1,0,1574\r", 25);
        			__updateStatus(M_SSLSECDATA, 10);
        		}
        	}
        	break;

        case M_SSLSECDATA://gateway1_qa_ae.cer
        	if (ME910_StringEqual(">", 1))
        	{
#ifdef HTTPS
        		char AUX_str[1670];
#define CTRL_Z 26
        		comm_serial_delete(comm_serial_rcx_bytes_n());
        		sprintf(AUX_str,
        				"-----BEGIN CERTIFICATE-----\n"
        				"MIIEXjCCAkagAwIBAgIIfr4N4JNXU4AwDQYJKoZIhvcNAQELBQAwRjESMBAGA1UE\n"
        				"AwwJQ0EgU3ViIFFBMRQwEgYDVQQLDAtBRERDIENBIFN1YjENMAsGA1UECgwEQURE\n"
        				"QzELMAkGA1UEBhMCQUUwHhcNMjMxMTIyMTEzNjE2WhcNMjYxMTIxMTEzNjE2WjAX\n"
        				"MRUwEwYDVQQDDAwxMC42NC4xOS4xNjMwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAw\n"
        				"ggEKAoIBAQCtG1qXo+xwToCP+euc5RMoWpa/f6dYMUXg05AY90wx2vU5vEHatuiq\n"
        				"+oCim0edAvwscJDqsdXRLpfEuImRtWbLEzSatX7kIXMyhPYJJ8qFccQ+La84DbJH\n"
        				"FmhwcS4Q0lDwpX6o6mhkuAKPHMxeh7DSEW9F3hSNAFzZkJiKpNrEmmmwd2tyC2EY\n"
        				"+RMWe7VojW7H+VTw5sVmQtFwIreh8OrOI4Ct0op58htt6mPXpUDYWpBA9pSQRjj0\n"
        				"/Sd0xhycPOXt2h1IKvC7vdVM5yrcuEXuP/CIaCWPtGKRXtyVEJBBwMm1rZLvN1rH\n"
        				"8kb7p3LKCJ7mGhajHRq4AVDhkSdKkVV1AgMBAAGjfzB9MAwGA1UdEwEB/wQCMAAw\n"
        				"HwYDVR0jBBgwFoAUlAXt7Z9O2jerYCzlpFehPF7HDUowHQYDVR0lBBYwFAYIKwYB\n"
        				"BQUHAwIGCCsGAQUFBwMBMB0GA1UdDgQWBBT8RhGLqZYFvYew6g3OzDPQzOGC8jAO\n"
        				"BgNVHQ8BAf8EBAMCBaAwDQYJKoZIhvcNAQELBQADggIBAIBsk1+5uFgb93y0bQIC\n"
        				"QwMnEnequXFrcvrTXF62kUlQqqADonxz4uXTUAjSyuAY6LkuOi6Ba6+RE3Y/2JII\n"
        				"JksmRI4uOlPmImpCfy9hkLh7oFUwVPJi+1eWlcSMMNVJLvvvxjy7pnFOwGH7dREP\n"
        				"u27kn7+Xi2+XpmnHalbJgNwNXuJl/ydkFTWAYcVc/Q3kunCzk8WL35ibugOA8hPK\n"
        				"m8hH/FRSlNRdlmnELFOCrmA2EuMO53ezxxz4WMi9NjTWqiqj5wvM8HWCKj0M5epa\n"
        				"CYS0xJ3zUeHnXeh6BP3E9z5dJwDxUq4IWXitZPEH1wCMn3rQCQb3MJBtN0G2XcWL\n"
        				"eH8pjLrOlBmsjMUgCqm79umxZd+S5Ftk0RhjfdETigb8WDj7EOpXkSD/xrDUOk/j\n"
        				"HgBBTaEM/jzstCwqcnQNE+AYfBFTHPSpLdTGLRQVxBTqkke346B0SheBqatfUyvF\n"
        				"Q/L+a5D8FWcCmHcnkYS2s8zom2+/+4giwNyaMLjQV92vc39gwYpQ/YNA5LzlSqH5\n"
        				"En2Qoiqgkm5qpIkb45UJwsgV7gG+DvnYjzEfPzyoSdFKOJg18onHlx3u7H+8TjBY\n"
        				"LbLaNvqeF9RSxdwUZ196MRnGuZ4HI4A7f/EPyYrpcfrk0QhFAJriUW0rZ6sbCq/F\n"
        				"xTxEcD3WE29d3eZMyhCUBwMy\n"
        				"-----END CERTIFICATE-----\n%c", CTRL_Z);
//            	Telit_write_data(AUX_str);
        		comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
#endif
        		__updateStatus(M_SENDSSLSECDATA, 10);
        	}
        	break;

        case M_SENDSSLSECDATA:
        	if (CHECK_ELAPSED_MILISEC(2 * 10, TICK_TELIT))
        	{
        		if (ME910_StringEqual("\r\nOK\r\n", 6))
        		{
        			__processOKCommand(CCLKMODE, NULL, 20, M_CCLKMODE);
        		}
        	}
        break;

        case M_CCLKMODE:
        	if (CHECK_ELAPSED_MILISEC(2 * 10, TICK_TELIT))
        	{
        		if (ME910_StringEqual("\r\nOK\r\n", 6))
        		{
        			comm_serial_delete(comm_serial_rcx_bytes_n());
        			__write_cmd(CCLK, NULL);
        			__updateStatus(M_CLOCK, 20);
        		}
        	}
        break;

        case M_CLOCK:
        	if (CHECK_ELAPSED_MILISEC(50 * 10, TICK_TELIT))
        	{
        		if (ME910_StringEqual("\r\nOK\r\n", 6))
        		{
        			if (ME910_StringEqual("+CCLK:", 6))
        			{
        				char AUX_str[140];
        				//+CCLK: \"19/04/01,11:58:24+08\"\r\n\r\nOK\r\n
        				leds_set_NET_status( NB_Y );
        				s2 = s = (char *) ME910_data;
        				s  = strstr(s2, "\"");
        				s  = s + 1;
        				telit_time = __getDate(s);
        				/* Time was not initiated */
                        if ((0 == Tick_cloud_time_init()) || (1 == rtc_system_getGetTime()))
                        {
                        	if ((telit_time > 1573670014) && (telit_time < 1973880000))
                        	{
                        		Tick_system_time_init(telit_time);
                        		rtc_system_SetServerTime(Tick_Get(SECONDS));
                        	}
//                        	if ( 1 == rtc_system_getGetTime() ) {
                        		rtc_system_setGetTime(0);
                        		rtc_refresh_time = 1;
//                        	}
                        }
                        /* Time was initiated before */
                        else
                        {
                        	if ((telit_time > 1573670014) && (telit_time < 1973880000))
                        	{
                        		Tick_system_time_init(telit_time);
                        		rtc_system_SetServerTime(Tick_Get(SECONDS));
                        	}
                        	else
                        	{
                        		Tick_system_time_init(Tick_Get(SECONDS));
                        		rtc_system_SetServerTime(Tick_Get(SECONDS));
                        	}
//                        	if ( get_server_time++ > 4 ) {
//                        		get_server_time  = 0;
                        		rtc_refresh_time = 1;
//                        	}
                        }

//                      shutdown_set_wchdg_time(params_get_timeout_connection() - HAL_GetTick() / 1000);
                        uint32_t timeout_conn = params_get_timeout_connection() - Tick_Get(MILLISECONDS)/1000;//HAL_GetTick()/1000;
                        if ( Tick_Get(MILLISECONDS)/1000 <  params_get_timeout_connection() ) {
                        	shutdown_set_wchdg_time(timeout_conn);
                        } else {
                        	shutdown_set_wchdg_time(params_get_timeout_connection() - 10);//In case of negative timeout_conn value, we consider 10 seconds lapsed.
                        }
        				comm_serial_delete(comm_serial_rcx_bytes_n());

        				if (PROTOCOL_UDP == params_protocol_mode())
        				{
        					sprintf(AUX_str, "AT#SD=1,1,%d,\"%s\",0,1234,0\r", (int) param.server[ME910_server_get_index()].port, param.server[ME910_server_get_index()].name);
    					    LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Opening UDP socket in: %s:%i\r\n", (int)Tick_Get( SECONDS ),
    					    		param.server[ME910_server_get_index()].name, (int) param.server[ME910_server_get_index()].port);
        				}
        				else if (PROTOCOL_TCP == params_protocol_mode())
        				{
        					if ( 0 == mqtt_timer_get_idle_mode() )
        					{
#ifndef HTTPS
        						sprintf(AUX_str, "AT#SD=1,0,%d,\"%s\"\r", (int)1883, param.server[ME910_server_get_index()].name);
        						LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Opening TCP socket in: %s:%i\r\n", (int)Tick_Get( SECONDS ),
										param.server[ME910_server_get_index()].name, (int)1883);
#else
        						sprintf(AUX_str, "AT#SSLD=1,8883,\"%s\",0,0,1200\r", param.server[ME910_server_get_index()].name);
        						LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Opening TCP socket in: %s:%i\r\n", (int)Tick_Get( SECONDS ),
        							    param.server[ME910_server_get_index()].name, (int)8883);
#endif
        					}
        					else
        					{
#ifndef HTTPS
        						param.server[ME910_server_get_index()].port = 80;
        						sprintf(AUX_str, "AT#SD=1,0,%d,\"%s\"\r", (int) param.server[ME910_server_get_index()].port, param.server[ME910_server_get_index()].name);
#else
           						param.server[ME910_server_get_index()].port = 443;
            					sprintf(AUX_str, "AT#SSLD=1,443,\"%s\",0,0,1200\r", param.server[ME910_server_get_index()].name);
#endif
        						LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> Opening TCP socket in: %s:%i\r\n", (int)Tick_Get( SECONDS ),
										param.server[ME910_server_get_index()].name, (int) param.server[ME910_server_get_index()].port);
        					}
        				}

        				comm_serial_trx(&huart1, (unsigned char *) AUX_str, strlen(AUX_str));
#ifdef HTTPS
        				__updateStatus(M_SD, 50);
#else
        				__updateStatus(M_SD, /*35*/10/*20*/);
#endif
        			}
        		}
        	}
        break;

        case M_CALA:
         	if (CHECK_ELAPSED_MILISEC(2 * 10, TICK_TELIT))
         	{
         		if ((ME910_StringEqual("\r\nOK\r\n", 6)) || (ME910_StringEqual("CME ERROR:", 10)))
         		{
         			char AUX_str[120];
    				comm_serial_delete(comm_serial_rcx_bytes_n());
    				if (PROTOCOL_UDP == params_protocol_mode())
    				{
    					sprintf(AUX_str, "AT#SD=1,1,%d,\"%s\",0,1234,0\r", (int) param.server[ME910_server_get_index()].port, param.server[ME910_server_get_index()].name);
    				}
    				else if (PROTOCOL_TCP == params_protocol_mode())
    				{
    					sprintf(AUX_str, "AT#SD=1,0,%d,\"%s\"\r", (int) param.server[ME910_server_get_index()].port, param.server[ME910_server_get_index()].name);
    				}

    				comm_serial_trx(&huart1, ( unsigned char * )AUX_str, strlen(AUX_str));
					__updateStatus( M_SD, /*35*/10/*20*/ );
         		}
         	}
        break;

        case M_SD:
        	if ((ME910_StringEqual("\r\nOK\r\n", 6)) || (ME910_StringEqual("CONNECT",7)))
        	{
            	comm_serial_delete(comm_serial_rcx_bytes_n());
            	Telit_socket_set_available(CONNECTED);
            	connect_time_end = HAL_GetTick();
			    LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> CONNECTED!\r\n", (int)Tick_Get( SECONDS ));
//            	int con_time = ME910_get_connection_time();
            	sd_try = 0;

				if (((ALARM_SEND == rtc_system_getCurrentAlarmFromBKUP() || ((1 == params_config_get_period()) /*&& (1 == shutdown_get_meter_send())*/)))
#if defined(UNE82326)
				  && (0 == une82326_get_start_comm())
				  && (1 == Tick_cloud_time_init())
#elif defined(MBUS)
				  && (0 == mbus_get_start_comm())
				  && (1 == Tick_cloud_time_init())
#endif
//				  && ((MODBUS_SESSION_END == modbus_get_end_session())
//					|| (MODBUS_SESSION_SEND_TELEGRAM == modbus_get_end_session()))
				  )
				{
					udp_protocol_set_send_pending(1);
				}
				ME910_set_error(ERROR_CONNECTED);
//                shutdown_set_wchdg_time(params_get_timeout_connection());
                status++;
                Tick_update_tick(TICK_TELIT);
            }
        	else if ((CHECK_ELAPSED_TIME(time_to_wait_cmd, TICK_TELIT))
        		  || (ME910_StringEqual("CME ERROR: context not opened\r\n", 31))
				  || (ME910_StringEqual("CME ERROR:", 10))
            	  || (ME910_StringEqual("\r\nERROR\r\n", 9)))
        	{
        		comm_serial_delete(comm_serial_rcx_bytes_n());

        		/* there is an alternative server */
        		if (ME910_server_get_new_index() > 1)
        		{
                	ME910_set_error(ERROR_TRYING_TO_CONNECT);
    				params_attach_error_inc_TSO_error();
    				params_attach_insert_log(TSO_ERR);
                    __updateStatus(M_DELAY, 10);
    			    LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> ERROR TSO! Time out in opening socket.\r\n", (int)Tick_Get( SECONDS ));
        		}
        		/* no alternative server */
        		else
        		{
        			/* number of attempts bigger than selected */
        			if (sd_try++ > params_get_retries_socket())
        			{
        				ME910_set_error(ERROR_TRYING_TO_CONNECT);
        				params_attach_error_inc_TSO_error();
        				params_attach_insert_log(TSO_ERR);
        				shutdown_set(1, params_config_read_time());
        			    LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> ERROR TSO! Time out in opening socket.\r\n", (int)Tick_Get( SECONDS ));
        				sd_try = 0;
        			}
        			else
        			{
        				params_attach_insert_log(IP_ERR + sd_try); // TOASK +sd_try??
        				ME910_set_error(ERROR_TRYING_TO_CONNECT);
           			    LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> ERROR TSO! Time out in opening socket.\r\n", (int)Tick_Get( SECONDS ));
        				__updateStatus(M_DELAY, 2/*10*/);
        			}
        		}
            }
        break;

        case M_CONNECTED: // Pasamos el control del puerto serie.
//        	Tick_update_tick( TICK_TELIT );
            if (Telit_is_socket_available())
            {
            	Tick_update_tick(TICK_TELIT);
            	if (0 == rest_upgrading_firmware())
            	{
            		if (comm_serial_get_NO_CARRIER())
            		{
            			if (Telit_in_PSM())
            			{
            				Telit_socket_enters_PSM();
            			}
            			else
            			{
//            				Telit_socket_close_and_shutdown();
            				Telit_socket_reconnect();
            			}
            			LOGLIVE(LEVEL_2, "\r\nLOGLIVE> %d ME910> NO CARRIER!!!!!!!.\r\n", (int)Tick_Get( SECONDS ));
            			comm_serial_delete(comm_serial_rcx_bytes_n());
            			comm_serial_rx_vars_init();
            			ME910_set_error(ERROR_TRYING_TO_CONNECT);
            			udp_protocol_reset_vars();
            		}
            	}
            }
            /* socket closed */
            else
            {
            	if ((SHUTDOWN == sME910.shutdownStatus) || (ENTERS_PSM == sME910.shutdownStatus))
            	{
//            		if( comm_serial_get_NO_CARRIER() ) {
						comm_serial_delete( comm_serial_rcx_bytes_n());
						comm_serial_trx(&huart1,  (unsigned char *) "+++", 3);
//						shutdown_set_wchdg_time(params_get_timeout_connection());
						__updateStatus(M_WAITING_FOR_SHUTDOWN, 5/*20*/);
//            		}
            	}
            	else if (QUICK_SHUTDOWN == sME910.shutdownStatus)
            	{
					comm_serial_delete(comm_serial_rcx_bytes_n());
					if (comm_serial_trx(&huart1, (unsigned char *) "+++", 3) != 0)
					{
						LOGLIVE(LEVEL_1, "LOGLIVE> ME910> RECOVERING!!\r\n");
						comm_serial_enable();
						comm_serial_trx(&huart1, (unsigned char *) "+++", 3);
					}	   													   
					__updateStatus(M_WAITING_FOR_SHUTDOWN, 5/*20*/);
            	}
            	else if (RECONNECT == sME910.shutdownStatus)
            	{
            		if ( ( comm_serial_get_NO_CARRIER() )
            		  || ( ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n()) )
					  || ( CHECK_ELAPSED_TIME( time_to_wait_cmd, TICK_TELIT )) )
            		{
            			if (GPIO_PIN_RESET == ME910_CHECK_PSM)
            			{
            				comm_serial_delete(comm_serial_rcx_bytes_n());
#ifndef HTTPS
            				comm_serial_trx(&huart1, (unsigned char *) "AT#SH=1\r", 8);
#else
            				comm_serial_trx(&huart1, (unsigned char *) "AT#SSLH=1\r", 10);
#endif
            				__updateStatus(M_RECONNECT, 25);
            			}
            			else
            			{
            				if (1 == Telit_wake_up_from_PSM())
            				{
            					comm_serial_delete(comm_serial_rcx_bytes_n());
#ifndef HTTPS
            					comm_serial_trx(&huart1, (unsigned char *) "AT#SH=1\r", 8);
#else
            					comm_serial_trx(&huart1, (unsigned char *) "AT#SSLH=1\r", 10);
#endif
            					__updateStatus(M_RECONNECT, 25);
            				}
            			}
            		}
            	}
            	else if (HTTP_CONNECTION == sME910.shutdownStatus)
            	{
            		if ((comm_serial_get_NO_CARRIER()) || (ME910_StringEnds("\r\nOK\r\n", 6, comm_serial_rcx_bytes_n())))
            		{
            			if ( GPIO_PIN_RESET == ME910_CHECK_PSM ) {
            				comm_serial_delete( comm_serial_rcx_bytes_n() );
#ifndef HTTPS
            				comm_serial_trx(&huart1, (unsigned char *) "AT#SH=1\r", 8);
#else
            				comm_serial_trx(&huart1, (unsigned char *) "AT#SSLH=1\r", 10);
#endif
            				__updateStatus( M_RECONNECT_HTTP, 5 );

            			} else {
            				if ( 1 == Telit_wake_up_from_PSM() ) {
            					comm_serial_delete( comm_serial_rcx_bytes_n() );
#ifndef HTTPS
            					comm_serial_trx(&huart1, (unsigned char *) "AT#SH=1\r", 8);
#else
            					comm_serial_trx(&huart1, (unsigned char *) "AT#SSLH=1\r", 10);
#endif
            					__updateStatus( M_RECONNECT_HTTP, 5 );
            				}
            			}
            		}
            	}
            	else
            	{
            		time_to_wait_cmd = 2;
            		status           = M_DELAY;
            	}
            }
        break;

        case M_WAITING_FOR_SHUTDOWN:
        	if ( ( comm_serial_get_NO_CARRIER() )
        	  || ( ME910_StringEnds( "\r\nOK\r\n", 6, comm_serial_rcx_bytes_n() )
        	  || ( CHECK_ELAPSED_TIME( time_to_wait_cmd, TICK_TELIT ) )
			     )
        	   ) {
        		LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> AT#SH=1 %s \r\n", (int)Tick_Get( SECONDS ), ME910_data);
        		ME910_EraseBuffer();
        		if ( 1 == shutdown_get_tamper() ) {
        			comm_serial_trx( &huart1, ( unsigned char *) "AT#GPIO=2,1,1\r", 14 );
        			HAL_Delay(10);
        		}
				comm_serial_delete( comm_serial_rcx_bytes_n() );
#ifndef HTTPS
				if (comm_serial_trx(&huart1, ( unsigned char *) "AT#SH=1\r", 8 ) != 0)
				{
					comm_serial_enable();
					comm_serial_trx(&huart1, ( unsigned char *) "AT#SH=1\r", 8 );
				}
#else
				if (comm_serial_trx(&huart1, ( unsigned char *) "AT#SSHL=1\r", 10 ) != 0)
				{
					comm_serial_enable();
					comm_serial_trx(&huart1, ( unsigned char *) "AT#SSHL=1\r", 10 );
				}
#endif
				leds_set_NET_status( LOW_POWER );
				__updateStatus( M_SHUTDOWN, 20 );
				LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> M_SHUTDOWN AT#SH=1.\r\n", (int)Tick_Get( SECONDS ));
        	}
        break;
        case M_SHUTDOWN:
        	if ( CHECK_ELAPSED_MILISEC( time_to_wait_cmd * 100, TICK_TELIT ) ) {
//        	if ( ME910_StringEnds( "\r\nOK\r\n", 6, comm_serial_rcx_bytes_n() ) ) {
        		if ( 1 == shutdown_get_tamper() ) {
        			comm_serial_trx( &huart1, ( unsigned char *) "AT#GPIO=2,1,1\r", 14 );
        			HAL_Delay(10);
        		}
        		LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> SHUTDOWN.\r\n", (int)Tick_Get( SECONDS ));
        		comm_serial_delete( comm_serial_rcx_bytes_n() );
#ifndef MODBUS
				if( ME910_StringEqual("\r\nOK\r\n", 6) ) {
					shutdown_set(1, params_config_read_time());
					comm_serial_delete( comm_serial_rcx_bytes_n() );
					Tick_update_tick( TICK_TELIT );
					time_to_wait_cmd = 10;
					status           = M_WAIT_EXIT_STOP_MODE;
					rtc_system_setBackUpRegister(RTC_BKP_DR11, status);
				}
#else
				if ( ( 0 == params_modbus_log_enabled() )
				  || ( ( 1 == params_modbus_log_enabled() ) && ( 0 == params_modbus_log_prev_enabled() ) && ( 0 == leds_device_switch_on() ) ) ) {
					if ( ( 1 == params_get_mqtt_dc_on() ) && ( 0 == message_queue_get_elements() ) && ( 0 == con_dlms_in_process() ) )
					{
						shutdown_set(1, params_config_read_time());
					}
					else if ( 0 == params_get_mqtt_dc_on() )
					{
						shutdown_set(1, params_config_read_time());
					}
					if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
						__updateStatus( M_WAIT_EXIT_STOP_MODE, 1 );
					} else if ( NBIOT_PWR_ONOFF == params_nbiotpwr_mode() ) {
						if ( 1 == params_get_mqtt_dc_on() )
						{
							leds_set_NET_status(NB_N);
//							ME910_set_error(ERROR_TRYING_TO_CONNECT);
							shutdown_reset_watchdog();
							connect_time_ini = HAL_GetTick();
//							__write_cmd(CEREG, NULL);
							if (HAL_ERROR == __write_cmd(CEREG, NULL))
							{
								LOGLIVE(LEVEL_1, "LOGLIVE> ME910> RECOVERING!!\r\n");
								comm_serial_enable();
								__write_cmd(CEREG, NULL);
							}
							__updateStatus( M_CEREG, 5 );
						}
						else
						{
							__updateStatus( M_BOOT0, 1 );
						}
					}
					if ( 2 == mqtt_timer_get_idle_mode() )
					{
						mqtt_timer_set_idle_mode(1);
					}
				} else {//Local Tool On.
					shutdown_setInitTelitModule(0);
//					shutdown_set_wchdg_time(params_get_timeout_connection());
					shutdown_set_wchdg_time(MODBUS_SESSION_TIME);
					if ( NBIOT_PWR_PSM == params_nbiotpwr_mode() ) {
						status = M_WAIT_EXIT_STOP_MODE;
					} else if ( NBIOT_PWR_ONOFF == params_nbiotpwr_mode() ) {
						status = M_BOOT0;
						Telit_modem_off_prc();
						do {
							asm("nop");
						}while( GPIO_PIN_RESET == ME910_CHECK_PSM );
						Telit_modem_disable();
					}
					LOGLIVE(LEVEL_2, "LOGLIVE> %d ME910> LOCAL TOOL ON.\r\n", (int)Tick_Get( SECONDS ));
					leds_set_NET_status(NB_LOCAL_TOOL);
				}
        	}
#endif
        break;

        case M_DELAY:
            if (CHECK_ELAPSED_TIME(time_to_wait_cmd, TICK_TELIT))
            {
            	comm_serial_delete(comm_serial_rcx_bytes_n());
                __updateStatus(M_BOOT0, 2);
            }
        break;

        case M_RECONNECT:
        	if ( ( ME910_StringEnds( "\nOK\r\n",   5, comm_serial_rcx_bytes_n() ) )
        	  || ( ME910_StringEnds( "\n\r\nOK\r", 6, comm_serial_rcx_bytes_n() ) )
			  || ( CHECK_ELAPSED_TIME( time_to_wait_cmd, TICK_TELIT ) )
        	) {
        		char AUX_str[140];
//        		shutdown_set_wchdg_time(params_get_timeout_connection());
        		if (NBIOT_PWR_PSM == params_nbiotpwr_mode())
        		{
        			ME910_setPSMMode(1);
        		}
        		comm_serial_delete( comm_serial_rcx_bytes_n() );
        		ME910_EraseBuffer();
				if ( PROTOCOL_UDP == params_protocol_mode() ) {
					sprintf(AUX_str, "AT#SD=1,1,%d,\"%s\",0,1234,0\r", (int)param.server[ ME910_server_get_index() ].port, param.server[ ME910_server_get_index() ].name);
//					sprintf(AUX_str, "AT#SD=1,1,9998,\"%s\",0,1234,0\r", param.server[ ME910_server_get_index() ].name);
				} else if ( PROTOCOL_TCP == params_protocol_mode() ){
#ifndef HTTPS
					sprintf(AUX_str, "AT#SD=1,0,%d,\"%s\"\r", (int)param.server[ ME910_server_get_index() ].port, param.server[ ME910_server_get_index() ].name);
//					sprintf(AUX_str, "AT#SD=1,0,80,\"%s\"\r", param.server[ ME910_server_get_index() ].name);
#else
        			param.server[ME910_server_get_index()].port = 443;
        			sprintf(AUX_str, "AT#SSLD=1,443,\"%s\",0,0,1200\r", param.server[ME910_server_get_index()].name);
#endif
				}
        		comm_serial_trx(&huart1, ( unsigned char * )AUX_str, strlen(AUX_str));
#ifndef HTTPS
        		__updateStatus( M_SD, 10 );
#else
        		__updateStatus( M_SD, 35 );
#endif
        	}
        	break;
        case M_WAIT_EXIT_STOP_MODE:
        	if ( CHECK_ELAPSED_TIME( time_to_wait_cmd, TICK_TELIT ) ) {
				if ( NBIOT_PWR_ONOFF == params_nbiotpwr_mode() ) {
					__updateStatus( M_BOOT0, 1 );
				} else if ( NBIOT_PWR_PSM== params_nbiotpwr_mode() ) {
					if (   ( 1 == shutdown_initTelitModule() )
					    && ( 2 == ME910_getPSMMode() )
						) {
						leds_set_NET_status(NB_N);
						ME910_set_error(ERROR_TRYING_TO_CONNECT);
						shutdown_reset_watchdog();
						ME910_setPSMMode(1);
						connect_time_ini = HAL_GetTick();
						if (HAL_ERROR == __write_cmd(CEREG, NULL))
						{
							LOGLIVE(LEVEL_1, "LOGLIVE> ME910> RECOVERING!!\r\n");
							comm_serial_enable();
							__write_cmd(CEREG, NULL);
						}
						__updateStatus( M_CEREG, 5 );
					}
				}
        	}
        	break;
        case M_DISABLE_PSM:
        	if ( CHECK_ELAPSED_MILISEC( time_to_wait_cmd * 100, TICK_TELIT ) ) {
            	comm_serial_delete( comm_serial_rcx_bytes_n() );
            	__write_cmd( CPSMS_DISABLE, NULL );
                __updateStatus( M_DISABLE_PSM_RESP, 10 );
        	}
        	break;
        case M_DISABLE_PSM_RESP:
        	if ( CHECK_ELAPSED_MILISEC( time_to_wait_cmd * 100, TICK_TELIT ) ) {
        		ME910_EraseBuffer();
				comm_serial_delete( comm_serial_rcx_bytes_n() );
				comm_serial_trx( &huart1, ( unsigned char *) "AT#SHDN\r", 8 );
				leds_set_NET_status( LOW_POWER );
				__updateStatus( M_EXIT_WITHOUT_CONNECTION, 20 );
        	}
        	break;
        case M_EXIT_WITHOUT_CONNECTION:
        	if ( CHECK_ELAPSED_MILISEC( time_to_wait_cmd * 100, TICK_TELIT ) ) {
				if( ME910_StringEqual( "OK", 2 ) ) {
					shutdown_set( 1, params_config_read_time() );
					__updateStatus( M_DELAY, 10 );
				}
        	}
        	break;
        case M_RECONNECT_HTTP:
        	if ( ME910_StringEnds( "\r\nOK\r\n", 6, comm_serial_rcx_bytes_n() ) ) {
        		char AUX_str[120];
//        		shutdown_set_wchdg_time(params_get_timeout_connection());
        		comm_serial_delete( comm_serial_rcx_bytes_n() );
        		ME910_EraseBuffer();
//        		sprintf(AUX_str, "AT#SD=1,0,%d,\"%s\"\r", (int)param.server[ ME910_server_get_index() ].port, param.server[ ME910_server_get_index() ].name);
#ifdef ZONOS
        		sprintf(AUX_str, "AT#SD=1,0,443,\"%s\"\r", param.server[ ME910_server_get_index() ].name);
#else
        		sprintf(AUX_str, "AT#SD=1,0,80,\"%s\"\r", param.server[ ME910_server_get_index() ].name);
#endif
        		comm_serial_trx(&huart1, ( unsigned char * )AUX_str, strlen(AUX_str));
        		__updateStatus( M_SD, 10 );
        	}
        	break;
        default:
            time_to_wait_cmd = 5;
            status           = M_DELAY;
            break;
    }
    __checkTimeout(&time_to_wait_cmd);
}


/**
  * @brief  Checks whether the ME910 FSM is stuck in one state after a time given by t.
  * @param	*t pointer to a time variable.
  * @retval None
  */
static void __checkTimeout( uint32_t *t )
{
	uint32_t time = *t + 1;
	if( CHECK_ELAPSED_TIME(time, TICK_TELIT))
	{
    	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910> ERROR in status: %i\r\n", (int)Tick_Get( SECONDS ), (int)status);

    	error_status = status;

    	if (M_PIN == status)
		{
			csq_ok = -1;
		}
    	if (status > M_BOOT3)//if (status != M_BOOT3)
		{
			asm("nop");
		}
		else
		{
			csq_ok = -1;
		}
		Tick_update_tick(TICK_TELIT);
		*t     = 1;
		status = M_DELAY;
		rtc_system_setBackUpRegister(RTC_BKP_DR11, status);
	}
}


static unsigned char __getICCID( void )
{
	char *str;
	// \r\n357129074618501\r\n\r\nOK\r\n
	str  = strstr(ME910_data, "\r\n");
	memcpy( sME910.ICCID, str + 9, 19 );
//	memcpy( param.imei, str + 2, 15 );
	return 1; // faltaria verificar
}

static unsigned char __getIMSI( void )
{
	char *str;
	// \r\n357129074618501\r\n\r\nOK\r\n
	str  = strstr(ME910_data, "\r\n");
	memcpy( sME910.IMSI, str + 2, 15 );
//	memcpy( param.imei, str + 2, 15 );
	return 1; // faltaria verificar
}

static char __getIMEI( void )
{
	char *str;
	// \r\n357129074618501\r\n\r\nOK\r\n
	str  = strstr(ME910_data, "\r\n");
	memcpy( sME910.IMEI, str + 2, 15 );
	memcpy( param.imei, str + 2, 15 );
	return 1; // faltaria verificar
}

static char __getIP(void)
{
	char *str;
	memset(sME910.IP,0,sizeof(sME910.IP));
	// \r\n#SGACT: 10.130.100.52\r\n\r\nOK\r\n
	str = strstr(ME910_data, "\r\n");
	str = strtok( str, ":" );
	str = strtok( NULL, ":" );
	str = strtok( str, "\r" );
	memcpy(sME910.IP,str,strlen(str));
	return 1;
}


////////////////////////////////////////////////////////////////////////////////
static uint8_t __write_cmd( ME910_command cmd, uint8_t *params )
{
	uint8_t *str         = (uint8_t *) ME910_strings[cmd];
	uint32_t data_length = strlen( ME910_strings[cmd] );

	if (params != NULL) {
		uint8_t cmd_len, params_len, i;

		cmd_len    = strlen( ME910_strings[ cmd ]);
		params_len = strlen( (char *) params );

		for( i = params_len; i > 0; i-- ) {
			params[ i + cmd_len - 1 ] = params[ i - 1 ];
		}
		for( i = 0; i < cmd_len; i++ ) {
			params[ i ] = ME910_strings[ cmd ][ i ];
		}
		str = params;
		data_length = cmd_len + params_len;
	}
	if ( HAL_OK == comm_serial_trx(&huart1, str, data_length)) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

/**
 * @fn uint8_t Telit_write_data(char*)
 * @brief Sends a string pointed by *str to the ME910 via UART interface.
 * @param str pointer to the string to send.
 * @return 0
 */
uint8_t Telit_write_data(char * str)
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910-TCP>\r\nTX:%s.\r\n", (int)Tick_Get( SECONDS ), str);
	comm_serial_trx(&huart1, (unsigned char *) str, strlen(str));
	return 0;
}

/**
 * @fn uint8_t Telit_write_data_length(char*, uint32_t)
 * @brief Sends a string of length length pointed by *str to the ME910 via UART interface.
 * @param str pointer to the string to send.
 * @param length length of the string to send.
 * @return 0
 */
uint8_t Telit_write_data_length( char * str, uint32_t length )
{
	LOGLIVE(LEVEL_1, "LOGLIVE> %d ME910>length: %d TX:%s.\r\n", (int)Tick_Get( SECONDS ), (int)length, str);
	comm_serial_trx(&huart1, (uint8_t *) str, length);
	return 0;
}

uint8_t Telit_write_header_data( char * str, char * str_hdr )
{
	comm_serial_trx_header_msg( &huart1, (unsigned char *)str, strlen(str), (unsigned char *)str_hdr, strlen(str_hdr) );

	return 0;
}

int8_t ME910_StringEqual(char * string, size_t lenstring)
{
    uint32_t  n = comm_serial_rcx_bytes_n();
    char *str;
    uint8_t *ret_aux;
    int8_t ret = 0;

    ME910_EraseBuffer();
    str = (char *)ME910_data;
    n   = comm_serial_rcx_n( (uint8_t *)str, n );

    ret_aux = (uint8_t *)memmem( (char *)str, n, string, lenstring );

    if (ret_aux != NULL) {
        ret = 1;
    }
    return ret;
}

uint8_t ME910_StringEnds(char * string, size_t lenstring, uint32_t end_len)
{
    uint32_t  n   = comm_serial_rcx_bytes_n();
    uint8_t *ret_aux;
    int8_t ret 	  = 0;

    if (end_len > (sizeof(ME910_data)/sizeof(ME910_data[0]))) {
    	return 0;
    }
    n = comm_serial_rcx_n( (uint8_t *)ME910_data, end_len );

    ret_aux = (uint8_t *) memmem(  & ME910_data[ end_len - lenstring ], n, string, lenstring );

    if (ret_aux != NULL) {
        ret = 1;
    }
    return ret;
}

uint32_t ME910_getCSQ( void )
{
	sME910.CSQ = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
	return sME910.CSQ;
}

void ME910_setPSMMode( uint8_t mode )
{
	sME910.PSMMode = mode;
}

uint32_t ME910_getPSMMode( void )
{
	return sME910.PSMMode;
}

/**
 * @fn char Telit_getSignalStrengthCondition*(void)
 * @brief
 * @return
 */
char *Telit_getSignalStrengthCondition( void )
{
	uint32_t csq;

	csq = ME910_getCSQ();

	if (csq < 10) {
//		return "MARGINAL";
		return "0";
	}
	else if ( ( csq >= 10 ) && ( csq <= 14 ) ) {
//		return "OK";
		return "1";
	}
	else if ( ( csq >= 15 ) && ( csq <= 19 ) ) {
//		return "GOOD" ;
		return "2";
	}
	else if ( ( csq >= 20 ) && ( csq != 99 ) ) {
//		return "EXCELLENT";
		return "3";
	}
	else {
		return "?";
	}
}
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief
 */
static uint8_t APN_i = 1, APN_curr = 1;;

/**
 * @fn uint8_t ME910_APN_get_new_index(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t ME910_APN_get_new_index(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        APN_i = APN_i ? 0 : 1;
        if (param.APN[APN_i].name[0] != '\0')
        {
        	break;
        }
    }

    return APN_i;
}

/**
 * @fn uint8_t ME910_APN_get_index(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t ME910_APN_get_index( void )
{
    return APN_i;
}

/**
 * @fn uint8_t ME910_APN_get_current(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
uint8_t ME910_APN_get_current( void )
{
    return APN_curr;
}

/**
 * @fn uint8_t ME910_APN_get_current(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
void ME910_APN_set_current( uint8_t _apn )
{
    APN_curr = _apn;
}

/**
 * @fn void ME910_APN_reset_index(void)
 * @brief
 *
 * @pre
 * @post
 */
void ME910_APN_reset_index( void )
{
	APN_i = 1;
}
////////////////////////////////////////////////////////////////////////////////

static uint8_t server_i = 0;

/**
 * @fn uint8_t ME910_server_get_new_index(void)
 * @brief
 * @return
 */
uint8_t ME910_server_get_new_index( void )
{
    for (uint8_t i = 0; i < 2; i++)
    {
        server_i = server_i ? 0 : 1;
        if (param.server[server_i].name[0] != '\0' )
        {
        	break;
        }
    }

    return server_i;
}

/**
 * @fn uint8_t ME910_server_get_index(void)
 * @brief
 * @return
 */
uint8_t ME910_server_get_index( void )
{
    return server_i;
}

/**
 * @fn void ME910_server_reset_index(void)
 * @brief
 */
void ME910_server_reset_index( void )
{
    server_i = 0;
}

////////////////////////////////////////////////////////////////////////////////
//#define NL865_LOG
#ifdef ME910_LOG

#define ME910_LOG_SIZE 500

void ME910_log( unsigned int n )
{
    static unsigned int i = 0;
    static struct
    {
        unsigned int time[ NL865_LOG_SIZE ], value[ NL865_LOG_SIZE ];
    } data;

    data.time[ i ] = Time_Get(SECONDS);
    data.value[ i ] = n;
    i++;

    if( i == 360 )
        i = 0;
}

#endif
////////////////////////////////////////////////////////////////////////////////

void ME910_set_error( sME910_error error )
{
    sME910.error = error;
#ifdef ME910_LOG
    ME910_log(error);
#endif
}

sME910_error ME910_get_error( void )
{
    return sME910.error;
}

////////////////////////////////////////////////////////////////////////////////

char * ME910_ICCID( void )
{
    return sME910.ICCID;
}

char * ME910_IMSI( void )
{
    return sME910.IMSI;
}

char * ME910_IMEI( void )
{
	if (sME910.IMEI[0] != '\0') {
		return sME910.IMEI;
	} else if ( 1 == params_imei_ok() ) {
		return params_get_imei();
	}
	return (char *)"0";
}

/**
 * @fn char Telit_dev_identifier*(void)
 * @brief Builds up the device identifier from the IMEI
 * @return pointer to the dev_identifier string.
 */
char * Telit_dev_identifier( void )
{
	for (uint8_t i = 0; i < 9; i++)
	{
		sME910.dev_identifier[i] = param.imei[i + 5];
	}
    return sME910.dev_identifier;
}

uint8_t Telit_is_socket_available( void )
{
    return sME910.udpSocketAvailable;
}

void Telit_socket_set_available( uint8_t available )
{
	sME910.udpSocketAvailable = available;
}

void Telit_socket_close( void )
{
	sME910.udpSocketAvailable = 0;
}

void Telit_socket_close_and_shutdown( void )
{
	sME910.udpSocketAvailable = 0;
	sME910.shutdownStatus     = SHUTDOWN;
	Tick_update_tick( TICK_TELIT );
}

void Telit_socket_quick_close_and_shutdown( void )
{
	sME910.udpSocketAvailable = NOT_CONNECTED;
	sME910.shutdownStatus     = QUICK_SHUTDOWN;
	Tick_update_tick( TICK_TELIT );
}

void Telit_socket_reconnect( void )
{
	sME910.udpSocketAvailable = NOT_CONNECTED;//sME910.tcpSocketAvailable = NOT_CONNECTED;
	sME910.shutdownStatus     = RECONNECT;
	time_to_wait_cmd          = 5;//20;
	Tick_update_tick( TICK_TELIT );
//	HAL_Delay(1000);
//	comm_serial_trx( &huart3,  ( unsigned char *) "+++", 3 );
}

void Telit_socket_enters_PSM( void )
{
	sME910.udpSocketAvailable = NOT_CONNECTED;
	sME910.shutdownStatus     = ENTERS_PSM;
	Tick_update_tick( TICK_TELIT );
}

void Telit_socket_HTTP_connection( void )
{
	sME910.udpSocketAvailable = NOT_CONNECTED;
	sME910.shutdownStatus     = HTTP_CONNECTION;
	time_to_wait_cmd          = 20;
	Tick_update_tick( TICK_TELIT );
//	HAL_Delay(1000);
//	comm_serial_trx( &huart3,  ( unsigned char *) "+++", 3 );
}

/**
 * @}
 */
