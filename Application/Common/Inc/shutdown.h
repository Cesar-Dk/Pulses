/*
 * shutdown.h
 *
 *  Created on: 16 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_USER_COMMON_INC_SHUTDOWN_H_
#define APPLICATION_USER_COMMON_INC_SHUTDOWN_H_

#include "main.h"
#include "params.h"

#if defined (HTTP)
#define SHUTDOWN_STAND_BY_TIME	    (30)
#define SHUTDOWN_STAND_BY_MAX_TIME	(60)
#elif defined (UDP)
#define SHUTDOWN_POLL_TIME	        (10)
#define SHUTDOWN_DETECT_TIME        (1)
#define SHUTDOWN_STAND_BY_TIME	    (40)//(180)//(30)
#define SHUTDOWN_STAND_BY_MAX_TIME	(180)
#else
#define SHUTDOWN_STAND_BY_TIME	    (30)
#define SHUTDOWN_STAND_BY_MAX_TIME	(60)
#endif

#define TAMPER_TSEND            			  (0)
#define TAMPER_TAMPER_ON        			  (1)
#define TAMPER_HARD_RESET_DAYLY 			  (2)
#define TAMPER_HARD_RESET_MW    			  (3)
#define TAMPER_LOCAL_TOOL       			  (4)
#define TAMPER_SWITCH_ON        			  (5)
#define TAMPER_DATALOGGER_RD    			  (6)
#define TAMPER_BATTERY_VBACKUP  			  (7)
#define TAMPER_WATCHDOG         			  (8)
#define TAMPER_REED_SENSOR_ON   			  (9)
#define TAMPER_LEVEL_ALARM_ON   			  (10)
#define TAMPER_PSM              			  (11)
#define TAMPER_MQTT_ENERGYPROF  			  (12)
#define TAMPER_MQTT_SWITCH_STATUS             (13)
#define TAMPER_MQTT_ON_DEMAND_LOAD_PROFILE    (14)
#define TAMPER_MQTT_ON_DEMAND_BILLING_PROFILE (15)
#define TAMPER_MQTT_METER_SYNCH   			  (16)
#define TAMPER_MQTT_RECONNECT 			  	  (17)
#define TAMPER_MQTT_WATERPROFILE              (18)
#define TAMPER_MQTT_GW_GET_TIME		          (19)
#define TAMPER_MQTT_GW_SET_TIME				  (20)
#define TAMPER_MQTT_GW_GET_INTERVAL			  (21)
#define TAMPER_MQTT_GW_SET_INTERVAL			  (22)
#define TAMPER_MQTT_GW_SET_FW_UPDATE		  (23)
#define TAMPER_MQTT_GW_GET_RTC				  (24)
#define TAMPER_BATTERY_VBACKUP_OFF  		  (70)
#define TAMPER_INPUT_ALARM_ON			  	  (80)
#define TAMPER_INPUT_ALARM_OFF			  	  (85)

#define TAMPER_MQTT_CMD_READ_REAL_TIME_CLOCK             (24)
#define TAMPER_MQTT_CMD_READ_LOAD_PROFILE_CAPTURE_PERIOD (25)
#define TAMPER_MQTT_CMD_SET_LOAD_PROFILE_CAPTURE_PERIOD  (26)
#define TAMPER_MQTT_CMD_MDIEOB_RESET                     (27)
#define TAMPER_MQTT_CMD_SET_LOAD_LIMITATION              (28)
#define TAMPER_MQTT_CMD_GET_LOAD_LIMIT_THRESHOLD         (29)
#define TAMPER_MQTT_CMD_SET_BILLING_DATE				 (30)
#define TAMPER_MQTT_CMD_GET_BILLING_DATE				 (31)
#define TAMPER_MQTT_CMD_CLEAR_ALARMS     				 (32)
#define TAMPER_MQTT_CMD_SET_DEMAND_INTEGRATION_PERIOD	 (33)
#define TAMPER_MQTT_CMD_GET_DEMAND_INTEGRATION_PERIOD	 (34)
#define TAMPER_MQTT_CMD_SET_PAYMENT_MODE	 			 (35)
#define TAMPER_MQTT_CMD_GET_PAYMENT_MODE				 (36)
#define TAMPER_MQTT_CMD_SET_METERING_MODE	 			 (37)
#define TAMPER_MQTT_CMD_GET_METERING_MODE				 (38)
#define TAMPER_MQTT_CMD_SET_CURR_ACTIVE_TARIFF			 (39)
#define TAMPER_MQTT_CMD_GET_CURR_ACTIVE_TARIFF			 (40)
#define TAMPER_MQTT_CMD_SET_VOLTAGE_LOW					 (44)
#define TAMPER_MQTT_CMD_SET_VOLTAGE_UP  				 (45)
#define TAMPER_MQTT_CMD_GET_VOLTAGE_LOW					 (46)
#define TAMPER_MQTT_CMD_GET_VOLTAGE_UP				   	 (47)
#define TAMPER_MQTT_CMD_SET_CURRENT_LOW					 (48)
#define TAMPER_MQTT_CMD_SET_CURRENT_UP  				 (49)
#define TAMPER_MQTT_CMD_GET_CURRENT_LOW					 (50)
#define TAMPER_MQTT_CMD_GET_CURRENT_UP				   	 (51)
#define TAMPER_MQTT_CMD_GET_METER_STATUS				 (52)
#define TAMPER_MQTT_CMD_GET_GW_NAMEPLATE                 (53)
#define TAMPER_MQTT_CMD_GET_GW_PING	                     (54)
#define TAMPER_MQTT_CMD_METER_PING						 (55)
#define TAMPER_MQTT_CMD_READ_METER_NAMEPLATE             (56)
#define TAMPER_MQTT_COMMAND_ERROR 						 (99)

#if 0
TAMPER=19 ONDEMAND GW Read Time and Date: getgwclocktime
TAMPER=20 ONDEMAND GW Set Time Synchronization: gwsynchronization
TAMPER=21 ONDEMAND GW Read Profile Interval/period: getgwinterval
TAMPER=22 ONDEMAND GW Set Profile Interval/period: setgwinterval
TAMPER=23 ONDEMAND GW Gateway Firmware Update: gwfirmwareupdate
TAMPER=24 ONDEMAND M Read Real Time Clock: clocktime

TAMPER=53 ONDEMAND DMM Read Gateway Name plate information: gwnameplate
TAMPER=54 ONDEMAND DMM Gateway Ping: ping

      TAMPER=25 ONDEMAND M Read Load Profile Capture Period: getprofileinterval


      TAMPER=26 ONDEMAND M Set Load Profile Capture Period: setprofileinterval


      TAMPER=27 ONDEMAND M Maximum Demand Reset ->  MDI / End of billing period Reset: mdieobreset


      TAMPER=28 ONDEMAND M Set Load Limitation / Load Curtailment: loadlimit


      TAMPER=29 ONDEMAND M Get Load Limit Threshold: readloadlimit


      TAMPER=30 ONDEMAND M Set Billing Date: setbillingdate


      TAMPER=31 ONDEMAND M Get Billing Date: getbillingdate


      TAMPER=32 ONDEMAND M Clear Alarms: clearalarms


      TAMPER=33 ONDEMAND M Set Demand Integration Period: setdemandInterval


      TAMPER=34 ONDEMAND M Get Demand Integration Period: getdemandInterval


      TAMPER=35 ONDEMAND M Set Payment Mode: setpaymentmode


      TAMPER=36 ONDEMAND M Get Payment Mode: readpaymentmode


      TAMPER=37 ONDEMAND M Set Metering Mode: setmeteringmode


      TAMPER=38 ONDEMAND M Get Metering Mode: getmeteringmode


      TAMPER=39 ONDEMAND M Set Tariff agreement / TOU Table: toutariff


      TAMPER=40 ONDEMAND M Get Current Active tariff: getactivetariff


      TAMPER=41 ONDEMAND M Firmware Upgrade: firmwareupdate


      TAMPER=42 ONDEMAND M Get Firmware upgrade Status: firmwareupgradestatus


      TAMPER=43 ONDEMAND M Change Meter password: setmeterpassword


      TAMPER=44 ONDEMAND M Set Volt range Low (Threshold for voltage sag): setvoltrangelow


      TAMPER=45 ONDEMAND M Set Volt range Up (Threshold for voltage swell): setvoltrangeUp


      TAMPER=46 ONDEMAND M Get Volt range Low (Threshold for voltage sag): getvoltrangeLow


      TAMPER=47 ONDEMAND M Get Volt range Up (Threshold for voltage swell): getvoltrangeUp


      TAMPER=48 ONDEMAND M Set Current range Low: currentrangelow


      TAMPER=49 ONDEMAND M Set Current range Up: currentrangeUp


      TAMPER=50 ONDEMAND M Get Current range Low: getcurrentrangeLow


      TAMPER=51 ONDEMAND M Get Current range Up: getcurrentrangeUp

      TAMPER=52 ONDEMAND M Get MeterStatus: meterstatus

	  TAMPER=53 ONDEMAND DMM Read Gateway Name plate information: gwnameplate

	  TAMPER=54 ONDEMAND DMM Gateway Ping: ping

	  TAMPER=55 ONDEMAND DMM Meter Ping: meterping

	  TAMPER=56 ONDEMAND DMM Read Meter Name Plate Information: meternameplate
#endif
void     shutdown_set_sync_meter( uint32_t _sync );
void     shutdown_set_sync_modbus( uint32_t _sync );
uint32_t shutdown_initTelitModule( void );
void     shutdown_setInitTelitModule( uint32_t init );
uint32_t shutdown_sendKeepalive( void );
void     shutdown_setKeepalive( uint32_t send_keepalive );
uint32_t shutdown_get_tamper( void );
void     shutdown_set_tamper_param( uint32_t _tamper );
uint32_t shutdown_get_tamper_param( void );
void     shutdown_set_tamper_param_cmd( uint32_t _tamper );
uint32_t shutdown_get_tamper_param_cmd( void );
uint32_t shutdown_is_tamper_send_on_demand( void );
uint32_t shutdown_get_tamper_sending( void );
void     shutdown_set_tamper_sending( uint32_t _sending );
uint32_t shutdown_get_vbackup_sending( void );
void     shutdown_set_vbackup_sending( uint32_t _sending );
uint32_t shutdown_get_alarm_sending( void );
void     shutdown_set_alarm_sending( uint32_t _sending );
void     shutdown_set_power_mon( uint32_t _power_mon );
uint32_t shutdown_get_power_mon( void );
void     shutdown_mqttWaterCommandprocess( void );
void     shutdown_mqttCommandprocess( void );
void     shutdown_mqttCommandReconnectionprocess( uint32_t reconnection );
void     shutdown_mqttOnDemandCommandprocess( void );
void     shutdown_mqtt_restoreContext( void );
void     shutdown_mqtt_restorePressureContext( void );
void     shutdown_check_period_from_reset( void );
void     shutdown_on( uint32_t asleep_time );
uint32_t shutdown_is_shutdown_on( void );
void     shutdown_reset_watchdog( void );
void     shutdown_restart_watchdog( void );
void     shutdown_set_wchdg_time( uint32_t time );
void     shutdown_task( void );
void     shutdown_set_start_count( uint8_t _start_count );
uint8_t  shutdown_init_telit_module( void );
uint32_t shutdown_get_first_profile(uint32_t curr_dev);
int32_t  shutdown_get_last_profile_to_send( uint32_t curr_dev );
int32_t  shutdown_get_last_obis_profile_to_send( uint32_t curr_dev );
void     shutdown_reset_start_count( void );
void     shutdown_set_alarm_pressure_check( uint8_t _alarms_pressure_check );
void     shutdown_reset_alarm_pressure_check( void );
void     shutdown_check_pressure_alarm( void );
void     shutdown_set_alarm_generic_sensor_check( uint8_t _alarms_generic_sensor_check );
void     shutdown_reset_alarm_generic_sensor_check( void );
void     shutdown_check_generic_sensor_alarm( void );
uint32_t shutdown_init_dlms_count(void);
uint32_t shutdown_write_dlms_count(void);
uint32_t shutdown_read_dlms_count(void);
void     shutdown_reset_modbus_count( void );
void     shutdown_reset_meter_count( void );
void     shutdown_synch_count_params( void );
uint32_t shutdown_get_meter_send( void );
void     shutdown_set_meter_send( uint32_t _meter_send );
uint32_t shutdown_check_dlms_count_send_frame(void);
void     shutdown_set_reset_psm( uint32_t _reset_psm );
uint32_t shutdown_get_reset_psm( void );
void     shutdown_inc_num_readings( void );
void     shutdown_set_check_count_mqtt( uint32_t __check_count_mqtt );
void     shutdown_set_sending_mqtt_adc_on( uint32_t __sending_mqtt_adc_on );
uint32_t shutdown_get_sending_mqtt_adc_on( void );
void     shutdown_set_check_datalogger( uint32_t __check_datalogger );
uint32_t shutdown_get_check_datalogger( void );
uint8_t  shutdown_get_send_counter( void );
void     shutdown_set( uint32_t on, uint32_t time );
void     shutdown_max_allowed_readings_and_sendings( void );

int32_t  shutdown_get_obis_num_msg( void );
int32_t  shutdown_get_generic_profile_num_msg( void );
void     shutdown_set_obis_num_msg( int32_t _obis_num_msg );
void     shutdown_set_generic_profile_num_msg( int32_t _profile_num_msg );
int32_t  shutdown_get_obis_read_num_msg( void );
int32_t  shutdown_get_generic_profile_read_num_msg( void );
void 	 shutdown_set_obis_read_num_msg( int32_t _obis_num_msg );
void 	 shutdown_set_generic_profile_read_num_msg( int32_t _profile_num_msg );
int32_t  shutdown_get_profile_to_send(uint32_t client_id, uint32_t index);
int32_t  shutdown_init_profile_to_send(void);



#endif /* APPLICATION_USER_COMMON_INC_SHUTDOWN_H_ */
