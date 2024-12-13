/*
 * mqtt_task.h
 *
 *  Created on: Jan 29, 2021
 *      Author: Sergio Millán López
 */

#ifndef MQTT_INC_MQTT_TASK_H_
#define MQTT_INC_MQTT_TASK_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

#include "params.h"
#include "MQTTPacket.h"
#include "MQTTConnect.h"
#include "MQTTFormat.h"
#include "MQTTPublish.h"
#include "MQTTSubscribe.h"
#include "MQTTUnsubscribe.h"

typedef enum {
    PT_CONNECTION,
	PT_REGISTER,
    PT_STATUS,
    PT_CONFIG_TIME,
	PT_FRAME_P_NETWORK_PARAMS,
	PT_FRAME_F,
	PT_FRAME_MF,
	PT_FRAME_S,
	PT_FRAME_S_PLUS,
    PT_TEST_CONNECTION,
	PT_TEST_RESULT,
	PT_DEV_REGISTERS,
	PT_COMMAND_ON_DEMAND_ENERGY_REGISTERS,
	PT_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS,
	PT_COMMAND_ON_DEMAND_LOAD_PROFILE_1,
	PT_COMMAND_ON_DEMAND_LOAD_PROFILE_2,
	PT_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE,
	PT_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE,
	PT_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES,
	PT_COMMAND_ON_DEMAND_BILLING_PROFILE,
	PT_COMMAND_ON_DEMAND_EVENT_PROFILE,
	PT_COMMAND_READ_TIME_CLOCK,
	PT_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD,
	PT_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD,
	PT_COMMAND_GET_SWITCH_STATUS,
	PT_COMMAND_CUTTOFF_RECONNECTION,
	PT_COMMAND_MAXIMUM_DEMAND_RESET,
	PT_COMMAND_SET_LOAD_LIMITATION,
	PT_COMMAND_GET_LOAD_LIMIT_THRESHOLD,
	PT_COMMAND_SET_BILLING_DATE,
	PT_COMMAND_GET_BILLING_DATE,
	PT_COMMAND_CLEAR_ALARMS,
	PT_COMMAND_SET_DEMAND_INTEGRATION_PERIOD,
	PT_COMMAND_GET_DEMAND_INTEGRATION_PERIOD,
	PT_COMMAND_SET_PAYMENT_MODE,
	PT_COMMAND_GET_PAYMENT_MODE,
	PT_COMMAND_SET_METERING_MODE,
	PT_COMMAND_GET_METERING_MODE,
	PT_COMMAND_SET_VOLT_RANGE_LOW,
	PT_COMMAND_SET_VOLT_RANGE_UP,
	PT_COMMAND_GET_VOLT_RANGE_LOW,
	PT_COMMAND_GET_VOLT_RANGE_UP,
	PT_COMMAND_SET_CURRENT_RANGE_LOW,
	PT_COMMAND_SET_CURRENT_RANGE_UP,
	PT_COMMAND_GET_CURRENT_RANGE_LOW,
	PT_COMMAND_GET_CURRENT_RANGE_UP,
	PT_COMMAND_GET_METER_STATUS,
	PT_COMMAND_READ_METER_PLATE,
	PT_COMMAND_METER_SYNCHRONIZE,
	PT_COMMAND_RECONNECTION,
	PT_COMMAND_GET_CURRENT_ACTIVE_TARIFF,
	PT_COMMAND_METER_PING,
	PT_COMMAND_WATERPROFILE,
	PT_COMMAND_SET_TARIFF_AGREEMENT,
	PT_COMMAND_GET_GW_TIME,
	PT_COMMAND_SET_GW_SYNCH,
	PT_COMMAND_SET_GW_INTERVAL,
	PT_COMMAND_GET_GW_INTERVAL,
	PT_COMMAND_GET_GW_NAMEPLATE,
	PT_COMMAND_GW_PING,
	PT_COMMAND_GW_FIRMWARE_UPDATE,
	PT_LAST
} publication_topic;

typedef enum {
	QOS0,
	QOS1,
	QOS2
} QoS;

// all failure return codes must be negative
typedef enum {
	RC_BUFFER_OVERFLOW = -2,
	RC_FAILURE = -1,
	RC_SUCCESS = 0
} returnCode;

typedef struct MQTTMessage MQTTMessage;
typedef struct MessageData MessageData;

struct MQTTMessage
{
    QoS      qos;
    char     retained;
    char     dup;
    uint16_t id;
    void     *payload;
    size_t   payloadlen;
};

struct MessageData
{
    MQTTMessage* message;
    MQTTString* topicName;
};

typedef void (*messageHandler)(MessageData*);

void     mqtt_task( void );

uint32_t mqtt_get_start_frames( void );
uint32_t mqtt_get_send_activation( void );
uint32_t mqtt_set_send_activation( uint32_t _send_act );
uint32_t mqtt_get_reply_get_topic( void );
uint32_t mqtt_set_reply_get_topic( uint32_t _reply_get );
uint32_t mqtt_get_relay( void );
uint32_t mqtt_set_relay( uint32_t _mqtt_relay );
void     mqtt_task_get_firmware( void );
char     mqtt_server_get_new_index( void );
char     mqtt_server_get_index( void );
void     mqtt_server_reset_index( void );
uint32_t mqtt_get_connection_time(void);
uint32_t mqtt_get_desconnection_time(void);
uint8_t  mqtt_is_idle( void );
void 	 mqtt_start( void );
void 	 mqtt_stop( void );
uint8_t  mqtt_is_state_idle( void );
uint8_t  mqtt_is_state_send_ping( void );
uint8_t  mqtt_is_state_disconnected( void );
void     mqtt_set_connection_time(uint32_t conn_time);
void     mqtt_set_desconnection_time(uint32_t desconn_time);

uint8_t  mqtt_supervisor_get_fall_FLAG( void );
uint32_t mqtt_task_get_send_pending( void );
void     mqtt_task_set_send_pending( uint32_t  pend );
uint32_t mqtt_task_upgrading_firmware(void);
void     mqtt_task_set_upgrading_firmware(uint32_t _mqtt_upgrading_firmware);
void     mqtt_task_get_upgrading_data( void );
uint32_t mqtt_task_check_upgrading_data(void);
void     mqtt_task_reset_upgrading_data(void);
char *   mqtt_task_get_app_data( void );
size_t   mqtt_task_msg_payloadlen( void );

#endif /* MQTT_INC_MQTT_TASK_H_ */
