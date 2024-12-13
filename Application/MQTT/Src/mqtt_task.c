/*
 * mqtt_task.c
 *
 *  Created on: Jan 29, 2021
 *      Author: Sergio Millán López
 */
#define _GNU_SOURCE

#include "params.h"
#ifdef MQTT
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "mqtt_task.h"
#include "mqtt_buffer.h"
#include "mqtt_timer.h"
#include "mqtt_frames.h"
//#include "message_queue.h"
#include "comm_udp.h"
#include "comm_serial.h"
#include "tick.h"
#include "json.h"
#include "shutdown.h"
#include "battery.h"
#include "fw_update.h"

#define MQTT_BUFFER_SIZE    ( RXSIZE )
#define MAX_PACKET_ID       ( 65535 )

////////////////////////////////////////////////////////////////////////////////

static enum
{
  	MQTT_IDLE,
	TCP_DISCONNECTED, TCP_WAITING_FOR_CONNECTION, MQTT_WAITING_FOR_CONNECTION_1,
   	MQTT_SELECT_LOOP,
    MQTT_SEND_ACK, MQTT_SEND_SUBSCRIPTION, MQTT_SEND_APP_MSG, MQTT_SEND_CLOSE, MQTT_SEND_PING,
    MQTT_RECEIVE,
    MQTT_RECEIVE_NEW_FIRMWARE,
    TCP_DISCONNECT
} status = MQTT_IDLE;

// functions for connection
static int32_t mqtt_set_connection_options( void );

// functions for subscriptions
static char MyID[ 18 ];
static char MyGROUP[ 18 ];
static char MyPLATFORM[ 18 ];

#define MAX_MESSAGE_HANDLERS    ( 8 )//( 4 )
static char   subscription_topic[ MAX_MESSAGE_HANDLERS ][ 64 ];
static int    subscription_qos[   MAX_MESSAGE_HANDLERS ];

static int32_t  mqtt_set_subscription_options( uint8_t hwID_device_FLAG );

static uint8_t  deliverMessage( MQTTString *topic, MQTTMessage *msg, uint32_t rcv_len );

// functions for publications
static int8_t   mqtt_publish_topic( uint8_t topicc_n );

// aux functions
static int8_t   mqtt_get_packet_type( void );
static uint32_t getNextPacketId( void );

static MQTTString  topicName;
static MQTTMessage msg;

// supervisor
static void mqtt_supervisor( void );
static void mqtt_supervisor_clear( void );
//static void mqtt_supervisor_set( void );

extern struct params param;
extern uint32_t socket_data_len;

char   app_data[ MQTT_BUFFER_SIZE ];
static uint32_t updating_firmware;
static uint32_t connection_time, desconnection_time;
MQTTString fw_update_topicName;
MQTTMessage fw_update_msg;

// SUPERVISOR //////////////////////////////////////////////////////////////////

// Flag para marcar cuando, tras enviar el último paquete de configuración "ans/config/output_config", perdemos la conexión

uint8_t  mqtt_supervisor_status, mqtt_supervisor_fall_FLAG;
uint32_t mqtt_task_send_pending = 0, mqtt_upgrading_firmware = 0, mqtt_upgrade_rx_data = 0;
uint32_t mqtt_start_frames = 0, mqtt_send_activation = 0, mqtt_reply_get_topic = 0, mqtt_relay = 0;

uint32_t mqtt_get_start_frames( void )
{
	return mqtt_start_frames;
}

uint32_t mqtt_get_send_activation( void )
{
	return mqtt_send_activation;
}

uint32_t mqtt_set_send_activation( uint32_t _send_act )
{
	mqtt_send_activation = _send_act;
	return 0;
}

uint32_t mqtt_get_reply_get_topic( void )
{
	return mqtt_reply_get_topic;
}

uint32_t mqtt_set_reply_get_topic( uint32_t _reply_get )
{
	mqtt_reply_get_topic = _reply_get;
	return 0;
}

uint32_t mqtt_get_relay( void )
{
	return mqtt_relay;
}

uint32_t mqtt_set_relay( uint32_t _mqtt_relay )
{
	mqtt_relay = _mqtt_relay;
	return 0;
}

void mqtt_task_get_firmware( void )
{
	fw_update_erase_flash();
}

static void mqtt_supervisor()
{
    static uint8_t last_mqtt_supervisor_status;

    if ( ! last_mqtt_supervisor_status &&   mqtt_supervisor_status ) {
    	mqtt_supervisor_fall_FLAG = 0;
    }
    else if ( last_mqtt_supervisor_status && ! mqtt_supervisor_status ) {
    	mqtt_supervisor_fall_FLAG = 1;
    }

    last_mqtt_supervisor_status = mqtt_supervisor_status;
}

static void mqtt_supervisor_clear()
{
    mqtt_supervisor_status = 0;
}

//static void mqtt_supervisor_set()
//{
//    mqtt_supervisor_status = 1;
//}

uint8_t mqtt_supervisor_get_fall_FLAG()
{
    return mqtt_supervisor_fall_FLAG;
}

uint32_t mqtt_task_get_send_pending( void )
{
	return mqtt_task_send_pending;
}

void mqtt_task_set_send_pending( uint32_t  pend )
{
	mqtt_task_send_pending = pend;
}

uint32_t mqtt_task_upgrading_firmware(void)
{
    return mqtt_upgrading_firmware;
}

void mqtt_task_set_upgrading_firmware(uint32_t _mqtt_upgrading_firmware)
{
	mqtt_upgrading_firmware = _mqtt_upgrading_firmware;
}

void mqtt_task_get_upgrading_data( void )
{
	if (comm_serial_rcx_bytes_n()) {
//		memset( app_data, 0, strlen(app_data) );
		memset( app_data, 0, sizeof(uint8_t) * MQTT_BUFFER_SIZE );
		mqtt_upgrade_rx_data = comm_serial_read_while_data((uint8_t *)app_data);
	} else {
		comm_serial_enable_rts();
	}
}

uint32_t mqtt_task_check_upgrading_data(void)
{
    return mqtt_upgrade_rx_data;
}

void mqtt_task_reset_upgrading_data(void)
{
	mqtt_upgrade_rx_data = 0;
}

char * mqtt_task_get_app_data( void )
{
	return app_data;
}

size_t mqtt_task_msg_payloadlen( void )
{
	return msg.payloadlen;
}

// AUX /////////////////////////////////////////////////////////////////////////

int8_t mqtt_get_packet_type()
{
    MQTTHeader header = {0};

    header.byte = app_data[0];

    return header.bits.type;
}

uint32_t getNextPacketId()
{
    static uint32_t packetId = 1;

    packetId = ( packetId == MAX_PACKET_ID ) ? 1 : packetId + 1;

    return packetId;
}

////////////////////////////////////////////////////////////////////////////////

static char server_i;

char mqtt_server_get_new_index() // OK = 0 o 1, KO > 1
{
    unsigned char i;

    server_i++;

    for( i = server_i; i < 4; i++ ) {
        if( param.server[ i >> 1 ].name[ 0 ] != '\0' ) break;
    }

    server_i = i;

    return server_i >> 1;
}

char mqtt_server_get_index()
{
    return server_i >> 1;
}

void mqtt_server_reset_index()
{
    server_i = -1;
}

uint32_t mqtt_firmware_update( void )
{
    return updating_firmware;
}

void mqtt_set_updating_firmware( uint32_t onoff )
{
    updating_firmware = onoff;
}

uint32_t mqtt_get_connection_time(void)
{
    return connection_time;
}

void mqtt_set_connection_time( uint32_t conn_time )
{
    connection_time = conn_time;
}

uint32_t mqtt_get_desconnection_time(void)
{
    return desconnection_time;
}

void mqtt_set_desconnection_time( uint32_t desconn_time )
{
    desconnection_time = desconn_time;
}

uint8_t mqtt_is_idle( void )
{
    return( status == MQTT_SELECT_LOOP ) ? 1 : 0;
}

void mqtt_start( void )
{
	status = TCP_DISCONNECTED;
}

void mqtt_stop( void )
{
	if (status != MQTT_IDLE)
	{
		LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT> STOP MQTT CONN.\r\n", (int)Tick_Get( SECONDS ));
		status = MQTT_SEND_CLOSE;//TCP_DISCONNECT;
	}
}

uint8_t mqtt_is_state_idle( void )
{
	uint8_t ret = 0;
	if ( MQTT_IDLE == status )
	{
		ret = 1;
	}
	return ret;
}

uint8_t mqtt_is_state_send_ping( void )
{
	uint8_t ret = 0;
	if ( MQTT_SEND_PING == status )
	{
		ret = 1;
	}
	return ret;
}

uint8_t mqtt_is_state_disconnected( void )
{
	uint8_t ret = 0;
	if ( TCP_DISCONNECTED == status )
	{
		ret = 1;
	}
	return ret;
}
//static publication_topic __decode_topic( publication_topic _queue_msg )
//{
//	publication_topic pt = PT_LAST;
//
//	switch (_queue_msg) {
//	case SEND_MESSAGE:
//		pt = PT_FRAME_F;
//		break;
//	case SEND_NETWORK_PARAMS:
//		pt = PT_FRAME_P_NETWORK_PARAMS;
//		break;
//	case SEND_SENSOR:
//		if ( 0 == params_pulses_on() ) {
//			pt = PT_FRAME_S;
//		} else if ( 1 == params_pulses_on() ) {
//			pt = PT_FRAME_S_PLUS;
//		}
//		break;
//	case SEND_MODBUS_SENSOR:
//		pt = PT_FRAME_MF;
//		break;
//	case SEND_PT_CONNECTION:
//		pt = PT_CONNECTION;
//		break;
//	case SEND_PT_REGISTER:
//		pt = PT_REGISTER;
//		break;
//	case SEND_PT_TIME:
//		pt = PT_CONFIG_TIME;
//		break;
//	case SEND_PT_STATUS:
//		pt = PT_STATUS;
//		break;
//	case SEND_PT_TEST_CONNECTION:
//		pt = PT_TEST_CONNECTION;
//		break;
//	case SEND_PT_TEST_RESULT:
//		pt = PT_TEST_RESULT;
//		break;
//	default:
//		break;
//	}
//
//	return pt;
//}

uint32_t __checkSysIdle( void )
{
	uint32_t ret = 0;

	if ( 1 == mqtt_timer_get_idle_mode() )
	{
		ret = 1;
	}
	else
	{
		__NOP();
	}

	return ret;
}
//uint32_t __checkSysIdle( void )
//{
//	uint32_t ret = 1;
//
//	if ( 1 == params_config_get_sync_read() ) {
//		if ( 1 == mqtt_timer_get_idle_mode() ) {
//			if ( 1 == rtc_system_getAlarmOccured() ) {
//				LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> Wake-up from Alarm", (int)Tick_Get( SECONDS ));
//				/* Disable the Alarm A */
//				HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
//				rtc_system_setAlarmOccured(0);
////				shutdown_mqtt_restoreContext();
//				shutdown_set_start_count(0);
//				mqtt_timer_set_idle_mode(0);
//				if ( 0 == mqtt_is_state_idle() )
//				{
//					mqtt_stop();
//				}
//				ret = 0;
//			}
//		} else {
//			ret = 0;
//		}
//	}
//	if ( 0 == params_config_get_sync_read() ) {
//		if ( 1 == mqtt_timer_get_idle_mode() ) {
//			if ( 0 == mqtt_timer_get( T_IDLE ) ) {
//				LOGLIVE(LEVEL_1, "-----------------------------Wake-up from T_IDLE:%d", (int)Tick_Get( SECONDS ));
////				shutdown_mqtt_restoreContext();
//				shutdown_set_start_count(0);
//				mqtt_timer_set_idle_mode(0);
//				if ( 0 == mqtt_is_state_idle() )
//				{
//					mqtt_stop();
//				}
//				ret = 0;
//			}
//		} else {
//			ret = 0;
//		}
//	}
//
//	if ( 1 == mqtt_timer_get_idle_mode() ) {
//		if ( 1 == mqtt_get_send_activation() ) {
//			LOGLIVE(LEVEL_1, "-----------------------------Wake-up from PT_DEV_REGISTERS:%d", (int)Tick_Get( SECONDS ));
////			shutdown_mqtt_restoreContext();
//			shutdown_set_start_count(0);
//			mqtt_timer_set_idle_mode(0);
//			if ( 0 == mqtt_is_state_idle() )
//			{
//				mqtt_stop();
//			}
//			ret = 0;
//		}
//		if ( 1 == mqtt_get_reply_get_topic() ) {
//			LOGLIVE(LEVEL_1, "-----------------------------Wake-up from GET TOPIC:%d", (int)Tick_Get( SECONDS ));
////			shutdown_mqtt_restoreContext();
//			shutdown_set_start_count(0);
//			mqtt_timer_set_idle_mode(0);
//			if ( 0 == mqtt_is_state_idle() )
//			{
//				mqtt_stop();
//			}
//			ret = 0;
//		}
//	}
//
//	return ret;
//}

uint32_t time_mqtt_protocol = 0;
uint32_t fw_length          = 0;
void mqtt_task( void )
{
    static uint32_t len;
    static uint8_t  subscribed, ping_n, registered;

    uint32_t sys_idle = __checkSysIdle();

	if ( udp_is_socket_open() && ( 0 == mqtt_timer_get_idle_mode() )/*&& ( mqtt_firmware_update() == 0 ) && mqtt_is_idle()*/ )
	{
        if ( comm_serial_get_NO_CARRIER() )
        {
        	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> NO CARRIER\r\n", (int)Tick_Get( SECONDS ));
        	comm_serial_delete( comm_serial_rcx_bytes_n() );
        	Tick_update_tick( TICK_UDPPROT );
            status = TCP_DISCONNECT;
        }
    }

    mqtt_supervisor();

    switch( status )
    {
    	case MQTT_IDLE:
    		break;

    	case TCP_DISCONNECTED:
            mqtt_supervisor_clear();
            mqtt_start_frames = 0;
            if( udp_is_socket_open() )
            {
                subscribed = ping_n = param.version.crc = 0;
                memset( (char *) app_data, 0, sizeof( app_data ));
                memset( (char *) &topicName, 0, sizeof( topicName ));
                memset( (char *) &msg, 0, sizeof( msg ));
                MyPLATFORM[0] = MyGROUP[0] = MyID[0] = '\0';
                time_mqtt_protocol = 10;
                Tick_update_tick( TICK_UDPPROT );
                status++;
            }
            break;
        case TCP_WAITING_FOR_CONNECTION:
            if( udp_is_socket_open() )
            {
                if( comm_serial_is_put_ready() )
                {
                	Telit_write_data_length( app_data, mqtt_set_connection_options() );
                    registered         = param.registered;
                    time_mqtt_protocol = 4;
                    Tick_update_tick( TICK_UDPPROT );
                    status++;
                }
            }
            else if ( CHECK_ELAPSED_TIME( time_mqtt_protocol, TICK_UDPPROT ) )
            {
                status = TCP_DISCONNECTED;
            }
            break;
        case MQTT_WAITING_FOR_CONNECTION_1:
            if( udp_is_socket_open() )
            {
            	if ( CHECK_ELAPSED_TIME( time_mqtt_protocol + 1, TICK_UDPPROT ) )
            	{
                    status = TCP_DISCONNECT;
                    break;
                }
            	if ( CHECK_ELAPSED_TIME( time_mqtt_protocol, TICK_UDPPROT ) )
            	{
                    len = comm_serial_rcx_n( (uint8_t *)app_data, comm_serial_rcx_bytes_n() );
                    if ( len && ( mqtt_get_packet_type() == CONNACK ) )
                    {
                        unsigned char sessionPresent, connack_rc;
                        if ( MQTTDeserialize_connack( &sessionPresent, &connack_rc, (unsigned char *)app_data, len ) == 1 )
                        {
                            if( !connack_rc )
                            {
                                subscribed = 0;
                                ping_n     = 0;
                                mqtt_buffer_init();
                                mqtt_buffer_ACKs_init();
                                mqtt_timer_init();
//                                LEDs_set_net_status( MQTT_Y );
                                comm_serial_delete(4);
                                time_mqtt_protocol = 4;
                                Tick_update_tick( TICK_UDPPROT );
//                                Alive( WD_COMM );
                                status++;
                                mqtt_set_connection_time( Tick_Get( SECONDS ) );
                                LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> MQTT Connected.\r\n", (int)Tick_Get( SECONDS ));
                            }
                            else
                            {
                                time_mqtt_protocol = 2;
                                Tick_update_tick( TICK_UDPPROT );
                                status = TCP_DISCONNECT;
                            }
                        }
                    }
                }
            }
            else
            {
                status = TCP_DISCONNECTED;
            }
            break;
        case MQTT_SELECT_LOOP:
            if ( udp_is_socket_open() )
            {
//                uint32_t sys_idle = __checkSysIdle();
            	if ( ! mqtt_timer_get( T_KEEPALIVE ) )
            	{
                    ping_n = 3;
                    status = MQTT_SEND_PING;
//                    printf("timeout keepalive:%d\n", (int)Tick_Get( SECONDS ));
                }
            	else if ( ping_n && ! mqtt_timer_get( T_PING ) )
            	{
                    if ( --ping_n )
                    {
                        status = MQTT_SEND_PING;
                    }
                    else
                    {
                        status             = TCP_DISCONNECT;
                    	time_mqtt_protocol = 2;
                        Tick_update_tick( TICK_UDPPROT );
                    }
                }
            	else if ( mqtt_buffer_ACKs_get_elements() && ( 0 == mqtt_timer_get( T_ACK ) ) )
            	{
                    status = MQTT_SEND_ACK;
                }
            	else if ( comm_serial_rcx_bytes_n() )
            	{
                    status             = MQTT_RECEIVE;
                	time_mqtt_protocol = 3;
                    Tick_update_tick( TICK_UDPPROT );
                }
            	else if ( ! subscribed && ! mqtt_timer_get( T_SUBS ) )
            	{
                    status = MQTT_SEND_SUBSCRIPTION;
                }
            	else if ( ( mqtt_frames_get_wait_server_resp() != LAST_SERVER_RESP ) && ( 0 == mqtt_timer_get( T_TIMEOUT_SERVER ) ) )
            	{
                	mqtt_frames_processTimeoutWaitingResponse();
                	mqtt_timer_set( T_TIMEOUT_SERVER );
                }
            	else if ( ( ( mqtt_buffer_get_elements() != 0 ) || ( mqtt_frames_get_wait_server_resp() != LAST_SERVER_RESP ) )
                		 && ( 0 == mqtt_timer_get( T_PUB ) )
						 && ( mqtt_timer_get( T_KEEPALIVE ) )
						 && ( 0 == ping_n )
						 && ( 0 == sys_idle )
						  )
            	{
                	if ( (mqtt_frames_get_wait_server_resp() != LAST_SERVER_RESP) )
                	{
                		mqtt_frames_processNextMsg(mqtt_frames_get_wait_server_resp());
                	}
                	if ( ( (mqtt_frames_get_wait_server_resp() == LAST_SERVER_RESP) && ( mqtt_buffer_get_elements() == 0 ) ) )
                	{
                		status = MQTT_SELECT_LOOP;
                	}
                	else
                	{
                		status = MQTT_SEND_APP_MSG;
                	}
                }
            	else if ( param.version.crc && ! mqtt_timer_get( T_UPDATE_READY ) )
            	{
                    status = MQTT_SEND_CLOSE;
                }
            	else if ( registered != param.registered )
            	{
                    status = MQTT_SEND_CLOSE;
                }
//            	else if (1 == mqtt_timer_get_idle_mode())
//            	{
//            		LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> MQTT SEND CLOSE FROM LOOP.\r\n", (int)Tick_Get( SECONDS ));
//            		status = MQTT_SEND_CLOSE;
//            	}
                if ( LAST_SERVER_RESP == mqtt_frames_get_wait_server_resp() )
                {// Not waiting for server reply, reload timer.
                	mqtt_timer_set( T_TIMEOUT_SERVER );
                }
            }
            else
            {
            	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK>TCP_DISCONNECTED!!!!!!!\r\n", (int)Tick_Get( SECONDS ));
            	status = TCP_DISCONNECTED;
            }
            break;
        case MQTT_SEND_ACK:
            if ( udp_is_socket_open() &&  comm_serial_is_put_ready() )
            {
                unsigned short mypacketid;
                unsigned char type;
                type = mqtt_buffer_ACKs_read( &mypacketid );
                if ( mypacketid == msg.id )
                {
                    len = MQTTSerialize_ack( (unsigned char *)app_data, MQTT_BUFFER_SIZE, type, 0, mypacketid );
                    if ( len > 0 )
                    {
                    	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> ACK snd.\r\n", (int)Tick_Get( SECONDS ));
                    	Telit_write_data_length( app_data, len );
                        if ( type == PUBREL )
                        {
                            mqtt_timer_set( T_ACK );
                        }
                        else
                        {
                            mqtt_buffer_ACKs_delete( 1 );
                        }
                    }
                    if (mqtt_firmware_update() == 1)
                    {
                        status = MQTT_RECEIVE_NEW_FIRMWARE;
                    }
                    else
                    {
                        status = MQTT_SELECT_LOOP;
                    }
                }
                else
                {
                    mqtt_buffer_ACKs_delete( 1 );
                	time_mqtt_protocol = 2;
                    Tick_update_tick( TICK_UDPPROT );
                    status = TCP_DISCONNECT;
                    LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> ACK not snd. %d %d.\r\n", (int)Tick_Get( SECONDS ),
                    	(int)mypacketid,(int)msg.id	);
                }
            }
            break;
        case MQTT_SEND_SUBSCRIPTION:
            if ( udp_is_socket_open() && comm_serial_is_put_ready() )
            {
                len = mqtt_set_subscription_options( param.registered );
                if ( len > 0 )
                {
                	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> MQTT SEND SUB\r\n", (int)Tick_Get( SECONDS ));
                	Telit_write_data_length( app_data, len );
                    mqtt_timer_set( T_SUBS );
                }
            }
            status = MQTT_SELECT_LOOP;
            break;
        case MQTT_SEND_APP_MSG:
            if ( udp_is_socket_open() && comm_serial_is_put_ready() )
            {
                MQTTString topic = MQTTString_initializer;
                int8_t     topic_n, qos;
                int32_t    pub_id;

                memset( (char *) app_data, 0, sizeof( app_data ) );
                if ( 0 == mqtt_buffer_get_elements() )
                {
                	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> no message_queue to send.\r\n", (int)Tick_Get( SECONDS ));
                	status = MQTT_SELECT_LOOP;
                    break;
                }
//                topic_n = __decode_topic((publication_topic)mqtt_buffer_read());
                topic_n = mqtt_buffer_read();
                qos     = mqtt_publish_topic( topic_n );
                pub_id  = getNextPacketId();
                LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> PUBLISH TOPIC:%d. QoS:%d. Id:%d.\r\n", (int)Tick_Get( SECONDS ), (int)topic_n, (int)qos, (int)pub_id);
                if ( qos != -1 )
                {
                    len = json_mqtt_encode( topic_n, (char *) &app_data[64] ); // pq header + length + topic < 64
                    topic.cstring = (char *) &app_data[ 5 ];
                    len = MQTTSerialize_publish( (unsigned char *)app_data, MQTT_BUFFER_SIZE, 0, qos, 0, pub_id, topic, (unsigned char *)&app_data[64], len );
                    if ( len )
                    {
                    	Telit_write_data_length( app_data, len );
                        if ( qos == QOS0 )
                        {
                        	mqtt_buffer_delete( 1 );
                        }
                        else
                        {
                            mqtt_timer_set( T_PUB );
                            mqtt_timer_set( T_TIMEOUT_SERVER );
                        }
//                        LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> PUBLISH", (int)Tick_Get( SECONDS ));
//                        if( topic_n == PT_CONFIG_OUTPUT_CONFIG ) {
//                            mqtt_supervisor_set();
//                        }
                    }
                }
            }
            status = MQTT_SELECT_LOOP;
            break;
        case MQTT_SEND_CLOSE:
            if ( udp_is_socket_open() && comm_serial_is_put_ready() )
            {
            	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> SEND CLOSE.\r\n", (int)Tick_Get( SECONDS ));
            	len = MQTTSerialize_disconnect( (unsigned char *)app_data, MQTT_BUFFER_SIZE );
                if ( len > 0 )
                {
                	Telit_write_data_length( app_data, len );
                	time_mqtt_protocol = 2;
                    Tick_update_tick( TICK_UDPPROT );
                    status = TCP_DISCONNECT;
                    break;
                }
            }
            else
            {
            	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> CANNOT SEND CLOSE!!!.\r\n", (int)Tick_Get( SECONDS ));
            	time_mqtt_protocol = 2;
                Tick_update_tick( TICK_UDPPROT );
                status = TCP_DISCONNECT;
                break;
            }
            status = MQTT_SELECT_LOOP;
            break;
        case MQTT_SEND_PING:
            if ( udp_is_socket_open() && comm_serial_is_put_ready() )
            {
                len = MQTTSerialize_pingreq( (unsigned char *)app_data, MQTT_BUFFER_SIZE );
                if ( len > 0 )
                {
                	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> PING\r\n", (int)Tick_Get( SECONDS ));
//                	rtc_system_SetServerTime( Tick_Get(SECONDS) );
                	Telit_write_data_length( app_data, len );
                    mqtt_timer_set( T_PING );
                    mqtt_timer_set( T_KEEPALIVE );
                }
            }
            status = MQTT_SELECT_LOOP;
            break;
        case MQTT_RECEIVE:
            if ( udp_is_socket_open() )
            {
            	if ( CHECK_ELAPSED_TIME( time_mqtt_protocol, TICK_UDPPROT ) )
            	{
            		comm_serial_delete( comm_serial_rcx_bytes_n() );
//                    Alive( WD_COMM );
                    status = MQTT_SELECT_LOOP;
                    break;
                }

                uint16_t id;
                uint8_t  dup, type;
                int      count;

                if ( CHECK_ELAPSED_MILISEC( 100, TICK_UDPPROT ) )
                {
                    memset((char *) app_data, 0, sizeof ( app_data));
                    len = comm_serial_rcx_n((uint8_t *)app_data, comm_serial_rcx_bytes_n());

                    switch (mqtt_get_packet_type())
                    {
                        case SUBACK:
                            if ( MQTTDeserialize_suback(&id, MAX_MESSAGE_HANDLERS, &count, &subscription_qos[0], (unsigned char *)app_data, len) == 1 )
                            {
                                uint8_t i;
                                for (i = count; i < MAX_MESSAGE_HANDLERS; i++)
                                {
                                	subscription_qos[ i ] = 0x80;
                                }
                                subscribed = 1;
                                mqtt_timer_reset(T_SUBS);
                                comm_serial_delete(4 + count);
                                if (param.registered)
                                {
                                	mqtt_buffer_write(PT_CONNECTION);
//                                	shutdown_set_start_count(0);
                                }
                                else
                                {
                                	mqtt_buffer_write( PT_TEST_CONNECTION );
                                }
//                                Alive(WD_COMM);
                                status = MQTT_SELECT_LOOP;
                            }
                            break;
                        case PUBACK:
                            if (MQTTDeserialize_ack(&type, &dup, &id, (unsigned char *)app_data, len) == 1)
                            {
                            	mqtt_buffer_delete( 1 );
                                mqtt_timer_reset( T_PUB );
                                mqtt_timer_set( T_TIMEOUT_SERVER );
                                comm_serial_delete( 4 );
//                                Alive( WD_COMM );
                                status = MQTT_SELECT_LOOP;
                                LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> PUBACK.\r\n", (int)Tick_Get( SECONDS ));
                                if ( ( 1 == mqtt_get_reply_get_topic() ) || ( 1 == mqtt_get_send_activation() ) )
                                {
                                	shutdown_set( 1, rtc_system_getReadSensorAlarmCycleNextInSeconds() );
                                }
                            }
                            break;
                        case PUBREC:
                            if (MQTTDeserialize_ack(&type, &dup, &id, (unsigned char *)app_data, len) == 1)
                            {
                            	mqtt_buffer_delete(1);
                                mqtt_timer_reset(T_PUB);
                                mqtt_buffer_ACKs_write(PUBREL, id);
                                comm_serial_delete(4);
//                                Alive(WD_COMM);
                                status = MQTT_SELECT_LOOP;
                            }
                            break;
                        case PUBREL:
                            if (MQTTDeserialize_ack(&type, &dup, &id, (unsigned char *)app_data, len) == 1)
                            {
                                mqtt_buffer_ACKs_write(PUBCOMP, id);
                                comm_serial_delete(4);
//                                Alive(WD_COMM);
                                status = MQTT_SELECT_LOOP;
                            }
                            break;
                        case PUBCOMP:
                            if (MQTTDeserialize_ack(&type, &dup, &id, (unsigned char *)app_data, len) == 1)
                            {
                                mqtt_buffer_ACKs_delete(1);
                                mqtt_timer_reset(T_ACK);
                                comm_serial_delete(4);
//                                Alive(WD_COMM);
                                status = MQTT_SELECT_LOOP;
                            }
                            break;
                        case PUBLISH:
                        {
                            if (MQTTDeserialize_publish((unsigned char *) &msg.dup, (int *) &msg.qos,
                                    (unsigned char*) &msg.retained, (unsigned short*) &msg.id, &topicName,
                                    (unsigned char**) &msg.payload, (int *) &msg.payloadlen, (unsigned char *)app_data, len) == 1)
                            {
                                switch (deliverMessage(&topicName, &msg, len))
                                {
                                    case 0:
                                        break;
                                    case 1:
                                        if ( msg.qos == QOS0 ) {
                                            if (status != MQTT_SEND_CLOSE)
                                            {
                                            	status = MQTT_SELECT_LOOP;
                                            }
                                        } else if (msg.qos == QOS1) {
                                            mqtt_buffer_ACKs_write(PUBACK, msg.id);
                                            status = MQTT_SELECT_LOOP;
                                        } else if (msg.qos == QOS2) {
                                            mqtt_buffer_ACKs_write(PUBREC, msg.id);
                                            status = MQTT_SELECT_LOOP;
                                        } else {
                                        	time_mqtt_protocol = 2;
                                            Tick_update_tick( TICK_UDPPROT );
                                            status = TCP_DISCONNECT;
                                        }
                                        LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> PUBLISH RECV.\r\n", (int)Tick_Get( SECONDS ));
//                                        Alive(WD_COMM);
                                        break;
                                    case 2:
                                    	memcpy( (void *) &fw_update_topicName, (void *) &topicName, sizeof(topicName) );
										memcpy( (void *) &fw_update_msg, (void *) &msg, sizeof(msg) );
										socket_data_len = len;//comm_serial_rcx_n((uint8_t *)app_data, comm_serial_rcx_bytes_n());//comm_serial_rcx_bytes_n();//
										comm_serial_delete(socket_data_len);
										socket_data_len -= (char *) fw_update_msg.payload - (char *) app_data;
//										LOGLIVE(LEVEL_1, "update firmware:%d socket_data_len:%d", (int)Tick_Get( SECONDS ), (int)socket_data_len);
                                    	mqtt_set_updating_firmware(1);
                                        mqtt_buffer_ACKs_write(PUBACK, msg.id);
                                        status = MQTT_SEND_ACK;//MQTT_SELECT_LOOP;
                                        mqtt_timer_set( T_KEEPALIVE );
//                                        Alive(WD_COMM);
//                                        status = MQTT_RECEIVE_NEW_FIRMWARE;
                                        break;
                                    default:
                                    	time_mqtt_protocol = 2;
                                        Tick_update_tick( TICK_UDPPROT );
                                        status = TCP_DISCONNECT; // MQTT_CLEAN;
                                        break;
                                }
                            }
                        }
                            break;
                        case PINGRESP:
                            if (len > 1) {
                                mqtt_timer_reset(T_PING);
                                shutdown_reset_watchdog();
                                ping_n = 0;
                                comm_serial_delete(2);
//                              LOGLIVE(LEVEL_1, "PINGRESP:%d", (int)Tick_Get( SECONDS ));
                                rtc_system_SetServerTime((int)Tick_Get( SECONDS ));
//                                battery_voltage_meas();
//                                Alive(WD_COMM);
                                status = MQTT_SELECT_LOOP;
                            }
                            break;
                        default:
                        	comm_serial_delete(comm_serial_rcx_bytes_n());
//                            Alive(WD_COMM);
                            status = MQTT_SELECT_LOOP;
                            break;
                    }
                }
                mqtt_timer_set( T_KEEPALIVE );
            }
            else
            {
            	status = TCP_DISCONNECTED;
            }
            break;
        case MQTT_RECEIVE_NEW_FIRMWARE:
#if 0
            if( udp_is_socket_open() )
            {
                if ( 0 == fw_update_FSM( 0, &fw_length ) )
                {
                    if( param.version.crc )
                    {
                        if( msg.qos == QOS1 ) {
                        	mqtt_buffer_ACKs_write( PUBACK, msg.id );
                        }
                        else if( msg.qos == QOS2 ){
                        	mqtt_buffer_ACKs_write( PUBREC, msg.id );
                        }
                        mqtt_timer_set( T_UPDATE_READY );
//                        Alive( WD_COMM );
                        status = MQTT_SELECT_LOOP;
                    }
                    else {
                        time_mqtt_protocol = 2;
                        Tick_update_tick( TICK_UDPPROT );
                    	status = TCP_DISCONNECT;
                    }
                    mqtt_set_updating_firmware(0);
                }
            }
            else {
//            	fw_update_FSM( app_data, 1, &fw_length );
            	status = TCP_DISCONNECTED;
            }
#endif
            break;
/*        case MQTT_CLEAN:
            if( TCPIsConnected_3G() )
            {
                if( ! mqtt_clean_FSM() ) { t = TickGet(); status = TCP_DISCONNECT; }
            }
            else status = TCP_DISCONNECTED;
            break;*/
        case TCP_DISCONNECT:
        default:
        	if ( CHECK_ELAPSED_TIME( time_mqtt_protocol, TICK_UDPPROT ) ) {
                if ( param.version.crc ) {
//                	SoftReset();
                }
                else {
//                	uint32_t sys_idle_disc = __checkSysIdle();
//                	if ( 0 == sys_idle_disc ) {
                		mqtt_start_frames = 0;
                		mqtt_set_desconnection_time( Tick_Get( SECONDS ) );
                		Telit_socket_quick_close_and_shutdown();
                		mqtt_timer_set_idle_mode(2);
                		status = MQTT_IDLE;//TCP_DISCONNECTED;
                		LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> DISCONNECT.\r\n", (int)Tick_Get( SECONDS ));
//                	}
                }
            }
            break;
    }
}

// CONNECT /////////////////////////////////////////////////////////////////////

static int32_t mqtt_set_connection_options()
{
    char will_topic[128];
    char will_payload[256];
    char pass[36];

    MQTTPacket_connectData opt = MQTTPacket_connectData_initializer;

    opt.MQTTVersion = 4;

    if ( param.registered ) {
    	sprintf( MyID, "%d",       (int)param.id.device );
        sprintf( MyGROUP, "%d",    (int)param.id.group );
        sprintf( MyPLATFORM, "%d", (int)param.id.platform );
        sprintf( will_topic, "/device/%s/%s/ans/connection", MyPLATFORM, MyID );
        sprintf( will_payload, "{\"status\":\"offline\",\"APN\":\"%s\",\"ip\":\"%s\",\"fw\":\"%02d.%02d\"}",
	        param.APN[ME910_APN_get_new_index()].name, "NULL", (int)param.version.major, (int)param.version.minor );
    }
    else {
        sprintf( MyID, "%s", ME910_IMEI() );
        sprintf( will_topic, "/hwID/%s/connection", MyID );
        sprintf( will_payload, "%s", "offline" );
    }

    opt.username.cstring = ME910_IMEI();//"353081091135940";//IMEI Hardcoded
    opt.clientID.cstring = ME910_IMEI();//"353081091135940";//IMEI Hardcoded
    sprintf( pass, "pass%s", /*"353081091135940"*/ME910_IMEI() );//IMEI Hardcoded
    opt.password.cstring = pass;
    opt.cleansession = 1;
    opt.keepAliveInterval = param.server[ (uint8_t)mqtt_server_get_index() ].keepalive;
    opt.willFlag = 1;
    opt.will.topicName.cstring = will_topic;
    opt.will.message.cstring = (char *) will_payload;
    opt.will.qos = 1;
    opt.will.retained = 1;//0;

    return MQTTSerialize_connect( (unsigned char *)app_data, sizeof(app_data), &opt );
}

// SUBSCRIBE ///////////////////////////////////////////////////////////////////

static int32_t mqtt_set_subscription_options( uint8_t hwID_device_FLAG )
{
    MQTTString topic[MAX_MESSAGE_HANDLERS] = { 0 };

    if ( hwID_device_FLAG ) {
        sprintf( &subscription_topic[0][0], "/device/%s/%s/set/#", MyPLATFORM, MyID );
        topic[0].cstring = (char *) &subscription_topic[0][0];
        subscription_qos[0] = QOS1;

//    	sprintf( &subscription_topic[0][0], "/device/%s/%s/set/time", MyPLATFORM, MyID );
//        topic[0].cstring = (char *) &subscription_topic[0][0];
//        subscription_qos[0] = QOS1;
//
//        sprintf( &subscription_topic[1][0], "/device/%s/%s/set/config/device_config", MyPLATFORM, MyID );
//        topic[1].cstring = (char *) &subscription_topic[1][0];
//        subscription_qos[1] = QOS1;

        sprintf( &subscription_topic[1][0], "/device/%s/%s/get/#", MyPLATFORM, MyID );
        topic[1].cstring = (char *) &subscription_topic[1][0];
        subscription_qos[1] = QOS1;

        sprintf( &subscription_topic[2][0], "/group/%s/%s/set/#", MyPLATFORM, MyGROUP );
        topic[2].cstring = (char *) &subscription_topic[2][0];
        subscription_qos[2] = QOS1;
        sprintf( &subscription_topic[3][0], "/group/%s/%s/get/#", MyPLATFORM, MyGROUP );
        topic[3].cstring = (char *) &subscription_topic[3][0];
        subscription_qos[3] = QOS1;

//        sprintf( &subscription_topic[3][0], "/group/%s/%s/set/time", MyPLATFORM, MyGROUP );
//        topic[3].cstring = (char *) &subscription_topic[3][0];
//        subscription_qos[3] = QOS1;
//
//        sprintf( &subscription_topic[4][0], "/group/%s/%s/set/config/device_config", MyPLATFORM, MyGROUP );
//        topic[4].cstring = (char *) &subscription_topic[4][0];
//        subscription_qos[4] = QOS1;

//        sprintf( &subscription_topic[5][0], "/group/%s/%s/get/#", MyPLATFORM, MyGROUP );
//        topic[5].cstring = (char *) &subscription_topic[5][0];
//        subscription_qos[5] = QOS1;

//    	sprintf( &subscription_topic[6][0], "/device/%s/%s/set/registers", MyPLATFORM, MyID );
//        topic[6].cstring = (char *) &subscription_topic[6][0];
//        subscription_qos[6] = QOS1;
//
//        sprintf( &subscription_topic[7][0], "/group/%s/%s/set/registers", MyPLATFORM, MyGROUP );
//        topic[7].cstring = (char *) &subscription_topic[7][0];
//        subscription_qos[7] = QOS1;

        return MQTTSerialize_subscribe( (unsigned char *)app_data, MQTT_BUFFER_SIZE, 0, 1, 4, topic, subscription_qos );
//        return MQTTSerialize_subscribe( (unsigned char *)app_data, MQTT_BUFFER_SIZE, 0, 1, 6, topic, subscription_qos );
    } else {
        sprintf( &subscription_topic[0][0], "/hwID/%s/test/run", MyID );
        topic[0].cstring = (char *) &subscription_topic[0][0];
        subscription_qos[0] = QOS1;
        sprintf( &subscription_topic[1][0], "/hwID/%s/register", MyID );
        topic[1].cstring = (char *) &subscription_topic[1][0];
        subscription_qos[1] = QOS1;

        return MQTTSerialize_subscribe( (unsigned char *)app_data, MQTT_BUFFER_SIZE, 0, 1, 2, topic, subscription_qos );
    }
}

// RECEIVE /////////////////////////////////////////////////////////////////////

static uint8_t deliverMessage( MQTTString *topic, MQTTMessage *msg, uint32_t rcv_len )
{
// 0 - Incompleto, 1 - Completo, 2 - No cabe y fw update, 3 - No cabe y basura
    uint8_t  argc = 0, i, r = 1;
    uint32_t len;
    char    *argv[10], str[64], *ptr;

    len = (uint8_t *) msg->payload - (uint8_t *) app_data + msg->payloadlen;

// parse topic
    if( topic->lenstring.len < sizeof(str) ) {
        memcpy( str, topic->lenstring.data, topic->lenstring.len );
        str[ topic->lenstring.len ] = '\0';

        ptr = strtok( str, "/" );
        if( ptr != NULL ) {
            //ptr = strtok( NULL, "/" );
            for( i = 0; i < 10; i++ ) {
                if( ptr != NULL ) {
                	argv[i] = ptr;
                	argc++;
                } else {
                	break;
                }
                ptr = strtok( NULL, "/" );
            }
        }
    } else {
    	goto CHAO;
    }

// test lengths
    if( len <= MQTT_BUFFER_SIZE ) { // Cabe en buffer
        if( rcv_len < len ) {
        	r = 0; // No esta completo
        } else {
        	r = 1;
        }
    } else {
        if( ! strcmp( argv[0], "device" ) && ! strcmp( argv[1], MyPLATFORM ) && ! strcmp( argv[2], MyID ) &&
            ! strcmp( argv[3], "set" )    && ! strcmp( argv[4], "update" )   && param.registered ) {
        	r = 2;
        } else {
            r = 3;
        }
    }
    if ( r != 1 ) {
    	goto CHAO;
    }

// process payload
    if ( param.registered ) {
        // "device/%s/%s/set/#"
        if ( ( argc >= 5 ) &&
             (( ( ! strcmp( argv[0], "device" ) && ! strcmp( argv[1], MyPLATFORM ) && ! strcmp( argv[2], MyID )) ) ||
              ( ( ! strcmp( argv[0], "group"  ) && ! strcmp( argv[1], MyPLATFORM ) && ! strcmp( argv[2], MyGROUP )))
			 )
		   ) {
            if ( ! strcmp( argv[3], "set" ) ) {
                if ( ! strcmp( argv[4], "status" )) {
                    json_mqtt_decode( PT_STATUS, msg->payload, msg->payloadlen );
                } else if ( ! strcmp( argv[4], "config" ) ) {
                	if ( ! strcmp( argv[5], "device_config" ) ) {
                		json_mqtt_decode( PT_FRAME_P_NETWORK_PARAMS, msg->payload, msg->payloadlen );
                	}
                } else if ( ! strcmp( argv[4], "time" ) ) {
            		json_mqtt_decode( PT_CONFIG_TIME, msg->payload, msg->payloadlen );
            		mqtt_start_frames = 1;
            	} else if ( ! strcmp( argv[4], "registers" ) ) {
            		json_mqtt_decode( PT_DEV_REGISTERS, msg->payload, msg->payloadlen );
            		mqtt_send_activation = 1;
            	} else if ( ! strcmp( argv[4], "command" ) ) {
                	if ( ! strcmp( argv[5], "energyprofile" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_ENERGY_REGISTERS, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "instvalues" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "maxdemand" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "loadprofile1" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_LOAD_PROFILE_1, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "loadprofile2" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_LOAD_PROFILE_2, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "powerquality" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "instrumentationprofile" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "billingprofile" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_BILLING_PROFILE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "events" ) ) {
                		json_mqtt_decode( PT_COMMAND_ON_DEMAND_EVENT_PROFILE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "synchronization" ) ) {
                	    json_mqtt_decode( PT_COMMAND_METER_SYNCHRONIZE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "clocktime" ) ) {
                		json_mqtt_decode( PT_COMMAND_READ_TIME_CLOCK, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getprofileinterval" ) ) {
                		json_mqtt_decode( PT_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setprofileinterval" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "switchstatus" ) ) {
                	    json_mqtt_decode( PT_COMMAND_GET_SWITCH_STATUS, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "reconnection" ) ) {
                		json_mqtt_decode( PT_COMMAND_RECONNECTION, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "mdieobreset" ) ) {
                		json_mqtt_decode( PT_COMMAND_MAXIMUM_DEMAND_RESET, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "loadlimit" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_LOAD_LIMITATION, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "readloadlimit" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_LOAD_LIMIT_THRESHOLD, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setbillingDate" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_BILLING_DATE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getbillingDate" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_BILLING_DATE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "clearalarms" ) ) {
                		json_mqtt_decode( PT_COMMAND_CLEAR_ALARMS, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setdemandInterval" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_DEMAND_INTEGRATION_PERIOD, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getdemandInterval" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_DEMAND_INTEGRATION_PERIOD, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setPaymentmode" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_PAYMENT_MODE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "readpaymentmode" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_PAYMENT_MODE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setMeteringmode" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_METERING_MODE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getmeteringmode" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_METERING_MODE, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "meterstatus" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_METER_STATUS, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setvoltrangeLow" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_VOLT_RANGE_LOW, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "setvoltrangeUp" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_VOLT_RANGE_UP, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getvoltrangeLow" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_VOLT_RANGE_LOW, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getvoltrangeup" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_VOLT_RANGE_UP, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "currentrangeLow" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_CURRENT_RANGE_LOW, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "currentrangeUp" ) ) {
                		json_mqtt_decode( PT_COMMAND_SET_CURRENT_RANGE_UP, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getcurrentrangelow" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_CURRENT_RANGE_LOW, msg->payload, msg->payloadlen );
                	}
                	else if ( ! strcmp( argv[5], "getcurrentrangeup" ) ) {
                		json_mqtt_decode( PT_COMMAND_GET_CURRENT_RANGE_UP, msg->payload, msg->payloadlen );
                	}
                   	else if ( ! strcmp( argv[5], "meternameplate" ) ) {
                    	json_mqtt_decode( PT_COMMAND_READ_METER_PLATE, msg->payload, msg->payloadlen );
                    }
                   	else if ( ! strcmp( argv[5], "getactivetariff" ) ) {
                    	json_mqtt_decode( PT_COMMAND_GET_CURRENT_ACTIVE_TARIFF, msg->payload, msg->payloadlen );
                    }
                   	else if ( ! strcmp( argv[5], "meterping" ) ) {
                    	json_mqtt_decode( PT_COMMAND_METER_PING, msg->payload, msg->payloadlen );
                    }
                   	else if ( ! strcmp( argv[5], "waterprofile" ) ) {
                   		json_mqtt_decode( PT_COMMAND_WATERPROFILE, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "toutariff" ) ) {
                   		json_mqtt_decode( PT_COMMAND_SET_TARIFF_AGREEMENT, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "getgwclocktime" ) ) {
                   		json_mqtt_decode( PT_COMMAND_GET_GW_TIME, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "gwsynchronization" ) ) {
                   		json_mqtt_decode( PT_COMMAND_SET_GW_SYNCH, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "setgwinterval" ) ) {
                   		json_mqtt_decode( PT_COMMAND_SET_GW_INTERVAL, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "getgwinterval" ) ) {
                   		json_mqtt_decode( PT_COMMAND_GET_GW_INTERVAL, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "gwnameplate" ) ) {
                   		json_mqtt_decode( PT_COMMAND_GET_GW_NAMEPLATE, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "ping" ) ) {
                   		json_mqtt_decode( PT_COMMAND_GW_PING, msg->payload, msg->payloadlen );
                   	}
                   	else if ( ! strcmp( argv[5], "gwfirmwareupdate" ) ) {
                   		json_mqtt_decode( PT_COMMAND_GW_FIRMWARE_UPDATE, msg->payload, msg->payloadlen );
                   	}
            	}
            } else if ( ! strcmp( argv[3], "get" ) ) {
                if ( ! strcmp( argv[4], "status" ) ) {
//                	mqtt_buffer_write( PT_STATUS );
//                	mqtt_set_reply_get_topic(1);
                } else if ( ! strcmp( argv[4], "config" ) ) {
                    if ( ! strcmp( argv[5], "device_params" ) ) {
//                    	mqtt_buffer_write( PT_FRAME_NETWORK_PARAMS );
//                    	mqtt_set_reply_get_topic(1);
                    }
                } else if ( ! strcmp( argv[4], "registers" ) ) {
                	mqtt_buffer_write( PT_REGISTER );
                	mqtt_set_reply_get_topic(1);
                	LOGLIVE(LEVEL_1, "LOGLIVE> %d MQTT_TASK> send get/registers.\r\n", (int)Tick_Get( SECONDS ));
                }  else if ( ! strcmp( argv[4], "update" ) ) {
                	mqtt_task_get_firmware();
//                	LOGLIVE(LEVEL_1, "get/firmware %d", (int)Tick_Get( SECONDS ));
                }
            }
        }
    } else {
        // run test
        if ( ! strncmp( &subscription_topic[0][0], topic->lenstring.data, topic->lenstring.len ) ) {
            if ( ! strncmp( (char *) msg->payload, "1", msg->payloadlen ) ) {
            	mqtt_buffer_write( PT_TEST_RESULT );
            }
        }
        // register
        else if ( ! strncmp( &subscription_topic[1][0], topic->lenstring.data, topic->lenstring.len )) {
            // payload: {"idplatform":1, "idgroup":2, "iddevice":1}
            char *ptr;
            uint16_t platform, group;

            ptr = strstr( (char *) msg->payload, "idplatform" );
            if ( ptr != NULL ) {
                if ( isdigit( (unsigned char)ptr[12] )) {
                    platform = atoi( &ptr[12] );
                    ptr = strstr( (char *) msg->payload, "idgroup" );
                    if ( ptr != NULL ) {
                        if ( isdigit( (unsigned char)ptr[9] )) {
                            group = atoi( &ptr[9] );
                            ptr = strstr( (char *) msg->payload, "iddevice" );
                            if ( ptr != NULL ) {
                                if ( isdigit( (unsigned char)ptr[10] )) {
                                    param.id.platform = platform;
                                    param.id.group = group;
                                    param.id.device = atoi( &ptr[10] );
                                    param.registered = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    comm_serial_delete( len );
CHAO:
    return r;
}

// PUBLISH /////////////////////////////////////////////////////////////////////

static int8_t mqtt_publish_topic( uint8_t topicc_n )
{
	char *str;
	int8_t qos = 1;

	str = (char *) &app_data[5]; // pq header + length < 5

	switch( topicc_n )
	{
		case PT_CONNECTION:
			sprintf( str, "/device/%s/%s/ans/connection", MyPLATFORM, MyID );
			break;
		case PT_REGISTER:
			sprintf( str, "/device/%s/%s/ans/registers", MyPLATFORM, MyID );
			break;
		case PT_STATUS:
			sprintf( str, "/device/%s/%s/ans/status", MyPLATFORM, MyID );
			break;
		case PT_FRAME_P_NETWORK_PARAMS:
			sprintf( str, "/device/%s/%s/ans/frame/p", MyPLATFORM, MyID );
			break;
		case PT_FRAME_F:
			sprintf( str, "/device/%s/%s/ans/frame/f", MyPLATFORM, MyID );
			break;
		case PT_FRAME_MF:
			sprintf( str, "/device/%s/%s/ans/frame/mf", MyPLATFORM, MyID );
			break;
		case PT_FRAME_S:
			sprintf( str, "/device/%s/%s/ans/frame/s", MyPLATFORM, MyID );
			break;
		case PT_FRAME_S_PLUS:
			sprintf( str, "/device/%s/%s/ans/frame/s+", MyPLATFORM, MyID );
			break;
		case PT_TEST_CONNECTION:
			sprintf( str, "/hwID/%s/connection", MyID );
			break;
		case PT_TEST_RESULT:
			sprintf( str, "/hwID/%s/test/result", MyID );
			break;
		default:
			qos = -1;
			break;
	}

	return qos;
}

#endif
