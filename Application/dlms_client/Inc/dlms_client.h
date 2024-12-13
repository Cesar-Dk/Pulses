/*
 * dlms_client.h
 *
 *  Created on: 20 may. 2021
 *      Author: Sergio Millán López
 */

#ifndef DLMS_CLIENT_INC_DLMS_CLIENT_H_
#define DLMS_CLIENT_INC_DLMS_CLIENT_H_

#include <stdint.h>
#include <math.h>

#include "connection.h"
#include "communication.h"
#include "circular_buffer.h"
#include "params.h"

#include "message_queue.h"

#define DLMS_SLAVES_TABLE_INIT_ADDRESS  (0x303000)//(0x302000)//(0x402000)//En 0x302000 almacenamos dlms_count
#define DLMS_SLAVES_TABLE_SIZE          (0x1C2000)
#define DLMS_SLAVES_TABLE_END_ADDRESS   (DLMS_SLAVES_TABLE_INIT_ADDRESS + DLMS_SLAVES_TABLE_SIZE)//(0x622000)

#define DLMS_INSTANTANEOUS_PROFILE_NUM  (4)
#define DLMS_ENERGY_PROFILE_NUM 		(3)
#define DLMS_MAX_DEMAND_PROFILE_NUM 	(1)
#define DLMS_OBIS_PROFILE_NUM			(DLMS_INSTANTANEOUS_PROFILE_NUM + DLMS_ENERGY_PROFILE_NUM + DLMS_MAX_DEMAND_PROFILE_NUM)
#define DLMS_GENERIC_PROFILE_NUM    	(6)
#define DLMS_PROFILE_NUM 				(DLMS_OBIS_PROFILE_NUM) + (DLMS_GENERIC_PROFILE_NUM)
#define DLMS_MAX_DEMAND_PROFILE_INDEX   (7)
#define DLMS_BILLING_PROFILE_INDEX      (12)
#define DLMS_EVENT_PROFILE_INDEX        (13)

typedef struct __dlms_client_t   			     *dlms_client_t_Ptr;
typedef struct __dlms_client_t    			      dlms_client_t;
typedef struct __dlms_client_obis_profile_t      *dlms_client_obis_profile_t_Ptr;
typedef struct __dlms_client_obis_profile_t    	  dlms_client_obis_profile_t;
typedef struct __dlms_client_generic_profile_t   *dlms_client_generic_profile_t_Ptr;
typedef struct __dlms_client_generic_profile_t    dlms_client_generic_profile_t;

/** DLMS communication functional states */
typedef enum {
	DLMS_COMM_DISABLED = 0,
	DLMS_COMM_STARTED  = 1,
	DLMS_COMM_END      = 2,
	DLMS_COMM_WRITE    = 3
} dlms_comm_state_t;

#define DLMS_OBIS_MAX_NUM_VALUES (20)

typedef enum {
	DLMS_HES_COMMAND_NONE = 0,
	DLMS_HES_COMMAND_ON_DEMAND_ENERGY_REGISTERS = 1,
	DLMS_HES_COMMAND_ON_DEMAND_MAX_DEMAND_REGISTERS,
	DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_1,
	DLMS_HES_COMMAND_ON_DEMAND_LOAD_PROFILE_2,
	DLMS_HES_COMMAND_ON_DEMAND_POWER_QUALITY_PROFILE,
	DLMS_HES_COMMAND_ON_DEMAND_INSTRUMENTATION_PROFILE,
	DLMS_HES_COMMAND_ON_DEMAND_INSTANTANEOUS_VALUES,
	DLMS_HES_COMMAND_ON_DEMAND_BILLING_PROFILE,
	DLMS_HES_COMMAND_ON_DEMAND_EVENTS_PROFILE,
	DLMS_HES_COMMAND_READ_TIME_CLOCK,
	DLMS_HES_COMMAND_READ_LOAD_PROFILE_CAPTURE_PERIOD,
	DLMS_HES_COMMAND_SET_LOAD_PROFILE_CAPTURE_PERIOD,
	DLMS_HES_COMMAND_GET_SWITCH_STATUS,
	DLMS_HES_COMMAND_CUTTOFF_RECONNECTION,
	DLMS_HES_COMMAND_MAXIMUM_DEMAND_RESET,
	DLMS_HES_COMMAND_SET_LOAD_LIMITATION,
	DLMS_HES_COMMAND_GET_LOAD_LIMIT_THRESHOLD,
	DLMS_HES_COMMAND_SET_BILLING_DATE,
	DLMS_HES_COMMAND_GET_BILLING_DATE,
	DLMS_HES_COMMAND_CLEAR_ALARMS,
	DLMS_HES_COMMAND_SET_DEMAND_INTEGRATION_PERIOD,
	DLMS_HES_COMMAND_GET_DEMAND_INTEGRATION_PERIOD,
	DLMS_HES_COMMAND_SET_PAYMENT_MODE,
	DLMS_HES_COMMAND_GET_PAYMENT_MODE,
	DLMS_HES_COMMAND_SET_METERING_MODE,
	DLMS_HES_COMMAND_GET_METERING_MODE,
	DLMS_HES_COMMAND_SET_VOLT_RANGE_LOW,
	DLMS_HES_COMMAND_SET_VOLT_RANGE_UP,
	DLMS_HES_COMMAND_GET_VOLT_RANGE_LOW,
	DLMS_HES_COMMAND_GET_VOLT_RANGE_UP,
	DLMS_HES_COMMAND_SET_CURRENT_RANGE_LOW,
	DLMS_HES_COMMAND_SET_CURRENT_RANGE_UP,
	DLMS_HES_COMMAND_GET_CURRENT_RANGE_LOW,
	DLMS_HES_COMMAND_GET_CURRENT_RANGE_UP,
	DLMS_HES_COMMAND_GET_METER_STATUS,
	DLMS_HES_COMMAND_READ_METER_NAMEPLATE,
	DLMS_HES_COMMAND_METER_SYNCHRONIZE,
	DLMS_HES_COMMAND_RECONNECTION,
	DLMS_HES_COMMAND_GET_ACTIVE_TARIFF,
	DLMS_HES_COMMAND_METER_PING,
	DLMS_HES_COMMAND_METER_SET_TARIFF_AGREEMENT,
	DLMS_HES_COMMAND_GET_GW_TIME,
	DLMS_HES_COMMAND_SET_GW_SYNCH,
	DLMS_HES_COMMAND_SET_GW_INTERVAL,
	DLMS_HES_COMMAND_GET_GW_INTERVAL,
	DLMS_HES_COMMAND_GET_GW_NAMEPLATE,
	DLMS_HES_COMMAND_GW_PING,
	DLMS_HES_COMMAND_GW_FIRMWARE_UPDATE,
	DLMS_HES_COMMAND_LAST
} dlms_client_command_t;

typedef enum {
	DLMS_WRITE_INST_PROF_1 = 0,
	DLMS_WRITE_INST_PROF_2,
	DLMS_WRITE_INST_PROF_3,
	DLMS_WRITE_INST_PROF_4,
	DLMS_WRITE_ENERGY_PROF_1,
	DLMS_WRITE_ENERGY_PROF_2,
	DLMS_WRITE_ENERGY_PROF_3,
	DLMS_WRITE_MAX_DEMAND_PROF_1,
	DLMS_WRITE_LOAD_PROF_1,
	DLMS_WRITE_LOAD_PROF_2,
	DLMS_WRITE_POWER_QUALITY_PROF,
	DLMS_WRITE_INSTRUMENTATION_PROF_1,
	DLMS_WRITE_BILLING_PROF,
	DLMS_WRITE_EVENT_LOG_PROF,
	DLMS_WRITE_NONE = 0xFF
} write_task_t;

typedef enum {
	DLMS_SEND_INST_PROF_1 = 0,
	DLMS_SEND_INST_PROF_2,
	DLMS_SEND_INST_PROF_3,
	DLMS_SEND_INST_PROF_4,
	DLMS_SEND_ENERGY_PROF_1,
	DLMS_SEND_ENERGY_PROF_2,
	DLMS_SEND_ENERGY_PROF_3,
	DLMS_SEND_MAX_DEMAND_PROF_1,
	DLMS_SEND_LOAD_PROF_1,
	DLMS_SEND_LOAD_PROF_2,
	DLMS_SEND_POWER_QUALITY_PROF,
	DLMS_SEND_INSTRUMENTATION_PROF_1,
	DLMS_SEND_BILLING_PROF,
	DLMS_SEND_EVENT_LOG_PROF,
	DLMS_SEND_DATA_CMD,

	DLMS_WAIT_FOR_SEND_INST_PROF_1 = 20,
	DLMS_WAIT_FOR_SEND_INST_PROF_2,
	DLMS_WAIT_FOR_SEND_INST_PROF_3,
	DLMS_WAIT_FOR_SEND_INST_PROF_4,
	DLMS_WAIT_FOR_SEND_ENERGY_PROF_1,
	DLMS_WAIT_FOR_SEND_ENERGY_PROF_2,
	DLMS_WAIT_FOR_SEND_ENERGY_PROF_3,
	DLMS_WAIT_FOR_SEND_MAX_DEMAND_PROF_1,
	DLMS_WAIT_FOR_SEND_LOAD_PROF_1,
	DLMS_WAIT_FOR_SEND_LOAD_PROF_2,
	DLMS_WAIT_FOR_SEND_POWER_QUALITY_PROF,
	DLMS_WAIT_FOR_SEND_INSTRUMENTATION_PROF_1,
	DLMS_WAIT_FOR_SEND_BILLING_PROF,
	DLMS_WAIT_FOR_SEND_EVENT_LOG_PROF,
	DLMS_WAIT_FOR_SEND_DATA_CMD,

	DLMS_SEND_NONE = 0xFF
} send_task_t;

typedef enum {
	DLMS_READ_INST_PROF_1 = 0,
	DLMS_READ_INST_PROF_2,
	DLMS_READ_INST_PROF_3,
	DLMS_READ_INST_PROF_4,
	DLMS_READ_ENERGY_PROF_1,
	DLMS_READ_ENERGY_PROF_2,
	DLMS_READ_ENERGY_PROF_3,
	DLMS_READ_MAX_DEMAND_PROF_1,
	DLMS_READ_LOAD_PROF_1,
	DLMS_READ_LOAD_PROF_2,
	DLMS_READ_POWER_QUALITY_PROF,
	DLMS_READ_INSTRUMENTATION_PROF_1,
	DLMS_READ_BILLING_PROF,
	DLMS_READ_EVENT_LOG_PROF,

	DLMS_READ_CMD_INST_PROF_1,
	DLMS_READ_CMD_INST_PROF_2,
	DLMS_READ_CMD_INST_PROF_3,
	DLMS_READ_CMD_INST_PROF_4,
	DLMS_READ_CMD_ENERGY_PROF_1,
	DLMS_READ_CMD_ENERGY_PROF_2,
	DLMS_READ_CMD_ENERGY_PROF_3,
	DLMS_READ_CMD_MAX_DEMAND_PROF_1,
	DLMS_READ_CMD_LOAD_PROF_1,
	DLMS_READ_CMD_LOAD_PROF_2,
	DLMS_READ_CMD_POWER_QUALITY_PROF,
	DLMS_READ_CMD_INSTRUMENTATION_PROF_1,
	DLMS_READ_CMD_BILLING_PROF,
	DLMS_READ_CMD_EVENT_LOG_PROF,
	DLMS_READ_DATA_CMD,

	DLMS_READ_NONE = 0xFF
} read_task_t;

typedef enum {
	DLMS_OBIS_INST_PROF_1 = 0,
	DLMS_OBIS_INST_PROF_2,
	DLMS_OBIS_INST_PROF_3,
	DLMS_OBIS_INST_PROF_4,
	DLMS_OBIS_ENERGY_PROF_1,
	DLMS_OBIS_ENERGY_PROF_2,
	DLMS_OBIS_ENERGY_PROF_3,
	DLMS_OBIS_MAX_DEMAND_PROF_1,
	DLMS_OBIS_LOAD_PROF_1,
	DLMS_OBIS_LOAD_PROF_2,
	DLMS_OBIS_POWER_QUALITY_PROF,
	DLMS_OBIS_INSTRUMENTATION_PROF_1,
	DLMS_OBIS_BILLING_PROF,
	DLMS_OBIS_EVENT_LOG_PROF,
} obis_profile_type_t;

uint32_t dlms_client_init(void);

CircularBuffer dlms_client_get_read_msg_queue(void);
CircularBuffer dlms_client_get_write_msg_queue(void);
CircularBuffer dlms_client_get_send_msg_queue(void);

uint32_t dlms_client_get_checksum_param( char * modbus_params );
void     dlms_client_set_checksum_param( char * _checksum_param );

uint32_t dlms_client_get_dlms_load_profile_read_time(void);
uint32_t dlms_client_get_dlms_load_profile_send_time(void);
uint32_t dlms_client_set_dlms_load_profile_read_time( uint32_t __read_time );
uint32_t dlms_client_set_dlms_load_profile_send_time( uint32_t __send_time );
uint32_t dlms_client_get_dlms_load_profile_get_wait_for_command(void);
uint32_t dlms_client_set_dlms_load_profile_set_wait_for_command( uint32_t __wait_for_comm );
uint32_t dlms_client_get_dlms_load_profile_get_from_command(void);
uint32_t dlms_client_set_dlms_load_profile_set_from_command( uint32_t __from_comm );
uint32_t dlms_client_upload_read_send_time_with_load_profile( void );
uint32_t dlms_client_get_dlms_load_profile_message_length(void);
uint32_t dlms_client_set_dlms_load_profile_message_length( uint32_t __message_length );
uint32_t dlms_client_get_dlms_load_profile_num_obis(void);
uint32_t dlms_client_set_dlms_load_profile_num_obis( uint32_t __num_obis );
char   * dlms_client_get_load_profile_obis_n( uint32_t n);
void     dlms_client_set_load_profile_obis_n(char *obis_n);
void     dlms_client_set_load_profile_obis_block(char *obis_n);
uint32_t dlms_client_get_dlms_load_profile_start_time(void);
uint32_t dlms_client_set_dlms_load_profile_start_time( uint32_t __start_time );
uint32_t dlms_client_reset_id_request_index(void);
uint32_t dlms_client_reset_id_request_queue(void);
uint32_t dlms_client_inc_id_request_index_rd(void);
char   * dlms_client_get_dlms_id_request(void);
uint32_t dlms_client_set_dlms_id_request( char * __id_request );
uint32_t dlms_client_clear_dlms_data_cmd_on_demand(void);
char   * dlms_client_get_dlms_data_cmd_on_demand(void);
uint32_t dlms_client_set_dlms_data_cmd_on_demand( char * __data_cmd_on_demand );
uint32_t dlms_client_get_dlms_load_profile_end_time(void);
uint32_t dlms_client_set_dlms_load_profile_end_time( uint32_t __end_time );
void     dlms_client_reset_load_profile_obis( void );

dlms_client_t 			      * dlms_client_get_client( void );
dlms_client_t 			      * dlms_client_get_client_backup( void );
uint32_t                        dlms_client_copy_read_send_times(dlms_client_t *client_backup, dlms_client_t *client);
dlms_client_obis_profile_t    * dlms_client_get_client_obis_profile( void );
dlms_client_generic_profile_t * dlms_client_get_client_generic_profile( void );
size_t   dlms_client_get_size( void );
size_t 	 dlms_client_obis_profile_get_size( void );
size_t 	 dlms_client_generic_profile_get_size( void );
uint32_t dlms_client_get_dlms_enable(void);
uint32_t dlms_client_get_dlms_num_obis_profiles(void);
uint32_t dlms_client_get_dlms_num_generic_profiles(void);
uint32_t dlms_client_get_dlms_comm_state(void);
void     dlms_client_set_dlms_comm_state(uint32_t _dlms_comm_state);
uint32_t dlms_client_get_read_time(void);
uint32_t dlms_client_get_read_meter(void);
uint32_t dlms_client_get_send_meter(void);
uint32_t dlms_client_get_send_time(void);
write_task_t dlms_client_get_frame_type_last_write(void);
void 		 dlms_client_set_frame_type_last_write(write_task_t _last_write);
uint32_t dlms_client_get_energy_profile_1(void);
void 	 dlms_client_set_energy_profile_1(uint32_t _profile);
uint32_t dlms_client_get_energy_profile_2(void);
void 	 dlms_client_set_energy_profile_2(uint32_t _profile);
uint32_t dlms_client_get_energy_profile_3(void);
void 	 dlms_client_set_energy_profile_3(uint32_t _profile);

uint32_t dlms_client_get_inst_profile_1(void);
void 	 dlms_client_set_inst_profile_1(uint32_t _profile);
uint32_t dlms_client_get_inst_profile_2(void);
void 	 dlms_client_set_inst_profile_2(uint32_t _profile);
uint32_t dlms_client_get_inst_profile_3(void);
void 	 dlms_client_set_inst_profile_3(uint32_t _profile);
uint32_t dlms_client_get_inst_profile_4(void);
void 	 dlms_client_set_inst_profile_4(uint32_t _profile);
void 	 dlms_client_set_dlms_reading_items(uint32_t dev_id, uint32_t _items);
void     dlms_client_inc_dlms_reading_items(uint32_t dev_id);
uint32_t dlms_client_get_dlms_reading_items(uint32_t dev_id);
uint32_t dlms_client_get_logical_address(void);
uint16_t dlms_client_get_cl_address(void);
uint32_t dlms_client_get_srv_address(void);
uint32_t dlms_client_get_baudrate(void);
void     dlms_client_get_authen(char * authen);
uint32_t dlms_client_get_pass_is_hex(void);
uint32_t dlms_client_get_pass_level(void);
uint32_t dlms_client_get_short_name_ref(void);
uint32_t dlms_client_get_billingday(void);
uint32_t dlms_client_get_num_devices(void);
void     dlms_client_set_pass_is_hex(uint32_t _is_hex);
void     dlms_client_set_pass_level(uint32_t _pass_level);
void     dlms_client_set_short_name_ref(uint32_t _short_name);
void     dlms_client_set_billingday(uint32_t _bill);
void     dlms_client_set_num_devices(uint32_t _num_dev);
void     dlms_client_get_pass_in_hex(char * authen);
uint32_t dlms_client_get_send_time_obis_n(uint32_t obis_n);
uint32_t dlms_client_get_read_time_obis_n(uint32_t obis_n);
uint32_t dlms_client_set_send_time_obis_n(uint32_t obis_n, uint32_t _time);
uint32_t dlms_client_set_read_time_obis_n(uint32_t obis_n, uint32_t _time);
char   * dlms_client_get_obis_n( uint32_t n);
char   * dlms_client_get_generic_obis_n( uint32_t n);
uint32_t dlms_client_get_obis_profile_read_time( void );
uint32_t dlms_client_get_obis_profile_send_time( void );
void 	 dlms_client_set_obis_profile_read_time( uint32_t _read_time );
void 	 dlms_client_set_obis_profile_send_time( uint32_t _send_time );
uint32_t dlms_client_get_obis_profile_frame_type( void );
uint32_t dlms_client_get_obis_profile_num_obis(void);
uint32_t dlms_client_set_obis_profile_num_obis( uint32_t __num_obis );
uint32_t dlms_client_get_obis_profile_from_command(void);
uint32_t dlms_client_set_obis_profile_from_command( uint32_t __obis_from_command );
void     dlms_client_set_obis_profile_max_demand_profile_extra( uint32_t _max_demand_profile_extra );
uint32_t dlms_client_get_max_demand_profile_extra_frame_type(uint32_t curr_device, uint32_t frame_type);
uint32_t dlms_client_get_obis_profile_max_demand_profile_extra(void);
uint32_t dlms_client_get_generic_profile_frame_type( void );
void 	 dlms_client_set_obis_profile_frame_type( uint32_t _frame_type );
void 	 dlms_client_set_generic_profile_frame_type( uint32_t _frame_type );
char   * dlms_client_get_meter_id( uint32_t device );
char   * dlms_client_get_obis_value_next(void);
uint32_t dlms_client_get_param_change( void );
uint32_t dlms_client_get_resend( void );
uint32_t dlms_client_get_disconnected( void );
uint32_t dlms_client_get_command( void );
uint32_t dlms_client_get_synch_time( void );
uint32_t dlms_client_get_read_energy_profile(void);
void     dlms_client_set_read_energy_profile(uint32_t _energy_profile);

void   dlms_client_set_dlms_enable(uint32_t _dlms_enable);
void   dlms_client_set_dlms_num_obis_profiles(uint32_t _num_obis_profiles);
void   dlms_client_set_dlms_num_generic_profiles(uint32_t _num_generic_profiles);
void   dlms_client_set_authen(char *authen);
void   dlms_client_set_read_time(uint32_t _read_time);
void   dlms_client_set_send_time(uint32_t _send_time);
void   dlms_client_set_read_meter(uint32_t _read_data);
void   dlms_client_set_send_meter(uint32_t _read_data);
void   dlms_client_set_logical_address(uint32_t _logical_address);
void   dlms_client_set_cl_address(uint32_t _cl_address);
void   dlms_client_set_srv_address(uint32_t _srv_address);
void   dlms_client_set_baudrate(uint32_t _baudrate);
void   dlms_client_set_obis_n(char *obis_n, uint32_t n);
void   dlms_client_set_generic_obis_n(char *obis_n, uint32_t n);
void   dlms_client_set_obis_value(char *obis_value, uint32_t n, unsigned char attributeOrdinal);
void   dlms_client_append_obis_value(char *obis_value, unsigned char attributeOrdinal);
void   dlms_client_dec_next_obis_val(void);
void   dlms_client_set_param_change(uint32_t _param_change);
void   dlms_client_set_resend(uint32_t _resend);
void   dlms_client_set_disconnected(uint32_t _disconnected);
void   dlms_client_set_command(uint32_t _command);
void   dlms_client_set_synch_time(uint32_t _synch);
void   dlms_client_reset_indexes( void);

char * dlms_client_get_params(char * modbus_params);
uint64_t dlms_client_on_demand_synch( time_t server_time );

char * dlms_client_get_raw_modbus( char *buff, uint32_t *total_chars );

size_t getGinUse(void);
size_t getFS_totalAllocated(void);
void * mmalloc(size_t size, uint8_t *file, uint32_t line);
void * mcalloc(size_t n, size_t size, uint8_t *file, uint32_t line);
void * mrealloc(void *p, size_t size, uint8_t *file, uint32_t line);
void   mfree(void *p, uint8_t *file, uint32_t line);
void   mfree_all(void);

#endif /* DLMS_CLIENT_INC_DLMS_CLIENT_H_ */
