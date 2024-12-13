/*
 * mbus_protocol_aux.h
 *
 *  Created on: 29 dic. 2018
 *      Author: smillan
 */

#ifndef MBUS_PROTOCOL_AUX_H_
#define MBUS_PROTOCOL_AUX_H_

#include "mbus_protocol.h"

#define MBUS_BAUDRATE_2400 (0)
#define MBUS_BAUDRATE_300  (1)

#define MBUS_PROBE_NOTHING   0
#define MBUS_PROBE_SINGLE    1
#define MBUS_PROBE_COLLISION 2
#define MBUS_PROBE_ERROR     -1

#define MBUS_FRAME_PURGE_S2M  2
#define MBUS_FRAME_PURGE_M2S  1
#define MBUS_FRAME_PURGE_NONE 0

int32_t mbus_vif_unit_normalize(int vif, double value, char **unit_out, double *value_out, char **quantity_out);
int32_t mbus_recv_frame(mbus_frame *frame);
int32_t mbus_purge_frames( void );
int32_t mbus_send_frame(mbus_frame *frame);
int32_t mbus_send_application_reset_frame(int address, int subcode);
int32_t mbus_send_change_long_telegram(char * secondary_address, int subcode);
int32_t mbus_send_application_reset_frame_for_billing(int address, int subcode, int subcode2);
int32_t mbus_send_info_data_send_frame(int address, int dif, int dife, int vif, uint8_t *data, uint32_t num_data);
int32_t mbus_send_change_parametrise(char * secondary_address, int dif, int dife, int vif, uint8_t *data, uint32_t num_data);
int32_t mbus_send_read_billing(char * secondary_address, int dif,int vif);
int32_t mbus_send_data_request_secondary_address( void );
int32_t mbus_send_request_frame(int address);
int32_t mbus_send_ping_frame(int address);
int32_t mbus_send_select_frame(const char *secondary_addr_str);
int32_t mbus_select_secondary_address(const char *mask);
int32_t mbus_probe_secondary_address(const char *mask, char *matching_addr);
//int32_t mbus_send_info_data_send_frame(int address, int dif, int dife, int vif, uint8_t *data, uint32_t num_data);
//int32_t mbus_send_change_parametrise(char * secondary_address, int dif, int dife, int vif, uint8_t *data, uint32_t num_data);
int32_t mbus_scan_2nd_address_range(int32_t pos, char *addr_mask);

#endif /* MBUS_PROTOCOL_AUX_H_ */
