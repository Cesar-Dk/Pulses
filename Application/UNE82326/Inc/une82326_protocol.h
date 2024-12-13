/*
 * une82326_protocol.h
 *
 *  Created on: 18 jul. 2019
 *      Author: smill
 */
#ifndef APPLICATION_UNE82326_INC_UNE82326_PROTOCOL_H_
#define APPLICATION_UNE82326_INC_UNE82326_PROTOCOL_H_

#include "stm32u5xx_hal.h"

#ifdef UNE82326
#define V_CHAR 		 (0x56)
#define TAB_CHAR 	 (0x09)
#define CR_CHAR		 (0x0D)
#define COMMA_CHAR   (0x2C)

#define C_FIELD_SIZE (4)
#define A_FRAME_SIZE (32)
#define APLUS_FRAME_MAX_SIZE (128)

#define FRAME_A_NUM_BYTES (32)

typedef enum {
	ERROR_A_FRAME_PARSE_OK = 33,//in A Frames we return remaining bytes, max 32.
	ERROR_A_FRAME_PARSE_CRC,
	ERROR_A_FRAME_PARSE_LENGTH,
	ERROR_A_FRAME_PARSE_TX_OK,
	ERROR_A_FRAME_PARSE_TX_CRC,
	ERROR_A_PLUS_FRAME_PARSE_OK,
	ERROR_A_PLUS_FRAME_PARSE_CRC,
	ERROR_A_PLUS_FRAME_PARSE_LENGTH,
	ERROR_A_PLUS_FRAME_PARSE_TX_OK,
	ERROR_A_PLUS_FRAME_PARSE_TX_CRC,
	ERROR_PARSE_UNKNOWN,
}error_frame_parse;

typedef enum {
	BAT_NO_BATT = 0,
	BAT_LESS_1_MONTHS,
	BAT_MORE_1_MONTHS,
	BAT_MORE_3_MONTHS,
}a_frame_bat;

typedef union _a_frame_status {
	struct {
		uint8_t dummy:1;
		uint8_t prefix:3;
		uint8_t fugas:1;
		uint8_t libre1:1;
		uint8_t libre2:1;
		uint8_t error:1;
	}bits;
    uint8_t byte;
}a_frame_status;

typedef struct _UNE82326_A_Frame {
	uint8_t 			REG[10];
	uint8_t 			IDA[8];
	a_frame_status  	STATUS;
	uint8_t 			UNIT;
	uint8_t 			TYPE[2];
	a_frame_bat 		BAT;
	uint8_t 			SIZE;
	uint8_t 			RES[4];
	uint8_t 			CRC16[4];
	uint8_t             NullChar;
}UNE82326_A_Frame;

typedef struct _UNE82326_A {
	UNE82326_A_Frame 	a_frame;
	uint32_t         	size;
}UNE82326_A;

typedef struct _UNE82326_Aplus_Frame {
	uint8_t             V_Field;
	uint8_t 			S_Field[18];
	uint8_t 			*R_Field;
	uint8_t 			X_Field[4];
	uint8_t 			A_Field[16];
	uint8_t 			*F_Field;
	uint8_t 			*N_Field;
	uint8_t 			*Q_Field;
	uint8_t 			*B_Field;
	uint8_t 			*J_Field;
	uint8_t 			CRC16[4];
	uint8_t             CR;
	uint8_t             NullChar;
}UNE82326_Aplus_Frame;

typedef struct _UNE82326_Aplus {
	UNE82326_Aplus_Frame *aplus_frame;
	uint32_t             size;
}UNE82326_Aplus;

typedef struct _UNE82326_Frame {
	UNE82326_A     		 a;
	UNE82326_Aplus 		 aplus;
}UNE82326_Frame;

int32_t   une82326_protocol_tx_a_plus_frame( uint8_t *data, uint32_t data_len );
char    * une82326_get_r_field_values( void );
uint32_t  une82326_get_num_r_field_values( void );
char    * une82326_get_crc_field_value( void );
uint32_t  une82326_decode_r_field( char *r_field_value );
uint32_t  une82326_add_r_field_value( char *r_field_value );
uint32_t  une82326_add_crc_field_value( char *crc_field_value );
uint32_t  une82326_add_r_field_time_value( void );
void      une828326_meter_frame_append_checksum( char *crc_field );
uint8_t * une82326_s_field( void );
uint8_t * une82326_r_field( void );
uint8_t * une82326_x_field( void );
uint8_t * une82326_a_field( void );
uint8_t * une82326_f_field( void );
uint8_t * une82326_n_field( void );
uint8_t * une82326_q_field( void );
uint8_t * une82326_b_field( void );
uint8_t * une82326_j_field( void );
uint8_t * une82326_crc_field(void);
uint8_t   une82326_b_frame_support( void );
int32_t   une82326_protocol_parse( UNE82326_Frame *frame, uint8_t *data, size_t data_size );
int32_t   une82326_protocol_parse_a_frame( UNE82326_A *frame, uint8_t *data );
int32_t   une82326_protocol_parse_aplus_frame( UNE82326_Aplus *frame, uint8_t *data );
void      une828326_a_plus_frame_append_checksum( char *crc_field );
void      une82326_build_a_plus_information_message( void );
char    * une82326_get_a_plus_information_message( void );

#endif

#endif /* APPLICATION_UNE82326_INC_UNE82326_PROTOCOL_H_ */
