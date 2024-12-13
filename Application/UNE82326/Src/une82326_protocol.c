/*
 * une82326_protocol.c
 *
 *  Created on: 18 jul. 2019
 *      Author: smill
 */
#define _GNU_SOURCE
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "common_lib.h"
#include "usart.h"
#include "une82326_protocol.h"
#include "serial_une82326.h"
#include "une82326_device_table.h"
#include "rtc_system.h"

#ifdef UNE82326
typedef struct {
	char     r_field_values[1024];
	char     crc[5];
	uint32_t num;
} r_field_string_values_st;

static UNE82326_Frame une82326_frame_st;
//static char une82326_a_frame[32];
//static char une82326_a_plus_frame[128];
static UNE82326_Aplus_Frame une82326_a_plus_frame_st;
//static char une82326_b_frame[128];
static r_field_string_values_st r_field_string_values;

//static int32_t __decodeAFrame( UNE82326_A_Frame *a_frame, size_t pos );
static int32_t __parseAFrame( UNE82326_Frame *frame, uint8_t *data, size_t data_size );
static int32_t __parseAplusFrame( UNE82326_Frame *frame, uint8_t *data, size_t data_size );
static int32_t __a_frame_verify( UNE82326_A_Frame *frame );
static int32_t __a_plus_frame_verify( char *data);
static int     __cz2_crc( char read_frame[], uint8_t frame_type );
static int     __cz2_append_crc( char read_frame[] );

#define A_TYPE     0
#define APLUS_TYPE 1
#define B_TYPE     2

static int __cz2_crc( char read_frame[], uint8_t frame_type )
{
	static unsigned short i, data;
	static unsigned int pos, crc_pos;
	static unsigned short accum, crc, genpoly = 0x1021;
	char read_byte;
	uint32_t last_crc_pos = 1;

	accum = 0;
	crc   = 0;
	pos   = 0;

	if ( frame_type == A_TYPE ) {
		crc_pos = 4;
	}else if ( frame_type == APLUS_TYPE ) {
		read_frame[0] = V_CHAR;
		crc_pos       = 6;
	}else if ( frame_type == B_TYPE ) {
		crc_pos = 5;
	}

	uint32_t len_frame = strlen(read_frame);
	if ( len_frame > crc_pos ) {
		while( pos != len_frame - crc_pos ) {
			data = read_frame[pos++] << 8;
			for ( i = 8; i > 0; i-- ) {
				if ( ( data ^ accum ) & 0x8000 ) {
					accum = ( accum << 1 ) ^ genpoly;
				} else {
					accum <<= 1;
				}
				data <<= 1;
			}
		}
		if ( frame_type == APLUS_TYPE ) {
			crc_pos = 5;
			last_crc_pos = 2;
		}
		for ( pos = crc_pos; pos >= last_crc_pos ; pos-- ) {
			crc <<= 4;
			read_byte = read_frame[ strlen(read_frame) - pos ];
			crc      |= read_byte & 0xF;
		}
	} else {
		accum = 1;
	}

	return ( accum == crc );
}

//A Frames.
static int32_t __a_frame_verify( UNE82326_A_Frame *frame )
{
	if ( frame != NULL ) {
		frame->NullChar = '\0';
		return __cz2_crc((char *)frame, A_TYPE);
	}
	return -1;
}

#if 0
static int32_t __decodeAFrameReg( UNE82326_A_Frame *a_frame, char * value )
{
	int32_t ret = 0;

	switch (a_frame->UNIT) {
	case 0:
		value[10] = a_frame->REG[9];
		value[9]  = a_frame->REG[8];
		value[8]  = a_frame->REG[7];
		value[7]  = a_frame->REG[6];
		value[6]  = a_frame->REG[5];
		value[5]  = a_frame->REG[4];
		value[4]  = '.';
		value[3]  = a_frame->REG[3];
		value[2]  = a_frame->REG[2];
		value[1]  = a_frame->REG[1];
		value[0]  = a_frame->REG[0];
		break;
	case 1:
		value[10] = a_frame->REG[9];
		value[9]  = a_frame->REG[8];
		value[8]  = a_frame->REG[7];
		value[7]  = a_frame->REG[6];
		value[6]  = a_frame->REG[5];
		value[5]  = a_frame->REG[4];
		value[4]  = a_frame->REG[3];
		value[3]  = '.';
		value[2]  = a_frame->REG[2];
		value[1]  = a_frame->REG[1];
		value[0]  = a_frame->REG[0];
		break;
	case 2:
		value[10] = a_frame->REG[9];
		value[9]  = a_frame->REG[8];
		value[8]  = a_frame->REG[7];
		value[7]  = a_frame->REG[6];
		value[6]  = a_frame->REG[5];
		value[5]  = a_frame->REG[4];
		value[4]  = a_frame->REG[3];
		value[3]  = a_frame->REG[2];
		value[2]  = '.';
		value[1]  = a_frame->REG[1];
		value[0]  = a_frame->REG[0];
		break;
	case 3:
		value[10] = a_frame->REG[9];
		value[9]  = a_frame->REG[8];
		value[8]  = a_frame->REG[7];
		value[7]  = a_frame->REG[6];
		value[6]  = a_frame->REG[5];
		value[5]  = a_frame->REG[4];
		value[4]  = a_frame->REG[3];
		value[3]  = a_frame->REG[2];
		value[2]  = a_frame->REG[1];
		value[1]  = '.';
		value[0]  = a_frame->REG[0];
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}
#endif

#if 0
static int32_t __decodeStatus( UNE82326_A_Frame *a_frame, char * status )
{
	if ( a_frame->STATUS.bits.fugas == 1) {
		snprintf(status, 5, "%s", "Fugas");
		status = status + 5;
	}
	if ( a_frame->STATUS.bits.error == 1) {
		snprintf(status, 5, "%s", "Error");
	}
	return 0;
}
#endif

#if 0
static int32_t __decodeAFrame( UNE82326_A_Frame *a_frame, size_t pos )
{
	char reg[11], status[11];

	memset( status, 0, sizeof(status) );
	memset( reg,    0, sizeof(reg) );

	if ( __decodeAFrameReg( a_frame, reg ) == -1 ) {
		return -1;
	}

	snprintf( &une82326_a_frame[pos], sizeof(a_frame->REG), "%s m3", reg);
	pos += strlen(reg);
	une82326_a_frame[pos++] ='|';
	snprintf( &une82326_a_frame[pos], sizeof(a_frame->IDA), "%s", a_frame->IDA);
	pos += sizeof(a_frame->IDA);
	une82326_a_frame[pos++] ='|';
	__decodeStatus(a_frame, status);
	snprintf( &une82326_a_frame[pos], strlen(status), "%s", status);
	pos += strlen(status);
	une82326_a_frame[pos++] ='|';
	snprintf( &une82326_a_frame[pos], sizeof(a_frame->UNIT), "%d", a_frame->UNIT);
	pos += sizeof(a_frame->UNIT);
	une82326_a_frame[pos++] ='|';
	snprintf( &une82326_a_frame[pos], sizeof(a_frame->TYPE), "%s", a_frame->TYPE);
	pos += sizeof(a_frame->TYPE);
	une82326_a_frame[pos++] ='|';
	snprintf( &une82326_a_frame[pos], sizeof(a_frame->BAT), "%d", a_frame->BAT);
	pos += sizeof(a_frame->BAT);
	une82326_a_frame[pos++] ='|';
	snprintf( &une82326_a_frame[pos], sizeof(a_frame->SIZE), "%d", a_frame->SIZE);
	pos += sizeof(a_frame->SIZE);
	une82326_a_frame[pos++] ='|';
	snprintf( &une82326_a_frame[pos], sizeof(a_frame->RES), "%s", a_frame->RES);
	pos += sizeof(a_frame->RES);
	une82326_a_frame[pos++] ='|';

	return 0;
}
#endif

static int32_t __parseAFrame(  UNE82326_Frame *frame, uint8_t *data, size_t data_size )
{
	size_t i, rem_bytes;

	if ( data_size < FRAME_A_NUM_BYTES ) {
		rem_bytes = FRAME_A_NUM_BYTES - data_size;
		return rem_bytes;
	}

	for ( i = 0; i < 10; i++) {
		frame->a.a_frame.REG[i] = data[i];
	}

	for ( i = 0; i < 7; i++) {
		frame->a.a_frame.IDA[i] = data[9+i];
	}

	frame->a.a_frame.STATUS.byte  = data[18];
	frame->a.a_frame.UNIT    	  = data[19];
	frame->a.a_frame.TYPE[0] 	  = data[20];
	frame->a.a_frame.TYPE[1] 	  = data[21];
	frame->a.a_frame.BAT     	  = data[22];
	frame->a.a_frame.SIZE    	  = data[23];
	frame->a.a_frame.RES[0]  	  = data[24];
	frame->a.a_frame.RES[1]  	  = data[25];
	frame->a.a_frame.RES[2]  	  = data[26];
	frame->a.a_frame.RES[3]  	  = data[27];

	if (__a_frame_verify(&frame->a.a_frame) != 1) {
		return ERROR_A_FRAME_PARSE_CRC;
	}

	return ERROR_A_FRAME_PARSE_OK;
}

//A+ Frames.
static int32_t __a_plus_frame_verify( char *data )
{
	if (( data[0] == V_CHAR ) || ( data[0] == COMMA_CHAR )) {
		return __cz2_crc(data, APLUS_TYPE);
	}
	return -1;
}

static int32_t __decodeNextField(UNE82326_Aplus_Frame *aplus_frame, char *next_field, uint32_t field_length)
{
	switch (next_field[0]) {
	case 'A':
		strncpy( (char *)aplus_frame->A_Field, next_field + 1, field_length );
		break;
	case 'F':
		aplus_frame->F_Field = calloc( field_length, sizeof(uint8_t) );
		strncpy( (char *)aplus_frame->F_Field, next_field + 1, field_length );
		break;
	case 'N':
		aplus_frame->N_Field = calloc( field_length, sizeof(uint8_t) );
		strncpy( (char *)aplus_frame->N_Field, next_field + 1, field_length );
		break;
	case 'Q':
		aplus_frame->Q_Field = calloc( field_length, sizeof(uint8_t) );
		strncpy( (char *)aplus_frame->Q_Field, next_field + 1, field_length );
		break;
	case 'B':
		aplus_frame->B_Field = calloc( field_length, sizeof(uint8_t) );
		strncpy( (char *)aplus_frame->B_Field, next_field + 1, field_length );
		break;
	case 'J':
		aplus_frame->J_Field = calloc( field_length, sizeof(uint8_t) );
		strncpy( (char *)aplus_frame->J_Field, next_field + 1, field_length );
		break;
	case 'C':
		strncpy( (char *)aplus_frame->CRC16, next_field + 1, 4 );
		break;
	default:
		asm("nop");
		break;
	}

	return 0;
}

static int32_t __parseAplusFrame(  UNE82326_Frame *frame, uint8_t *data, size_t data_size )
{
	char *s, *str_end;
	char *s_field, *r_field, *x_field, *next_field = NULL, *next_next_field = NULL;
	char tab[2], cr[2];
	uint32_t frame_length = 0, field_length = 0;

	s = (char *)data;

	sprintf( tab, "%c", TAB_CHAR );
	sprintf( cr,  "%c", CR_CHAR);

	// Calculate total length of A+ frame.
	str_end = (char *)memmem( (char *)s, data_size, cr, 1 );// Search for the end of the a+ frame.
	if ( str_end != NULL ) {
		frame_length = str_end - s; // Calculate total length of A+ frame.
		if (frame_length > APLUS_FRAME_MAX_SIZE) {
			return -1;
		}
	} else {
		return -1;
	}

	// Search for S field. Begins with 'S' char.
	s_field = (char *)memmem( (char *)s, frame_length, "S", 1 );
	s_field = strtok( s_field, tab );

	// Search for R field. Begins with 'R' char.
	r_field = strtok( NULL, tab );
	if (*r_field != 'R') {
		return -1;
	}
	field_length = ( r_field - s_field );
	strncpy( (char *)frame->aplus.aplus_frame->S_Field, s_field + 1, field_length );

	// Search for X field. Begins with 'X' char.
	x_field = strtok( NULL, tab );
	if (*x_field != 'X') {
		return -1;
	}
	field_length = ( x_field - r_field );
	frame->aplus.aplus_frame->R_Field = calloc(field_length, sizeof(uint8_t) );
	strncpy( (char *)frame->aplus.aplus_frame->R_Field, r_field + 1, field_length );

	// Search for optional fields: A, F, N, Q, B, J fields.
	uint8_t copy_x_field = 0;
	do {
		if ( ( NULL == next_field ) && ( 0 == copy_x_field ) ) {
			copy_x_field = 1;
		}
		next_field      = strtok( NULL, tab );
		if ( 1 == copy_x_field ) {
			field_length = ( next_field - x_field );
			strncpy( (char *)frame->aplus.aplus_frame->X_Field, x_field + 1, field_length );
			copy_x_field = 2;
		}
		if ( NULL != next_next_field ) {
			if ( NULL != next_field ) {
				__decodeNextField( frame->aplus.aplus_frame, next_next_field, next_field - next_next_field );
			} else {
				__decodeNextField( frame->aplus.aplus_frame, next_next_field, str_end - next_next_field );
			}
		}
		next_next_field = strtok( NULL, tab );
		if ( next_next_field != NULL ) {
			__decodeNextField( frame->aplus.aplus_frame, next_field, next_next_field - next_field );
		}else {
			__decodeNextField( frame->aplus.aplus_frame, next_field, str_end - C_FIELD_SIZE - next_field );
		}
	}while (next_next_field != NULL);

	frame->aplus.aplus_frame->V_Field = V_CHAR;

	return 0;
}

static int __cz2_append_crc( char read_frame[] )
{
	static unsigned short i, data;
	static unsigned int pos;
	static unsigned short accum, genpoly = 0x1021;

	accum = 0;
	pos   = 1;

	uint32_t len_frame = strlen(read_frame);

	if ( len_frame > 0 ) {
		while( pos != len_frame - 5 ) {
			data = read_frame[pos++] << 8;
			for ( i = 8; i > 0; i-- ) {
				if ( ( data ^ accum ) & 0x8000 ) {
					accum = ( accum << 1 ) ^ genpoly;
				} else {
					accum <<= 1;
				}
				data <<= 1;
			}
		}

	} else {
		accum = 1;
	}

	return accum;
}

static int __cz2_append_crc_frame( char read_frame[] )
{
	static unsigned short i, data;
	static unsigned int pos;
	static unsigned short accum, genpoly = 0x1021;

	accum = 0;
	pos   = 1;

	uint32_t len_frame = strlen(read_frame);

	if ( len_frame > 0 ) {
		while( pos != len_frame ) {
			data = read_frame[pos++] << 8;
			for ( i = 8; i > 0; i-- ) {
				if ( ( data ^ accum ) & 0x8000 ) {
					accum = ( accum << 1 ) ^ genpoly;
				} else {
					accum <<= 1;
				}
				data <<= 1;
			}
		}

	} else {
		accum = 1;
	}

	return accum;
}

//static void __bin2bcd( unsigned int val, char digit[] )
//{
//	char i;
//
//	i = '0' - 1;
//	do {
//		i++;
//	}while( !((val -= 10000) & 0x8000) );
//	digit[4] = i;
//
//	i = '0' + 10;
//	do{
//		i--;
//	}while( (val += 1000) & 0x8000 );
//	digit[3] = i;
//
//	i = '0' - 1;
//	do{
//		i++;
//	}while( !((val -= 100) & 0x8000) );
//	digit[2] = i;
//
//	i = '0' + 10;
//	do{
//		i--;
//	}while( (val += 10) & 0x8000 );
//	digit[1] = i;
//
//	digit[0] = val | '0';
//}

int32_t une82326_protocol_tx_a_plus_frame( uint8_t *data, uint32_t data_len )
{
	unsigned short accum;
	uint32_t val;

	accum = __cz2_append_crc( (char *)data );
	val   = ( accum & 0xF000 ) >> 12 | 0x30;
	*(data + data_len - 5) = val;
	val   = ( accum & 0x0F00 ) >> 8  | 0x30;
	*(data + data_len - 4) = val;
	val   = ( accum & 0x00F0 ) >> 4  | 0x30;
	*(data + data_len - 3) = val;
	val   = ( accum & 0x000F ) >> 0  | 0x30;
	*(data + data_len - 2) = val;
	serial_une82326_trx( &UartEmulHandle, data, data_len );

	return ERROR_A_PLUS_FRAME_PARSE_TX_OK;
}

// Link layer.
char *une82326_get_r_field_values( void )
{
	return r_field_string_values.r_field_values;
}

uint32_t une82326_get_num_r_field_values( void )
{
	return r_field_string_values.num;
}

char *une82326_get_crc_field_value( void )
{
	return r_field_string_values.crc;
}

uint32_t une82326_decode_r_field( char *r_field_value )
{
	uint32_t len = strlen(r_field_value);
	uint64_t u_subc = 0, a_subc = 0, t_subc = 0;
	int32_t  f_subc = 0;
	double   final_value, exp;
	char   * r_field_decoded = NULL;
	char     r_field_a_subc[17];

	memset( r_field_a_subc, 0, sizeof( r_field_a_subc ) );

//	sprintf( r_field_value, "%s" , "C000000000498,1,-5" );

	r_field_decoded = (char *)memmem( (char *)r_field_value, len, ",", 1 );
	if ( r_field_decoded != NULL ) {
		strncpy( (char *)r_field_a_subc, r_field_value + 1, r_field_decoded - r_field_value - 1 );
		a_subc = atoll(r_field_a_subc);
		if ( r_field_decoded != NULL ) {
			r_field_decoded = strtok( r_field_decoded, "," );
			if ( r_field_decoded != NULL ) {
				u_subc          = atoll(r_field_decoded);
				r_field_decoded = strtok( NULL, "," );
				if ( r_field_decoded != NULL ) {
					f_subc          = atoll(r_field_decoded);
					r_field_decoded = strtok( NULL, "," );
					if ( r_field_decoded != NULL ) {
						t_subc = atoll(r_field_decoded);
						(void)t_subc;
					}
				}
			}

			if ( u_subc != 0 ) {
				exp = exp10(f_subc);
				final_value = (double)a_subc*exp;
				common_lib_dftoa( final_value, r_field_value + 1, abs(f_subc) );
			} else {
				sprintf(r_field_value + 1, "%d", (int)a_subc);
			}
			return 1;
		}
	} else {
		r_field_decoded = (char *)memmem( (char *)r_field_value, len, ".", 1 );
		if ( r_field_decoded != NULL ) {
			strncpy( (char *)r_field_a_subc, r_field_decoded + 1, strlen( r_field_decoded ) );
			a_subc = atoi(r_field_a_subc);
			sprintf(r_field_value + 1, "0.%d", (int)a_subc);
		} else {
			a_subc = atoi(r_field_value);
			sprintf(r_field_value + 1, "%d", (int)a_subc);
		}
	}
	return 0;
}

uint32_t une82326_add_r_field_value( char *r_field_value )
{
	uint32_t len = strlen(r_field_value);
	rtc_system_SetCreatedMeterValueTime( Tick_Get( SECONDS ) );
	rtc_system_SetCreatedMeterValueDate( Tick_Get( SECONDS ) );

	une82326_decode_r_field( r_field_value);
	memcpy(&r_field_string_values.r_field_values[0], rtc_system_getCreatedMeterValueTime(), 5);
	memcpy(&r_field_string_values.r_field_values[4], r_field_value, len);

	return (r_field_string_values.num);
}

uint32_t une82326_add_r_field_time_value( void )
{
	memcpy(&r_field_string_values.r_field_values[0], rtc_system_getCreatedTimeFileName() + 8, 4);

	return 0;
}

uint32_t une82326_add_crc_field_value( char *crc_field_value )
{
	uint32_t len = strlen(crc_field_value);

	memcpy(&r_field_string_values.crc, crc_field_value, len);

	return (r_field_string_values.num);
}

void une828326_meter_frame_append_checksum( char *crc_field )
{
	unsigned short accum;
	uint32_t val;

	accum = __cz2_append_crc_frame( r_field_string_values.r_field_values );
	val   = ( accum & 0xF000 ) >> 12 | 0x30;
	crc_field[0] = val;
	val   = ( accum & 0x0F00 ) >> 8  | 0x30;
	crc_field[1] = val;
	val   = ( accum & 0x00F0 ) >> 4  | 0x30;
	crc_field[2] = val;
	val   = ( accum & 0x000F ) >> 0  | 0x30;
	crc_field[3] = val;
}

uint8_t *une82326_s_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.S_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.S_Field;
	}
}

uint8_t *une82326_r_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.R_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.R_Field;
	}
}

uint8_t *une82326_x_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.X_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.X_Field;
	}
}

uint8_t *une82326_a_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.A_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.A_Field;
	}
}

uint8_t *une82326_f_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.F_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.F_Field;
	}
}

uint8_t *une82326_n_field(void)
{
	if (NULL == une82326_a_plus_frame_st.N_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.N_Field;
	}
}

uint8_t *une82326_q_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.Q_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.Q_Field;
	}
}

uint8_t *une82326_b_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.B_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.B_Field;
	}
}

uint8_t *une82326_j_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.J_Field ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.J_Field;
	}
}

uint8_t *une82326_crc_field(void)
{
	if ( NULL == une82326_a_plus_frame_st.CRC16 ) {
		return (uint8_t *)'*';
	} else {
		return une82326_a_plus_frame_st.CRC16;
	}
}

uint8_t une82326_b_frame_support(void)
{
	return (une82326_a_plus_frame_st.X_Field[3] & 0x0001);
}

int32_t une82326_protocol_parse( UNE82326_Frame *frame, uint8_t *data, size_t data_size )
{
    if ( ( frame && data && data_size ) > 0 ) {
    	if ( ( data[0] == V_CHAR ) || ( data[0] == COMMA_CHAR ) ) {//A+ frame.
        	frame                    = &une82326_frame_st;
        	frame->aplus.aplus_frame = &une82326_a_plus_frame_st;
        	if ( 1 == __a_plus_frame_verify( (char *)data ) ) {
        		uint8_t dev_num;
        		__parseAplusFrame(frame, data, serial_une82326_rx_num());
        		une828326_a_plus_frame_append_checksum((char *)frame->aplus.aplus_frame->CRC16);
        		if (une82326_device_table_manager_is_new_device( (char *)frame->aplus.aplus_frame->S_Field, &dev_num ) != 0xFF) {
        			une82326_set_frame_type(une82326_device_table_manager_check_frame_type(dev_num, (char *)frame->aplus.aplus_frame->CRC16)); // Check CRC to determine frame type.
        			une82326_device_table_manager_set_new_device_parameters(
        					(char *)frame->aplus.aplus_frame->S_Field,
        					(char *)frame->aplus.aplus_frame->CRC16);
//        			une82326_decode_r_field((char *)frame->aplus.aplus_frame->R_Field);//DEBUG:
        			une82326_set_write_record(1);
        		} else {
        			une82326_set_frame_type(une82326_device_table_manager_check_frame_type(dev_num, (char *)frame->aplus.aplus_frame->CRC16)); // Check CRC to determine frame type.
        			if ( WATER_METER_GATHERING_VALUES == une82326_get_frame_type() ) {
        				une82326_add_r_field_value((char *)frame->aplus.aplus_frame->R_Field);
        				une82326_add_crc_field_value((char *)frame->aplus.aplus_frame->CRC16);
        				une82326_set_write_record(1);
        			} else if ( A_PLUS_FRAME == une82326_get_frame_type() ) {
        				une82326_set_write_record(1);
        			}
        		}
        		return ERROR_A_PLUS_FRAME_PARSE_OK;
        	} else {
        		return ERROR_A_PLUS_FRAME_PARSE_CRC;
        	}
        } else {//A frame.
        	return __parseAFrame( frame, data, data_size );
        }
    }

    return ERROR_PARSE_UNKNOWN;
}

void une828326_a_plus_frame_append_checksum( char *crc_field )
{
	char data[128];
	unsigned short accum;
	uint32_t val;

	sprintf(data,                "%s", (char *)une82326_s_field());
	sprintf(data + strlen(data), "%s", (char *)une82326_x_field());
	sprintf(data + strlen(data), "%s", (char *)une82326_a_field());
	sprintf(data + strlen(data), "%s", (char *)une82326_f_field());
	sprintf(data + strlen(data), "%s", (char *)une82326_n_field());
	sprintf(data + strlen(data), "%s", (char *)une82326_q_field());
	sprintf(data + strlen(data), "%s", (char *)une82326_b_field());

	accum = __cz2_append_crc_frame( (char *)data );
	val   = ( accum & 0xF000 ) >> 12 | 0x30;
	crc_field[0] = val;
	val   = ( accum & 0x0F00 ) >> 8  | 0x30;
	crc_field[1] = val;
	val   = ( accum & 0x00F0 ) >> 4  | 0x30;
	crc_field[2] = val;
	val   = ( accum & 0x000F ) >> 0  | 0x30;
	crc_field[3] = val;
}

char dst[128];
void une82326_build_a_plus_information_message( void )
{
	char sep[2];
	char serial_number[10];
	char crc[5];

	memset(dst, 0, sizeof(dst));

	sep[0] = ';';
	sep[1] = '\0';
	crc[4] = '\0';

	sprintf(serial_number, "%s", une82326_s_field());
	une828326_a_plus_frame_append_checksum(crc);

//	rtc_system_SetCreatedMessageTime( Tick_Get( SECONDS ) );
//	strcat(dst, rtc_system_getCreatedMessageTime());				//created
//	strcat(dst, sep);
	strcat(dst, serial_number);
	strcat(dst, sep);
	strcat(dst, (char *)une82326_r_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_x_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_a_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_f_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_n_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_q_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_b_field());
	strcat(dst, sep);
	strcat(dst, (char *)une82326_j_field());
	strcat(dst, sep);
	strcat(dst, crc);
	strcat(dst, sep);
}

char *une82326_get_a_plus_information_message( void )
{
	return dst;
}

#endif
