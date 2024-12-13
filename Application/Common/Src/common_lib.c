/*
 * common_lib.c
 *
 *  Created on: 18 ago. 2019
 *      Author: Sergio Millán López
 */
#include "common_lib.h"

// reverses a string 'str' of length 'len'
/**@private
 */
static void __reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[]. d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
/** @private
 */
static int __intToStr(int x, char str[], int d)
{
	int i   = 0;

	if (x == 0) {
		str[i++] = '0';
	} else if (x < 0) {
		x   = x * -1;
	}
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	__reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void common_lib_ftoa(float n, char *res, int afterpoint)
{
	int i;

	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	float fpart = n - (float)ipart;

	// convert integer part to string
	i = __intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		__intToStr((int)fpart, res + i + 1, afterpoint);
	}

	if (ipart < 0) {
		char res_backup[12];
		uint8_t res_len = strlen(res);
		memcpy( &res_backup[1], res, (res_len+1) * sizeof(char) );
		res_backup[0]         = '-';
		res_backup[res_len+2] = '\0';
		memcpy( res, res_backup, (res_len+2) * sizeof(char) );
	}
}

// Converts a floating point number to string.
void common_lib_dftoa(double n, char *res, int afterpoint)
{
	int i;

	// Extract integer part
	int ipart = (int)n;

	// Extract floating part
	double fpart = n - (double)ipart;

	// convert integer part to string
	i = __intToStr(ipart, res, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.'; // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		__intToStr((int)fpart, res + i + 1, afterpoint);
	}

	if (ipart < 0) {
		char res_backup[12];
		uint8_t res_len = strlen(res);
		memcpy( &res_backup[1], res, (res_len+1) * sizeof(char) );
		res_backup[0]         = '-';
		res_backup[res_len+2] = '\0';
		memcpy( res, res_backup, (res_len+2) * sizeof(char) );
	}
}

static char byteabs(signed char x)
{
	if (x < 0)
		return -x;
	return x;
}

size_t common_lib_i64tostrn_a(signed long long x, char *s, size_t buff_size)
{
	signed long long t = x;
	size_t	i, r = 1, sign;

	if (x < 0) {
		sign = 1;
		while (t <= -10) {
			t /= 10;
			r++;
		}
	}
	else {
		sign = 0;
		while (t >= 10) {
			t /= 10;
			r++;
		}
	}

	if (s == 0)
		return r + sign;

	if (r > buff_size)
		r = buff_size - 1;

	if (sign) {
		*s = '-';
		s++;
	}

	for (i = r; i != 0; i--) {
		s[i - 1] = (char)byteabs(x % 10) + '0';
		x /= 10;
	}

	s[r] = (char)0;

	return r + sign;

}

size_t common_lib_u64tostrn_a(unsigned long long x, char *s, size_t buff_size)
{
	unsigned long long	t = x;
	size_t	i, r=1;

	while ( t >= 10 ) {
		t /= 10;
		r++;
	}

	if (s == 0)
		return r;

//	if (r > buff_size)
		r = buff_size;// - 1;

	for (i = r; i != 0; i--) {
		s[i-1] = (char)(x % 10) + '0';
		x /= 10;
	}

	s[r] = (char)0;

	return r;
}

//function to convert ascii char[] to hex-string (char[])
void common_lib_string2hexString(char* input, char* output, uint32_t len)
{
    int loop;
    int i, j;

    i    = 0;
    loop = 0;

    for ( j = 0; j < 2 * len; j++ ) {
        sprintf( (char*)(output + i),"%02X", input[loop] );
        loop += 1;
        i    += 2;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}

//function to convert ascii char[] to hex-string (char[]) adding separators
void common_lib_string2hexString_with_separators(char* input, char* output, uint32_t len)
{
    int loop;
    int i, j;

    i    = 0;
    loop = 0;

    for ( j = 0; j < 2 * len; j++ ) {
        if ((input[loop] == 0x7C) && (input[loop + 1] == 0x7C))
        {
        	output[i]      = '|';
        	output[i + 1]  = '|';
        	loop          += 2;
        	i             += 2;
        	j++;
        }
        else if ((input[loop] == 0x2A) && (input[loop + 1] == 0x7C))
        {
//        	output[i]    = '*';
//        	output[i+1]  = '|';
//        	loop        += 2;
//        	i           += 2;
//        	j++;
        	output[i]    = '|';
        	loop        += 2;
        	i           += 1;
        	j++;
        }
        else
        {
        	sprintf( (char*)(output + i),"%02X", input[loop] );
        	loop += 1;
        	i    += 2;
        }
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}
