/**
  ******************************************************************************
  * @file           fw_update.h
  * @author 		Datakorum Development Team
  * @brief			Header file for fw_update.c file
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 Datakorum Solutions S.L.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/*
 * fw_update.h
 *
 *  Created on: 19 nov. 2018
 *      Author: Sergio Millán López
 */

#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>

uint32_t fw_update_get_file_size( void );
uint8_t  fw_update_erase_flash( void );
uint8_t  fw_update_FSM( char *ptr, uint8_t reset, uint32_t *fw_length );

#endif /* FW_UPDATE_H_ */
