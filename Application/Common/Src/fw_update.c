/**
  ******************************************************************************
  * @file           fw_update.c
  * @author 		Datakorum Development Team
  * @brief			Driver to handle FOTA Firmware Over The Air update.
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
 * fw_update.c
 *
 *  Created on: 19 nov. 2018
 *      Author: Sergio Millán López
 */
#include "fw_update.h"
#include "iwdg.h"

#include "stm32u5xx_hal.h"
#include "params.h"
#include "comm_serial.h"
#include "crc32.h"
#include "udp_protocol.h"
#include "tick.h"
#include "shutdown.h"
#include "spi_flash.h"

/**
 * @defgroup System_FW_Update System FOTA Update
 * @brief
 * @{
 * */

uint32_t file_size, socket_data_len, data_get, t, fw_update_addr, rem_len;

extern char _fwupdate_size[], _fwupdate_start_address[], _params_start_address[];

#if 0
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  return FLASH_BANK_1;
}
#endif

uint32_t fw_update_get_file_size( void )
{
	return file_size;
}

uint8_t fw_update_erase_flash( void )
{
//    FLASH_EraseInitTypeDef EraseInitStruct;
//    uint32_t PAGEError = 0, flash_page = 0, last_page, nb_of_pages;
//
//	/* Unlock the Flash to enable the flash control register access *************/
//	HAL_FLASH_Unlock();
//	/* Clear OPTVERR bit set on virgin samples */
//	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_SR_ERRORS);
//	/* Get the 1st page to erase */
//	flash_page 				  = GetPage((uint32_t) _fwupdate_start_address) ;
//	last_page                 = GetPage((uint32_t)_params_start_address - FLASH_PAGE_SIZE );
//	nb_of_pages               = last_page - flash_page + 1;
//	/* Fill EraseInit structure*/
//	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//	EraseInitStruct.Banks     = GetBank(flash_page);
//	EraseInitStruct.Page      = flash_page;
//	EraseInitStruct.NbPages   = nb_of_pages;
//	FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
//	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
//		return 0;
//	}
	sFLASH_EraseBulk();
	sFLASH_WaitBusy();
	return 1;
}

uint8_t fw_update_FSM( char *ptr, uint8_t reset, uint32_t *fw_length )
{
    static enum {
        FW_INIT,
        FW_PROGRAM,
        FW_WAIT_FOR_DATA,
        FW_CHECK,
        FW_DUMP,
        FW_DUMP_WAIT_FOR_MODULE_DATA,
        FW_DUMP_WAIT_FOR_DATA,
        FW_EXIT
    } fw_update_status = FW_INIT;

    static struct {
        uint32_t major;
        uint32_t minor;
        uint32_t size;
        uint32_t crc;
    } fw_update_version = { .major = 0, .minor = 0, .size = 0, .crc = 0 };

    uint8_t  page[SPI_FLASH_PAGE_SIZE];
    uint8_t  page_check[SPI_FLASH_PAGE_SIZE];
    uint32_t n, crc;
//    int32_t  data_len_sign;

    if( reset ) {
        fw_update_status = FW_EXIT;
    }

    shutdown_reset_watchdog();

    switch( fw_update_status )
    {
        case FW_INIT:
    		if ( HAL_IWDG_Refresh(&hiwdg) != HAL_OK ) {
    			/* Refresh Error */
    			Error_Handler();
    		}
    		shutdown_set_wchdg_time(SHUTDOWN_STAND_BY_MAX_TIME);
    		rest_set_upgrading_firmware(1);
        	rest_reset_upgrading_data();
            comm_serial_disable_rts();
			Tick_update_tick( TICK_FW );
			t = 60;
			socket_data_len = comm_serial_rcx_bytes_n();
            if( socket_data_len > 16 ) {
//    			n = min( socket_data_len, UDP_LENGTH );
//    			n = comm_serial_rcx_n( (uint8_t *)ptr, n );
//    			comm_serial_delete(n);
            	comm_serial_rcx_n( (uint8_t *)ptr, 16 );
            	comm_serial_delete(16);
                memcpy( (void *) &fw_update_version.major, (void *) ptr, 4 );
                ptr += 4;
                memcpy( (void *) &fw_update_version.minor, (void *) ptr, 4 );
                ptr += 4;
                memcpy( (void *) &fw_update_version.size,  (void *) ptr, 4 );
                ptr += 4;
                memcpy( (void *) &fw_update_version.crc,   (void *) ptr, 4 );
                ptr += 4;
                file_size        = *fw_length - 16;
                socket_data_len -= 16;
                data_get        += socket_data_len;
                if ( ( fw_update_version.size == file_size )
//                  && ( fw_update_version.major >= params_version_major() )
//				  && ( fw_update_version.minor != params_version_minor() )
				   ) {
                    fw_update_addr   = (uint32_t)(sFLASH_SPI_SECTOR_SIZE * 2);
                    fw_update_status = FW_WAIT_FOR_DATA;

                } else {
                    fw_update_status = FW_EXIT;
                }
            } else {
                fw_update_status = FW_INIT;
            }
            comm_serial_enable_rts();
            break;
        case FW_PROGRAM:
            break;
        case FW_WAIT_FOR_DATA:
        	if ( file_size ) {
        		uint32_t check_file = 0, num = 0;
        		n = min( file_size, sFLASH_SPI_PAGESIZE );
        		while ( n && ( comm_serial_rcx_bytes_n() >= n ) )
        		{
            		if ( HAL_IWDG_Refresh(&hiwdg) != HAL_OK ) {
            			/* Refresh Error */
            			Error_Handler();
            		}

        			comm_serial_rcx_n( page, n );

        			do
        			{
        				sFLASH_WriteBuffer(page, fw_update_addr, n);
        				sFLASH_ReadBuffer(page_check, fw_update_addr, n );
        				if (0 == memcmp(page, page_check, n))
        				{
        					check_file = 0;
        				}
        				else
        				{
        					check_file = 1;
        					num++;
        				}
        			} while ( ( 1 == check_file ) && ( num < 5 ) );
        			if (1==check_file)
        			{
        				LOGLIVE(LEVEL_1,"LOGLIVE> %d FW UPDATE>tERROR FLASH!!! In Write num:%d Size:%d\r\n", (int)Tick_Get( SECONDS ), (int)n, (int)file_size);
        			}

        			fw_update_addr += n;
        			file_size      -= n;
        			comm_serial_delete(n);

        			Tick_update_tick( TICK_FW );
        			t = 30;

        			LOGLIVE(LEVEL_1,"LOGLIVE> %d FW UPDATE>Write num:%d Size:%d\r\n", (int)Tick_Get( SECONDS ), (int)n, (int)file_size);
        		}
        	} else {
        		fw_update_status = FW_CHECK;
        	}

//			if (comm_serial_get_NO_CARRIER())
//			{
//        		fw_update_status = FW_EXIT;
//        		LOGLIVE(LEVEL_1,"LOGLIVE> %d FW UPDATE>EXIT NO CARRIER\r\n", (int)Tick_Get( SECONDS ));
//			}

        	if ( CHECK_ELAPSED_TIME( t, TICK_FW ) ) {
        		fw_update_status = FW_EXIT;
        		LOGLIVE(LEVEL_1,"LOGLIVE> %d FW UPDATE>EXIT TIMEOUT\r\n", (int)Tick_Get( SECONDS ));
        	}
            break;
        case FW_CHECK:
            file_size      = fw_update_version.size;
            fw_update_addr = (uint32_t)(sFLASH_SPI_SECTOR_SIZE * 2);
            crc            = 0;

            while ( (n = min( file_size, sFLASH_SPI_PAGESIZE )) )
            {
            	sFLASH_ReadBuffer( (unsigned char *) page, fw_update_addr, n );
            	crc = crc32( (char *) page, n, ~crc );
            	fw_update_addr += n;
            	file_size      -= n;
            }
        	if( fw_update_version.crc == crc ) {
            	LOGLIVE(LEVEL_1,"LOGLIVE> %d FW UPDATE>FW version:%d.%d\r\n", (int)Tick_Get( SECONDS ), (int)fw_update_version.major, (int)fw_update_version.minor);
            	params_init(); // FIXME - A eliminar - bug q corrompe page_i y pages_n al bajar fichero - No ocurre con fichero de patterns, no sé si por ser más pequeño...
                param.version.to_major  = fw_update_version.major;
                param.version.to_minor  = fw_update_version.minor;
                param.version.size      = fw_update_version.size;
                param.version.crc       = fw_update_version.crc;
                params_check_for_changes();
                __disable_irq();
                NVIC_SystemReset();
                return 0;
            }
            else {
                fw_update_status = FW_EXIT;

            }
            break;
        case FW_DUMP:
            if( file_size ) {
                file_size -= socket_data_len;
            }
            break;
        case FW_DUMP_WAIT_FOR_MODULE_DATA:
            socket_data_len = comm_serial_rcx_n( (uint8_t *)ptr, comm_serial_rcx_bytes_n() );
            if( socket_data_len ) {
                fw_update_status++;
            }
            break;
        case FW_DUMP_WAIT_FOR_DATA:
            socket_data_len = comm_serial_rcx_bytes_n();
            if( socket_data_len ) {
                fw_update_status = FW_PROGRAM;
            }
            break;
        case FW_EXIT:
        	rest_set_upgrading_firmware(0);
            fw_update_status = FW_INIT;
            return 0;
            break;
    }

    return 1;
}

#if 0
uint8_t fw_update_FSM( char *ptr, uint8_t reset, uint32_t *fw_length )
{
    static enum {
        FW_INIT,
        FW_PROGRAM,
        FW_WAIT_FOR_DATA,
        FW_CHECK,
        FW_DUMP,
        FW_DUMP_WAIT_FOR_MODULE_DATA,
        FW_DUMP_WAIT_FOR_DATA,
        FW_EXIT
    } fw_update_status = FW_INIT;

    static struct {
        uint32_t major;
        uint32_t minor;
        uint32_t size;
        uint32_t crc;
    } fw_update_version = { .major = 0, .minor = 0, .size = 0, .crc = 0 };

    static uint32_t fw_update_addr, word_size;
    static struct {
        union
        {
            uint64_t word;
            uint8_t  byte[8];
        } data;
        uint8_t size;
    } fw_tmp;

    uint32_t n, num_bytes_written;
    uint64_t data;
    int32_t  data_len_sign;

//    FLASH_EraseInitTypeDef EraseInitStruct;
//    uint32_t PAGEError = 0, flash_page = 0, last_page, nb_of_pages;

    if( reset ) {
        fw_update_status = FW_EXIT;
    }

    shutdown_reset_watchdog();

    switch( fw_update_status )
    {
        case FW_INIT:
    		if ( HAL_IWDG_Refresh(&hiwdg) != HAL_OK ) {
    			/* Refresh Error */
    			Error_Handler();
    		}
    		shutdown_set_wchdg_time(SHUTDOWN_STAND_BY_MAX_TIME);
    		rest_set_upgrading_firmware(1);
        	rest_reset_upgrading_data();
            comm_serial_disable_rts();
			Tick_update_tick( TICK_FW );
			t = 60;
			socket_data_len = comm_serial_rcx_bytes_n();
            if( socket_data_len > 16 ) {
    			n = min( socket_data_len, UDP_LENGTH );
    			n = comm_serial_rcx_n( (uint8_t *)ptr, n );
    			comm_serial_delete(n);
                memcpy( (void *) &fw_update_version.major, (void *) ptr, 4 );
                ptr += 4;
                memcpy( (void *) &fw_update_version.minor, (void *) ptr, 4 );
                ptr += 4;
                memcpy( (void *) &fw_update_version.size,  (void *) ptr, 4 );
                ptr += 4;
                memcpy( (void *) &fw_update_version.crc,   (void *) ptr, 4 );
                ptr += 4;
                file_size        = *fw_length - 16;
                socket_data_len -= 16;
                data_get        += socket_data_len;
                if ( ( fw_update_version.size == file_size )
//                  && ( fw_update_version.major >= params_version_major() )
//				  && ( fw_update_version.minor != params_version_minor() )
				   ) {
                    fw_update_addr = (uint32_t) _fwupdate_start_address;
                    fw_tmp.size    = 0;

                    if ( file_size ) {
                        if ( file_size > 7 ) {
                            word_size = 7;
                        } else {
                            word_size = file_size - 1;
                        }
                    } else {
                        fw_update_status = FW_CHECK;
                        break;
                    }
                    socket_data_len  += fw_tmp.size;
                    num_bytes_written = 0;
                    while( socket_data_len > word_size ) {
                        if( fw_update_addr >= 0x0807E000 ) {
                            break;
                        }
                        if ( fw_tmp.size ) {
                            memcpy( (void *) &fw_tmp.data.byte[fw_tmp.size], (void *) ptr, 8 - fw_tmp.size );
                            data = fw_tmp.data.word;
                            ptr += 8 - fw_tmp.size;
                            fw_tmp.size = 0;
                        } else {
                            memcpy( (void *) &data, (void *) ptr, word_size + 1 );
                            ptr += word_size + 1;
                        }
                        if ( HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD, fw_update_addr, data ) != HAL_OK ) {
                        	break;
                        }
                        num_bytes_written += 8;
                        fw_update_addr    += 8;
                        file_size         -= word_size + 1;
                        socket_data_len   -= 8;
                        if( file_size < 8 ) {
                            word_size = file_size - 1;
                        }
                    }
                    fw_tmp.size      = (uint8_t) socket_data_len;
                    fw_tmp.data.word = 0;
                    if ( fw_tmp.size ) {
                        memcpy( (void *) &fw_tmp.data.word, (void *) ptr, fw_tmp.size );
                    }
                    if ( file_size ) {
                        fw_update_status = FW_WAIT_FOR_DATA;
                    } else {
                        fw_update_status = FW_CHECK;
                    }
                } else {
                    fw_update_status = FW_EXIT;
                }
            } else {
                fw_update_status = FW_INIT;
            }
            comm_serial_enable_rts();
            break;
        case FW_PROGRAM:
            break;
        case FW_WAIT_FOR_DATA:
        do {
        	shutdown_reset_watchdog();
            rest_get_upgrading_data();
            socket_data_len = rest_check_upgrading_data();
            rest_reset_upgrading_data();
            data_len_sign = (int32_t) socket_data_len;
            if (data_len_sign != -1) {
                if (socket_data_len) {
                	LOGLIVE(LEVEL_1,"Size:%d\r\n", (int)file_size);
                	comm_serial_disable_rts();
                    ptr = rest_get_str_rx();
                    if (file_size) {
                        if (file_size > 7) {
                            word_size = 7;
                        } else {
                            word_size = file_size - 1;
                        }
                    } else {
                        comm_serial_enable_rts();
                        fw_update_status = FW_CHECK;
                        break;
                    }
                    socket_data_len += fw_tmp.size;
                    while (socket_data_len > word_size) {
                		if ( HAL_IWDG_Refresh(&hiwdg) != HAL_OK ) {
                			/* Refresh Error */
                			Error_Handler();
                		}
                        if (fw_update_addr >= 0x0807E000) {
                        	comm_serial_enable_rts();
                            break;
                        }
                        if (fw_tmp.size) {
                            memcpy((void *) &fw_tmp.data.byte[fw_tmp.size], (void *) ptr, 8 - fw_tmp.size);
                            data = fw_tmp.data.word;
                            ptr += 8 - fw_tmp.size;
                            fw_tmp.size = 0;
                        } else {
                            memcpy((void *) &data, (void *) ptr, word_size + 1);
                            ptr += word_size + 1;
                        }
                        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, fw_update_addr, data) != HAL_OK) {
//                        	comm_serial_enable_rts();
                        	break;
                        }
                        fw_update_addr  += 8;
                        file_size       -= word_size + 1;
                        socket_data_len -= 8;
                        if (file_size < 8) {
                            word_size = file_size - 1;
                        }
                    }
                    fw_tmp.size      = (uint8_t) socket_data_len;
                    fw_tmp.data.word = 0;
                    if (fw_tmp.size) {
                        memcpy((void *) &fw_tmp.data.word, (void *) ptr, fw_tmp.size);
                    }
                    if (file_size) {
                        fw_update_status = FW_WAIT_FOR_DATA;
                    } else {
                        fw_update_status = FW_CHECK;
                    }
                    comm_serial_enable_rts();
        			Tick_update_tick( TICK_FW );
        			t = 30;
                } else {
                	if ( CHECK_ELAPSED_TIME( t, TICK_FW ) ) {
                        fw_update_status = FW_EXIT;
                    }
                }
            } else {
    			Tick_update_tick( TICK_FW );
    			t = 30;
            }
        }while(fw_update_status == FW_WAIT_FOR_DATA);
            break;
        case FW_CHECK:
        	HAL_FLASH_Lock();
            if( fw_update_version.crc == crc32( (char *) 0x08046000, fw_update_version.size, 0xFFFFFFFF )) {
            	LOGLIVE(LEVEL_1,"FW version:%d.%d\r\n", (int)fw_update_version.major, (int)fw_update_version.minor);
            	params_init(); // FIXME - A eliminar - bug q corrompe page_i y pages_n al bajar fichero - No ocurre con fichero de patterns, no sé si por ser más pequeño...
                param.version.to_major  = fw_update_version.major;
                param.version.to_minor  = fw_update_version.minor;
                param.version.size      = fw_update_version.size;
                param.version.crc       = fw_update_version.crc;
                params_check_for_changes();
                __disable_irq();
                NVIC_SystemReset();
                return 0;
            }
            else {
                fw_update_status = FW_EXIT;

            }
            break;
        case FW_DUMP:
        	HAL_FLASH_Lock();
            if( file_size ) {
                file_size -= socket_data_len;
            }
            break;
        case FW_DUMP_WAIT_FOR_MODULE_DATA:
            socket_data_len = comm_serial_rcx_n( (uint8_t *)ptr, comm_serial_rcx_bytes_n() );
            if( socket_data_len ) {
                fw_update_status++;
            }
            break;
        case FW_DUMP_WAIT_FOR_DATA:
            socket_data_len = comm_serial_rcx_bytes_n();
            if( socket_data_len ) {
                fw_update_status = FW_PROGRAM;
            }
            break;
        case FW_EXIT:
        	rest_set_upgrading_firmware(0);
            fw_update_status = FW_INIT;
            return 0;
            break;
    }

    return 1;
}
#endif
/**
 * }@
 *
 */ // End defgroup System_FW_Update
