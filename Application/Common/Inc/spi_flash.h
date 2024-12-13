/*
 * spi_flash.h
 *
 *  Created on: 17 jul. 2019
 *      Author: smill
 */

#ifndef APPLICATION_USER_COMMON_INC_SPI_FLASH_H_
#define APPLICATION_USER_COMMON_INC_SPI_FLASH_H_

/**

  ******************************************************************************

  * @file    spi_flash.h

  * @author  MCD Application Team

  * @version V1.1.0

  * @date    13-April-2012

  * @brief   This file contains all the functions prototypes for the spi_flash

  *          firmware driver.

  ******************************************************************************

  * @attention

  *

  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>

  *

  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");

  * You may not use this file except in compliance with the License.

  * You may obtain a copy of the License at:

  *

  *        http://www.st.com/software_license_agreement_liberty_v2

  *

  * Unless required by applicable law or agreed to in writing, software

  * distributed under the License is distributed on an "AS IS" BASIS,

  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

  * See the License for the specific language governing permissions and

  * limitations under the License.

  *

  ******************************************************************************

  */



/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __SPI_FLASH_H

#define __SPI_FLASH_H

#ifdef __cplusplus

 extern "C" {

#endif


/* Includes ------------------------------------------------------------------*/

#include "stm32u5xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* M25P SPI Flash supported commands */

#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_WRDIS          0x04
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_RDCR           0x35
#define sFLASH_CMD_RDID           0x9F  /* Read identification */
#define sFLASH_CMD_SE             0x20  /* Sector Erase instruction */
#define sFLASH_CMD_BE             0xD8  /* Bulk Erase instruction */
#define sFLASH_CMD_CE             0xC7  /* Chip Erase instruction */
#define sFLASH_CMD_ULBPR          0x98
#define sFLASH_CMD_WBPR           0x42
#define sFLASH_CMD_RSTEN          0x66
#define sFLASH_CMD_RST            0x99

#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */
#define sFLASH_BUSY_FLAG          0x80  /* Busy (Busy) flag */
#define sFLASH_DUMMY_BYTE         0x00

#define sFLASH_SPI_PAGESIZE       0x100
#define sFLASH_SPI_SECTOR_SIZE   ( 4096 )

#define sFLASH_M25P128_ID         0x202018
#define sFLASH_M25P64_ID          0x202017

/* Communication boards SPIx Interface */
#define sFLASH_SPI                       SPI2

#define sFLASH_SPI_SCK_PIN               GPIO_Pin_13
#define sFLASH_SPI_SCK_GPIO_PORT         GPIOB

#define sFLASH_SPI_MISO_PIN              GPIO_Pin_14
#define sFLASH_SPI_MISO_GPIO_PORT        GPIOB

#define sFLASH_SPI_MOSI_PIN              GPIO_Pin_15
#define sFLASH_SPI_MOSI_GPIO_PORT        GPIOB

#define sFLASH_CS_PIN                    GPIO_PIN_12//GPIO_PIN_4
#define sFLASH_CS_GPIO_PORT              GPIOB//GPIOA

//#define sFLASH_WP_PIN                    GPIO_PIN_0
//#define sFLASH_WP_GPIO_PORT              GPIOB

#define sFLASH_ENABLE_PIN                GPIO_PIN_3
#define sFLASH_ENABLE_GPIO_PORT          GPIOH

/* Exported macro ------------------------------------------------------------*/
#define SYNCHRO_WAIT(NB)       for(int i=0; i<NB; i++){__asm("dsb\n");}
/* Select sFLASH: Chip Select pin low */
#define sFLASH_CS_LOW()       HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_RESET)//HAL_GPIO_WritePin(sFLASH_ENABLE_GPIO_PORT, sFLASH_ENABLE_PIN, GPIO_PIN_RESET);

/* Deselect sFLASH: Chip Select pin high */
#define sFLASH_CS_HIGH()      HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_SET)//HAL_GPIO_WritePin(sFLASH_ENABLE_GPIO_PORT, sFLASH_ENABLE_PIN, GPIO_PIN_SET);
/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void     sFLASH_initSST26V( void );
void     sFLASH_DeInit(void);
void     sFLASH_Init(void);
void sFLASH_Init_Test(void);
void     sFLASH_EraseSector(uint32_t SectorAddr);
void     sFLASH_EraseBlock(uint32_t BlockAddr);
void     sFLASH_EraseBulk(void);
void     sFLASH_EraseChip(void);
void     sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void     sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void     sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void     sFLASH_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t  sFLASH_ReadByte(void);
uint8_t  sFLASH_SendByte(uint8_t byte);
void     sFLASH_WriteEnable(void);
void     sFLASH_WriteDisable(void);
void     sFLASH_ResetEnable(void);
void     sFLASH_Reset(void);
uint8_t  sFLASH_ReadConfigurationRegister(void);
uint8_t  sFLASH_ReadStatusRegister(void);
void     sFLASH_GlobalBlockProtectionUnlock(void);
void     sFLASH_WriteBlockProtection( uint16_t *block_protection_data );
void     sFLASH_WaitForWriteEnd(void);
void     sFLASH_WaitBusy(void);

#ifdef __cplusplus

}

#endif



#endif /* __SPI_FLASH_H */





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#endif /* APPLICATION_USER_COMMON_INC_SPI_FLASH_H_ */
