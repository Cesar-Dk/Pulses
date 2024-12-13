/*
 * spi_flash.c
 *
 *  Created on: 17 jul. 2019
 *      Author: smill
 */
/*
 * spi_flash.c
 *
 *  Created on: 18 sept. 2018
 *      Author: Sergio Mill�n L�pez
 */
/**
 ******************************************************************************

 * @file    SPI/SPI_FLASH/spi_flash.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    13-April-2012
 * @brief   This file provides a set of functions needed to manage the SPI M25Pxxx
 *          FLASH memory.
 *
 *          ===================================================================
 *          Notes:
 *           - There is no SPI FLASH memory available in STM322xG-EVAL board,
 *             to use this driver you have to build your own hardware.
 *          ===================================================================
 *
 *          It implements a high level communication layer for read and write
 *          from/to this memory. The needed STM32 hardware resources (SPI and
 *          GPIO) are defined in spi_flash.h file, and the initialization is
 *          performed in sFLASH_LowLevel_Init() function.
 *
 *          You can easily tailor this driver to any development board, by just
 *          adapting the defines for hardware resources and sFLASH_LowLevel_Init()
 *          function.
 *
 *          +-----------------------------------------------------------+
 *          |                     Pin assignment                        |
 *          +-----------------------------+---------------+-------------+
 *          |  STM32 SPI Pins             |     sFLASH    |    Pin      |
 *          +-----------------------------+---------------+-------------+
 *          | sFLASH_CS_PIN               | ChipSelect(/S)|    1        |
 *          | sFLASH_SPI_MISO_PIN / MISO  |   DataOut(Q)  |    2        |
 *          |                             |   VCC         |    3 (3.3 V)|
 *          |                             |   GND         |    4 (0 V)  |
 *          | sFLASH_SPI_MOSI_PIN / MOSI  |   DataIn(D)   |    5        |
 *          | sFLASH_SPI_SCK_PIN / SCK    |   Clock(C)    |    6        |
 *          |                             |    VCC        |    7 (3.3 V)|
 *          |                             |    VCC        |    8 (3.3 V)|
 *          +-----------------------------+---------------+-------------+

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

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include "spi_flash.h"
#include "spi.h"

/** @addtogroup SPI_FLASH
 * @{
 */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t spi_tx[260], spi_rx[260];
extern IWDG_HandleTypeDef hiwdg;
/* Private function prototypes -----------------------------------------------*/
void sFLASH_LowLevel_DeInit(void);
void sFLASH_LowLevel_Init(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @fn void sFLASH_disable_flash(void)
 * @brief Disables the power rail to supply the SPI Flash
 */
void sFLASH_disable_flash( void )
{
	HAL_GPIO_WritePin(sFLASH_ENABLE_GPIO_PORT, sFLASH_ENABLE_PIN, GPIO_PIN_RESET);
}

/**
 * @fn void sFLASH_enable_flash(void)
 * @brief Enables the power rail to supply the SPI Flash.
 */
void sFLASH_enable_flash( void )
{
	HAL_GPIO_WritePin(sFLASH_ENABLE_GPIO_PORT, sFLASH_ENABLE_PIN, GPIO_PIN_SET);
}

/**
 * @fn void sFLASH_initSST26V(void)
 * @brief Power up sequence to initialize the SST26V SPI memory
 */
void sFLASH_initSST26V( void )
{
	uint16_t block_protection_18[1];

	block_protection_18[0] = 0x00;

	sFLASH_ResetEnable();
	sFLASH_WaitBusy();
	sFLASH_Reset();
	sFLASH_WaitBusy();

	sFLASH_GlobalBlockProtectionUnlock();
	sFLASH_WaitBusy();

	sFLASH_WriteBlockProtection(&block_protection_18[0]);
	sFLASH_WaitBusy();

	sFLASH_ReadID();
	sFLASH_WaitBusy();
}

/**
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 * @param  None
 */
void sFLASH_DeInit(void)
{
	sFLASH_LowLevel_DeInit();
	sFLASH_disable_flash();
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @param  None
 */
void sFLASH_Init(void)
{
//	SPI_InitTypeDef SPI_InitStructure;

	sFLASH_enable_flash();

	sFLASH_LowLevel_Init();

	sFLASH_enable_flash();

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	MX_SPI2_Init();

	/* drain SPI TX buffer, just in case*/
	while (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_TXP) == RESET) {;}
//	SPI_I2S_ReceiveData16(sFLASH_SPI);
	/* set the line direction to prepare the send of the first command*/
	SPI_1LINE_TX(&hspi2);
	__HAL_SPI_ENABLE(&hspi2);
}

void sFLASH_Init_Test(void)
{
//	SPI_InitTypeDef SPI_InitStructure;

	sFLASH_enable_flash();

//	sFLASH_LowLevel_Init();
//
//	sFLASH_enable_flash();

//	/*!< Deselect the FLASH: Chip Select high */
//	sFLASH_CS_HIGH();
//
//	MX_SPI2_Init();
}

/**
 * @brief  Erases the specified FLASH sector.
 * @param  SectorAddr: address of the sector to erase.
 */
void sFLASH_EraseSector(uint32_t SectorAddr)
{
	/*!< Send write enable instruction */
	sFLASH_WriteEnable();

	/*!< Sector Erase */
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send Sector Erase instruction */
	sFLASH_SendByte(sFLASH_CMD_SE);

	/*!< Send SectorAddr high nibble address byte */
	sFLASH_SendByte((SectorAddr & 0xFF0000) >> 16);

	/*!< Send SectorAddr medium nibble address byte */
	sFLASH_SendByte((SectorAddr & 0xFF00) >> 8);

	/*!< Send SectorAddr low nibble address byte */
	sFLASH_SendByte(SectorAddr & 0xFF);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 */
void sFLASH_EraseBlock(uint32_t BlockAddr)
{
	/*!< Send write enable instruction */
	sFLASH_WriteEnable();

	/*!< Block Erase */
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send Block Erase instruction */
	sFLASH_SendByte(sFLASH_CMD_BE);

	/*!< Send BlockAddr high nibble address byte */
	sFLASH_SendByte((BlockAddr & 0xFF0000) >> 16);

	/*!< Send BlockAddr medium nibble address byte */
	sFLASH_SendByte((BlockAddr & 0xFF00) >> 8);

	/*!< Send BlockAddr low nibble address byte */
	sFLASH_SendByte(BlockAddr & 0xFF);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 */
void sFLASH_EraseBulk(void)
{
	/*!< Send write enable instruction */
	sFLASH_WriteEnable();

	/*!< Bulk Erase */
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send Bulk Erase instruction  */
	sFLASH_SendByte(sFLASH_CMD_CE);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 */
void sFLASH_EraseChip(void)
{
	/*!< Send write enable instruction */
	sFLASH_WriteEnable();

	/*!< Bulk Erase */
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send Bulk Erase instruction  */
	sFLASH_SendByte(sFLASH_CMD_CE);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 */
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	/*!< Enable the write access to the FLASH */
	sFLASH_WriteEnable();

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Write to Memory " instruction */
	sFLASH_SendByte(sFLASH_CMD_WRITE);

	/*!< Send WriteAddr high nibble address byte to write to */
	sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);

	/*!< Send WriteAddr medium nibble address byte to write to */
	sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);

	/*!< Send WriteAddr low nibble address byte to write to */
	sFLASH_SendByte(WriteAddr & 0xFF);

	/*!< while there is data to be written on the FLASH */
	while (NumByteToWrite--)
	{
		/*!< Send the current byte */
		sFLASH_SendByte(*pBuffer);

		/*!< Point on the next byte to be written */
		pBuffer++;
	}

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	sFLASH_WriteDisable();

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 */
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

	Addr        = WriteAddr % sFLASH_SPI_PAGESIZE;
	count       = sFLASH_SPI_PAGESIZE - Addr;
	NumOfPage   = NumByteToWrite / sFLASH_SPI_PAGESIZE;
	NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

	if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
	{
		if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
		{
			sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
		}
		else /*!< NumByteToWrite > sFLASH_PAGESIZE */
		{
			while (NumOfPage--)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
				WriteAddr += sFLASH_SPI_PAGESIZE;
				pBuffer   += sFLASH_SPI_PAGESIZE;
			}
			sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
		}
	}
	else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
	{
		if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
		{
			if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
			{
				temp = NumOfSingle - count;
				sFLASH_WritePage(pBuffer, WriteAddr, count);
				WriteAddr += count;
				pBuffer   += count;
				sFLASH_WritePage(pBuffer, WriteAddr, temp);
			}
			else
			{
				sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
			}
		}
		else /*!< NumByteToWrite > sFLASH_PAGESIZE */
		{
			NumByteToWrite -= count;
			NumOfPage       = NumByteToWrite / sFLASH_SPI_PAGESIZE;
			NumOfSingle     = NumByteToWrite % sFLASH_SPI_PAGESIZE;
			sFLASH_WritePage(pBuffer, WriteAddr, count);
			WriteAddr += count;
			pBuffer   += count;

			while (NumOfPage--)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
				WriteAddr += sFLASH_SPI_PAGESIZE;
				pBuffer   += sFLASH_SPI_PAGESIZE;
			}

			if (NumOfSingle != 0)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
			}
		}
	}
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 */
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Read from Memory " instruction */
	sFLASH_SendByte(sFLASH_CMD_READ);

	/*!< Send ReadAddr high nibble address byte to read from */
	sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);

	/*!< Send ReadAddr medium nibble address byte to read from */
	sFLASH_SendByte((ReadAddr & 0xFF00) >> 8);

	/*!< Send ReadAddr low nibble address byte to read from */
	sFLASH_SendByte(ReadAddr & 0xFF);

	while (NumByteToRead--) /*!< while there is data to be read */
	{
		/*!< Read a byte from the FLASH */
		*pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

		/*!< Point to the next location where the byte read will be saved */
		pBuffer++;
	}

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief  Reads FLASH identification.
 * @retval FLASH identification
 */
uint32_t sFLASH_ReadID(void)

{
	uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "RDID " instruction */
	sFLASH_SendByte(0x9F);

	/*!< Read a byte from the FLASH */
	Temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

	/*!< Read a byte from the FLASH */
	Temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

	/*!< Read a byte from the FLASH */
	Temp2 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

	return Temp;
}

/**
 * @brief  Initiates a read data byte (READ) sequence from the Flash.
 *   This is done by driving the /CS line low to select the device, then the READ
 *   instruction is transmitted followed by 3 bytes address. This function exit
 *   and keep the /CS line low, so the Flash still being selected. With this
 *   technique the whole content of the Flash is read with a single READ instruction.
 * @param  ReadAddr: FLASH's internal address to read from.
 */
void sFLASH_StartReadSequence(uint32_t ReadAddr)
{
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Read from Memory " instruction */
	sFLASH_SendByte(sFLASH_CMD_READ);

	/*!< Send the 24-bit address of the address to read from -------------------*/

	/*!< Send ReadAddr high nibble address byte */
	sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);

	/*!< Send ReadAddr medium nibble address byte */
	sFLASH_SendByte((ReadAddr & 0xFF00) >> 8);

	/*!< Send ReadAddr low nibble address byte */
	sFLASH_SendByte(ReadAddr & 0xFF);
}

/**
  * @brief  This function send a command through SPI bus.
  * @param  command command id.
  * @param  val value.
  */
uint8_t sFLASH_ReadByte(void)
{
	uint8_t val;

	/* In master RX mode the clock is automaticaly generated on the SPI enable.
	 So to guarantee the clock generation for only one data, the clock must be
	 disabled after the first bit and before the latest bit */
//	__HAL_SPI_ENABLE(&hspi2);
//	SET_BIT(hspi2.Instance->CR1, SPI_CR1_CSTART);
//	__asm("dsb\n");
//	__asm("dsb\n");
//	__asm("dsb\n");
//	__asm("dsb\n");
//	__HAL_SPI_DISABLE(&hspi2);
	/* Set the number of data at current transfer */
	MODIFY_REG(hspi2.Instance->CR2, SPI_CR2_TSIZE, 1);

	while ((hspi2.Instance->SR & SPI_FLAG_RXP) != SPI_FLAG_RXP);
	/* read the received data */
	val = *(__IO uint8_t *) &hspi2.Instance->RXDR;
//	while ((hspi2.Instance->SR & SPI_FLAG_SUSP) != SPI_FLAG_SUSP);

	return val;
}

/**
 * @brief Receives a byte
 * @return val byte received
 */
uint8_t sFLASH_ReceiveByte( void )
{
	uint8_t val;

	SYNCHRO_WAIT(300);
	__HAL_SPI_DISABLE(&hspi2);
	SPI_1LINE_RX(&hspi2);
	SYNCHRO_WAIT(300);
	val = sFLASH_ReadByte();
	SYNCHRO_WAIT(300);
	SPI_1LINE_TX(&hspi2);
	__HAL_SPI_ENABLE(&hspi2);
	SYNCHRO_WAIT(300);

	return val;
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte byte to send.
 * @retval The value of the received byte.
 */
uint8_t sFLASH_SendByte(uint8_t byte)
{
	__HAL_SPI_DISABLE(&hspi2);
	/* Set the number of data at current transfer */
	MODIFY_REG(hspi2.Instance->CR2, SPI_CR2_TSIZE, 1);
	__HAL_SPI_ENABLE(&hspi2);
	/* check TXE flag */
	while ((hspi2.Instance->SR & SPI_FLAG_TXP) == 0);
	/* Master transfer start */
	SET_BIT(hspi2.Instance->CR1, SPI_CR1_CSTART);
	/* Write the data */
	*((__IO uint8_t*) &hspi2.Instance->TXDR) = byte;

	/* Wait BSY flag */
	while ((hspi2.Instance->SR & SPI_FLAG_EOT)  != SPI_FLAG_EOT);
//	while ((hspi2.Instance->SR & SPI_FLAG_SUSP) != SPI_FLAG_SUSP);
	__HAL_SPI_CLEAR_EOTFLAG(&hspi2);
	__HAL_SPI_CLEAR_TXTFFLAG(&hspi2);

	return sFLASH_ReadByte();//sFLASH_ReceiveByte();
}

/**
 * @brief  Enables the write access to the FLASH.
 */
void sFLASH_WriteEnable(void)
{
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Write Enable" instruction */
	sFLASH_SendByte(sFLASH_CMD_WREN);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief  Enables the write access to the FLASH.
 */
void sFLASH_WriteDisable(void)
{
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Write Disable" instruction */
	sFLASH_SendByte(sFLASH_CMD_WRDIS);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief Enables the Reset command
 */
void sFLASH_ResetEnable(void)
{
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Reset Enable" instruction */
	sFLASH_SendByte(sFLASH_CMD_RSTEN);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief Sends the Reset command
 */
void sFLASH_Reset(void)
{
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Rest" instruction */
	sFLASH_SendByte(sFLASH_CMD_RST);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief Reads configuration Register
 * @return cr Configuration Register byte
 */
uint8_t sFLASH_ReadConfigurationRegister(void)
{
	uint8_t cr;

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send  instruction */
	sFLASH_SendByte(sFLASH_CMD_RDCR);

	cr = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	return cr;
}

/**
 * @brief Reads Status Register
 * @return sr Status Register byte
 */
uint8_t sFLASH_ReadStatusRegister(void)
{
	uint8_t sr;

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send  instruction */
	sFLASH_SendByte(sFLASH_CMD_RDSR);

	sr = sFLASH_SendByte(sFLASH_DUMMY_BYTE);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	return sr;
}

/**
 * @brief  Erases the entire FLASH.
 */
void sFLASH_GlobalBlockProtectionUnlock(void)
{
	/*!< Send write enable instruction */
	sFLASH_WriteEnable();

	/*!< Bulk Erase */
	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send Bulk Erase instruction  */
	sFLASH_SendByte(sFLASH_CMD_ULBPR);

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();

	/*!< Wait the end of Flash writing */
	sFLASH_WaitForWriteEnd();
}

/**
 * @brief Protects a block of memory
 * @param block_protection_data
 */
void sFLASH_WriteBlockProtection( uint16_t *block_protection_data )
{
	uint8_t i;
	i = 0;

	/*!< Send write enable instruction */
	sFLASH_WriteEnable();

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send " Write block protection register " instruction */
	sFLASH_SendByte(sFLASH_CMD_WBPR);

	for ( i = 18 ; i > 0; i-- ) {

		sFLASH_SendByte(*block_protection_data);

		/*!< Point to the next location where the byte read will be saved */
//		block_protection_data++;
	}

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @param  None
 * @retval None
 */
void sFLASH_WaitForWriteEnd(void)
{
	uint8_t flashstatus = 0;

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Read Status Register" instruction */
	sFLASH_SendByte(sFLASH_CMD_RDSR);

	/*!< Loop as long as the memory is busy with a write cycle */
	do {
		/*!< Send a dummy byte to generate the clock needed by the FLASH
		 and put the value of the status register in FLASH_Status variable */
		flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
		HAL_IWDG_Refresh(&hiwdg);
	} while ((flashstatus & sFLASH_WIP_FLAG) != 0); /* Write in progress */

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief Waits while busy
 */
void sFLASH_WaitBusy(void)
{
	uint8_t flashstatus = 0;

	/*!< Select the FLASH: Chip Select low */
	sFLASH_CS_LOW();

	/*!< Send "Read Status Register" instruction */
	sFLASH_SendByte(sFLASH_CMD_RDSR);

	/*!< Loop as long as the memory is busy with a write cycle */
	do {
		/*!< Send a dummy byte to generate the clock needed by the FLASH
		 and put the value of the status register in FLASH_Status variable */
		flashstatus = sFLASH_SendByte( sFLASH_DUMMY_BYTE );
		if (flashstatus != 0) {
			asm("nop");
		}
	} while ((flashstatus & sFLASH_BUSY_FLAG) == sFLASH_BUSY_FLAG); /* waste time until busy */

	/*!< Deselect the FLASH: Chip Select high */
	sFLASH_CS_HIGH();
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 */
void sFLASH_LowLevel_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	HAL_SPI_MspInit(&hspi2);

	/*!< Configure sFLASH Card CS pin in output pushpull mode ********************/
	GPIO_InitStruct.Pin = sFLASH_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStruct);

	/* SPI WP pin configuration */
	GPIO_InitStruct.Pin   = sFLASH_ENABLE_PIN;// | sFLASH_WP_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(sFLASH_ENABLE_GPIO_PORT, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(sFLASH_WP_GPIO_PORT, sFLASH_WP_PIN, GPIO_PIN_SET);
}

/**
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 */
void sFLASH_LowLevel_DeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	HAL_SPI_MspDeInit(&hspi2);

	GPIO_InitStruct.Pin   = sFLASH_CS_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(sFLASH_CS_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @}
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



