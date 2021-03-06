/**
 ******************************************************************************
 * @file    usbd_storage_msd.c
 * @author  MCD application Team
 * @version V1.2.1
 * @date    17-March-2018
 * @brief   This file provides the disk operations functions.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      <http://www.st.com/SLA0044>
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------
 */
#include "usbd_msc_mem.h"
#include "bsp_driver_sd.h"
#include "led.h"
#include <stdbool.h>

#define GET_TIME_MS() HAL_GetTick()

#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINT_RAW(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define PRINT_INF(fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#define PRINT_ERR(fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#else
#define PRINT_RAW(fmt, ...)
#define PRINT_INF(fmt, ...)
#define PRINT_ERR(fmt, ...)
#endif

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup STORAGE
 * @brief media storage application module
 * @{
 */

/** @defgroup STORAGE_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup STORAGE_Private_Defines
 * @{
 */

#define STORAGE_LUN_NBR 1
/**
 * @}
 */

/** @defgroup STORAGE_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup STORAGE_Private_Variables
 * @{
 */

static uint32_t timestamp_last_read, timestamp_last_write;

/* USB Mass storage Standard Inquiry Data */
const int8_t STORAGE_Inquirydata[] = {
	// 36

	/* LUN 0 */
	0x00, 0x80, 0x02, 0x02, (USBD_STD_INQUIRY_LENGTH - 5),
	0x00, 0x00, 0x00, 'F',	'l',
	'y',  'F',	'i',  'r',	'e',
	' ', /* Manufacturer : 8 bytes */
	'P',  'a',	'r',  'a',	'c',
	'h',  'u',	't',  'e', /* Product : 16 Bytes */
	' ',  ' ',	' ',  ' ',	' ',
	' ',  '1',	'.',  '0',	' ', /* Version : 4 Bytes */
};

/**
 * @}
 */

/** @defgroup STORAGE_Private_FunctionPrototypes
 * @{
 */
int8_t STORAGE_Init(uint8_t lun);

int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t* block_num, uint32_t* block_size);

int8_t STORAGE_IsReady(uint8_t lun);

int8_t STORAGE_IsWriteProtected(uint8_t lun);

int8_t STORAGE_Read(uint8_t lun, uint8_t* buf, uint32_t blk_addr, uint16_t blk_len);

int8_t STORAGE_Write(uint8_t lun, uint8_t* buf, uint32_t blk_addr, uint16_t blk_len);

int8_t STORAGE_GetMaxLun(void);

USBD_STORAGE_cb_TypeDef USBD_MICRO_SDIO_fops = {
	STORAGE_Init, STORAGE_GetCapacity, STORAGE_IsReady,	  STORAGE_IsWriteProtected,
	STORAGE_Read, STORAGE_Write,	   STORAGE_GetMaxLun, (int8_t*)STORAGE_Inquirydata,
};

USBD_STORAGE_cb_TypeDef* USBD_STORAGE_fops = &USBD_MICRO_SDIO_fops;
//#ifndef USE_STM3210C_EVAL
// extern SD_CardInfo SDCardInfo;
//#endif
__IO uint32_t count = 0;
/**
 * @}
 */

/** @defgroup STORAGE_Private_Functions
 * @{
 */

/**
 * @brief  Initialize the storage medium
 * @param  lun : logical unit number
 * @retval Status
 */
static bool sdcard_is_ready = false;
int8_t		STORAGE_Init(uint8_t lun) {
	 int err = (int)BSP_SD_Init();
	 if(err == MSD_OK) {
		 sdcard_is_ready = true;
		 return 0;
	 }
	 return (-1);
}

/**
 * @brief  return medium capacity and block size
 * @param  lun : logical unit number
 * @param  block_num :  number of physical block
 * @param  block_size : size of a physical block
 * @retval Status
 */
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t* block_num, uint32_t* block_size) {
	BSP_SD_CardInfo card_info;
	BSP_SD_GetCardInfo(&card_info);
	*block_size = card_info.BlockSize;
	*block_num	= card_info.BlockNbr;
	return (0);
}

/**
 * @brief  check whether the medium is ready
 * @param  lun : logical unit number
 * @retval Status
 */
int8_t STORAGE_IsReady(uint8_t lun) {
	if(sdcard_is_ready) {
		return 0;
	} else {
		return -1;
	}
}

/**
 * @brief  check whether the medium is write-protected
 * @param  lun : logical unit number
 * @retval Status
 */
int8_t STORAGE_IsWriteProtected(uint8_t lun) {
	return 0;
}

/**
 * @brief  Read data from the medium
 * @param  lun : logical unit number
 * @param  buf : Pointer to the buffer to save data
 * @param  blk_addr :  address of 1st block to be read
 * @param  blk_len : nmber of blocks to be read
 * @retval Status
 */
int8_t STORAGE_Read(uint8_t lun, uint8_t* buf, uint32_t blk_addr, uint16_t blk_len) {
	timestamp_last_read = GET_TIME_MS();
	while(BSP_SD_GetCardState() == SD_TRANSFER_BUSY)
		;
	int result = BSP_SD_ReadBlocks(buf, blk_addr, blk_len, 2000);
	if(result == MSD_OK) {
		return 0;
	} else {
		return -1;
	}
}

/**
 * @brief  Write data to the medium
 * @param  lun : logical unit number
 * @param  buf : Pointer to the buffer to write from
 * @param  blk_addr :  address of 1st block to be written
 * @param  blk_len : nmber of blocks to be read
 * @retval Status
 */
int8_t STORAGE_Write(uint8_t lun, uint8_t* buf, uint32_t blk_addr, uint16_t blk_len) {
	timestamp_last_write = GET_TIME_MS();
	while(BSP_SD_GetCardState() == SD_TRANSFER_BUSY)
		;
	int result = BSP_SD_WriteBlocks(buf, blk_addr, blk_len, 2000);
	if(result == MSD_OK) {
		return 0;
	} else {
		return -1;
	}
}

/**
 * @brief  Return number of supported logical unit
 * @param  None
 * @retval number of logical unit
 */

int8_t STORAGE_GetMaxLun(void) {
	return (STORAGE_LUN_NBR - 1);
}

/**
 * @}
 */

int USBD_USR_IsStorageActive(void) {
	if(GET_TIME_MS() - timestamp_last_read < 200 || GET_TIME_MS() - timestamp_last_write < 200) {
		return 1;
	}
	return 0;
}

/**
 * @}
 */
uint32_t USBD_USR_GetTimestamp_LastWrite(void) {
	return timestamp_last_write;
}

/**
 * @}
 */
uint32_t USBD_USR_GetTimestamp_LastRead(void) {
	return timestamp_last_read;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
