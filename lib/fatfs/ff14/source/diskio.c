/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "bsp_driver_sd.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_FLASH	2	/* Example: Map flash to physical drive 2 */
#define DEV_USB		3	/* Example: Map USB MSD to physical drive 3 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	// case DEV_RAM :
	// 	result = RAM_disk_status();

	// 	// translate the reslut code here

	// 	return stat;

	case DEV_MMC :
		result = MMC_disk_status();
		// translate the reslut code here
		switch( result ){
		case 0: return STA_NOINIT;
		case 1: return 0;
		case 2: return STA_PROTECT;
		}

	// case DEV_USB :
	// 	result = USB_disk_status();

	// 	// translate the reslut code here

	// 	return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	switch (pdrv) {
	// case DEV_RAM :
	// 	result = RAM_disk_initialize();

	// 	// translate the reslut code here

	// 	return stat;

	case DEV_MMC :
		result = MMC_disk_initialize();
		// translate the reslut code here
		switch(result){
		case MSD_OK:
			stat = RES_OK;
			break;
		default:
			stat = RES_ERROR;
			break; 
		}
		return stat;

	// case DEV_USB :
	// 	result = USB_disk_initialize();

	// 	// translate the reslut code here

	// 	return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	// case DEV_RAM :
	// 	// translate the arguments here

	// 	result = RAM_disk_read(buff, sector, count);

	// 	// translate the reslut code here

	// 	return res;

	case DEV_MMC :
		// translate the arguments here

		result = MMC_disk_read(buff, sector, count);

		// translate the reslut code here
		if( result == (int)count ){
			res = RES_OK;
		}else{
			res = RES_ERROR;
		}
		return res;

	// case DEV_USB :
	// 	// translate the arguments here

	// 	result = USB_disk_read(buff, sector, count);

	// 	// translate the reslut code here

	// 	return res;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	switch (pdrv) {
	// case DEV_RAM :
	// 	// translate the arguments here

	// 	result = RAM_disk_write(buff, sector, count);

	// 	// translate the reslut code here

	// 	return res;

	case DEV_MMC :
		// translate the arguments here
		result = MMC_disk_write(buff, sector, count);

		// translate the reslut code here
		if( result == (int)count ){
			res = RES_OK;
		}else{
			res = RES_ERROR;
		}
		return res;

	// case DEV_USB :
	// 	// translate the arguments here

	// 	result = USB_disk_write(buff, sector, count);

	// 	// translate the reslut code here

	// 	return res;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	int result;
    BSP_SD_CardInfo CardInfo;
	switch (pdrv) {
	// case DEV_RAM :

	// 	// Process of the command for the RAM drive

	// 	return res;

	case DEV_MMC :

		switch( cmd ){
		case CTRL_SYNC			:	//0	/* Complete pending write process (needed at FF_FS_READONLY == 0) */
		#if FF_FS_READONLY == 0
			while( BSP_SD_GetCardState() == SD_TRANSFER_BUSY ){

			}
//			res = RES_NOTRDY;
			res = RES_OK;
		#else
			res = RES_OK;
		#endif
			break;
		case GET_SECTOR_COUNT	:{	//1	/* Get media size (needed at FF_USE_MKFS == 1) */
            BSP_SD_GetCardInfo(&CardInfo);
            *(DWORD*)buff = CardInfo.LogBlockNbr;
			res = RES_OK;
			break;
		}case GET_SECTOR_SIZE	:	//2	/* Get sector size (needed at FF_MAX_SS != FF_MIN_SS) */
            BSP_SD_GetCardInfo(&CardInfo);
            *(DWORD*)buff = CardInfo.BlockSize;
			res = RES_OK;
			break;
		case GET_BLOCK_SIZE		:	//3	/* Get erase block size (needed at FF_USE_MKFS == 1) */
            BSP_SD_GetCardInfo(&CardInfo);
            *(DWORD*)buff = CardInfo.LogBlockSize/CardInfo.BlockSize;
			res = RES_OK;
			break;
		case CTRL_TRIM			:	//4	/* Inform device that the data on the block of sectors is no longer used (needed at FF_USE_TRIM == 1) */
			res = RES_NOTRDY;
			break;
		}
		// Process of the command for the MMC/SD card
		return res;

	// case DEV_USB :

	// 	// Process of the command the USB drive

	// 	return res;
	}

	return RES_PARERR;
}

