
#include "ff.h"
#include "bsp_driver_sd.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINT_RAW(fmt,...)	printf( fmt,##__VA_ARGS__ )
#define PRINT_INF(fmt,...)	printf( fmt"\r\n",##__VA_ARGS__ )
#define PRINT_ERR(fmt,...)	printf( "Error: "fmt"\r\n",##__VA_ARGS__ )
#else
#define PRINT_RAW(fmt,...)	
#define PRINT_INF(fmt,...)	
#define PRINT_ERR(fmt,...)	
#endif

static int disk_status = 0; // 0: Not initialized, 1: Initialized 2: Protected
int MMC_disk_status( void ){
	return disk_status;
}

int MMC_disk_initialize( void ){
	int err = (int)BSP_SD_Init();
	if( err == MSD_OK ){
		disk_status = 1;
	}
	return err;
}


int MMC_disk_read(
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
){
	while( BSP_SD_GetCardState() == SD_TRANSFER_BUSY );
	int result = BSP_SD_ReadBlocks( buff, sector, count, 100 );
	if( result == MSD_OK ){
		return (int)count;
	}else{
		return 0;
	}
}

int MMC_disk_write(
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */ 
){
	while( BSP_SD_GetCardState() == SD_TRANSFER_BUSY );
	int result = BSP_SD_WriteBlocks( buff, sector, count, 100 );
	if( result == MSD_OK ){
		return (int)count;
	}else{
		return 0;
	}
}