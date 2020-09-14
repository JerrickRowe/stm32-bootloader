
#include "ff.h"

int MMC_disk_status( void ){

	return 0;
}

int MMC_disk_initialize( void ){
	
	return 0;
}


int MMC_disk_read( 
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
){
	
	return (int)count;
}

int MMC_disk_write(
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */ 
){
	
	return (int)count;
}