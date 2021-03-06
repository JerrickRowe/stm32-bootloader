/**
 *******************************************************************************
 * STM32L4 Bootloader
 *******************************************************************************
 * @author Akos Pasztor
 * @file   main.c
 * @brief  Main program
 *	       This file demonstrates the usage of the bootloader.
 *
 * @see    Please refer to README for detailed information.
 *******************************************************************************
 * Copyright (c) 2020 Akos Pasztor.                     https://akospasztor.com
 *******************************************************************************
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_rcc.h"
#include "main.h"
#include "bootloader.h"
#include "tea.h"
#include "ff.h"
#include "led.h"
#include "usb_bsp.h"
#include "bsp_WS2812x.h"
#include "bsp_driver_sd.h"
#include "remote_control.h"

#include "Indicator.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "git.h"

#include "testcases.h"

#include "usbd_msc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#include "bsp_power.h"

#define DECRYPTION_ENABLED 1

#define OWL_RC_UPGRADE_FILESIZE	  (0xA000)
#define OWL_UPGRADE_FILESIZE	  ((APP1_SIZE) + 4u)
#define COMBINED_UPGRADE_FILESIZE (OWL_UPGRADE_FILESIZE + OWL_RC_UPGRADE_FILESIZE)

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

/* Private variables ---------------------------------------------------------*/
static uint8_t BTNcounter = 0;

#define VER 1

#define REV 0

#define PRJ_STR "FlyFire-bootloader"

/* External variables --------------------------------------------------------*/
char	   SDPath[4] = "SD:"; /* SD logical drive path */
FATFS	   SDFatFs;			  /* File system object for SD logical drive */
FIL		   SDFile;			  /* File object for SD */
static FIL upgrade_file;

IWDG_HandleTypeDef iwdg;
/* Private variables ---------------------------------------------------------*/
bool renew_freefile = false;

/* Function prototypes -------------------------------------------------------*/

void Enter_Bootloader(void);
void SD_DeInit(void);
void SD_Eject(void);
void GPIO_Startup(void);
void Console_Init(void);
void Console_DeInit(void);
void GPIO_DeInit(void);
void SystemClock_Config(void);
void Error_Handler(void);
void print(const char* str);

/* Private functions ---------------------------------------------------------*/
const char* months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun",
						 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
void		getBuildTimeStr_YYYYMMDDhhss(char* pDes) {
	   char			 strDate[] = __DATE__;
	   char			 strTime[] = __TIME__;
	   unsigned char i;
	   unsigned int	 month, day, hour, minute, year;

	   *(strTime + 2) = 0;
	   *(strTime + 5) = 0;
	   hour			  = atoi(strTime);

	   minute = atoi(strTime + 3);
	   year	  = atoi(strDate + 9);

	   *(strDate + 6) = 0;
	   day			  = atoi(strDate + 4);
	   *(strDate + 3) = 0;
	   for(i = 0; i < 12; i++) {
		   if(!strcmp(strDate, months[i])) {
			   month = i + 1;
			   sprintf(pDes, "20%02d%02d%02d%02d%02d", year, month, day, hour, minute);
			   return;
		   }
	   }
}

void print_logo(void) {
	printf("\n\n\n\n\n\r"
		   "\x1B[0;39m"
		   "______  _        ______  _             \r\n"
		   "|  ___|| |       |  ___|(_)            \r\n"
		   "| |_   | | _   _ | |_    _  _ __   ___ \r\n"
		   "|  _|  | || | | ||  _|  | || '__| / _ \\\r\n"
		   "| |    | || |_| || |    | || |   |  __/\r\n"
		   "\\_|    |_| \\__, |\\_|    |_||_|    \\___|\r\n"
		   "            __/ |                      \r\n"
		   "           |___/                       \r\n");
}

const char* get_version_string(void) {
	static char str[500];
	snprintf(str, sizeof(str), "%s-V%d.%d-%s-", PRJ_STR, VER, REV, GIT_COMMIT_HEAD);
	getBuildTimeStr_YYYYMMDDhhss(str + strlen(str));
	return str;
}

/**
 * @brief This function is called to increment  a global variable "uwTick"
 *        used as application time base.
 * @note In the default implementation, this variable is incremented each 1ms
 *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
 *      implementations in user file.
 * @retval None
 */
static uint32_t tick;
inline void		HAL_IncTick(void) {
	tick++;
	if(tick % 25 == 0) {
		led_poll();
	}
}

/**
 * @brief Provides a tick value in millisecond.
 * @note This function is declared as __weak to be overwritten in case of other
 *       implementations in user file.
 * @retval tick value
 */
uint32_t HAL_GetTick(void) {
	return tick;
}

FRESULT scan_files(char* path) { /* Start node to be scanned (***also used as work area***) */
	FRESULT		   res;
	DIR			   dir;
	UINT		   i;
	static FILINFO fno;

	res = f_opendir(&dir, path); /* Open the directory */
	if(res == FR_OK) {
		for(;;) {
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if(res != FR_OK || fno.fname[0] == 0)
				break;				   /* Break on error or end of dir */
			if(fno.fattrib & AM_DIR) { /* It is a directory */
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				res = scan_files(path); /* Enter the directory */
				if(res != FR_OK)
					break;
				path[i] = 0;
			} else { /* It is a file. */
				PRINT_RAW("%u/%02u/%02u %02u:%02u "
						  "%4u%c "
						  "%s/%s\r\n",
						  (fno.fdate >> 9) + 1980,
						  fno.fdate >> 5 & 15,
						  fno.fdate & 31,
						  fno.ftime >> 11,
						  fno.ftime >> 5 & 63,
						  fno.fsize > 1024
							  ? (fno.fsize > 1048576 ? fno.fsize / 1048576 : fno.fsize / 1024)
							  : fno.fsize,
						  fno.fsize > 1024 ? (fno.fsize > 1048576 ? 'M' : 'K') : 'B',
						  path,
						  fno.fname);
			}
		}
		f_closedir(&dir);
	}
	return res;
}

int ls(int argc, char** argv) {
	FRESULT res;
	DIR		dir;
	FILINFO fno;
	char	path[100] = "SD:";
	int		i;
	scan_files(path);
	return 0;
}

int testcase_4(void);

bool MountFilesystem(void) {
	// testcase_4();

	PRINT_RAW("Mount SD card and FatFS........");
	FRESULT	 res;
	UINT	 num;
	uint8_t	 i;
	uint8_t	 status;
	uint64_t data;
	uint32_t cntr;
	uint32_t addr;
	char	 msg[40] = { 0x00 };

	/* Mount SD card */
	res = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1);
	if(res != FR_OK) {
		/* f_mount failed */
		PRINT_INF("MNT_FAILED [0x%02X]", res);
		return false;
	}
	PRINT_INF("DONE");
	return true;
}

void led_toggling(uint32_t rgb1, uint32_t rgb2, uint32_t interval_ms) {
	static uint32_t toggle_timestamp = 0;
	if(HAL_GetTick() - toggle_timestamp > interval_ms) {
		toggle_timestamp   = HAL_GetTick();
		static bool toggle = false;
		if(toggle) {
			toggle = false;
			led_setAllRGB(rgb1);
		} else {
			toggle = true;
			led_setAllRGB(rgb2);
		}
	}
}

void led_filling(uint32_t rgb, uint32_t interval_ms) {
	static uint32_t timestamp = 0;
	if(HAL_GetTick() - timestamp > interval_ms) {
		timestamp		= HAL_GetTick();
		static int step = 0;
		switch(step) {
		case 0: led_setRGB(0, rgb); break;
		case 1: led_setRGB(1, rgb); break;
		case 2: led_setRGB(2, rgb); break;
		case 3: led_setAllRGB(0); break;
		}
		if(++step > 3) {
			step = 0;
		}
	}
}

void Copy_App2_To_App1(void) {
	PRINT_RAW("Copy App2 to App1.");

	Bootloader_FlashBegin();

	uint32_t cnt = 0;

	while(cnt < ((APP2_SIZE + 4) / 4)) {
		uint32_t data = *(uint32_t*)(APP2_ADDRESS + (cnt * 4));
		uint8_t	 status;
		status = Bootloader_FlashNextWord(data);
		if(cnt % 9830 == 0) {
			PRINT_RAW(".");
		}
		cnt++;
	}

	Bootloader_FlashEnd();

#if(USE_CHECKSUM)
	// Verify application checksum
	if(Bootloader_IsApp2ChecksumValid() != BL_OK) {
		PRINT_RAW("FAILED");
	} else {
		PRINT_RAW("DONE");
	}
#endif
	PRINT_RAW("\r\n");
}

void FeedGlobalWatchdog(void) {
	HAL_IWDG_Refresh(&iwdg);
	if(!bsp_power_isExtPowerOnline()) {
		bsp_power_ReleasePower();
	}
}

bool Copy_File_To_App1(FIL* fp) {
	FRESULT res;
	UINT	br = 0;
	uint8_t buff[512];
	PRINT_INF("Copy file to App1:");
	PRINT_RAW("  Copy.");
	f_rewind(fp);
	uint32_t scan	  = 0;
	uint32_t boundary = (APP1_SIZE + 4) / 512;
	led_setAllRGB(RGB(0xAD, 0x3D, 0x00));
	HAL_Delay(100);
	iwdg.Init.Prescaler = IWDG_PRESCALER_128;
	HAL_IWDG_Init(&iwdg);
	FeedGlobalWatchdog();
	Bootloader_FlashBegin();
	for(scan = 0; scan < boundary; scan++) {
		FeedGlobalWatchdog();
		res = f_read(fp, buff, 512, &br);
		if(res != FR_OK || br != 512) {
			PRINT_ERR("  f_read error [0x%02X] byte read=%d", res, br);
			Bootloader_FlashEnd();
			return false;
		}
		// Decryption
#if DECRYPTION_ENABLED
		btea_decrpyt(buff, TEA_key);
#endif
		// Copy
		uint32_t cnt = 0;
		for(cnt = 0; cnt < 512 / 4; cnt++) {
			uint32_t data = ((uint32_t*)buff)[cnt];
			uint8_t	 status;
			status = Bootloader_FlashNextWord(data);
			if(status != BL_OK) {
				PRINT_RAW("Failed\r\n");
				Bootloader_FlashEnd();
				return false;
			}
		}
		if(scan % 64 == 0) {
			PRINT_RAW(".");
		}
		led_filling(RGB(0xAD, 0x3D, 0x00), 150);
	}
	Bootloader_FlashEnd();
	iwdg.Init.Prescaler = IWDG_PRESCALER_16;
	HAL_IWDG_Init(&iwdg);
	FeedGlobalWatchdog();
	PRINT_RAW("Done\r\n");
	return true;
}

bool VerifyUpgradeFile(FIL* fp) {
	FRESULT res;
	UINT	br = 0;
	uint8_t buff[512];
	PRINT_INF("Verifying upgrade file:");

	if(f_size(fp) < OWL_UPGRADE_FILESIZE) {
		PRINT_ERR("  Incorrect file size");
		goto ERROR;
	}

	CRC_HandleTypeDef CrcHandle;
	volatile uint32_t calculatedCrc = 0;
	__HAL_RCC_CRC_CLK_ENABLE();
	CrcHandle.Instance = CRC;
	CrcHandle.State	   = HAL_CRC_STATE_RESET;
	if(HAL_CRC_Init(&CrcHandle) != HAL_OK) {
		PRINT_ERR("  CRC init error");
		goto ERROR;
	}

	PRINT_RAW("  CRC..");
	f_rewind(fp);
	uint32_t scan	  = 0;
	uint32_t boundary = (APP1_SIZE + 4) / 512;
	for(scan = 0; scan < boundary; scan++) {
		FeedGlobalWatchdog();
		res = f_read(fp, buff, 512, &br);
		if(res != FR_OK || br != 512) {
			PRINT_ERR("f_read error [0x%02X] byte read=%d", res, br);
			goto ERROR;
		}
		// Decryption
#if DECRYPTION_ENABLED
		btea_decrpyt(buff, TEA_key);
#endif
		// Verification
		if(scan == 0) {	 // First unit
			if(((*(uint32_t*)buff - RAM_BASE) > RAM_SIZE)) {
				PRINT_RAW("........................ABORT\r\n");
				PRINT_ERR("  Bad MSP[0x%08X]", *(uint32_t*)buff);
				goto ERROR;
			}
			calculatedCrc = HAL_CRC_Accumulate(&CrcHandle, (uint32_t*)buff, 512 / 4);
		} else if(scan == boundary - 1) {  // Last unit
			calculatedCrc = HAL_CRC_Accumulate(&CrcHandle, (uint32_t*)buff, (512 - 4) / 4);
		} else {
			calculatedCrc = HAL_CRC_Accumulate(&CrcHandle, (uint32_t*)buff, 512 / 4);
		}
		if(scan % 64 == 0) {
			PRINT_RAW(".");
		}
		led_filling(RGB(0xAD, 0x3D, 0x00), 150);
	}

	uint32_t fileCrc = *(uint32_t*)&(buff[512 - 4]);
	if(fileCrc != calculatedCrc) {
		PRINT_RAW("CRC unmatched. [%08X/%08X]", fileCrc, calculatedCrc);
		goto ERROR;
	} else {
		PRINT_RAW("CRC matched. [%08X]\r\n", fileCrc);
	}

	PRINT_INF("  %s is verified", UPGRADE_FILENAME);
	return true;
ERROR:
	PRINT_INF("  %s cannot be used", UPGRADE_FILENAME);
	__HAL_RCC_CRC_FORCE_RESET();
	__HAL_RCC_CRC_RELEASE_RESET();
	return false;
}

bool IsUpgradeFileExists(void) {
	FRESULT res;
	FILINFO fno;
	PRINT_RAW("Finding \"%s\"......", UPGRADE_FILENAME);
	res = f_stat(UPGRADE_FILENAME, &fno);
	if(res != FR_OK) {
		PRINT_INF("NOTFOUND");
		return false;
	}
	PRINT_INF("FOUND");
	PRINT_INF("  %s %dbytes", fno.fname, fno.fsize);
	return true;
}

bool IsBackupFileExists(void) {
	FRESULT res;
	FILINFO fno;
	PRINT_RAW("Finding \"%s\".......", BACKUP_FILENAME);
	res = f_stat(BACKUP_FILENAME, &fno);
	if(res != FR_OK) {
		PRINT_INF("NOTFOUND");
		return false;
	}
	PRINT_INF("FOUND");
	PRINT_INF("  %s %dbytes", fno.fname, fno.fsize);
	return true;
}

bool IsApp1Jumpable(void) {
	PRINT_RAW("Checking for App 1.............");
	// Check if there is application in user flash area
	if(Bootloader_CheckForApp1() == BL_OK) {
#if(USE_CHECKSUM)
		// Verify application checksum
		if(Bootloader_IsApp1ChecksumValid() != BL_OK) {
			PRINT_RAW("BAD\r\n");
			return false;
		} else {
			PRINT_RAW("PASS\r\n");
			return true;
		}
#endif
	} else {
		PRINT_RAW("NOTFOUND\r\n");
		return false;
	}
}

void printMem(const char* pTag, void* addr, unsigned int n) {
	int i = 0;
	if(!n) {
		return;
	}
	printf("\r\n\x1B[1;32m[M] Addr=0x%X "
		   "%s"
		   "\x1B[0;32m"
		   "\r\n"
		   "\x1B[4;92m"
		   "     | ",
		   (uint32_t)addr,
		   pTag);
	for(i = 0; i < ((n > 16) ? 16 : n); i++) {
		printf(" %X|", i, i + 16);
	}
	for(i = 0; i < ((n < 16) ? n : (((n - 1) / 16 + 1) * 16)); i++) {
		if(i % 16 == 0) {
			printf("\x1B[4;92m"
				   "\r\n"
				   "%4XH|"
				   "\x1B[0;39m"
				   " ",
				   i);	//\x1B[4;32m
		}
		if(i < n) {
			printf("%02X ", 0x000000FF & ((uint8_t*)addr)[i]);
		} else {
			printf("   ");
		}
	}
	printf("\x1B[0;39m"
		   "\r\n\n");
}

static uint8_t buff[4096];
bool		   GenerateRC_UpgradeFile(FIL* fp) {
	  FRESULT res;
	  UINT	  br, bw;
	  FIL	  rc_upgrade_file;
	  PRINT_INF("Generating " RC_UPGRADE_FILENAME);
	  res = f_open(&rc_upgrade_file, RC_UPGRADE_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
	  if(res != FR_OK) {
		  PRINT_INF("Failed to create " RC_UPGRADE_FILENAME);
		  return false;
	  }
	  res = f_lseek(fp, OWL_UPGRADE_FILESIZE);
	  if(res != FR_OK) {
		  PRINT_INF("Failed to seek 0x%08X", OWL_UPGRADE_FILESIZE);
		  return false;
	  }
	  while(!f_eof(fp)) {
		  FeedGlobalWatchdog();
		  res = f_read(fp, buff, sizeof(buff), &br);
		  if(res != FR_OK) {
			  PRINT_ERR("Error while reading [%d,%d]", res, br);
			  return false;
		  }
		  res = f_write(&rc_upgrade_file, buff, br, &bw);
		  if(res != FR_OK) {
			  PRINT_ERR("Error while writing [%d,%d]", res, bw);
			  return false;
		  }
		  led_filling(RGB(0xAD, 0x3D, 0x00), 150);
	  }
	  f_close(&rc_upgrade_file);
	  return true;
}

bool UpgradeFromSD(void) {
	FRESULT res;

	if(IsUpgradeFileExists() == false) {
		return false;
	}
	PRINT_RAW("Opening \"%s\"......", UPGRADE_FILENAME);
	res = f_open(&upgrade_file, UPGRADE_FILENAME, FA_OPEN_EXISTING | FA_READ);
	if(res != FR_OK) {
		PRINT_RAW("FAILED\r\n");
		return false;
	}
	PRINT_RAW("DONE\r\n");

	if(RC_Presented()) {
		uint32_t timestamp = HAL_GetTick();
		PRINT_INF("Send upgrade status to RC");
		while(HAL_GetTick() - timestamp < 1000) {
			RC_SimpleConnection(MAIN_STA_BL_UPGRADING);
		}
	}

	//	UINT br = 0;
	//	static uint8_t buff[512];
	//	res = f_read( &upgrade_file, buff, 512, &br );
	//	printMem( "upgrade_file #0", buff, 512 );
	if(VerifyUpgradeFile(&upgrade_file) == false) {
		return false;
	}
	// Generate RC upgrade file
	if(RC_Presented() && f_size(&upgrade_file) > OWL_UPGRADE_FILESIZE) {
		GenerateRC_UpgradeFile(&upgrade_file);
	}
	if(Copy_File_To_App1(&upgrade_file) == false) {
		return false;
	}
	f_close(&upgrade_file);

	led_setAllRGB(RGB(0xAD, 0x3D, 0x00));

	if(IsApp1Jumpable() == false) {
		return false;
	}

	PRINT_RAW("Backing up......");
	res = f_unlink(BACKUP_FILENAME);
	res = f_rename(UPGRADE_FILENAME, BACKUP_FILENAME);
	if(res != FR_OK) {
		PRINT_RAW("FAILED\r\n");
	} else {
		PRINT_RAW("DONE\r\n");
		f_chmod(BACKUP_FILENAME, AM_ARC | AM_SYS | AM_HID, AM_ARC | AM_SYS | AM_HID);
	}

	PRINT_RAW("Backup filename: %s\r\n", BACKUP_FILENAME);
	return true;
}

void LaunchApp1(void) {
	do {
		if(renew_freefile != true) {
			break;
		}
		FRESULT		res;
		FILINFO		fno;
		const char* path_freefile = "SD:/FREESPAC.E";
		PRINT_RAW("Freefile should be renewed.....");
		res = f_stat(path_freefile, &fno);
		if(res != FR_OK) {
			PRINT_INF("NOTFOUND");
			break;
		}
		PRINT_RAW("FOUND\r\n");
		PRINT_RAW("  %s %dbytes\r\n", fno.fname, fno.fsize);
		PRINT_RAW("  Delete %s........", path_freefile);
		res = f_chmod(path_freefile, 0, AM_RDO);
		if(res != FR_OK) {
			PRINT_RAW("chmod failed....");
		}
		res = f_unlink(path_freefile);
		if(res == FR_OK) {
			PRINT_RAW("Done\r\n");
		} else {
			PRINT_RAW("Failed\r\n");
		}
	} while(0);
	PRINT_RAW("Launching App 1.\r\n");
	SD_DeInit();
	GPIO_DeInit();
	bsp_power_HoldPower();
	HAL_Delay(100);
	// Launch application
	Bootloader_JumpToApp1();
}

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
static bool									  USB_OTG(void) {
	  static enum {
		  STARTUP = 0,
		  CHECK_USB_VOLTAGE,
		  INIT_USB,
		  RUNNING,
	  } sta = STARTUP;
	  switch(sta) {
	  case STARTUP: {
		  PRINT_INF("USB-OTG Startup");

		  sta = CHECK_USB_VOLTAGE;
	  }
	  case CHECK_USB_VOLTAGE: {
		  if(1) {
			  led_setAllRGB(RGB(0, 0, 50));
			  sta = INIT_USB;
			  return true;
		  }
		  break;
	  }
	  case INIT_USB: {
		  USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
				  USB_OTG_HS_CORE_ID,
#else
				  USB_OTG_FS_CORE_ID,
#endif
				  &USR_desc,
				  &USBD_MSC_cb,
				  &USR_cb);
		// The 2nd initialization gets device to be discovered by PC
		USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
				  USB_OTG_HS_CORE_ID,
#else
				  USB_OTG_FS_CORE_ID,
#endif
				  &USR_desc,
				  &USBD_MSC_cb,
				  &USR_cb);
		sta = RUNNING;
		return true;
	}
	case RUNNING: {
		if(!USBD_USR_IsReleased()) {
			return true;
		}
		break;
	}
	default: {
		break;
	}
	}
	// PRINT_INF( "USB-OTG released" );
	// led_setAllRGB( RGB(0,0,0) );
	// float usb_voltage;
	// do{
	// 	usb_voltage = bsp_power_GetExtPowerVoltage();
	// 	// PRINT_INF( "usb_voltage = %.2f", usb_voltage );
	// 	HAL_Delay(200);
	// }while( usb_voltage > 3.0f );
	// led_setAllRGB( RGB(0,0,0) );
	// sta = STARTUP;
	return false;
}

bool IsSDcardOnline(void) {
	int err = (int)BSP_SD_Init();
	if(err == MSD_OK) {
		return true;
	}
	return false;
}

bool IsKeyFilesPresent(void) {
	FRESULT		res;
	FILINFO		fno;
	static bool checkingIsNeeded = false;
	if(HAL_GetTick() - USBD_USR_GetTimestamp_LastWrite() < 1000) {
		checkingIsNeeded = true;
		return false;
	}
	if(checkingIsNeeded == false) {
		return false;
	}
	if(HAL_GetTick() - USBD_USR_GetTimestamp_LastRead() < 1000) {
		return false;
	}

	// __disable_irq();
	// ls( 1, "SD:" );
	// __enable_irq();

	PRINT_RAW("Finding \"%s\"......", UPGRADE_FILENAME);
	//	__disable_irq();
	USB_OTG_BSP_DisableInterrupt(NULL);
	/* Mount SD card */
	res = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1);
	if(res != FR_OK) {
		/* f_mount failed */
		PRINT_INF("MNT_FAILED [0x%02X]", res);
		return false;
	}
	res = f_stat(UPGRADE_FILENAME, &fno);
	//	__enable_irq();
	USB_OTG_BSP_EnableInterrupt(NULL);
	checkingIsNeeded = false;
	if(res != FR_OK) {
		PRINT_INF("NOTFOUND");
		return false;
	}
	PRINT_INF("FOUND");
	PRINT_INF("  %s %dbytes", fno.fname, fno.fsize);
	if(fno.fsize < OWL_UPGRADE_FILESIZE) {
		PRINT_ERR("  Incorrect file size");
		return false;
	}
	return true;
}

/* Main ----------------------------------------------------------------------*/
int main(void) {
	HAL_Init();
	GPIO_Startup();
	USB_OTG_BSP_DisableInterrupt(NULL);

#if DEBUG
	__HAL_DBGMCU_FREEZE_IWDG();
#endif

	iwdg.Instance		= IWDG;
	iwdg.Init.Prescaler = IWDG_PRESCALER_16;
	iwdg.Init.Reload	= 0xFFF;
	HAL_IWDG_Init(&iwdg);
	FeedGlobalWatchdog();

	Console_Init();

	while(HAL_GetTick() < 50) {
		FeedGlobalWatchdog();
	}

	bsp_power_HoldPower();
	FeedGlobalWatchdog();

	led_init();

	print_logo();
	printf("%s\r\n", get_version_string());

	led_allon();
	led_setAllRGB(RGB(255, 0, 0));
	HAL_Delay(200);
	led_setAllRGB(RGB(0, 255, 0));
	HAL_Delay(200);
	led_setAllRGB(RGB(0, 0, 255));
	//    HAL_Delay(200);
	//	led_setAllRGB( RGB(0,0,0) );
	FeedGlobalWatchdog();

	bsp_InitE22();
	_RC_bsp_RecoverFromUpgradeMode();

	//	while(1){
	//		FeedGlobalWatchdog();
	////		RC_SimpleConnection( MAIN_STA_BL_NOAPP );
	//		RC_SimpleConnection( MAIN_STA_USB_STORAGE );
	//	}

	if(MountFilesystem() == true) {
		//		PRINT_INF(	"List files:" );
		//		ls( 1, "SD:" );

		// Run USB-OTG service
		float usb_volt = bsp_power_GetExtPowerVoltage();
		PRINT_INF("USB voltage = %.2fV", usb_volt);
		if(usb_volt > 3.0f && usb_volt < 7.0f) {
			USB_OTG_BSP_EnableInterrupt(NULL);
			while(USB_OTG()) {
				FeedGlobalWatchdog();
				if(IsKeyFilesPresent() && !USBD_USR_IsStorageActive() && HAL_GetTick() > 5000) {
					USB_OTG_BSP_DisableInterrupt(NULL);
					HAL_Delay(100);
					// PRINT_INF("Upgrade now");
					// Try to upgrade with new file

					// Try to upgrade from SD card
					if(UpgradeFromSD() == true) {
						PRINT_INF("Upgrade succeed");
						// renew_freefile = true;
						LaunchApp1();
					} else {
						PRINT_ERR("Upgrade failed");
						USB_OTG_BSP_EnableInterrupt(NULL);
						continue;
					}
					break;
				}
				if(!bsp_power_isExtPowerOnline()) {
					bsp_power_ReleasePower();
				}
				if(USBD_USR_IsStorageActive()) {
					led_toggling(RGB(0, 0, 50), RGB(0, 0, 100), 50);
				} else {
					led_setAllRGB(RGB(0, 0, 50));
				}
				RC_SimpleConnection(MAIN_STA_USB_STORAGE);
			}

			//			HAL_Delay(200);
			//			led_setAllRGB( RGB(0,0,0) );
			//			while(1){
			//				if( !bsp_power_isExtPowerOnline() ){
			//					bsp_power_ReleasePower();
			//				}
			//			}
		}

		// No need to upgrade or upgrade failed.
		if(IsApp1Jumpable() == true) {
			LaunchApp1();
		}

		// App is broken, try to recover from backup file
		if(IsBackupFileExists() == true) {
			FRESULT res;
			PRINT_RAW("Recover backup file......");
			res = f_rename(BACKUP_FILENAME, UPGRADE_FILENAME);
			if(res == FR_OK) {
				PRINT_RAW("DONE\r\n");
				if(UpgradeFromSD() == true) {
					// renew_freefile = true;
					LaunchApp1();
				}
			} else {
				PRINT_RAW("FAILED\r\n");
			}
		}
	} else {
		// Run USB service if sdcard is online
		if(IsSDcardOnline()) {
			PRINT_INF("SDcard is online");
			// Run USB-OTG service
			float usb_volt = bsp_power_GetExtPowerVoltage();
			PRINT_INF("USB voltage = %.2fV", usb_volt);
			if(usb_volt > 3.0f && usb_volt < 7.0f) {
				while(USB_OTG()) {
					RC_SimpleConnection(MAIN_STA_USB_STORAGE);
					FeedGlobalWatchdog();
				}
			}
		}
		// Try to run app without SD card.
		if(IsApp1Jumpable() == true) {
			LaunchApp1();
		}
	}

	// Fatal: no app, no backup file available, upgrade is needed.
	PRINT_INF("No application available.");
	while(1) {
		if(USB_OTG()) {
			RC_SimpleConnection(MAIN_STA_USB_STORAGE);
			if(USBD_USR_IsStorageActive()) {
				led_toggling(RGB(0, 0, 50), RGB(0, 0, 100), 50);
			} else {
				led_setAllRGB(RGB(0, 0, 50));
			}
		} else {
			led_toggling(RGB(20, 20, 20), RGB(0, 0, 0), 150);
			RC_SimpleConnection(MAIN_STA_BL_NOAPP);
		}
		FeedGlobalWatchdog();
	}
}

/*** Bootloader ***************************************************************/

void Enter_Bootloader(void) {
	//    FRESULT  res;
	//    UINT     num;
	//    uint8_t  i;
	//    uint8_t  status;
	//    uint64_t data;
	//    uint32_t cntr;
	//    uint32_t addr;
	//    char     msg[40] = {0x00};

	//    /* Check for flash write protection */
	//    if(Bootloader_GetProtectionStatus() & BL_PROTECTION_WRP)
	//    {
	//        PRINT_INF("Application space in flash is write protected.");
	//        PRINT_INF("Press button to disable flash write protection...");
	//        LED_R_ON();
	//        for(i = 0; i < 100; ++i)
	//        {
	//            LED_Y_TG();
	//            HAL_Delay(50);
	//            if(IS_BTN_PRESSED())
	//            {
	//                PRINT_INF("Disabling write protection and generating system "
	//                      "reset...");
	//                Bootloader_ConfigProtection(BL_PROTECTION_NONE);
	//            }
	//        }
	//        LED_R_OFF();
	//        LED_Y_OFF();
	//        PRINT_INF("Button was not pressed, write protection is still active.");
	//        PRINT_INF("Exiting Bootloader.");
	//        return;
	//    }

	//    /* Initialize SD card */
	//    if(SD_Init())
	//    {
	//        /* SD init failed */
	//        PRINT_INF("SD card cannot be initialized.");
	//        return;
	//    }

	//    /* Mount SD card */
	//    res = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1);
	//    if(res != FR_OK)
	//    {
	//        /* f_mount failed */
	//        PRINT_INF("SD card cannot be mounted.");
	//        sprintf(msg, "FatFs error code: %u", res);
	//        PRINT_INF("%s",msg);
	//        return;
	//    }
	//    PRINT_INF("SD mounted.");

	//    /* Open file for programming */
	//    res = f_open(&SDFile, CONF_FILENAME, FA_READ);
	//    if(res != FR_OK)
	//    {
	//        /* f_open failed */
	//        PRINT_INF("File cannot be opened.");
	//        sprintf(msg, "FatFs error code: %u", res);
	//        PRINT_INF("%s",msg);

	//        SD_Eject();
	//        PRINT_INF("SD ejected.");
	//        return;
	//    }
	//    PRINT_INF("Software found on SD.");

	//    /* Check size of application found on SD card */
	//    if(Bootloader_CheckSize(f_size(&SDFile)) != BL_OK)
	//    {
	//        PRINT_INF("Error: app on SD card is too large.");

	//        f_close(&SDFile);
	//        SD_Eject();
	//        PRINT_INF("SD ejected.");
	//        return;
	//    }
	//    PRINT_INF("App size OK.");

	//    /* Step 1: Init Bootloader and Flash */
	//    Bootloader_Init();

	//    /* Step 2: Erase Flash */
	//    PRINT_INF("Erasing flash...");
	//    LED_Y_ON();
	//    Bootloader_Erase();
	//    LED_Y_OFF();
	//    PRINT_INF("Flash erase finished.");

	//    /* If BTN is pressed, then skip programming */
	//    if(IS_BTN_PRESSED())
	//    {
	//        PRINT_INF("Programming skipped.");

	//        f_close(&SDFile);
	//        SD_Eject();
	//        PRINT_INF("SD ejected.");
	//        return;
	//    }

	//    /* Step 3: Programming */
	//    PRINT_INF("Starting programming...");
	//    LED_Y_ON();
	//    cntr = 0;
	//    Bootloader_FlashBegin();
	//    do
	//    {
	//        data = 0xFFFFFFFFFFFFFFFF;
	//        res   = f_read(&SDFile, &data, 8, &num);
	//        if(num)
	//        {
	//            status = Bootloader_FlashNext(data);
	//            if(status == BL_OK)
	//            {
	//                cntr++;
	//            }
	//            else
	//            {
	//                sprintf(msg, "Programming error at: %lu byte", (cntr * 8));
	//				PRINT_INF("%s",msg);

	//                f_close(&SDFile);
	//                SD_Eject();
	//                PRINT_INF("SD ejected.");

	//                LED_G_OFF();
	//                LED_Y_OFF();
	//                return;
	//            }
	//        }
	//        if(cntr % 256 == 0)
	//        {
	//            /* Toggle green LED during programming */
	//            LED_G_TG();
	//        }
	//    } while((res == FR_OK) && (num > 0));

	//    /* Step 4: Finalize Programming */
	//    Bootloader_FlashEnd();
	//    f_close(&SDFile);
	//    LED_G_OFF();
	//    LED_Y_OFF();
	//    PRINT_INF("Programming finished.");
	//    sprintf(msg, "Flashed: %lu bytes.", (cntr * 8));
	//    PRINT_INF("%s",msg);

	//    /* Open file for verification */
	//    res = f_open(&SDFile, CONF_FILENAME, FA_READ);
	//    if(res != FR_OK)
	//    {
	//        /* f_open failed */
	//        PRINT_INF("File cannot be opened.");
	//        sprintf(msg, "FatFs error code: %u", res);
	//        PRINT_INF("%s",msg);

	//        SD_Eject();
	//        PRINT_INF("SD ejected.");
	//        return;
	//    }

	//    /* Step 5: Verify Flash Content */
	//    addr = APP_ADDRESS;
	//    cntr = 0;
	//    do
	//    {
	//        data = 0xFFFFFFFFFFFFFFFF;
	//        res   = f_read(&SDFile, &data, 4, &num);
	//        if(num)
	//        {
	//            if(*(uint32_t*)addr == (uint32_t)data)
	//            {
	//                addr += 4;
	//                cntr++;
	//            }
	//            else
	//            {
	//                sprintf(msg, "Verification error at: %lu byte.", (cntr * 4));
	//				PRINT_INF("%s",msg);

	//                f_close(&SDFile);
	//                SD_Eject();
	//                PRINT_INF("SD ejected.");

	//                LED_G_OFF();
	//                return;
	//            }
	//        }
	//        if(cntr % 256 == 0)
	//        {
	//            /* Toggle green LED during verification */
	//            LED_G_TG();
	//        }
	//    } while((res == FR_OK) && (num > 0));
	//    PRINT_INF("Verification passed.");
	//    LED_G_OFF();

	//    /* Eject SD card */
	//    SD_Eject();
	//    PRINT_INF("SD ejected.");

	//    /* Enable flash write protection */
	//#if(USE_WRITE_PROTECTION)
	//    PRINT_INF("Enablig flash write protection and generating system reset...");
	//    if(Bootloader_ConfigProtection(BL_PROTECTION_WRP) != BL_OK)
	//    {
	//        PRINT_INF("Failed to enable write protection.");
	//        PRINT_INF("Exiting Bootloader.");
	//    }
	//#endif
}

/*** SD Card ******************************************************************/
uint8_t SD_Init(void) {
	//    SDCARD_ON();

	//    if(FATFS_Init())
	//    {
	//        /* Error */
	//        return 1;
	//    }

	//    if(BSP_SD_Init())
	//    {
	//        /* Error */
	//        return 2;
	//    }

	return 0;
}

void SD_DeInit(void) {
	//    BSP_SD_DeInit();
	//    FATFS_DeInit();
	//    SDCARD_OFF();
}

void SD_Eject(void) {
	f_mount(NULL, (TCHAR const*)SDPath, 0);
}

/*** GPIO Configuration ***/
void GPIO_Startup(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	bsp_power_HoldPower();

	/* Configure GPIO pin output levels */
	HAL_GPIO_WritePin(LED_G_Port, LED_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_Y_Port, LED_Y_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_R_Port, LED_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SD_PWR_Port, SD_PWR_Pin, GPIO_PIN_SET);

	/* LED_G_Pin, LED_Y_Pin, LED_R_Pin */
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = LED_G_Pin;
	HAL_GPIO_Init(LED_G_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_Y_Pin;
	HAL_GPIO_Init(LED_Y_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LED_R_Pin;
	HAL_GPIO_Init(LED_R_Port, &GPIO_InitStruct);

	/* SD Card Power Pin */
	GPIO_InitStruct.Pin	  = SD_PWR_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_PWR_Port, &GPIO_InitStruct);

	/* User Button */
	GPIO_InitStruct.Pin	  = BTN_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BTN_Port, &GPIO_InitStruct);
}
void GPIO_DeInit(void) {
	HAL_GPIO_DeInit(BTN_Port, BTN_Pin);
	HAL_GPIO_DeInit(LED_G_Port, LED_G_Pin);
	HAL_GPIO_DeInit(LED_Y_Port, LED_Y_Pin);
	HAL_GPIO_DeInit(LED_R_Port, LED_R_Pin);
	HAL_GPIO_DeInit(SD_PWR_Port, SD_PWR_Pin);

	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOC_CLK_DISABLE();
}

/*** Console Configuration ***/
static USART_HandleTypeDef console_usart_handle;
void					   Console_Init(void) {
	  USART_HandleTypeDef*	   handle = &console_usart_handle;
	  RCC_PeriphCLKInitTypeDef clk_init;
	  GPIO_InitTypeDef		   GPIO_InitStructure;

	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_USART2_CLK_ENABLE();

	  handle->Instance		  = USART2;
	  handle->State			  = HAL_USART_STATE_RESET;
	  handle->Init.BaudRate	  = 115200;
	  handle->Init.StopBits	  = USART_STOPBITS_1;
	  handle->Init.WordLength = USART_WORDLENGTH_8B;
	  handle->Init.Parity	  = USART_PARITY_NONE;
	  handle->Init.Mode		  = USART_MODE_TX_RX;
	  HAL_USART_Init(&console_usart_handle);

	  GPIO_InitStructure.Pin	   = GPIO_PIN_5;  // Tx
	  GPIO_InitStructure.Mode	   = GPIO_MODE_AF_PP;
	  GPIO_InitStructure.Speed	   = GPIO_SPEED_FAST;
	  GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	  GPIO_InitStructure.Pull	   = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIO_InitStructure.Pin	   = GPIO_PIN_6;  // Rx
	  GPIO_InitStructure.Mode	   = GPIO_MODE_AF_OD;
	  GPIO_InitStructure.Speed	   = GPIO_SPEED_FAST;
	  GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	  GPIO_InitStructure.Pull	   = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void Console_DeInit(void) {
}

/*** System Clock Configuration ***/
void SystemClock_Config(void) {
	RCC_OscInitTypeDef		 RCC_OscInitStruct;
	RCC_ClkInitTypeDef		 RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	HAL_RCC_DeInit();

	/* Initializes the CPU, AHB and APB bus clocks */
#if defined(STM32F4)
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.PLL.PLLState	 = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource	 = RCC_PLLCFGR_PLLSRC_HSE;
	RCC_OscInitStruct.PLL.PLLM		 = 8;
	RCC_OscInitStruct.PLL.PLLN		 = 336;
	RCC_OscInitStruct.PLL.PLLP		 = 2;
	RCC_OscInitStruct.PLL.PLLQ		 = 7;

	RCC_OscInitStruct.HSEState		 = RCC_HSE_ON;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
#elif defined(STM32L4)
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 24;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
#endif
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		//        Error_Handler();
	}

	RCC_ClkInitStruct.ClockType =
		RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource	 = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider	 = LL_RCC_SYSCLK_DIV_1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		//        Error_Handler();
	}

	//    PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_SDMMC1;
	//    PeriphClkInit.Sdmmc1ClockSelection    = RCC_SDMMC1CLKSOURCE_PLLSAI1;
	//    PeriphClkInit.PLLSAI1.PLLSAI1Source   = RCC_PLLSOURCE_MSI;
	//    PeriphClkInit.PLLSAI1.PLLSAI1M        = 1;
	//    PeriphClkInit.PLLSAI1.PLLSAI1N        = 24;
	//    PeriphClkInit.PLLSAI1.PLLSAI1P        = RCC_PLLP_DIV2;
	//    PeriphClkInit.PLLSAI1.PLLSAI1Q        = RCC_PLLQ_DIV2;
	//    PeriphClkInit.PLLSAI1.PLLSAI1R        = RCC_PLLR_DIV2;
	//    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	//    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	//    {
	//        Error_Handler();HAL_RCC_OscConfig
	//    }

	/* Configure the main internal regulator output voltage */
	if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
		Error_Handler();
	}

	/* Configure the Systick */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/*** HAL MSP init ***/
void HAL_MspInit(void) {
	SystemClock_Config();

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);

	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

int fputc(int ch, FILE* fp) {
	uint8_t data = (uint8_t)ch;
	//	fp = NULL;
	if((ch > 0 && ch < 7) || (ch > 13 && ch < 27)) {
		data = '?';
	}
	HAL_USART_Transmit(&console_usart_handle, &data, 1, 100);
	return (ch);
}

/*** Debug ***/
void print(const char* str) {
#if(USE_SWO_TRACE)
	puts(str);
#else
	PRINT_RAW("%s\r\n", str);
#endif
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
_ARMABI_NORETURN void Error_Handler(void) {
	while(1) {
		led_setAllRGB(RGB(20, 20, 20));
		HAL_Delay(150);
		led_setAllRGB(RGB_OFF);
		HAL_Delay(150);
	}
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
}

#endif
