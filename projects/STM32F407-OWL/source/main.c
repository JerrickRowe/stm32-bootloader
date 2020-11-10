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
#include "ff.h"
#include "led.h"
#include "bsp_WS2812x.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "git.h"

#include "testcases.h"

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


/* Private variables ---------------------------------------------------------*/
static uint8_t BTNcounter = 0;

#define VER		0

#define REV		1

#define PRJ_STR	"FlyFire-bootloader"

/* External variables --------------------------------------------------------*/
char  SDPath[4]; /* SD logical drive path */
FATFS SDFatFs;   /* File system object for SD logical drive */
FIL   SDFile;    /* File object for SD */

/* Function prototypes -------------------------------------------------------*/
void    Enter_Bootloader(void);
uint8_t SD_Init(void);
void    SD_DeInit(void);
void    SD_Eject(void);
void    GPIO_Startup(void);
void    Console_Init(void);
void    Console_DeInit(void);
void    GPIO_DeInit(void);
void    SystemClock_Config(void);
void    Error_Handler(void);
void    print(const char* str);

/* Private functions ---------------------------------------------------------*/
const char *months[] = {
	  "Jan", "Feb", "Mar", "Apr", "May", "Jun"
	, "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
void getBuildTimeStr_YYYYMMDDhhss(char *pDes){
	char strDate [] = __DATE__;
	char strTime [] = __TIME__;
	unsigned char i;
	unsigned int month, day, hour, minute, year;

	*(strTime + 2) = 0;
	*(strTime + 5) = 0;
	hour = atoi(strTime);
	
	minute = atoi(strTime+3);
	year = atoi(strDate + 9);
	
	*(strDate + 6) = 0;
	day = atoi(strDate + 4);
	*(strDate + 3) = 0;
	for (i = 0; i < 12; i++){
		if (!strcmp(strDate, months[i])){
			month = i + 1;
			sprintf( pDes, "20%02d%02d%02d%02d%02d", year, month, day, hour, minute );
			return;
		}
	}
}


const char* get_version_string( void ){
	static char str[500];
	print( "\n\n\n\n\n\r" 
		"______  _        ______  _             \r\n"
		"|  ___|| |       |  ___|(_)            \r\n"
		"| |_   | | _   _ | |_    _  _ __   ___ \r\n"
		"|  _|  | || | | ||  _|  | || '__| / _ \\\r\n"
		"| |    | || |_| || |    | || |   |  __/\r\n"
		"\\_|    |_| \\__, |\\_|    |_||_|    \\___|\r\n"
		"            __/ |                      \r\n"
		"           |___/                       \r\n"
	);
	snprintf( str, sizeof(str), "%s-V%d.%d-%s-", PRJ_STR, VER, REV, GIT_COMMIT_HEAD );
	getBuildTimeStr_YYYYMMDDhhss(str+strlen(str));
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
void HAL_IncTick(void)
{
  tick ++;
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other 
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return tick;
}



int testcase_4 ( void );

void MountFilesystem( void ){
	
	testcase_4();
	
    PRINT_RAW("Mount SD card and FatFS........");
    FRESULT  fr;
    UINT     num;
    uint8_t  i;
    uint8_t  status;
    uint64_t data;
    uint32_t cntr;
    uint32_t addr;
    char     msg[40] = {0x00};
    /* Initialize SD card */
//    if(SD_Init())
//    {
//        /* SD init failed */
//        PRINT_INF("INIT_FAILED");
//        return;
//    }

    /* Mount SD card */
    fr = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1);
    if(fr != FR_OK)
    {
        /* f_mount failed */
        PRINT_INF("MNT_FAILED [0x%02X]",fr);
        return;
    }
    PRINT_INF("DONE");

}



void Copy_App2_To_App1( void ){
	PRINT_RAW("Copy App2 to App1.");
	
	Bootloader_FlashBegin();
	
	uint32_t cnt = 0;
	
	while( cnt < ((APP2_SIZE+4) / 4 ) ){
		uint32_t data = *(uint32_t *)(APP2_ADDRESS + (cnt*4));
		uint8_t  status;
		status = Bootloader_FlashNextWord(data);
		if( cnt % 9830 == 0 ){
			PRINT_RAW(".");
		}
		cnt ++;
	}
	
	Bootloader_FlashEnd();
	
#if(USE_CHECKSUM)
	// Verify application checksum 
	if(Bootloader_IsApp2ChecksumValid() != BL_OK){
		PRINT_RAW("FAILED");
	}else{
		PRINT_RAW("DONE");
	}
#endif
	PRINT_RAW("\r\n");
}


bool VerifyUpgradeFile( FIL *fp ){
	if( !fp ){
		return false;
	}
	return false;
}


static FIL upgrade_file;
bool UpdateApp2( void ){
	FRESULT fr;
	FILINFO file_info;
	FILINFO fno;
	
	PRINT_INF(	"List files:" );
	
	
	
	DIR dir;
    fr = f_opendir(&dir, "/");                       /* Open the directory */
    if (fr == FR_OK) {
		uint32_t i;
        for (;;) {
            fr = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (fr != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR) {                    /* It is a directory */
				PRINT_INF(	"#%s"
				,	fno.fname
				);
            } else {                                       /* It is a file. */
				PRINT_INF(	"%s, %dbytes"
				,	fno.fname
				,	fno.fsize
				);
            }
        }
        f_closedir(&dir);
    }
	
	PRINT_RAW("Create \"%s\"......", UPGRADE_FILENAME);
	fr = f_open(&upgrade_file, UPGRADE_FILENAME, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if(fr != FR_OK){
		PRINT_INF("FAILED");
	}else{
		
		PRINT_INF("DONE");
		f_close(&upgrade_file);
	}
	return false;
	
	PRINT_RAW("Finding \"%s\"......", UPGRADE_FILENAME);
	fr = f_stat(UPGRADE_FILENAME, &file_info);
    if(fr != FR_OK){
		PRINT_INF("NOTFOUND");
        return false;
	}
	PRINT_INF("FOUND");
	PRINT_INF(	"File info: %s %dbytes"
	,	file_info.fname
	,	file_info.fsize
	);
	
	PRINT_RAW("Opening \"%s\"......", UPGRADE_FILENAME);
	fr = f_open(&upgrade_file, UPGRADE_FILENAME, FA_READ);
	if( fr != FR_OK ){
		PRINT_INF("FAILED");
		return false;
	}
	PRINT_INF("DONE");
	
	PRINT_RAW("Verifying upgrade file......");
	if( VerifyUpgradeFile(&upgrade_file) == false ){
		PRINT_INF("FAILED");
		f_close(&upgrade_file);
		PRINT_RAW("Delete %s........", UPGRADE_FILENAME);
		f_unlink(UPGRADE_FILENAME);
		fr = f_stat(UPGRADE_FILENAME, &file_info);
		if(fr != FR_OK){
			PRINT_INF("DONE");
		}else{
			PRINT_INF("FAILED");
		}
		return false;
	}
	PRINT_INF("PASS");
	
	
	
	Copy_App2_To_App1();
	f_close(&upgrade_file);
	return true;
}



/* Main ----------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    GPIO_Startup();

	Console_Init();
	
	led_init();
    PRINT_RAW("%s\r\n",get_version_string());

	led_allon();
	
	
	MountFilesystem();

	
	/*
	char *cwd[50];
	f_getcwd(cwd,50);
	PRINT_RAW("Current path: \"%s\"\r\n", cwd);
	
	if( UpdateApp2() == true ){
		PRINT_RAW("Launching new app.\r\n");
		// De-initialize bootloader hardware & peripherals
		SD_DeInit();
		GPIO_DeInit();
		// Launch application 
		Bootloader_JumpToApp1();
	}
	*/
	static bool app1_available = false;
	static bool app2_available = false;

	PRINT_RAW("Checking for App 1......");
    // Check if there is application in user flash area
    if(Bootloader_CheckForApp1() == BL_OK){
#if(USE_CHECKSUM)
		// Verify application checksum 
		if(Bootloader_IsApp1ChecksumValid() != BL_OK){
			PRINT_RAW("BAD");
		}else{
			PRINT_RAW("PASS");
			app1_available = true;
		}
#endif
	}else{
		PRINT_RAW("NOTFOUND");
	}
	PRINT_RAW("\r\n");
/*
	PRINT_RAW("Checking for App 2......");
    // Check if there is application in user flash area
    if(Bootloader_CheckForApp2() == BL_OK){
#if(USE_CHECKSUM)
		// Verify application checksum 
		if(Bootloader_IsApp2ChecksumValid() != BL_OK){
			PRINT_RAW("BAD");
		}else{
			PRINT_RAW("PASS");
			app2_available = true;
		}
#endif
	}else{
		PRINT_RAW("NOTFOUND");
	}
	PRINT_RAW("\r\n");
*/
/*
	// Check if backup app are the same.
	bool isApp2TheSameAsApp1 = false;
	if( app1_available && app2_available ){
		PRINT_RAW("Comparing App1 and App2......");
		if( memcmp( APP1_ADDRESS, APP2_ADDRESS, APP2_SIZE ) == 0 ){
			isApp2TheSameAsApp1 = true;
			PRINT_RAW("Same\r\n");
			PRINT_RAW("Launching App 1.\r\n");
			// De-initialize bootloader hardware & peripherals
			SD_DeInit();
			GPIO_DeInit();
			// Launch application 
			Bootloader_JumpToApp1();
		}else{
			PRINT_INF("Different");
			Copy_App2_To_App1();
			PRINT_INF("Launching App 1.");
			// De-initialize bootloader hardware & peripherals
			SD_DeInit();
			GPIO_DeInit();
			// Launch application 
			Bootloader_JumpToApp1();
		}
	}
*/
/*
	// Recover from backup app
	while( !app1_available && app2_available ){
		
		Copy_App2_To_App1();
		
		if(Bootloader_CheckForApp1() == BL_OK){
	#if(USE_CHECKSUM)
			// Verify application checksum 
			if(Bootloader_IsApp1ChecksumValid() == BL_OK){
				app1_available = true;
				PRINT_RAW("Launching App 1.\r\n");
				SD_DeInit();
				GPIO_DeInit();
				// Launch application 
				Bootloader_JumpToApp1();
			}
	#endif
		}else{
			PRINT_RAW("Failed to recover, retry.\r\n");
		}
		HAL_Delay(2000);
	}
*/

	// Launch app 1
	if( app1_available && !app2_available ){
		PRINT_RAW("Launching App 1.\r\n");
		SD_DeInit();
		GPIO_DeInit();
		// Launch application 
		Bootloader_JumpToApp1();
	}
	
    // No application found
    PRINT_INF("No application in flash.");
    while(1){
        setAllPixelColor( 20,20,20 );
        HAL_Delay(150);
        setAllPixelColor( 0,0,0 );
        HAL_Delay(150);
    }
}

/*** Bootloader ***************************************************************/

void Enter_Bootloader(void)
{
//    FRESULT  fr;
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
//    fr = f_mount(&SDFatFs, (TCHAR const*)SDPath, 1);
//    if(fr != FR_OK)
//    {
//        /* f_mount failed */
//        PRINT_INF("SD card cannot be mounted.");
//        sprintf(msg, "FatFs error code: %u", fr);
//        PRINT_INF("%s",msg);
//        return;
//    }
//    PRINT_INF("SD mounted.");

//    /* Open file for programming */
//    fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
//    if(fr != FR_OK)
//    {
//        /* f_open failed */
//        PRINT_INF("File cannot be opened.");
//        sprintf(msg, "FatFs error code: %u", fr);
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
//        fr   = f_read(&SDFile, &data, 8, &num);
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
//    } while((fr == FR_OK) && (num > 0));

//    /* Step 4: Finalize Programming */
//    Bootloader_FlashEnd();
//    f_close(&SDFile);
//    LED_G_OFF();
//    LED_Y_OFF();
//    PRINT_INF("Programming finished.");
//    sprintf(msg, "Flashed: %lu bytes.", (cntr * 8));
//    PRINT_INF("%s",msg);

//    /* Open file for verification */
//    fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
//    if(fr != FR_OK)
//    {
//        /* f_open failed */
//        PRINT_INF("File cannot be opened.");
//        sprintf(msg, "FatFs error code: %u", fr);
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
//        fr   = f_read(&SDFile, &data, 4, &num);
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
//    } while((fr == FR_OK) && (num > 0));
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
uint8_t SD_Init(void)
{
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

void SD_DeInit(void)
{
//    BSP_SD_DeInit();
//    FATFS_DeInit();
//    SDCARD_OFF();
}

void SD_Eject(void)
{
    f_mount(NULL, (TCHAR const*)SDPath, 0);
}

/*** GPIO Configuration ***/
void GPIO_Startup(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

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
    GPIO_InitStruct.Pin   = SD_PWR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_PWR_Port, &GPIO_InitStruct);

    /* User Button */
    GPIO_InitStruct.Pin   = BTN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BTN_Port, &GPIO_InitStruct);
}
void GPIO_DeInit(void)
{
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
void Console_Init(void)
{
	USART_HandleTypeDef *handle = &console_usart_handle;
	RCC_PeriphCLKInitTypeDef clk_init;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	
	handle->Instance = USART2;
	handle->State = HAL_USART_STATE_RESET;
	handle->Init.BaudRate = 921600;
	handle->Init.StopBits = USART_STOPBITS_1;
	handle->Init.WordLength = USART_WORDLENGTH_8B;
	handle->Init.Parity = USART_PARITY_NONE;
	handle->Init.Mode = USART_MODE_TX_RX;
	HAL_USART_Init(&console_usart_handle);
	
	
	GPIO_InitStructure.Pin   = GPIO_PIN_5;	// Tx
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init( GPIOD, &GPIO_InitStructure );
	
	GPIO_InitStructure.Pin   = GPIO_PIN_6;	// Rx
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init( GPIOD, &GPIO_InitStructure );
}

void Console_DeInit(void)
{
	
}

/*** System Clock Configuration ***/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct;
    RCC_ClkInitTypeDef       RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

	HAL_RCC_DeInit();
	
    /* Initializes the CPU, AHB and APB bus clocks */
#if defined(STM32F4)
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;	
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLCFGR_PLLSRC_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = 2;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
	
	RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
	RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
#elif defined(STM32L4)
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_6;
	
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 24;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
#endif
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
//        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = LL_RCC_SYSCLK_DIV_1;
    RCC_ClkInitStruct.APB1CLKDivider = LL_RCC_APB1_DIV_4;
    RCC_ClkInitStruct.APB2CLKDivider = LL_RCC_APB2_DIV_2;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
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
    if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure the Systick */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/*** HAL MSP init ***/
void HAL_MspInit(void)
{
	SystemClock_Config();
	
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



int fputc( int ch, FILE *fp ){
	uint8_t data = (uint8_t)ch;
//	fp = NULL;
	if( (ch>0 && ch<7)
	||	(ch>13 && ch<27)
	){
		data = '?';
	}
	HAL_USART_Transmit( &console_usart_handle, &data, 1, 100 );
	return (ch);
}


/*** Debug ***/
void print(const char* str)
{
#if(USE_SWO_TRACE)
    puts(str);
#else
	PRINT_RAW( "%s\r\n", str );
#endif
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
_ARMABI_NORETURN void Error_Handler(void)
{
    while(1){
        setAllPixelColor( 20,20,20 );
        HAL_Delay(150);
        setAllPixelColor( 0,0,0 );
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
void assert_failed(uint8_t* file, uint32_t line)
{
}

#endif
