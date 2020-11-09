/**
 *******************************************************************************
 * STM32 Bootloader Header
 *******************************************************************************
 * @author Akos Pasztor
 * @file   bootloader.h
 * @brief  This file contains the bootloader configuration parameters,
 *	       function prototypes and other required macros and definitions.
 *
 * @see    Please refer to README for detailed information.
 *******************************************************************************
 * @copyright (c) 2020 Akos Pasztor.                    https://akospasztor.com
 *******************************************************************************
 */

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

/** Bootloader Configuration
 * @defgroup Bootloader_Configuration Bootloader Configuration
 * @{
 */

/** Select target MCU family: please define the target MCU family type below.
 * Currently supported MCU families:
 *  - STM32L4
 */
//#define STM32L4

#define STM32F4

/** Check application checksum on startup */
#define USE_CHECKSUM 1

/** Enable write protection after performing in-app-programming */
#define USE_WRITE_PROTECTION 0

/** Automatically set vector table location before launching application */
#define SET_VECTOR_TABLE 1

#ifdef STM32L4
#define OBL_RESET_IS_AVAILABLE	1
#else
#define OBL_RESET_IS_AVAILABLE	0
#endif

/** Clear reset flags
 *  - If enabled: bootloader clears reset flags. (This occurs only when OBL RST
 * flag is active.)
 *  - If disabled: bootloader does not clear reset flags, not even when OBL RST
 * is active.
 */
#define CLEAR_RESET_FLAGS 1


/** Start address of application space in flash */
#define APP_ADDRESS 0x08020000u

/** End address of application space (address of last byte) */
#define END_ADDRESS (uint32_t)0x0807FFFF

/** Start address of application checksum in flash */
#define CRC_ADDRESS (uint32_t)(END_ADDRESS-3)

/** Address of System Memory (ST Bootloader) */
#define SYSMEM_ADDRESS (uint32_t)0x1FFF0000




#define APP1_ADDRESS	(0x08020000u)
#define APP1_SIZE		(0xC0000u-4u)
#define APP1_CRC_ADDR	(APP1_ADDRESS+APP1_SIZE)

#define APP2_ADDRESS	(0x080E0000u)
#define APP2_SIZE		(0x20000u-4u)
#define APP2_CRC_ADDR	(APP2_ADDRESS+APP2_SIZE)

/** @} */
/* End of configuration ------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* Include the appropriate header file */
#if defined(STM32L4)
#include "stm32l4xx.h"
#elif defined(STM32F4)
#include "stm32f4xx.h"
#else
#error "Target MCU header file is not defined or unsupported."
#endif

/* Defines -------------------------------------------------------------------*/
/** Size of application in DWORD (32bits or 4bytes) */
#define APP_SIZE (uint32_t)(((END_ADDRESS - APP_ADDRESS)) / 4)

/** Number of pages per bank in flash */
#define FLASH_PAGE_NBPERBANK (256)

/* MCU RAM information (to check whether flash contains valid application) */
#define RAM_BASE SRAM1_BASE     /*!< Start address of RAM */
#define RAM_SIZE 0x20000 /*!< RAM size in bytes */

/* Enumerations --------------------------------------------------------------*/
/** Bootloader error codes */
enum eBootloaderErrorCodes
{
    BL_OK = 0,      /*!< No error */
    BL_NO_APP,      /*!< No application found in flash */
    BL_SIZE_ERROR,  /*!< New application is too large for flash */
    BL_CHKS_ERROR,  /*!< Application checksum error */
    BL_ERASE_ERROR, /*!< Flash erase error */
    BL_WRITE_ERROR, /*!< Flash write error */
    BL_OBP_ERROR    /*!< Flash option bytes programming error */
};

/** Flash Protection Types */
enum eFlashProtectionTypes
{
    BL_PROTECTION_NONE  = 0,   /*!< No flash protection */
    BL_PROTECTION_WRP   = 0x1, /*!< Flash write protection */
    BL_PROTECTION_RDP   = 0x2, /*!< Flash read protection */
    BL_PROTECTION_PCROP = 0x4, /*!< Flash propietary code readout protection */
};

/* Functions -----------------------------------------------------------------*/
uint8_t Bootloader_Init(void);
uint8_t Bootloader_Erase(void);

uint8_t Bootloader_FlashBegin(void);
uint8_t Bootloader_FlashNext(uint64_t data);
uint8_t Bootloader_FlashNextWord(uint32_t data);
uint8_t Bootloader_FlashEnd(void);

uint8_t Bootloader_GetProtectionStatus(void);
uint8_t Bootloader_ConfigProtection(uint32_t protection);

uint8_t Bootloader_CheckSize(uint32_t appsize);
uint8_t Bootloader_VerifyChecksum(void);
uint8_t Bootloader_CheckForApplication(void);
void    Bootloader_JumpToApplication(void);

uint8_t Bootloader_IsApp1ChecksumValid(void);
uint8_t Bootloader_CheckForApp1(void);
void    Bootloader_JumpToApp1(void);

uint8_t Bootloader_IsApp2ChecksumValid(void);
uint8_t Bootloader_CheckForApp2(void);
void    Bootloader_JumpToApp2(void);

void    Bootloader_JumpToSysMem(void);

uint32_t Bootloader_GetVersion(void);

#endif /* __BOOTLOADER_H */
