/**
 ******************************************************************************
 * @file    usbd_usr.c
 * @author  MCD Application Team
 * @version V1.2.1
 * @date    17-March-2018
 * @brief   This file includes the user application layer
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

/* Includes ------------------------------------------------------------------ */
#include "usbd_usr.h"
#include <stdio.h>
/** @addtogroup USBD_USER
 * @{
 */

/** @addtogroup USBD_MSC_DEMO_USER_CALLBACKS
 * @{
 */

/** @defgroup USBD_USR
 * @brief    This file includes the user application layer
 * @{
 */

/** @defgroup USBD_USR_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Macros
 * @{
 */

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

/**
 * @}
 */

/** @defgroup USBD_USR_Private_Variables
 * @{
 */
static int		found_host;
static int		is_connected;
static uint32_t operation_timestamp = 0;
static uint32_t init_timestamp		= 0;

/* Points to the DEVICE_PROP structure of current device */
/* The purpose of this register is to speed up the execution */

USBD_Usr_cb_TypeDef USR_cb = {
	USBD_USR_Init,
	USBD_USR_DeviceReset,
	USBD_USR_DeviceConfigured,
	USBD_USR_DeviceSuspended,
	USBD_USR_DeviceResumed,

	USBD_USR_DeviceConnected,
	USBD_USR_DeviceDisconnected,
};

/**
 * @}
 */

/** @defgroup USBD_USR_Private_Constants
 * @{
 */

#define USER_INFORMATION1 (uint8_t*)"INFO : Single Lun configuration"
#define USER_INFORMATION2 (uint8_t*)"INFO : microSD is used"
/**
 * @}
 */

/** @defgroup USBD_USR_Private_FunctionPrototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Functions
 * @{
 */

/**
 * @brief  Displays the message on LCD on device lib initialization
 * @param  None
 * @retval None
 */
void USBD_USR_Init(void) {
	operation_timestamp = GET_TIME_MS();
	init_timestamp		= GET_TIME_MS();
	PRINT_INF("USBD user init");
}

/**
 * @brief  Displays the message on LCD on device reset event
 * @param  speed : device speed
 * @retval None
 */
void USBD_USR_DeviceReset(uint8_t speed) {
	operation_timestamp = GET_TIME_MS();
	switch(speed) {
	case USB_OTG_SPEED_HIGH: PRINT_INF("USB Device Library V1.2.1 [HS]"); break;

	case USB_OTG_SPEED_FULL: PRINT_INF("USB Device Library V1.2.1 [FS]"); break;
	default: PRINT_INF("USB Device Library V1.2.1 [??]");
	}
}

/**
 * @brief  Device Configured user callback
 * @param  None
 * @retval Status
 */
void USBD_USR_DeviceConfigured(void) {
	found_host			= 1;
	is_connected		= 1;
	operation_timestamp = GET_TIME_MS();
	PRINT_INF("USB device configured");
}

/**
 * @brief  Device Suspended user callback
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceSuspended(void) {
	is_connected		= 0;
	operation_timestamp = GET_TIME_MS();
	PRINT_INF("USB device suspended");
}

/**
 * @brief  Displays the message on LCD on device resume event
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceResumed(void) {
	found_host			= 1;
	is_connected		= 1;
	operation_timestamp = GET_TIME_MS();
	PRINT_INF("USB device resumed");
}

/**
 * @brief  USBD_USR_DeviceConnected
 *         Displays the message on LCD on device connection Event
 * @param  None
 * @retval Status
 */
void USBD_USR_DeviceConnected(void) {
	operation_timestamp = GET_TIME_MS();
	PRINT_INF("USB device connected");
}

/**
 * @brief  USBD_USR_DeviceDisonnected
 *         Displays the message on LCD on device disconnection Event
 * @param  None
 * @retval Status
 */
void USBD_USR_DeviceDisconnected(void) {
	is_connected		= 0;
	operation_timestamp = GET_TIME_MS();
	PRINT_INF("USB device disconnected");
}

/**
 * @}
 */

int USBD_USR_IsReleased(void) {
	if(is_connected) {
		return 0;
	}
	if((operation_timestamp && GET_TIME_MS() - operation_timestamp > 1500)
	   && (init_timestamp && GET_TIME_MS() - init_timestamp > 6000)) {
		return 1;
	}
	return 0;
}

int USBD_USR_HostNotFound(void) {
	if(found_host) {
		return 0;
	}
	return 1;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
