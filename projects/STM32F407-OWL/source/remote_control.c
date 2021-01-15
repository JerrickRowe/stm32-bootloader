#ifdef __cplusplus
extern "C"{
#endif

#include "remote_control.h"
#include "bsp_E22.h"
#include <stdint.h>
#include <stdbool.h>

#define GET_TIME_MS()	HAL_GetTick()

const uint8_t rc_reply_usb[10] = {0xAA,0x59,0,5,MAIN_STA_USB_STORAGE,0,0,0,0xFF,0xF7};
const uint8_t rc_reply_noapp[10] = {0xAA,0x59,0,5,MAIN_STA_BL_NOAPP,0,0,0,0xFF,0xF8};
const uint8_t rc_reply_upgrading[10] = {0xAA,0x59,0,5,MAIN_STA_BL_UPGRADING,0,0,0,0xFF,0xF9};



void RC_SimpleConnection( main_sta_t sta ){
	const uint8_t *pData;
//	static bool isInit = true;
//	if( isInit ){
//		bsp_InitE22();
//		_RC_bsp_RecoverFromUpgradeMode();
//		isInit = false;
//	}
	if( bsp_E22_GetIncomingDataCnt()
	&&	GET_TIME_MS() - bsp_E22_GetLastRxTimestamp() > 50 
	){
		switch( sta ){
		case MAIN_STA_USB_STORAGE: pData = rc_reply_usb; break;
		case MAIN_STA_BL_NOAPP: pData = rc_reply_noapp; break;
		case MAIN_STA_BL_UPGRADING: pData = rc_reply_upgrading; break;
		}
		_RC_bsp_SendNByte( pData, 10 );
		bsp_E22_ClearIncomingData();
	}
}




#ifdef __cplusplus
}
#endif

