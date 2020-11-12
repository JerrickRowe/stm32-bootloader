#ifndef __INDICATOR_H__
#define __INDICATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "RGB.h"
	
typedef enum indicator_status{
	INDICATOR_STA_QUICKBLINK,
	INDICATOR_STA_SLOWBLINK,
	INDICATOR_STA_FADEOUT,
	INDICATOR_STA_FADEIN,
	INDICATOR_STA_BREATH,
	INDICATOR_STA_STATIC,
	INDICATOR_STA_BOUNCING,
	INDICATOR_STA_FILLING,
	
	INDICATOR_STA_LAST,
	INDICATOR_STA_NONE,
}indicator_sta_t;

typedef enum indicator_prompt_status{
	INDICATOR_PROMPT_RC_COMMTEST,
	INDICATOR_PROMPT_RC_INVALID_COMMAND,
	INDICATOR_PROMPT_NONE,
}indicator_prompt_t;

void indicator_prompt( indicator_prompt_t mode, indicator_sta_t return_to, uint32_t time_ms );

void indicator_set( indicator_sta_t );

bool indicator_is_prompting( void );

bool indicator_is_in( indicator_sta_t mode );

void indecator_poll( void );

void indicator_init( void );

void indicator_disable( void );

void indicator_enable( void );

void indicator_breath( uint32_t color, int8_t step );

void indicator_blink_quick( uint32_t color );

void indicator_blink_slow( uint32_t color );

void indicator_fadeout( uint32_t color );

void indicator_fadein( uint32_t color );

void indicator_static( uint32_t color );

void indicator_bouncing( uint32_t color, uint32_t interval );

void indicator_filling( uint32_t color, uint32_t interval );

const char* indicator_getname( indicator_sta_t sta );

#ifdef __cplusplus
}
#endif
	
#endif

