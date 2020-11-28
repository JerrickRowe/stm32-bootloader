#include "led.h"

#ifdef USE_SERIAL_LED

#include "led_io.h"
#include "bsp_WS2812x.h"

#define DEBUG_LED	1
#if DEBUG_LED
#include <stdio.h>
#define PRINT_INF(fmt,...)	printf(fmt"\r\n",##__VA_ARGS__);
#define PRINT_ERR(fmt,...)	printf(fmt"\r\n",##__VA_ARGS__);
#define ENABLE_ASSERT	1
#else
#define PRINT_INF(fmt,...)	
#define PRINT_ERR(fmt,...)	
#endif

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include <stdbool.h>

#ifndef ENABLE_ASSERT
#define ENABLE_ASSERT	1
#endif
#if ENABLE_ASSERT
#include <assert.h>
#define ASSERT(e)	assert(e)
#else
#define ASSERT(e)	
#endif

//#include "bsp_board.h"
#include "bsp_WS2812x.h"

typedef struct led_param{
	int index;
	uint32_t color;
	uint32_t color_new;
	led_mode_t mode;
}led_param_t;

static led_param_t led_map[] = {
	{ 0, 0x000000, LED_ON },
	{ 1, 0x000000, LED_ON },
	{ 2, 0x000000, LED_ON },
};

void led_init( void ){
	bsp_InitWS2812();
}

void led_poll( void ){
	led_param_t *pLED;
	for( int i=0; i<__LED_ENUM_BUTT; i++ ){
		pLED = &led_map[i]; 
		if( pLED->mode == LED_ON ){
			SetPixelColor( pLED->index, pLED->color );
		}else if( pLED->mode == LED_OFF ){
			SetPixelColor( pLED->index, 0 );
		}
	}
	for( int i=0; i<__LED_ENUM_BUTT; i++ ){
		pLED = &led_map[i]; 
		pLED->color = pLED->color_new;
	}
	bsp_WS2812_SyncAllPixel();
}	// Poll this function to update LED status

void led_set( led_id_t led, led_mode_t mode ){
	if( mode==LED_OFF ){
		led_off( led );
	}else{
		led_on( led );
	}
}

void led_allon( void ){
	for( int i=0; i<__LED_ENUM_BUTT; i++ ){
		led_map[i].mode = LED_ON;
	}
}

void led_alloff( void ){
	for( int i=0; i<__LED_ENUM_BUTT; i++ ){
		led_map[i].mode = LED_OFF;
	}
}

void led_on( led_id_t led ){
	led_param_t *pLED = &led_map[led];
	pLED->mode = LED_ON;
}

void led_off( led_id_t led ){
	led_param_t *pLED = &led_map[led];
	pLED->mode = LED_OFF;
}

void led_onoff( led_id_t led, bool onoff ){
	led_param_t *pLED = &led_map[led];
	if( onoff ){
		pLED->mode = LED_ON;
	}else{
		pLED->mode = LED_OFF;
	}
}

void led_all_onoff(    bool onoff ){
	for( int i=0; i<__LED_ENUM_BUTT; i++ ){
		led_map[i].mode = onoff;
	}
}

void led_toggle( led_id_t led ){
	led_param_t *pLED = &led_map[led];
	if( pLED->mode == LED_ON ){
		led_off(led);
	}else{
		led_on(led);
	}
}

void led_setAllRGB( uint32_t color ){
	for( int i=0; i<__LED_ENUM_BUTT; i++ ){
		led_map[i].color_new = color;
	}
}

void led_setRGB( led_id_t led, uint32_t color ){
	led_param_t *pLED = &led_map[led];
	pLED->color_new = color;
}

#ifdef __cplusplus
}
#endif

#endif

