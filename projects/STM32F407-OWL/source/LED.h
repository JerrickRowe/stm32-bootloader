#ifndef __LED_H__
#define __LED_H__

#ifdef __cplusplus
extern "C"{
#endif

#include <stdbool.h>	
#include <stdint.h>

#define USE_SERIAL_LED
//#define USE_GPIO_LED
	
	
#ifdef USE_SERIAL_LED
	
	typedef enum led_id{
		LED_STA,
		LED_SIG,
		LED_LOG,
//		LED_PWR,
		__LED_ENUM_BUTT,
	}led_id_t;


	typedef enum led_mode{
		LED_OFF,
		LED_ON,
		LED_BREATH,
	}led_mode_t;
	
#endif
	
	
#ifdef USE_GPIO_LED
	typedef enum led_id{
		LED_PWR,
		LED_R,
		LED_G,
		LED_B,
		__LED_ENUM_BUTT,
	}led_id_t;


	typedef enum led_mode{
		LED_OFF,
		LED_ON,
		LED_BLINK_1,
		LED_BLINK_2,
	}led_mode_t;
#endif


void led_init( void );

void led_poll( void );	// Poll this function if blink mode is needed.

void led_set( led_id_t, led_mode_t );

void led_allon( void );

void led_alloff( void );

void led_on( led_id_t );

void led_off( led_id_t );

void led_onoff( led_id_t led, bool onoff );

void led_all_onoff(    bool onoff );

void led_toggle( led_id_t );

#include "RGB.h"

//void led_setRGB( led_id_t, led_RGB_Color_t );

void led_setRGB( led_id_t, uint32_t );

void led_setAllRGB( uint32_t );


#ifdef __cplusplus
}
#endif

#endif
