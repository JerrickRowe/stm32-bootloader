#ifdef __cplusplus
extern "C" {
#endif

// controlling part of this module: Indicator LED, Beeper, Remote feedback

#include "indicator.h"

#include "FreeRTOS.h"
#include "timer.h"
#include "remote_control.h"

#include "led.h"
#include "beeper.h"
#include "bsp_board.h"
//#include "remote.h"
#include "sd_log.hpp"
#include "FlyFire_Vehicle_DJI.hpp"

#define GET_TIME_MS() uptime_ms_get()

static bool		blinkflag_slow;
static bool		blinkflag_quick;
static bool		force_reswitch	  = true;
static bool		isIntegratedLight = true;
static bool		sync;
static uint32_t blink_period_base = 250;

static indicator_prompt_t prompt_now, prompt_next, prompt_previous;
static uint32_t			  prompt_endtime;

static uint32_t blink_color;
static uint32_t static_color;

static void blink_LED_STA(uint32_t color, bool blinkflag) {
	led_setRGB(LED_STA, color);
	led_onoff(LED_STA, blinkflag);
}

static void blink_LED_SIG(uint32_t color, bool blinkflag) {
	led_setRGB(LED_SIG, color);
	led_onoff(LED_SIG, blinkflag);
}

static void blink_LED_LOG(uint32_t color, bool blinkflag) {
	led_setRGB(LED_LOG, color);
	led_onoff(LED_LOG, blinkflag);
}

static void AllBlink(uint32_t color, bool blinkflag) {
	led_setAllRGB(color);
	led_all_onoff(blinkflag);
	sync = blinkflag;
}

/*
static void AlternateBlink( uint32_t color1, uint32_t color2, uint32_t period ){
	static uint32_t timeLog;
	static bool blinkflag;
	if( GET_TIME_MS() - timeLog < period ){
		return;
	}
	timeLog = GET_TIME_MS();
	led_on( LED_STA );
	if( blinkflag ){
		blinkflag = false;
		led_setRGB( LED_STA, color1 );
	}else{
		blinkflag = true;
		led_setRGB( LED_STA, color2 );
	}
}
*/

typedef struct BreathParam_t {
	uint32_t stepPeriod;
	uint8_t	 stepNum;
	uint8_t	 stepCnt;

	int8_t colorStep_R;
	int8_t colorStep_G;
	int8_t colorStep_B;

	uint32_t currentRGB;
} BreathParam_t;
BreathParam_t all_breath_param;
BreathParam_t all_fade_param;

static void AllBreath_Init(uint32_t stepPeriodMS, uint32_t stepNum, uint32_t colorBase) {
	int16_t cb_R = (uint16_t)((colorBase & 0x00FF0000) >> 16);
	int16_t cb_G = (uint16_t)((colorBase & 0x0000FF00) >> 8);
	int16_t cb_B = (uint16_t)((colorBase & 0x000000FF) >> 0);

	if(cb_R) {
		all_breath_param.colorStep_R = 3;
	} else if(0) {
		all_breath_param.colorStep_R = -3;
	} else {
		all_breath_param.colorStep_R = 0;
	}

	if(cb_G) {
		all_breath_param.colorStep_G = 3;
	} else if(0) {
		all_breath_param.colorStep_G = -3;
	} else {
		all_breath_param.colorStep_G = 0;
	}

	if(cb_B) {
		all_breath_param.colorStep_B = 3;
	} else if(0) {
		all_breath_param.colorStep_B = -3;
	} else {
		all_breath_param.colorStep_B = 0;
	}

	all_breath_param.currentRGB = colorBase;
	all_breath_param.stepPeriod = stepPeriodMS;
	all_breath_param.stepNum	= stepNum;
	all_breath_param.stepCnt	= 0;
	led_allon();
}

static void AllBreath(void) {
	static uint32_t timeLog;
	BreathParam_t * param = &all_breath_param;
	if(GET_TIME_MS() - timeLog < param->stepPeriod) {
		return;
	}
	timeLog = GET_TIME_MS();
	sync	= false;

	// static uint16_t extended_step = 0;
	// if( extended_step ){
	// 	extended_step --;
	// 	return;
	// }

	uint32_t currentR = ((param->currentRGB & 0x00FF0000) >> 16);
	uint32_t currentG = ((param->currentRGB & 0x0000FF00) >> 8);
	uint32_t currentB = ((param->currentRGB & 0x000000FF) >> 0);

	if(param->stepCnt < (param->stepNum >> 1)) {
		if(currentR + param->colorStep_R <= 255) {
			currentR += param->colorStep_R;
		}
		if(currentG + param->colorStep_G <= 255) {
			currentG += param->colorStep_G;
		}
		if(currentB + param->colorStep_B <= 255) {
			currentB += param->colorStep_B;
		}
	} else {
		if(currentR >= param->colorStep_R) {
			currentR -= param->colorStep_R;
		}
		if(currentG >= param->colorStep_G) {
			currentG -= param->colorStep_G;
		}
		if(currentB >= param->colorStep_B) {
			currentB -= param->colorStep_B;
		}
	}
	param->currentRGB = (currentR << 16 | (currentG << 8) | (currentB << 0));

	led_setAllRGB(param->currentRGB);

	param->stepCnt++;
	if(param->stepCnt >= param->stepNum) {
		param->stepCnt = 0;
		// extended_step = 10;
		sync = true;
	}
}

static uint32_t bouncing_color		 = 0;
static uint32_t bouncing_interval_ms = 500;
static void		Bouncing(void) {
	static int		step = 0;
	static uint32_t timestamp;
	if(GET_TIME_MS() - timestamp < bouncing_interval_ms) {
		return;
	}
	timestamp = GET_TIME_MS();
	switch(step) {
	case 0:
		led_setRGB_exclusive(0, bouncing_color);
		// led_setRGB( 0, RGB(0,10,10) );
		// led_setRGB( 1, RGB(0,50,50) );
		// led_setRGB( 2, RGB(0,10,10) );
		break;
	case 1:
		led_setRGB_exclusive(1, bouncing_color);
		// led_setRGB( 0, RGB(0,5,5) );
		// led_setRGB( 1, RGB(0,10,10) );
		// led_setRGB( 2, RGB(0,50,50) );
		break;
	case 2:
		led_setRGB_exclusive(2, bouncing_color);
		// led_setRGB( 0, RGB(0,10,10) );
		// led_setRGB( 1, RGB(0,50,50) );
		// led_setRGB( 2, RGB(0,10,10) );
		break;
	case 3:
		led_setRGB_exclusive(1, bouncing_color);
		// led_setRGB( 0, RGB(0,50,50) );
		// led_setRGB( 1, RGB(0,10,10) );
		// led_setRGB( 2, RGB(0,5,5) );
		break;
	}
	if(++step > 3) {
		step = 0;
	}
}

static uint32_t filling_color		= 0;
static uint32_t filling_interval_ms = 500;
static void		Filling(void) {
	static int		step = 0;
	static uint32_t timestamp;
	if(GET_TIME_MS() - timestamp < filling_interval_ms) {
		return;
	}
	timestamp = GET_TIME_MS();
	sync	  = false;
	switch(step) {
	case 0:
		led_setRGB(0, RGB(0, 0, 0));
		led_setRGB(1, RGB(0, 0, 0));
		led_setRGB(2, RGB(0, 0, 0));
		break;
	case 1:
		led_setRGB(0, filling_color);
		led_setRGB(1, RGB(0, 0, 0));
		led_setRGB(2, RGB(0, 0, 0));
		break;
	case 2:
		led_setRGB(0, filling_color);
		led_setRGB(1, filling_color);
		led_setRGB(2, RGB(0, 0, 0));
		break;
	case 3:
		led_setRGB(0, filling_color);
		led_setRGB(1, filling_color);
		led_setRGB(2, filling_color);
		sync = true;
		break;
	case 4:
		led_setRGB(0, RGB(0, 0, 0));
		led_setRGB(1, filling_color);
		led_setRGB(2, filling_color);
		break;
	case 5:
		led_setRGB(0, RGB(0, 0, 0));
		led_setRGB(1, RGB(0, 0, 0));
		led_setRGB(2, filling_color);
		break;
	}
	if(++step > 5) {
		step = 0;
	}
}

static void AllFade_Init(uint32_t stepPeriodMS, uint32_t stepNum, uint32_t colorBase) {
	//	int16_t cb_R=(uint16_t)((colorBase&0x00FF0000)>>16);
	//	int16_t cb_G=(uint16_t)((colorBase&0x0000FF00)>>8);
	//	int16_t cb_B=(uint16_t)((colorBase&0x000000FF)>>0);

	all_fade_param.colorStep_R = 2;
	all_fade_param.colorStep_G = 2;
	all_fade_param.colorStep_B = 2;

	all_fade_param.currentRGB = colorBase;
	all_fade_param.stepPeriod = stepPeriodMS;
	all_fade_param.stepNum	  = stepNum;
	all_fade_param.stepCnt	  = 0;
	led_allon();
}

static void AllFade(void) {
	static uint32_t timeLog;
	BreathParam_t * param = &all_fade_param;
	if(GET_TIME_MS() - timeLog < param->stepPeriod) {
		return;
	}
	timeLog = GET_TIME_MS();
	sync	= false;

	uint8_t *pR = (uint8_t *)(&param->currentRGB) + 2;
	uint8_t *pG = (uint8_t *)(&param->currentRGB) + 1;
	uint8_t *pB = (uint8_t *)(&param->currentRGB) + 0;

	if(*pR >= param->colorStep_R) {
		*pR -= param->colorStep_R;
	} else {
		*pR = 0;
	}
	if(*pG >= param->colorStep_G) {
		*pG -= param->colorStep_G;
	} else {
		*pG = 0;
	}
	if(*pB >= param->colorStep_B) {
		*pB -= param->colorStep_B;
	} else {
		*pB = 0;
	}
	led_setAllRGB(param->currentRGB);

	if(param->stepCnt < param->stepNum) {
		param->stepCnt++;
	} else {
		sync = true;
	}
}

static void scanningRGB(uint32_t period) {
	static int		i;
	static uint32_t timeLog;
	if(GET_TIME_MS() - timeLog < period) {
		return;
	}
	timeLog = GET_TIME_MS();
	switch(i) {
	case 0:
		led_setAllRGB(RGB_R);
		i++;
		break;
	case 1:
		led_setAllRGB(RGB_G);
		i++;
		break;
	case 2:
		led_setAllRGB(RGB_B);
		i++;
		break;
	case 3: i = 0; break;
	}
}

static indicator_sta_t mode_now, mode_next, mode_previous, prompt_return = INDICATOR_STA_LAST;

// Return:
// true: keep running
// false:	switch now
static void sta_enter(indicator_sta_t sta) {
	switch(sta) {
	case INDICATOR_STA_STARTUP: break;
	case INDICATOR_STA_CONDITION_VIOLATED: led_setRGB(LED_STA, RGB_R); break;
	case INDICATOR_STA_IDLE_ALL_SYSTEM_GO: led_setRGB(LED_STA, RGB_G); break;

	case INDICATOR_STA_CONDITION_CHECKING: break;
	case INDICATOR_STA_SHUTTINGDOWN: led_setRGB(LED_STA, RGB_B); break;
	default: led_setRGB(LED_STA, RGB_OFF); break;
	}
}

static void sta_running(indicator_sta_t sta) {
	switch(sta) {
	case INDICATOR_STA_STARTUP:
		// scanningRGB( 200 );
		break;
	case INDICATOR_STA_FACTORY: blink_LED_STA(RGB_C, blinkflag_quick); break;
	case INDICATOR_STA_IDLE_ALL_SYSTEM_GO: break;
	case INDICATOR_STA_CONDITION_CHECKING: blink_LED_STA(RGB_Y, blinkflag_quick); break;
	default: break;
	}
}

static void sta_exit(indicator_sta_t sta) {
	switch(sta) {
	default: break;
	}
}

static bool initializing = true;
static bool disabled	 = false;

void indicator_init(void) {
	led_init();
	led_allon();
	indicator_set(INDICATOR_STA_STARTUP);
	initializing = false;
	disabled	 = false;
}

void indicator_set(indicator_sta_t mode) {
	mode_next = mode;

	switch(mode) {
	case INDICATOR_STA_FACTORY: isIntegratedLight = false; break;
	default: isIntegratedLight = true; break;
	}
}

void indicator_prompt(indicator_prompt_t mode, indicator_sta_t return_to, uint32_t time_ms) {
	prompt_endtime = GET_TIME_MS() + time_ms;
	prompt_next	   = mode;
	prompt_now	   = INDICATOR_PROMPT_NONE;	 // Force to switch sta
	force_reswitch = true;
}

bool indicator_is_prompting(void) {
	if(GET_TIME_MS() < prompt_endtime) {
		return true;
	}
	return false;
}

void indicator_disable(void) {
	disabled = true;
}

void indicator_enable(void) {
	disabled = false;
}

bool indicator_is_in(indicator_sta_t mode) {
	if(mode_next == mode || mode_now == mode) {
		return true;
	}
	return false;
}

void indicator_breath(uint32_t color, int8_t step) {
	all_breath_param.currentRGB = color;
	all_breath_param.stepPeriod = 20;
	all_breath_param.stepNum	= 120;
	all_breath_param.stepCnt	= 0;

	int16_t cb_R = (uint16_t)((color & 0x00FF0000) >> 16);
	int16_t cb_G = (uint16_t)((color & 0x0000FF00) >> 8);
	int16_t cb_B = (uint16_t)((color & 0x000000FF) >> 0);

	if(cb_R) {
		all_breath_param.colorStep_R = step;
	} else {
		all_breath_param.colorStep_R = 0;
	}

	if(cb_G) {
		all_breath_param.colorStep_G = step;
	} else {
		all_breath_param.colorStep_G = 0;
	}

	if(cb_B) {
		all_breath_param.colorStep_B = step;
	} else {
		all_breath_param.colorStep_B = 0;
	}

	led_allon();
	indicator_set(INDICATOR_STA_BREATH);
}

void indicator_fadeout(uint32_t color) {
	AllFade_Init(5, 30, color);
	indicator_set(INDICATOR_STA_FADEOUT);
}

void indicator_fadein(uint32_t color) {
	AllFade_Init(5, 30, color);
	indicator_set(INDICATOR_STA_FADEIN);
}

void indicator_blink_quick(uint32_t color) {
	blink_color = color;
	led_allon();
	indicator_set(INDICATOR_STA_QUICKBLINK);
}

void indicator_blink_slow(uint32_t color) {
	blink_color = color;
	led_allon();
	indicator_set(INDICATOR_STA_SLOWBLINK);
}

void indicator_bouncing(uint32_t color, uint32_t interval) {
	led_allon();
	bouncing_color		 = color;
	bouncing_interval_ms = interval;
	indicator_set(INDICATOR_STA_BOUNCING);
}

void indicator_filling(uint32_t color, uint32_t interval) {
	led_allon();
	filling_color		= color;
	filling_interval_ms = interval;
	indicator_set(INDICATOR_STA_FILLING);
}

void indicator_static(uint32_t color) {
	led_allon();
	static_color = color;
	indicator_set(INDICATOR_STA_STATIC);
}

const char *indicator_getname(indicator_sta_t sta) {
	switch(sta) {
	case INDICATOR_STA_STARTUP: return "Startup";
	case INDICATOR_STA_STANDBY: return "Standby";
	case INDICATOR_STA_FACTORY: return "Factory";
	case INDICATOR_STA_CONDITION_CHECKING: return "Condition Checking";
	case INDICATOR_STA_CONDITION_VIOLATED: return "Condition Violated";
	case INDICATOR_STA_IDLE_ALL_SYSTEM_GO: return "Idle All System GO";
	case INDICATOR_STA_FLYING_BELOW_THRESHOLD: return "Flying Velow Threchold";
	case INDICATOR_STA_FLYING_ABOVE_THRESHOLD: return "Flying Above Threchold";
	case INDICATOR_STA_TESTFIRE_READY: return "Testfire Ready";
	case INDICATOR_STA_TESTFIRE_FIRED: return "Testfile Fired";
	case INDICATOR_STA_TOUCHDOWN: return "Touchdown";
	default: return "Undefined";
	}
}

static void prompt_enter(indicator_prompt_t sta) {
	switch(sta) {
	case INDICATOR_PROMPT_RC_COMMTEST: beep(30); break;
	case INDICATOR_PROMPT_RC_INVALID_COMMAND:
		beep_rythm(50, 30);
		beep_rythm(50, 30);
		beep_rythm(50, 30);
		break;
	default: break;
	}
}

static void prompt_running(indicator_prompt_t sta) {
	switch(sta) {
	case INDICATOR_PROMPT_RC_COMMTEST: AllBlink(RGB_G, blinkflag_quick); break;
	case INDICATOR_PROMPT_RC_INVALID_COMMAND: AllBlink(RGB_Y, blinkflag_quick); break;
	default: break;
	}
}

static void prompt_exit(indicator_prompt_t sta) {
	switch(sta) {
	case INDICATOR_PROMPT_RC_COMMTEST: break;
	default: break;
	}
}

static void prompt_poll(void) {
	static enum {
		ENTER,
		RUNNING,
		EXIT,
	} sta = ENTER;
	switch(sta) {
	case ENTER:
		prompt_enter(prompt_next);
		prompt_now = prompt_next;
		sta		   = RUNNING;
	case RUNNING:
		prompt_running(prompt_now);
		if(prompt_next != prompt_now) {
			sta = EXIT;
		} else {
			break;
		}
	case EXIT:
		prompt_previous = prompt_now;
		prompt_exit(prompt_previous);
		sta = ENTER;
	}
}

static void integrated_enter(indicator_sta_t sta) {
	switch(sta) {
	case INDICATOR_STA_QUICKBLINK: led_allon(); break;

	case INDICATOR_STA_SLOWBLINK: led_allon(); break;

	case INDICATOR_STA_FADEOUT: led_allon(); break;

	case INDICATOR_STA_FADEIN: led_allon(); break;

	case INDICATOR_STA_BREATH: led_allon(); break;

	case INDICATOR_STA_STATIC: led_allon(); break;

	case INDICATOR_STA_BOUNCING: led_allon(); break;

	case INDICATOR_STA_FILLING: led_allon(); break;

	default:
		led_alloff();
		led_setRGB(LED_STA, RGB_OFF);
		break;
	}
}

static void integrated_running(indicator_sta_t sta) {
	static uint32_t beepTime;
	switch(sta) {
	case INDICATOR_STA_QUICKBLINK: AllBlink(blink_color, blinkflag_quick); break;
	case INDICATOR_STA_SLOWBLINK: AllBlink(blink_color, blinkflag_slow); break;
	case INDICATOR_STA_FADEOUT: AllFade(); break;
	case INDICATOR_STA_FADEIN: AllFade(); break;
	case INDICATOR_STA_BREATH: AllBreath(); break;
	case INDICATOR_STA_STATIC:
		led_setAllRGB(static_color);
		sync = true;
		break;
	case INDICATOR_STA_BOUNCING: Bouncing(); break;
	case INDICATOR_STA_FILLING: Filling(); break;
	default: break;
	}
}

static void integrated_exit(indicator_sta_t sta) {
	switch(sta) {
	default: break;
	}
}

static void integrated_poll(void) {
	static enum {
		ENTER,
		RUNNING,
		EXIT,
	} sta = ENTER;
	switch(sta) {
	case ENTER:
		integrated_enter(mode_next);
		mode_now = mode_next;
		sta		 = RUNNING;
	case RUNNING:
		if(force_reswitch && mode_next == mode_now) {
			sta = ENTER;
			break;
		}
		integrated_running(mode_now);
		if(mode_next != mode_now && sync) {
			sta = EXIT;
		} else {
			break;
		}
	case EXIT:
		mode_previous = mode_now;
		integrated_exit(mode_previous);
		sta = ENTER;
	}
}

static void LED_STA_poll(void) {
	static enum {
		ENTER,
		RUNNING,
		EXIT,
	} sta = ENTER;
	switch(sta) {
	case ENTER:
		sta_enter(mode_next);
		mode_now = mode_next;
		sta		 = RUNNING;
	case RUNNING:
		if(force_reswitch) {
			mode_next = mode_now;
			sta		  = ENTER;
			break;
		}
		sta_running(mode_now);
		if(mode_next != mode_now) {
			sta = EXIT;
		} else {
			break;
		}
	case EXIT:
		mode_previous = mode_now;
		sta_exit(mode_previous);
		sta = ENTER;
	}
}

static void LED_SIG_poll(void) {
	if(mode_now == INDICATOR_STA_IDLE_ALL_SYSTEM_GO) {
		return;
	}
	if(RC_isConnected() == false) {	 // Is remote disconnected?
		led_on(LED_SIG);
		led_setRGB(LED_SIG, RGB_R);
		return;
	}

	static uint32_t testEventStartTime;
	if(RC_isTestEvent()) {
		testEventStartTime = GET_TIME_MS();
	}
	if(GET_TIME_MS() - testEventStartTime < 2000) {
		blink_LED_SIG(RGB_G, blinkflag_quick);
		return;
	}

	rc_sig_strength_t signal_quality = RC_sig_strength_get();
	//	switch( signal_quality ){
	//	case RC_SIG_0:
	//		led_on( LED_SIG );
	//		led_setRGB( LED_SIG, RGB_R );
	//		break;
	//	case RC_SIG_1:
	//		led_on( LED_SIG );
	//		led_setRGB( LED_SIG, RGB(100,200,0) );
	//		break;
	//	case RC_SIG_2:
	//		led_on( LED_SIG );
	//		led_setRGB( LED_SIG, RGB_Y );
	//		break;
	//	case RC_SIG_3:
	//		led_on( LED_SIG );
	//		led_setRGB( LED_SIG, RGB_G );
	//		break;
	//	}
}

static void LED_LOG_poll(void) {
	if(mode_now == INDICATOR_STA_IDLE_ALL_SYSTEM_GO) {
		return;
	}
	if(sd_logger_isReady() == false) {
		led_on(LED_LOG);
		led_setRGB(LED_LOG, RGB_R);
		return;
	}
	if(sd_logger_isStandingby() == true) {
		led_on(LED_LOG);
		uint8_t checkFlag = 0;
		if(FlyFire_IsVehicleConnected() == true) {
			checkFlag |= 0x01;
		}
		if(1 == true) {	 // Check onboard sensor
			checkFlag |= 0x02;
		}
		switch(checkFlag) {
		case 0x1:  // Only vehicle online.
			led_setRGB(LED_LOG, RGB_P);
			break;
		case 0x2:  // Only onboard sensor online.
			led_setRGB(LED_LOG, RGB_Y);
			break;
		case 0x3:  // Both OBS and vehicle online.
			led_setRGB(LED_LOG, RGB_G);
			break;
		case 0x0:  // Both OBS and vehicle offline.
			led_setRGB(LED_LOG, RGB_R);
			break;
		}
		return;
	}
	if(sd_logger_isEnabled()) {
		if(sd_logger_isLogging() == true) {
			uint8_t checkFlag = 0;
			if(FlyFire_IsVehicleSendingData() == true) {
				checkFlag |= 0x01;
			}
			if(1 == true) {	 // Check onboard sensor
				checkFlag |= 0x02;
			}
			switch(checkFlag) {
			case 0x1:  // Data of vehicle is logged without onboard sensor.
				blink_LED_LOG(RGB_P, blinkflag_slow);
				break;
			case 0x2:  // Data of onboard sensor islogged without vehicle.
				blink_LED_LOG(RGB_Y, blinkflag_slow);
				break;
			case 0x3:  // Both data of OBS and vehicle are logged.
				blink_LED_LOG(RGB_G, blinkflag_slow);
				break;
			case 0x0:  // Logging blank data.
				blink_LED_LOG(RGB_R, blinkflag_slow);
				break;
			}
		} else {
			led_on(LED_LOG);
			led_setRGB(LED_LOG, RGB_B);
		}
	}
}

static inline void blinkflag_poll(void) {
	static uint32_t timeLog;
	if(GET_TIME_MS() - timeLog < blink_period_base) {
		return;
	}
	timeLog			= GET_TIME_MS();
	blinkflag_quick = !blinkflag_quick;
	if(blinkflag_quick) {
		blinkflag_slow = !blinkflag_slow;
	}
}

// Poll this function in 100ms period.
void indecator_poll(void) {
	if(initializing) {
		return;
	}
	if(disabled) {
		return;
	}
	if((prompt_return != INDICATOR_STA_LAST) && (GET_TIME_MS() > prompt_endtime)) {
		mode_next	  = prompt_return;
		prompt_return = INDICATOR_STA_LAST;
	}
	blinkflag_poll();

	if(GET_TIME_MS() < prompt_endtime) {
		prompt_poll();
	} else {
		if(isIntegratedLight) {
			integrated_poll();
		} else {
			LED_STA_poll();
			LED_SIG_poll();
			LED_LOG_poll();
		}
		if(force_reswitch) {
			force_reswitch = false;
		}
	}
	led_poll();
}

#ifdef __cplusplus
}
#endif
