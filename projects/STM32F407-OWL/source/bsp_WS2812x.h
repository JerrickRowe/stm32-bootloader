/*
*********************************************************************************************************
*
*	模块名称 : WS2812x_RGB灯驱动模块
*	文件名称 : bsp_WS2812x.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_WS2812x_H
#define __BSP_WS2812x_H

#include <stdint.h>

/* 定义全局变量的结构体 */

typedef struct
{
	/* 第一盏灯 STATE*/
	uint8_t R1;		//第一盏灯红色RGB值
	uint8_t G1;		//第一盏灯绿色RGB值
	uint8_t B1;		//第一盏灯蓝色RGB值

	/* 第二盏灯 GRAND*/
	uint8_t R2;		//第二盏灯红色RGB值
	uint8_t G2;		//第二盏灯绿色RGB值
	uint8_t B2;		//第二盏灯蓝色RGB值

	/* 第三盏灯 SIGNAL*/
	uint8_t R3;		//第三盏灯红色RGB值
	uint8_t G3;		//第三盏灯绿色RGB值
	uint8_t B3;		//第三盏灯蓝色RGB值

}WS2812_T;

extern WS2812_T g_tWs2812RGB;
void PAR_RGBUPdate(void);


void bsp_InitWS2812(void);

void BoardStartPointOut(void);

/* 供外部调用的函数声明 */
void Send_2811_24bits(uint8_t GData,uint8_t RData,uint8_t BData);
void rst(void);
void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b);
void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void SetPixelColor(uint16_t n, uint32_t c);
void PixelUpdate(void);
uint32_t Color(uint8_t r, uint8_t g, uint8_t b);
uint32_t Wheel(uint8_t WheelPos);
void rainbow(uint8_t wait);
void rainbowCycle(uint8_t wait);
void theaterChase(uint32_t c, uint8_t wait) ;
void theaterChaseRainbow(uint8_t wait) ;
void colorWipe(uint32_t c, uint8_t wait) ;

void BoardStartPointOut(void);  //把所有灯用到的颜色全部展示一遍，完成后响一声蜂鸣器

#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
