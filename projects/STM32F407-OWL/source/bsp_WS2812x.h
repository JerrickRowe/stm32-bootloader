/*
*********************************************************************************************************
*
*	ģ������ : WS2812x_RGB������ģ��
*	�ļ����� : bsp_WS2812x.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __BSP_WS2812x_H
#define __BSP_WS2812x_H

#include <stdint.h>

/* ����ȫ�ֱ����Ľṹ�� */

typedef struct {
	/* ��һյ�� STATE*/
	uint8_t R1;	 //��һյ�ƺ�ɫRGBֵ
	uint8_t G1;	 //��һյ����ɫRGBֵ
	uint8_t B1;	 //��һյ����ɫRGBֵ

	/* �ڶ�յ�� GRAND*/
	uint8_t R2;	 //�ڶ�յ�ƺ�ɫRGBֵ
	uint8_t G2;	 //�ڶ�յ����ɫRGBֵ
	uint8_t B2;	 //�ڶ�յ����ɫRGBֵ

	/* ����յ�� SIGNAL*/
	uint8_t R3;	 //����յ�ƺ�ɫRGBֵ
	uint8_t G3;	 //����յ����ɫRGBֵ
	uint8_t B3;	 //����յ����ɫRGBֵ

} WS2812_T;

extern WS2812_T g_tWs2812RGB;
void			PAR_RGBUPdate(void);

void bsp_InitWS2812(void);

void BoardStartPointOut(void);

/* ���ⲿ���õĺ������� */
void	 Send_2811_24bits(uint8_t GData, uint8_t RData, uint8_t BData);
void	 rst(void);
void	 setAllPixelColor(uint8_t r, uint8_t g, uint8_t b);
void	 setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void	 SetPixelColor(uint16_t n, uint32_t c);
void	 PixelUpdate(void);
uint32_t Color(uint8_t r, uint8_t g, uint8_t b);
uint32_t Wheel(uint8_t WheelPos);
void	 rainbow(uint8_t wait);
void	 rainbowCycle(uint8_t wait);
void	 theaterChase(uint32_t c, uint8_t wait);
void	 theaterChaseRainbow(uint8_t wait);
void	 colorWipe(uint32_t c, uint8_t wait);

void BoardStartPointOut(void);	//�����е��õ�����ɫȫ��չʾһ�飬��ɺ���һ��������

void bsp_WS2812_SyncAllPixel(void);
#endif

/***************************** ���������� www.armfly.com (END OF FILE)
 * *********************************/
