//#include＂bsp.h＂
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "bsp_WS2812x.h"


WS2812_T g_tWs2812RGB;  

#define		WS2812_PORT		GPIOB
#define 	WS2812_PIN		GPIO_Pin_13
 
#define		RGB_LED_HIGH	(GPIOB->ODR |= (1<<13))
#define 	RGB_LED_LOW		(GPIOB->ODR &= ~(1<<13))

#define numLEDs 3

void delay_100ns( uint16_t t ){
	while(--t){
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
	}
}

static inline void RGB_LED_Write0(void)
{
    RGB_LED_HIGH;
	delay_100ns(1);
    RGB_LED_LOW;
	delay_100ns(8);
}

static inline void RGB_LED_Write1(void)
{
    RGB_LED_HIGH;
	delay_100ns(8);
    RGB_LED_LOW;
	delay_100ns(1);
}


static void Send_8bits(uint8_t dat)
{
    uint8_t i;
//		RGB_LED_Write0();
    for(i=0; i<8; i++)
    {
        if(dat & 0x80)//1,for "1",H:0.8us,L:0.45us;
        {
            RGB_LED_Write1();
        }
        else 	//0 ,for "0",H:0.4us,L:	0.85us
        {
            RGB_LED_Write0();
        }
        dat=dat<<1;
    }
}

//R--G--B
//MSB first
void Send_2811_24bits(uint8_t RData,uint8_t GData,uint8_t BData)
{
	__disable_irq();
    Send_8bits(GData);
    Send_8bits(RData);
    Send_8bits(BData);
	__enable_irq();
}

void setAllPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
   Send_2811_24bits(r,g,b);
   Send_2811_24bits(r,g,b);
   Send_2811_24bits(r,g,b);
}

//void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
//{
//    echoDeng_u8Type i=0;
//    rBuffer[n]=r;
//    gBuffer[n]=g;
//    bBuffer[n]=b;
//    for(i=0; i<numLEDs; i++)
//    {
//        Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
//    }
//}

typedef uint8_t  echoDeng_u8Type;

echoDeng_u8Type rBuffer[numLEDs]= {0};
echoDeng_u8Type gBuffer[numLEDs]= {0};
echoDeng_u8Type bBuffer[numLEDs]= {0};

void SetPixelColor(uint16_t n, uint32_t c)
{
    echoDeng_u8Type i=0;
    rBuffer[n]=(uint8_t)(c>>16);
    gBuffer[n]=(uint8_t)(c>>8);
    bBuffer[n]=(uint8_t)c;
    for(i=0; i<numLEDs; i++)
    {
        Send_2811_24bits(rBuffer[i],gBuffer[i],bBuffer[i]);
    }
}

/*
*********************************************************************************************************
*   函 数 名: WS2812VarInit
*   功能说明: 初始化让所有灯灭掉
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/
static void WS2812VarInit(void)
{
	g_tWs2812RGB.R1 = 0x00;
	g_tWs2812RGB.G1 = 0x00;
	g_tWs2812RGB.B1 = 0x00;

	g_tWs2812RGB.R2 = 0x00;
	g_tWs2812RGB.G2 = 0x00;
	g_tWs2812RGB.B2 = 0x00;

	g_tWs2812RGB.R3 = 0x00;
	g_tWs2812RGB.G3 = 0x00;
	g_tWs2812RGB.B3 = 0x00;
}

/*
*********************************************************************************************************
*   函 数 名: bsp_InitWS2812Hard
*   功能说明: 初始化WS2812的GPIO口
*   形    参: 无
*   返 回 值: 无
*********************************************************************************************************
*/

static void bsp_InitWS2812Hard(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin   = GPIO_PIN_13;	// Tx
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init( GPIOB, &GPIO_InitStructure );
}

void bsp_InitWS2812(void)
{
	WS2812VarInit();
	bsp_InitWS2812Hard();
	setAllPixelColor(0,0,0);
	setAllPixelColor(0,0,0);
	setAllPixelColor(0,0,0);
	
	delay_100ns(100);
	setAllPixelColor(255,255,255);
}


