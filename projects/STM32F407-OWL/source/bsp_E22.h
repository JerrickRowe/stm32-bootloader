/*
*********************************************************************************************************
*
*	模块名称 : 串口中断+FIFO驱动模块
*	文件名称 : bsp_E22.h
*	说    明 : 头文件
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_E22_H_
#define _BSP_E22_H_

#ifdef __cplusplus
extern "C"{
#endif
	
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"


#define	UART1_FIFO_EN	1

/* 定义端口号 */
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 或  PB6, PB7*/
	COM2 = 1,	/* USART2, PD5,PD6 或 PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
	COM4 = 3,	/* UART4, PC10, PC11 */
	COM5 = 4,	/* UART5, PC12, PD2 */
	COM6 = 5	/* USART6, PC6, PC7 */
}COM_PORT_E;

/* 定义串口波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
#if UART1_FIFO_EN == 1
	#define UART1_BAUD			115200
	#define UART1_TX_BUF_SIZE	1*128
	#define UART1_RX_BUF_SIZE	1*10
#endif

/* 串口设备结构体 */
typedef struct
{
	USART_TypeDef *uart;		/* STM32内部串口设备指针 */
	uint8_t *pTxBuf;			/* 发送缓冲区 */
	uint8_t *pRxBuf;			/* 接收缓冲区 */
	uint16_t usTxBufSize;		/* 发送缓冲区大小 */
	uint16_t usRxBufSize;		/* 接收缓冲区大小 */
	__IO uint16_t usTxWrite;	/* 发送缓冲区写指针 */
	__IO uint16_t usTxRead;		/* 发送缓冲区读指针 */
	__IO uint16_t usTxCount;	/* 等待发送的数据个数 */

	__IO uint16_t usRxWrite;	/* 接收缓冲区写指针 */
	__IO uint16_t usRxRead;		/* 接收缓冲区读指针 */
	__IO uint16_t usRxCount;	/* 还未读取的新数据个数 */

	void (*SendBefor)(void); 	/* 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） */
	void (*SendOver)(void); 	/* 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） */
	void (*ReciveNew)(uint8_t _byte);	/* 串口收到数据的回调函数指针 */
}UART_T;

void bsp_InitUart(void);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte);
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte);
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen);
void comClearTxFifo(COM_PORT_E _ucPort);
void comClearRxFifo(COM_PORT_E _ucPort);
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate);

void USART_SetBaudRate(USART_TypeDef* USARTx, uint32_t BaudRate);



/* E22无线模块寄存器结构体 */
typedef struct
{
	uint8_t ADDH;		//00H模块地址高字节
	uint8_t ADDL;		//01H模块地址低字节
	uint8_t NETID;	//02H网络地址  相互通信应该设置为相同
	uint8_t REG0;    //03H-E5  串口波特率115200 串口8N1 无线空中速率19.2k
	uint8_t REG1;    //04H-20    分包240字节  RSSI环境噪声使能  发射功率22dbm
	uint8_t REG2;		//05H- 0    CH 实际频率= 410.125 + CH *1M    信道不同不能通信
	uint8_t REG3;		//06H- 80启用RSSI字节  在每个串口数据后面加一个
	uint8_t CRYPT_H;  //07H  密钥高字节
	uint8_t CRYPT_L; 	//08H	密钥低字节
}E22_T;



void bsp_InitE22(void);
uint8_t E22GetWorkMode( void );
void E22SetWorkMode(uint8_t);
static void InitHardE22(void);
static void E22VarInit(void);
void E22ParaConfigure(void);

void bsp_InitE22AndUart1(void);
void E22SetPublicConfigure(void);

void E22_Config_Write( uint8_t*, size_t );

void _RC_bsp_RecoverFromUpgradeMode( void );

void _RC_bsp_SendNByte( uint16_t *pData, int n );
uint32_t bsp_E22_GetLastRxTimestamp( void );
uint32_t bsp_E22_GetIncomingDataCnt( void );
void bsp_E22_ClearIncomingData( void );

#ifdef __cplusplus
}
#endif

#endif

