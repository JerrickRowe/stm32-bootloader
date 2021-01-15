/*
*********************************************************************************************************
*
*	ģ������ : �����ж�+FIFO����ģ��
*	�ļ����� : bsp_E22.h
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2015-2020, ���������� www.armfly.com
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

/* ����˿ں� */
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 ��  PB6, PB7*/
	COM2 = 1,	/* USART2, PD5,PD6 �� PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
	COM4 = 3,	/* UART4, PC10, PC11 */
	COM5 = 4,	/* UART5, PC12, PD2 */
	COM6 = 5	/* USART6, PC6, PC7 */
}COM_PORT_E;

/* ���崮�ڲ����ʺ�FIFO��������С����Ϊ���ͻ������ͽ��ջ�����, ֧��ȫ˫�� */
#if UART1_FIFO_EN == 1
	#define UART1_BAUD			115200
	#define UART1_TX_BUF_SIZE	1*128
	#define UART1_RX_BUF_SIZE	1*10
#endif

/* �����豸�ṹ�� */
typedef struct
{
	USART_TypeDef *uart;		/* STM32�ڲ������豸ָ�� */
	uint8_t *pTxBuf;			/* ���ͻ����� */
	uint8_t *pRxBuf;			/* ���ջ����� */
	uint16_t usTxBufSize;		/* ���ͻ�������С */
	uint16_t usRxBufSize;		/* ���ջ�������С */
	__IO uint16_t usTxWrite;	/* ���ͻ�����дָ�� */
	__IO uint16_t usTxRead;		/* ���ͻ�������ָ�� */
	__IO uint16_t usTxCount;	/* �ȴ����͵����ݸ��� */

	__IO uint16_t usRxWrite;	/* ���ջ�����дָ�� */
	__IO uint16_t usRxRead;		/* ���ջ�������ָ�� */
	__IO uint16_t usRxCount;	/* ��δ��ȡ�������ݸ��� */

	void (*SendBefor)(void); 	/* ��ʼ����֮ǰ�Ļص�����ָ�루��Ҫ����RS485�л�������ģʽ�� */
	void (*SendOver)(void); 	/* ������ϵĻص�����ָ�루��Ҫ����RS485������ģʽ�л�Ϊ����ģʽ�� */
	void (*ReciveNew)(uint8_t _byte);	/* �����յ����ݵĻص�����ָ�� */
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



/* E22����ģ��Ĵ����ṹ�� */
typedef struct
{
	uint8_t ADDH;		//00Hģ���ַ���ֽ�
	uint8_t ADDL;		//01Hģ���ַ���ֽ�
	uint8_t NETID;	//02H�����ַ  �໥ͨ��Ӧ������Ϊ��ͬ
	uint8_t REG0;    //03H-E5  ���ڲ�����115200 ����8N1 ���߿�������19.2k
	uint8_t REG1;    //04H-20    �ְ�240�ֽ�  RSSI��������ʹ��  ���书��22dbm
	uint8_t REG2;		//05H- 0    CH ʵ��Ƶ��= 410.125 + CH *1M    �ŵ���ͬ����ͨ��
	uint8_t REG3;		//06H- 80����RSSI�ֽ�  ��ÿ���������ݺ����һ��
	uint8_t CRYPT_H;  //07H  ��Կ���ֽ�
	uint8_t CRYPT_L; 	//08H	��Կ���ֽ�
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

