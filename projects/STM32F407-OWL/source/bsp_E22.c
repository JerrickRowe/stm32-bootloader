/*
*********************************************************************************************************
*
*	ģ������ : E22���ߺʹ���1FIFO����
*	�ļ����� : bsp_E22.c
*	��    �� : V2.0
*	˵    �� : ���bsp_uart_fifo.c����E22���ߴ����ڲ��Ĵ���,�����˴���1��FIFO����
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		V1.0               ԭ�Ӻ�  ��ʽ����
        v2.0    20200727   ԭ�Ӻ�  ���Ӷ��뺯�������ù���ֵ��
        v2.1    20200903   ԭ�Ӻ�  �޸Ķ��빫��ֵ
*
*
*********************************************************************************************************
*/

#include "bsp_E22.h"
#include "main.h"
#ifdef __cplusplus
extern "C"{
#endif

//#include "BspUsart.h"

//#include "FreeRTOS.h"
//#include "task.h"

//#define DEBUG_PRINT	1
//#include "debug_print.h"

#ifndef DEBUG
#define DEBUG 1
#endif
#if DEBUG
#include <stdio.h>
#define PRINT_RAW(fmt,...)	printf( fmt,##__VA_ARGS__ )
#define PRINT_INF(fmt,...)	printf( fmt"\r\n",##__VA_ARGS__ )
#define PRINT_ERR(fmt,...)	printf( fmt"\r\n",##__VA_ARGS__ )
#else
#define PRINT_RAW(fmt,...)	
#define PRINT_INF(fmt,...)	
#define PRINT_ERR(fmt,...)	
#endif


//#include "timer.h"
#define GET_TIME_MS()	HAL_GetTick()
#define WAIT_MS(ms)		Delay_MS(ms)

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__enable_irq()	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__disable_irq()	/* ��ֹȫ���ж� */

// E22������GPIO  PA8, PD10   AUX 
// #define E22_M0_GPIO_CLK_ENABLE()	  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
// #define E22_M0_GPIO_PORT              RC_M0_Port
// #define E22_M0_PIN                    RC_M0_Pin

#define E22_M1_GPIO_CLK_ENABLE()      //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
#define E22_M1_GPIO_PORT              GPIOD
#define E22_M1_PIN                    GPIO_PIN_10

#define E22_AUX_GPIO_CLK_ENABLE()      //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
#define E22_AUX_GPIO_PORT              GPIOA
#define E22_AUX_PIN                    GPIO_PIN_8

E22_T g_tE22Self;   //ң�����˵�E22�ṹ��  
E22_T g_tE22Other;	//����ɡ�˵�E22�ṹ��   ���Խ���Զ�����á�ֻ��ң����������


/* ����1��GPIO --- RS323 */
#define UART1_TX_PORT      GPIOA
#define UART1_TX_PIN       GPIO_PIN_9
#define UART1_TX_CLK       RCC_AHB1Periph_GPIOA
#define UART1_TX_SOURCE    GPIO_PinSource9

#define UART1_RX_PORT      GPIOA
#define UART1_RX_PIN       GPIO_PIN_10
#define UART1_RX_CLK       RCC_AHB1Periph_GPIOA
#define UART1_RX_SOURCE    GPIO_PinSource10


/* ����ÿ�����ڽṹ����� */
#if UART1_FIFO_EN == 1
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* ���ͻ����� */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* ���ջ����� */
#endif

static void UartVarInit(void);

static void InitHardUart(void);
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);
static void ConfigUartNVIC(void);

void RS485_InitTXE(void);


static void Delay_MS( uint32_t ms ){
	uint32_t target_tick = GET_TIME_MS() + ms;
	while( GET_TIME_MS() < target_tick );
}


/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUart
*	����˵��: ��ʼ������Ӳ��������ȫ�ֱ�������ֵ.
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	UartVarInit();		/* �����ȳ�ʼ��ȫ�ֱ���,������Ӳ�� */

	ConfigUartNVIC();	/* ���ô����ж� */
	
	InitHardUart();		/* ���ô��ڵ�Ӳ������(�����ʵ�) */
}

/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��ΪUARTָ��
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: uartָ��
*********************************************************************************************************
*/
UART_T *ComToUart(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		#if UART1_FIFO_EN == 1
			return &g_tUart1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM2)
	{
		#if UART2_FIFO_EN == 1
			return &g_tUart2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM3)
	{
		#if UART3_FIFO_EN == 1
			return &g_tUart3;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM4)
	{
		#if UART4_FIFO_EN == 1
			return &g_tUart4;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM5)
	{
		#if UART5_FIFO_EN == 1
			return &g_tUart5;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM6)
	{
		#if UART6_FIFO_EN == 1
			return &g_tUart6;
		#else
			return 0;
		#endif
	}
	else
	{
		/* �����κδ��� */
		return 0;
	}
}


/*
*********************************************************************************************************
*	�� �� ��: ComToUart
*	����˵��: ��COM�˿ں�ת��Ϊ USART_TypeDef* USARTx
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: USART_TypeDef*,  USART1, USART2, USART3, UART4, UART5
*********************************************************************************************************
*/
USART_TypeDef *ComToUSARTx(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		#if UART1_FIFO_EN == 1
			return USART1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM2)
	{
		#if UART2_FIFO_EN == 1
			return USART2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM3)
	{
		#if UART3_FIFO_EN == 1
			return USART3;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM4)
	{
		#if UART4_FIFO_EN == 1
			return USART4;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM5)
	{
		#if UART5_FIFO_EN == 1
			return USART5;
		#else
			return 0;
		#endif
	}
	else
	{
		/* �����κδ��� */
		return 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: comSendBuf
*	����˵��: �򴮿ڷ���һ�����ݡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*			  _ucaBuf: �����͵����ݻ�����
*			  _usLen : ���ݳ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
  while (_usLen--)
  {
     
  }
}

/*
*********************************************************************************************************
*	�� �� ��: comSendChar
*	����˵��: �򴮿ڷ���1���ֽڡ����ݷŵ����ͻ��������������أ����жϷ�������ں�̨��ɷ���
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*			  _ucByte: �����͵�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1);
}

/*
*********************************************************************************************************
*	�� �� ��: comGetChar
*	����˵��: �ӽ��ջ�������ȡ1�ֽڣ��������������������ݾ��������ء�
*	��    ��: _ucPort: �˿ں�(COM1 - COM5)
*			  _pByte: ���յ������ݴ���������ַ
*	�� �� ֵ: 0 ��ʾ������, 1 ��ʾ��ȡ����Ч�ֽ�
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return 0;
	}

	return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	�� �� ��: comClearTxFifo
*	����˵��: ���㴮�ڷ��ͻ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->usTxWrite = 0;
	pUart->usTxRead = 0;
	pUart->usTxCount = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: comClearRxFifo
*	����˵��: ���㴮�ڽ��ջ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM6)
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if (pUart == 0)
	{
		return;
	}

	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: comSetBaud
*	����˵��: ���ô��ڵĲ�����
*	��    ��: _ucPort: �˿ں�(COM1 - COM5)
*			  _BaudRate: �����ʣ�0-4500000�� ���4.5Mbps
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate)
{
	USART_TypeDef* USARTx;
	
	USARTx = ComToUSARTx(_ucPort);
	if (USARTx == 0)
	{
		return;
	}
	
	USART_SetBaudRate(USARTx, _BaudRate);
}

/*
*********************************************************************************************************
*	�� �� ��: USART_SetBaudRate
*	����˵��: �޸Ĳ����ʼĴ������������������á����ʹ�� USART_Init����, ����޸�Ӳ�����ز�����RX,TX����
*			  ���ݹ̼����� USART_Init���������������ò����ʵĲ��ֵ����������װΪһ������
*	��    ��: USARTx : USART1, USART2, USART3, UART4, UART5
*			  BaudRate : �����ʣ�ȡֵ 0 - 4500000
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static USART_HandleTypeDef rc_usart_handle;
void USART_SetBaudRate(USART_TypeDef* USARTx, uint32_t BaudRate)
{
	USART_HandleTypeDef *handle = &rc_usart_handle;
	
	handle->Instance = USART1;
	handle->State = HAL_USART_STATE_RESET;
	handle->Init.BaudRate = BaudRate;
	handle->Init.StopBits = USART_STOPBITS_1;
	handle->Init.WordLength = USART_WORDLENGTH_8B;
	handle->Init.Parity = USART_PARITY_NONE;
	handle->Init.Mode = USART_MODE_TX_RX;
	HAL_USART_Init( handle );
	__HAL_USART_ENABLE_IT( handle, USART_IT_RXNE );
	// SET_BIT( handle->Instance->CR1, USART_CR1_RXNEIE );
}




///*
//*********************************************************************************************************
//*	�� �� ��: RS485_SendStr
//*	����˵��: ��485���߷���һ���ַ�����0������
//*	��    ��: _pBuf �ַ�����0����
//*	�� �� ֵ: ��
//*********************************************************************************************************
//*/
//void RS485_SendStr(char *_pBuf)
//{
//	RS485_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
//}

///*
//*********************************************************************************************************
//*	�� �� ��: RS485_ReciveNew
//*	����˵��: ���յ��µ�����
//*	��    ��: _byte ���յ���������
//*	�� �� ֵ: ��
//*********************************************************************************************************
//*/
//void RS485_ReciveNew(uint8_t _byte)
//{
//}

/*
*********************************************************************************************************
*	�� �� ��: UartVarInit
*	����˵��: ��ʼ��������صı���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;						/* STM32 �����豸 */
	g_tUart1.pTxBuf = g_TxBuf1;					/* ���ͻ�����ָ�� */
	g_tUart1.pRxBuf = g_RxBuf1;					/* ���ջ�����ָ�� */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE;	/* ���ͻ�������С */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;	/* ���ջ�������С */
	g_tUart1.usTxWrite = 0;						/* ����FIFOд���� */
	g_tUart1.usTxRead = 0;						/* ����FIFO������ */
	g_tUart1.usRxWrite = 0;						/* ����FIFOд���� */
	g_tUart1.usRxRead = 0;						/* ����FIFO������ */
	g_tUart1.usRxCount = 0;						/* ���յ��������ݸ��� */
	g_tUart1.usTxCount = 0;						/* �����͵����ݸ��� */
	g_tUart1.SendBefor = 0;						/* ��������ǰ�Ļص����� */
	g_tUart1.SendOver = 0;						/* ������Ϻ�Ļص����� */
	g_tUart1.ReciveNew = 0;						/* ���յ������ݺ�Ļص����� */
#endif

}

/*
*********************************************************************************************************
*	�� �� ��: InitHardUart
*	����˵��: ���ô��ڵ�Ӳ�������������ʣ�����λ��ֹͣλ����ʼλ��У��λ���ж�ʹ�ܣ��ʺ���STM32-F4������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitHardUart(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin   = UART1_TX_PIN;	// Tx
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init( UART1_TX_PORT, &GPIO_InitStructure );
	
	GPIO_InitStructure.Pin   = UART1_RX_PIN;	// Rx
	GPIO_InitStructure.Mode  = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	GPIO_InitStructure.Pull  = GPIO_PULLUP;
	HAL_GPIO_Init( UART1_RX_PORT, &GPIO_InitStructure );
	
	
}

/*
*********************************************************************************************************
*	�� �� ��: ConfigUartNVIC
*	����˵��: ���ô���Ӳ���ж�.
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void ConfigUartNVIC(void)
{
	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	/* ʹ�ܴ���1�ж� */
//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	HAL_NVIC_EnableIRQ( USART1_IRQn );
//#endif

}

/*
*********************************************************************************************************
*	�� �� ��: UartSend
*	����˵��: ��д���ݵ�UART���ͻ�����,�����������жϡ��жϴ�����������Ϻ��Զ��رշ����ж�
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* ������ͻ������Ѿ����ˣ���ȴ��������� */
	#if 0
		/*
			�ڵ���GPRS����ʱ������Ĵ������������while ��ѭ��
			ԭ�� ���͵�1���ֽ�ʱ _pUart->usTxWrite = 1��_pUart->usTxRead = 0;
			������while(1) �޷��˳�
		*/
		while (1)
		{
			uint16_t usRead;

			DISABLE_INT();
			usRead = _pUart->usTxRead;
			ENABLE_INT();

			if (++usRead >= _pUart->usTxBufSize)
			{
				usRead = 0;
			}

			if (usRead != _pUart->usTxWrite)
			{
				break;
			}
		}
	#else
		/* �� _pUart->usTxBufSize == 1 ʱ, ����ĺ���������(������) */
		while (1)
		{
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = _pUart->usTxCount;
			ENABLE_INT();

			if (usCount < _pUart->usTxBufSize)
			{
				break;
			}
		}
	#endif

		/* �����������뷢�ͻ����� */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		DISABLE_INT();
		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
		{
			_pUart->usTxWrite = 0;
		}
		_pUart->usTxCount++;
		ENABLE_INT();
	}

	USART_ITConfig(_pUart->uart, USART_IT_TXE, ENABLE);
}

/*
*********************************************************************************************************
*	�� �� ��: UartGetChar
*	����˵��: �Ӵ��ڽ��ջ�������ȡ1�ֽ����� ��������������ã�
*	��    ��: _pUart : �����豸
*			  _pByte : ��Ŷ�ȡ���ݵ�ָ��
*	�� �� ֵ: 0 ��ʾ������  1��ʾ��ȡ������
*********************************************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{

	return 0;
}

/*
*********************************************************************************************************
*	�� �� ��: UartIRQ
*	����˵��: ���жϷ��������ã�ͨ�ô����жϴ�����
*	��    ��: _pUart : �����豸
*	�� �� ֵ: ��
*********************************************************************************************************
*/
/*
static void UartIRQ(UART_T *_pUart)
{
//	 ��������ж�  
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET)
	{
//		 �Ӵ��ڽ������ݼĴ�����ȡ���ݴ�ŵ�����FIFO 
		uint8_t ch;

		ch = USART_ReceiveData(_pUart->uart);
		_pUart->pRxBuf[_pUart->usRxWrite] = ch;
		if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
		{
			_pUart->usRxWrite = 0;
		}
		if (_pUart->usRxCount < _pUart->usRxBufSize)
		{
			_pUart->usRxCount++;
		}

// �ص�����,֪ͨӦ�ó����յ�������,һ���Ƿ���1����Ϣ��������һ����� 
		//if (_pUart->usRxWrite == _pUart->usRxRead)
		//if (_pUart->usRxCount == 1)
		{
			if (_pUart->ReciveNew)
			{
				_pUart->ReciveNew(ch);
			}
		}
	}

// �����ͻ��������ж� 
	if (USART_GetITStatus(_pUart->uart, USART_IT_TXE) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			// ���ͻ�������������ȡ��ʱ�� ��ֹ���ͻ��������ж� ��ע�⣺��ʱ���1�����ݻ�δ����������ϣ�
			USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);

			// ʹ�����ݷ�������ж� 
			USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			// �ӷ���FIFOȡ1���ֽ�д�봮�ڷ������ݼĴ��� 
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}

	}
	// ����bitλȫ��������ϵ��ж� 
	else if (USART_GetITStatus(_pUart->uart, USART_IT_TC) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			// �������FIFO������ȫ��������ϣ���ֹ���ݷ�������ж� 
			USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);

			// �ص�����, һ����������RS485ͨ�ţ���RS485оƬ����Ϊ����ģʽ��������ռ���� 
			if (_pUart->SendOver)
			{
				_pUart->SendOver();
			}
		}
		else
		{
			// ��������£��������˷�֧ 

			// �������FIFO�����ݻ�δ��ϣ���ӷ���FIFOȡ1������д�뷢�����ݼĴ��� 
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}
	}
}
*/

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUart
*	����˵��: ��ʼ������Ӳ��������ȫ�ֱ�������ֵ.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitE22(void)
{
	E22VarInit();		/* �����ȳ�ʼ������ȫ�ֱ���,������Ӳ�� */

	bsp_InitUart();
	
	InitHardE22();		/* ����E22��Ӳ������ */

}

/*
*********************************************************************************************************
*	�� �� ��: InitHardE22
*	����˵��: ��ʼ��E22�������ţ�������Ϊ����ģʽ0.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitHardE22(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/* ����M0���� */
	// E22_M0_GPIO_CLK_ENABLE();
	// GPIO_InitStruct.GPIO_Pin       = E22_M0_PIN;
	// GPIO_InitStruct.GPIO_Mode      = GPIO_Mode_OUT;
	// GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	// GPIO_InitStruct.GPIO_PuPd      = GPIO_PuPd_DOWN;
	// GPIO_InitStruct.GPIO_Speed     = GPIO_Speed_100MHz;
	// GPIO_Init(E22_M0_GPIO_PORT, &GPIO_InitStruct);	
	
	/* ����M1���� */
	E22_M1_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin       = E22_M1_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
	HAL_GPIO_Init(E22_M1_GPIO_PORT, &GPIO_InitStruct);	

	/* ����AUX���� */
	E22_AUX_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin       = E22_AUX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;  
	GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
	HAL_GPIO_Init(E22_AUX_GPIO_PORT, &GPIO_InitStruct);	

//	E22ParaConfigure();//�ϵ�����һ��  ʵ��ʹ���в���Ҫÿһ�ζ���ʼ��
	E22SetWorkMode(0); //���ù���ģʽΪ0

}

/*
*********************************************************************************************************
*	�� �� ��: E22VarInit
*	����˵��: ��ʼ��E22��صı���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void E22VarInit(void)
{
	/*��ʼ��ģ���ַ*/
	g_tE22Self.ADDH=0x00;
	g_tE22Self.ADDL=0x00;
	g_tE22Self.NETID=0x00;
	g_tE22Self.REG0=0xE5;   //���ڲ�����Ϊ115200������У��Ϊ8N1�����߿�������Ϊ19.2K
	g_tE22Self.REG1=0X00;   //000	
	g_tE22Self.REG2=0x17;
	g_tE22Self.REG3=0x00;
	g_tE22Self.CRYPT_H=0x00;
	g_tE22Self.CRYPT_L=0x00;//ʮ����Ϊ1

}

/*
*********************************************************************************************************
*	�� �� ��: E22SetWorkMode
*	����˵��: ֻ������ң�����ϵ�E22����ģʽ��
*	��    ��: 0:����ģʽ  1:WORģʽ  2:����ģʽ  3:�������    ����ģʽ��ֻ֧�� 9600��8N1 ��ʽ
*	�� �� ֵ: ��
*   ��ϸ˵���� 

һ��ģʽ��ģʽ 0��
���� �� M0 = 0��M1 = 0 ʱ��ģ�鹤����ģʽ 0
���� �û�����ͨ�������������ݣ�ģ����������߷��䡣
���� ģ�����߽��չ��ܴ򿪣��յ��������ݺ��ͨ������ TXD ���������

WOR ģʽ��ģʽ 1��
���� �� M0 = 1��M1 = 0 ʱ��ģ�鹤����ģʽ 1
���� ������Ϊ���䷽ʱ������ǰ���Զ�����һ��ʱ��Ļ�����
���� ���������������ݣ����չ��ܵ�ͬ��ģʽ 0

����ģʽ��ģʽ 2��
���� �� M0 = 0��M1 = 1 ʱ��ģ�鹤����ģʽ 2
���� ���߷���ر�
���� ���߽��չر�
���� �û����Է��ʼĴ������Ӷ�����ģ�鹤��״̬

�������ģʽ��ģʽ 3��
���� �� M0 = 1��M1 = 1 ʱ��ģ�鹤����ģʽ 3
���� �޷������������ݡ�
���� �޷������������ݡ�
ע��
��������ģʽ���뵽����ģʽ��ģ����������ò�����
���ù����У�AUX ���ֵ͵�ƽ�� ��Ϻ�����ߵ�ƽ��
���Խ����û���� T_BUSY �����ء�
*********************************************************************************************************
*/
static uint8_t E22WorkMode;
void E22SetWorkMode(uint8_t mode)
{
	uint32_t timestamp = GET_TIME_MS();
	while( HAL_GPIO_ReadPin(E22_AUX_GPIO_PORT, E22_AUX_PIN) == GPIO_PIN_RESET ){
		if( GET_TIME_MS() - timestamp > 1000 ){
			PRINT_ERR( "E22-AUX timeout." );
			break;
		}
		FeedGlobalWatchdog();
	}
	E22WorkMode = mode;
	if(mode == 0)
	{
	HAL_GPIO_WritePin(E22_M1_GPIO_PORT,E22_M1_PIN,GPIO_PIN_RESET);    //M1=0
	// GPIO_ResetBits(E22_M0_GPIO_PORT,E22_M0_PIN);    //M0=0
	USART_SetBaudRate( USART1, 115200 );
	}
	else if(mode == 1)
	{
	HAL_GPIO_WritePin(E22_M1_GPIO_PORT,E22_M1_PIN,GPIO_PIN_RESET);    //M1=0
	// GPIO_SetBits(E22_M0_GPIO_PORT,E22_M0_PIN);      //M0=1
	USART_SetBaudRate( USART1, 115200 );
	}
	else if(mode == 2)
	{
	HAL_GPIO_WritePin(E22_M1_GPIO_PORT,E22_M1_PIN,GPIO_PIN_SET);      //M1=1
	// GPIO_ResetBits(E22_M0_GPIO_PORT,E22_M0_PIN);    //M0=0
	USART_SetBaudRate( USART1, 9600 );
	}
	else if(mode == 3)
	{
	HAL_GPIO_WritePin(E22_M1_GPIO_PORT,E22_M1_PIN,GPIO_PIN_SET);    //M1=1
	// GPIO_SetBits(E22_M0_GPIO_PORT,E22_M0_PIN);    //M0=1
	USART_SetBaudRate( USART1, 115200 );
	}
	else
	{
//		Error_Handler(__FILE__, __LINE__);
//		printf("ѡ��E22ģʽ����");
		PRINT_ERR( "Invalid mode. [0~3]" );
	}
	WAIT_MS(40);
	timestamp = GET_TIME_MS();
	while( HAL_GPIO_ReadPin(E22_AUX_GPIO_PORT, E22_AUX_PIN) == GPIO_PIN_RESET ){
		if( GET_TIME_MS() - timestamp > 500 ){
			PRINT_ERR( "E22-AUX timeout." );
			break;
		}
		FeedGlobalWatchdog();
		WAIT_MS(1);
	}
}

uint8_t E22GetWorkMode( void ){
	return E22WorkMode;
}

/*
*********************************************************************************************************
*	�� �� ��: E22ParaConfigure
*	����˵��: ��������E22ģ�����������ģ��һ��Ҫ������ģʽ2������ģʽ��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void E22ParaConfigure(void)
{
	/*����ģʽ�µĴ��ڲ����ʱ�����9600*/
	comSetBaud(COM1,9600);
	/*��������ģ���ڹ���ģʽ2������ģʽ��*/
	E22SetWorkMode(2);
	WAIT_MS(300);
	/*��������ģ�����*/
	comSendChar(COM1,0xC0);	//�������üĴ���ָ��
	comSendChar(COM1,0x00);	//������ʼ��ַ
	comSendChar(COM1,0x09);	//���ͳ���
	comSendChar(COM1,g_tE22Self.ADDH);	//���͵�ַ���ֽ�
	comSendChar(COM1,g_tE22Self.ADDL);	//���͵�ַ���ֽ�
	comSendChar(COM1,g_tE22Self.NETID);	//��������ID
	comSendChar(COM1,g_tE22Self.REG0);	//���ͼĴ���0
	comSendChar(COM1,g_tE22Self.REG1);	//���ͼĴ���1
	comSendChar(COM1,g_tE22Self.REG2);	//���ͼĴ���2
	comSendChar(COM1,g_tE22Self.REG3);	//���ͼĴ���3
	comSendChar(COM1,g_tE22Self.CRYPT_H);	//������Կ���ֽ�
	comSendChar(COM1,g_tE22Self.CRYPT_L);	//������Կ���ֽ�

//��д����ȷ�Ϻ���
//��дԶ�����ú���   ����һ������������ÿ���ϵ綼��Ҫ����

	/*�ָ����ڲ�����Ϊ115200*/
	WAIT_MS(500);
	comSetBaud(COM1,115200);
	/*�ָ�E22����ģʽΪһ��ģʽ͸������*/
	E22SetWorkMode(0);
	WAIT_MS(20);

}

/*
*********************************************************************************************************
*	�� �� ��: E22ParaConfigure
*	����˵��: ����Զ��E22ģ�����������ģ��һ��Ҫ������ģʽ2������ģʽ����Զ��ģ����Ҫ������͸������ģʽ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void E22ParaConfigureOther(void)
{
	/*����ģʽ�µĴ��ڲ����ʱ�����9600*/
	comSetBaud(COM1,9600);
	/*��������ģ���ڹ���ģʽ2������ģʽ��*/
	E22SetWorkMode(2);
	WAIT_MS(20);
	/*��������ģ�����*/
	comSendChar(COM1,0xC0);	//�������üĴ���ָ��
	comSendChar(COM1,0x00);	//������ʼ��ַ
	comSendChar(COM1,0x09);	//���ͳ���
	comSendChar(COM1,g_tE22Self.ADDH);	//���͵�ַ���ֽ�
	comSendChar(COM1,g_tE22Self.ADDL);	//���͵�ַ���ֽ�
	comSendChar(COM1,g_tE22Self.NETID);	//��������ID
	comSendChar(COM1,g_tE22Self.REG0);	//���ͼĴ���0
	comSendChar(COM1,g_tE22Self.REG1);	//���ͼĴ���1
	comSendChar(COM1,g_tE22Self.REG2);	//���ͼĴ���2
	comSendChar(COM1,g_tE22Self.REG3);	//���ͼĴ���3
	comSendChar(COM1,g_tE22Self.CRYPT_H);	//������Կ���ֽ�
	comSendChar(COM1,g_tE22Self.CRYPT_L);	//������Կ���ֽ�

//��д����ȷ�Ϻ���
//��дԶ�����ú���

	/*�ָ����ڲ�����Ϊ115200*/
	WAIT_MS(300);
	comSetBaud(COM1,115200);
	/*�ָ�E22����ģʽΪһ��ģʽ͸������*/
	E22SetWorkMode(0);
	WAIT_MS(20);

}

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUart
*	����˵��: ��ʼ������Ӳ��������ȫ�ֱ�������ֵ.
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitE22AndUart1(void)
{
	
	bsp_InitE22();
	bsp_InitUart();

}

/*
*********************************************************************************************************
*	�� �� ��: E22ParaConfigure
*	����˵��: ��������E22ģ��Ϊ����ֵ���������ģʽ��������������E22�Ĵ���ΪĬ��ֵ�����鿴��������ɡ��Ҫ������ΪĬ�ϲ�����Ȼ��Զ������
�Է�Ϊ����ң��������Ψһ�����룬����ٻָ���������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void E22SetPublicConfigure(void)
{
	uint8_t publicConfigurePar[] ={0xC0,0x00,0x09,0xFF,0XFF,0X00,0XE4,0XC0,0X17,0X00,0X00,0X01};

	E22_Config_Write( publicConfigurePar, sizeof(publicConfigurePar) );
	
	/*�ָ����ڲ�����Ϊ115200*/
	WAIT_MS(300);
	comSetBaud(COM1,115200);
	/*�ָ�E22����ģʽΪһ��ģʽ͸������*/
	E22SetWorkMode(0);
	WAIT_MS(20);
}


void E22_Config_Write( uint8_t* pData, size_t n )
{
	/*��������ģ���ڹ���ģʽ2������ģʽ��*/
	E22SetWorkMode(2);
	WAIT_MS(100);
	uint8_t s_sendCount = 0;
	DISABLE_INT();

	HAL_USART_Transmit( &rc_usart_handle, pData, n, 0 );
	// E22SetWorkMode(0);
	ENABLE_INT();
}


void _RC_bsp_SendByte( uint16_t byte ){
	HAL_USART_Transmit( &rc_usart_handle, &byte, 1, 0 );
}

void _RC_bsp_SendNByte( uint16_t *pData, int n ){
	HAL_USART_Transmit( &rc_usart_handle, &pData, n, 0 );
}


void _RC_bsp_ConfigUpgradeMode( void ){
	// Baud 9600
	uint8_t config_03[] ={0xC0,0x03,0x01,0x67};
	// Baud 57600
	// uint8_t config_03[] ={0xC0,0x03,0x01,0xC7};
	// Baud 115200
	// uint8_t config_03[] ={0xC0,0x03,0x01,0xE7};
	uint8_t config_06[] ={0xC0,0x06,0x01,0x00};

	E22_Config_Write( config_03, sizeof(config_03) );
	WAIT_MS(500);
	E22_Config_Write( config_06, sizeof(config_06) );
	WAIT_MS(500);
	
	/*�ָ����ڲ�����Ϊ115200*/
	// WAIT_MS(300);
	comSetBaud(COM1,9600);
	/*�ָ�E22����ģʽΪһ��ģʽ͸������*/
	E22SetWorkMode(0);
	WAIT_MS(200);
}


void _RC_bsp_RecoverFromUpgradeMode( void ){

	uint8_t config_03[] ={0xC0,0x03,0x01,0xE3};
	uint8_t config_06[] ={0xC0,0x06,0x01,0x80};

	E22_Config_Write( config_03, sizeof(config_03) );
	WAIT_MS(100);
	E22_Config_Write( config_06, sizeof(config_06) );
	WAIT_MS(100);
	
	/*�ָ����ڲ�����Ϊ115200*/
	// WAIT_MS(300);
	comSetBaud(COM1,115200);
	/*�ָ�E22����ģʽΪһ��ģʽ͸������*/
	E22SetWorkMode(0);
	WAIT_MS(200);
}


static uint32_t RxTimestamp;
static uint32_t dataCnt = 0;
void USART1_IRQHandler( void ){
	FeedGlobalWatchdog();
	if (__HAL_USART_GET_FLAG(&rc_usart_handle, USART_FLAG_RXNE) ){
		uint16_t data = rc_usart_handle.Instance->DR;
		RxTimestamp = GET_TIME_MS();
		dataCnt ++;
	}
	// HAL_USART_IRQHandler( &rc_usart_handle );
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart){
	RxTimestamp = GET_TIME_MS();
	dataCnt ++;
}

uint32_t bsp_E22_GetLastRxTimestamp( void ){
	return RxTimestamp;
}

uint32_t bsp_E22_GetIncomingDataCnt( void ){
	return dataCnt;
}

void bsp_E22_ClearIncomingData( void ){
	dataCnt = 0;
}


#ifdef __cplusplus
}
#endif

