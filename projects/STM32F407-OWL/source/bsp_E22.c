/*
*********************************************************************************************************
*
*	模块名称 : E22无线和串口1FIFO驱动
*	文件名称 : bsp_E22.c
*	版    本 : V2.0
*	说    明 : 结合bsp_uart_fifo.c控制E22无线串口内部寄存器,附带了串口1的FIFO驱动
*	修改记录 :
*		版本号  日期       作者    说明
*		V1.0               原子号  正式发布
		v2.0    20200727   原子号  增加对码函数（设置公共值）
		v2.1    20200903   原子号  修改对码公共值
*
*
*********************************************************************************************************
*/

#include "bsp_E22.h"
#include "main.h"
#ifdef __cplusplus
extern "C" {
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
#define PRINT_RAW(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define PRINT_INF(fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#define PRINT_ERR(fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#else
#define PRINT_RAW(fmt, ...)
#define PRINT_INF(fmt, ...)
#define PRINT_ERR(fmt, ...)
#endif

//#include "timer.h"
#define GET_TIME_MS() HAL_GetTick()
#define WAIT_MS(ms)	  Delay_MS(ms)

/* 开关全局中断的宏 */
#define ENABLE_INT()  __enable_irq()  /* 使能全局中断 */
#define DISABLE_INT() __disable_irq() /* 禁止全局中断 */

// E22的配置GPIO  PA8, PD10   AUX
// #define E22_M0_GPIO_CLK_ENABLE()	  __HAL_RCC_GPIOA_CLK_ENABLE()
// #define E22_M0_GPIO_PORT              RC_M0_Port
// #define E22_M0_PIN                    RC_M0_Pin

#define E22_M1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define E22_M1_GPIO_PORT		 GPIOD
#define E22_M1_PIN				 GPIO_PIN_10

#define E22_AUX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define E22_AUX_GPIO_PORT		  GPIOA
#define E22_AUX_PIN				  GPIO_PIN_8

E22_T g_tE22Self;  //遥控器端的E22结构体
E22_T g_tE22Other;	//降落伞端的E22结构体   可以进行远程配置。只在遥控器端有用

/* 串口1的GPIO --- RS323 */
#define UART1_TX_PORT	GPIOA
#define UART1_TX_PIN	GPIO_PIN_9
#define UART1_TX_CLK	RCC_AHB1Periph_GPIOA
#define UART1_TX_SOURCE GPIO_PinSource9

#define UART1_RX_PORT	GPIOA
#define UART1_RX_PIN	GPIO_PIN_10
#define UART1_RX_CLK	RCC_AHB1Periph_GPIOA
#define UART1_RX_SOURCE GPIO_PinSource10

/* 定义每个串口结构体变量 */
#if UART1_FIFO_EN == 1
static UART_T  g_tUart1;
static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE]; /* 发送缓冲区 */
static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE]; /* 接收缓冲区 */
#endif

static void UartVarInit(void);

static void	   InitHardUart(void);
static void	   UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void	   UartIRQ(UART_T *_pUart);
static void	   ConfigUartNVIC(void);

void RS485_InitTXE(void);

static void Delay_MS(uint32_t ms) {
	uint32_t target_tick = GET_TIME_MS() + ms;
	while(GET_TIME_MS() < target_tick)
		;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitUart(void) {
	UartVarInit(); /* 必须先初始化全局变量,再配置硬件 */

	ConfigUartNVIC(); /* 配置串口中断 */

	InitHardUart(); /* 配置串口的硬件参数(波特率等) */
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为UART指针
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: uart指针
*********************************************************************************************************
*/
UART_T *ComToUart(COM_PORT_E _ucPort) {
	if(_ucPort == COM1) {
#if UART1_FIFO_EN == 1
		return &g_tUart1;
#else
		return 0;
#endif
	} else if(_ucPort == COM2) {
#if UART2_FIFO_EN == 1
		return &g_tUart2;
#else
		return 0;
#endif
	} else if(_ucPort == COM3) {
#if UART3_FIFO_EN == 1
		return &g_tUart3;
#else
		return 0;
#endif
	} else if(_ucPort == COM4) {
#if UART4_FIFO_EN == 1
		return &g_tUart4;
#else
		return 0;
#endif
	} else if(_ucPort == COM5) {
#if UART5_FIFO_EN == 1
		return &g_tUart5;
#else
		return 0;
#endif
	} else if(_ucPort == COM6) {
#if UART6_FIFO_EN == 1
		return &g_tUart6;
#else
		return 0;
#endif
	} else {
		/* 不做任何处理 */
		return 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为 USART_TypeDef* USARTx
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: USART_TypeDef*,  USART1, USART2, USART3, UART4, UART5
*********************************************************************************************************
*/
USART_TypeDef *ComToUSARTx(COM_PORT_E _ucPort) {
	if(_ucPort == COM1) {
#if UART1_FIFO_EN == 1
		return USART1;
#else
		return 0;
#endif
	} else if(_ucPort == COM2) {
#if UART2_FIFO_EN == 1
		return USART2;
#else
		return 0;
#endif
	} else if(_ucPort == COM3) {
#if UART3_FIFO_EN == 1
		return USART3;
#else
		return 0;
#endif
	} else if(_ucPort == COM4) {
#if UART4_FIFO_EN == 1
		return USART4;
#else
		return 0;
#endif
	} else if(_ucPort == COM5) {
#if UART5_FIFO_EN == 1
		return USART5;
#else
		return 0;
#endif
	} else {
		/* 不做任何处理 */
		return 0;
	}
}

/*
*********************************************************************************************************
*	函 数 名: comSendBuf
*	功能说明: 向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _ucaBuf: 待发送的数据缓冲区
*			  _usLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen) {
	while(_usLen--) {
	}
}

/*
*********************************************************************************************************
*	函 数 名: comSendChar
*	功能说明: 向串口发送1个字节。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _ucByte: 待发送的数据
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte) {
	comSendBuf(_ucPort, &_ucByte, 1);
}

/*
*********************************************************************************************************
*	函 数 名: comGetChar
*	功能说明: 从接收缓冲区读取1字节，非阻塞。无论有无数据均立即返回。
*	形    参: _ucPort: 端口号(COM1 - COM5)
*			  _pByte: 接收到的数据存放在这个地址
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte) {
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if(pUart == 0) {
		return 0;
	}

	return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	函 数 名: comClearTxFifo
*	功能说明: 清零串口发送缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT_E _ucPort) {
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if(pUart == 0) {
		return;
	}

	pUart->usTxWrite = 0;
	pUart->usTxRead	 = 0;
	pUart->usTxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comClearRxFifo
*	功能说明: 清零串口接收缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT_E _ucPort) {
	UART_T *pUart;

	pUart = ComToUart(_ucPort);
	if(pUart == 0) {
		return;
	}

	pUart->usRxWrite = 0;
	pUart->usRxRead	 = 0;
	pUart->usRxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comSetBaud
*	功能说明: 设置串口的波特率
*	形    参: _ucPort: 端口号(COM1 - COM5)
*			  _BaudRate: 波特率，0-4500000， 最大4.5Mbps
*	返 回 值: 无
*********************************************************************************************************
*/
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate) {
	USART_TypeDef *USARTx;

	USARTx = ComToUSARTx(_ucPort);
	if(USARTx == 0) {
		return;
	}

	USART_SetBaudRate(USARTx, _BaudRate);
}

/*
*********************************************************************************************************
*	函 数 名: USART_SetBaudRate
*	功能说明: 修改波特率寄存器，不更改其他设置。如果使用 USART_Init函数,
*则会修改硬件流控参数和RX,TX配置 根据固件库中
*USART_Init函数，将其中配置波特率的部分单独提出来封装为一个函数 形    参: USARTx : USART1, USART2,
*USART3, UART4, UART5 BaudRate : 波特率，取值 0 - 4500000 返 回 值: 无
*********************************************************************************************************
*/
static USART_HandleTypeDef rc_usart_handle;
void					   USART_SetBaudRate(USART_TypeDef *USARTx, uint32_t BaudRate) {
	  USART_HandleTypeDef *handle = &rc_usart_handle;

	  handle->Instance		  = USART1;
	  handle->State			  = HAL_USART_STATE_RESET;
	  handle->Init.BaudRate	  = BaudRate;
	  handle->Init.StopBits	  = USART_STOPBITS_1;
	  handle->Init.WordLength = USART_WORDLENGTH_8B;
	  handle->Init.Parity	  = USART_PARITY_NONE;
	  handle->Init.Mode		  = USART_MODE_TX_RX;
	  HAL_USART_Init(handle);
	  __HAL_USART_ENABLE_IT(handle, USART_IT_RXNE);
	  CLEAR_BIT(handle->Instance->CR2, USART_CR2_CLKEN);
}

///*
//*********************************************************************************************************
//*	函 数 名: RS485_SendStr
//*	功能说明: 向485总线发送一个字符串，0结束。
//*	形    参: _pBuf 字符串，0结束
//*	返 回 值: 无
//*********************************************************************************************************
//*/
// void RS485_SendStr(char *_pBuf)
//{
//	RS485_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
//}

///*
//*********************************************************************************************************
//*	函 数 名: RS485_ReciveNew
//*	功能说明: 接收到新的数据
//*	形    参: _byte 接收到的新数据
//*	返 回 值: 无
//*********************************************************************************************************
//*/
// void RS485_ReciveNew(uint8_t _byte)
//{
//}

/*
*********************************************************************************************************
*	函 数 名: UartVarInit
*	功能说明: 初始化串口相关的变量
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartVarInit(void) {
#if UART1_FIFO_EN == 1
	g_tUart1.uart		 = USART1;			  /* STM32 串口设备 */
	g_tUart1.pTxBuf		 = g_TxBuf1;		  /* 发送缓冲区指针 */
	g_tUart1.pRxBuf		 = g_RxBuf1;		  /* 接收缓冲区指针 */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE; /* 发送缓冲区大小 */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE; /* 接收缓冲区大小 */
	g_tUart1.usTxWrite	 = 0;				  /* 发送FIFO写索引 */
	g_tUart1.usTxRead	 = 0;				  /* 发送FIFO读索引 */
	g_tUart1.usRxWrite	 = 0;				  /* 接收FIFO写索引 */
	g_tUart1.usRxRead	 = 0;				  /* 接收FIFO读索引 */
	g_tUart1.usRxCount	 = 0;				  /* 接收到的新数据个数 */
	g_tUart1.usTxCount	 = 0;				  /* 待发送的数据个数 */
	g_tUart1.SendBefor	 = 0;				  /* 发送数据前的回调函数 */
	g_tUart1.SendOver	 = 0;				  /* 发送完毕后的回调函数 */
	g_tUart1.ReciveNew	 = 0;				  /* 接收到新数据后的回调函数 */
#endif
}

/*
*********************************************************************************************************
*	函 数 名: InitHardUart
*	功能说明:
*配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）适合于STM32-F4开发板 形 参:
*无 返 回 值: 无
*********************************************************************************************************
*/
static void InitHardUart(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin		 = UART1_TX_PIN;  // Tx
	GPIO_InitStructure.Mode		 = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed	 = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	GPIO_InitStructure.Pull		 = GPIO_PULLUP;
	HAL_GPIO_Init(UART1_TX_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin		 = UART1_RX_PIN;  // Rx
	GPIO_InitStructure.Mode		 = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed	 = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
	GPIO_InitStructure.Pull		 = GPIO_PULLUP;
	HAL_GPIO_Init(UART1_RX_PORT, &GPIO_InitStructure);
}

/*
*********************************************************************************************************
*	函 数 名: ConfigUartNVIC
*	功能说明: 配置串口硬件中断.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ConfigUartNVIC(void) {
	//	NVIC_InitTypeDef NVIC_InitStructure;
	//	/* 使能串口1中断 */
	//	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//	NVIC_Init(&NVIC_InitStructure);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	//#endif
}

/*
*********************************************************************************************************
*	函 数 名: UartSend
*	功能说明: 填写数据到UART发送缓冲区,并启动发送中断。中断处理函数发送完毕后，自动关闭发送中断
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen) {
	uint16_t i;

	for(i = 0; i < _usLen; i++) {
		/* 如果发送缓冲区已经满了，则等待缓冲区空 */
#if 0
		/*
			在调试GPRS例程时，下面的代码出现死机，while 死循环
			原因： 发送第1个字节时 _pUart->usTxWrite = 1；_pUart->usTxRead = 0;
			将导致while(1) 无法退出
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
		/* 当 _pUart->usTxBufSize == 1 时, 下面的函数会死掉(待完善) */
		while(1) {
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = _pUart->usTxCount;
			ENABLE_INT();

			if(usCount < _pUart->usTxBufSize) {
				break;
			}
		}
#endif

		/* 将新数据填入发送缓冲区 */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		DISABLE_INT();
		if(++_pUart->usTxWrite >= _pUart->usTxBufSize) {
			_pUart->usTxWrite = 0;
		}
		_pUart->usTxCount++;
		ENABLE_INT();
	}

	USART_ITConfig(_pUart->uart, USART_IT_TXE, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: UartGetChar
*	功能说明: 从串口接收缓冲区读取1字节数据 （用于主程序调用）
*	形    参: _pUart : 串口设备
*			  _pByte : 存放读取数据的指针
*	返 回 值: 0 表示无数据  1表示读取到数据
*********************************************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte) {
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: UartIRQ
*	功能说明: 供中断服务程序调用，通用串口中断处理函数
*	形    参: _pUart : 串口设备
*	返 回 值: 无
*********************************************************************************************************
*/
/*
static void UartIRQ(UART_T *_pUart)
{
//	 处理接收中断
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET)
	{
//		 从串口接收数据寄存器读取数据存放到接收FIFO
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

// 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记
		//if (_pUart->usRxWrite == _pUart->usRxRead)
		//if (_pUart->usRxCount == 1)
		{
			if (_pUart->ReciveNew)
			{
				_pUart->ReciveNew(ch);
			}
		}
	}

// 处理发送缓冲区空中断
	if (USART_GetITStatus(_pUart->uart, USART_IT_TXE) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			// 发送缓冲区的数据已取完时， 禁止发送缓冲区空中断
（注意：此时最后1个数据还未真正发送完毕） USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);

			// 使能数据发送完毕中断
			USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			// 从发送FIFO取1个字节写入串口发送数据寄存器
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
			{
				_pUart->usTxRead = 0;
			}
			_pUart->usTxCount--;
		}

	}
	// 数据bit位全部发送完毕的中断
	else if (USART_GetITStatus(_pUart->uart, USART_IT_TC) != RESET)
	{
		//if (_pUart->usTxRead == _pUart->usTxWrite)
		if (_pUart->usTxCount == 0)
		{
			// 如果发送FIFO的数据全部发送完毕，禁止数据发送完毕中断
			USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);

			// 回调函数, 一般用来处理RS485通信，将RS485芯片设置为接收模式，避免抢占总线
			if (_pUart->SendOver)
			{
				_pUart->SendOver();
			}
		}
		else
		{
			// 正常情况下，不会进入此分支

			// 如果发送FIFO的数据还未完毕，则从发送FIFO取1个数据写入发送数据寄存器
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
*	函 数 名: bsp_InitUart
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitE22(void) {
	USART_SetBaudRate(USART1, 115200);

	E22VarInit(); /* 必须先初始化配置全局变量,再配置硬件 */

	InitHardE22(); /* 配置E22的硬件参数 */

	bsp_InitUart();
}

/*
*********************************************************************************************************
*	函 数 名: InitHardE22
*	功能说明: 初始化E22三个引脚，并配置为工作模式0.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void InitHardE22(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* 配置M0引脚 */
	// E22_M0_GPIO_CLK_ENABLE();
	// GPIO_InitStruct.GPIO_Pin       = E22_M0_PIN;
	// GPIO_InitStruct.GPIO_Mode      = GPIO_Mode_OUT;
	// GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	// GPIO_InitStruct.GPIO_PuPd      = GPIO_PuPd_DOWN;
	// GPIO_InitStruct.GPIO_Speed     = GPIO_Speed_100MHz;
	// GPIO_Init(E22_M0_GPIO_PORT, &GPIO_InitStruct);

	/* 配置M1引脚 */
	E22_M1_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin	  = E22_M1_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(E22_M1_GPIO_PORT, &GPIO_InitStruct);

	/* 配置AUX引脚 */
	E22_AUX_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin	  = E22_AUX_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(E22_AUX_GPIO_PORT, &GPIO_InitStruct);

	//	E22ParaConfigure();//上电配置一下  实际使用中不需要每一次都初始化
	E22SetWorkMode(0);	//设置工作模式为0
}

/*
*********************************************************************************************************
*	函 数 名: E22VarInit
*	功能说明: 初始化E22相关的变量
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void E22VarInit(void) {
	/*初始化模块地址*/
	g_tE22Self.ADDH	 = 0x00;
	g_tE22Self.ADDL	 = 0x00;
	g_tE22Self.NETID = 0x00;
	g_tE22Self.REG0 = 0xE5;	 //串口波特率为115200，串口校验为8N1，无线空中速率为19.2K
	g_tE22Self.REG1	   = 0X00;	// 000
	g_tE22Self.REG2	   = 0x17;
	g_tE22Self.REG3	   = 0x00;
	g_tE22Self.CRYPT_H = 0x00;
	g_tE22Self.CRYPT_L = 0x00;	//十进制为1
}

/*
*********************************************************************************************************
*	函 数 名: E22SetWorkMode
*	功能说明: 只能设置遥控器上的E22工作模式。
*	形    参: 0:传输模式  1:WOR模式  2:配置模式  3:深度休眠    配置模式下只支持 9600，8N1 格式
*	返 回 值: 无
*   详细说明：

一般模式（模式 0）
类型 当 M0 = 0，M1 = 0 时，模块工作在模式 0
发射 用户可以通过串口输入数据，模块会启动无线发射。
接收 模块无线接收功能打开，收到无线数据后会通过串口 TXD 引脚输出。

WOR 模式（模式 1）
类型 当 M0 = 1，M1 = 0 时，模块工作在模式 1
发射 当定义为发射方时，发射前会自动增加一定时间的唤醒码
接收 可以正常接收数据，接收功能等同于模式 0

配置模式（模式 2）
类型 当 M0 = 0，M1 = 1 时，模块工作在模式 2
发射 无线发射关闭
接收 无线接收关闭
配置 用户可以访问寄存器，从而配置模块工作状态

深度休眠模式（模式 3）
类型 当 M0 = 1，M1 = 1 时，模块工作在模式 3
发射 无法发射无线数据。
接收 无法接收无线数据。
注意
当从休眠模式进入到其他模式，模块会重新配置参数，
配置过程中，AUX 保持低电平； 完毕后输出高电平，
所以建议用户检测 T_BUSY 上升沿。
*********************************************************************************************************
*/
static uint8_t E22WorkMode;
void		   E22SetWorkMode(uint8_t mode) {
	  uint32_t timestamp = GET_TIME_MS();
	  while(HAL_GPIO_ReadPin(E22_AUX_GPIO_PORT, E22_AUX_PIN) == GPIO_PIN_RESET) {
		  if(GET_TIME_MS() - timestamp > 1000) {
			  PRINT_ERR("E22-AUX timeout.");
			  break;
		  }
		  FeedGlobalWatchdog();
	  }
	  E22WorkMode = mode;
	  if(mode == 0) {
		  HAL_GPIO_WritePin(E22_M1_GPIO_PORT, E22_M1_PIN, GPIO_PIN_RESET);	// M1=0
		  // GPIO_ResetBits(E22_M0_GPIO_PORT,E22_M0_PIN);    //M0=0
		  USART_SetBaudRate(USART1, 115200);
	  } else if(mode == 1) {
		  HAL_GPIO_WritePin(E22_M1_GPIO_PORT, E22_M1_PIN, GPIO_PIN_RESET);	// M1=0
		  // GPIO_SetBits(E22_M0_GPIO_PORT,E22_M0_PIN);      //M0=1
		  USART_SetBaudRate(USART1, 115200);
	  } else if(mode == 2) {
		  HAL_GPIO_WritePin(E22_M1_GPIO_PORT, E22_M1_PIN, GPIO_PIN_SET);  // M1=1
		  // GPIO_ResetBits(E22_M0_GPIO_PORT,E22_M0_PIN);    //M0=0
		  USART_SetBaudRate(USART1, 9600);
	  } else if(mode == 3) {
		  HAL_GPIO_WritePin(E22_M1_GPIO_PORT, E22_M1_PIN, GPIO_PIN_SET);  // M1=1
		  // GPIO_SetBits(E22_M0_GPIO_PORT,E22_M0_PIN);    //M0=1
		  USART_SetBaudRate(USART1, 115200);
	  } else {
		  //		Error_Handler(__FILE__, __LINE__);
		  //		printf("选择E22模式错误");
		  PRINT_ERR("Invalid mode. [0~3]");
	  }
	  WAIT_MS(40);
	  timestamp = GET_TIME_MS();
	  while(HAL_GPIO_ReadPin(E22_AUX_GPIO_PORT, E22_AUX_PIN) == GPIO_PIN_RESET) {
		  if(GET_TIME_MS() - timestamp > 500) {
			  PRINT_ERR("E22-AUX timeout.");
			  break;
		  }
		  FeedGlobalWatchdog();
		  WAIT_MS(1);
	  }
}

uint8_t E22GetWorkMode(void) {
	return E22WorkMode;
}

/*
*********************************************************************************************************
*	函 数 名: E22ParaConfigure
*	功能说明: 配置自身E22模块参数，自身模块一定要工作在模式2（配置模式）
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void E22ParaConfigure(void) {
	/*配置模式下的串口波特率必须是9600*/
	comSetBaud(COM1, 9600);
	/*首先设置模块在工作模式2（配置模式）*/
	E22SetWorkMode(2);
	WAIT_MS(300);
	/*设置自身模块参数*/
	comSendChar(COM1, 0xC0);				//发送设置寄存器指令
	comSendChar(COM1, 0x00);				//发送起始地址
	comSendChar(COM1, 0x09);				//发送长度
	comSendChar(COM1, g_tE22Self.ADDH);		//发送地址高字节
	comSendChar(COM1, g_tE22Self.ADDL);		//发送地址低字节
	comSendChar(COM1, g_tE22Self.NETID);	//发送网络ID
	comSendChar(COM1, g_tE22Self.REG0);		//发送寄存器0
	comSendChar(COM1, g_tE22Self.REG1);		//发送寄存器1
	comSendChar(COM1, g_tE22Self.REG2);		//发送寄存器2
	comSendChar(COM1, g_tE22Self.REG3);		//发送寄存器3
	comSendChar(COM1, g_tE22Self.CRYPT_H);	//发送密钥高字节
	comSendChar(COM1, g_tE22Self.CRYPT_L);	//发送密钥低字节

	//待写接受确认函数
	//待写远程配置函数   单独一个函数，不是每次上电都需要配置

	/*恢复串口波特率为115200*/
	WAIT_MS(500);
	comSetBaud(COM1, 115200);
	/*恢复E22工作模式为一般模式透明传输*/
	E22SetWorkMode(0);
	WAIT_MS(20);
}

/*
*********************************************************************************************************
*	函 数 名: E22ParaConfigure
*	功能说明:
*配置远程E22模块参数，自身模块一定要工作在模式2（配置模式），远程模块需要工作在透明传输模式 形 参:
*无 返 回 值: 无
*********************************************************************************************************
*/

void E22ParaConfigureOther(void) {
	/*配置模式下的串口波特率必须是9600*/
	comSetBaud(COM1, 9600);
	/*首先设置模块在工作模式2（配置模式）*/
	E22SetWorkMode(2);
	WAIT_MS(20);
	/*设置自身模块参数*/
	comSendChar(COM1, 0xC0);				//发送设置寄存器指令
	comSendChar(COM1, 0x00);				//发送起始地址
	comSendChar(COM1, 0x09);				//发送长度
	comSendChar(COM1, g_tE22Self.ADDH);		//发送地址高字节
	comSendChar(COM1, g_tE22Self.ADDL);		//发送地址低字节
	comSendChar(COM1, g_tE22Self.NETID);	//发送网络ID
	comSendChar(COM1, g_tE22Self.REG0);		//发送寄存器0
	comSendChar(COM1, g_tE22Self.REG1);		//发送寄存器1
	comSendChar(COM1, g_tE22Self.REG2);		//发送寄存器2
	comSendChar(COM1, g_tE22Self.REG3);		//发送寄存器3
	comSendChar(COM1, g_tE22Self.CRYPT_H);	//发送密钥高字节
	comSendChar(COM1, g_tE22Self.CRYPT_L);	//发送密钥低字节

	//待写接受确认函数
	//待写远程配置函数

	/*恢复串口波特率为115200*/
	WAIT_MS(300);
	comSetBaud(COM1, 115200);
	/*恢复E22工作模式为一般模式透明传输*/
	E22SetWorkMode(0);
	WAIT_MS(20);
}

/*
*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitE22AndUart1(void) {
	bsp_InitE22();
	bsp_InitUart();
}

/*
*********************************************************************************************************
*	函 数 名: E22ParaConfigure
*	功能说明:
设置自身E22模块为公共值，进入对码模式后，首先配置自身E22寄存器为默认值，详情看表。（降落伞需要已设置为默认参数）然后远程配置
对方为各个遥控器自身唯一的密码，最后再恢复自生配置
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/

void E22SetPublicConfigure(void) {
	uint8_t publicConfigurePar[] = { 0xC0, 0x00, 0x09, 0xFF, 0XFF, 0X00,
									 0XE4, 0XC0, 0X17, 0X00, 0X00, 0X01 };

	E22_Config_Write(publicConfigurePar, sizeof(publicConfigurePar));

	/*恢复串口波特率为115200*/
	WAIT_MS(300);
	comSetBaud(COM1, 115200);
	/*恢复E22工作模式为一般模式透明传输*/
	E22SetWorkMode(0);
	WAIT_MS(20);
}

void E22_Config_Write(uint8_t *pData, size_t n) {
	/*首先设置模块在工作模式2（配置模式）*/
	E22SetWorkMode(2);
	WAIT_MS(100);
	uint8_t s_sendCount = 0;
	DISABLE_INT();

	_RC_bsp_SendNByte(pData, n);
	// E22SetWorkMode(0);
	ENABLE_INT();
}

void _RC_bsp_SendByte(uint8_t byte) {
	rc_usart_handle.Instance->DR = byte;
	while(!__HAL_USART_GET_FLAG(&rc_usart_handle, USART_FLAG_TC)) {
	}
	__HAL_USART_CLEAR_FLAG(&rc_usart_handle, USART_FLAG_TC);
}

void _RC_bsp_SendNByte(uint8_t *pData, int n) {
	int cnt = 0;
	while(cnt < n) {
		_RC_bsp_SendByte(pData[cnt]);
		cnt++;
	}
}

void _RC_bsp_ConfigUpgradeMode(void) {
	// Baud 9600
	uint8_t config_03[] = { 0xC0, 0x03, 0x01, 0x67 };
	// Baud 57600
	// uint8_t config_03[] ={0xC0,0x03,0x01,0xC7};
	// Baud 115200
	// uint8_t config_03[] ={0xC0,0x03,0x01,0xE7};
	uint8_t config_06[] = { 0xC0, 0x06, 0x01, 0x00 };

	E22_Config_Write(config_03, sizeof(config_03));
	WAIT_MS(500);
	E22_Config_Write(config_06, sizeof(config_06));
	WAIT_MS(500);

	/*恢复串口波特率为115200*/
	// WAIT_MS(300);
	comSetBaud(COM1, 9600);
	/*恢复E22工作模式为一般模式透明传输*/
	E22SetWorkMode(0);
	WAIT_MS(200);
}

void _RC_bsp_RecoverFromUpgradeMode(void) {
	uint8_t config_03[] = { 0xC0, 0x03, 0x01, 0xE3 };
	uint8_t config_06[] = { 0xC0, 0x06, 0x01, 0x80 };

	E22_Config_Write(config_03, sizeof(config_03));
	WAIT_MS(100);
	E22_Config_Write(config_06, sizeof(config_06));
	WAIT_MS(100);

	/*恢复串口波特率为115200*/
	// WAIT_MS(300);
	comSetBaud(COM1, 115200);
	/*恢复E22工作模式为一般模式透明传输*/
	E22SetWorkMode(0);
	WAIT_MS(200);
}

static uint32_t RxTimestamp;
static uint32_t dataCnt = 0;
void			USART1_IRQHandler(void) {
	   FeedGlobalWatchdog();
	   if(__HAL_USART_GET_FLAG(&rc_usart_handle, USART_FLAG_RXNE)) {
		   uint16_t data = rc_usart_handle.Instance->DR;
		   RxTimestamp	 = GET_TIME_MS();
		   dataCnt++;
	   }
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart) {
	RxTimestamp = GET_TIME_MS();
	dataCnt++;
}

uint32_t bsp_E22_GetLastRxTimestamp(void) {
	return RxTimestamp;
}

uint32_t bsp_E22_GetIncomingDataCnt(void) {
	return dataCnt;
}

void bsp_E22_ClearIncomingData(void) {
	dataCnt = 0;
}

#ifdef __cplusplus
}
#endif
