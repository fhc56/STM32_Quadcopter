/**
 * @file Serial.c
 * @brief USART1异步收发、TX环形缓冲区和轻量整数日志格式化。
 * @note  TX由中断后台发送，缓冲区满时丢弃调试字节，绝不等待串口而阻塞飞控。
 */
#include "stm32f10x.h"

#include "Serial.h"

#include <stdarg.h>

/*
 * 蓝牙接收
 */
char Serial_RxPacket[100];
volatile uint8_t Serial_RxFlag = 0;

/*
 * ★USART1非阻塞发送环形缓冲区★
 * HC-08只有9600波特率。
 * 飞控绝对不能：
 * while(TXE == 0)
 * 一直在那里等蓝牙。
 * 所以现在：
 * main
 * ↓
 * 只负责把数据塞进Buffer
 * ↓
 * USART1_IRQHandler
 * ↓
 * 后台慢慢发送
 * 256字节对于当前调试完全够。
 */
#define SERIAL_TX_BUFFER_SIZE 256

static volatile uint8_t Serial_TxBuffer[SERIAL_TX_BUFFER_SIZE];
static volatile uint16_t Serial_TxHead = 0;
static volatile uint16_t Serial_TxTail = 0;

/*
 * 初始化
 */
void Serial_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*
     * 时钟
 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/*
     * PA9
     * USART1 TX
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
     * PA10
     * USART1 RX
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
     * USART1
     * HC-08 = 9600
 */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

	/*
     * RX接收中断
 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/*
     * TXE发送中断：
     * 这里暂时不开。
     * 有数据要发送的时候，
     * Serial_SendByte()会自动开启。
 */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	/*
     * NVIC
     * 优先级分组由main统一设置为Group 4。USART1使用优先级12，
     * 以后如果改用FreeRTOS的...FromISR接口也处于合法范围。
     *======================================================*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	/*
     * 清发送Buffer
 */
	Serial_TxHead = 0;
	Serial_TxTail = 0;
	USART_Cmd(USART1, ENABLE);
}

/*
 * ★非阻塞发送一个字节★
 * 这里只把数据放入Buffer。
 * 不等待TXE。
 * 所以：
 * 蓝牙再慢
 * ↓
 * 飞控主循环也不会卡住。
 */
void Serial_SendByte(uint8_t Byte)
{
	uint16_t NextHead;

	NextHead = Serial_TxHead + 1;
	if (NextHead >= SERIAL_TX_BUFFER_SIZE)
	{
		NextHead = 0;
	}

	/*
     * Buffer满了
     * ★直接丢掉这个调试字节★
     * 绝对不允许因为调试信息影响飞控。
 */
	if (NextHead == Serial_TxTail)
	{
		return;
	}
	Serial_TxBuffer[Serial_TxHead] = Byte;
	Serial_TxHead = NextHead;

	/*
     * 开启TXE中断
     * USART自己后台发送。
 */
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

/*
 * 数组
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;

	for (i = 0; i < Length; i++)
	{
		Serial_SendByte(Array[i]);
	}
}

/*
 * 字符串
 */
void Serial_SendString(char *String)
{
	uint16_t i;

	for (i = 0; String[i] != '\0'; i++)
	{
		Serial_SendByte(String[i]);
	}
}

/*
 * Pow
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;

	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/*
 * Number
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;

	for (i = 0; i < Length; i++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*
 * 轻量格式化输出
 * 飞控只使用整数日志。完整vsnprintf会引入浮点、宽字符和本地化格式化
 * 代码，占用数KB Flash。下面只实现本工程实际需要的格式：
 * %d / %i   有符号十进制
 * %u / %lu  无符号十进制
 * %c / %s   字符和字符串
 * %%         百分号
 *==========================================================*/
static void Serial_SendUnsignedDecimal(unsigned long Value)
{
	char Digits[10];
	uint8_t Count;

	Count = 0U;
	do
	{
		Digits[Count] = (char)('0' + (Value % 10UL));
		Count++;
		Value /= 10UL;
	} while ((Value > 0UL) && (Count < sizeof(Digits)));
	while (Count > 0U)
	{
		Count--;
		Serial_SendByte((uint8_t)Digits[Count]);
	}
}

static void Serial_SendSignedDecimal(long Value)
{
	unsigned long Magnitude;

	if (Value < 0L)
	{
		Serial_SendByte((uint8_t)'-');

		/* 该写法也能安全处理LONG_MIN。 */
		Magnitude = (unsigned long)(-(Value + 1L));
		Magnitude++;
	}
	else
	{
		Magnitude = (unsigned long)Value;
	}
	Serial_SendUnsignedDecimal(Magnitude);
}

void Serial_Printf(const char *Format, ...)
{
	va_list Arguments;
	uint8_t LongArgument;
	long SignedValue;
	unsigned long UnsignedValue;

	va_start(Arguments, Format);
	while (*Format != '\0')
	{
		if (*Format != '%')
		{
			Serial_SendByte((uint8_t)*Format);
			Format++;
			continue;
		}
		Format++;
		LongArgument = 0U;
		if (*Format == 'l')
		{
			LongArgument = 1U;
			Format++;
		}
		if (*Format == '\0')
		{
			Serial_SendByte((uint8_t)'%');
			break;
		}
		switch (*Format)
		{
		case 'd':
		case 'i':
			if (LongArgument != 0U)
			{
				SignedValue = va_arg(Arguments, long);
			}
			else
			{
				SignedValue = (long)va_arg(Arguments, int);
			}
			Serial_SendSignedDecimal(SignedValue);
			break;
		case 'u':
			if (LongArgument != 0U)
			{
				UnsignedValue = va_arg(Arguments, unsigned long);
			}
			else
			{
				UnsignedValue = (unsigned long)va_arg(Arguments, unsigned int);
			}
			Serial_SendUnsignedDecimal(UnsignedValue);
			break;
		case 'c':
			Serial_SendByte((uint8_t)va_arg(Arguments, int));
			break;
		case 's':
			Serial_SendString(va_arg(Arguments, char *));
			break;
		case '%':
			Serial_SendByte((uint8_t)'%');
			break;
		default:
			/* 未支持的格式按原字符输出，避免静默丢失日志内容。 */
			Serial_SendByte((uint8_t)'%');
			if (LongArgument != 0U)
			{
				Serial_SendByte((uint8_t)'l');
			}
			Serial_SendByte((uint8_t)*Format);
			break;
		}
		Format++;
	}
	va_end(Arguments);
}

/*
 * USART1中断
 * 同时负责：
 * RX
 * TX
 */
void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	uint8_t RxData;

	/*
     * 1. RX
 */
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		RxData = (uint8_t)USART_ReceiveData(USART1);

		/*
         * 等待 [
 */
		if (RxState == 0)
		{
			if (RxData == '[' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}

		/*
         * 正在接收
 */
		else
		{
			/*
             * ]
             * 一帧结束
 */
			if (RxData == ']')
			{
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Serial_RxFlag = 1;
			}

			/*
             * 普通数据
 */
			else
			{
				if (pRxPacket < sizeof(Serial_RxPacket) - 1)
				{
					Serial_RxPacket[pRxPacket] = RxData;
					pRxPacket++;
				}
				else
				{
					/*
                     * 超长数据直接丢弃。
 */
					RxState = 0;
					pRxPacket = 0;
				}
			}
		}
	}

	/*
     * 2. ★TX后台发送★
 */
	if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
	{
		/*
         * Buffer还有东西
 */
		if (Serial_TxTail != Serial_TxHead)
		{
			USART_SendData(USART1, Serial_TxBuffer[Serial_TxTail]);
			Serial_TxTail++;
			if (Serial_TxTail >= SERIAL_TX_BUFFER_SIZE)
			{
				Serial_TxTail = 0;
			}
		}

		/*
         * 已经发完
         * 关闭TXE中断。
 */
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
}
