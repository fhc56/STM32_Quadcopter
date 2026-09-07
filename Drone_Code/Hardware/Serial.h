#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

extern char Serial_RxPacket[100];
extern volatile uint8_t Serial_RxFlag;
void Serial_Init(void);

/*
 * 以下发送函数全部改成：
 * 非阻塞发送
 * 数据进入发送缓冲区后立即返回，
 * USART1中断负责慢慢发送。
 */
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(const char *Format, ...);

#endif /* SERIAL_H */
