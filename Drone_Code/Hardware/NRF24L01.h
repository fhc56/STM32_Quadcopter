#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"

#define NRF24L01_PACKET_SIZE 32

/*
 * 接收缓冲区
 */
extern uint8_t NRF24L01_RxPacket[NRF24L01_PACKET_SIZE];

/*
 * 初始化NRF24L01
 */
void NRF24L01_Init(void);

/*
 * 尝试接收一包数据
 * 返回：
 * 0 = 没有新数据
 * 1 = 成功收到32字节
 * 2 = NRF状态异常
 */
uint8_t NRF24L01_Receive(void);

/*
 * 调试：读取STATUS
 */
uint8_t NRF24L01_GetStatus(void);

#endif
