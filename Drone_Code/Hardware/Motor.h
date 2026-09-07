#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);
void Motor_SetSpeed1(uint8_t Speed);
void Motor_SetSpeed2(uint8_t Speed);
void Motor_SetSpeed3(uint8_t Speed);
void Motor_SetSpeed4(uint8_t Speed);
void Motor_StopAll(void);

#endif
