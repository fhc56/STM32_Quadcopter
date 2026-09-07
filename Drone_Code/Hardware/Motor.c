/**
 * @file Motor.c
 * @brief 四路电机速度接口和统一急停入口。
 * @note  底层使用PWM模块的TIM2四通道；除故障处理外，仅飞控任务可写电机。
 */
#include "stm32f10x.h"

#include "PWM.h"
#include "Motor.h"

/*
 * ★单个电机硬件最终最大允许90%
 * 基础Throttle仍然最大80。
 * 多出来的10主要给：
 * Roll PID
 * Pitch PID
 * Yaw PID
 * Yaw Trim
 * 留控制余量。
 */
#define MOTOR_MAX_SPEED 90

void Motor_Init(void)
{
	PWM_Init();
	PWM_SetCompare1(0);
	PWM_SetCompare2(0);
	PWM_SetCompare3(0);
	PWM_SetCompare4(0);
}

void Motor_SetSpeed1(uint8_t Speed)
{
	if (Speed > MOTOR_MAX_SPEED)
	{
		Speed = MOTOR_MAX_SPEED;
	}
	PWM_SetCompare1(Speed);
}

void Motor_SetSpeed2(uint8_t Speed)
{
	if (Speed > MOTOR_MAX_SPEED)
	{
		Speed = MOTOR_MAX_SPEED;
	}
	PWM_SetCompare2(Speed);
}

void Motor_SetSpeed3(uint8_t Speed)
{
	if (Speed > MOTOR_MAX_SPEED)
	{
		Speed = MOTOR_MAX_SPEED;
	}
	PWM_SetCompare3(Speed);
}

void Motor_SetSpeed4(uint8_t Speed)
{
	if (Speed > MOTOR_MAX_SPEED)
	{
		Speed = MOTOR_MAX_SPEED;
	}
	PWM_SetCompare4(Speed);
}

void Motor_StopAll(void)
{
	PWM_SetCompare1(0);
	PWM_SetCompare2(0);
	PWM_SetCompare3(0);
	PWM_SetCompare4(0);
}
