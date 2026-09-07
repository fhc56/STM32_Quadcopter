/**
 * @file PID.c
 * @brief 通用PID初始化、状态复位和单步计算。
 * @note  PID实例由上层控制模块持有，本文件不创建共享全局控制器。
 */
#include "stm32f10x.h"

#include "PID.h"

static float PID_Limit(float Value, float Limit)
{
	if (Value > Limit)
	{
		Value = Limit;
	}
	if (Value < -Limit)
	{
		Value = -Limit;
	}
	return Value;
}

void PID_Init(PID_t *PID, float Kp, float Ki, float Kd, float OutputLimit, float IntegralLimit)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->Integral = 0.0f;
	PID->PrevError = 0.0f;
	PID->OutputLimit = OutputLimit;
	PID->IntegralLimit = IntegralLimit;
	PID->FirstRun = 1;
}

void PID_Reset(PID_t *PID)
{
	PID->Integral = 0.0f;
	PID->PrevError = 0.0f;
	PID->FirstRun = 1;
}

float PID_Update(PID_t *PID, float Target, float Feedback, float dt)
{
	float Error;
	float P;
	float I;
	float D;
	float Output;

	/*
     * 防止异常dt
 */
	if (dt < 0.0005f || dt > 0.05f)
	{
		dt = 0.005f;
	}

	/*
     * Error = 目标 - 实际
 */
	Error = Target - Feedback;

	/*
     * P
 */
	P = PID->Kp * Error;

	/*
     * I
 */
	PID->Integral += PID->Ki * Error * dt;
	PID->Integral = PID_Limit(PID->Integral, PID->IntegralLimit);
	I = PID->Integral;

	/*
     * D
 */
	if (PID->FirstRun)
	{
		D = 0.0f;
		PID->FirstRun = 0;
	}
	else
	{
		D = PID->Kd * (Error - PID->PrevError) / dt;
	}
	PID->PrevError = Error;

	/*
     * PID输出
 */
	Output = P + I + D;
	Output = PID_Limit(Output, PID->OutputLimit);
	return Output;
}
