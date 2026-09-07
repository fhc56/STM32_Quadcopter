#ifndef __PID_H
#define __PID_H

#include "stm32f10x.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Integral;
	float PrevError;
	float OutputLimit;
	float IntegralLimit;
	uint8_t FirstRun;

} PID_t;

void PID_Init(PID_t *PID, float Kp, float Ki, float Kd, float OutputLimit, float IntegralLimit);
void PID_Reset(PID_t *PID);
float PID_Update(PID_t *PID, float Target, float Feedback, float dt);

#endif /* PID_H */
