#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "stm32f10x.h"

typedef struct
{
	/*
     * 飞机机体姿态角
     * 单位：
     * °
 */
	float Roll;
	float Pitch;
	float Yaw;

	/*
     * 飞机机体角速度
     * 单位：
     * °/s
     * 已经：
     * 1. 减去上电静态Bias
     * 2. Roll/Pitch加入Mahony慢速Bias修正
 */
	float GyroX;
	float GyroY;
	float GyroZ;

	/*
     * 本次更新周期
     * 单位：
     * s
 */
	float dt;

} Attitude_t;

void Attitude_Init(void);
void Attitude_Update(Attitude_t *Attitude);
float Attitude_GetGyroBiasX(void);
float Attitude_GetGyroBiasY(void);
float Attitude_GetGyroBiasZ(void);

#endif
