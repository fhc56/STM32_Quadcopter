#ifndef __BAROMETER_H
#define __BAROMETER_H

#include "stm32f10x.h"

uint8_t Barometer_Init(void);
void Barometer_Update(float dt);

/*
 * 地面零点自动跟踪
 * Enable = 1：
 * 飞机在地面，允许慢慢修正零点
 * Enable = 0：
 * 飞机已经工作，冻结零点
 */
void Barometer_GroundZeroUpdate(uint8_t Enable);
uint32_t Barometer_GetPressure(void);
float Barometer_GetHeight(void);

/* Continuous filtered height for control loops (m). */
float Barometer_GetControlHeight(void);
uint8_t Barometer_IsOK(void);

#endif
