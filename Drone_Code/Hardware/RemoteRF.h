#ifndef __REMOTE_RF_H
#define __REMOTE_RF_H

#include "stm32f10x.h"

void RemoteRF_Init(void);
void RemoteRF_Update(void);
uint8_t RemoteRF_GetThrottle(void);
int8_t RemoteRF_GetVerticalCommand(void);
float RemoteRF_GetTargetRoll(void);
float RemoteRF_GetTargetPitch(void);
int8_t RemoteRF_GetFB(void);
int8_t RemoteRF_GetLR(void);
uint8_t RemoteRF_IsConnected(void);

/*
 * ★K2急停状态★
 * 1：
 * 必须立即停止所有电机
 * 0：
 * 正常
 */
uint8_t RemoteRF_GetEmergencyStop(void);

#endif
