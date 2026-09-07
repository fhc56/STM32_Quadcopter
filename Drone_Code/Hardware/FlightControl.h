#ifndef __FLIGHT_CONTROL_H
#define __FLIGHT_CONTROL_H

#include "stm32f10x.h"

#include "Attitude.h"

void FlightControl_Init(void);
void FlightControl_Update(uint8_t Throttle, const Attitude_t *Attitude);
uint8_t FlightControl_IsFaultLatched(void);
float FlightControl_GetYawOutput(void);
uint8_t FlightControl_GetM1(void);
uint8_t FlightControl_GetM2(void);
uint8_t FlightControl_GetM3(void);
uint8_t FlightControl_GetM4(void);
float FlightControl_GetTargetRoll(void);
float FlightControl_GetTargetPitch(void);
float FlightControl_GetPitchRateTarget(void);
float FlightControl_GetPitchOutput(void);
float FlightControl_GetRollRateTarget(void);
float FlightControl_GetRollOutput(void);

#endif
