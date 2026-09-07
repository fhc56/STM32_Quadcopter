#ifndef __ALTITUDE_CONTROL_H
#define __ALTITUDE_CONTROL_H

#include "stm32f10x.h"

/*
 * Altitude control built on top of Barometer.c.
 * VerticalCommand:
 *   -100 .. +100
 *   + : climb
 *    0: hold target height
 *   - : descend
 *  <=-90 near ground: controlled landing and motor cut
 */
void AltitudeControl_Init(void);
void AltitudeControl_Update(int8_t VerticalCommand, float dt, uint8_t RFConnected);
uint8_t AltitudeControl_GetThrottle(void);
float AltitudeControl_GetHeight(void);
float AltitudeControl_GetTargetHeight(void);
float AltitudeControl_GetVerticalSpeed(void);
float AltitudeControl_GetThrottleFloat(void);
uint8_t AltitudeControl_IsActive(void);
uint8_t AltitudeControl_IsLanding(void);
uint8_t AltitudeControl_IsSensorOK(void);

#endif
