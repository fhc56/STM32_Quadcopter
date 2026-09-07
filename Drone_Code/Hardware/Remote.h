#ifndef __REMOTE_H
#define __REMOTE_H

#include "stm32f10x.h"

void Remote_Init(void);
void Remote_Update(void);
uint8_t Remote_GetThrottle(void);

#endif
