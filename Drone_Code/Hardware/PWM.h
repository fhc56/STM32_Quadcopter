#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

void PWM_Init(void);

/* Motor 1£ºPA0 */
void PWM_SetCompare1(uint16_t Compare);

/* Motor 2£ºPA1 */
void PWM_SetCompare2(uint16_t Compare);

/* Motor 3£ºPA2 */
void PWM_SetCompare3(uint16_t Compare);

/* Motor 4£ºPA3 */
void PWM_SetCompare4(uint16_t Compare);

#endif
