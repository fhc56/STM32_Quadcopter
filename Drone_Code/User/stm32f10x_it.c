#include "stm32f10x_it.h"

/*
 * 飞控异常时的紧急处理
 * 不调用Motor_StopAll()
 * 因为如果已经发生Fault，
 * 尽量不要再进入一层层C函数。
 * 直接操作TIM2寄存器最简单。
 */
static void EmergencyStopAndReset(void)
{
	/*
     * 四路PWM立刻归0
 */
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	/*
     * 立即复位STM32
 */
	NVIC_SystemReset();

	/*
     * 理论上不会运行到这里。
 */
	while (1)
	{
	}
}

/*
 * NMI
 */
void NMI_Handler(void)
{
}

/*
 * HardFault
 */
void HardFault_Handler(void)
{
	EmergencyStopAndReset();
}

/*
 * Memory Fault
 */
void MemManage_Handler(void)
{
	EmergencyStopAndReset();
}

/*
 * Bus Fault
 */
void BusFault_Handler(void)
{
	EmergencyStopAndReset();
}

/*
 * Usage Fault
 */
void UsageFault_Handler(void)
{
	EmergencyStopAndReset();
}

/*
 * DebugMon
 *==========================================================*/
void DebugMon_Handler(void)
{
}

/*
 * SVC_Handler、PendSV_Handler和SysTick_Handler由
 * FreeRTOS/port/port.c提供。这里不能再次定义，否则链接时会重名。
 */
