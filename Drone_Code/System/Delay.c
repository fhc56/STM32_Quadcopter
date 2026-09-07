#include "stm32f10x.h"

#include "Delay.h"

/* Cortex-M3 DWT registers used as a free-running 32-bit cycle counter. */
#define DELAY_DEMCR_REG		 (*((volatile uint32_t *)0xE000EDFCUL))
#define DELAY_DWT_CTRL_REG	 (*((volatile uint32_t *)0xE0001000UL))
#define DELAY_DWT_CYCCNT_REG (*((volatile uint32_t *)0xE0001004UL))
#define DELAY_DEMCR_TRCENA	 (1UL << 24)
#define DELAY_DWT_CYCCNTENA	 (1UL << 0)

/**
 * @brief  初始化DWT周期计数器。
 * @note   必须在第一次调用Delay_us/Delay_ms之前执行一次。
 *         DWT与FreeRTOS SysTick相互独立，因此不会破坏系统节拍。
 */
void Delay_Init(void)
{
	SystemCoreClockUpdate();
	DELAY_DEMCR_REG |= DELAY_DEMCR_TRCENA;
	DELAY_DWT_CYCCNT_REG = 0UL;
	DELAY_DWT_CTRL_REG |= DELAY_DWT_CYCCNTENA;
}

/**
 * @brief  微秒级忙等待延时。
 * @param  xus 延时时长，建议范围0~233015 us。
 * @note   仅用于上电初始化和必须精确到微秒的短延时，不用于任务调度。
 */
void Delay_us(uint32_t xus)
{
	uint32_t StartCycle;
	uint32_t WaitCycles;

	if (xus == 0UL)
	{
		return;
	}
	StartCycle = DELAY_DWT_CYCCNT_REG;
	WaitCycles = (SystemCoreClock / 1000000UL) * xus;
	while ((uint32_t)(DELAY_DWT_CYCCNT_REG - StartCycle) < WaitCycles)
	{
	}
}

/**
 * @brief  毫秒级忙等待延时。
 * @note   当前只用于调度器启动前的传感器初始化和校准。任务周期请使用
 *         vTaskDelayUntil()，避免占用CPU。
 */
void Delay_ms(uint32_t xms)
{
	while (xms > 0UL)
	{
		Delay_us(1000UL);
		xms--;
	}
}

/**
 * @brief  秒级忙等待延时。
 */
void Delay_s(uint32_t xs)
{
	while (xs > 0UL)
	{
		Delay_ms(1000UL);
		xs--;
	}
}
