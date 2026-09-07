/**
 * @file PWM.c
 * @brief TIM2四通道电机PWM初始化和比较值写入。
 * @note  四个CCR寄存器对应四路电机，紧急故障路径可直接将它们清零。
 */
#include "stm32f10x.h"
#include "PWM.h"

void PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/*
     * 1. 开启时钟
 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/*
     * 2. GPIO
     * PA0 -> TIM2_CH1 -> Motor 1
     * PA1 -> TIM2_CH2 -> Motor 2
     * PA2 -> TIM2_CH3 -> Motor 3
     * PA3 -> TIM2_CH4 -> Motor 4
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
     * 3. TIM2使用内部时钟
 */
	TIM_InternalClockConfig(TIM2);

	/*
     * STM32F103：
     * SYSCLK = 72MHz
     * TIM2 = 72MHz
     * 72MHz / 36 = 2MHz
     * ARR = 100
     * 2MHz / 100 = 20kHz
     * 所以PWM约20kHz
 */
	/*
     * 4. TIM2时基
 */
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	/*
     * 5. PWM公共配置
 */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	/*
     * 上电PWM = 0
     * 防止电机初始化瞬间启动
 */
	TIM_OCInitStructure.TIM_Pulse = 0;

	/*
     * 6. Motor 1
     * PA0
     * TIM2_CH1
 */
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/*
     * 7. Motor 2
     * PA1
     * TIM2_CH2
 */
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/*
     * 8. Motor 3
     * PA2
     * TIM2_CH3
 */
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/*
     * 9. Motor 4
     * PA3
     * TIM2_CH4
 */
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/*
     * 10. ARR预装载
 */
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	/*
     * 11. 启动TIM2
 */
	TIM_Cmd(TIM2, ENABLE);
}

/**
  * Motor 1
  *
  * PA0 -> TIM2_CH1
 */
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM2, Compare);
}

/**
  * Motor 2
  *
  * PA1 -> TIM2_CH2
 */
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM2, Compare);
}

/**
  * Motor 3
  *
  * PA2 -> TIM2_CH3
 */
void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare3(TIM2, Compare);
}

/**
  * Motor 4
  *
  * PA3 -> TIM2_CH4
 */
void PWM_SetCompare4(uint16_t Compare)
{
	TIM_SetCompare4(TIM2, Compare);
}
