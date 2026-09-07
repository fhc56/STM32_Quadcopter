/**
 * @file Remote.c
 * @brief 旧版本地遥控输入接口。
 * @note  当前飞机使用RemoteRF无线协议，本模块保留兼容性但不参与飞控主路径。
 */
#include "stm32f10x.h"

#include "Remote.h"
#include "Serial.h"

#include <stdio.h>
#include <string.h>

/*
 * 手机基础总油门
 */
static uint8_t Remote_Throttle = 0;

void Remote_Init(void)
{
	Remote_Throttle = 0;

	/*
     * 清除可能残留的串口数据
 */
	Serial_RxFlag = 0;
}

void Remote_Update(void)
{
	char Name[20];
	int Value;

	if (Serial_RxFlag == 1)
	{
		/*
         * 手机格式：
         * [slider,Throttle,50]
         * Serial_RxPacket：
         * slider,Throttle,50
 */
		if (sscanf(Serial_RxPacket, "slider,%19[^,],%d", Name, &Value) == 2)
		{
			/*
             * 只处理总油门Throttle。
             * 下面Motor1~Motor4滑杆全部忽略。
 */
			if (strcmp(Name, "Throttle") == 0)
			{
				if (Value < 0)
				{
					Value = 0;
				}

				/*
                 * ★基础Throttle最大仍然80★
                 * 不改成90。
                 * 90只是单电机经过PID以后
                 * 最终允许达到的上限。
 */
				if (Value > 80)
				{
					Value = 80;
				}
				Remote_Throttle = (uint8_t)Value;
			}
		}
		Serial_RxFlag = 0;
	}
}

uint8_t Remote_GetThrottle(void)
{
	return Remote_Throttle;
}
