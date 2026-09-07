/**
 * @file RemoteRF.c
 * @brief 无线遥控数据包校验、ARM/急停状态和失联保护。
 * @note  失联阈值按100 Hz调用频率计数，必须由遥控任务每10 ms调用一次。
 */
#include "stm32f10x.h"

#include "RemoteRF.h"
#include "NRF24L01.h"

#define REMOTE_PACKET_HEADER	   0xA5
#define REMOTE_PROTOCOL_VERSION	   0xC3
#define REMOTE_CHECK_BASE		   0x5A
#define REMOTE_FLIGHT_THROTTLE_MAX 70
#define REMOTE_MAX_ROLL_DEG		   5.0f
#define REMOTE_MAX_PITCH_DEG	   5.0f

/*
 * 遥控任务约100 Hz
 * 20次约200 ms
 */
#define REMOTE_LOST_COUNT_MAX 20

/*
 * Byte6
 */
#define REMOTE_SAFE_NONE  0
#define REMOTE_SAFE_ESTOP 1
#define REMOTE_SAFE_ARM	  2

/*
 * 控制量
 */
static uint8_t RemoteThrottle = 0;
static int8_t RemoteVerticalCommand = 0;
static int8_t RemoteFB = 0;
static int8_t RemoteLR = 0;
static float RemoteTargetRoll = 0.0f;
static float RemoteTargetPitch = 0.0f;
static uint8_t RemoteConnected = 0;
static uint16_t RemoteLostCount = 0;

/*
 * ★飞机上电默认锁死★
 * 必须收到K1的ARM命令，
 * 才允许解除。
 */
static uint8_t RemoteEmergencyStop = 1;

/*
 * 清空控制
 */
static void RemoteRF_ClearControl(void)
{
	RemoteThrottle = 0;
	RemoteVerticalCommand = 0;
	RemoteFB = 0;
	RemoteLR = 0;
	RemoteTargetRoll = 0.0f;
	RemoteTargetPitch = 0.0f;
}

/*
 * 校验数据包
 */
static uint8_t RemoteRF_CheckPacket(uint8_t *Packet)
{
	uint8_t Check;

	if (Packet[0] != REMOTE_PACKET_HEADER)
	{
		return 0;
	}

	/*
     * 防止旧版遥控程序误控制新版飞机
 */
	if (Packet[7] != REMOTE_PROTOCOL_VERSION)
	{
		return 0;
	}
	Check = REMOTE_CHECK_BASE;
	Check ^= Packet[0];
	Check ^= Packet[1];
	Check ^= Packet[2];
	Check ^= Packet[3];
	Check ^= Packet[4];
	Check ^= Packet[6];
	Check ^= Packet[7];
	if (Check != Packet[5])
	{
		return 0;
	}
	return 1;
}

/*
 * 初始化
 */
void RemoteRF_Init(void)
{
	RemoteRF_ClearControl();
	RemoteConnected = 0;
	RemoteLostCount = 0;
	RemoteEmergencyStop = 1;
	NRF24L01_Init();
}

/*
 * 更新
 */
void RemoteRF_Update(void)
{
	uint8_t ReceiveFlag;
	uint8_t SafetyCommand;
	int8_t VerticalCommand;
	int8_t FB;
	int8_t LR;

	ReceiveFlag = NRF24L01_Receive();
	if (ReceiveFlag == 1)
	{
		if (RemoteRF_CheckPacket(NRF24L01_RxPacket))
		{
			RemoteLostCount = 0;
			RemoteConnected = 1;
			VerticalCommand = (int8_t)NRF24L01_RxPacket[1];
			FB = (int8_t)NRF24L01_RxPacket[2];
			LR = (int8_t)NRF24L01_RxPacket[3];
			SafetyCommand = NRF24L01_RxPacket[6];

			/*
             * 限幅
 */
			if (VerticalCommand > 100)
			{
				VerticalCommand = 100;
			}
			if (VerticalCommand < -100)
			{
				VerticalCommand = -100;
			}
			if (FB > 100)
			{
				FB = 100;
			}
			if (FB < -100)
			{
				FB = -100;
			}
			if (LR > 100)
			{
				LR = 100;
			}
			if (LR < -100)
			{
				LR = -100;
			}

			/*
             * ★K2急停★
 */
			if (SafetyCommand == REMOTE_SAFE_ESTOP)
			{
				RemoteEmergencyStop = 1;
				RemoteRF_ClearControl();
				return;
			}

			/*
             * ★K1重新解锁★
             * 必须左杆在中间。
 */
			if (SafetyCommand == REMOTE_SAFE_ARM && VerticalCommand == 0)
			{
				RemoteEmergencyStop = 0;
			}

			/*
             * 还在急停锁死状态
 */
			if (RemoteEmergencyStop)
			{
				RemoteRF_ClearControl();
				return;
			}

			/*
             * 正常控制
 */
			RemoteVerticalCommand = VerticalCommand;
			RemoteFB = FB;
			RemoteLR = LR;

			/*
             * 兼容旧接口
 */
			if (VerticalCommand > 0)
			{
				RemoteThrottle =
					(uint8_t)(((uint16_t)VerticalCommand * REMOTE_FLIGHT_THROTTLE_MAX) / 100);
			}
			else
			{
				RemoteThrottle = 0;
			}

			/*
             * Roll
 */
			RemoteTargetRoll = ((float)LR / 100.0f) * REMOTE_MAX_ROLL_DEG;

			/*
             * Pitch
 */
			RemoteTargetPitch = -(((float)FB / 100.0f) * REMOTE_MAX_PITCH_DEG);
			return;
		}
	}

	/*
     * 没收到有效包
 */
	if (RemoteLostCount < REMOTE_LOST_COUNT_MAX)
	{
		RemoteLostCount++;
	}

	/*
     * RF失联
     * 直接重新锁死。
 */
	if (RemoteLostCount >= REMOTE_LOST_COUNT_MAX)
	{
		RemoteConnected = 0;
		RemoteEmergencyStop = 1;
		RemoteRF_ClearControl();
	}
}

/*
 * Getter
 */
uint8_t RemoteRF_GetThrottle(void)
{
	return RemoteThrottle;
}

int8_t RemoteRF_GetVerticalCommand(void)
{
	return RemoteVerticalCommand;
}

float RemoteRF_GetTargetRoll(void)
{
	return RemoteTargetRoll;
}

float RemoteRF_GetTargetPitch(void)
{
	return RemoteTargetPitch;
}

int8_t RemoteRF_GetFB(void)
{
	return RemoteFB;
}

int8_t RemoteRF_GetLR(void)
{
	return RemoteLR;
}

uint8_t RemoteRF_IsConnected(void)
{
	return RemoteConnected;
}

uint8_t RemoteRF_GetEmergencyStop(void)
{
	return RemoteEmergencyStop;
}
