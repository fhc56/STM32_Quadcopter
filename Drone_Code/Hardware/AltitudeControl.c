/**
 * @file AltitudeControl.c
 * @brief 定高状态机与油门补偿。
 * @note  输入为遥控升降指令、姿态周期和气压高度，输出为0~100油门。
 *        本模块只允许由200 Hz飞控任务调用，算法参数和状态转换保持原样。
 */
#include "stm32f10x.h"

#include "AltitudeControl.h"
#include "Barometer.h"

#include <math.h>

/*
 * 最大高度
 */
#define ALT_MAX_HEIGHT_M 1.50f

/*
 * 左杆死区
 */
#define ALT_STICK_DEADBAND 10

/* 起飞确认：高度连续5个20 Hz周期超过3 cm，总确认时间约250 ms。 */
#define ALT_TAKEOFF_CONFIRM_HEIGHT_M 0.03f
#define ALT_TAKEOFF_CONFIRM_COUNT	 5

/*
 * 起飞阶段直接油门控制：左杆正值时不完全依赖高度PID。
 * 轻推杆约输出45%油门，满杆允许升到70%。
 */
#define ALT_TAKEOFF_MIN_THROTTLE 45.0f
#define ALT_TAKEOFF_RISE_RATE	 60.0f
#define ALT_TAKEOFF_FALL_RATE	 80.0f

/*
 * 目标高度变化速度
 */
#define ALT_MAX_CLIMB_RATE_MPS	 0.30f
#define ALT_MAX_DESCEND_RATE_MPS 0.30f

/*
 * 定高基础油门
 */
#define ALT_HOVER_THROTTLE		 55.0f
#define ALT_MIN_CONTROL_THROTTLE 32.0f
#define ALT_MAX_CONTROL_THROTTLE 70.0f

/*
 * 高度PID
 */
#define ALT_KP				6.0f
#define ALT_KI				0.60f
#define ALT_KD_VSPEED		3.2f
#define ALT_INTEGRAL_LIMIT	3.5f
#define ALT_HEIGHT_I_ZONE_M 0.35f

/*
 * 20Hz高度环
 */
#define ALT_CONTROL_PERIOD_S 0.050f

/*
 * 垂直速度
 */
#define ALT_VSPEED_LPF_OLD	 0.75f
#define ALT_VSPEED_LPF_NEW	 0.25f
#define ALT_VSPEED_LIMIT_MPS 2.0f

/*
 * 定高状态下油门变化速度
 */
#define ALT_FLIGHT_RISE_RATE 22.0f
#define ALT_FLIGHT_FALL_RATE 35.0f

/*
 * 手动下降：左杆为负时直接降低油门并跳过高度PID，
 * 防止PID立即回补刚刚减掉的油门。
 */
#define ALT_MANUAL_DESCEND_BASE_RATE  25.0f
#define ALT_MANUAL_DESCEND_EXTRA_RATE 65.0f

/* 手动下降保护：高度超过15 cm时保留28%油门；低空仍允许逐步降到0。 */
#define ALT_MANUAL_DESCEND_AIR_FLOOR	28.0f
#define ALT_MANUAL_DESCEND_FLOOR_HEIGHT 0.15f

/*
 * 状态
 */
#define ALT_STATE_GROUND  0
#define ALT_STATE_TAKEOFF 1
#define ALT_STATE_FLIGHT  2

static uint8_t SensorOK = 0;
static uint8_t AltitudeState = ALT_STATE_GROUND;
static uint8_t ManualLanding = 0;
static float TargetHeight = 0.0f;
static float HeightIntegral = 0.0f;
static float LastHeight = 0.0f;
static float VerticalSpeed = 0.0f;
static float OutputThrottle = 0.0f;
static float ControlTimer = 0.0f;
static uint8_t TakeoffConfirmCounter = 0;

/*
 * 限幅
 */
static float Altitude_Limit(float Value, float Minimum, float Maximum)
{
	if (Value > Maximum)
	{
		Value = Maximum;
	}
	if (Value < Minimum)
	{
		Value = Minimum;
	}
	return Value;
}

/*
 * 油门缓变
 */
static void Altitude_MoveThrottle(float DesiredThrottle, float RiseRate, float FallRate, float dt)
{
	float MaxChange;

	/*
     * 加速
 */
	if (DesiredThrottle > OutputThrottle)
	{
		MaxChange = RiseRate * dt;
		if (DesiredThrottle - OutputThrottle > MaxChange)
		{
			OutputThrottle += MaxChange;
		}
		else
		{
			OutputThrottle = DesiredThrottle;
		}
	}

	/*
     * 减速
 */
	else
	{
		MaxChange = FallRate * dt;
		if (OutputThrottle - DesiredThrottle > MaxChange)
		{
			OutputThrottle -= MaxChange;
		}
		else
		{
			OutputThrottle = DesiredThrottle;
		}
	}
	OutputThrottle = Altitude_Limit(OutputThrottle, 0.0f, ALT_MAX_CONTROL_THROTTLE);
}

/*
 * 回到地面
 */
static void Altitude_ResetToGround(void)
{
	AltitudeState = ALT_STATE_GROUND;
	ManualLanding = 0;
	TargetHeight = 0.0f;
	HeightIntegral = 0.0f;
	LastHeight = Barometer_GetControlHeight();
	VerticalSpeed = 0.0f;
	OutputThrottle = 0.0f;
	ControlTimer = 0.0f;
	TakeoffConfirmCounter = 0;
	Barometer_GroundZeroUpdate(1);
}

/*
 * 更新垂直速度
 */
static void Altitude_UpdateVerticalSpeed(float Height, float SampleDt)
{
	float RawVerticalSpeed;

	RawVerticalSpeed = (Height - LastHeight) / SampleDt;
	LastHeight = Height;
	RawVerticalSpeed =
		Altitude_Limit(RawVerticalSpeed, -ALT_VSPEED_LIMIT_MPS, ALT_VSPEED_LIMIT_MPS);
	VerticalSpeed = ALT_VSPEED_LPF_OLD * VerticalSpeed + ALT_VSPEED_LPF_NEW * RawVerticalSpeed;
}

/*
 * 初始化
 */
void AltitudeControl_Init(void)
{
	SensorOK = Barometer_IsOK();
	Altitude_ResetToGround();
}

/*
 * 更新
 */
void AltitudeControl_Update(int8_t VerticalCommand, float dt, uint8_t RFConnected)
{
	float Height;
	float CommandScale;
	float DesiredThrottle;
	float HeightError;
	float SampleDt;
	float DescendRate;
	float MinThrottle;

	/*
     * dt保护
 */
	if (dt < 0.0005f || dt > 0.05f)
	{
		dt = 0.005f;
	}
	SensorOK = Barometer_IsOK();

	/*
     * BMP异常：
     * 停止。
 */
	if (!SensorOK)
	{
		Altitude_ResetToGround();
		return;
	}

	/*
     * 摇杆限幅
 */
	if (VerticalCommand > 100)
	{
		VerticalCommand = 100;
	}
	if (VerticalCommand < -100)
	{
		VerticalCommand = -100;
	}

	/*
     * 中心死区
 */
	if (VerticalCommand <= ALT_STICK_DEADBAND && VerticalCommand >= -ALT_STICK_DEADBAND)
	{
		VerticalCommand = 0;
	}

	/*
     * RF失联：
     * 本高度控制直接回地面状态。
     * 此外RemoteRF还会把急停锁住。
 */
	if (!RFConnected)
	{
		Altitude_ResetToGround();
		return;
	}
	Height = Barometer_GetControlHeight();

	/*
     * 地面待机
 */
	if (AltitudeState == ALT_STATE_GROUND)
	{
		OutputThrottle = 0.0f;
		TargetHeight = 0.0f;
		HeightIntegral = 0.0f;
		VerticalSpeed = 0.0f;
		LastHeight = Height;
		TakeoffConfirmCounter = 0;
		ManualLanding = 0;
		Barometer_GroundZeroUpdate(1);

		/*
         * 左杆向上：
         * 开始起飞。
 */
		if (VerticalCommand > ALT_STICK_DEADBAND)
		{
			AltitudeState = ALT_STATE_TAKEOFF;
			if (Height > 0.0f)
			{
				TargetHeight = Height;
			}
			else
			{
				TargetHeight = 0.0f;
			}
			LastHeight = Height;
			ControlTimer = 0.0f;
			Barometer_GroundZeroUpdate(0);
		}
		else
		{
			return;
		}
	}

	/*
     * 起飞以后冻结地面气压零点
 */
	Barometer_GroundZeroUpdate(0);

	/*
     * ★★★ 最重要：VC < 0 ★★★
     * 只要左杆往下：
     * 1. 立即减T
     * 2. 清高度I项
     * 3. TH不得继续高于当前H
     * 4. 本轮直接return
     * 所以高度PID绝对没有机会
     * 又把油门加回去。
 */
	if (VerticalCommand < -ALT_STICK_DEADBAND)
	{
		CommandScale =
			((float)(-VerticalCommand) - ALT_STICK_DEADBAND) / (100.0f - ALT_STICK_DEADBAND);
		CommandScale = Altitude_Limit(CommandScale, 0.0f, 1.0f);

		/*
         * -90以下：
         * 调试数据显示Landing=1。
 */
		if (VerticalCommand <= -90)
		{
			ManualLanding = 1;
		}
		else
		{
			ManualLanding = 0;
		}

		/*
         * 防止旧TH还在很高的位置
         * 例如：
         * H=5cm
         * TH=30cm
         * 现在直接把TH拉回H。
 */
		if (TargetHeight > Height)
		{
			TargetHeight = Height;
		}

		/*
         * 同时降低目标高度
 */
		TargetHeight -= CommandScale * ALT_MAX_DESCEND_RATE_MPS * dt;
		if (TargetHeight < 0.0f)
		{
			TargetHeight = 0.0f;
		}

		/*
         * 清掉高度I项
         * 防止之前积累的油门继续顶着。
 */
		HeightIntegral = 0.0f;

		/*
         * 直接减基础油门
         * VC越负：
         * 减得越快。
 */
		DescendRate = ALT_MANUAL_DESCEND_BASE_RATE + ALT_MANUAL_DESCEND_EXTRA_RATE * CommandScale;
		OutputThrottle -= DescendRate * dt;

		/*
         * 高于15cm时：
         * 暂时不让它一下减到0，
         * 防止自由落体。
         * 低于15cm：
         * 可以一路减到0。
 */
		if (Height > ALT_MANUAL_DESCEND_FLOOR_HEIGHT && AltitudeState == ALT_STATE_FLIGHT)
		{
			MinThrottle = ALT_MANUAL_DESCEND_AIR_FLOOR;
		}
		else
		{
			MinThrottle = 0.0f;
		}
		if (OutputThrottle < MinThrottle)
		{
			OutputThrottle = MinThrottle;
		}

		/*
         * 已经低空且油门归0：
         * 回地面待机。
 */
		if (OutputThrottle <= 0.5f && Height < 0.12f)
		{
			Altitude_ResetToGround();
		}

		/*
         * ★一定return★
 */
		return;
	}
	ManualLanding = 0;

	/*
     * ★起飞阶段★
     * 左杆正值直接决定起飞基础油门。
     * 满杆VC=100：
     * DesiredThrottle = 70
 */
	if (AltitudeState == ALT_STATE_TAKEOFF)
	{
		/*
         * 还没确认离地前：
         * 松杆就停。
 */
		if (VerticalCommand == 0)
		{
			Altitude_ResetToGround();
			return;
		}
		CommandScale =
			((float)VerticalCommand - ALT_STICK_DEADBAND) / (100.0f - ALT_STICK_DEADBAND);
		CommandScale = Altitude_Limit(CommandScale, 0.0f, 1.0f);
		TargetHeight += CommandScale * ALT_MAX_CLIMB_RATE_MPS * dt;
		TargetHeight = Altitude_Limit(TargetHeight, 0.0f, ALT_MAX_HEIGHT_M);

		/*
         * VC=100：
         * 真正允许拉到70。
 */
		DesiredThrottle = ALT_TAKEOFF_MIN_THROTTLE
						  + CommandScale * (ALT_MAX_CONTROL_THROTTLE - ALT_TAKEOFF_MIN_THROTTLE);
		Altitude_MoveThrottle(DesiredThrottle, ALT_TAKEOFF_RISE_RATE, ALT_TAKEOFF_FALL_RATE, dt);
	}

	/*
     * 已进入定高
 */
	else if (AltitudeState == ALT_STATE_FLIGHT)
	{
		/*
         * VC>0：
         * 提高TH。
 */
		if (VerticalCommand > ALT_STICK_DEADBAND)
		{
			CommandScale =
				((float)VerticalCommand - ALT_STICK_DEADBAND) / (100.0f - ALT_STICK_DEADBAND);
			CommandScale = Altitude_Limit(CommandScale, 0.0f, 1.0f);
			TargetHeight += CommandScale * ALT_MAX_CLIMB_RATE_MPS * dt;
			TargetHeight = Altitude_Limit(TargetHeight, 0.0f, ALT_MAX_HEIGHT_M);
		}

		/*
         * VC=0：
         * TargetHeight完全不改，
         * 即定高。
 */
	}

	/*
     * 20Hz高度处理
 */
	ControlTimer += dt;
	if (ControlTimer < ALT_CONTROL_PERIOD_S)
	{
		return;
	}
	SampleDt = ControlTimer;
	ControlTimer = 0.0f;
	Height = Barometer_GetControlHeight();
	Altitude_UpdateVerticalSpeed(Height, SampleDt);

	/*
     * ★起飞确认★
     * 现在只要求：
     * H >= 3cm
     * 连续5次。
 */
	if (AltitudeState == ALT_STATE_TAKEOFF)
	{
		if (Height >= ALT_TAKEOFF_CONFIRM_HEIGHT_M)
		{
			if (TakeoffConfirmCounter < 255)
			{
				TakeoffConfirmCounter++;
			}
			if (TakeoffConfirmCounter >= ALT_TAKEOFF_CONFIRM_COUNT)
			{
				AltitudeState = ALT_STATE_FLIGHT;
				TakeoffConfirmCounter = 0;
				HeightIntegral = 0.0f;
				if (TargetHeight < Height)
				{
					TargetHeight = Height;
				}
			}
		}
		else
		{
			TakeoffConfirmCounter = 0;
		}

		/*
         * 还没确认进入FLIGHT：
         * 起飞油门已经在上面算了，
         * 不执行高度PID。
 */
		if (AltitudeState == ALT_STATE_TAKEOFF)
		{
			return;
		}
	}

	/*
     * 正常定高
 */
	HeightError = TargetHeight - Height;

	/*
     * I项
 */
	if (fabsf(HeightError) < ALT_HEIGHT_I_ZONE_M)
	{
		HeightIntegral += ALT_KI * HeightError * SampleDt;
		HeightIntegral = Altitude_Limit(HeightIntegral, -ALT_INTEGRAL_LIMIT, ALT_INTEGRAL_LIMIT);
	}

	/*
     * 定高油门
 */
	DesiredThrottle =
		ALT_HOVER_THROTTLE + ALT_KP * HeightError + HeightIntegral - ALT_KD_VSPEED * VerticalSpeed;
	DesiredThrottle =
		Altitude_Limit(DesiredThrottle, ALT_MIN_CONTROL_THROTTLE, ALT_MAX_CONTROL_THROTTLE);
	Altitude_MoveThrottle(DesiredThrottle, ALT_FLIGHT_RISE_RATE, ALT_FLIGHT_FALL_RATE, SampleDt);
}

/*
 * Getter
 */
uint8_t AltitudeControl_GetThrottle(void)
{
	float Value;

	if (AltitudeState == ALT_STATE_GROUND)
	{
		return 0;
	}
	Value = Altitude_Limit(OutputThrottle, 0.0f, ALT_MAX_CONTROL_THROTTLE);
	return (uint8_t)(Value + 0.5f);
}

float AltitudeControl_GetHeight(void)
{
	return Barometer_GetControlHeight();
}

float AltitudeControl_GetTargetHeight(void)
{
	return TargetHeight;
}

float AltitudeControl_GetVerticalSpeed(void)
{
	return VerticalSpeed;
}

float AltitudeControl_GetThrottleFloat(void)
{
	return OutputThrottle;
}

uint8_t AltitudeControl_IsActive(void)
{
	if (AltitudeState != ALT_STATE_GROUND)
	{
		return 1;
	}
	return 0;
}

uint8_t AltitudeControl_IsLanding(void)
{
	return ManualLanding;
}

uint8_t AltitudeControl_IsSensorOK(void)
{
	return SensorOK;
}
