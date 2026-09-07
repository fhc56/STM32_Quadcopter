/**
 * @file FlightControl.c
 * @brief Roll/Pitch/Yaw串级控制、故障锁存和四电机混控。
 * @note  这是飞行安全核心模块，只能在FlightControl任务中更新；本次移植
 *        不改变任何PID参数、限幅条件或电机混控公式。
 */
#include "stm32f10x.h"

#include "FlightControl.h"
#include "Motor.h"
#include "PID.h"

#include <math.h>

/*
 * 基础参数
 */
#define FC_MOTOR_MAX			  90.0f
#define FC_THROTTLE_MAX			  70
#define FC_CONTROL_START_THROTTLE 10
#define FC_MAX_TILT_DEG			  35.0f
#define FC_I_ENABLE_THROTTLE	  50

/*
 * 目标姿态
 * 新算法第一轮全部归零。
 */
#define FC_TARGET_ROLL	   0.0f
#define FC_TARGET_PITCH	   0.0f
#define FC_TARGET_YAW_RATE 0.0f

/*
 * 江协式“姿态零点Trim”
 * 注意：
 * 不再直接给某两个电机±PWM。
 * 如果以后发现固定偏差，
 * 再调这里。
 * 第一轮全部0。
 */
#define FC_ROLL_TRIM_DEG		 0.0f
#define FC_PITCH_TRIM_DEG		 0.0f
#define FC_FRONT_MOTOR_TRIM_GAIN 0.0605f
#define FC_FRONT_MOTOR_TRIM_MAX	 4.0f

/*
 * Angle Loop分频
 * Rate：
 * 每次 ≈200Hz
 * Angle：
 * 每2次 ≈100Hz
 */
#define FC_ANGLE_LOOP_DIVIDER 2

/*
 * PID
 */
static PID_t RollAnglePID;
static PID_t PitchAnglePID;
static PID_t RollRatePID;
static PID_t PitchRatePID;
static PID_t YawRatePID;

/*
 * Angle Loop状态
 */
static uint8_t AngleLoopCount = 0;
static float AngleLoopDt = 0.0f;
static float RollRateTarget = 0.0f;
static float PitchRateTarget = 0.0f;

/*
 * Fault
 */
static uint8_t FaultLatched = 0;

/*
 * Debug
 */
static float DebugYawOutput = 0.0f;
static uint8_t DebugM1 = 0;
static uint8_t DebugM2 = 0;
static uint8_t DebugM3 = 0;
static uint8_t DebugM4 = 0;
static float DebugTargetRoll = 0.0f;
static float DebugTargetPitch = 0.0f;

/* Roll轴调试数据 */
static float DebugRollRateTarget = 0.0f;
static float DebugRollOutput = 0.0f;

/*
 * Pitch串级PID调试
 */
/*
 * Pitch角度外环输出：
 * 也就是希望飞机达到的Pitch角速度
 * 单位：°/s
 */
static float DebugPitchRateTarget = 0.0f;

/*
 * Pitch角速度内环最终输出
 * 这个值最终进入电机混控
 */
static float DebugPitchOutput = 0.0f;

/* 浮点值限幅。 */
static float FlightControl_LimitFloat(float Value, float Minimum, float Maximum)
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

/* 电机输出限幅并四舍五入为整数。 */
static uint8_t FlightControl_LimitMotor(float Value)
{
	Value = FlightControl_LimitFloat(Value, 0.0f, FC_MOTOR_MAX);
	return (uint8_t)(Value + 0.5f);
}

/* 复位全部串级PID状态及角度环累计量。 */
static void FlightControl_ResetPID(void)
{
	PID_Reset(&RollAnglePID);
	PID_Reset(&PitchAnglePID);
	PID_Reset(&RollRatePID);
	PID_Reset(&PitchRatePID);
	PID_Reset(&YawRatePID);
	RollRateTarget = 0.0f;
	PitchRateTarget = 0.0f;
	AngleLoopCount = 0;
	AngleLoopDt = 0.0f;
}

/* 清除本周期对外提供的调试数据。 */
static void FlightControl_ClearDebug(void)
{
	DebugYawOutput = 0.0f;
	DebugM1 = 0;
	DebugM2 = 0;
	DebugM3 = 0;
	DebugM4 = 0;
	DebugTargetRoll = FC_TARGET_ROLL;
	DebugTargetPitch = FC_TARGET_PITCH;
	DebugPitchRateTarget = 0.0f;
	DebugPitchOutput = 0.0f;
	DebugRollRateTarget = 0.0f;
	DebugRollOutput = 0.0f;
}

/*
 * Yaw剩余余量限制
 * Roll/Pitch优先。
 * 保留我们之前已经验证过的逻辑。
 */
static float FlightControl_LimitYawByMotorHeadroom(float YawOutput, float BaseM1, float BaseM2,
												   float BaseM3, float BaseM4)
{
	float YawMin;
	float YawMax;

	YawMin = -BaseM1;
	if (-BaseM4 > YawMin)
	{
		YawMin = -BaseM4;
	}
	if (BaseM2 - FC_MOTOR_MAX > YawMin)
	{
		YawMin = BaseM2 - FC_MOTOR_MAX;
	}
	if (BaseM3 - FC_MOTOR_MAX > YawMin)
	{
		YawMin = BaseM3 - FC_MOTOR_MAX;
	}
	YawMax = FC_MOTOR_MAX - BaseM1;
	if (FC_MOTOR_MAX - BaseM4 < YawMax)
	{
		YawMax = FC_MOTOR_MAX - BaseM4;
	}
	if (BaseM2 < YawMax)
	{
		YawMax = BaseM2;
	}
	if (BaseM3 < YawMax)
	{
		YawMax = BaseM3;
	}
	if (YawMin > YawMax)
	{
		return 0.0f;
	}
	return FlightControl_LimitFloat(YawOutput, YawMin, YawMax);
}

/*
 * Init
 */
void FlightControl_Init(void)
{
	FaultLatched = 0;

	/*
     * Angle PID
     * 江协：
     * Roll  Kp=3
     * Pitch Kp=3
     * 第一版我们也从3开始。
 */
	PID_Init(&RollAnglePID, 3.0f, 0.0f, 0.0f, 100.0f, 0.0f);
	PID_Init(&PitchAnglePID, 3.0f, 0.0f, 0.0f, 100.0f, 0.0f);

	/*
     * Rate PID
     * 江协原参数：
     * Kp=0.15
     * Ki=0.10
     * Kd=0
     * 我们保持接近，
     * 但限制Motor修正范围。
 */
	PID_Init(&RollRatePID,
			 0.06f, //0.10
			 0.3f, 0.0f, 15.0f, 3.0f

			 /*
        0.15f,
        0.08f,
        0.0f,
        18.0f,
        5.0f
	 */
	);
	PID_Init(&PitchRatePID, 0.09f, 0.005f, 0.0f, 14.0f, 3.0f);

	/*
     * 当前没有磁力计，
     * Yaw只做Rate=0稳定，
     * 不做Yaw角度保持。
 */
	PID_Init(&YawRatePID, 0.20f, 0.0f, 0.0f, 20.0f, 0.0f);
	FlightControl_ResetPID();
	FlightControl_ClearDebug();
	Motor_StopAll();
}

/*
 * Update
 */
void FlightControl_Update(uint8_t Throttle, const Attitude_t *Attitude)
{
	float dt;
	float RollFeedback;
	float PitchFeedback;
	float RollOutput;
	float PitchOutput;
	float YawOutput;
	float FrontMotorTrim;
	float BaseM1;
	float BaseM2;
	float BaseM3;
	float BaseM4;
	float Motor1;
	float Motor2;
	float Motor3;
	float Motor4;

	/*
     * 1. Fault
 */
	if (FaultLatched)
	{
		Motor_StopAll();
		FlightControl_ClearDebug();
		return;
	}

	/*
     * 2. Throttle limit
 */
	if (Throttle > FC_THROTTLE_MAX)
	{
		Throttle = FC_THROTTLE_MAX;
	}

	/*
     * 3. Low Throttle
 */
	if (Throttle < FC_CONTROL_START_THROTTLE)
	{
		Motor_StopAll();
		FlightControl_ResetPID();
		FlightControl_ClearDebug();
		return;
	}

	/*
     * 4. Tilt Fault
 */
	if (fabsf(Attitude->Roll) > FC_MAX_TILT_DEG || fabsf(Attitude->Pitch) > FC_MAX_TILT_DEG)
	{
		FaultLatched = 1;
		Motor_StopAll();
		FlightControl_ClearDebug();
		return;
	}

	/*
     * 5. dt
 */
	dt = Attitude->dt;
	if (dt < 0.0005f || dt > 0.05f)
	{
		dt = 0.005f;
	}

	/*
     * 6. 江协式姿态Trim
     * Trim作用在Feedback，
     * 不直接改Motor。
 */
	RollFeedback = Attitude->Roll + FC_ROLL_TRIM_DEG;
	PitchFeedback = Attitude->Pitch + FC_PITCH_TRIM_DEG;
	DebugTargetRoll = FC_TARGET_ROLL;
	DebugTargetPitch = FC_TARGET_PITCH;

	/*
			 * 地面防积分 Wind-up
			 * 低于真正起飞附近油门时，
			 * Roll/Pitch Rate I项强制清零。
			 * 这样：
			 * 地面上不会提前积累控制量；
			 * 起飞以后I项才慢慢学习机械偏差。
			 */
	if (Throttle < FC_I_ENABLE_THROTTLE)
	{
		RollRatePID.Integral = 0.0f;
		PitchRatePID.Integral = 0.0f;
	}

	/*
     * 7. Angle Outer Loop
     * ≈100Hz
 */
	AngleLoopCount++;
	AngleLoopDt += dt;
	if (AngleLoopCount >= FC_ANGLE_LOOP_DIVIDER)
	{
		RollRateTarget = PID_Update(&RollAnglePID, FC_TARGET_ROLL, RollFeedback, AngleLoopDt);
		PitchRateTarget = PID_Update(&PitchAnglePID, FC_TARGET_PITCH, PitchFeedback, AngleLoopDt);
		AngleLoopCount = 0;
		AngleLoopDt = 0.0f;
	}
	DebugPitchRateTarget = PitchRateTarget;

	/*
     * 8. Rate Inner Loop
     * ≈200Hz
 */
	RollOutput = PID_Update(&RollRatePID, RollRateTarget, Attitude->GyroX, dt);
	DebugRollRateTarget = RollRateTarget;
	DebugRollOutput = RollOutput;
	PitchOutput = PID_Update(&PitchRatePID, PitchRateTarget, Attitude->GyroY, dt);
	DebugPitchOutput = PitchOutput;
	YawOutput = PID_Update(&YawRatePID, FC_TARGET_YAW_RATE, Attitude->GyroZ, dt);

	/*
     * 9. Roll/Pitch Mixer
     *           前
     *     M1          M3
     *     M2          M4
     *           后
     * M1 = 左前
     * M2 = 左后
     * M3 = 右前
     * M4 = 右后
     * 这是我们之前已经实测确认过的方向。
 */
	/*
			 * 前电机基础补偿
			 * M1/M3实测偏猛，
			 * 按Throttle比例稍微削弱。
			 * T=50 -> 减2
			 * T=60 -> 减2.4
			 * T=70 -> 减2.8
			 * 最大只减3。
			 */
	FrontMotorTrim = (float)Throttle * FC_FRONT_MOTOR_TRIM_GAIN;
	if (FrontMotorTrim > FC_FRONT_MOTOR_TRIM_MAX)
	{
		FrontMotorTrim = FC_FRONT_MOTOR_TRIM_MAX;
	}
	BaseM1 = (float)Throttle + RollOutput + PitchOutput - FrontMotorTrim;
	BaseM2 = (float)Throttle + RollOutput - PitchOutput;
	BaseM3 = (float)Throttle - RollOutput + PitchOutput - FrontMotorTrim;
	BaseM4 = (float)Throttle - RollOutput - PitchOutput;

	/*
     * 10. Yaw只使用剩余Motor余量
 */
	YawOutput = FlightControl_LimitYawByMotorHeadroom(YawOutput, BaseM1, BaseM2, BaseM3, BaseM4);
	DebugYawOutput = YawOutput;

	/*
     * 11. Yaw Mixer
     * 之前已经实机验证：
     * M1/M4 +
     * M2/M3 -
     * 不要再反这个符号。
 */
	Motor1 = BaseM1 + YawOutput;
	Motor2 = BaseM2 - YawOutput;
	Motor3 = BaseM3 - YawOutput;
	Motor4 = BaseM4 + YawOutput;

	/*
     * 12. Limit
 */
	DebugM1 = FlightControl_LimitMotor(Motor1);
	DebugM2 = FlightControl_LimitMotor(Motor2);
	DebugM3 = FlightControl_LimitMotor(Motor3);
	DebugM4 = FlightControl_LimitMotor(Motor4);

	/*
     * 13. Motor
 */
	Motor_SetSpeed1(DebugM1);
	Motor_SetSpeed2(DebugM2);
	Motor_SetSpeed3(DebugM3);
	Motor_SetSpeed4(DebugM4);
}

/*
 * Debug
 */
uint8_t FlightControl_IsFaultLatched(void)
{
	return FaultLatched;
}

float FlightControl_GetYawOutput(void)
{
	return DebugYawOutput;
}

uint8_t FlightControl_GetM1(void)
{
	return DebugM1;
}

uint8_t FlightControl_GetM2(void)
{
	return DebugM2;
}

uint8_t FlightControl_GetM3(void)
{
	return DebugM3;
}

uint8_t FlightControl_GetM4(void)
{
	return DebugM4;
}

float FlightControl_GetTargetRoll(void)
{
	return DebugTargetRoll;
}

float FlightControl_GetTargetPitch(void)
{
	return DebugTargetPitch;
}

float FlightControl_GetPitchRateTarget(void)
{
	return DebugPitchRateTarget;
}

float FlightControl_GetPitchOutput(void)
{
	return DebugPitchOutput;
}

float FlightControl_GetRollRateTarget(void)
{
	return DebugRollRateTarget;
}

float FlightControl_GetRollOutput(void)
{
	return DebugRollOutput;
}
