/**
 * @file IMU.c
 * @brief 六轴Mahony四元数姿态融合和陀螺零偏修正。
 * @note  模块内部保存四元数与积分状态，由Attitude模块单线程调用。
 */
#include "stm32f10x.h"

#include "IMU.h"

#include <math.h>

#define IMU_DEG_TO_RAD 0.01745329252f
#define IMU_RAD_TO_DEG 57.2957795f

/*
 * Mahony参数
 * 江协原代码：
 * Kp = 0.2
 * Ki = 0.1
 * 第一版我们先沿用这个量级，
 * 后续根据你的MPU6050实测再调整。
 */
#define IMU_KP 0.05f
#define IMU_KI 0.01f

/*
 * 积分修正最大允许约：
 * 0.50 rad/s
 * ≈ 28.6 °/s
 * 防止异常振动导致积分无限累积。
 */
#define IMU_INTEGRAL_LIMIT 0.08f

/*
 * 四元数
 */
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

/*
 * Mahony积分修正
 * 单位：
 * rad/s
 * 它本质上会慢慢学习Gyro的偏差。
 */
static float IntegralX = 0.0f;
static float IntegralY = 0.0f;
static float IntegralZ = 0.0f;

/*
 * float限幅
 */
static float IMU_LimitFloat(float Value, float Limit)
{
	if (Value > Limit)
	{
		Value = Limit;
	}
	if (Value < -Limit)
	{
		Value = -Limit;
	}
	return Value;
}

/*
 * 重置
 */
void IMU_Reset(void)
{
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	IntegralX = 0.0f;
	IntegralY = 0.0f;
	IntegralZ = 0.0f;
}

/*
 * 根据加速度初始化四元数
 * 算法结构参考江协IMU_InitQuaternion()
 */
void IMU_InitQuaternion(float ax, float ay, float az)
{
	float Roll;
	float Pitch;
	float Yaw;
	float a;
	float b;
	float g;
	float ca;
	float sa;
	float cb;
	float sb;
	float cg;
	float sg;
	float Norm;

	/*
     * 加速度非法
 */
	Norm = sqrtf(ax * ax + ay * ay + az * az);
	if (Norm < 0.001f)
	{
		IMU_Reset();
		return;
	}

	/*
     * 当前传感器姿态
 */
	Roll = atan2f(ay, az);
	Pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

	/*
     * 没有磁力计，
     * 初始Yaw定义为0。
 */
	Yaw = 0.0f;
	a = Yaw * 0.5f;
	b = Pitch * 0.5f;
	g = Roll * 0.5f;
	ca = cosf(a);
	sa = sinf(a);
	cb = cosf(b);
	sb = sinf(b);
	cg = cosf(g);
	sg = sinf(g);
	q0 = ca * cb * cg + sa * sb * sg;
	q1 = ca * cb * sg - sa * sb * cg;
	q2 = ca * sb * cg + sa * cb * sg;
	q3 = sa * cb * cg - ca * sb * sg;

	/*
     * 四元数归一化
 */
	Norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (Norm > 0.001f)
	{
		q0 /= Norm;
		q1 /= Norm;
		q2 /= Norm;
		q3 /= Norm;
	}
	IntegralX = 0.0f;
	IntegralY = 0.0f;
	IntegralZ = 0.0f;
}

/*
 * 6轴Mahony姿态融合
 * 核心结构来自江协：
 * 加速度归一化
 *      ↓
 * 计算Quaternion预测的重力方向
 *      ↓
 * 实际重力 × 预测重力
 *      ↓
 * 得到ex/ey/ez
 *      ↓
 * Kp + Ki修正Gyro
 *      ↓
 * Quaternion积分
 *      ↓
 * Quaternion归一化
 *      ↓
 * Euler Angle
 * 我们增加：
 * 1. UseAcc动态门控
 * 2. 积分限幅
 * 3. 使用临时Quaternion更新，
 *    避免q0更新后立即影响q1。
 */
void IMU_6Axis_Gyro_Acc(float ax, float ay, float az, float gx, float gy, float gz, float *Roll,
						float *Pitch, float *Yaw, float dt, uint8_t UseAcc)
{
	float Norm;
	float vx;
	float vy;
	float vz;
	float ex;
	float ey;
	float ez;
	float q0New;
	float q1New;
	float q2New;
	float q3New;
	float SinPitch;

	if (dt < 0.0005f || dt > 0.05f)
	{
		dt = 0.005f;
	}

	/*
     * °/s → rad/s
 */
	gx *= IMU_DEG_TO_RAD;
	gy *= IMU_DEG_TO_RAD;
	gz *= IMU_DEG_TO_RAD;

	/*
     * Acc可信时进行Mahony重力修正
 */
	if (UseAcc)
	{
		Norm = sqrtf(ax * ax + ay * ay + az * az);
		if (Norm > 0.001f)
		{
			/*
             * 加速度归一化
 */
			ax /= Norm;
			ay /= Norm;
			az /= Norm;

			/*
             * Quaternion预测重力方向
             * 和江协公式一致
 */
			vx = 2.0f * (q1 * q3 - q0 * q2);
			vy = 2.0f * (q0 * q1 + q2 * q3);
			vz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

			/*
             * 实际重力与预测重力叉乘
 */
			ex = ay * vz - az * vy;
			ey = az * vx - ax * vz;
			ez = ax * vy - ay * vx;

			/*
             * Integral
             * 用于慢慢估计Gyro偏差
 */
			IntegralX += IMU_KI * ex * dt;
			IntegralY += IMU_KI * ey * dt;
			IntegralZ += IMU_KI * ez * dt;
			IntegralX = IMU_LimitFloat(IntegralX, IMU_INTEGRAL_LIMIT);
			IntegralY = IMU_LimitFloat(IntegralY, IMU_INTEGRAL_LIMIT);
			IntegralZ = IMU_LimitFloat(IntegralZ, IMU_INTEGRAL_LIMIT);

			/*
             * P + I修正Gyro
 */
			gx += IMU_KP * ex;
			gy += IMU_KP * ey;
			gz += IMU_KP * ez;
		}
	}

	/*
     * 无论当前Acc是否可信，
     * 已经学习到的Integral偏差修正
     * 都继续作用。
 */
	gx += IntegralX;
	gy += IntegralY;

	/*
     * 六轴系统对Yaw不可观，
     * 所以Yaw角速度暂时不使用
     * IntegralZ长期校正。
     * 防止错误Yaw积分。
 */
	/* gz += IntegralZ; */
	/*
     * Quaternion更新
 */
	q0New = q0 + 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
	q1New = q1 + 0.5f * (q0 * gx - q3 * gy + q2 * gz) * dt;
	q2New = q2 + 0.5f * (q3 * gx + q0 * gy - q1 * gz) * dt;
	q3New = q3 + 0.5f * (-q2 * gx + q1 * gy + q0 * gz) * dt;
	q0 = q0New;
	q1 = q1New;
	q2 = q2New;
	q3 = q3New;

	/*
     * Quaternion归一化
 */
	Norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	if (Norm > 0.001f)
	{
		q0 /= Norm;
		q1 /= Norm;
		q2 /= Norm;
		q3 /= Norm;
	}
	else
	{
		IMU_Reset();
	}

	/*
     * Quaternion → Euler
 */
	*Roll = atan2f(2.0f * (q2 * q3 + q0 * q1), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * IMU_RAD_TO_DEG;

	SinPitch = 2.0f * (q0 * q2 - q1 * q3);

	/*
     * 防止浮点误差导致asinf输入>1
 */
	if (SinPitch > 1.0f)
	{
		SinPitch = 1.0f;
	}
	if (SinPitch < -1.0f)
	{
		SinPitch = -1.0f;
	}

	*Pitch = asinf(SinPitch) * IMU_RAD_TO_DEG;

	*Yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * IMU_RAD_TO_DEG;
}

/*
 * 获取积分Gyro修正
 * Roll/Pitch轴可以利用重力慢慢学习偏差。
 * Yaw没有磁力计，CorrZ不使用。
 */
void IMU_GetGyroCorrection(float *CorrX, float *CorrY, float *CorrZ)
{
	*CorrX = IntegralX * IMU_RAD_TO_DEG;

	*CorrY = IntegralY * IMU_RAD_TO_DEG;

	*CorrZ = 0.0f;
}
