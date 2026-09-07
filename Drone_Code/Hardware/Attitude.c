/**
 * @file Attitude.c
 * @brief MPU6050数据预处理、上电校准和姿态解算入口。
 * @note  使用DWT计算真实dt，并调用IMU模块完成Mahony四元数融合。
 *        该模块保存滤波和零偏状态，不能被多个任务并发调用。
 */
#include "stm32f10x.h"

#include "Delay.h"
#include "MPU6050.h"
#include "Attitude.h"
#include "IMU.h"

#include <math.h>

/*
 * Cortex-M3 DWT
 * 兼容旧版CMSIS
 */
#define DEMCR_REG	   (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL_REG   (*((volatile uint32_t *)0xE0001000))
#define DWT_CYCCNT_REG (*((volatile uint32_t *)0xE0001004))
#define RAD_TO_DEG	   57.2957795f

/*
 * MPU6050量程
 * Gyro ±2000°/s
 * Acc  ±16g
 */
#define GYRO_SCALE 16.4f
#define ACC_SCALE  2048.0f

/*
 * Acc软件低通
 * 旧数据90%
 * 新数据10%
 * 这部分之前已经实测有效，
 * 继续保留。
 */
#define ACC_LPF_OLD 0.96f
#define ACC_LPF_NEW 0.04f

/*
 * ★Gyro软件低通★
 * 旧数据65%
 * 新数据35%
 * 目的：
 * 抑制高油门时空心杯电机、
 * 桨叶和碳纤维机架传入MPU6050的
 * 高频角速度尖峰。
 * Gyro不能像Acc一样滤得太重，
 * 否则角速度内环响应会明显变慢。
 */
#define GYRO_LPF_OLD 0.20f
#define GYRO_LPF_NEW 0.80f

/*
 * Acc可信区间
 * 只有总加速度接近1g时，
 * Mahony才允许使用Acc修正姿态。
 */
#define ACC_MAG_MIN 0.90f
#define ACC_MAG_MAX 1.10f

/*
 * Gyro静态零偏
 * 上电静止校准得到。
 */
static float GyroBiasX = 0.0f;
static float GyroBiasY = 0.0f;
static float GyroBiasZ = 0.0f;

/*
 * MPU6050安装水平零点
 * 根据之前实测：
 * SensorRoll  实际对应飞机Pitch
 * SensorPitch 实际对应飞机Roll
 */
static float SensorZeroRoll = 0.0f;
static float SensorZeroPitch = 0.0f;

/*
 * Acc软件低通状态
 */
static float AccXFiltered = 0.0f;
static float AccYFiltered = 0.0f;
static float AccZFiltered = 1.0f;

/*
 * ★Gyro软件低通状态★
 * 这里仍然是MPU6050原生XYZ坐标，
 * 后面输出给飞机时再交换X/Y。
 */
static float GyroXFiltered = 0.0f;
static float GyroYFiltered = 0.0f;
static float GyroZFiltered = 0.0f;

/*
 * DWT
 */
static uint32_t LastCycle = 0;

/*
 * DWT初始化
 */
static void Attitude_TimeInit(void)
{
	SystemCoreClockUpdate();

	/*
     * 开启DWT
 */
	DEMCR_REG |= (1UL << 24);

	/*
     * Cycle Counter清零
 */
	DWT_CYCCNT_REG = 0;

	/*
     * 开启Cycle Counter
 */
	DWT_CTRL_REG |= 1UL;
	LastCycle = DWT_CYCCNT_REG;
}

/*
 * 获取真实dt
 * 单位：
 * s
 */
static float Attitude_GetDt(void)
{
	uint32_t NowCycle;
	uint32_t DeltaCycle;
	float dt;

	NowCycle = DWT_CYCCNT_REG;
	DeltaCycle = NowCycle - LastCycle;
	LastCycle = NowCycle;
	dt = (float)DeltaCycle / (float)SystemCoreClock;

	/*
     * 防止：
     * 第一次运行
     * 程序中断异常
     * DWT异常
     * 导致dt出现离谱数值。
 */
	if (dt < 0.0005f || dt > 0.05f)
	{
		dt = 0.005f;
	}
	return dt;
}

/*
 * MPU6050静态校准
 * 完成：
 * 1. Gyro静态Bias
 * 2. 当前水平零点
 * 3. Acc低通初始值
 * 4. Gyro低通清零
 * 5. Quaternion初始姿态
 * 上电期间：
 * 飞机必须：
 * 平放
 * 静止
 * 电机不转
 */
static void Attitude_Calibrate(void)
{
	uint16_t i;
	int16_t AX;
	int16_t AY;
	int16_t AZ;
	int16_t GX;
	int16_t GY;
	int16_t GZ;
	float GyroSumX;
	float GyroSumY;
	float GyroSumZ;
	float AccSumX;
	float AccSumY;
	float AccSumZ;
	float AvgAX;
	float AvgAY;
	float AvgAZ;

	GyroSumX = 0.0f;
	GyroSumY = 0.0f;
	GyroSumZ = 0.0f;
	AccSumX = 0.0f;
	AccSumY = 0.0f;
	AccSumZ = 0.0f;

	/*
     * 1. 丢弃前50帧
     * 等待MPU6050内部滤波和PLL稳定。
 */
	for (i = 0; i < 50; i++)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		Delay_ms(2);
	}

	/*
     * 2. 读取500帧
     * 大约1秒。
 */
	for (i = 0; i < 500; i++)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

		/*
         * Gyro原始Bias
 */
		GyroSumX += (float)GX;
		GyroSumY += (float)GY;
		GyroSumZ += (float)GZ;

		/*
         * Acc平均值
 */
		AccSumX += (float)AX / ACC_SCALE;
		AccSumY += (float)AY / ACC_SCALE;
		AccSumZ += (float)AZ / ACC_SCALE;
		Delay_ms(2);
	}

	/*
     * 3. Gyro静态Bias
 */
	GyroBiasX = GyroSumX / 500.0f;
	GyroBiasY = GyroSumY / 500.0f;
	GyroBiasZ = GyroSumZ / 500.0f;

	/*
     * 4. 平均Acc
 */
	AvgAX = AccSumX / 500.0f;
	AvgAY = AccSumY / 500.0f;
	AvgAZ = AccSumZ / 500.0f;

	/*
     * 5. 当前安装姿态作为机体水平零点
 */
	SensorZeroRoll = atan2f(AvgAY, AvgAZ) * RAD_TO_DEG;
	SensorZeroPitch = atan2f(-AvgAX, sqrtf(AvgAY * AvgAY + AvgAZ * AvgAZ)) * RAD_TO_DEG;

	/*
     * 6. Acc软件LPF从真实静态值开始
     * 避免启动后从：
     * 0,0,1
     * 慢慢向真实传感器姿态爬。
 */
	AccXFiltered = AvgAX;
	AccYFiltered = AvgAY;
	AccZFiltered = AvgAZ;

	/*
     * 7. Gyro已经减静态Bias，
     * 所以上电时滤波状态直接从0开始。
 */
	GyroXFiltered = 0.0f;
	GyroYFiltered = 0.0f;
	GyroZFiltered = 0.0f;

	/*
     * 8. 初始化Quaternion
 */
	IMU_Reset();
	IMU_InitQuaternion(AvgAX, AvgAY, AvgAZ);
}

/*
 * 姿态初始化
 */
void Attitude_Init(void)
{
	/*
     * MPU6050
 */
	MPU6050_Init();

	/*
     * MPU6050内部稳定
 */
	Delay_ms(200);

	/*
     * 静态校准
 */
	Attitude_Calibrate();

	/*
     * 时间
 */
	Attitude_TimeInit();
}

/*
 * 姿态更新
 */
void Attitude_Update(Attitude_t *Attitude)
{
	int16_t AX;
	int16_t AY;
	int16_t AZ;
	int16_t GX;
	int16_t GY;
	int16_t GZ;
	float ax;
	float ay;
	float az;
	float gx;
	float gy;
	float gz;
	float SensorRoll;
	float SensorPitch;
	float SensorYaw;
	float RawAccMagnitude;
	float FilteredAccMagnitude;
	float GyroCorrX;
	float GyroCorrY;
	float GyroCorrZ;
	float CorrectedGX;
	float CorrectedGY;
	float dt;
	uint8_t UseAcc;

	/*
     * 1. 一次读取MPU6050六轴数据
 */
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

	/*
     * 2. dt
 */
	dt = Attitude_GetDt();

	/*
     * 3. Acc → g
     * ±16g
     * 2048 LSB/g
 */
	ax = (float)AX / ACC_SCALE;
	ay = (float)AY / ACC_SCALE;
	az = (float)AZ / ACC_SCALE;

	/*
     * 4. 原始Acc模长
     * 如果瞬时加速度明显偏离1g，
     * 就暂时不让Mahony相信Acc。
 */
	RawAccMagnitude = sqrtf(ax * ax + ay * ay + az * az);

	/*
     * 5. Acc软件低通
 */
	AccXFiltered = ACC_LPF_OLD * AccXFiltered + ACC_LPF_NEW * ax;
	AccYFiltered = ACC_LPF_OLD * AccYFiltered + ACC_LPF_NEW * ay;
	AccZFiltered = ACC_LPF_OLD * AccZFiltered + ACC_LPF_NEW * az;
	FilteredAccMagnitude = sqrtf(AccXFiltered * AccXFiltered + AccYFiltered * AccYFiltered
								 + AccZFiltered * AccZFiltered);

	/*
     * 6. Gyro → °/s
     * ±2000°/s
     * 16.4 LSB/(°/s)
     * 先减上电静态Bias。
 */
	gx = ((float)GX - GyroBiasX) / GYRO_SCALE;
	gy = ((float)GY - GyroBiasY) / GYRO_SCALE;
	gz = ((float)GZ - GyroBiasZ) / GYRO_SCALE;

	/*
     * 7. ★Gyro软件低通★
     * 这是针对你高油门Roll来回摇摆
     * 新增加的一层。
     * 例如原来：
     * +140°/s
     * -65°/s
     * +90°/s
     * -88°/s
     * 这种高频变化会先被压一部分，
     * 再进入Quaternion和Rate PID。
 */
	GyroXFiltered = GYRO_LPF_OLD * GyroXFiltered + GYRO_LPF_NEW * gx;
	GyroYFiltered = GYRO_LPF_OLD * GyroYFiltered + GYRO_LPF_NEW * gy;
	GyroZFiltered = GYRO_LPF_OLD * GyroZFiltered + GYRO_LPF_NEW * gz;

	/*
     * 8. 当前Acc是否可信
 */
	UseAcc = 0;

	/*
     * 同时检查：
     * 1. 未滤波Acc
     * 2. 已滤波Acc
     * 防止软件低通把很强的瞬时冲击隐藏掉。
 */
	if (RawAccMagnitude > 0.85f && RawAccMagnitude < 1.15f && FilteredAccMagnitude > ACC_MAG_MIN
		&& FilteredAccMagnitude < ACC_MAG_MAX)
	{
		UseAcc = 1;
	}

	/*
     * 9. 6轴Mahony Quaternion姿态融合
     * ★这里现在使用滤波后的Gyro★
 */
	IMU_6Axis_Gyro_Acc(AccXFiltered, AccYFiltered, AccZFiltered, GyroXFiltered, GyroYFiltered,
					   GyroZFiltered, &SensorRoll, &SensorPitch, &SensorYaw, dt, UseAcc);

	/*
     * 10. 获取Mahony慢速Gyro偏差修正
     * Roll/Pitch可以利用重力方向
     * 慢慢修正Gyro低频Bias。
 */
	IMU_GetGyroCorrection(&GyroCorrX, &GyroCorrY, &GyroCorrZ);

	/*
     * 11. 最终Gyro
     * ★使用滤波后的Gyro★
     * 再叠加Mahony慢速Bias修正。
 */
	CorrectedGX = GyroXFiltered + GyroCorrX;
	CorrectedGY = GyroYFiltered + GyroCorrY;

	/*
     * 12. 转换成飞机机体坐标系
     * 之前已经实测确认：
     * SensorRoll  → 飞机Pitch
     * SensorPitch → 飞机Roll
     * 正负号都不改。
 */
	/*
     * 飞机Roll
     * 左侧低：
     * Roll < 0
     * 右侧低：
     * Roll > 0
 */
	Attitude->Roll = SensorPitch - SensorZeroPitch;

	/*
     * 飞机Pitch
     * 机头低：
     * Pitch < 0
     * 机头高：
     * Pitch > 0
 */
	Attitude->Pitch = SensorRoll - SensorZeroRoll;

	/*
     * Yaw
     * 没有磁力计，
     * 所以只能得到短时间相对Yaw。
 */
	Attitude->Yaw = SensorYaw;

	/*
     * 13. Gyro轴交换
     * MPU6050安装方向已经实测确认：
     * 原GyroY → 飞机Roll Rate
     * 原GyroX → 飞机Pitch Rate
 */
	/*
     * Roll Rate
 */
	Attitude->GyroX = CorrectedGY;

	/*
     * Pitch Rate
 */
	Attitude->GyroY = CorrectedGX;

	/*
     * Yaw Rate
     * Yaw没有磁力计参考，
     * 所以不使用Mahony IntegralZ修正。
     * 但仍然使用软件低通后的GyroZ。
 */
	Attitude->GyroZ = GyroZFiltered;

	/*
     * 14. dt
 */
	Attitude->dt = dt;
}

/*
 * 静态Gyro Bias调试
 */
float Attitude_GetGyroBiasX(void)
{
	return GyroBiasX;
}

float Attitude_GetGyroBiasY(void)
{
	return GyroBiasY;
}

float Attitude_GetGyroBiasZ(void)
{
	return GyroBiasZ;
}
