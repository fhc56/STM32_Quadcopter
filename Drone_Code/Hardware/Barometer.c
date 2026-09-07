/**
 * @file Barometer.c
 * @brief BMP280气压高度、地面零点和20 Hz高度滤波。
 * @note  Barometer_Update由200 Hz飞控任务调用，模块内部自行限频到20 Hz。
 */
#include "stm32f10x.h"

#include "Barometer.h"
#include "BMP280.h"
#include "Delay.h"

#include <math.h>

/*
 * BMP280更新周期
 * 50ms = 20Hz
 */
#define BARO_UPDATE_PERIOD 0.050f

/*
 * 初始化预热
 * BMP280上电以后先稳定3秒，
 * 不立即拿它的气压建立0点。
 */
#define BARO_WARMUP_TIME_MS 3000

/*
 * 建立地面零点时：
 * 先丢掉50次
 * 再平均200次
 */
#define BARO_CAL_DISCARD_COUNT 50
#define BARO_CAL_SAMPLE_COUNT  200

/*
 * 气压软件低通
 * 旧值90%
 * 新值10%
 * 这一层直接滤Pressure。
 */
#define BARO_PRESSURE_LPF_OLD 0.90f
#define BARO_PRESSURE_LPF_NEW 0.10f

/*
 * 高度软件低通
 * 再对高度做一次滤波。
 * 旧值90%
 * 新值10%
 */
#define BARO_HEIGHT_LPF_OLD 0.90f
#define BARO_HEIGHT_LPF_NEW 0.10f

/*
 * 高度死区
 * 单位m
 * ±0.08m = ±8cm
 * 小于这个范围的变化，
 * 暂时认为是气压噪声。
 */
#define BARO_HEIGHT_DEADBAND 0.08f

/*
 * 高度合理范围
 * 测试阶段先防止异常数据。
 */
#define BARO_HEIGHT_MIN -5.0f
#define BARO_HEIGHT_MAX 20.0f

/*
 * 地面零点跟踪速度
 * 0.02：
 * 每次20Hz更新时，
 * GroundPressure向当前气压靠近2%。
 * 不会突然跳零点，
 * 但放地面几秒后会慢慢归零。
 */
#define BARO_GROUND_ZERO_ALPHA 0.02f

/*
 * 状态
 */
static uint8_t BarometerOK = 0;

/*
 * 当前滤波后的气压
 */
static float PressureFiltered = 0.0f;

/*
 * 开机地面气压
 */
static float GroundPressure = 101325.0f;

/*
 * 是否允许地面零点自动跟踪
 * 1 = 允许
 * 0 = 冻结
 */
static uint8_t GroundZeroEnable = 1;

/*
 * 原始高度
 */
static float HeightRaw = 0.0f;

/*
 * 滤波后的高度
 */
static float HeightFiltered = 0.0f;

/*
 * 上一次最终高度
 * 用于死区判断
 */
static float HeightOutput = 0.0f;

/*
 * 更新时间计数
 */
static float UpdateTimer = 0.0f;

/*
 * 限幅
 */
static float Barometer_LimitFloat(float Value, float Minimum, float Maximum)
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
 * 气压 → 相对高度
 * 当前GroundPressure对应0m。
 */
static float Barometer_PressureToHeight(float Pressure)
{
	float Ratio;
	float Height;

	if (Pressure <= 0.0f || GroundPressure <= 0.0f)
	{
		return 0.0f;
	}
	Ratio = Pressure / GroundPressure;
	Height = 44330.0f * (1.0f - powf(Ratio, 0.19029495f));
	Height = Barometer_LimitFloat(Height, BARO_HEIGHT_MIN, BARO_HEIGHT_MAX);
	return Height;
}

/*
 * 初始化
 */
uint8_t Barometer_Init(void)
{
	uint16_t i;
	int32_t Temperature;
	uint32_t Pressure;
	float PressureSum;
	uint16_t ValidCount;

	BarometerOK = 0;
	PressureFiltered = 0.0f;
	GroundPressure = 101325.0f;
	GroundZeroEnable = 1;
	HeightRaw = 0.0f;
	HeightFiltered = 0.0f;
	HeightOutput = 0.0f;
	UpdateTimer = 0.0f;

	/*
     * 1. BMP280初始化
 */
	if (BMP280_Init() == 0)
	{
		return 0;
	}

	/*
     * 2. ★预热3秒★
     * 这一步很重要。
     * 不要BMP刚启动，
     * 马上把那一刻的气压定义成地面零点。
 */
	Delay_ms(BARO_WARMUP_TIME_MS);

	/*
     * 3. 丢弃前50帧
 */
	for (i = 0; i < BARO_CAL_DISCARD_COUNT; i++)
	{
		BMP280_GetData(&Temperature, &Pressure);
		Delay_ms(20);
	}

	/*
     * 4. 平均200帧建立地面气压
     * 飞机必须：
     * 静止
     * 平放
     * 电机不转
 */
	PressureSum = 0.0f;
	ValidCount = 0;
	for (i = 0; i < BARO_CAL_SAMPLE_COUNT; i++)
	{
		if (BMP280_GetData(&Temperature, &Pressure))
		{
			/*
             * 简单合理范围判断
 */
			if (Pressure > 80000 && Pressure < 120000)
			{
				PressureSum += (float)Pressure;
				ValidCount++;
			}
		}
		Delay_ms(20);
	}

	/*
     * 有效数据太少
 */
	if (ValidCount < (BARO_CAL_SAMPLE_COUNT * 8 / 10))
	{
		return 0;
	}

	/*
     * 5. 地面零点
 */
	GroundPressure = PressureSum / (float)ValidCount;

	/*
     * 初始滤波值直接设成地面气压
     * 不要从0慢慢爬。
 */
	PressureFiltered = GroundPressure;
	HeightRaw = 0.0f;
	HeightFiltered = 0.0f;
	HeightOutput = 0.0f;
	BarometerOK = 1;
	return 1;
}

/*
 * 更新
 */
/*
 * 地面零点自动跟踪开关
 */
void Barometer_GroundZeroUpdate(uint8_t Enable)
{
	GroundZeroEnable = Enable;
}

void Barometer_Update(float dt)
{
	int32_t Temperature;
	uint32_t Pressure;
	float HeightDifference;

	if (BarometerOK == 0)
	{
		return;
	}

	/*
     * dt保护
 */
	if (dt < 0.0005f || dt > 0.05f)
	{
		dt = 0.005f;
	}
	UpdateTimer += dt;

	/*
     * 20Hz读取BMP280
 */
	if (UpdateTimer < BARO_UPDATE_PERIOD)
	{
		return;
	}
	UpdateTimer = 0.0f;

	/*
     * 读取气压
 */
	if (BMP280_GetData(&Temperature, &Pressure) == 0)
	{
		return;
	}

	/*
     * 异常气压直接丢弃
 */
	if (Pressure < 80000 || Pressure > 120000)
	{
		return;
	}

	/*
     * ★第一层：
     * 气压LPF
     *======================================================*/
	PressureFiltered =
		BARO_PRESSURE_LPF_OLD * PressureFiltered + BARO_PRESSURE_LPF_NEW * (float)Pressure;

	/*
		 * ★地面零点慢速自动修正★
		 * 只有飞机还没有启动时才允许。
		 * 例如：
		 * 当前GroundPressure = 100209
		 * 当前Pressure        = 100214
		 * 它不会一下跳到100214，
		 * 而是慢慢靠近。
		 * 最终BH自然回到0附近。
		 */
	if (GroundZeroEnable)
	{
		GroundPressure = GroundPressure * (1.0f - BARO_GROUND_ZERO_ALPHA)
						 + PressureFiltered * BARO_GROUND_ZERO_ALPHA;
	}

	/*
     * 根据滤波后的气压算高度
 */
	HeightRaw = Barometer_PressureToHeight(PressureFiltered);

	/*
     * ★第二层：
     * 高度LPF
 */
	HeightFiltered = BARO_HEIGHT_LPF_OLD * HeightFiltered + BARO_HEIGHT_LPF_NEW * HeightRaw;

	/*
     * ★第三层：
     * 高度死区
     * 如果新的高度与当前输出相差不到8cm，
     * 就认为它只是气压噪声。
 */
	HeightDifference = HeightFiltered - HeightOutput;
	if (fabsf(HeightDifference) >= BARO_HEIGHT_DEADBAND)
	{
		HeightOutput = HeightFiltered;
	}
}

/*
 * 当前气压
 * Pa
 */
uint32_t Barometer_GetPressure(void)
{
	return (uint32_t)(PressureFiltered + 0.5f);
}

/*
 * 当前高度
 * m
 */
float Barometer_GetHeight(void)
{
	return HeightOutput;
}

/*
 * 状态
 */
/*
 * Continuous filtered height for control loops
 * Unlike Barometer_GetHeight(), this value has no 8-cm
 * display deadband and is therefore suitable for PID use.
 * Unit: m
 *==========================================================*/
float Barometer_GetControlHeight(void)
{
	return HeightFiltered;
}

uint8_t Barometer_IsOK(void)
{
	return BarometerOK;
}
