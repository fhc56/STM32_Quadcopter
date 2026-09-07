/**
 * @file BMP280.c
 * @brief BMP280寄存器访问、标定参数读取和温压补偿。
 * @note  采用带超时的轮询I2C2；I2C2同时连接MPU6050，因此当前统一由
 *        飞控任务顺序访问，不允许并发事务。
 */
#include "stm32f10x.h"

#include "BMP280.h"

#include "Delay.h"

/*
 * BMP280
 * 7bit Address = 0x76
 * STM32 SPL使用左移后的：
 * 0xEC
 */
#define BMP280_ADDRESS 0xEC

/*
 * Register
 */
#define BMP280_REG_ID		   0xD0
#define BMP280_REG_RESET	   0xE0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG	   0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88

/*
 * I2C Timeout
 */
#define BMP280_I2C_TIMEOUT 50000UL

/*
 * Calibration
 */
typedef struct
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

} BMP280_Calib_t;

static BMP280_Calib_t Calib;
static int32_t T_Fine = 0;

/*
 * Wait Event
 */
static uint8_t BMP280_WaitEvent(uint32_t Event)
{
	uint32_t Timeout;

	Timeout = BMP280_I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C2, Event) != SUCCESS)
	{
		if (Timeout == 0)
		{
			I2C_GenerateSTOP(I2C2, ENABLE);
			I2C_AcknowledgeConfig(I2C2, ENABLE);
			return 0;
		}
		Timeout--;
	}
	return 1;
}

/*
 * Wait Flag
 */
static uint8_t BMP280_WaitFlag(uint32_t Flag)
{
	uint32_t Timeout;

	Timeout = BMP280_I2C_TIMEOUT;
	while (I2C_GetFlagStatus(I2C2, Flag) == RESET)
	{
		if (Timeout == 0)
		{
			I2C_GenerateSTOP(I2C2, ENABLE);
			I2C_AcknowledgeConfig(I2C2, ENABLE);
			return 0;
		}
		Timeout--;
	}
	return 1;
}

/*
 * Write Register
 */
static uint8_t BMP280_WriteReg(uint8_t Reg, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		return 0;
	}
	I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Transmitter);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		return 0;
	}
	I2C_SendData(I2C2, Reg);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		return 0;
	}
	I2C_SendData(I2C2, Data);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		return 0;
	}
	I2C_GenerateSTOP(I2C2, ENABLE);
	return 1;
}

/*
 * Read one byte
 * STM32F1单字节接收流程。
 */
static uint8_t BMP280_ReadReg(uint8_t Reg, uint8_t *Data)
{
	I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		return 0;
	}
	I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Transmitter);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		return 0;
	}
	I2C_SendData(I2C2, Reg);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		return 0;
	}
	I2C_GenerateSTART(I2C2, ENABLE);
	if (!BMP280_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT))
	{
		return 0;
	}

	/*
     * 单字节接收：
     * ADDR清除之前关ACK。
 */
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_Send7bitAddress(I2C2, BMP280_ADDRESS, I2C_Direction_Receiver);
	if (!BMP280_WaitFlag(I2C_FLAG_ADDR))
	{
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		return 0;
	}

	/*
     * Clear ADDR
 */
	(void)I2C2->SR1;
	(void)I2C2->SR2;
	I2C_GenerateSTOP(I2C2, ENABLE);
	if (!BMP280_WaitFlag(I2C_FLAG_RXNE))
	{
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		return 0;
	}

	*Data = I2C_ReceiveData(I2C2);

	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return 1;
}

/*
 * Read 16bit Little Endian
 */
static uint8_t BMP280_ReadU16LE(uint8_t Reg, uint16_t *Value)
{
	uint8_t Low;
	uint8_t High;

	if (!BMP280_ReadReg(Reg, &Low))
	{
		return 0;
	}
	if (!BMP280_ReadReg(Reg + 1, &High))
	{
		return 0;
	}

	*Value = ((uint16_t)High << 8) | Low;

	return 1;
}

/*
 * Read Calibration
 */
static uint8_t BMP280_ReadCalibration(void)
{
	uint16_t Temp;

	if (!BMP280_ReadU16LE(0x88, &Calib.dig_T1))
		return 0;
	if (!BMP280_ReadU16LE(0x8A, &Temp))
		return 0;
	Calib.dig_T2 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x8C, &Temp))
		return 0;
	Calib.dig_T3 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x8E, &Calib.dig_P1))
		return 0;
	if (!BMP280_ReadU16LE(0x90, &Temp))
		return 0;
	Calib.dig_P2 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x92, &Temp))
		return 0;
	Calib.dig_P3 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x94, &Temp))
		return 0;
	Calib.dig_P4 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x96, &Temp))
		return 0;
	Calib.dig_P5 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x98, &Temp))
		return 0;
	Calib.dig_P6 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x9A, &Temp))
		return 0;
	Calib.dig_P7 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x9C, &Temp))
		return 0;
	Calib.dig_P8 = (int16_t)Temp;
	if (!BMP280_ReadU16LE(0x9E, &Temp))
		return 0;
	Calib.dig_P9 = (int16_t)Temp;
	return 1;
}

/*
 * Temperature Compensation
 */
static int32_t BMP280_CompensateTemperature(int32_t ADC_T)
{
	int32_t Var1;
	int32_t Var2;
	int32_t Temperature;

	Var1 = (((ADC_T >> 3) - ((int32_t)Calib.dig_T1 << 1)) * (int32_t)Calib.dig_T2) >> 11;
	Var2 =
		(((((ADC_T >> 4) - (int32_t)Calib.dig_T1) * ((ADC_T >> 4) - (int32_t)Calib.dig_T1)) >> 12)
		 * (int32_t)Calib.dig_T3)
		>> 14;
	T_Fine = Var1 + Var2;
	Temperature = (T_Fine * 5 + 128) >> 8;
	return Temperature;
}

/*
 * Pressure Compensation
 */
static uint32_t BMP280_CompensatePressure(int32_t ADC_P)
{
	int64_t Var1;
	int64_t Var2;
	int64_t Pressure;

	Var1 = (int64_t)T_Fine - 128000;
	Var2 = Var1 * Var1 * (int64_t)Calib.dig_P6;
	Var2 += (Var1 * (int64_t)Calib.dig_P5) << 17;
	Var2 += ((int64_t)Calib.dig_P4) << 35;
	Var1 = ((Var1 * Var1 * (int64_t)Calib.dig_P3) >> 8) + ((Var1 * (int64_t)Calib.dig_P2) << 12);
	Var1 = (((((int64_t)1 << 47) + Var1) * (int64_t)Calib.dig_P1) >> 33);
	if (Var1 == 0)
	{
		return 0;
	}
	Pressure = 1048576 - ADC_P;
	Pressure = (((Pressure << 31) - Var2) * 3125) / Var1;
	Var1 = (((int64_t)Calib.dig_P9 * (Pressure >> 13) * (Pressure >> 13)) >> 25);
	Var2 = (((int64_t)Calib.dig_P8 * Pressure) >> 19);
	Pressure = ((Pressure + Var1 + Var2) >> 8) + ((int64_t)Calib.dig_P7 << 4);

	/*
     * Q24.8 -> Pa
 */
	return (uint32_t)(Pressure >> 8);
}

/*
 * ID
 */
uint8_t BMP280_GetID(void)
{
	uint8_t ID;

	ID = 0;
	BMP280_ReadReg(BMP280_REG_ID, &ID);
	return ID;
}

/*
 * Init
 * ★这里绝对不初始化I2C2★
 * I2C2由MPU6050初始化。
 */
uint8_t BMP280_Init(void)
{
	uint8_t ID;

	/*
     * 给BMP280一点上电时间
 */
	Delay_ms(20);
	ID = BMP280_GetID();
	if (ID != 0x58)
	{
		return 0;
	}

	/*
     * Soft Reset
 */
	if (!BMP280_WriteReg(BMP280_REG_RESET, 0xB6))
	{
		return 0;
	}
	Delay_ms(10);
	if (!BMP280_ReadCalibration())
	{
		return 0;
	}

	/*
     * ctrl_meas = 0x53
     * Temperature x2
     * Pressure    x8
     * Normal Mode
     * 定高比你以前x1更适合。
 */
	if (!BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x53))
	{
		return 0;
	}

	/*
     * config = 0x2C
     * standby约62.5ms
     * IIR filter coefficient 8
     * 对高度信号比完全追求速度重要。
 */
	if (!BMP280_WriteReg(BMP280_REG_CONFIG, 0x2C))
	{
		return 0;
	}
	Delay_ms(100);
	return 1;
}

/*
 * Data
 * 这里只读6个单字节。
 * 速度对于20Hz高度环完全够用，
 * 而且比在STM32F1上重新写复杂Burst
 * 更稳妥。
 */
uint8_t BMP280_GetData(int32_t *Temperature, uint32_t *Pressure)
{
	uint8_t Data[6];
	int32_t ADC_P;
	int32_t ADC_T;
	uint8_t i;

	for (i = 0; i < 6; i++)
	{
		if (!BMP280_ReadReg(BMP280_REG_PRESS_MSB + i, &Data[i]))
		{
			return 0;
		}
	}
	ADC_P = (((int32_t)Data[0] << 12) | ((int32_t)Data[1] << 4) | ((int32_t)Data[2] >> 4));
	ADC_T = (((int32_t)Data[3] << 12) | ((int32_t)Data[4] << 4) | ((int32_t)Data[5] >> 4));

	*Temperature = BMP280_CompensateTemperature(ADC_T);

	*Pressure = BMP280_CompensatePressure(ADC_P);

	if (*Pressure < 30000 || *Pressure > 120000)
	{
		return 0;
	}
	return 1;
}
