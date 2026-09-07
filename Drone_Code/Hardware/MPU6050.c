/**
 * @file MPU6050.c
 * @brief MPU6050初始化、I2C2突发读取和通信超时保护。
 * @note  读取失败时保留最近一次有效数据，避免异常总线值直接进入姿态算法。
 */
#include "stm32f10x.h"

#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "Delay.h"

/*
 * STM32标准库 I2C_Send7bitAddress()
 * 这里使用左移后的地址。
 * MPU6050：
 * AD0 = 0
 * 7bit地址 = 0x68
 * 左移1位   = 0xD0
 */
#define MPU6050_ADDRESS 0xD0

/*
 * I2C超时计数
 * 防止I2C异常以后程序永久卡死
 */
#define MPU6050_I2C_TIMEOUT 50000

/*
 * 最近一次有效数据
 * 如果偶尔出现一次I2C读取失败，
 * 暂时沿用上一帧数据，
 * 防止输出未初始化的随机值。
 */
static int16_t LastAccX = 0;
static int16_t LastAccY = 0;
static int16_t LastAccZ = 2048;
static int16_t LastGyroX = 0;
static int16_t LastGyroY = 0;
static int16_t LastGyroZ = 0;

/*
 * 等待I2C事件
 * 成功：
 * return 1
 * 超时：
 * return 0
 */
static uint8_t MPU6050_WaitEvent(uint32_t Event)
{
	uint32_t Timeout;

	Timeout = MPU6050_I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C2, Event) != SUCCESS)
	{
		if (Timeout == 0)
		{
			/*
             * 尝试终止当前通信
 */
			I2C_GenerateSTOP(I2C2, ENABLE);

			/*
             * 恢复ACK
 */
			I2C_AcknowledgeConfig(I2C2, ENABLE);
			return 0;
		}
		Timeout--;
	}
	return 1;
}

/*
 * 等待指定I2C FLAG置位
 * Burst Read最后几个字节需要直接检测
 * RXNE / BTF等状态。
 */
static uint8_t MPU6050_WaitFlag(uint32_t Flag)
{
	uint32_t Timeout;

	Timeout = MPU6050_I2C_TIMEOUT;
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
 * 指定地址写一个寄存器
 */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	/*
     * START
 */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0)
	{
		return;
	}

	/*
     * MPU6050地址 + 写
 */
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0)
	{
		return;
	}

	/*
     * 寄存器地址
 */
	I2C_SendData(I2C2, RegAddress);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0)
	{
		return;
	}

	/*
     * 数据
 */
	I2C_SendData(I2C2, Data);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0)
	{
		return;
	}

	/*
     * STOP
 */
	I2C_GenerateSTOP(I2C2, ENABLE);
}

/*
 * 单寄存器读取
 * 主要留给：
 * WHO_AM_I
 * 等初始化/调试用途。
 * 飞控六轴数据不再使用它一字节一字节读取。
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;

	Data = 0;

	/*
     * 下一次接收使用默认NACK位置
 */
	I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);

	/*
     * 单字节读取：
     * 必须提前关闭ACK。
     * STM32F1硬件I2C对1字节接收
     * 时序要求比较严格。
 */
	I2C_AcknowledgeConfig(I2C2, ENABLE);

	/*
     * START
 */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0)
	{
		return 0;
	}

	/*
     * 地址 + 写
 */
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0)
	{
		return 0;
	}

	/*
     * 指定寄存器
 */
	I2C_SendData(I2C2, RegAddress);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0)
	{
		return 0;
	}

	/*
     * Re-START
 */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0)
	{
		return 0;
	}

	/*
     * 单字节接收的关键：
     * 在ADDR被清除以前关闭ACK
 */
	I2C_AcknowledgeConfig(I2C2, DISABLE);

	/*
     * 地址 + 读
 */
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);

	/*
     * 等待ADDR置位
     * 这里不能使用
     * I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED
     * 因为它会读取SR1/SR2，
     * 提前清除ADDR。
 */
	if (MPU6050_WaitFlag(I2C_FLAG_ADDR) == 0)
	{
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		return 0;
	}

	/*
     * 清除ADDR：
     * 读取SR1
     * 再读取SR2
 */
	(void)I2C2->SR1;
	(void)I2C2->SR2;

	/*
     * 只接收1字节，
     * ADDR清除以后立刻STOP
 */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/*
     * 等待RXNE
 */
	if (MPU6050_WaitFlag(I2C_FLAG_RXNE) == 0)
	{
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		return 0;
	}
	Data = I2C_ReceiveData(I2C2);

	/*
     * 恢复ACK，
     * 准备下次通信
 */
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return Data;
}

/*
 * 连续读取多个字节
 * 这是飞控现在最重要的修改。
 * MPU6050寄存器支持地址自动递增。
 * 我们会：
 * 0x3B开始
 * 一次连续读取14字节
 * 顺序：
 * 0  ACCEL_X_H
 * 1  ACCEL_X_L
 * 2  ACCEL_Y_H
 * 3  ACCEL_Y_L
 * 4  ACCEL_Z_H
 * 5  ACCEL_Z_L
 * 6  TEMP_H
 * 7  TEMP_L
 * 8  GYRO_X_H
 * 9  GYRO_X_L
 * 10 GYRO_Y_H
 * 11 GYRO_Y_L
 * 12 GYRO_Z_H
 * 13 GYRO_Z_L
 * 注意：
 * 当前函数按照 STM32F1
 * N>2字节接收流程处理最后3个字节。
 * 成功return 1
 * 失败return 0
 */
static uint8_t MPU6050_ReadBurst14(uint8_t *Buffer)
{
	uint8_t Index;
	uint8_t Remaining;

	Index = 0;
	Remaining = 14;

	/*
     * 默认NACK位置
 */
	I2C_NACKPositionConfig(I2C2, I2C_NACKPosition_Current);

	/*
     * 多字节接收开始时开启ACK
 */
	I2C_AcknowledgeConfig(I2C2, ENABLE);

	/*
     * 1. START
 */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0)
	{
		return 0;
	}

	/*
     * 2. 地址 + 写
 */
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0)
	{
		return 0;
	}

	/*
     * 3. 起始寄存器
     * ACCEL_XOUT_H = 0x3B
 */
	I2C_SendData(I2C2, MPU6050_ACCEL_XOUT_H);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0)
	{
		return 0;
	}

	/*
     * 4. Re-START
 */
	I2C_GenerateSTART(I2C2, ENABLE);
	if (MPU6050_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) == 0)
	{
		return 0;
	}

	/*
     * 5. 地址 + 读
 */
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);

	/*
     * 等待ADDR
 */
	if (MPU6050_WaitFlag(I2C_FLAG_ADDR) == 0)
	{
		return 0;
	}

	/*
     * 多字节：
     * 此时ACK保持开启。
     * 读取SR1/SR2清除ADDR
 */
	(void)I2C2->SR1;
	(void)I2C2->SR2;

	/*
     * 6. 接收前11字节
     * 一直收到只剩3字节。
 */
	while (Remaining > 3)
	{
		if (MPU6050_WaitFlag(I2C_FLAG_RXNE) == 0)
		{
			I2C_AcknowledgeConfig(I2C2, ENABLE);
			return 0;
		}
		Buffer[Index] = I2C_ReceiveData(I2C2);
		Index++;
		Remaining--;
	}

	/*
     * 现在只剩3字节
     * STM32F1必须按照特殊流程处理。
 */
	/*
     * 等待：
     * DR + Shift Register
     * 都已经有数据
 */
	if (MPU6050_WaitFlag(I2C_FLAG_BTF) == 0)
	{
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		return 0;
	}

	/*
     * 剩3字节时关闭ACK
 */
	I2C_AcknowledgeConfig(I2C2, DISABLE);

	/*
     * 读取倒数第3个字节
 */
	Buffer[Index] = I2C_ReceiveData(I2C2);
	Index++;
	Remaining--;

	/*
     * 此时剩2字节
     * 等待最后两个字节全部到达
 */
	if (MPU6050_WaitFlag(I2C_FLAG_BTF) == 0)
	{
		I2C_AcknowledgeConfig(I2C2, ENABLE);
		return 0;
	}

	/*
     * 最后两个字节已经进入：
     * DR
     * Shift Register
     * 此时发送STOP
 */
	I2C_GenerateSTOP(I2C2, ENABLE);

	/*
     * 读取倒数第2个
 */
	Buffer[Index] = I2C_ReceiveData(I2C2);
	Index++;
	Remaining--;

	/*
     * 读取最后1个
 */
	Buffer[Index] = I2C_ReceiveData(I2C2);
	Index++;
	Remaining--;

	/*
     * 恢复ACK
 */
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return 1;
}

/*
 * MPU6050初始化
 */
void MPU6050_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/*
     * 1. 开启时钟
 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/*
     * 2. GPIO
     * PB10 = I2C2_SCL
     * PB11 = I2C2_SDA
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*
     * 3. I2C2重新初始化
 */
	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;

	/*
     * ★目前先使用100kHz★
     * 你的板子是自制板，
     * 现在优先保证稳定。
     * Burst Read以后，
     * 即使100kHz也已经够用了。
     * 等以后稳定以后可以尝试：
     * 400000
 */
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

	/*
     * 4. MPU6050软件复位
 */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
	Delay_ms(100);

	/*
     * 5. 唤醒
     * CLKSEL = 1
     * 使用X轴陀螺仪PLL
 */
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	Delay_ms(10);

	/*
     * 6. Sample Rate
     * DLPF开启：
     * Gyro输出 = 1kHz
     * Sample Rate =
     * 1000 / (1 + DIV)
     * DIV = 4
     * = 200Hz
 */
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);

	/*
     * 7. DLPF
     * DLPF_CFG = 3
     * Gyro约44Hz
     * Acc约44Hz
     * 对小四轴作为第一版
     * 比0x06更合适。
 */
	MPU6050_WriteReg(MPU6050_CONFIG, 0x04);

	/*
     * 8. Gyro
     * ±2000 °/s
     * 16.4 LSB/(°/s)
     * 对应Attitude.c：
     * GYRO_SCALE = 16.4
 */
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);

	/*
     * 9. Accelerometer
     * ±16g
     * 2048 LSB/g
     * 对应Attitude.c：
     * ACC_SCALE = 2048
 */
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
	Delay_ms(20);
}

/*
 * 获取WHO_AM_I
 */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/*
 * 获取六轴数据
 * ★现在改为14字节连续读取★
 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, int16_t *GyroX, int16_t *GyroY,
					 int16_t *GyroZ)
{
	uint8_t Data[14];

	/*
     * 一次读取：
     * 0x3B ~ 0x48
     * 共14字节。
 */
	if (MPU6050_ReadBurst14(Data) == 1)
	{
		/*
         * Accelerometer X
 */
		LastAccX = (int16_t)(((uint16_t)Data[0] << 8) | Data[1]);

		/*
         * Accelerometer Y
 */
		LastAccY = (int16_t)(((uint16_t)Data[2] << 8) | Data[3]);

		/*
         * Accelerometer Z
 */
		LastAccZ = (int16_t)(((uint16_t)Data[4] << 8) | Data[5]);

		/*
         * Data[6], Data[7]
         * 是温度，
         * 当前飞控不用。
 */
		/*
         * Gyroscope X
 */
		LastGyroX = (int16_t)(((uint16_t)Data[8] << 8) | Data[9]);

		/*
         * Gyroscope Y
 */
		LastGyroY = (int16_t)(((uint16_t)Data[10] << 8) | Data[11]);

		/*
         * Gyroscope Z
 */
		LastGyroZ = (int16_t)(((uint16_t)Data[12] << 8) | Data[13]);
	}

	/*
     * 无论本次I2C是否成功，
     * 都给调用者一组确定的数据。
     * 如果偶发一次读取失败，
     * 就暂时使用上一帧。
 */
	*AccX = LastAccX;

	*AccY = LastAccY;

	*AccZ = LastAccZ;

	*GyroX = LastGyroX;

	*GyroY = LastGyroY;

	*GyroZ = LastGyroZ;
}
