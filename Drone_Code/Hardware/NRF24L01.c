/**
 * @file NRF24L01.c
 * @brief NRF24L01接收模式驱动和软件SPI时序。
 * @note  驱动采用GPIO轮询，不在中断中访问；接收到新包后立即读取并清空FIFO。
 */
#include "stm32f10x.h"

#include "NRF24L01.h"

/*
 * NRF24L01命令
 */
#define NRF_R_REGISTER	 0x00
#define NRF_W_REGISTER	 0x20
#define NRF_R_RX_PAYLOAD 0x61
#define NRF_FLUSH_RX	 0xE2
#define NRF_NOP			 0xFF

/*
 * NRF24L01寄存器
 */
#define NRF_CONFIG	   0x00
#define NRF_EN_AA	   0x01
#define NRF_EN_RXADDR  0x02
#define NRF_SETUP_AW   0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH	   0x05
#define NRF_RF_SETUP   0x06
#define NRF_STATUS	   0x07
#define NRF_RX_ADDR_P0 0x0A
#define NRF_RX_PW_P0   0x11

/*
 * 遥控器和飞机共同的无线参数
 * 必须和遥控器一致
 */
/*
 * 江协遥控器代码当前地址：
 * 11 22 33 44 55
 */
static uint8_t NRF_RxAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

/*
 * 接收缓冲
 */
uint8_t NRF24L01_RxPacket[NRF24L01_PACKET_SIZE];

/*
 * 飞机原理图引脚
 * CSN  PB12
 * SCK  PB13
 * MISO PB14
 * MOSI PB15
 * CE   PA8
 * IRQ  PA11
 * 当前使用轮询方式，
 * IRQ暂时不参与接收逻辑。
 */
static void NRF_W_CE(uint8_t Value)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)Value);
}

static void NRF_W_CSN(uint8_t Value)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)Value);
}

static void NRF_W_SCK(uint8_t Value)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)Value);
}

static void NRF_W_MOSI(uint8_t Value)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)Value);
}

static uint8_t NRF_R_MISO(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
}

/*
 * GPIO
 * 当前沿用江协的“软件SPI”思路。
 * 优点：
 * 不需要配置SPI2外设，
 * 逻辑简单，
 * 和遥控器底层一致。
 */
static void NRF_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	/*
     * PA8 = CE
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
     * PA11 = IRQ
     * 当前暂时不使用中断，
     * 配成上拉输入即可。
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
     * PB12 = CSN
     * PB13 = SCK
     * PB15 = MOSI
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*
     * PB14 = MISO
 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*
     * 默认电平
 */
	NRF_W_CE(0);
	NRF_W_CSN(1);
	NRF_W_SCK(0);
	NRF_W_MOSI(0);
}

/*
 * 软件SPI
 * Mode 0
 * MSB First
 */
static uint8_t NRF_SPI_SwapByte(uint8_t Byte)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		if (Byte & 0x80)
		{
			NRF_W_MOSI(1);
		}
		else
		{
			NRF_W_MOSI(0);
		}
		Byte <<= 1;
		NRF_W_SCK(1);
		if (NRF_R_MISO())
		{
			Byte |= 0x01;
		}
		NRF_W_SCK(0);
	}
	return Byte;
}

/*
 * 读单寄存器
 */
static uint8_t NRF_ReadReg(uint8_t Reg)
{
	uint8_t Data;

	NRF_W_CSN(0);
	NRF_SPI_SwapByte(NRF_R_REGISTER | Reg);
	Data = NRF_SPI_SwapByte(NRF_NOP);
	NRF_W_CSN(1);
	return Data;
}

/*
 * 写单寄存器
 */
static void NRF_WriteReg(uint8_t Reg, uint8_t Data)
{
	NRF_W_CSN(0);
	NRF_SPI_SwapByte(NRF_W_REGISTER | Reg);
	NRF_SPI_SwapByte(Data);
	NRF_W_CSN(1);
}

/*
 * 写多个寄存器
 */
static void NRF_WriteRegs(uint8_t Reg, uint8_t *Data, uint8_t Count)
{
	uint8_t i;

	NRF_W_CSN(0);
	NRF_SPI_SwapByte(NRF_W_REGISTER | Reg);
	for (i = 0; i < Count; i++)
	{
		NRF_SPI_SwapByte(Data[i]);
	}
	NRF_W_CSN(1);
}

/*
 * 读取RX Payload
 */
static void NRF_ReadPayload(uint8_t *Data, uint8_t Count)
{
	uint8_t i;

	NRF_W_CSN(0);
	NRF_SPI_SwapByte(NRF_R_RX_PAYLOAD);
	for (i = 0; i < Count; i++)
	{
		Data[i] = NRF_SPI_SwapByte(NRF_NOP);
	}
	NRF_W_CSN(1);
}

/*
 * 清RX FIFO
 */
static void NRF_FlushRx(void)
{
	NRF_W_CSN(0);
	NRF_SPI_SwapByte(NRF_FLUSH_RX);
	NRF_W_CSN(1);
}

/*
 * STATUS
 */
uint8_t NRF24L01_GetStatus(void)
{
	uint8_t Status;

	NRF_W_CSN(0);
	Status = NRF_SPI_SwapByte(NRF_NOP);
	NRF_W_CSN(1);
	return Status;
}

/*
 * 初始化
 * 参数严格和遥控器保持一致：
 * 地址：11 22 33 44 55
 * Payload：32 bytes
 * RF_CH = 2
 * -> 2402MHz
 * RF_SETUP = 0x0E
 * -> 2Mbps
 * 自动应答开启
 */
void NRF24L01_Init(void)
{
	NRF_GPIO_Init();

	/*
     * 先退出收发模式
 */
	NRF_W_CE(0);

	/*
     * EN_CRC = 1
     * 1-byte CRC
     * PWR_UP = 0
     * PRIM_RX = 0
 */
	NRF_WriteReg(NRF_CONFIG, 0x08);

	/*
     * Pipe0自动应答
 */
	NRF_WriteReg(NRF_EN_AA, 0x01);

	/*
     * 只开Pipe0
 */
	NRF_WriteReg(NRF_EN_RXADDR, 0x01);

	/*
     * 5字节地址
 */
	NRF_WriteReg(NRF_SETUP_AW, 0x03);

	/*
     * 接收端本身不用自动重发，
     * 但配置保持正常。
 */
	NRF_WriteReg(NRF_SETUP_RETR, 0x03);

	/*
     * 2402MHz
 */
	NRF_WriteReg(NRF_RF_CH, 0x02);

	/*
     * 2Mbps
     * 0dBm
     * 必须和遥控器一致
 */
	NRF_WriteReg(NRF_RF_SETUP, 0x0E);

	/*
     * Pipe0地址
 */
	NRF_WriteRegs(NRF_RX_ADDR_P0, NRF_RxAddress, 5);

	/*
     * 固定32字节包
 */
	NRF_WriteReg(NRF_RX_PW_P0, NRF24L01_PACKET_SIZE);
	NRF_FlushRx();

	/*
     * 清：
     * RX_DR
     * TX_DS
     * MAX_RT
 */
	NRF_WriteReg(NRF_STATUS, 0x70);

	/*
     * 进入RX模式
     * CONFIG：
     * EN_CRC = 1
     * CRC = 1 byte
     * PWR_UP = 1
     * PRIM_RX = 1
     * = 0x0B
 */
	NRF_WriteReg(NRF_CONFIG, 0x0B);

	/*
     * CE = 1
     * 持续监听无线数据
 */
	NRF_W_CE(1);
}

/*
 * 接收
 */
uint8_t NRF24L01_Receive(void)
{
	uint8_t Status;
	uint8_t Config;

	Status = NRF24L01_GetStatus();
	Config = NRF_ReadReg(NRF_CONFIG);

	/*
     * 读到0xFF一般说明模块没接好
 */
	if (Status == 0xFF || Config == 0xFF)
	{
		return 2;
	}

	/*
     * RX_DR
     * bit6 = 1
     * 收到新数据
 */
	if (Status & 0x40)
	{
		NRF_ReadPayload(NRF24L01_RxPacket, NRF24L01_PACKET_SIZE);

		/*
         * 清RX_DR
 */
		NRF_WriteReg(NRF_STATUS, 0x40);

		/*
         * 我们只需要最新遥控命令。
         * 旧FIFO不要积压。
 */
		NRF_FlushRx();
		return 1;
	}
	return 0;
}
