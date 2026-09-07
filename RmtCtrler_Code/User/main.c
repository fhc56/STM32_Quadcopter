#include "stm32f10x.h"

#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "AD.h"
#include "NRF24L01.h"
#include "Key.h"



/*==========================================================
 * 摇杆ADC
 *
 * PA0 = LH 左杆左右
 * PA1 = LV 左杆上下
 * PA2 = RH 右杆左右
 * PA3 = RV 右杆上下
 *==========================================================*/

#define ADC_LH_CHANNEL          ADC_Channel_0
#define ADC_LV_CHANNEL          ADC_Channel_1
#define ADC_RH_CHANNEL          ADC_Channel_2
#define ADC_RV_CHANNEL          ADC_Channel_3



#define STICK_CENTER            2048

#define LEFT_STICK_DEADZONE     100
#define RIGHT_STICK_DEADZONE    100



/*==========================================================
 * 方向
 *
 * 左杆往上应该：
 * VC > 0
 *
 * 左杆往下应该：
 * VC < 0
 *
 * 如果实测刚好反了，
 * 只把 VERTICAL_REVERSE 改成1。
 *==========================================================*/

#define VERTICAL_REVERSE        0

#define FB_REVERSE              0

#define LR_REVERSE              0



/*==========================================================
 * 调度
 *==========================================================*/

#define ADC_PERIOD_MS           5
#define RADIO_PERIOD_MS         10
#define OLED_PERIOD_MS          40



/*==========================================================
 * 无线协议
 *
 * Byte0 = 0xA5
 *
 * Byte1 = VC
 *         int8_t
 *         -100 ~ +100
 *
 * Byte2 = FB
 *
 * Byte3 = LR
 *
 * Byte4 = Sequence
 *
 * Byte5 = Checksum
 *
 * Byte6 = SafetyCommand
 *
 *         0 = 普通控制
 *         1 = K2急停
 *         2 = K1解锁
 *
 * Byte7 = 协议版本 0xC3
 *==========================================================*/

#define PACKET_HEADER           0xA5

#define PROTOCOL_VERSION        0xC3

#define CHECK_BASE              0x5A



#define SAFE_NONE               0
#define SAFE_ESTOP              1
#define SAFE_ARM                2



/*==========================================================
 * 定时标志
 *==========================================================*/

volatile uint8_t ADCUpdateFlag = 0;

volatile uint8_t RadioUpdateFlag = 0;

volatile uint8_t OLEDUpdateFlag = 0;



/*==========================================================
 * ADC原始值
 *==========================================================*/

static uint16_t AD_LH = 0;

static uint16_t AD_LV = 0;

static uint16_t AD_RH = 0;

static uint16_t AD_RV = 0;



/*==========================================================
 * 控制量
 *==========================================================*/

/*
 * 左杆上下
 *
 * +100 = 上升
 *    0 = 定高
 * -100 = 下降
 */
static int8_t VerticalCommand = 0;



static int8_t ForwardBack = 0;

static int8_t LeftRight = 0;



/*==========================================================
 * NRF
 *==========================================================*/

static uint8_t Sequence = 0;

static uint8_t NRFSendResult = 0;



/*==========================================================
 * 安全锁
 *
 * ★上电默认锁死★
 *
 * 必须：
 *
 * 左杆回中
 * +
 * 按K1
 *
 * 才允许起飞。
 *==========================================================*/

static uint8_t EmergencyStop = 1;



/*
 * K1解锁以后，
 * 连续20个无线包发送ARM。
 *
 * 100Hz下约200ms。
 */
static uint8_t ArmPulseCount = 0;



static uint8_t LastK1State = 0;



/*==========================================================
 * ADC读取
 *==========================================================*/

static uint16_t Remote_ReadADC(
    uint8_t Channel
)
{
    uint32_t Sum;



    Sum =
        AD_GetValue(
            Channel
        );


    Sum +=
        AD_GetValue(
            Channel
        );



    return
        (uint16_t)(
            Sum / 2
        );
}



/*==========================================================
 * 中心回弹摇杆
 *
 * 输出：
 *
 * -100 ~ +100
 *==========================================================*/

static int8_t Remote_ProcessCenterStick(
    uint16_t ADValue,
    uint8_t Reverse,
    uint16_t Deadzone
)
{
    int32_t Value;

    int32_t Output;



    Value =
        (int32_t)ADValue
        -
        STICK_CENTER;



    if (Reverse)
    {
        Value =
            -Value;
    }



    /*======================================================
     * 中心死区
     *======================================================*/

    if (
        Value >=
        -(int32_t)Deadzone

        &&

        Value <=
        (int32_t)Deadzone
    )
    {
        return 0;
    }



    /*======================================================
     * 正方向
     *======================================================*/

    if (Value > 0)
    {
        Value -=
            Deadzone;



        Output =
            Value
            *
            100
            /
            (
                2047
                -
                Deadzone
            );
    }



    /*======================================================
     * 负方向
     *======================================================*/

    else
    {
        Value +=
            Deadzone;



        Output =
            Value
            *
            100
            /
            (
                2048
                -
                Deadzone
            );
    }



    if (Output > 100)
    {
        Output =
            100;
    }



    if (Output < -100)
    {
        Output =
            -100;
    }



    return
        (int8_t)Output;
}



/*==========================================================
 * 摇杆更新
 *==========================================================*/

static void Remote_UpdateSticks(void)
{
    AD_LH =
        Remote_ReadADC(
            ADC_LH_CHANNEL
        );


    AD_LV =
        Remote_ReadADC(
            ADC_LV_CHANNEL
        );


    AD_RH =
        Remote_ReadADC(
            ADC_RH_CHANNEL
        );


    AD_RV =
        Remote_ReadADC(
            ADC_RV_CHANNEL
        );



    /*======================================================
     * 左杆上下
     *======================================================*/

    VerticalCommand =
        Remote_ProcessCenterStick(
            AD_LV,
            VERTICAL_REVERSE,
            LEFT_STICK_DEADZONE
        );



    /*======================================================
     * 右杆前后
     *======================================================*/

    ForwardBack =
        Remote_ProcessCenterStick(
            AD_RV,
            FB_REVERSE,
            RIGHT_STICK_DEADZONE
        );



    /*======================================================
     * 右杆左右
     *======================================================*/

    LeftRight =
        Remote_ProcessCenterStick(
            AD_RH,
            LR_REVERSE,
            RIGHT_STICK_DEADZONE
        );
}



/*==========================================================
 * ★K1 / K2★
 *
 * K1 = PA6
 *
 * K2 = PA7
 *==========================================================*/

static void Remote_UpdateSafetyKeys(void)
{
    uint8_t K1State;

    uint8_t K2State;



    /*
     * 上拉输入：
     *
     * 按下 = 0
     */

    K1State =
        (
            GPIO_ReadInputDataBit(
                GPIOA,
                GPIO_Pin_6
            )
            ==
            Bit_RESET
        )
        ?
        1
        :
        0;



    K2State =
        (
            GPIO_ReadInputDataBit(
                GPIOA,
                GPIO_Pin_7
            )
            ==
            Bit_RESET
        )
        ?
        1
        :
        0;



    /*======================================================
     * ★K2最高优先级★
     *
     * 只要K2按住：
     *
     * 就一直发急停。
     *======================================================*/

    if (K2State)
    {
        EmergencyStop =
            1;


        ArmPulseCount =
            0;


        LastK1State =
            K1State;


        return;
    }



    /*======================================================
     * K1按下瞬间
     *
     * 必须：
     *
     * STOP=1
     * &&
     * VC=0
     *
     * 才允许重新解锁。
     *======================================================*/

    if (
        K1State
        &&
        !LastK1State
    )
    {
        if (
            EmergencyStop
            &&
            VerticalCommand == 0
        )
        {
            EmergencyStop =
                0;



            ArmPulseCount =
                20;
        }
    }



    LastK1State =
        K1State;
}



/*==========================================================
 * Checksum
 *==========================================================*/

static uint8_t Remote_Checksum(void)
{
    uint8_t Check;



    Check =
        CHECK_BASE;



    Check ^=
        NRF24L01_TxPacket[0];


    Check ^=
        NRF24L01_TxPacket[1];


    Check ^=
        NRF24L01_TxPacket[2];


    Check ^=
        NRF24L01_TxPacket[3];


    Check ^=
        NRF24L01_TxPacket[4];


    Check ^=
        NRF24L01_TxPacket[6];


    Check ^=
        NRF24L01_TxPacket[7];



    return Check;
}



/*==========================================================
 * 无线数据包
 *==========================================================*/

static void Remote_PreparePacket(void)
{
    uint8_t i;

    uint8_t SafetyCommand;



    for (i = 0; i < 32; i++)
    {
        NRF24L01_TxPacket[i] =
            0;
    }



    /*======================================================
     * 安全命令
     *======================================================*/

    if (EmergencyStop)
    {
        SafetyCommand =
            SAFE_ESTOP;
    }

    else if (
        ArmPulseCount >
        0
    )
    {
        SafetyCommand =
            SAFE_ARM;


        ArmPulseCount--;
    }

    else
    {
        SafetyCommand =
            SAFE_NONE;
    }



    NRF24L01_TxPacket[0] =
        PACKET_HEADER;



    NRF24L01_TxPacket[1] =
        (uint8_t)
        VerticalCommand;



    NRF24L01_TxPacket[2] =
        (uint8_t)
        ForwardBack;



    NRF24L01_TxPacket[3] =
        (uint8_t)
        LeftRight;



    NRF24L01_TxPacket[4] =
        Sequence;



    NRF24L01_TxPacket[6] =
        SafetyCommand;



    NRF24L01_TxPacket[7] =
        PROTOCOL_VERSION;



    Sequence++;



    NRF24L01_TxPacket[5] =
        Remote_Checksum();
}



/*==========================================================
 * OLED
 *==========================================================*/

static void Remote_UpdateOLED(void)
{
    OLED_Clear();



    OLED_Printf(
        0,
        0,
        OLED_8X16,

        "VC :%+4d",

        VerticalCommand
    );



    OLED_Printf(
        0,
        16,
        OLED_8X16,

        "FB :%+4d",

        ForwardBack
    );



    OLED_Printf(
        0,
        32,
        OLED_8X16,

        "LR :%+4d",

        LeftRight
    );



    OLED_Printf(
        0,
        48,
        OLED_6X8,

        "STOP:%d RF:%d",

        EmergencyStop,

        NRFSendResult
    );



    OLED_Update();
}



/*==========================================================
 * main
 *==========================================================*/

int main(void)
{
    OLED_Init();


    AD_Init();


    Key_Init();


    NRF24L01_Init();


    Timer_Init();



    Remote_UpdateSticks();


    Remote_UpdateSafetyKeys();


    Remote_UpdateOLED();



    while (1)
    {
        /*==================================================
         * 200Hz摇杆
         *==================================================*/

        if (ADCUpdateFlag)
        {
            ADCUpdateFlag =
                0;



            Remote_UpdateSticks();



            Remote_UpdateSafetyKeys();
        }



        /*==================================================
         * 100Hz无线
         *==================================================*/

        if (RadioUpdateFlag)
        {
            RadioUpdateFlag =
                0;



            Remote_PreparePacket();



            NRFSendResult =
                NRF24L01_Send();
        }



        /*==================================================
         * 25Hz OLED
         *==================================================*/

        if (OLEDUpdateFlag)
        {
            OLEDUpdateFlag =
                0;



            Remote_UpdateOLED();
        }
    }
}



/*==========================================================
 * TIM1 1ms
 *==========================================================*/

void TIM1_UP_IRQHandler(void)
{
    static uint8_t ADCCount = 0;

    static uint8_t RadioCount = 0;

    static uint8_t OLEDCount = 0;



    if (
        TIM_GetITStatus(
            TIM1,
            TIM_IT_Update
        )
        ==
        SET
    )
    {
        ADCCount++;


        if (
            ADCCount >=
            ADC_PERIOD_MS
        )
        {
            ADCCount =
                0;


            ADCUpdateFlag =
                1;
        }



        RadioCount++;


        if (
            RadioCount >=
            RADIO_PERIOD_MS
        )
        {
            RadioCount =
                0;


            RadioUpdateFlag =
                1;
        }



        OLEDCount++;


        if (
            OLEDCount >=
            OLED_PERIOD_MS
        )
        {
            OLEDCount =
                0;


            OLEDUpdateFlag =
                1;
        }



        TIM_ClearITPendingBit(
            TIM1,
            TIM_IT_Update
        );
    }
}

