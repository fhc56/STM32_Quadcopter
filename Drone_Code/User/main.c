#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "Delay.h"

#include "Motor.h"
#include "LED.h"
#include "Serial.h"

#include "RemoteRF.h"
#include "Attitude.h"
#include "FlightControl.h"

#include "Barometer.h"
#include "AltitudeControl.h"

/* 三个业务任务的周期、栈深度和优先级。栈深度的单位是StackType_t。 */
#define FLIGHT_TASK_PERIOD_MS	 5U
#define REMOTE_TASK_PERIOD_MS	 10U
#define TELEMETRY_TASK_PERIOD_MS 200U

#define FLIGHT_TASK_STACK_DEPTH	   384U
#define REMOTE_TASK_STACK_DEPTH	   192U
#define TELEMETRY_TASK_STACK_DEPTH 256U

#define FLIGHT_TASK_PRIORITY	(tskIDLE_PRIORITY + 4U)
#define REMOTE_TASK_PRIORITY	(tskIDLE_PRIORITY + 3U)
#define TELEMETRY_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)

/**
 * @brief RemoteTask发布给FlightTask的一帧完整遥控命令。
 * @note  通过极短临界区整体复制，避免飞控读到来自不同数据包的混合字段。
 */
typedef struct
{
	int8_t VerticalCommand;
	float TargetRoll;
	float TargetPitch;
	uint8_t Connected;
	uint8_t EmergencyStop;
} RemoteCommand_t;

/**
 * @brief FlightTask发布给TelemetryTask的只读遥测快照。
 * @note  遥测任务不直接访问姿态、气压、PID或电机模块的内部状态。
 */
typedef struct
{
	float Height;
	float TargetHeight;
	float VerticalSpeed;
	float Roll;
	float Pitch;
	uint32_t Pressure;
	int8_t VerticalCommand;
	uint8_t Throttle;
	uint8_t BarometerOK;
	uint8_t AltitudeActive;
	uint8_t AltitudeLanding;
	uint8_t RFConnected;
	uint8_t EmergencyStop;
	uint8_t FaultLatched;
	uint8_t Motor1;
	uint8_t Motor2;
	uint8_t Motor3;
	uint8_t Motor4;
	uint8_t Valid;
} FlightTelemetry_t;

/**
 * @brief 飞控任务在多次循环之间需要保留的状态。
 *
 * 将这些变量集中管理只是为了提高可读性；它们的初始值和原main函数中的
 * 局部变量完全一致，不改变姿态、定高或电机控制算法。
 */
typedef struct
{
	Attitude_t Attitude;
	uint8_t EmergencyStopLast;
} FlightRuntime_t;

static RemoteCommand_t SharedRemoteCommand;
static FlightTelemetry_t SharedFlightTelemetry;

/**
 * @brief 立即关闭四路PWM并复位MCU。
 *
 * FreeRTOS断言、任务栈溢出、内存分配失败或调度器异常返回时都走同一条
 * 失效安全路径。直接写CCR寄存器可以避免依赖任务或普通驱动调用。
 */
static void FlightSystemFailSafe(void)
{
	TIM2->CCR1 = 0U;
	TIM2->CCR2 = 0U;
	TIM2->CCR3 = 0U;
	TIM2->CCR4 = 0U;

	__disable_irq();
	NVIC_SystemReset();

	while (1)
	{
	}
}

/**
 * @brief RemoteTask原子发布一帧遥控命令。
 */
static void FlightWriteRemoteCommand(const RemoteCommand_t *Command)
{
	taskENTER_CRITICAL();
	SharedRemoteCommand = *Command;
	taskEXIT_CRITICAL();
}

/**
 * @brief FlightTask原子读取一帧遥控命令。
 */
static void FlightReadRemoteCommand(RemoteCommand_t *Command)
{
	taskENTER_CRITICAL();
	*Command = SharedRemoteCommand;
	taskEXIT_CRITICAL();
}

/**
 * @brief FlightTask原子发布一帧遥测数据。
 */
static void FlightWriteTelemetry(const FlightTelemetry_t *Telemetry)
{
	taskENTER_CRITICAL();
	SharedFlightTelemetry = *Telemetry;
	taskEXIT_CRITICAL();
}

/**
 * @brief TelemetryTask原子读取最近一帧遥测数据。
 * @retval 1 已经存在有效飞控数据。
 * @retval 0 FlightTask尚未完成第一帧。
 */
static uint8_t FlightReadTelemetry(FlightTelemetry_t *Telemetry)
{
	taskENTER_CRITICAL();
	*Telemetry = SharedFlightTelemetry;
	taskEXIT_CRITICAL();

	return Telemetry->Valid;
}

/**
 * @brief 初始化独立看门狗。
 * @note  只有200 Hz飞控任务可以喂狗，其他任务不得代替它喂狗。
 */
static void SafetyWatchdog_Init(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);
	IWDG_SetReload(500U);
	IWDG_ReloadCounter();
	IWDG_Enable();
}

/**
 * @brief 按原裸机顺序初始化全部飞控硬件和算法模块。
 *
 * 初始化在调度器启动前完成。此阶段的传感器预热、校准延时使用DWT，
 * 不占用也不修改FreeRTOS的SysTick。
 */
static void FlightHardware_Init(void)
{
	uint8_t BarometerInitOK;

	/* 上电后第一件事是建立PWM并确保所有电机关闭。 */
	Motor_Init();
	Motor_StopAll();

	LED_Init();
	Serial_Init();

	/* MPU6050初始化、静态校准及姿态时间基准。 */
	Attitude_Init();

	/* BMP280预热和地面气压标定。 */
	BarometerInitOK = Barometer_Init();

	/* 遥控器默认处于急停状态，收到合法ARM命令后才解除。 */
	RemoteRF_Init();

	FlightControl_Init();
	AltitudeControl_Init();

	LED1_ON();

	if (BarometerInitOK != 0U)
	{
		Serial_Printf("BMP280 OK\r\n");
	}
	else
	{
		Serial_Printf("BMP280 ERROR\r\n");
	}

	/* 所有耗时初始化结束后再启动看门狗。 */
	SafetyWatchdog_Init();
}

/**
 * @brief 写入安全的初始遥控状态。
 * @note  初始化发生在调度器启动前，FlightTask第一帧就能读到上电急停状态。
 */
static void FlightSharedData_Init(void)
{
	SharedRemoteCommand.VerticalCommand = RemoteRF_GetVerticalCommand();
	SharedRemoteCommand.TargetRoll = RemoteRF_GetTargetRoll();
	SharedRemoteCommand.TargetPitch = RemoteRF_GetTargetPitch();
	SharedRemoteCommand.Connected = RemoteRF_IsConnected();
	SharedRemoteCommand.EmergencyStop = RemoteRF_GetEmergencyStop();

	SharedFlightTelemetry.Valid = 0U;
}

/**
 * @brief 处理遥控急停状态。
 * @retval 1 当前处于急停，本周期不执行姿态和电机控制。
 * @retval 0 当前允许进入正常控制流程。
 */
static uint8_t FlightHandleEmergencyStop(FlightRuntime_t *Runtime, const RemoteCommand_t *Command)
{
	if (Command->EmergencyStop == 0U)
	{
		return 0U;
	}

	/* 急停期间每个周期都强制四路PWM为0。 */
	Motor_StopAll();

	/* 只在进入急停的第一个周期清除控制器历史状态。 */
	if (Runtime->EmergencyStopLast == 0U)
	{
		Runtime->EmergencyStopLast = 1U;

		FlightControl_Init();
		AltitudeControl_Init();
		Motor_StopAll();
	}

	IWDG_ReloadCounter();
	return 1U;
}

/**
 * @brief 输出原程序的低频调试数据。
 *
 * TelemetryTask每200 ms调用一次。串口驱动使用中断环形缓冲区，
 * 缓冲区满时丢弃调试字节，不会阻塞FlightTask。
 */
static void FlightPrintTelemetry(const FlightTelemetry_t *Telemetry)
{
	if (Telemetry->EmergencyStop != 0U)
	{
		Serial_Printf("ESTOP=1 RF=%d M=0,0,0,0\r\n", Telemetry->RFConnected);
		return;
	}

	Serial_Printf("B=%d BP=%lu H=%d TH=%d VS=%d "
				  "VC=%d T=%d A=%d L=%d RF=%d E=%d "
				  "R=%d P=%d M=%d,%d,%d,%d\r\n",
				  Telemetry->BarometerOK, Telemetry->Pressure, (int)(Telemetry->Height * 100.0f),
				  (int)(Telemetry->TargetHeight * 100.0f), (int)(Telemetry->VerticalSpeed * 100.0f),
				  (int)Telemetry->VerticalCommand, Telemetry->Throttle, Telemetry->AltitudeActive,
				  Telemetry->AltitudeLanding, Telemetry->RFConnected, Telemetry->EmergencyStop,
				  (int)(Telemetry->Roll * 10.0f), (int)(Telemetry->Pitch * 10.0f),
				  Telemetry->Motor1, Telemetry->Motor2, Telemetry->Motor3, Telemetry->Motor4);
}

/**
 * @brief 在FlightTask内采集并发布一帧一致的遥测快照。
 */
static void FlightPublishTelemetry(const FlightRuntime_t *Runtime, const RemoteCommand_t *Command,
								   int8_t VerticalCommand, uint8_t Throttle)
{
	FlightTelemetry_t Telemetry;

	Telemetry.Height = AltitudeControl_GetHeight();
	Telemetry.TargetHeight = AltitudeControl_GetTargetHeight();
	Telemetry.VerticalSpeed = AltitudeControl_GetVerticalSpeed();
	Telemetry.Roll = Runtime->Attitude.Roll;
	Telemetry.Pitch = Runtime->Attitude.Pitch;
	Telemetry.Pressure = Barometer_GetPressure();
	Telemetry.VerticalCommand = VerticalCommand;
	Telemetry.Throttle = Throttle;
	Telemetry.BarometerOK = Barometer_IsOK();
	Telemetry.AltitudeActive = AltitudeControl_IsActive();
	Telemetry.AltitudeLanding = AltitudeControl_IsLanding();
	Telemetry.RFConnected = Command->Connected;
	Telemetry.EmergencyStop = Command->EmergencyStop;
	Telemetry.FaultLatched = FlightControl_IsFaultLatched();
	Telemetry.Motor1 = FlightControl_GetM1();
	Telemetry.Motor2 = FlightControl_GetM2();
	Telemetry.Motor3 = FlightControl_GetM3();
	Telemetry.Motor4 = FlightControl_GetM4();
	Telemetry.Valid = 1U;

	FlightWriteTelemetry(&Telemetry);
}

/**
 * @brief 执行一次正常飞控周期。
 *
 * 调用顺序与原裸机while(1)保持一致：遥控目标已经更新后，依次执行姿态、
 * 气压计、定高控制、姿态控制和电机混控。
 */
static void FlightRunControlCycle(FlightRuntime_t *Runtime, const RemoteCommand_t *Command)
{
	Attitude_t ControlAttitude;
	uint8_t Throttle;

	Runtime->EmergencyStopLast = 0U;

	Attitude_Update(&Runtime->Attitude);
	Barometer_Update(Runtime->Attitude.dt);

	AltitudeControl_Update(Command->VerticalCommand, Runtime->Attitude.dt, Command->Connected);

	Throttle = AltitudeControl_GetThrottle();

	ControlAttitude = Runtime->Attitude;
	ControlAttitude.Roll = Runtime->Attitude.Roll - Command->TargetRoll;
	ControlAttitude.Pitch = Runtime->Attitude.Pitch - Command->TargetPitch;

	FlightControl_Update(Throttle, &ControlAttitude);
	FlightPublishTelemetry(Runtime, Command, Command->VerticalCommand, Throttle);

	IWDG_ReloadCounter();
}

/**
 * @brief 200 Hz高优先级飞控任务。
 * @note  独占MPU6050、BMP280、Mahony、定高、串级PID和电机控制链路。
 */
static void FlightTask(void *Argument)
{
	FlightRuntime_t Runtime = {0};
	RemoteCommand_t Command;
	TickType_t LastWakeTime;

	(void)Argument;

	LastWakeTime = xTaskGetTickCount();

	while (1)
	{
		FlightReadRemoteCommand(&Command);

		if (FlightHandleEmergencyStop(&Runtime, &Command) == 0U)
		{
			FlightRunControlCycle(&Runtime, &Command);
		}
		else
		{
			FlightPublishTelemetry(&Runtime, &Command, 0, 0U);
		}

		vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS));
	}
}

/**
 * @brief 100 Hz遥控任务。
 * @note  该任务独占NRF24L01和RemoteRF状态，完成收包后一次性发布命令快照。
 */
static void RemoteTask(void *Argument)
{
	RemoteCommand_t Command;
	TickType_t LastWakeTime;

	(void)Argument;

	LastWakeTime = xTaskGetTickCount();

	while (1)
	{
		RemoteRF_Update();

		Command.VerticalCommand = RemoteRF_GetVerticalCommand();
		Command.TargetRoll = RemoteRF_GetTargetRoll();
		Command.TargetPitch = RemoteRF_GetTargetPitch();
		Command.Connected = RemoteRF_IsConnected();
		Command.EmergencyStop = RemoteRF_GetEmergencyStop();
		FlightWriteRemoteCommand(&Command);

		vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(REMOTE_TASK_PERIOD_MS));
	}
}

/**
 * @brief 5 Hz低优先级遥测任务。
 * @note  只发送FlightTask发布的快照，不直接访问飞控算法和硬件状态。
 */
static void TelemetryTask(void *Argument)
{
	FlightTelemetry_t Telemetry;
	TickType_t LastWakeTime;
	uint8_t FaultReported = 0U;

	(void)Argument;

	LastWakeTime = xTaskGetTickCount();

	while (1)
	{
		/* 首次输出也等待完整的200 ms周期，与原单任务版本保持一致。 */
		vTaskDelayUntil(&LastWakeTime, pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS));

		if (FlightReadTelemetry(&Telemetry) == 0U)
		{
			continue;
		}

		if ((Telemetry.FaultLatched != 0U) && (FaultReported == 0U))
		{
			FaultReported = 1U;

			Serial_Printf("FAULT R=%d P=%d\r\n", (int)(Telemetry.Roll * 10.0f),
						  (int)(Telemetry.Pitch * 10.0f));
		}

		FlightPrintTelemetry(&Telemetry);
	}
}

/* FreeRTOS安全钩子 --------------------------------------------------------- */

void vApplicationMallocFailedHook(void)
{
	FlightSystemFailSafe();
}

void vApplicationStackOverflowHook(TaskHandle_t Task, char *TaskName)
{
	(void)Task;
	(void)TaskName;
	FlightSystemFailSafe();
}

void vApplicationAssertHook(const char *File, unsigned long Line)
{
	(void)File;
	(void)Line;
	FlightSystemFailSafe();
}

int main(void)
{
	BaseType_t CreateResult;

	/* FreeRTOS要求STM32F1的4个优先级位全部用于抢占优先级。 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/* DWT延时必须先于任何传感器初始化。 */
	Delay_Init();
	FlightHardware_Init();
	FlightSharedData_Init();

	CreateResult = xTaskCreate(FlightTask, "FlightTask", FLIGHT_TASK_STACK_DEPTH, NULL,
							   FLIGHT_TASK_PRIORITY, NULL);

	if (CreateResult != pdPASS)
	{
		FlightSystemFailSafe();
	}

	CreateResult = xTaskCreate(RemoteTask, "RemoteTask", REMOTE_TASK_STACK_DEPTH, NULL,
							   REMOTE_TASK_PRIORITY, NULL);

	if (CreateResult != pdPASS)
	{
		FlightSystemFailSafe();
	}

	CreateResult = xTaskCreate(TelemetryTask, "TelemetryTask", TELEMETRY_TASK_STACK_DEPTH, NULL,
							   TELEMETRY_TASK_PRIORITY, NULL);

	if (CreateResult != pdPASS)
	{
		FlightSystemFailSafe();
	}

	vTaskStartScheduler();

	/* 调度器正常情况下永远不会返回。 */
	FlightSystemFailSafe();
	return 0;
}
