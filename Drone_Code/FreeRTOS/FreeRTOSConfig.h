/*
 * FreeRTOS Kernel V10.4.6 configuration for the STM32F103C8 flight controller.
 *
 * The application uses a 200 Hz flight task, a 100 Hz remote-control task and
 * a 5 Hz telemetry task.  Short critical-section snapshots isolate the flight
 * control chain from asynchronous remote and telemetry work.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler and system tick ------------------------------------------------ */
#define configUSE_PREEMPTION                         1
#define configUSE_TIME_SLICING                       1
#define configUSE_IDLE_HOOK                          0
#define configUSE_TICK_HOOK                          0
#define configUSE_TICKLESS_IDLE                      0

/* SystemCoreClock is configured to 72 MHz by system_stm32f10x.c. */
#define configCPU_CLOCK_HZ                           ( ( unsigned long ) 72000000UL )
#define configTICK_RATE_HZ                           ( ( TickType_t ) 1000U )

#define configMAX_PRIORITIES                         5
#define configMINIMAL_STACK_SIZE                     ( ( unsigned short ) 128U )
#define configMAX_TASK_NAME_LEN                      16
#define configUSE_16_BIT_TICKS                       0
#define configIDLE_SHOULD_YIELD                      1

/* Memory ------------------------------------------------------------------- */
/*
 * STM32F103C8 has 20 KiB SRAM.  Task stacks and TCBs are allocated from the
 * FreeRTOS heap, so the old 17 KiB value left no safe margin for application
 * data and the Cortex-M main stack.
 */
#define configSUPPORT_DYNAMIC_ALLOCATION             1
#define configSUPPORT_STATIC_ALLOCATION              0
#define configTOTAL_HEAP_SIZE                        ( ( size_t ) ( 8U * 1024U ) )
#define configUSE_MALLOC_FAILED_HOOK                 1
#define configCHECK_FOR_STACK_OVERFLOW               2

/* Optional kernel facilities ---------------------------------------------- */
#define configUSE_TIMERS                              0
#define configUSE_MUTEXES                             0
#define configUSE_RECURSIVE_MUTEXES                   0
#define configUSE_COUNTING_SEMAPHORES                 0
#define configUSE_CO_ROUTINES                         0
#define configMAX_CO_ROUTINE_PRIORITIES               2
#define configUSE_TRACE_FACILITY                      0
#define configUSE_STATS_FORMATTING_FUNCTIONS          0
#define configUSE_TASK_NOTIFICATIONS                  1

/* API inclusion ------------------------------------------------------------ */
#define INCLUDE_vTaskPrioritySet                      1
#define INCLUDE_uxTaskPriorityGet                    1
#define INCLUDE_vTaskDelete                           1
#define INCLUDE_vTaskCleanUpResources                 0
#define INCLUDE_vTaskSuspend                          1
#define INCLUDE_vTaskDelayUntil                       1
#define INCLUDE_vTaskDelay                            1
#define INCLUDE_uxTaskGetStackHighWaterMark           1

/* Cortex-M3 interrupt priorities ------------------------------------------ */
/*
 * STM32F1 implements four priority bits.  Interrupts that call a FreeRTOS
 * ...FromISR API must use a numerical priority from 11 through 15.
 */
#define configPRIO_BITS                               4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 11
#define configKERNEL_INTERRUPT_PRIORITY               \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY          \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* Fail-safe assertions ----------------------------------------------------- */
extern void vApplicationAssertHook( const char * pcFile, unsigned long ulLine );

#define configASSERT( expression )                    \
    do                                                \
    {                                                 \
        if( ( expression ) == 0 )                     \
        {                                             \
            vApplicationAssertHook( __FILE__, __LINE__ ); \
        }                                             \
    } while( 0 )

/* Cortex-M exception handlers are supplied by FreeRTOS/port/port.c. */
#define xPortPendSVHandler                            PendSV_Handler
#define vPortSVCHandler                               SVC_Handler
#define xPortSysTickHandler                           SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
