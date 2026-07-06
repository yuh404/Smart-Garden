/*
 * FreeRTOSConfig.h
 *
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>


extern uint32_t SystemCoreClock;


#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0

#define configCPU_CLOCK_HZ                      ((unsigned long)72000000)
#define configTICK_RATE_HZ                      ((TickType_t)1000)

#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)   /* words, cho Idle Task */
#define configMAX_TASK_NAME_LEN                 12
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* ========================================================================== */
/* HEAP - RAM                                                                 */
/* ========================================================================== */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(6 * 1024))  /* 6KB, dung heap_4.c */
#define configAPPLICATION_ALLOCATED_HEAP         0

/* ========================================================================== */
/* MUTEX / SEMAPHORE / QUEUE                                                   */
/* ========================================================================== */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               4
#define configUSE_QUEUE_SETS                    0

/* ========================================================================== */
/* SOFTWARE TIMER - dang khong dung trong project nay, tat de tiet kiem RAM   */
/* ========================================================================== */
#define configUSE_TIMERS                        0
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

/* ========================================================================== */
/* HOOK FUNCTIONS - da dinh nghia vApplicationStackOverflowHook() va          */
/* vApplicationMallocFailedHook() trong main.c                                */
/* ========================================================================== */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2   /* bat buoc bat khi RAM it, giup bat loi tran stack */
#define configUSE_MALLOC_FAILED_HOOK            1   /* bat de phat hien som khi het heap 6KB */
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* ========================================================================== */
/* DEBUG / STATS - tat het de tiet kiem RAM va Flash, bat lai khi can debug   */
/* ========================================================================== */
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configGENERATE_RUN_TIME_STATS           0

/* ========================================================================== */
/* CO-ROUTINE - khong dung                                                    */
/* ========================================================================== */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

/* ========================================================================== */
/* CAC HAM API DUOC BAT (INCLUDE_...)                                          */
/* ========================================================================== */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetTickCount               1
#define INCLUDE_uxTaskGetStackHighWaterMark     1   /* dung de do stack thuc te tung task sau khi chay thu */
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   1

/* ========================================================================== */
/* CAU HINH NGAT (CORTEX-M3)                                                  */
/* ========================================================================== */
#define configPRIO_BITS                                    4  /* STM32F1 dung 4 bit uu tien */


#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY            15


#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY       5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Alias ten CMSIS chuan (mot so ban port cu dung ten nay thay vi ten tren) */
#define configKERNEL_INTERRUPT_PRIORITY_REGISTER            configKERNEL_INTERRUPT_PRIORITY

/* ========================================================================== */
/* ASSERT                                                                     */
/* ========================================================================== */
#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for (;;) {} }


#define vPortSVCHandler      SVC_Handler
#define xPortPendSVHandler   PendSV_Handler
#define xPortSysTickHandler  SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
