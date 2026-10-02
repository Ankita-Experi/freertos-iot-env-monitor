#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * FreeRTOS kernel configuration for STM32F446RE @ 180 MHz.
 * See https://www.freertos.org/a00110.html for every option.
 */

#if defined(__GNUC__) || defined(__ICCARM__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
void app_assert_failed(const char *file, int line);
#endif

/* ---- Scheduler ---------------------------------------------------------- */
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1   /* CLZ-based ready-list lookup */
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     8
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                  12
#define configTICK_TYPE_WIDTH_IN_BITS            TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                  1

/* ---- Synchronisation primitives ---------------------------------------- */
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_QUEUE_SETS                     1   /* CAN task waits on TX + RX queues */
#define configUSE_TASK_NOTIFICATIONS             1
#define configQUEUE_REGISTRY_SIZE                8   /* names queues for debugger views */

/* ---- Memory ------------------------------------------------------------- */
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configSUPPORT_STATIC_ALLOCATION          0
#define configTOTAL_HEAP_SIZE                    ((size_t)(32 * 1024))

/* ---- Hooks & diagnostics ----------------------------------------------- */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             1
#define configCHECK_FOR_STACK_OVERFLOW           2   /* pattern check on context switch */
#define configUSE_TRACE_FACILITY                 1
#define configGENERATE_RUN_TIME_STATS            0
#define configRECORD_STACK_HIGH_ADDRESS          1

/* ---- Software timers (unused, kept off to save RAM) -------------------- */
#define configUSE_TIMERS                         0

/* ---- Optional API ------------------------------------------------------- */
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskDelete                      0
#define INCLUDE_vTaskSuspend                     1   /* lets portMAX_DELAY block forever */
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1

/* ---- Cortex-M interrupt priorities --------------------------------------
 * STM32F4 implements 4 priority bits. ISRs that call FreeRTOS *FromISR()
 * APIs must have a numerical priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 * (i.e. logically lower or equal urgency). See board.h for the IRQ map.
 */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                          __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                          4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY       15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define configASSERT(x) do { if ((x) == 0) { app_assert_failed(__FILE__, __LINE__); } } while (0)

/* Map FreeRTOS port handlers onto the CMSIS vector names. */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
