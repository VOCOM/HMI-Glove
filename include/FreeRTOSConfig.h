#ifndef FREERTOSCONFIG
#define FREERTOSCONFIG

/* System */
#define configSTACK_DEPTH_TYPE uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE size_t

/* Scheduler Related */
#define configUSE_PREEMPTION 1
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configUSE_16_BIT_TICKS 0
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_TICKLESS_IDLE 0
#define configMINIMAL_STACK_SIZE (configSTACK_DEPTH_TYPE)512
#define configIDLE_SHOULD_YIELD 1
#define configMAX_PRIORITIES 32

/* Custom Priorities */
#define PRIORITY_IDLE 0
#define PRIORITY_LOW 1
#define PRIORITY_MEDIUM 2
#define PRIORITY_HIGH 3

/* Synchronization Related */
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_APPLICATION_TASK_TAG 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configQUEUE_REGISTRY_SIZE 8
#define configUSE_QUEUE_SETS 1
#define configUSE_TIME_SLICING 1
#define configUSE_NEWLIB_REENTRANT 0
// todo need this for lwip FreeRTOS sys_arch to compile
#define configENABLE_BACKWARD_COMPATIBILITY 1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE (128 * 1024)
#define configAPPLICATION_ALLOCATED_HEAP 0

/* Hook function related definitions. */
#define configCHECK_FOR_STACK_OVERFLOW 0
#define configUSE_MALLOC_FAILED_HOOK 0
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS 0
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES 1

/* Software timer related definitions. */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH 1024

/* Interrupt nesting behaviour configuration. */
// #define configKERNEL_INTERRUPT_PRIORITY         [dependent of processor]
// #define configMAX_SYSCALL_INTERRUPT_PRIORITY    [dependent on processor and application]
// #define configMAX_API_CALL_INTERRUPT_PRIORITY   [dependent on processor and application]

/* Armv8-M */
/* CPU configuration */
#define configCPU_CLOCK_HZ (125000000)
#define configNUMBER_OF_CORES 2
#define configTICK_CORE 0
#define configRUN_MULTIPLE_PRIORITIES 1
#define configUSE_CORE_AFFINITY 1
#define configUSE_PASSIVE_IDLE_HOOK 0

/* Not currently supported */
#define configENABLE_MPU 0
// #define configSYSTEM_CALL_STACK_SIZE            ( configSTACK_DEPTH_TYPE ) 512
#define configENABLE_FPU 1
/* Not currently supported */
#define configENABLE_TRUSTZONE 0
#define configRUN_FREERTOS_SECURE_ONLY 1
// see https://www.freertos.org/RTOS-Cortex-M3-M4.html
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 16

/* RP2xxx specific */
#define configSUPPORT_PICO_SYNC_INTEROP 1
#define configSUPPORT_PICO_TIME_INTEROP 1

/* Set the following definitions to 1 to include the API function, or zero to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_eTaskGetState 1
#define INCLUDE_xTimerPendFunctionCall 1
#define INCLUDE_xTaskAbortDelay 1
#define INCLUDE_xTaskGetHandle 1
#define INCLUDE_xTaskResumeFromISR 1
#define INCLUDE_xQueueGetMutexHolder 1

#endif /* FREERTOSCONFIG */
