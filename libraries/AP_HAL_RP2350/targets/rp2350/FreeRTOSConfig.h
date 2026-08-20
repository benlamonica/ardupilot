#pragma once

/*
  FreeRTOS configuration for the RP2350 HAL.

  The Raspberry Pi RP2350 port is an SMP kernel (it defines
  FREE_RTOS_KERNEL_SMP=1 unconditionally), and both Cortex-M33 cores are
  used. Threads are pinned to a core via vTaskCoreAffinitySet(), the same
  split AP_HAL_ESP32 uses: latency-sensitive work on core 0, and work that
  can block for a long time on core 1.

  The values in the "required for RP2350" block below are mandated by the
  port - see portable/ThirdParty/GCC/RP2350_ARM_NTZ/README.md.
*/

// --- required for RP2350 by the port ---
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          1
// ArduPilot does floating point work in several threads, so FPU state must
// be saved and restored across context switches
#define configENABLE_FPU                        1
// the only value tested by the port
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    16

// --- SMP ---
#define configNUMBER_OF_CORES                   2
#define configUSE_CORE_AFFINITY                 1
#define configTICK_CORE                         0
#define configRUN_MULTIPLE_PRIORITIES           1
#define configUSE_PASSIVE_IDLE_HOOK             0

// --- scheduling ---
#define configUSE_PREEMPTION                    1
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    32
#define configMINIMAL_STACK_SIZE                512
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TIME_SLICING                  1
#define configUSE_TASK_NOTIFICATIONS            1

// --- synchronisation ---
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               0

// --- memory ---
// heap_3 is used, which forwards to malloc()/free(). AP_HAL_RP2350 supplies a
// locking, zeroing __wrap_malloc, so FreeRTOS and ArduPilot share one heap
// rather than statically partitioning RAM between two.
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         1
#define configAPPLICATION_ALLOCATED_HEAP        0
// let the kernel supply the idle/passive-idle task memory rather than making
// the HAL provide vApplicationGetIdleTaskMemory() boilerplate
#define configKERNEL_PROVIDED_STATIC_MEMORY     1

// --- hooks / diagnostics ---
#define configCHECK_FOR_STACK_OVERFLOW          2
// FreeRTOS allocation failures already surface as failed return codes which
// callers check (Scheduler::thread_create), so no hook is needed
#define configUSE_MALLOC_FAILED_HOOK            0

/*
  Deliberately not assert(): the build defines NDEBUG, which would compile
  every FreeRTOS assertion out. These catch kernel misuse (bad priorities,
  ISR-unsafe calls, spinlock exhaustion) and are worth keeping in a flight
  build, so they route to AP_HAL::panic() instead.
*/
#ifndef __ASSEMBLER__
#ifdef __cplusplus
extern "C" {
#endif
extern void rp2350_freertos_assert(const char *file, int line);
#ifdef __cplusplus
}
#endif
#define configASSERT(x)                                     \
    do {                                                    \
        if (!(x)) {                                         \
            rp2350_freertos_assert(__FILE__, __LINE__);     \
        }                                                   \
    } while (0)
#endif
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

// ArduPilot schedules its own periodic work, but the timer service is not
// optional here: the RP2350 port's SMP lock path calls
// xTimerPendFunctionCallFromISR(), which needs configUSE_TIMERS.
#define configUSE_TIMERS                        1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

// --- features not needed ---
#define configUSE_CO_ROUTINES                   0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configUSE_TICKLESS_IDLE                 0

// --- API subset actually used ---
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelete                     1
// required by pico_flash: flash_safe_execute() raises the priority of the
// lockout task it starts on the other core
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xSemaphoreGetMutexHolder        1
// required by the RP2350 port's SMP lock path
#define INCLUDE_xTimerPendFunctionCall          1
