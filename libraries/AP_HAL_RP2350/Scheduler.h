#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP2350_Namespace.h"
#include "Semaphores.h"

#include <FreeRTOS.h>
#include <task.h>

#define RP2350_SCHEDULER_MAX_TIMER_PROCS 10
#define RP2350_SCHEDULER_MAX_IO_PROCS 10

/*
  Threads are pinned across the two Cortex-M33 cores using the same split
  AP_HAL_ESP32 uses: latency-sensitive work (main loop, timers, UARTs) on
  core 0, and work that can block for a long time (IO, storage) on core 1.
*/
class RP2350::Scheduler : public AP_HAL::Scheduler
{
public:
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb) { callbacks = cb; }

    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size,
                       priority_base base, int8_t priority) override;

    // core assignment
    static const int CORE_FAST = 0;
    static const int CORE_SLOW = 1;

    // FreeRTOS priorities, 1..configMAX_PRIORITIES-1
    static const int MAIN_PRIO    = 24;
    static const int TIMER_PRIO   = 23;
    static const int UART_PRIO    = 22;
    static const int SPI_PRIORITY = 21;
    static const int I2C_PRIORITY = 20;
    static const int RCIN_PRIO    = 10;
    static const int RCOUT_PRIO   = 10;
    static const int IO_PRIO      = 6;
    static const int STORAGE_PRIO = 5;

    // stack sizes, in words (FreeRTOS counts StackType_t, not bytes)
    static const int MAIN_SS      = 2048;
    static const int TIMER_SS     = 1024;
    static const int UART_SS      = 1024;
    static const int IO_SS        = 1024;
    static const int STORAGE_SS   = 1024;
    static const int DEVICE_SS    = 1024;

private:
    AP_HAL::HAL::Callbacks *callbacks;

    static bool _initialized;

    AP_HAL::MemberProc _timer_proc[RP2350_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;
    bool _in_timer_proc;
    Semaphore _timer_sem;

    AP_HAL::MemberProc _io_proc[RP2350_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;
    bool _in_io_proc;
    Semaphore _io_sem;

    AP_HAL::Proc _failsafe;

    TaskHandle_t _main_task_handle;
    TaskHandle_t _timer_task_handle;
    TaskHandle_t _uart_task_handle;
    TaskHandle_t _io_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);

    static void thread_create_trampoline(void *ctx);

    // create a task pinned to the given core
    static bool create_pinned(TaskFunction_t fn, const char *name, uint32_t stack,
                              void *arg, UBaseType_t prio, TaskHandle_t *handle,
                              int core);

    void _run_timers();
    void _run_io();
};
