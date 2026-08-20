#include "Scheduler.h"
#include "UARTDriver.h"

#include <AP_Math/AP_Math.h>

#include <pico/bootrom.h>
#include <hardware/timer.h>
#include <hardware/watchdog.h>

using namespace RP2350;

extern const AP_HAL::HAL& hal;

bool Scheduler::_initialized = false;

bool Scheduler::create_pinned(TaskFunction_t fn, const char *name, uint32_t stack,
                              void *arg, UBaseType_t prio, TaskHandle_t *handle,
                              int core)
{
    return xTaskCreateAffinitySet(fn, name, stack, arg, prio,
                                  1u << core, handle) == pdPASS;
}

void Scheduler::init()
{
    // the main thread runs callbacks->setup()/loop(); it is created here
    // rather than run inline so that it is a normal FreeRTOS task and can be
    // preempted by the timer thread
    create_pinned(_main_thread, "APM_MAIN", MAIN_SS, this, MAIN_PRIO,
                  &_main_task_handle, CORE_FAST);
    create_pinned(_timer_thread, "APM_TIMER", TIMER_SS, this, TIMER_PRIO,
                  &_timer_task_handle, CORE_FAST);
    create_pinned(_uart_thread, "APM_UART", UART_SS, this, UART_PRIO,
                  &_uart_task_handle, CORE_FAST);
    create_pinned(_io_thread, "APM_IO", IO_SS, this, IO_PRIO,
                  &_io_task_handle, CORE_SLOW);
}

void Scheduler::delay(uint16_t ms)
{
    if (ms == 0) {
        return;
    }
    if (in_main_thread() && _min_delay_cb_ms <= ms) {
        // give the delay callback a chance to run while we wait
        uint32_t start = AP_HAL::millis();
        while (AP_HAL::millis() - start < ms) {
            vTaskDelay(1);
            call_delay_cb();
        }
        return;
    }
    vTaskDelay(ms);
}

void Scheduler::delay_microseconds(uint16_t us)
{
    if (us >= 1000) {
        // long enough to be worth yielding the core to another task
        vTaskDelay(us / 1000);
        us = us % 1000;
    }
    if (us > 0) {
        // shorter than a tick, so busy-wait; sleep_us() would not return
        // control any sooner
        busy_wait_us(us);
    }
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
    WITH_SEMAPHORE(_timer_sem);
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= RP2350_SCHEDULER_MAX_TIMER_PROCS) {
        return;
    }
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
    WITH_SEMAPHORE(_io_sem);
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            return;
        }
    }
    if (_num_io_procs >= RP2350_SCHEDULER_MAX_IO_PROCS) {
        return;
    }
    _io_proc[_num_io_procs] = proc;
    _num_io_procs++;
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    if (hold_in_bootloader) {
        reset_usb_boot(0, 0);
    }
    watchdog_reboot(0, 0, 0);
    for (;;) {}
}

bool Scheduler::in_main_thread() const
{
    return _main_task_handle == xTaskGetCurrentTaskHandle();
}

void Scheduler::set_system_initialized()
{
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }
    _initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _initialized;
}

void Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);
    vTaskDelete(nullptr);
}

bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name,
                              uint32_t stack_size, priority_base base, int8_t priority)
{
    // take a copy of the MemberProc; it is freed when the thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)malloc(sizeof(proc));
    if (tproc == nullptr) {
        return false;
    }
    *tproc = proc;

    uint8_t thread_priority = IO_PRIO;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, IO_PRIO },
        { PRIORITY_MAIN, MAIN_PRIO },
        { PRIORITY_SPI, SPI_PRIORITY },
        { PRIORITY_I2C, I2C_PRIORITY },
        { PRIORITY_CAN, IO_PRIO },
        { PRIORITY_TIMER, TIMER_PRIO },
        { PRIORITY_RCIN, RCIN_PRIO },
        { PRIORITY_IO, IO_PRIO },
        { PRIORITY_UART, UART_PRIO },
        { PRIORITY_STORAGE, STORAGE_PRIO },
        { PRIORITY_SCRIPTING, IO_PRIO },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
            thread_priority = constrain_int16(priority_map[i].p + priority,
                                              1, configMAX_PRIORITIES - 1);
            break;
        }
    }

    // stack_size is in bytes, FreeRTOS counts words
    const uint32_t stack_words = stack_size / sizeof(StackType_t);

    // bus and driver threads are left unpinned so the SMP scheduler can run
    // them on whichever core is free; only the threads created in init()
    // have a fixed core
    if (xTaskCreate(thread_create_trampoline, name, stack_words, tproc,
                    thread_priority, nullptr) != pdPASS) {
        free(tproc);
        return false;
    }
    return true;
}

void Scheduler::_run_timers()
{
    if (_in_timer_proc) {
        return;
    }
    _in_timer_proc = true;

    uint8_t num_procs;
    {
        WITH_SEMAPHORE(_timer_sem);
        num_procs = _num_timer_procs;
    }

    for (uint8_t i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void Scheduler::_run_io()
{
    if (_in_io_proc) {
        return;
    }
    _in_io_proc = true;

    uint8_t num_procs;
    {
        WITH_SEMAPHORE(_io_sem);
        num_procs = _num_io_procs;
    }

    for (uint8_t i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }

    _in_io_proc = false;
}

void Scheduler::_main_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    hal.serial(0)->begin(115200);

    sched->callbacks->setup();
    sched->set_system_initialized();

    for (;;) {
        sched->callbacks->loop();
    }
}

void Scheduler::_timer_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        vTaskDelay(1);
    }
    for (;;) {
        vTaskDelay(1);
        sched->_run_timers();
    }
}

void Scheduler::_uart_thread(void *arg)
{
    while (!_initialized) {
        vTaskDelay(1);
    }
    for (;;) {
        vTaskDelay(1);
        for (uint8_t i = 0; i < hal.num_serial; i++) {
            auto *p = hal.serial(i);
            if (p != nullptr) {
                p->_timer_tick();
            }
        }
    }
}

void Scheduler::_io_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        vTaskDelay(1);
    }
    for (;;) {
        vTaskDelay(1);
        sched->_run_io();
    }
}
