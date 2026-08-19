#include "Scheduler.h"

#include <pico/time.h>
#include <pico/bootrom.h>
#include <hardware/watchdog.h>

using namespace RP2350;

bool Scheduler::_initialized = false;

void Scheduler::init()
{}

void Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start) / 1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            call_delay_cb();
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us)
{
    sleep_us(us);
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
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
    // single-core, no RTOS yet: there is only one execution context
    return true;
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
