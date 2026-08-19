#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP2350_Namespace.h"

#define RP2350_SCHEDULER_MAX_TIMER_PROCS 10
#define RP2350_SCHEDULER_MAX_IO_PROCS 10

/*
  Phase 1 Scheduler: single Cortex-M33 core, no RTOS. HAL_RP2350::run()
  drives callbacks->setup()/loop() directly rather than this class spawning
  its own thread (there is nowhere to spawn one to yet). register_*_process
  and register_timer_failsafe store their callback but nothing invokes them
  yet: doing so correctly needs the real thread-context concurrency model
  driver code assumes (safe to briefly block on a semaphore), which this
  port doesn't have until Phase 2 vendors FreeRTOS-Kernel. thread_create()
  is intentionally left unimplemented (AP_HAL::Scheduler's default, which
  returns false) for the same reason.
*/
class RP2350::Scheduler : public AP_HAL::Scheduler
{
public:
    void     init() override;
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

private:
    static bool _initialized;

    AP_HAL::MemberProc _timer_proc[RP2350_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs = 0;

    AP_HAL::MemberProc _io_proc[RP2350_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs = 0;

    AP_HAL::Proc _failsafe = nullptr;
};
