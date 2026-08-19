#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "HAL_RP2350_Namespace.h"

/*
  Phase 1 of this port is single-core with no RTOS, so there is only one
  thread of execution and Semaphore cannot actually be contended: take()
  always succeeds immediately. This is NOT a real mutex - in particular it
  does not lock out an interrupt handler, so it must not be relied on to
  protect state shared with an ISR. Phase 2 adds real threads and must
  replace this with a genuine blocking primitive.
*/
class RP2350::Semaphore : public AP_HAL::Semaphore
{
public:
    bool give() override { return true; }
    bool take(uint32_t timeout_ms) override { return true; }
    bool take_nonblocking() override { return true; }
};

/*
  BinarySemaphore does carry a real signal, since an ISR may signal a
  waiter running in the main context. With no scheduler to sleep on, wait()
  spins on the flag rather than blocking.
*/
class RP2350::BinarySemaphore : public AP_HAL::BinarySemaphore
{
public:
    BinarySemaphore(bool initial_state=false);

    CLASS_NO_COPY(BinarySemaphore);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking() override;
    void signal() override;

private:
    bool take_pending();

    volatile bool _pending;
};
