#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "HAL_RP2350_Namespace.h"

#include <FreeRTOS.h>
#include <semphr.h>

/*
  Both Cortex-M33 cores run tasks, so these must exclude the other core as
  well as other tasks. FreeRTOS SMP primitives do that; masking interrupts
  would not, as that only affects the calling core.

  ArduPilot semaphores are recursive (a thread holding one may take it
  again), so this is a FreeRTOS recursive mutex.
*/
class RP2350::Semaphore : public AP_HAL::Semaphore
{
public:
    Semaphore();

    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

protected:
    SemaphoreHandle_t _sem;
    StaticSemaphore_t _sem_storage;
};

class RP2350::BinarySemaphore : public AP_HAL::BinarySemaphore
{
public:
    BinarySemaphore(bool initial_state=false);

    CLASS_NO_COPY(BinarySemaphore);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking() override;
    void signal() override;
    void signal_ISR() override;

protected:
    SemaphoreHandle_t _sem;
    StaticSemaphore_t _sem_storage;
};
