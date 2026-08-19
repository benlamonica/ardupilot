#include "Semaphores.h"

#include <hardware/sync.h>
#include <hardware/timer.h>

using namespace RP2350;

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state),
    _pending(initial_state)
{}

void BinarySemaphore::signal()
{
    _pending = true;
}

/*
  consume a pending signal if there is one. The signal may be set by an
  ISR, so the test-and-clear is done with interrupts disabled.
*/
bool BinarySemaphore::take_pending()
{
    uint32_t save = save_and_disable_interrupts();
    bool got_it = _pending;
    _pending = false;
    restore_interrupts(save);
    return got_it;
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    uint64_t deadline = time_us_64() + timeout_us;
    do {
        if (take_pending()) {
            return true;
        }
    } while (time_us_64() < deadline);
    return false;
}

bool BinarySemaphore::wait_blocking()
{
    while (!take_pending()) {}
    return true;
}
