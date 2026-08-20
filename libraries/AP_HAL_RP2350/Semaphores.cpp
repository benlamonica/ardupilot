#include "Semaphores.h"

using namespace RP2350;

/*
  Static allocation is used throughout: many semaphores are members of
  objects constructed before the heap is in a useful state, and a failed
  allocation in a constructor has nowhere sensible to report to.
*/

Semaphore::Semaphore()
{
    _sem = xSemaphoreCreateRecursiveMutexStatic(&_sem_storage);
}

bool Semaphore::give()
{
    return xSemaphoreGiveRecursive(_sem) == pdTRUE;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return xSemaphoreTakeRecursive(_sem, portMAX_DELAY) == pdTRUE;
    }
    return xSemaphoreTakeRecursive(_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool Semaphore::take_nonblocking()
{
    return xSemaphoreTakeRecursive(_sem, 0) == pdTRUE;
}

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    _sem = xSemaphoreCreateBinaryStatic(&_sem_storage);
    if (initial_state) {
        xSemaphoreGive(_sem);
    }
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    // the tick is 1ms, so sub-millisecond timeouts are rounded up rather
    // than truncated to zero, which would turn a short wait into a poll.
    // wait(0) must stay non-blocking, as wait_nonblocking() uses it.
    const uint32_t timeout_ms = (timeout_us + 999U) / 1000U;
    return xSemaphoreTake(_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool BinarySemaphore::wait_blocking()
{
    return xSemaphoreTake(_sem, portMAX_DELAY) == pdTRUE;
}

void BinarySemaphore::signal()
{
    xSemaphoreGive(_sem);
}

void BinarySemaphore::signal_ISR()
{
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(_sem, &wake);
    portYIELD_FROM_ISR(wake);
}
