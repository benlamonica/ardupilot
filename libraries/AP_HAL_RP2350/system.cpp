#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include <AP_Math/div1000.h>

#include <hardware/timer.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdint.h>
#include <stdio.h>

namespace AP_HAL
{

void init()
{}

void panic(const char *errormsg, ...)
{
    va_list ap;

    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);

    while (1) {}
}

uint32_t micros()
{
    return micros64();
}

uint32_t millis()
{
    return millis64();
}

uint64_t micros64()
{
    return time_us_64();
}

uint64_t millis64()
{
    return uint64_div1000(micros64());
}

} // namespace AP_HAL

/*
  target of configASSERT() - see targets/rp2350/FreeRTOSConfig.h
*/
extern "C" void rp2350_freertos_assert(const char *file, int line)
{
    AP_HAL::panic("FreeRTOS assert %s:%d", file, line);
}

/*
  configCHECK_FOR_STACK_OVERFLOW. A thread that has run off its stack has
  already corrupted memory, so there is nothing safe to continue with.
*/
extern "C" void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    AP_HAL::panic("stack overflow in %s", name);
}

static HAL_RP2350 hal_rp2350;

const AP_HAL::HAL& AP_HAL::get_HAL()
{
    return hal_rp2350;
}

AP_HAL::HAL& AP_HAL::get_HAL_mutable()
{
    return hal_rp2350;
}
