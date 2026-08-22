/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Adapted from AP_HAL_ESP32/DeviceBus.cpp
 */

#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#include "Scheduler.h"
#include "Semaphores.h"

using namespace RP2350;

extern const AP_HAL::HAL &hal;

DeviceBus::DeviceBus(AP_HAL::Scheduler::priority_base _priority) :
    semaphore(),
    priority(_priority)
{
}

/*
  Run the callbacks registered on this bus, each at its own rate.
*/
void DeviceBus::bus_thread(void)
{
    // recorded here rather than at creation because thread_create() does not
    // hand back a handle; adjust_timer() needs it to tell callers apart
    bus_thread_handle = xTaskGetCurrentTaskHandle();

    while (true) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // find a callback to run
        for (callback = callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with the semaphore held, so a driver's transfers
                // cannot interleave with another device's on this bus
                if (semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                    callback->cb();
                    semaphore.give();
                }
            }
        }

        // work out when the next one is due
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = callbacks; callback; callback = callback->next) {
            if (next_needed == 0 || callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // sleep for at most 50ms, so that a callback registered after this
        // thread started is picked up promptly
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        // and for at least 100us, so one bus cannot monopolise a core
        if (delay < 100) {
            delay = 100;
        }
        hal.scheduler->delay_microseconds(delay);
    }
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec,
                                                                    AP_HAL::Device::PeriodicCb cb,
                                                                    AP_HAL::Device *_hal_device)
{
    if (!thread_started) {
        hal_device = _hal_device;

        char name[configMAX_TASK_NAME_LEN];
        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name, sizeof(name), "APM_I2C:%u", hal_device->bus_num());
            break;
        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name, sizeof(name), "APM_SPI:%u", hal_device->bus_num());
            break;
        default:
            snprintf(name, sizeof(name), "APM_BUS");
            break;
        }

        // left unpinned, so the SMP scheduler places it on whichever core is
        // free; only the threads Scheduler::init() creates have a fixed core
        thread_started = hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&DeviceBus::bus_thread, void), name,
            Scheduler::DEVICE_SS * sizeof(StackType_t), priority, 0);
        if (!thread_started) {
            return nullptr;
        }
    }

    DeviceBus::callback_info *callback = NEW_NOTHROW DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to the linked list this thread walks
    callback->next = callbacks;
    callbacks = callback;

    return callback;
}

/*
  Adjust when a callback next runs. This has to come from the bus thread
  itself, as anywhere else it would race with the walk above.
*/
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (xTaskGetCurrentTaskHandle() != bus_thread_handle) {
        return false;
    }
    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;
    return true;
}
