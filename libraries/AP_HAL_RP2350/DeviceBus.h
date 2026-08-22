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
 * Adapted from AP_HAL_ESP32/DeviceBus.h
 */

#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include <AP_HAL/Scheduler.h>
#include "HAL_RP2350_Namespace.h"
#include "Semaphores.h"

#include <FreeRTOS.h>
#include <task.h>

namespace RP2350
{

/*
  One bus and the thread that services it. Sensor drivers ask to be called
  back at a fixed rate and do their transfers from that callback, so every
  transfer on a bus happens on one thread with the bus semaphore held. That is
  what lets a driver talk to its device without knowing what else shares the
  wires with it.
*/
class DeviceBus
{
public:
    DeviceBus(AP_HAL::Scheduler::priority_base _priority);

    struct DeviceBus *next;
    Semaphore semaphore;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(uint32_t period_usec,
                                                              AP_HAL::Device::PeriodicCb cb,
                                                              AP_HAL::Device *hal_device);
    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);

private:
    void bus_thread(void);

    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    } *callbacks;

    AP_HAL::Scheduler::priority_base priority;
    TaskHandle_t bus_thread_handle;
    bool thread_started;
    AP_HAL::Device *hal_device;
};

}
