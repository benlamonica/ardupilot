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
 * Code by Ben La Monica
 */

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
