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
/*
  malloc wrapper for RP2350.

  ArduPilot and FreeRTOS (configured for heap_3) share newlib's heap, and
  both cores allocate, so the allocator needs cross-core mutual exclusion.
  Masking interrupts is not sufficient on this part: it only affects the
  calling core, leaving the other one free to enter the allocator.

  vTaskSuspendAll()/xTaskResumeAll() is the lock, which is what FreeRTOS's
  own heap implementations use. In the SMP kernel it takes portGET_TASK_LOCK(),
  a cross-core spinlock, so it excludes the other core as well as other tasks.
  Before the scheduler is running it does nothing at all, which is safe: only
  core 0 is awake then, so the early-boot and static-constructor allocations
  cannot race.

  Note this means malloc() must not be called from an interrupt handler -
  vTaskSuspendAll() contains portASSERT_IF_IN_ISR(). Allocating in an ISR is
  already disallowed, so this turns a latent bug into a loud one.

  Like the generic wrapper in AP_Common/c++.cpp - which excludes this board -
  the returned memory is zeroed, as ArduPilot code relies on that.
*/

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_RP2350

#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

extern "C" {
    void *__wrap_malloc(size_t size);
    void *__real_malloc(size_t size);
    void *__wrap_calloc(size_t nmemb, size_t size);
    void *__real_calloc(size_t nmemb, size_t size);
    void *__wrap_realloc(void *ptr, size_t size);
    void *__real_realloc(void *ptr, size_t size);
    void __wrap_free(void *ptr);
    void __real_free(void *ptr);
}

/*
  all four entry points are wrapped, not just malloc: newlib's calloc() and
  realloc() manipulate the heap themselves rather than going through
  malloc()/free() alone, so locking only malloc would leave them unprotected.
  Nesting is safe, as vTaskSuspendAll() is a counter.
*/

void *__wrap_malloc(size_t size)
{
    vTaskSuspendAll();
    void *ret = __real_malloc(size);
    xTaskResumeAll();

    if (ret != nullptr) {
        memset(ret, 0, size);
    }
    return ret;
}

void *__wrap_calloc(size_t nmemb, size_t size)
{
    // calloc() zeroes by definition, so only the locking is added here
    vTaskSuspendAll();
    void *ret = __real_calloc(nmemb, size);
    xTaskResumeAll();
    return ret;
}

void *__wrap_realloc(void *ptr, size_t size)
{
    // deliberately not zeroed: the grown region cannot be zeroed without
    // knowing the old size. Callers needing that guarantee use
    // mem_realloc() from AP_Common/c++.cpp, which is built on malloc().
    vTaskSuspendAll();
    void *ret = __real_realloc(ptr, size);
    xTaskResumeAll();
    return ret;
}

void __wrap_free(void *ptr)
{
    vTaskSuspendAll();
    __real_free(ptr);
    xTaskResumeAll();
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_RP2350
