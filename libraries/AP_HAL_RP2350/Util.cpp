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

#include "Util.h"

#include <hardware/timer.h>

using namespace RP2350;

void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    _rtc_offset_usec = time_utc_usec - time_us_64();
    _rtc_set = true;
}

uint64_t Util::get_hw_rtc() const
{
    if (!_rtc_set) {
        return 0;
    }
    return _rtc_offset_usec + time_us_64();
}
