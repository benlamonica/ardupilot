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
