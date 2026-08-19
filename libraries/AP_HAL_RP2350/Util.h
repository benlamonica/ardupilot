#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP2350_Namespace.h"

class RP2350::Util : public AP_HAL::Util
{
public:
    void set_hw_rtc(uint64_t time_utc_usec) override;
    uint64_t get_hw_rtc() const override;

private:
    // RP2350 has no battery-backed RTC, so UTC time is tracked as an
    // offset from the free-running timer and is lost across a reboot.
    uint64_t _rtc_offset_usec = 0;
    bool _rtc_set = false;
};
