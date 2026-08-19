#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>
#include <AP_HAL_RP2350/HAL_RP2350_Namespace.h>

class HAL_RP2350 : public AP_HAL::HAL
{
public:
    HAL_RP2350();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};
