#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP2350_Namespace.h"

class RP2350::GPIO : public AP_HAL::GPIO
{
public:
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    AP_HAL::DigitalSource* channel(uint16_t n) override;

    bool    usb_connected(void) override;
};

class RP2350::DigitalSource : public AP_HAL::DigitalSource
{
public:
    DigitalSource(uint8_t pin);
    void    mode(uint8_t output) override;
    uint8_t read() override;
    void    write(uint8_t value) override;
    void    toggle() override;

private:
    uint8_t _pin;
};
