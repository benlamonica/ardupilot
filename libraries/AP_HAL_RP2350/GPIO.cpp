#include "GPIO.h"

#include <hardware/gpio.h>

using namespace RP2350;

extern const AP_HAL::HAL& hal;

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    if (pin >= NUM_BANK0_GPIOS) {
        return;
    }
    gpio_init(pin);
    gpio_set_dir(pin, output ? GPIO_OUT : GPIO_IN);
}

uint8_t GPIO::read(uint8_t pin)
{
    if (pin >= NUM_BANK0_GPIOS) {
        return 0;
    }
    return gpio_get(pin);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
    if (pin >= NUM_BANK0_GPIOS) {
        return;
    }
    gpio_put(pin, value);
}

void GPIO::toggle(uint8_t pin)
{
    if (pin >= NUM_BANK0_GPIOS) {
        return;
    }
    gpio_put(pin, !gpio_get(pin));
}

AP_HAL::DigitalSource* GPIO::channel(uint16_t pin)
{
    if (pin >= NUM_BANK0_GPIOS) {
        return nullptr;
    }
    return NEW_NOTHROW DigitalSource(pin);
}

bool GPIO::usb_connected(void)
{
    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_pin, output);
}

uint8_t DigitalSource::read()
{
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value)
{
    hal.gpio->write(_pin, value);
}

void DigitalSource::toggle()
{
    hal.gpio->toggle(_pin);
}
